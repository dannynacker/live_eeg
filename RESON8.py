#!/usr/bin/env python3
"""
Unified EEG-to-Strobe Driver

Modes:
- achilles: continuous IAF-driven strobe
- icarus: continuous strobe at IAF with smooth centroid
- mobius: strobe ON only when alpha amplitude is high
- helix: flashes only when IAF changes significantly

Usage:
    python RESON8.py --mode achilles
"""

import time, threading, argparse
import numpy as np
from scipy.signal import butter, filtfilt, detrend, windows
from collections import deque
from pylsl import StreamInlet, resolve_byprop
from strobe_device import StrobeDevice

# ====== Parameters ======
OZ_CH = 63
FS_EEG = 2048
BUF_SEC = 2.0
LOOP_FS = 250
PULSE_DUR = 0.1
IAF_UPDATE = 1.0
BANDPASS = (7, 13)
b_bp, a_bp = butter(4, [BANDPASS[0]/(FS_EEG/2), BANDPASS[1]/(FS_EEG/2)], 'band')

LED_MASK = 510
BRIGHTNESS = 255
AMP_THRESH = 2e-6
IAF_JUMP_THRESH = 0.01

# ====== Shared State ======
eeg_buf = deque(maxlen=int(FS_EEG * BUF_SEC))
buf_lock = threading.Lock()
stop_evt = threading.Event()
curr_iaf = 10.0
last_iaf = 10.0
pulse_start = None
next_iaf = time.perf_counter()

# ====== LSL Thread ======
def lsl_reader():
    streams = resolve_byprop('type', 'EEG', timeout=5)
    if not streams:
        raise RuntimeError("No EEG stream found.")
    inlet = StreamInlet(streams[0])
    while not stop_evt.is_set():
        samp, _ = inlet.pull_sample(timeout=1.0)
        if samp:
            with buf_lock:
                eeg_buf.append(samp[OZ_CH])

# ====== IAF Estimator ======
def estimate_iaf(filt):
    global curr_iaf
    S = np.fft.rfft(filt, len(filt)*4)
    freqs = np.fft.rfftfreq(len(S)*2 - 1, d=1/FS_EEG)
    P = np.abs(S)**2
    mask = (freqs >= 7) & (freqs <= 13)
    af, ap = freqs[mask], P[mask]
    ap /= (np.sum(ap) + 1e-10)
    if len(ap) > 0:
        curr_iaf = np.sum(af * ap)
    return curr_iaf

# ====== Mode Behaviors ======
def run_driver(mode):
    global pulse_start, next_iaf, curr_iaf, last_iaf

    device = StrobeDevice(port='COM3', log=True)
    device.send_command('C')
    time.sleep(0.1)

    while not stop_evt.is_set():
        t_now = time.perf_counter()
        loop_start = t_now

        with buf_lock:
            if len(eeg_buf) < FS_EEG:
                time.sleep(0.01)
                continue
            seg = np.array(eeg_buf)

        seg = detrend(seg)
        filt = filtfilt(b_bp, a_bp, seg)
        filt *= windows.hann(len(filt))

        # Update IAF
        if t_now >= next_iaf:
            iaf = estimate_iaf(filt)
            print(f"[IAF] {iaf:.2f} Hz")
            next_iaf = t_now + IAF_UPDATE

        # Mode-specific logic
        if mode == 'achilles' or mode == 'icarus':
            if t_now >= pulse_start + (1.0 / curr_iaf) if pulse_start else True:
                device.send_command('6', f"0,{LED_MASK},{BRIGHTNESS},{curr_iaf:.2f}")
                pulse_start = t_now
            if pulse_start and (t_now - pulse_start) >= PULSE_DUR:
                device.send_command('6', "0,0,0,0")
                pulse_start = None

        elif mode == 'mobius':
            amp = np.sqrt(np.mean(filt**2))
            if amp > AMP_THRESH and pulse_start is None:
                device.send_command('6', f"0,{LED_MASK},{BRIGHTNESS},{curr_iaf:.2f}")
                pulse_start = t_now
                print(f"[Mobius] TRIGGER at {curr_iaf:.2f} Hz")
            if pulse_start and (t_now - pulse_start) >= PULSE_DUR:
                device.send_command('6', "0,0,0,0")
                pulse_start = None

        elif mode == 'helix':
            if abs(curr_iaf - last_iaf) > IAF_JUMP_THRESH:
                device.send_command('6', f"0,{LED_MASK},{BRIGHTNESS},{curr_iaf:.2f}")
                pulse_start = t_now
                print(f"[Helix] ΔIAF = {curr_iaf - last_iaf:.2f} Hz")
                last_iaf = curr_iaf
            if pulse_start and (t_now - pulse_start) >= PULSE_DUR:
                device.send_command('6', "0,0,0,0")
                pulse_start = None

        time.sleep(max(0, (1.0 / LOOP_FS) - (time.perf_counter() - loop_start)))

# ====== Entrypoint ======
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, choices=['achilles','icarus','mobius','helix'], required=True)
    args = parser.parse_args()

    try:
        threading.Thread(target=lsl_reader, daemon=True).start()
        time.sleep(1.0)
        run_driver(args.mode)
    except KeyboardInterrupt:
        stop_evt.set()
        device = StrobeDevice(port='COM3', log=True)
        for _ in range(3):
            device.send_command('C')
            device.send_command('6', "0,0,0,0")
            time.sleep(0.1)
        device.close()
        print("[RESON8] Stopped cleanly.")