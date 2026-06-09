#!/usr/bin/env python3
"""
Real-time IAF strobe driver at maximum responsiveness (250 Hz loop)
Precise, minimal buffering with robust start-up and cancel.
Includes ACK readout and proper Ctrl+C handling.
"""

import time, threading
import numpy as np
from scipy.signal import butter, filtfilt, detrend, windows
from pylsl import resolve_byprop, StreamInlet
from collections import deque
from strobe_device import StrobeDevice

# — CONFIGURATION —
OZ_CH      = 63
FS_EEG     = 2048
LOOP_FS    = 250                    # maximum responsiveness: 250 Hz
BUF_SEC    = 2.0
HOP_SEC    = 1.0 / LOOP_FS
IAF_BAND   = (8.0, 12.0)
BP_BAND    = (7.0, 13.0)
NFFT_MULT  = 4
LED_MASK   = 510
BRIGHTNESS = 200

# — BANDPASS FILTER DESIGN —
b_bp, a_bp = butter(4, [BP_BAND[0]/(FS_EEG/2), BP_BAND[1]/(FS_EEG/2)], 'bandpass')

# — THREAD-SAFE EEG BUFFER —
eeg_buf  = deque(maxlen=int(BUF_SEC * FS_EEG))
buf_lock = threading.Lock()
stop_evt = threading.Event()

# — LSL EEG Reader Thread —
def lsl_reader():
    streams = resolve_byprop('type','EEG', timeout=5.0)
    if not streams:
        raise RuntimeError("No EEG LSL stream")
    inlet = StreamInlet(streams[0], max_chunklen=1)
    while not stop_evt.is_set():
        samp, _ = inlet.pull_sample(timeout=1.0)
        if samp is not None:
            with buf_lock:
                eeg_buf.append(float(samp[OZ_CH]))

# Start LSL EEG buffer filling
thr = threading.Thread(target=lsl_reader, daemon=True)
thr.start()

print(f"Priming buffer ({BUF_SEC}s)...")
while True:
    with buf_lock:
        if len(eeg_buf) >= int(BUF_SEC * FS_EEG):
            break
    time.sleep(0.001)
print("Buffer primed — starting loop.")

# Open strobe device with ACK logging enabled
print("Connecting to strobe...")
device = StrobeDevice(port='COM3', log=True)
device.send_command('C')
device.send_command('6', "0,0,0,0")
time.sleep(0.1)  # initial flush

next_fft = time.perf_counter()
last_sent_iaf = None
iaf_threshold = 0.01  # minimal meaningful IAF change (Hz)

try:
    while True:
        now = time.perf_counter()

        if now >= next_fft:
            next_fft += HOP_SEC

            # EEG processing
            with buf_lock:
                seg = np.array(eeg_buf, dtype=float)

            seg = detrend(seg, type='linear')
            filt = filtfilt(b_bp, a_bp, seg)
            filt *= windows.hann(len(filt))

            nz = len(filt)*NFFT_MULT
            S = np.fft.rfft(filt, n=nz)
            freqs = np.fft.rfftfreq(nz, 1/FS_EEG)
            P = np.abs(S)**2

            mask = (freqs>=IAF_BAND[0]) & (freqs<=IAF_BAND[1])
            idxs = np.nonzero(mask)[0]
            k0 = idxs[np.argmax(P[mask])]
            if 1 <= k0 < len(P)-1:
                y0,y1,y2 = P[k0-1],P[k0],P[k0+1]
                p = 0.5*(y0 - y2)/(y0 - 2*y1 + y2)
            else:
                p = 0.0
            iaf = freqs[k0] + p*(freqs[1]-freqs[0])

            # Send command only if significant IAF change
            if last_sent_iaf is None or abs(iaf - last_sent_iaf) > iaf_threshold:
                device.send_command('6', f"0,{LED_MASK},{BRIGHTNESS},{iaf:.2f}")
                last_sent_iaf = iaf

        # Precise sleep
        time.sleep(max(0, next_fft - time.perf_counter()))

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected: stopping strobe...")
finally:
    stop_evt.set()
    for _ in range(5):  # increased repetitions for robustness
        device.send_command('C')
        device.send_command('6', "0,0,0,0")
        time.sleep(0.1)  # increased delay for buffer clearance
    device.close()
    print("Strobe stopped cleanly.")