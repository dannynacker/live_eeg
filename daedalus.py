#!/usr/bin/env python3
"""
daedalus.py — Live neuromodulatory SLS driver (instantaneous frequency + amplitude gating)

Goal:
- Avoid "IAF collapse" from peak-picking by driving the strobe using *instantaneous*
  alpha dynamics (phase-derivative) + envelope gating.
- Supports ROI choices (e.g., Temporal vs Oz) and easy channel-index edits.

Dependencies: numpy, scipy, pylsl, pyserial (WinPython-friendly)
Uses: strobe_device.py (your device wrapper)
"""

import time, csv, math
import numpy as np
from collections import deque
from scipy.signal import butter, filtfilt, hilbert

from pylsl import resolve_byprop, StreamInlet
from strobe_device import StrobeDevice  # :contentReference[oaicite:3]{index=3}

# ---------------- USER SETTINGS ----------------
PORT = "COM3"
BAUD = 250000                # match your device expectation (you've used 250k in several scripts)
CMD = "6"                    # '6' uses play_phase_locked style in your stack; fallback below if needed
USE_CMD3_FALLBACK = False    # set True if firmware ignores cmd '6'

CONTROL_HZ = 8               # how often we update (Hz)
BURST_S = 0.20               # how long each command runs / keepalive duration
BRIGHT_01 = 0.60             # 0..1

FS_FALLBACK = 2048.0         # if stream doesn't report nominal_srate
BUF_SEC = 4.0                # rolling buffer seconds used for inst-freq estimation
ALPHA_LO, ALPHA_HI = 7.0, 13.0

# Strobe clamp range
F_CLAMP_LO, F_CLAMP_HI = 6.0, 24.0

# Smoothing of command frequency (EMA)
EMA_ALPHA = 0.25             # 0..1 (higher = more responsive)

# Envelope gating:
# We compute envelope RMS over last GATE_WIN_SEC and compare to a baseline percentile.
BASELINE_SEC = 10.0
GATE_WIN_SEC = 1.0
GATE_PERCENTILE = 60         # raise to be stricter, lower to be more permissive
HOLD_LAST_WHEN_LOW = True    # if False, cancels strobe when low envelope

# ROI selection:
# Option A: a single channel index (e.g., Oz)
# Option B: average across a set of channel indices (e.g., temporal ROI)
ROI_MODE = "temporal"        # "oz" or "temporal" or "custom"

# IMPORTANT: these are 0-based indices into your LSL stream channel vector.
# You must set these to match YOUR stream ordering.
OZ_CH_INDEX = 63
TEMPORAL_CHS = [55, 56, 57, 58, 59, 60]  # <-- EDIT: put your T7/T8/TP7/TP8 etc indices here
CUSTOM_CHS = [63]                        # fallback

# LED / brightness (for cmd '6' path)
LED_MASK = 510
BRIGHTNESS_255 = int(round(BRIGHT_01 * 255))
# ------------------------------------------------


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def bandpass_filt(x, fs, lo, hi, order=4):
    b, a = butter(order, [lo/(fs/2.0), hi/(fs/2.0)], btype="band")
    return filtfilt(b, a, x)


def roi_signal(sample_vec):
    """Extract ROI signal from the current LSL sample vector."""
    if ROI_MODE == "oz":
        return float(sample_vec[OZ_CH_INDEX])
    elif ROI_MODE == "temporal":
        idx = TEMPORAL_CHS if len(TEMPORAL_CHS) else [OZ_CH_INDEX]
        return float(np.mean([sample_vec[i] for i in idx]))
    else:
        idx = CUSTOM_CHS if len(CUSTOM_CHS) else [OZ_CH_INDEX]
        return float(np.mean([sample_vec[i] for i in idx]))


def estimate_inst_freq_hz(x_filt, fs):
    """
    Instantaneous frequency estimate from analytic signal:
    f_inst(t) = (1/2π) dφ/dt
    We return a robust summary (median) over the last ~1 second.
    """
    an = hilbert(x_filt)
    phase = np.unwrap(np.angle(an))
    dphi = np.diff(phase)
    f_inst = (dphi / (2.0*np.pi)) * fs  # Hz, length N-1

    # Robustify: ignore crazy outliers, then take median of the tail
    f_inst = np.clip(f_inst, F_CLAMP_LO, F_CLAMP_HI)

    tail_n = int(round(1.0 * fs))
    if tail_n < 10:
        tail_n = min(len(f_inst), 50)
    tail = f_inst[-tail_n:] if len(f_inst) > tail_n else f_inst
    if len(tail) == 0:
        return float("nan")
    return float(np.median(tail))


def env_level(x_filt, fs):
    """Envelope level over last GATE_WIN_SEC (RMS of analytic amplitude)."""
    an = hilbert(x_filt)
    env = np.abs(an)
    n = int(round(GATE_WIN_SEC * fs))
    n = min(len(env), max(10, n))
    tail = env[-n:]
    return float(np.sqrt(np.mean(tail**2)))


def play(dev, freq_hz):
    freq_hz = float(freq_hz)
    if (hasattr(dev, "play_phase_locked")) and (not USE_CMD3_FALLBACK) and (CMD == "6"):
        dev.play_phase_locked(rate_hz=freq_hz, brightness=BRIGHTNESS_255)
    else:
        # cmd3 fallback: "3,<Hz>,<Brightness>"
        b255 = int(round(clamp(BRIGHT_01, 0, 1) * 255))
        dev.send_command('3', f",{freq_hz:.2f},{b255}")


def force_all_off(dev):
    try:
        dev.cancel_strobe()
        time.sleep(0.10)
        for i in range(9):
            try:
                dev.send_command('3', f",{i},10,0")
                time.sleep(0.01)
            except Exception:
                pass
        dev.cancel_strobe()
    except Exception:
        pass


def main():
    # LSL resolve
    streams = resolve_byprop("type", "EEG", timeout=5)
    if not streams:
        raise RuntimeError("No LSL EEG stream found. Start your EEG stream first.")
    inlet = StreamInlet(streams[0], max_buflen=240)

    fs = inlet.info().nominal_srate()
    if not fs or fs <= 0:
        fs = FS_FALLBACK
    fs = float(fs)

    # Rolling buffer for ROI signal
    nbuf = int(round(BUF_SEC * fs))
    buf = deque(maxlen=nbuf)

    # Device
    try:
        StrobeDevice.BAUDRATE = BAUD
    except Exception:
        pass
    dev = StrobeDevice(port=PORT, log=True)
    dev.send_command('C')
    dev.send_command('6', "0,0,0,0")
    time.sleep(0.1)

    # Logging
    logf = open("daedalus_log.csv", "w", newline="")
    w = csv.writer(logf)
    w.writerow(["t_mono", "event", "f_cmd", "f_inst", "env", "gate_thr", "roi_mode"])
    logf.flush()

    def log(evt, f_cmd=None, f_inst=None, env=None, thr=None):
        w.writerow([
            f"{time.monotonic():.6f}",
            evt,
            "" if f_cmd is None else f"{float(f_cmd):.4f}",
            "" if f_inst is None else f"{float(f_inst):.4f}",
            "" if env is None else f"{float(env):.6e}",
            "" if thr is None else f"{float(thr):.6e}",
            ROI_MODE
        ])
        logf.flush()

    # Fill buffer + baseline for gating
    print(f"[daedalus] Priming buffer ({BUF_SEC:.1f}s) + baseline ({BASELINE_SEC:.1f}s)...")
    baseline_env = []

    t_end = time.monotonic() + max(BUF_SEC, BASELINE_SEC)
    while time.monotonic() < t_end:
        s, _ = inlet.pull_sample(timeout=0.2)
        if s is None:
            continue
        buf.append(roi_signal(s))

        # once we have enough samples, compute env for baseline collection
        if len(buf) >= int(2.0 * fs):  # wait at least 2s
            x = np.asarray(buf, dtype=float)
            xf = bandpass_filt(x, fs, ALPHA_LO, ALPHA_HI, order=4)
            baseline_env.append(env_level(xf, fs))

    if len(baseline_env) < 10:
        gate_thr = np.median(baseline_env) if baseline_env else 0.0
    else:
        gate_thr = float(np.percentile(baseline_env, GATE_PERCENTILE))

    print(f"[daedalus] Gate threshold set at {GATE_PERCENTILE}th percentile: {gate_thr:.6e}")
    log("BASELINE_END", thr=gate_thr)

    # Control loop
    hop = 1.0 / CONTROL_HZ
    next_tick = time.monotonic()
    f_cmd = None

    try:
        while True:
            # Drain new samples
            got = True
            while got:
                s, _ = inlet.pull_sample(timeout=0.0)
                if s is None:
                    got = False
                else:
                    buf.append(roi_signal(s))

            if len(buf) < int(2.0 * fs):
                time.sleep(0.01)
                continue

            x = np.asarray(buf, dtype=float)
            xf = bandpass_filt(x, fs, ALPHA_LO, ALPHA_HI, order=4)

            f_inst = estimate_inst_freq_hz(xf, fs)
            env = env_level(xf, fs)

            # Decide: stimulate or not
            if math.isnan(f_inst):
                dev.cancel_strobe()
                log("NO_FREQ_CANCEL", f_inst=f_inst, env=env, thr=gate_thr)
            else:
                f_inst = clamp(f_inst, F_CLAMP_LO, F_CLAMP_HI)

                if env >= gate_thr:
                    # EMA smoothing toward f_inst
                    if f_cmd is None:
                        f_cmd = f_inst
                    else:
                        f_cmd = (1.0 - EMA_ALPHA) * f_cmd + EMA_ALPHA * f_inst

                    play(dev, f_cmd)
                    log("PLAY", f_cmd=f_cmd, f_inst=f_inst, env=env, thr=gate_thr)
                else:
                    if HOLD_LAST_WHEN_LOW and (f_cmd is not None):
                        # keep gentle hold (still “closed-loop” but won’t chase noise)
                        play(dev, f_cmd)
                        log("HOLD_LOWENV", f_cmd=f_cmd, f_inst=f_inst, env=env, thr=gate_thr)
                    else:
                        dev.cancel_strobe()
                        log("LOWENV_CANCEL", f_inst=f_inst, env=env, thr=gate_thr)

            # pace
            next_tick += hop
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        log("RUN_END")
    finally:
        try:
            force_all_off(dev)
        finally:
            try:
                logf.close()
            except Exception:
                pass
            try:
                dev.close()
            except Exception:
                pass
        print("[daedalus] Stopped cleanly.")


if __name__ == "__main__":
    main()