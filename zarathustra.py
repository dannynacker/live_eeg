# helix.py — Live Oz EEG → Strobe @ IAF (no fixed frequency)
# Works in offline WinPython; uses only numpy/scipy/pylsl/pyserial that ship with WinPython.

import time, csv, math
import numpy as np
from scipy.signal import butter, filtfilt, welch

# --------- USER SETTINGS (edit these if needed) ----------
PORT = "COM3"             # your Teensy COM port
BAUD = 115200             # 115200 is common; try 250000 if your sketch expects it
OZ_CH_INDEX = 63          # 0-based Oz channel index in your LSL EEG stream
CONTROL_HZ = 4            # control-loop rate (Hz): 4 = update every 250 ms
BURST_SECONDS = 0.25      # how long each command runs; reassert each tick
BRIGHT_01 = 0.60          # 0..1 brightness (used only for cmd '3' path)
USE_LED_INDEX = False     # set True if your firmware requires LED index first
LED_INDEX = 0             # which LED index to address when USE_LED_INDEX=True
CMD_STYLE = "cmd1"        # "cmd1" = 1,<Hz>,<Sec> ; "cmd3" = 3,<Hz>,<Bright>
IAF_LO_HZ, IAF_HI_HZ = 8.0, 12.5  # alpha-search band
IAF_CLAMP_LO, IAF_CLAMP_HI = 6.0, 24.0
PSD_SEC = 2.0             # window length for IAF Welch (seconds)
BP_LO, BP_HI, BP_ORDER = 1.0, 30.0, 4  # bandpass params
# ---------------------------------------------------------

# Serial driver
from strobe_device import StrobeDevice
try:
    StrobeDevice.BAUDRATE = BAUD
except Exception:
    pass

# LSL input
try:
    from pylsl import resolve_byprop, StreamInlet
except Exception as e:
    raise RuntimeError("pylsl is required to read EEG. If this build lacks pylsl, run on a box that includes it.") from e

# ---------- helpers ----------
def bandpass_filtfilt(x, fs, lo=1.0, hi=30.0, order=4):
    b, a = butter(order, [lo/(fs/2.0), hi/(fs/2.0)], btype='band')
    return filtfilt(b, a, x)

def estimate_iaf_hz(x, fs, lo=8.0, hi=12.5):
    """Welch PSD on the last PSD_SEC seconds; return peak Hz in [lo,hi], NaN if weak."""
    nperseg = max(128, int(fs * 0.5))        # ~0.5 s segments
    noverlap = int(nperseg * 0.5)
    f, p = welch(x, fs=fs, nperseg=nperseg, noverlap=noverlap, detrend='constant')
    m = (f >= lo) & (f <= hi)
    if not np.any(m):
        return float("nan")
    f_sub, p_sub = f[m], p[m]
    if p_sub.max() < (np.median(p) * 2.0):   # weak peak guard
        return float("nan")
    return float(f_sub[np.argmax(p_sub)])

def clamp(v, lo, hi): return max(lo, min(hi, v))

def send_strobe(dev, freq_hz, burst_s, bright_01):
    """Dispatch the right command shape for your firmware."""
    freq_hz = float(freq_hz)
    if CMD_STYLE == "cmd1":
        if USE_LED_INDEX:
            dev.send_command('1', f',{LED_INDEX},{freq_hz:.2f},{burst_s:.2f}')
        else:
            dev.send_command('1', f',{freq_hz:.2f},{burst_s:.2f}')
    elif CMD_STYLE == "cmd3":
        b255 = int(round(clamp(bright_01,0,1)*255))
        if USE_LED_INDEX:
            dev.send_command('3', f',{LED_INDEX},{freq_hz:.2f},{b255}')
        else:
            dev.send_command('3', f',{freq_hz:.2f},{b255}')
    else:
        raise ValueError("CMD_STYLE must be 'cmd1' or 'cmd3'.")

def force_all_off(dev):
    """Aggressive 'blackout' sequence to unlatch any stuck LEDs."""
    try:
        dev.cancel_strobe(); time.sleep(0.10)
        # try both cmd3 variants with 0 brightness
        try: dev.send_command('3', ',10,0'); time.sleep(0.02)
        except: pass
        try: dev.send_command('3', f',{LED_INDEX},10,0'); time.sleep(0.02)
        except: pass
        for i in range(9):
            try: dev.send_command('3', f',{i},10,0'); time.sleep(0.01)
            except: pass
        dev.cancel_strobe()
    except Exception:
        try: dev.cancel_strobe()
        except: pass

# ---------- main ----------
def main():
    # Connect EEG LSL
    streams = resolve_byprop('type','EEG', timeout=5)
    if not streams:
        raise RuntimeError("No LSL EEG stream found. Start your EEG stream first.")
    inlet = StreamInlet(streams[0], max_buflen=120)
    fs = inlet.info().nominal_srate()
    if not fs or fs <= 0:
        # fallback if stream doesn't report; adjust OZ_CH_INDEX if needed
        fs = 2048.0

    # Ring buffer
    buf_len = int(round(max(PSD_SEC*1.2, 4.0) * fs))  # keep a few seconds
    xbuf = np.zeros(buf_len, dtype=np.float64)

    # Device + logging
    dev = StrobeDevice(port=PORT, log=True)
    logf = open("stim_log.csv","w", newline="")
    logw = csv.writer(logf); logw.writerow(["t_mono_sec","event","freq_hz","meta"])

    def log(evt, f=None, meta=""):
        logw.writerow([f"{time.monotonic():.6f}", evt, "" if f is None else f"{f:.6f}", meta]); logf.flush()

    try:
        # Baseline fill (~2 s)
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            sample, _ = inlet.pull_sample(timeout=0.2)
            if sample is not None:
                xbuf = np.roll(xbuf, -1); xbuf[-1] = sample[OZ_CH_INDEX]

        log("RUN_START", meta=f"fs={fs:.1f}")

        # Control loop
        hop = 1.0/CONTROL_HZ
        next_tick = time.monotonic()
        last_sent_hz = None
        while True:
            # Drain EEG
            got = True
            while got:
                sample, _ = inlet.pull_sample(timeout=0.0)
                if sample is None:
                    got = False
                else:
                    xbuf = np.roll(xbuf, -1); xbuf[-1] = sample[OZ_CH_INDEX]

            # Preprocess (mirror MATLAB): 1–30 Hz bandpass
            xf = bandpass_filtfilt(xbuf, fs, BP_LO, BP_HI, BP_ORDER)

            # Use the last PSD_SEC seconds for IAF
            n_tail = int(round(PSD_SEC*fs))
            tail = xf[-n_tail:] if n_tail < len(xf) else xf
            iaf = estimate_iaf_hz(tail, fs, IAF_LO_HZ, IAF_HI_HZ)
            if not np.isnan(iaf):
                f_cmd = clamp(iaf, IAF_CLAMP_LO, IAF_CLAMP_HI)
                # Send only if changed enough OR to reassert each tick
                if (last_sent_hz is None) or (abs(f_cmd - last_sent_hz) >= 0.2) or (CMD_STYLE=="cmd1"):
                    send_strobe(dev, f_cmd, BURST_SECONDS, BRIGHT_01)
                    log("SET_FREQ", f_cmd, meta=CMD_STYLE)
                    last_sent_hz = f_cmd
            else:
                # If no stable alpha: either do nothing or gentle default
                # Here we simply stop/reassert darkness
                dev.cancel_strobe()
                log("NO_ALPHA", meta="cancel")

            # Pace loop
            next_tick += hop
            sleep = next_tick - time.monotonic()
            if sleep > 0: time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        log("RUN_END")
        try: force_all_off(dev)
        finally:
            try: logf.close()
            except: pass
            try: dev.close()
            except: pass

if __name__ == "__main__":
    main()