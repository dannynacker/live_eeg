# iaf_antiphase_control.py
# Control condition: Live Oz IAF, but strobe is driven to ANTI-PHASE (π rad) vs Oz.
# Requires photodiode channel to measure actual strobe phase, so the PLL can hold -π.

import time, math, csv
import numpy as np
from scipy.signal import butter, filtfilt, hilbert, welch

# ===== USER SETTINGS =====
PORT = "COM3"
BAUD = 250000
OZ_CH_INDEX = 63      # Oz index in your EEG LSL stream
PD_CH_INDEX = 0       # Photodiode index in the SAME LSL stream
BASELINE_SEC = 20.0   # optional pre-run (no light)
CONTROL_HZ = 8        # update rate (Hz): reassert every 125 ms
BRIGHT_01 = 0.60
IAF_SEARCH_LO, IAF_SEARCH_HI = 8.0, 12.5
IAF_CLAMP_LO, IAF_CLAMP_HI = 6.0, 24.0
PSD_SEC = 3.0         # seconds used for IAF Welch
BP_LO, BP_HI, BP_ORDER = 1.0, 30.0
# PLL gains (gentle)
Kp = 0.8              # proportional (Hz per rad)
Ki = 0.2              # integral (Hz per rad per second)
FREQ_SLEW_MAX = 2.0   # max |Δf| per second (Hz)
USE_CMD3_FALLBACK = False  # set True if your firmware ignores cmd '6'
# =========================

from strobe_device import StrobeDevice
try: StrobeDevice.BAUDRATE = BAUD
except: pass

# ---------- helpers ----------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def bandpass(x, fs, lo, hi, order=4):
    b,a = butter(order, [lo/(fs/2), hi/(fs/2)], btype='band')
    return filtfilt(b,a,x)

def est_iaf(x, fs, lo, hi):
    nper = max(128, int(fs*0.5)); no = int(0.5*nper)
    f,p = welch(x, fs=fs, nperseg=nper, noverlap=no, detrend='constant')
    m = (f>=lo)&(f<=hi)
    if not np.any(m): return float('nan')
    fsub, psub = f[m], p[m]
    # relaxed guard so IAF doesn't drop out too easily
    if psub.max() < (np.median(psub)*1.25): return float('nan')
    return float(fsub[np.argmax(psub)])

def play(dev, freq_hz, bright01):
    b255 = int(round(clamp(bright01,0,1)*255))
    if hasattr(dev, "play_phase_locked") and not USE_CMD3_FALLBACK:
        dev.play_phase_locked(rate_hz=float(freq_hz), brightness=b255,
                              leds=getattr(StrobeDevice, "RING_LEDS", None))
    else:
        dev.send_command('3', f',{float(freq_hz):.2f},{b255}')

def force_all_off(dev):
    try:
        dev.cancel_strobe(); time.sleep(0.10)
        for i in range(9):
            try: dev.send_command('3', f',{i},10,0'); time.sleep(0.01)
            except: pass
        dev.cancel_strobe()
    except: pass

# ---------- main ----------
def main():
    # LSL
    from pylsl import resolve_byprop, StreamInlet
    streams = resolve_byprop('type','EEG', timeout=5)
    if not streams: raise RuntimeError("No LSL EEG stream found.")
    inlet = StreamInlet(streams[0], max_buflen=180)
    fs = inlet.info().nominal_srate() or 2048.0

    # Buffers
    max_sec = max(BASELINE_SEC, PSD_SEC, 4.0)
    nbuf = int(round(max_sec*fs))
    oz = np.zeros(nbuf); pd = np.zeros(nbuf)

    # Device + log
    dev = StrobeDevice(port=PORT, log=True)
    logf = open("stim_log.csv","w",newline=""); w = csv.writer(logf)
    w.writerow(["t_mono_sec","event","freq_hz","phase_err_rad","meta"])
    def log(evt, f=None, e=None, meta=""):
        w.writerow([f"{time.monotonic():.6f}", evt,
                    "" if f is None else f"{f:.6f}",
                    "" if e is None else f"{e:.6f}",
                    meta]); logf.flush()

    try:
        # Baseline (no light)
        t_end = time.monotonic() + BASELINE_SEC
        while time.monotonic() < t_end:
            s,_ = inlet.pull_sample(timeout=0.2)
            if s is None: continue
            oz = np.roll(oz,-1); pd = np.roll(pd,-1)
            oz[-1], pd[-1] = s[OZ_CH_INDEX], s[PD_CH_INDEX]
        log("BASELINE_END", meta=f"fs={fs:.1f}")

        # PLL state
        integ = 0.0
        last_cmd = None
        hop = 1.0/CONTROL_HZ
        next_tick = time.monotonic()

        while True:
            # Drain
            got = True
            while got:
                s,_ = inlet.pull_sample(timeout=0.0)
                if s is None: got=False
                else:
                    oz = np.roll(oz,-1); pd = np.roll(pd,-1)
                    oz[-1], pd[-1] = s[OZ_CH_INDEX], s[PD_CH_INDEX]

            # Pre-filter wide (1–30)
            ozf = bandpass(oz, fs, BP_LO, BP_HI, BP_ORDER)
            pdf = bandpass(pd, fs, BP_LO, BP_HI, BP_ORDER)

            # IAF from Oz
            tail_n = int(round(PSD_SEC*fs))
            oz_tail = ozf[-tail_n:] if tail_n<len(ozf) else ozf
            iaf = est_iaf(oz_tail, fs, IAF_SEARCH_LO, IAF_SEARCH_HI)

            if math.isnan(iaf):
                # If IAF disappears, hold last command gently (keeps anti-phase near target)
                if last_cmd is not None:
                    play(dev, last_cmd, BRIGHT_01)
                    log("HOLD_IAF", last_cmd, 0.0, "weak_alpha")
                else:
                    dev.cancel_strobe(); log("NO_ALPHA_STOP")
            else:
                # For phase: very narrow filter around iaf ± 2 Hz
                lo = max(1.0, iaf-2.0); hi = min(30.0, iaf+2.0)
                oz_n = bandpass(oz[-tail_n:], fs, lo, hi, order=2)
                pd_n = bandpass(pd[-tail_n:], fs, lo, hi, order=2)
                phi_oz = np.angle(hilbert(oz_n))[-1]
                phi_pd = np.angle(hilbert(pd_n))[-1]

                # target = ANTI-PHASE → PD should lag Oz by π rad
                # error = (target - actual) wrapped to (-π, π]
                target = (phi_oz + math.pi)
                err = (target - phi_pd)
                err = math.atan2(math.sin(err), math.cos(err))

                # PI correction as Δf (Hz), limited by slew per tick
                dt = hop
                integ = np.clip(integ + err*dt, -5.0, 5.0)
                df = Kp*err + Ki*integ
                df = np.clip(df, -FREQ_SLEW_MAX*dt, FREQ_SLEW_MAX*dt)

                f_cmd = clamp(iaf + df, IAF_CLAMP_LO, IAF_CLAMP_HI)
                play(dev, f_cmd, BRIGHT_01)
                log("SET_FREQ_ANTIPHASE", f_cmd, err, f"iaf={iaf:.2f}")
                last_cmd = f_cmd

            # pace
            next_tick += hop
            sleep = next_tick - time.monotonic()
            if sleep > 0: time.sleep(sleep)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            log("RUN_END")
        except: pass
        try:
            force_all_off(dev)
        finally:
            try: logf.close()
            except: pass
            try: dev.close()
            except: pass

if __name__=="__main__":
    main()