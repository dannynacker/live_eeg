# record_replay_osc.py
# Record ~60s Oz oscillatory activity, derive inst. freq & envelope, then replay via strobe.

import time, math, csv
import numpy as np
from scipy.signal import butter, filtfilt, hilbert, welch

# ===== USER SETTINGS =====
PORT = "COM3"
BAUD = 250000
OZ_CH_INDEX = 63
RECORD_SEC = 60.0        # how long to record
REPLAY_SEC = None        # None = replay full length; or set shorter
BRIGHT_01_BASE = 0.55    # base brightness scaling
IAF_SEARCH_LO, IAF_SEARCH_HI = 8.0, 12.5
BAND_PAD = 2.0           # band = [IAF-2, IAF+2]
CONTROL_HZ = 50          # replay update cadence (Hz)
CSV_OUT = "osc_trace.csv"
USE_CMD3_FALLBACK = False
# =========================

from strobe_device import StrobeDevice
try: StrobeDevice.BAUDRATE = BAUD
except: pass

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def bandpass(x, fs, lo, hi, order=4):
    b,a=butter(order,[lo/(fs/2),hi/(fs/2)],btype='band'); return filtfilt(b,a,x)

def est_peak(x, fs, lo, hi):
    nper=max(128,int(fs*0.5)); no=int(0.5*nper)
    f,p=welch(x,fs=fs,nperseg=nper,noverlap=no,detrend='constant')
    m=(f>=lo)&(f<=hi)
    if not np.any(m): return float('nan')
    fsub,psub=f[m],p[m]
    return float(fsub[np.argmax(psub)])

def play(dev, freq_hz, bright01):
    b255=int(round(clamp(bright01,0,1)*255))
    if hasattr(dev,"play_phase_locked") and not USE_CMD3_FALLBACK:
        dev.play_phase_locked(rate_hz=float(freq_hz),brightness=b255,
                              leds=getattr(StrobeDevice,"RING_LEDS",None))
    else:
        dev.send_command('3', f',{float(freq_hz):.2f},{b255}')

def force_all_off(dev):
    try:
        dev.cancel_strobe(); time.sleep(0.1)
        for i in range(9):
            try: dev.send_command('3', f',{i},10,0'); time.sleep(0.01)
            except: pass
        dev.cancel_strobe()
    except: pass

def main():
    from pylsl import resolve_byprop, StreamInlet
    streams = resolve_byprop('type','EEG', timeout=5)
    if not streams: raise RuntimeError("No LSL EEG stream found.")
    inlet = StreamInlet(streams[0], max_buflen=300)
    fs = inlet.info().nominal_srate() or 2048.0

    nrec = int(round(RECORD_SEC*fs))
    oz = np.zeros(nrec)

    # Record
    t_end = time.monotonic()+RECORD_SEC
    i=0
    while time.monotonic()<t_end and i<nrec:
        s,_ = inlet.pull_sample(timeout=0.2)
        if s is None: continue
        oz[i]=s[OZ_CH_INDEX]; i+=1
    oz = oz[:i]

    # Determine IAF, then extract inst. freq & envelope
    iaf = est_peak(oz, fs, IAF_SEARCH_LO, IAF_SEARCH_HI)
    if math.isnan(iaf):
        iaf = clamp( (IAF_SEARCH_LO+IAF_SEARCH_HI)/2.0, IAF_SEARCH_LO, IAF_SEARCH_HI)
    lo,hi = clamp(iaf-BAND_PAD,1.0,30.0), clamp(iaf+BAND_PAD,1.0,30.0)
    zf = bandpass(oz, fs, lo, hi, 4)
    an = hilbert(zf)
    phase = np.unwrap(np.angle(an))
    inst_freq = (np.diff(phase)/(2*np.pi))*fs     # Hz, length N-1
    inst_freq = np.clip(inst_freq, 6.0, 24.0)
    env = np.abs(an)                               # same length as zf

    # Resample both to CONTROL_HZ for replay
    T = len(zf)/fs
    t_src = np.linspace(0, T, num=len(inst_freq), endpoint=False)
    t_env = np.linspace(0, T, num=len(env), endpoint=False)
    t_dst = np.arange(0, REPLAY_SEC if REPLAY_SEC else T, 1.0/CONTROL_HZ)
    f_series = np.interp(t_dst, t_src, inst_freq)
    e_series = np.interp(t_dst, t_env, env)
    # Normalize envelope to 0.3..1.0 scaling
    e_norm = e_series / (np.percentile(e_series, 95) + 1e-6)
    e_norm = np.clip(e_norm, 0.3, 1.0)
    b_series = BRIGHT_01_BASE * e_norm

    # Save CSV (time, freq, bright)
    with open(CSV_OUT,"w",newline="") as f:
        w=csv.writer(f); w.writerow(["t_s","freq_hz","bright_01"])
        for t,fq,br in zip(t_dst,f_series,b_series): w.writerow([f"{t:.4f}",f"{fq:.4f}",f"{br:.4f}"])

    # Replay
    dev = StrobeDevice(port=PORT, log=True)
    try:
        t0 = time.monotonic()
        for fq, br in zip(f_series, b_series):
            play(dev, float(fq), float(br))
            # wait until next tick
            t0 += 1.0/CONTROL_HZ
            sleep = t0 - time.monotonic()
            if sleep>0: time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        force_all_off(dev); dev.close()

if __name__=="__main__":
    main()