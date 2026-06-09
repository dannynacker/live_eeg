# iaf_fixed_duration.py
# Find peak IAF from baseline, then stimulate at that fixed Hz for DURATION_S.

import time, math, csv
import numpy as np
from scipy.signal import butter, filtfilt, welch

# ===== USER SETTINGS =====
PORT = "COM3"
BAUD = 250000
OZ_CH_INDEX = 63
BASELINE_SEC = 30.0     # data used to estimate IAF
DURATION_S = 120.0      # how long to stimulate at fixed IAF
BRIGHT_01 = 0.60
IAF_SEARCH_LO, IAF_SEARCH_HI = 8.0, 12.5
BP_LO, BP_HI, BP_ORDER = 1.0, 30.0
USE_CMD3_FALLBACK = False
# =========================

from strobe_device import StrobeDevice
try: StrobeDevice.BAUDRATE = BAUD
except: pass

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def bandpass(x, fs, lo, hi, order=4):
    b,a = butter(order,[lo/(fs/2),hi/(fs/2)],btype='band'); return filtfilt(b,a,x)

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
    inlet = StreamInlet(streams[0], max_buflen=240)
    fs = inlet.info().nominal_srate() or 2048.0

    n = int(round(BASELINE_SEC*fs))
    oz = np.zeros(n)

    # Baseline
    t_end = time.monotonic()+BASELINE_SEC
    i=0
    while time.monotonic()<t_end and i<n:
        s,_ = inlet.pull_sample(timeout=0.2)
        if s is None: continue
        oz[i]=s[OZ_CH_INDEX]; i+=1
    oz=oz[:i]

    # Clean & estimate IAF
    ozf = bandpass(oz, fs, BP_LO, BP_HI, BP_ORDER)
    iaf = est_peak(ozf, fs, IAF_SEARCH_LO, IAF_SEARCH_HI)
    if math.isnan(iaf):
        iaf = clamp((IAF_SEARCH_LO+IAF_SEARCH_HI)/2.0, IAF_SEARCH_LO, IAF_SEARCH_HI)

    # Stimulate at fixed IAF
    dev = StrobeDevice(port=PORT, log=True)
    logf = open("stim_log.csv","w",newline=""); w=csv.writer(logf)
    w.writerow(["t","event","freq_hz","meta"])
    t0=time.monotonic(); w.writerow([f"{t0:.6f}","RUN_START",f"{iaf:.6f}","fixed_iaf"]); logf.flush()
    try:
        t_stop = time.monotonic()+DURATION_S
        while time.monotonic()<t_stop:
            play(dev, iaf, BRIGHT_01)
            time.sleep(0.25)  # keepalive
    except KeyboardInterrupt:
        pass
    finally:
        t1=time.monotonic(); w.writerow([f"{t1:.6f}","RUN_END",f"{iaf:.6f}",""]); logf.flush()
        try: logf.close()
        except: pass
        force_all_off(dev); dev.close()

if __name__=="__main__":
    main()