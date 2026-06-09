# Closed-Loop ANT EEG → LSL → Strobe Demonstrations

Experimental Python demos for intercepting ANT EEG data over Lab Streaming Layer (LSL), estimating alpha/IAF-related control signals in real time, and transmitting stimulation commands to a custom stroboscopic light stimulation (SLS) device.

> **Important status note**
> These scripts are experimental demonstrations and remain a work in progress. They are not validated medical, clinical, therapeutic, neuromodulatory, or biofeedback tools. Several modes may entrain too locally, too simplistically, or too noisily to support meaningful neuromodulation or biofeedback claims. These demos also do **not** correctly bypass or solve unresolved issues with the strobe device’s thermal sensor / thermal safety behaviour.

## Project overview

This repository contains early closed-loop EEG-to-strobe control experiments built around:

* ANT EEG data streamed over LSL
* offline WinPython-compatible Python scripts
* Oz or ROI-based alpha / IAF estimation
* serial communication with a custom strobe device
* multiple experimental control strategies for sending frequency, brightness, phase, or replay commands to the strobe

The general intended flow is:

```text
ANT EEG amplifier
        ↓
LSL EEG stream
        ↓
Python / WinPython offline processing
        ↓
Alpha / IAF / phase / envelope estimate
        ↓
Serial command to strobe
        ↓
Light stimulation output
```

## Repository contents

| File               | Purpose                                                                                                                                                                                                              |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `strobe_device.py` | Shared serial interface for communicating with the strobe device. Handles COM-port connection, command sending, response reading, phase-locked command helper, cancellation, and file-style chunk streaming helpers. |
| `RESON8.py`        | Unified demo launcher with several closed-loop modes: `achilles`, `icarus`, `mobius`, and `helix`.                                                                                                                   |
| `achilles.py`      | High-responsiveness real-time IAF driver using an Oz channel buffer and frequent command updates.                                                                                                                    |
| `daedalus.py`      | Experimental “neuromodulatory” driver using instantaneous alpha frequency, envelope gating, ROI selection, and smoothed command frequency.                                                                           |
| `helix.py`         | Fixed-duration IAF demo: estimates peak IAF from a baseline period, then stimulates at that fixed frequency.                                                                                                         |
| `icarus.py`        | Live Oz IAF plus photodiode phase-alignment demo using a simple PLL-style correction loop.                                                                                                                           |
| `niagara.py`       | Anti-phase control demo, intended to drive the strobe toward anti-phase relative to Oz using a photodiode feedback channel.                                                                                          |
| `mobius.py`        | Record-and-replay demo: records oscillatory Oz activity, derives instantaneous frequency/envelope, writes a CSV trace, then replays the trace through the strobe.                                                    |
| `zarathustra.py`   | Live Oz EEG-to-strobe IAF demo using a simpler command style intended for offline WinPython use.                                                                                                                     |

## What these demos do

The scripts explore several possible closed-loop stimulation strategies:

1. **Continuous IAF tracking**
   Estimate an alpha/IAF-related frequency from EEG and continuously update the strobe.

2. **Fixed IAF stimulation**
   Estimate IAF during a baseline period, then stimulate at that fixed frequency for a set duration.

3. **Amplitude-gated stimulation**
   Stimulate only when alpha amplitude or envelope exceeds a threshold.

4. **Instantaneous-frequency driving**
   Use Hilbert-phase-derived instantaneous frequency rather than simple peak-picking.

5. **Photodiode-based phase alignment**
   Use a photodiode channel to estimate the actual strobe phase and gently correct the output frequency.

6. **Anti-phase control**
   Attempt to drive the strobe toward anti-phase relative to the EEG alpha rhythm.

7. **Record-and-replay control**
   Record an EEG-derived oscillatory trace, derive frequency/brightness over time, save it to CSV, and replay it through the strobe.

## Important limitations

These demos should be treated as exploratory engineering prototypes only.

### Not validated for neuromodulation or biofeedback

The code has not been validated as a neuromodulatory, therapeutic, or biofeedback system. Although several scripts use live EEG features to drive stimulation, this does not establish that the system produces reliable, beneficial, or spatially meaningful brain-state modulation.

### Local entrainment concern

Several approaches rely heavily on Oz or small ROI signals. This may lead to very local or sensor-specific entrainment behaviour. Apparent tracking of alpha/IAF at one electrode should not be interpreted as evidence of whole-brain modulation, network-level engagement, or clinically meaningful feedback.

### Thermal sensor issue not solved

These scripts do **not** correctly bypass, fix, or resolve any outstanding strobe thermal sensor / thermal protection issues. Do not use these scripts as a workaround for unsafe or unresolved hardware behaviour. Hardware-level safety constraints should remain active and should be verified independently.

### Experimental command compatibility

Different scripts assume different serial command styles, baud rates, and firmware behaviours. Some use command `6`, others use command `1` or `3`, and some contain fallback paths. These may need to be edited to match the currently flashed strobe firmware.

### Channel indices must be checked

Most scripts assume specific LSL channel indices, such as Oz at index `63` and, in some demos, a photodiode channel at a fixed index. These are 0-based indices and must be checked against the active ANT/LSL channel order before running.

### Timing is best-effort

The scripts use Python timing, serial writes, LSL buffering, and Windows scheduling. Timing is therefore best-effort rather than hard real-time. For experimental work requiring verified stimulation timing, use photodiode validation and logged trigger alignment.

## Safety warning

Stroboscopic stimulation can be uncomfortable and may pose risks for people with photosensitive epilepsy, migraine sensitivity, neurological conditions, or other contraindications. These scripts should only be used in an appropriate supervised research or engineering context with suitable screening, safety procedures, emergency stop procedures, and independent hardware validation.

Do not run these demos on participants, patients, or volunteers unless the full device, stimulation parameters, thermal behaviour, and study protocol have been reviewed and approved through the relevant institutional and ethical processes.

## Requirements

The scripts were written for an offline Windows / WinPython-style environment.

Typical Python dependencies:

```text
numpy
scipy
pylsl
pyserial
```

Hardware/software dependencies:

```text
ANT EEG system streaming over LSL
Custom strobe device connected over USB serial
Correct strobe firmware flashed to the device
Photodiode channel, where required by phase-control demos
Windows COM port access
```

## Basic setup

1. Start the ANT EEG system.
2. Confirm that EEG data are being streamed over LSL with stream type `EEG`.
3. Connect the strobe device over USB.
4. Confirm the correct COM port, usually edited near the top of each script:

```python
PORT = "COM3"
```

5. Confirm the expected baud rate:

```python
BAUD = 250000
```

or, for some older/simple firmware paths:

```python
BAUD = 115200
```

6. Confirm EEG channel indices:

```python
OZ_CH_INDEX = 63
PD_CH_INDEX = 67
```

7. Run only one driver script at a time.

## Example usage

Run the unified demo launcher:

```bash
python RESON8.py --mode achilles
```

Available `RESON8.py` modes:

```bash
python RESON8.py --mode achilles
python RESON8.py --mode icarus
python RESON8.py --mode mobius
python RESON8.py --mode helix
```

Run the instantaneous-frequency / envelope-gated demo:

```bash
python daedalus.py
```

Run the fixed-duration IAF demo:

```bash
python helix.py
```

Run the live Oz-to-strobe demo using the simpler command interface:

```bash
python zarathustra.py
```

## Configuration notes

Most scripts include a user-editable settings block near the top. Common fields include:

```python
PORT = "COM3"
BAUD = 250000
OZ_CH_INDEX = 63
CONTROL_HZ = 8
BRIGHT_01 = 0.60
IAF_SEARCH_LO = 8.0
IAF_SEARCH_HI = 12.5
```

Before running a script, check:

* COM port
* baud rate
* command style expected by the firmware
* Oz channel index
* photodiode channel index, if used
* brightness level
* stimulation duration
* update rate
* alpha/IAF search band
* whether the script cancels stimulation when alpha is weak or holds the last command

## Strobe command interface

The shared `StrobeDevice` wrapper provides:

* serial connection and optional auto-detection
* `send_command()`
* `get_response()`
* `play_phase_locked()`
* `cancel_strobe()`
* `close()`
* basic file/chunk streaming helpers

Several scripts send commands such as:

```python
device.send_command('C')
device.send_command('6', "0,0,0,0")
device.send_command('6', f"0,{LED_MASK},{BRIGHTNESS},{freq:.2f}")
```

Other scripts use older firmware command styles such as:

```python
1,<Hz>,<Seconds>
3,<Hz>,<Brightness>
```

Check the currently flashed strobe firmware before assuming any command format is valid.

## Logs and outputs

Some demos write CSV logs, for example:

```text
stim_log.csv
daedalus_log.csv
osc_trace.csv
```

These logs may include timestamps, estimated frequency, command frequency, phase error, envelope values, and event labels. They are useful for debugging, but they do not by themselves validate EEG entrainment or stimulation timing.

## Suggested validation before experimental use

Before using any mode in a formal experiment, validate at minimum:

1. The LSL channel order is correct.
2. The Oz signal is actually the intended EEG channel.
3. The photodiode channel is correctly assigned where needed.
4. The serial command received by the strobe matches the intended frequency and brightness.
5. The actual emitted flicker frequency is verified using a photodiode or oscilloscope.
6. The strobe stops reliably after keyboard interrupt or script failure.
7. Thermal sensor behaviour is understood and not bypassed unsafely.
8. Logs are synchronized with any EEG/stimulus recording pipeline.
9. The output does not exceed approved luminance, frequency, duty-cycle, or duration limits.

## Known issues / work in progress

* These are demo scripts rather than a unified production pipeline.
* Firmware command assumptions differ across scripts.
* Some filenames and script names reflect older exploratory versions.
* Closed-loop timing is not hard real-time.
* The approach may entrain too locally to support broader neuromodulation claims.
* The system does not solve unresolved thermal sensor issues.
* Some modes need photodiode feedback but assume hardcoded channel indices.
* Alpha peak-picking can be unstable or collapse to noisy estimates.
* Instantaneous-frequency estimates require careful filtering and smoothing.
* Safety, timing, and physiological efficacy require independent validation.

## Recommended wording for reuse

This repository contains exploratory closed-loop EEG-to-strobe demonstrations developed for offline WinPython use with ANT EEG LSL streams and a custom stroboscopic stimulation device. The code demonstrates several candidate approaches for estimating alpha/IAF-related control signals and transmitting them to the strobe over serial. These scripts are not validated neuromodulation or biofeedback tools, may entrain too locally or unreliably for such purposes, and do not resolve outstanding thermal sensor issues in the strobe hardware.

## Disclaimer

This code is provided for research engineering and reproducibility documentation only. It is not a medical device, not a treatment, and not a validated closed-loop neuromodulation system. Use requires appropriate expertise, hardware validation, and safety oversight.
