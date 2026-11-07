# Hybrid IMU + webcam head tracker — design

Owner: Leo. Coordinator: Claude. Date: 2026-11-07. Branch `hybrid-imu` of
fork `LeoArs06/aitrack` (`master` = upstream).

## Intent

DCS on Windows (monitor). AITrack with a C920 is too noisy/laggy for small
head movements. A MPU-class IMU rigidly mounted on AKG K371 headphones
(case K371 R03, separate activity) supplies fast rotation and
micro-movements; the webcam supplies X/Y/Z and slow absolute correction of
angular drift. Output stays the AITrack → OpenTrack UDP stream so the rest
of the chain (OpenTrack curves, centring hotkey, DCS) is unchanged.

Success: smooth, low-latency rotation including sub-degree movements; no
visible drift; slow voluntary movements preserved; almost plug-and-play
(plug USB, pick Hybrid, run guided calibration once).

## Non-goals / anti-goals

- No HID joystick, no OpenTrack plugin, no rewrite of AITrack architecture.
- No artificial recentring toward zero: correction target is always the
  absolute optical pose.
- Original webcam mode must remain selectable and behave as upstream.
- No new heavy libraries (Qt SerialPort is the only addition).

## Hardware (decided)

Pro Micro ATmega32U4 5 V/16 MHz ↔ HW-123 (GY-521 class, MPU6050 expected,
identified at runtime via WHO_AM_I). No level shifter (Leo's choice).

| HW-123 | Pro Micro | Note |
|---|---|---|
| VCC | VCC (5 V from USB) | only after checking the on-board 3.3 V LDO (see below) |
| GND | GND | |
| SDA | 2 (SDA) | pulled up to 3.3 V by module resistors |
| SCL | 3 (SCL) | idem |
| AD0 | GND | address 0x68 |
| INT | not connected (pin 7 reserved) | firmware polls DATA_RDY |
| XDA, XCL | not connected | |

Electrical rationale: the module's pull-ups go to its 3.3 V rail, so the
bus idles at 3.3 V and never sees 5 V as long as the ATmega internal
pull-ups are disabled (firmware does this right after `Wire.begin()`).
Risk accepted and documented: ATmega32U4 TWI V_IH min = 0.7·VCC = 3.5 V >
3.3 V, i.e. out of datasheet margin; it works in common practice with short
wires. If I²C errors appear in diagnostics, the fix is a BSS138 shifter.
Pre-check before wiring SDA/SCL: power module from 5 V, measure ≈3.3 V on
LDO output and ≈3.3 V on SDA/SCL with nothing else connected. If SDA/SCL
read 5 V the pull-ups go to VCC: do not connect, use a shifter.

Firmware: `firmware/headtracker_imu/`, arduino-cli FQBN
`arduino:avr:leonardo`. Protocol: `PROTOCOL.md` (binary frames, CRC16,
seq, MCU timestamp, 200 Hz).

## Host architecture (in the fork)

```
Pro Micro --USB CDC--> ImuSerialWorker (QThread, QSerialPort, autodetect,
                       reconnect, parser) --samples--> HybridFusion
camera loop (unchanged, GUI thread) --optical pose + capture time--> HybridFusion
HybridFusion (pure C++, no Qt) --200 Hz pose--> UDP sender (sole sender in Hybrid)
```

- `Client/src/hybrid/` pure C++17, no Qt: protocol parser, clock sync,
  fusion core, calibration, simulator. Unit tests + simulator built by a
  standalone CMake project in `tests/hybrid/` (no external deps, tiny
  assert harness).
- Qt glue: serial worker, Presenter wiring, UI.
- Hybrid mode: the camera loop stops sending UDP; output is produced on
  every IMU sample (≈200 Hz). Webcam mode: upstream behaviour.

## Fusion (decided behaviour, tuning left to builder)

- Clock sync: map MCU `t_us` (unwrapped) to host steady clock by a running
  linear fit (offset + drift), robust to USB jitter (min-latency envelope).
- Gyro bias: initial estimate during the still phase of calibration;
  continuous re-estimation only when gyro and optical both say still.
- Orientation: quaternion integration of bias-corrected gyro rotated into
  head frame by `R_imu→head`.
- Optical correction: ring buffer (~1 s) of fused orientation. Each optical
  pose has capture time = arrival − estimated camera latency; compute error
  vs buffered state at that time; apply a fraction to the current state
  (complementary filter on SO(3)). Strength slider maps to time constant
  τ ≈ 0.5 s (strong) … 20 s (weak), default ≈3 s.
- Gating: drop optical samples when face lost, PnP failed, jump vs previous
  optical > threshold, or error vs fused > threshold. If disagreement
  persists (>0.5 s), re-acquire with a fast but smooth ramp (no snap).
- Translation: from optical, time-aware smoothing, held/extrapolated at
  output rate.
- Accelerometer: stillness detection and saturation flags only in v1.
- Fallback: IMU silent >100 ms → crossfade (~0.5 s) to optical-only output;
  IMU back → re-seed from optical and crossfade back. Face lost → gyro only.
- Output: same 48-byte packet `[x,y,z,yaw,pitch,roll]`, cm/deg, same sign
  conventions as upstream AITrack. Conversion convention locked by
  round-trip tests.

## Calibration ("plug and play")

Guided, one button, ~15 s, result saved in `prefs.ini`:
1. Hold still 2 s → gyro bias, gravity direction (sanity), noise floor.
2. "Move your head slowly: left-right, up-down, tilt" ~10 s with face
   visible → `R_imu→head` by least-squares alignment (Kabsch) of gyro
   angular velocity vs optical angular velocity, and camera latency by
   cross-correlation of rates. Quality score shown; reject if poor.
Mounting and AITrack sign conventions are thus learned, not configured.

## UI (main window, minimal)

Group "Hybrid IMU": mode (Webcam / Hybrid), port (Auto + list), Connect,
Calibrate, optical correction strength slider, one status line:
IMU Hz · lost · CRC · optical Hz · latency · drift °/min · state.
Main window, because the config window is disabled while tracking.

## Increments

1. Build upstream unchanged (done 2026-11-07, VS Build Tools 2026 v145,
   Qt 5.15.2 msvc2019_64 via aqtinstall).
2. Firmware + Python diagnostic (`firmware/tools/imu_diag.py`). In
   parallel: pure C++ hybrid core + simulator + tests.
3. Integration: serial worker, Presenter, UI, prefs.
4. Small upstream fixes (separate commits): crop width/height swap and
   subpixel landmarks, solvePnP result check, UDP reconfigure bug,
   `onnx_` prefs round-trip, frame-size validation.
5. Windows instructions, build script, diagnostics doc.

## Verification limits

Hardware is not available yet: firmware is compiled, not run. Fusion is
verified against the simulator (synthetic head motion, delayed noisy
30 Hz optical with dropouts) and unit tests. Real-world tuning needs Leo's
runs of `imu_diag.py` and recorded logs.
