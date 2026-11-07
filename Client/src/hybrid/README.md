# Standalone hybrid core

Pure C++17; no Qt, OS headers, threads, I/O, or dependencies. The host owns these
objects and serializes calls on one thread (or supplies its own lock).

## Adapter contract

1. Feed serial bytes to `ImuProtocol::feed`. Its variant messages contain raw
   samples, device info, or status. `frames` counts every CRC-valid frame;
   `malformed` counts unsupported type/length or invalid INFO identity/version;
   `resyncs` counts discarded bytes. Sequence gaps count acquired samples lost
   in transport. Duplicates do not count as loss. Backward jumps (half-range
   rule) and re-init flags start a new unwrapping epoch and set `discontinuity`.
   The caller must propagate this to clock sync and fusion. Counter reset is
   explicit; reconnects should reset the parser and clock.
2. Call `ClockSync::observe(sample.timeUs, arrivalHostUs, discontinuity)` and
   `scale(sample, info.whoAmI)`. Supply steady-clock microseconds, with a common
   epoch for IMU arrivals, camera arrivals and output. Scaling implements the
   fixed v1 ranges; reject incompatible device configurations in the adapter.
   Unidentified temperature values use the 6500-family formula; the adapter
   should label them as unverified. Saturation is carried in `SampleSI::flags`.
3. Install a successful `CalibrationResult`, then call `addImuSample(mappedUs,
   sampleSI)`, `addOptical(arrivalUs, opticalPose)`, and `output(nowUs)`.
   The output cadence is normally 200 Hz. Continue calling output on a host
   timer during an IMU outage so fallback can run. Optical `valid=false` means
   face lost or failed PnP. Optical rotations must already use the presenter's
   yaw/pitch/roll values and translations must be centimetres.
4. `FusionOutput::pose.packet(buffer)` fills the six doubles in presenter order:
   `[x,y,z,yaw,pitch,roll]`, centimetres and degrees. Byte transport remains the
   adapter's responsibility. `valid` is false until either source initializes.

`PoseConvention.h` defines active body-to-world `Ry(yaw) Rx(pitch) Rz(roll)`.
Gyro vectors are body rates, integrated by right multiplication. Optical errors
are world-frame rotations applied on the left to current and buffered states.
Pitch angles strictly between -90 and +90 degrees round-trip numerically;
at the singular endpoints only orientation is recoverable. Matrix/quaternion
operations remain valid through those endpoints. The presenter maps solver
`[pitch,yaw,roll]` to the output ordering; no additional axis swap is applied.

## Calibration and tuning

Call `begin(startUs)`, feed both streams, and show prompts according to `phase()`.
The first 2 seconds validate gyro/acceleration variance and gravity and compute
IMU-frame bias (rad/s). Motion must provide about 10 seconds with all axes
excited. Call `finish()` after the prompt. No unsuccessful result is installed
automatically. The result includes IMU-to-head quaternion, camera latency,
dimensionless quality, residual in deg/s, and excitation per head axis in deg/s.
`serialize`/`deserialize` use a versioned, locale-independent compact prefs value.
Deserialization leaves its destination unchanged on failure.

Latency search uses 1 ms steps from 0 to 250 ms, matching four-frame optical
angular differences against integrated gyro windows. Rate magnitudes remove
mounting dependence. Horn's quaternion alignment uses a Jacobi eigensolver;
excitation is checked using the smallest eigenvalue of rate second moments.
Large optical flips are excluded from calibration's rate pairs.

Fusion strength `s` uses `tau = 20 * pow(0.5/20, s)` seconds, giving 3.162 s at
0.5. History holds 1.2 s. Optical jumps over max(25 degrees, 400 deg/s times
frame interval) or disagreement over 35 degrees are gated. A consistent stream
after 0.5 s can re-acquire; gain ramps over 1 s and correction is limited to
20 deg/s. Bias adaptation has a 60 s time constant and requires small gyro
rate, stable optical pose, low acceleration variance and gravity sanity.
Translation has an 80 ms time constant and is held when the face is lost.
Drift diagnostics are the norm of the signed optical correction-rate vector,
averaged with a 10 s time constant, expressed in deg/min (not head speed).

IMU silence over 100 ms starts a 0.5 s crossfade. On return, orientation seeds
from recent optical (or the last output if unavailable), and fades back over
0.5 s. Transition speed is limited to 150 deg/s. Optical-only rotation uses
45 ms smoothing. If both sources disappear, the last output is held.

## Simulator and verification limits

`Simulator` generates deterministic seeded event streams, analytic truth,
arbitrary mounting, noisy biased gyro with random walk, acceleration/gravity,
32-bit wire samples, drifting MCU clocks, ordered USB arrival jitter, delayed
30 Hz optical, face loss, IMU outages, translations and PnP outliers. Motion
profiles include guided calibration, natural motion with fast turns, a 0.2
degree sinusoid and a 2 deg/s voluntary ramp. It performs no file or network I/O.

Clock sync uses a 20 s window with one lower-envelope point per 250 ms and a
3 s drift time constant. Constant minimum USB transport delay is unobservable;
mapped time contains that offset. Latency calibration includes this offset and
any constant IMU filter delay relative to optical. Integration cannot recover
motion during an actual IMU outage. Accelerometers do not correct orientation.
Thresholds/noise assumptions need validation on hardware and recorded webcam
data; this simulator does not model flexible mounting, rolling shutter or PnP
convention errors that cannot be represented by a rigid rotation.
