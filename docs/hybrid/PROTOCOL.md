# IMU serial protocol v1 (Pro Micro → PC)

USB CDC serial (baud ignored by CDC; host opens at 115200, **never 1200** —
1200 baud triggers the bootloader). Host asserts DTR. All multi-byte fields
little-endian.

## Frame (device → host)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | sync0 = `0xA5` |
| 1 | 1 | sync1 = `0x5A` |
| 2 | 1 | type |
| 3 | 1 | len = payload length (bytes) |
| 4 | len | payload |
| 4+len | 2 | CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, xorout 0) over bytes `type, len, payload` |

Parser: resync by scanning for `A5 5A`; reject `len > 64`; on CRC failure skip
one byte after sync0 and rescan. Count CRC errors.

### type 0x01 SAMPLE (len 24)

| Off | Type | Field |
|---|---|---|
| 0 | u32 | seq — increments by 1 for every acquired sample (also for samples dropped because the USB TX buffer was full) |
| 4 | u32 | t_us — `micros()` captured right after DATA_RDY was seen (wraps at 2^32) |
| 8 | i16×7 | ax, ay, az, temp_raw, gx, gy, gz — raw registers 0x3B..0x48 in chip order |
| 22 | u8 | flags: bit0 accel saturated (any axis ±32767/−32768), bit1 gyro saturated, bit2 IMU was re-initialised since previous sent sample, bit3 one or more samples dropped (TX full) since previous sent sample |
| 23 | u8 | reserved = 0 |

Lost samples on host = seq gap − 1.

### type 0x02 INFO (len 28)

Sent once at boot after IMU init, on every successful re-init, and on request.

| Off | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x31495448` ("HTI1" as LE bytes `48 54 49 31`) |
| 4 | u8 | protocol version = 1 |
| 5 | u8 | firmware major |
| 6 | u8 | firmware minor |
| 7 | u8 | WHO_AM_I value read (0x68 MPU6050, 0x70 MPU6500, 0x71 MPU9250, 0x73 MPU9255, 0x98/0x19/0x12 other; 0 if not found) |
| 8 | u8 | gyro FS code (0..3 = ±250/500/1000/2000 dps) |
| 9 | u8 | accel FS code (0..3 = ±2/4/8/16 g) |
| 10 | u8 | DLPF_CFG |
| 11 | u8 | SMPLRT_DIV |
| 12 | u16 | nominal sample rate Hz |
| 14 | u16 | I²C clock kHz |
| 16 | u32 | i2c_error_count (since boot) |
| 20 | u32 | reinit_count |
| 24 | u32 | tx_drop_count |

### type 0x03 STATUS (len 8)

Sent at ≤2 Hz while the IMU is absent/failing, and once when it recovers.

| Off | Type | Field |
|---|---|---|
| 0 | u8 | code: 1 = IMU not found, 2 = I²C error/timeout, 3 = recovered, 4 = data-ready timeout (no sample for >50 ms) |
| 1 | u8 | WHO_AM_I last read |
| 2 | u16 | reserved |
| 4 | u32 | i2c_error_count |

## Commands (host → device)

ASCII, one char + `\n` (CR tolerated). Unknown ignored.

| Cmd | Effect |
|---|---|
| `?` | send INFO now |
| `R` | re-initialise IMU (bus recovery + register config), then INFO |
| `0` | stop streaming SAMPLE |
| `1` | resume streaming (default after boot) |

## Fixed configuration v1

Gyro ±1000 dps (32.8 LSB/dps), accel ±4 g (8192 LSB/g), DLPF_CFG = 3
(gyro ≈42 Hz, ≈4.8 ms delay), SMPLRT_DIV = 4 → 200 Hz, I²C 400 kHz.
Temperature: MPU6050 `t = raw/340 + 36.53`; MPU6500 family `t = raw/333.87 + 21`.
Host converts; firmware sends raw.
