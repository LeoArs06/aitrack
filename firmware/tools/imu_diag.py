#!/usr/bin/env python3
"""Protocol v1 raw IMU diagnostic. Runtime dependency: pyserial only."""
import argparse
import csv
import math
import statistics
import struct
import sys
import time

SYNC = b"\xa5\x5a"
SAMPLE = struct.Struct("<II7hBB")
INFO = struct.Struct("<I8BHHIII")
CHIPS = {0x68: "MPU6050", 0x70: "MPU6500", 0x71: "MPU9250",
         0x73: "MPU9255", 0x98: "compatible (0x98)",
         0x19: "compatible (0x19)", 0x12: "compatible (0x12)"}


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def frame(kind, payload):
    body = bytes((kind, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class Parser:
    """Resyncs counts discard operations, including CRC/length rejection.

    Keep a trailing A5 for split sync. CRC failure discards just sync0,
    allowing valid frames nested in a damaged frame to be recovered.
    """
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while self.buf:
            pos = self.buf.find(SYNC)
            if pos < 0:
                count = len(self.buf) - (self.buf[-1] == 0xA5)
                if count:
                    del self.buf[:count]
                    self.resyncs += 1
                break
            if pos:
                del self.buf[:pos]
                self.resyncs += 1
            if len(self.buf) < 4:
                break
            length = self.buf[3]
            if length > 64:
                del self.buf[0]
                self.resyncs += 1
                continue
            size = length + 6
            if len(self.buf) < size:
                break
            expected = struct.unpack_from("<H", self.buf, size - 2)[0]
            if crc16(self.buf[2:size-2]) != expected:
                self.crc_errors += 1
                self.resyncs += 1
                del self.buf[0]
                continue
            out.append((self.buf[2], bytes(self.buf[4:size-2])))
            del self.buf[:size]
        return out


def decode_info(payload):
    if len(payload) != 28:
        return None
    v = INFO.unpack(payload)
    if v[0] != 0x31495448 or v[1] != 1:
        return None
    return dict(zip(("magic", "protocol", "major", "minor", "who", "gyro_fs",
                     "accel_fs", "dlpf", "divider", "rate", "i2c_khz",
                     "i2c_errors", "reinits", "tx_drops"), v))


def temperature(raw, who):
    return raw / 340 + 36.53 if who == 0x68 else raw / 333.87 + 21


class Run:
    def __init__(self):
        self.info = None
        self.samples = []  # host perf_counter, MCU elapsed seconds, segment, raw tuple
        self.lost = 0
        self.flags = 0
        self.restarts = 0
        self.status = {}
        self.last = None
        self.elapsed_us = 0

    def accept(self, kind, payload, host):
        if kind == 2:
            info = decode_info(payload)
            if info:
                self.info = info
        elif kind == 3 and len(payload) == 8:
            code, _, _, errors = struct.unpack("<BBHI", payload)
            self.status[code] = self.status.get(code, 0) + 1
            print(f"\nSTATUS code={code}, I2C errors={errors}")
        elif kind == 1 and len(payload) == SAMPLE.size:
            v = SAMPLE.unpack(payload)
            if self.last:
                ds = (v[0] - self.last[0]) & 0xFFFFFFFF
                dt = (v[1] - self.last[1]) & 0xFFFFFFFF
                if ds == 0:  # Duplicate frame; do not distort loss or timing.
                    return
                if ds >= 0x80000000 or dt >= 0x80000000:
                    self.restarts += 1
                    self.elapsed_us += round(max(0, host - self.samples[-1][0]) * 1e6)
                else:
                    self.lost += ds - 1
                    self.elapsed_us += dt
            self.last = v
            self.flags |= v[9]
            self.samples.append((host, self.elapsed_us / 1e6, self.restarts, v))


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return float("nan")
    index = (len(values) - 1) * fraction
    lo = int(index)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def slope(points):
    if len(points) < 2:
        return float("nan")
    mx = statistics.fmean(x for x, _ in points)
    my = statistics.fmean(y for _, y in points)
    denominator = sum((x - mx)**2 for x, _ in points)
    return sum((x-mx)*(y-my) for x, y in points) / denominator if denominator else float("nan")


def report(run, parser):
    rows = run.samples
    info = run.info or {}
    who = info.get("who")
    print(f"\nChip: {CHIPS.get(who, 'unknown compatible')} / WHO_AM_I="
          f"{f'0x{who:02X}' if who is not None else 'unavailable'}")
    if info:
        print(f"Firmware {info['major']}.{info['minor']}; I2C errors={info['i2c_errors']}, "
              f"reinits={info['reinits']}, TX drops={info['tx_drops']} (last INFO)")
    if not rows:
        print(f"No samples. CRC errors={parser.crc_errors}, resyncs={parser.resyncs}; STATUS={run.status}")
        return 1
    duration = rows[-1][0] - rows[0][0]
    rate = (len(rows) - 1) / duration if duration else 0
    lost_pct = 100 * run.lost / (len(rows) + run.lost)
    print(f"Rate received: {rate:.3f} Hz; nominal: {info.get('rate', 200)} Hz; "
          f"samples={len(rows)}, lost={run.lost} ({lost_pct:.4f}%)")
    print(f"CRC errors={parser.crc_errors}, resyncs={parser.resyncs}, "
          f"flags=0x{run.flags:02X}, MCU restarts={run.restarts}")
    # Fit only the longest continuous boot segment. One minimum-latency
    # observation per MCU second reduces USB batching / scheduler outliers.
    segments = {}
    for row in rows:
        segments.setdefault(row[2], []).append(row)
    segment = max(segments.values(), key=len)
    h0, m0 = segment[0][:2]
    buckets = {}
    for host, mcu, _, _ in segment:
        x, y = mcu - m0, host - h0
        key = int(x)
        if key not in buckets or y-x < buckets[key][1]-buckets[key][0]:
            buckets[key] = (x, y)
    fit = slope(list(buckets.values()))
    ppm = (1 / fit - 1) * 1e6 if fit > 0 else float("nan")
    print(f"MCU vs host clock drift: {ppm:+.1f} ppm (positive = MCU faster; minimum-latency fit)")
    jitter = [(b[0]-a[0])*1000 for a, b in zip(rows, rows[1:])]
    print(f"USB host inter-arrival: p50={percentile(jitter, .50):.3f} ms, "
          f"p99={percentile(jitter, .99):.3f} ms (includes batching and gaps)")
    gyro = [[r[3][i] / 32.8 for r in rows] for i in (6, 7, 8)]
    accel = [[r[3][i] / 8192 for r in rows] for i in (2, 3, 4)]
    for axis, values in zip("XYZ", gyro):
        print(f"Gyro {axis}: mean (bias)={statistics.fmean(values):+.5f} dps, "
              f"std (noise)={statistics.pstdev(values):.5f} dps")
    norms = [math.sqrt(sum(r[3][i]**2 for i in (2, 3, 4))) / 8192 for r in rows]
    print(f"Accel norm mean={statistics.fmean(norms):.5f} g; "
          "axis std (g): " + ", ".join(f"{a}={statistics.pstdev(v):.5f}" for a, v in zip("XYZ", accel)))
    calibration = [r for r in rows if r[1] - rows[0][1] < 5 and r[2] == rows[0][2]]
    bias = [statistics.fmean(r[3][i] / 32.8 for r in calibration) for i in (6, 7, 8)]
    angle = [0.0] * 3
    integrated_time = 0.0
    for a, b in zip(rows, rows[1:]):
        if a[2] != b[2] or a[1] - rows[0][1] < 5:
            continue
        dt = b[1] - a[1]
        integrated_time += dt
        for j, i in enumerate((6, 7, 8)):
            angle[j] += ((a[3][i] + b[3][i]) / (2 * 32.8) - bias[j]) * dt
    if integrated_time > 0 and not run.restarts:
        print("Integrated gyro drift after first 5 s bias removal (still test), deg/min: " +
              ", ".join(f"{a}={v*60/integrated_time:+.5f}" for a, v in zip("XYZ", angle)))
    else:
        print("Integrated gyro drift: unavailable (need >5 s and no MCU restart)")
    if who is None:
        print("Temperature unavailable: no INFO/chip identity")
    else:
        print(f"Temperature: {temperature(rows[0][3][5], who):.2f} -> "
              f"{temperature(rows[-1][3][5], who):.2f} °C")
    return 0


def open_serial(serial, port):
    device = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.2)
    device.dtr = True
    device.port = port
    device.open()
    return device


def connect(serial, requested):
    if requested:
        device = open_serial(serial, requested)
        device.write(b"?\n")
        return device, Parser(), []
    from serial.tools import list_ports
    ports = sorted(list_ports.comports(), key=lambda p: p.vid not in (0x2341, 0x1B4F, 0x2A03))
    for port in ports:
        device = None
        try:
            print(f"Probing {port.device} at 115200, DTR on...")
            device = open_serial(serial, port.device)
            parser, received = Parser(), []
            end, query = time.perf_counter() + 2.5, 0
            while time.perf_counter() < end:
                now = time.perf_counter()
                if now >= query:
                    device.write(b"?\n")
                    query = now + 0.5
                data = device.read(min(max(device.in_waiting, 1), 4096))
                host = time.perf_counter()
                frames = parser.feed(data)
                received.extend((kind, payload, host) for kind, payload in frames)
                if any(kind == 2 and decode_info(payload) for kind, payload in frames):
                    return device, parser, received
        except (serial.SerialException, OSError) as exc:
            print(f"  {exc}", file=sys.stderr)
        if device is not None:
            device.close()
    raise RuntimeError("No protocol-v1 IMU found. Use --port COMx to select explicitly.")


def selftest():
    import contextlib
    import io
    assert crc16(b"123456789") == 0x29B1
    payload = SAMPLE.pack(7, 123, 1, -2, 8192, 340, -32768, 32767, 0, 15, 0)
    good = frame(1, payload)
    parser = Parser()
    assert parser.feed(good[:1]) == []
    assert parser.feed(good[1:9]) == []
    assert parser.feed(good[9:]) == [(1, payload)]
    bad = bytearray(good)
    bad[10] ^= 0x80
    parser = Parser()
    output = parser.feed(b"junk\xa5" + b"\xa5\x5a\x01\xff" + bytes(bad) + good)
    assert output == [(1, payload)] and parser.crc_errors == 1 and parser.resyncs >= 3
    # A corrupt outer frame contains a complete valid inner frame.
    parser = Parser()
    outer = b"\xa5\x5a\x7f\x20" + good + b"\x00\x00\x00\x00"
    assert parser.feed(outer) == [(1, payload)] and parser.crc_errors == 1
    info = INFO.pack(0x31495448, 1, 1, 0, 0x68, 2, 1, 3, 4, 200, 400, 2, 1, 9)
    assert decode_info(info)["rate"] == 200
    assert decode_info(info)["tx_drops"] == 9
    assert decode_info(bytes(28)) is None
    stream = good + frame(2, info) + frame(3, struct.pack("<BBHI", 2, 0x68, 0, 4))
    parser = Parser()
    output = []
    for byte in stream:
        output.extend(parser.feed(bytes([byte])))
    assert [kind for kind, _ in output] == [1, 2, 3] and parser.crc_errors == 0
    run = Run()
    for seq, stamp, host in ((0xFFFFFFFE, 0xFFFFF000, 0), (1, 0x2A98, .015), (2, 0x3E20, .020)):
        run.accept(1, SAMPLE.pack(seq, stamp, 0, 0, 8192, 0, 0, 0, 0, 12, 0), host)
    assert run.lost == 2 and run.flags == 12 and run.restarts == 0
    assert run.elapsed_us == 20000
    run.accept(1, SAMPLE.pack(0, 0, 0, 0, 8192, 0, 0, 0, 0, 0, 0), .03)
    assert run.restarts == 1 and run.lost == 2
    assert temperature(340, 0x68) == 37.53
    assert abs(temperature(0, 0x70) - 21) < 1e-9
    assert percentile([1, 2, 3], .5) == 2
    assert slope([(0, 0), (1, 1.0001), (2, 2.0002)]) == 1.0001
    # A synthetic still run with a 10 dps bias step after calibration gives
    # 600 deg/min residual drift. Exercise the actual report, including fit.
    run = Run()
    run.accept(2, info, 0)
    parser = Parser()
    for seq in range(1201):
        raw = SAMPLE.pack(seq, seq * 5000, 0, 0, 8192, 340,
                          0 if seq < 1000 else 328, 0, 0, 0, 0)
        for kind, payload in parser.feed(frame(1, raw)):
            run.accept(kind, payload, seq * .005 / 1.0001)
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        assert report(run, parser) == 0
        assert report(Run(), Parser()) == 1
    result = output.getvalue()
    assert '200.020 Hz' in result and '+100.0 ppm' in result
    assert 'X=+600.00000' in result and 'norm mean=1.00000 g' in result
    assert '37.53 -> 37.53' in result and 'No samples.' in result
    print("Selftest PASS: CRC vector, chunking, corrupt/nested frames, length limit, INFO/STATUS, sequence/timestamp wrap, restart, synthetic rate/clock/drift report, no-sample failure.")
    return 0


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--port", help="serial port; default: VID-prioritized discovery verified with '?' / INFO")
    args.add_argument("--seconds", type=float, default=30)
    args.add_argument("--csv", help="raw SAMPLE dump with host wall/monotonic receive time")
    args.add_argument("--selftest", action="store_true", help="test without pyserial or hardware")
    opts = args.parse_args()
    if opts.selftest:
        return selftest()
    if not math.isfinite(opts.seconds) or opts.seconds <= 0:
        args.error("--seconds must be finite and positive")
    try:
        import serial
    except ImportError:
        print("Install pyserial: python -m pip install pyserial", file=sys.stderr)
        return 2
    device = csv_file = None
    run, parser = Run(), Parser()
    exit_status = 0
    try:
        device, parser, initial = connect(serial, opts.port)
        print(f"Connected: {device.port}; hold still for bias/noise/drift measurements.")
        writer = None
        wall_offset = time.time() - time.perf_counter()
        if opts.csv:
            csv_file = open(opts.csv, "w", newline="", encoding="utf-8")
            writer = csv.writer(csv_file)
            writer.writerow(("host_unix_s", "host_monotonic_s", "seq", "t_us", "ax", "ay", "az",
                             "temp_raw", "gx", "gy", "gz", "flags", "reserved"))

        def accept(frames):
            for kind, payload, host in frames:
                run.accept(kind, payload, host)
                if writer and kind == 1 and len(payload) == SAMPLE.size:
                    writer.writerow((f"{host+wall_offset:.6f}", f"{host:.9f}", *SAMPLE.unpack(payload)))

        accept(initial)
        start = time.perf_counter()
        next_line, query_at, last_line, last_count = start + 1, start + 1, start, len(run.samples)
        while time.perf_counter() - start < opts.seconds:
            data = device.read(min(max(device.in_waiting, 1), 4096))
            now = time.perf_counter()
            accept((kind, payload, now) for kind, payload in parser.feed(data))
            if now >= query_at:
                device.write(b"?\n")
                query_at = now + 1
            if now >= next_line:
                count = len(run.samples)
                who = (run.info or {}).get("who")
                temp = temperature(run.samples[-1][3][5], who) if count and who is not None else float("nan")
                print(f"{(count-last_count)/(now-last_line):7.2f} Hz | lost {run.lost} | "
                      f"CRC {parser.crc_errors} | resync {parser.resyncs} | "
                      f"flags 0x{run.flags:02X} | {temp:.2f} °C", flush=True)
                last_count, last_line, next_line = count, now, now + 1
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except (serial.SerialException, OSError, RuntimeError) as exc:
        print(f"Diagnostic error: {exc}", file=sys.stderr)
        exit_status = 2
    finally:
        if device is not None:
            device.close()
        if csv_file is not None:
            csv_file.close()
    report_status = report(run, parser)
    return exit_status or report_status


if __name__ == "__main__":
    sys.exit(main())
