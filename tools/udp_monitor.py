"""Listen to the AITrack -> OpenTrack UDP stream and report rate and pose.

Usage: python tools/udp_monitor.py [--port 4242] [--seconds 10]
Close OpenTrack first (it owns the same port).
Packet: 6 little-endian doubles [x, y, z, yaw, pitch, roll] (cm, deg).
"""
import argparse
import socket
import statistics
import struct
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=4242)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)

    t_end = time.perf_counter() + args.seconds
    last_print = time.perf_counter()
    arrivals, poses, bad = [], [], 0
    window = 0
    while time.perf_counter() < t_end:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        now = time.perf_counter()
        if len(data) != 48:
            bad += 1
            continue
        arrivals.append(now)
        poses.append(struct.unpack("<6d", data))
        window += 1
        if now - last_print >= 1.0:
            x, y, z, yaw, pitch, roll = poses[-1]
            print(f"{window / (now - last_print):6.1f} Hz  "
                  f"x={x:6.2f} y={y:6.2f} z={z:6.2f}  "
                  f"yaw={yaw:7.2f} pitch={pitch:7.2f} roll={roll:7.2f}")
            window, last_print = 0, now

    if len(arrivals) < 2:
        print("no packets received")
        return 1
    gaps = [(b - a) * 1000 for a, b in zip(arrivals, arrivals[1:])]
    gaps.sort()
    rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
    print(f"\npackets={len(arrivals)} bad={bad} rate={rate:.1f} Hz "
          f"gap p50={gaps[len(gaps) // 2]:.2f} ms p99={gaps[int(len(gaps) * 0.99)]:.2f} ms "
          f"max={gaps[-1]:.2f} ms")
    for i, name in enumerate(["x", "y", "z", "yaw", "pitch", "roll"]):
        vals = [p[i] for p in poses]
        print(f"{name:>5}: mean={statistics.fmean(vals):8.3f} std={statistics.pstdev(vals):7.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
