#!/usr/bin/env python3
"""Gyro purity test logger for M20 airy_imu_bridge.

Subscribes to LCM topic `airy_imu_front`, captures N seconds of IMU data,
computes rho = sqrt(gx**2 + gy**2) / |gz| across "active rotation" samples.

Usage:
    python3 gyro_purity.py --duration 10 --out /tmp/gyro_test.csv
"""
import argparse
import math
import statistics
import sys
import time

import lcm
from dimos_lcm.sensor_msgs.Imu import Imu


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/airy_imu_front#sensor_msgs.Imu")
    ap.add_argument("--duration", type=float, default=10.0,
                    help="Seconds to record after first sample.")
    ap.add_argument("--out", default="/tmp/gyro_test.csv")
    ap.add_argument("--active-gz", type=float, default=0.3,
                    help="|gz| threshold (rad/s) to call a sample 'actively rotating'.")
    args = ap.parse_args()

    lc = lcm.LCM()
    rows: list[tuple[float, float, float, float, float, float, float]] = []
    first_wall: list[float] = []

    def handler(channel: str, data: bytes) -> None:
        try:
            m = Imu.lcm_decode(data)
        except Exception as e:
            print(f"decode err: {e}", file=sys.stderr, flush=True)
            return
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        if not first_wall:
            first_wall.append(time.monotonic())
            print(f"[logger] first sample, sensor_ts={t:.6f}",
                  file=sys.stderr, flush=True)
        rows.append((t,
                     m.angular_velocity.x,
                     m.angular_velocity.y,
                     m.angular_velocity.z,
                     m.linear_acceleration.x,
                     m.linear_acceleration.y,
                     m.linear_acceleration.z))

    lc.subscribe(args.topic, handler)
    print(f"[logger] subscribed to '{args.topic}', waiting for first sample...",
          file=sys.stderr, flush=True)

    while not first_wall:
        lc.handle_timeout(200)

    t_rec_start = first_wall[0]
    last_report = t_rec_start
    while time.monotonic() - t_rec_start < args.duration:
        lc.handle_timeout(100)
        now = time.monotonic()
        if now - last_report > 1.0:
            last_report = now
            elapsed = now - t_rec_start
            print(f"[logger] t+{elapsed:4.1f}s samples={len(rows)}",
                  file=sys.stderr, flush=True)

    print(f"[logger] done, {len(rows)} samples", file=sys.stderr, flush=True)
    if not rows:
        return 2

    with open(args.out, "w") as f:
        f.write("t,gx,gy,gz,ax,ay,az,rho,abs_gz\n")
        for t, gx, gy, gz, ax, ay, az in rows:
            denom = max(abs(gz), 1e-9)
            rho = math.sqrt(gx * gx + gy * gy) / denom
            f.write(f"{t:.6f},{gx:.6f},{gy:.6f},{gz:.6f},"
                    f"{ax:.4f},{ay:.4f},{az:.4f},{rho:.4f},{abs(gz):.4f}\n")

    active = [(gx, gy, gz) for (_, gx, gy, gz, _, _, _) in rows
              if abs(gz) >= args.active_gz]
    all_rho = [math.sqrt(gx * gx + gy * gy) / abs(gz) for gx, gy, gz in active]

    print("\n=== GYRO PURITY SUMMARY ===", file=sys.stderr)
    print(f"total samples:     {len(rows)}", file=sys.stderr)
    print(f"active (|gz|>={args.active_gz}): {len(active)}", file=sys.stderr)
    if active:
        gz_abs = [abs(g) for _, _, g in active]
        gx_vals = [g for g, _, _ in active]
        gy_vals = [g for _, g, _ in active]
        print(f"|gz| mean:  {statistics.mean(gz_abs):.3f} rad/s", file=sys.stderr)
        print(f"|gz| peak:  {max(gz_abs):.3f} rad/s", file=sys.stderr)
        print(f"gx mean:    {statistics.mean(gx_vals):+.3f} rad/s", file=sys.stderr)
        print(f"gy mean:    {statistics.mean(gy_vals):+.3f} rad/s", file=sys.stderr)
        print(f"rho mean:   {statistics.mean(all_rho):.3f}", file=sys.stderr)
        print(f"rho median: {statistics.median(all_rho):.3f}", file=sys.stderr)
        s = sorted(all_rho)
        p05 = s[int(0.05 * (len(s) - 1))]
        p95 = s[int(0.95 * (len(s) - 1))]
        print(f"rho p5/p95: {p05:.3f} / {p95:.3f}", file=sys.stderr)
        if statistics.median(all_rho) > 0.10:
            print("DIAGNOSIS:  yaw DOF error CONFIRMED (rho median > 0.10)",
                  file=sys.stderr)
        else:
            print("DIAGNOSIS:  gyro pure -- frame is OK on yaw axis",
                  file=sys.stderr)
    print(f"CSV:        {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
