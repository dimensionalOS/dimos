#!/usr/bin/env python3
"""Drive M20 yaw-in-place via LCM teleop + log IMU, safely.

Safety guards:
  - SIGINT/SIGTERM/atexit all flood 2s of zero Twists at 50Hz on exit.
  - On normal exit, flood 2s of zero Twists before ending.
  - Default yaw 0.4 rad/s (not 0.8).
  - Print "ROBOT SHOULD BE STOPPED" at end.
"""
import argparse
import atexit
import signal
import statistics
import sys
import time

import lcm
from dimos_lcm.geometry_msgs.Twist import Twist
from dimos_lcm.sensor_msgs.Imu import Imu


# ---------------------------------------------------------------------------
# Safety: publish zero Twists for 2s at 50Hz, survives SIGINT/SIGTERM/exit.
# Using a dedicated LCM instance for the stop-flood so a broken main-loop
# instance doesn't prevent us from reaching the mux.
# ---------------------------------------------------------------------------

STOP_TOPIC = "/tele_cmd_vel#geometry_msgs.Twist"
_stop_armed = [True]
_stop_topic = [STOP_TOPIC]


def _flood_zeros(reason: str) -> None:
    if not _stop_armed[0]:
        return
    _stop_armed[0] = False  # idempotent
    try:
        lc = lcm.LCM()
        z = Twist()
        z.angular.z = 0.0
        print(f"[safety] flooding zero Twists for 2s ({reason})",
              file=sys.stderr, flush=True)
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < 2.0:
            lc.publish(_stop_topic[0], z.lcm_encode())
            n += 1
            time.sleep(0.02)  # 50 Hz
        print(f"[safety] sent {n} zero Twists. ROBOT SHOULD BE STOPPED.",
              file=sys.stderr, flush=True)
    except Exception as e:
        print(f"[safety] WARN flood_zeros exception: {e!r}",
              file=sys.stderr, flush=True)


def _sig_handler(signum: int, frame: object) -> None:
    _flood_zeros(f"signal {signum}")
    sys.exit(128 + signum)


atexit.register(_flood_zeros, "atexit")
signal.signal(signal.SIGINT, _sig_handler)
signal.signal(signal.SIGTERM, _sig_handler)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic_imu", default="/airy_imu_front#sensor_msgs.Imu")
    ap.add_argument("--topic_cmd", default=STOP_TOPIC)
    ap.add_argument("--yaw", type=float, default=-0.4,
                    help="Angular velocity (rad/s). Default safe value.")
    ap.add_argument("--rotate_duration", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--cooldown", type=float, default=3.0)
    ap.add_argument("--pub_hz", type=float, default=20.0)
    ap.add_argument("--out", default="/tmp/gyro_auto.csv")
    args = ap.parse_args()
    _stop_topic[0] = args.topic_cmd

    lc = lcm.LCM()

    def mk_twist(wz: float) -> Twist:
        t = Twist(); t.angular.z = wz; return t

    rotate_msg = mk_twist(args.yaw)

    rows: list = []
    first_wall: list = []

    def handler(ch: str, data: bytes) -> None:
        try:
            m = Imu.lcm_decode(data)
        except Exception:
            return
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        if not first_wall:
            first_wall.append(time.monotonic())
        rows.append((t, time.monotonic(),
                     m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                     m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

    lc.subscribe(args.topic_imu, handler)
    print(f"[auto] IMU: {args.topic_imu}", file=sys.stderr, flush=True)
    print(f"[auto] CMD: {args.topic_cmd} wz={args.yaw} @ {args.pub_hz} Hz",
          file=sys.stderr, flush=True)

    while not first_wall:
        lc.handle_timeout(200)
    t0 = first_wall[0]
    pub_interval = 1.0 / args.pub_hz

    # WARMUP (no publishing)
    print(f"[auto] t+0.0  WARMUP ({args.warmup}s stationary)",
          file=sys.stderr, flush=True)
    while time.monotonic() < t0 + args.warmup:
        lc.handle_timeout(20)

    # ROTATION — interleave publish + handle in main thread
    t_rot = time.monotonic()
    rot_end = t_rot + args.rotate_duration
    print(f"[auto] t+{t_rot-t0:.1f}  ROTATE ({args.rotate_duration}s at wz={args.yaw})",
          file=sys.stderr, flush=True)
    next_pub = t_rot
    pub_count = 0
    while time.monotonic() < rot_end:
        now = time.monotonic()
        if now >= next_pub:
            lc.publish(args.topic_cmd, rotate_msg.lcm_encode())
            pub_count += 1
            next_pub += pub_interval
        remaining_ms = max(1, min(20, int((next_pub - now) * 1000)))
        lc.handle_timeout(remaining_ms)
    print(f"[auto] t+{time.monotonic()-t0:.1f}  published {pub_count} rotation cmds",
          file=sys.stderr, flush=True)

    # STOP FLOOD — rely on atexit handler (safe against exception / signal)
    _flood_zeros("end of rotation phase")

    # COOLDOWN with IMU capture
    cool_end = time.monotonic() + args.cooldown
    print(f"[auto] t+{time.monotonic()-t0:.1f}  COOLDOWN ({args.cooldown}s stationary)",
          file=sys.stderr, flush=True)
    while time.monotonic() < cool_end:
        lc.handle_timeout(20)

    # Save + summary
    with open(args.out, "w") as f:
        f.write("sensor_t,wall_t,gx,gy,gz,ax,ay,az\n")
        for r in rows:
            f.write(",".join(f"{v:.6f}" for v in r) + "\n")

    def stats(label: str, rs: list) -> None:
        if not rs:
            print(f"-- {label}: empty", file=sys.stderr); return
        print(f"\n-- {label} (n={len(rs)}):", file=sys.stderr)
        for i, name in enumerate(["gx","gy","gz","ax","ay","az"], start=2):
            vals = [r[i] for r in rs]
            print(f"   {name:4s} mean={statistics.mean(vals):+7.4f} stdev={statistics.stdev(vals) if len(vals)>1 else 0:.4f}",
                  file=sys.stderr)

    stat_rows = [r for r in rows if r[1] < t0 + args.warmup]
    rot_rows  = [r for r in rows if t_rot <= r[1] < rot_end]
    cool_rows = [r for r in rows if r[1] >= rot_end]
    stats("WARMUP stationary", stat_rows)
    stats("ROTATION active",   rot_rows)
    stats("COOLDOWN",          cool_rows)
    if stat_rows and rot_rows:
        for i, name in [(5,"ax"),(6,"ay"),(7,"az")]:
            d = statistics.mean(r[i] for r in rot_rows) - statistics.mean(r[i] for r in stat_rows)
            print(f"   Δ{name} (rot − stat) = {d:+.3f} m/s²", file=sys.stderr)

    print(f"\nCSV: {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
