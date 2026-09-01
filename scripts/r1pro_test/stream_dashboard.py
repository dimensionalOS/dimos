# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Live stream-flow dashboard — standalone (rclpy + stdlib only, no dimos).

Renders a per-topic sparkline of delivered Hz in the terminal (works over
SSH), so you can SEE how streams behave over time and whether they interact.

    stream               sub    Hz   MiB/s  gap(s) pubs  flow (1 char = 1s)
    imu_chassis          yes  200.1   0.06    0.0    1   ████████████████████
    chassis_front_left   yes   15.5   7.53    0.1    1   ▇███▇██▇▁···········
                                                              ^ collapse visible

Modes:
  ALL AT ONCE (default)   subscribe to every topic immediately.
  STAGGERED (--stagger N) subscribe to topics one at a time, lightest first,
      adding the next every N seconds. This answers "does it matter how many
      topics I subscribe to?" in a single run: watch whether the streams that
      were healthy degrade at the moment a heavy topic joins. Unsubscribed
      topics still show their `pubs` count (discovery state) before joining.

Run on the PC (measures wire delivery):
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/stream_dashboard.py --stagger 15 --csv /tmp/flow_pc.csv

Run on the ROBOT (measures what the drivers actually produce):
    scp scripts/r1pro_test/stream_dashboard.py <robot>:/tmp/
    ssh <robot>  # then: source ROS env, export ROS_DOMAIN_ID=41
    python3 /tmp/stream_dashboard.py --csv /tmp/flow_robot.csv

Interpreting PC vs robot runs:
    robot full rate + PC collapsed  -> egress/wire problem (send buffer, load)
    robot also collapsed            -> producer problem (driver/CPU on robot)
    (Local readers on the robot are usually served via shared memory, so the
    robot-side run adds ~no network load and is safe to run during PC tests.)

If stdout is not a TTY (e.g. `| tee log.txt`) it falls back to plain text.
"""

import argparse
from collections import deque
import sys
import threading
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# name -> (ros topic, type path). Ordered lightest-first for --stagger.
TOPICS: dict[str, tuple[str, str]] = {
    "imu_chassis": ("/hdas/imu_chassis", "sensor_msgs/msg/Imu"),
    "imu_torso": ("/hdas/imu_torso", "sensor_msgs/msg/Imu"),
    "lidar": ("/hdas/lidar_chassis_left", "sensor_msgs/msg/PointCloud2"),
    "wrist_left_color": (
        "/hdas/camera_wrist_left/color/image_raw/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "wrist_right_color": (
        "/hdas/camera_wrist_right/color/image_raw/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "head_color": (
        "/hdas/camera_head/left_raw/image_raw_color/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "chassis_front_left": (
        "/hdas/camera_chassis_front_left/rgb/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "chassis_front_right": (
        "/hdas/camera_chassis_front_right/rgb/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "chassis_left": ("/hdas/camera_chassis_left/rgb/compressed", "sensor_msgs/msg/CompressedImage"),
    "chassis_right": (
        "/hdas/camera_chassis_right/rgb/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "chassis_rear": ("/hdas/camera_chassis_rear/rgb/compressed", "sensor_msgs/msg/CompressedImage"),
    "wrist_left_depth": (
        "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
        "sensor_msgs/msg/Image",
    ),
    "wrist_right_depth": (
        "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
        "sensor_msgs/msg/Image",
    ),
    "head_depth": ("/hdas/camera_head/depth/depth_registered", "sensor_msgs/msg/Image"),
}

QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

BLOCKS = "▁▂▃▄▅▆▇█"


def import_msg_type(path: str):
    pkg, _, name = path.split("/")
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


class Stream:
    def __init__(self, width: int) -> None:
        self.count = 0  # current 1s bucket
        self.bytes = 0
        self.total = 0
        self.last_arrival = 0.0  # monotonic
        self.hist_hz: deque[float] = deque(maxlen=width)
        self.hist_bytes: deque[int] = deque(maxlen=width)
        self.subscribed_at: float | None = None


def sparkline(hist: deque[float]) -> str:
    peak = max(hist, default=0.0)
    if peak <= 0:
        return "·" * len(hist)
    out = []
    for v in hist:
        if v <= 0:
            out.append("·")
        else:
            out.append(BLOCKS[min(len(BLOCKS) - 1, int(v / peak * (len(BLOCKS) - 1) + 0.5))])
    return "".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description="Live per-topic stream-flow dashboard")
    ap.add_argument(
        "--stagger",
        type=float,
        default=0.0,
        help="subscribe to one topic every N seconds, lightest first (0 = all at once)",
    )
    ap.add_argument(
        "--only", action="append", default=None, help="substring filter on stream name; repeatable"
    )
    ap.add_argument("--width", type=int, default=60, help="sparkline history length (seconds)")
    ap.add_argument("--csv", type=str, default=None, help="append per-second rows to this CSV")
    args = ap.parse_args()

    names = list(TOPICS)
    if args.only:
        names = [n for n in names if any(f in n for f in args.only)]
        if not names:
            raise SystemExit(f"--only {args.only} matched no streams of: {list(TOPICS)}")

    rclpy.init()
    node = rclpy.create_node("dimos_stream_dashboard")

    lock = threading.Lock()
    streams: dict[str, Stream] = {n: Stream(args.width) for n in names}

    def make_cb(name: str):
        st = streams[name]

        def cb(raw: bytes) -> None:
            with lock:
                st.count += 1
                st.total += 1
                st.bytes += len(raw)
                st.last_arrival = time.monotonic()

        return cb

    def subscribe(name: str, t: float) -> None:
        topic, type_path = TOPICS[name]
        node.create_subscription(import_msg_type(type_path), topic, make_cb(name), QOS, raw=True)
        streams[name].subscribed_at = t

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "a", buffering=1)
        if csv_file.tell() == 0:
            csv_file.write("t,stream,subscribed,hz,mibps,gap_s,pubs,total,wall\n")

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    t0 = time.monotonic()
    pending = list(names)
    if args.stagger <= 0:
        for n in pending:
            subscribe(n, 0.0)
        pending = []

    tty = sys.stdout.isatty()
    if tty:
        sys.stdout.write("\x1b[2J")  # clear once; each frame homes + overwrites

    try:
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            t = now - t0

            if (
                pending
                and t
                >= len([s for s in streams.values() if s.subscribed_at is not None]) * args.stagger
            ):
                subscribe(pending.pop(0), t)

            rows = []
            total_bytes = 0
            with lock:
                for name in names:
                    st = streams[name]
                    hz, by = float(st.count), st.bytes
                    st.count = 0
                    st.bytes = 0
                    if st.subscribed_at is not None:
                        st.hist_hz.append(hz)
                        st.hist_bytes.append(by)
                        total_bytes += by
                    gap = (now - st.last_arrival) if st.last_arrival else float("inf")
                    rows.append((name, st, hz, by, gap))

            frame = []
            n_sub = sum(1 for _, st, *_ in rows if st.subscribed_at is not None)
            frame.append(
                f"R1 Pro stream flow   t=+{t:6.1f}s   subscribed {n_sub}/{len(names)}"
                f"   aggregate {total_bytes / (1024 * 1024):6.2f} MiB/s"
                f"{'   [staggered +1 every ' + str(args.stagger) + 's]' if args.stagger > 0 else ''}"
            )
            frame.append(
                f"{'stream':<20} {'sub':>4} {'Hz':>7} {'MiB/s':>7} {'gap(s)':>7} {'pubs':>4}  flow (1 char = 1s, · = silent)"
            )
            for name, st, hz, by, gap in rows:
                pubs = node.count_publishers(TOPICS[name][0])
                if st.subscribed_at is None:
                    frame.append(
                        f"{name:<20} {'--':>4} {'':>7} {'':>7} {'':>7} {pubs:>4}  (waiting to subscribe)"
                    )
                    continue
                gap_s = f"{gap:.1f}" if gap != float("inf") else "-"
                frame.append(
                    f"{name:<20} {'yes':>4} {hz:>7.1f} {by / (1024 * 1024):>7.2f} {gap_s:>7} {pubs:>4}  {sparkline(st.hist_hz)}"
                )
                if csv_file:
                    csv_file.write(
                        f"{t:.1f},{name},1,{hz:.1f},{by / (1024 * 1024):.3f},"
                        f"{gap if gap != float('inf') else -1:.2f},{pubs},{st.total}\n"
                    )

            if tty:
                sys.stdout.write("\x1b[H" + "\x1b[K\n".join(frame) + "\x1b[K\n\x1b[J")
            else:
                sys.stdout.write("\n" + "\n".join(frame) + "\n")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
        # Ctrl-C races rclpy's own signal handler, which may already have
        # shut the context down — swallow the double-shutdown error.
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
