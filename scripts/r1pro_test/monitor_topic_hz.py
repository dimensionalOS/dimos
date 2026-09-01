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
Sensor topic frequency monitor — logs per-topic delivery over time, from launch.

Standalone (no dimos imports). Subscribes RAW (no deserialization) with
BEST_EFFORT QoS to the same R1 Pro sensor topics the dimos connection uses,
and every interval prints per topic:

    hz       messages delivered this window
    MiB/s    bytes delivered this window
    gap      seconds since the last message arrived (silence detector)
    stale    now - msg.header.stamp of the newest message (includes any
             robot<->PC clock offset — watch the IMU row for the offset
             baseline; camera staleness beyond that baseline is real lag)
    pubs     count_publishers() — the DDS discovery state for this topic

Interpretation during a stall (the decisive columns are gap + pubs):
    pubs drops to 0        -> discovery/liveliness flap: the PC lost the
                              robot's participant; ALL its topics die together
                              and revive together after rediscovery.
    pubs stays 1, hz -> 0  -> data-path starvation (robot send buffer, wire),
                              discovery is fine.

Protocol:
    1. Baseline: run this WITHOUT dimos running. If topics already stall,
       the problem is robot/wire/discovery — dimos is exonerated.
    2. Contention: run it WITH dimos up. Clean alone + stalling with dimos
       = PC-side contention (CPU / DDS receive) or robot egress overload.

NOTE: Fast DDS unicasts separately to EVERY reader — each topic this monitor
subscribes to roughly doubles the robot's egress for that topic while dimos
is also running. For step 2 use --only to subscribe to a small subset, e.g.:

    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/monitor_topic_hz.py                 # all topics
    python3 scripts/r1pro_test/monitor_topic_hz.py --only imu --only front_left
    python3 scripts/r1pro_test/monitor_topic_hz.py --csv /tmp/hz.csv
"""

import argparse
import struct
import threading
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# name -> (ros topic, message type path). Mirrors R1ProConnection subscriptions.
TOPICS: dict[str, tuple[str, str]] = {
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
    "head_depth": ("/hdas/camera_head/depth/depth_registered", "sensor_msgs/msg/Image"),
    "lidar": ("/hdas/lidar_chassis_left", "sensor_msgs/msg/PointCloud2"),
    "imu_chassis": ("/hdas/imu_chassis", "sensor_msgs/msg/Imu"),
    "imu_torso": ("/hdas/imu_torso", "sensor_msgs/msg/Imu"),
    "wrist_left_color": (
        "/hdas/camera_wrist_left/color/image_raw/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "wrist_left_depth": (
        "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
        "sensor_msgs/msg/Image",
    ),
    "wrist_right_color": (
        "/hdas/camera_wrist_right/color/image_raw/compressed",
        "sensor_msgs/msg/CompressedImage",
    ),
    "wrist_right_depth": (
        "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
        "sensor_msgs/msg/Image",
    ),
}

QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def stamp_from_cdr(buf: bytes) -> float | None:
    """header.stamp as epoch seconds, parsed from raw CDR bytes.

    Every monitored type starts with std_msgs/Header, so after the 4-byte CDR
    encapsulation header the layout is int32 sec, uint32 nanosec (LE).
    """
    if len(buf) < 12:
        return None
    sec, nsec = struct.unpack_from("<iI", buf, 4)
    if sec <= 0:
        return None
    return sec + nsec * 1e-9


class TopicStat:
    __slots__ = ("bytes", "count", "last_arrival", "last_stamp", "total")

    def __init__(self) -> None:
        self.count = 0  # window
        self.bytes = 0  # window
        self.total = 0  # cumulative
        self.last_arrival = 0.0  # monotonic
        self.last_stamp: float | None = None


def import_msg_type(path: str):
    pkg, _, name = path.split("/")
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--interval", type=float, default=2.0, help="report interval seconds")
    ap.add_argument("--csv", type=str, default=None, help="also append rows to this CSV")
    ap.add_argument(
        "--only", action="append", default=None, help="substring filter on stream name; repeatable"
    )
    args = ap.parse_args()

    names = list(TOPICS)
    if args.only:
        names = [n for n in names if any(f in n for f in args.only)]
        if not names:
            raise SystemExit(f"--only {args.only} matched no streams of: {list(TOPICS)}")

    rclpy.init()
    node = rclpy.create_node("dimos_hz_monitor")

    lock = threading.Lock()
    stats: dict[str, TopicStat] = {n: TopicStat() for n in names}

    def make_cb(name: str):
        st = stats[name]

        def cb(raw: bytes) -> None:
            now = time.monotonic()
            with lock:
                st.count += 1
                st.total += 1
                st.bytes += len(raw)
                st.last_arrival = now
                st.last_stamp = stamp_from_cdr(raw)

        return cb

    for name in names:
        topic, type_path = TOPICS[name]
        node.create_subscription(import_msg_type(type_path), topic, make_cb(name), QOS, raw=True)

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "a", buffering=1)
        if csv_file.tell() == 0:
            csv_file.write("t,stream,hz,mibps,gap_s,stale_s,pubs,total\n")

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    t0 = time.monotonic()
    print(f"monitoring {len(names)} topics, interval {args.interval}s — Ctrl-C to stop")
    try:
        while True:
            time.sleep(args.interval)
            now_mono = time.monotonic()
            now_wall = time.time()
            t = now_mono - t0
            rows = []
            with lock:
                for name in names:
                    st = stats[name]
                    hz = st.count / args.interval
                    mibps = st.bytes / args.interval / (1024 * 1024)
                    gap = (now_mono - st.last_arrival) if st.last_arrival else float("inf")
                    stale = (now_wall - st.last_stamp) if st.last_stamp else float("nan")
                    st.count = 0
                    st.bytes = 0
                    rows.append((name, hz, mibps, gap, stale, st.total))

            n_nodes = len(node.get_node_names())
            print(f"\nt=+{t:7.1f}s  visible_nodes={n_nodes}")
            print(
                f"  {'stream':<20} {'hz':>6} {'MiB/s':>7} {'gap(s)':>7} {'stale(s)':>8} {'pubs':>4} {'total':>7}"
            )
            for name, hz, mibps, gap, stale, total in rows:
                pubs = node.count_publishers(TOPICS[name][0])
                flag = "  <-- SILENT" if gap > 2 * args.interval else ""
                print(
                    f"  {name:<20} {hz:>6.1f} {mibps:>7.2f} {gap:>7.1f} {stale:>8.2f} {pubs:>4} {total:>7}{flag}"
                )
                if csv_file:
                    csv_file.write(
                        f"{t:.1f},{name},{hz:.2f},{mibps:.3f},{gap:.2f},{stale:.3f},{pubs},{total}\n"
                    )
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
