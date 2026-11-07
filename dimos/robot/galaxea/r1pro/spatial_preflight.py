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

"""Read-only hardware readiness check used by bin/run-r1pro-spatial."""

import importlib
import math
import time
from typing import Any


def main() -> None:
    rclpy = importlib.import_module("rclpy")
    msgs = importlib.import_module("sensor_msgs.msg")
    qos = importlib.import_module("rclpy.qos").qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node("r1_spatial_preflight")
    received: dict[str, float] = {}
    required = {"torso": 4, "arm_left": 7, "arm_right": 7, "gripper_left": 1, "gripper_right": 1}

    def record(msg: Any, part: str) -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (
            len(msg.position) >= required[part]
            and all(math.isfinite(q) for q in msg.position)
            and abs(time.time() - ts) < 0.5
        ):
            received[part] = time.monotonic()

    try:
        for part in required:
            node.create_subscription(
                msgs.JointState,
                "/hdas/feedback_" + part,
                lambda msg, part=part: record(msg, part),
                qos,
            )
        # Allow DDS discovery to complete before checking for competing command publishers.
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = [part for part in required if time.monotonic() - received.get(part, 0) > 0.5]
        if missing:
            raise SystemExit(
                "Missing fresh robot feedback: "
                + ", ".join(missing)
                + ". Start/check the vendor stack first."
            )
        topics = (
            [
                f"/motion_target/target_joint_state_{part}"
                for part in ("torso", "arm_left", "arm_right")
            ]
            + [f"/motion_target/target_position_gripper_{s}" for s in ("left", "right")]
            + ["/motion_target/target_speed_chassis"]
        )
        conflicts = {
            topic: [p.node_name for p in node.get_publishers_info_by_topic(topic)]
            for topic in topics
        }
        conflicts = {topic: names for topic, names in conflicts.items() if names}
        if conflicts:
            raise SystemExit(
                f"Another controller is publishing robot commands: {conflicts}. Stop that controller first."
            )
        print("R1 Pro ready: fresh torso/arm/gripper feedback; no competing command publishers.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
