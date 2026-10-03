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

"""CI-only conformance with ROS2 Jazzy's independently generated types and RMW."""

import argparse
from pathlib import Path

from demo_msgs.msg import Telemetry as RosTelemetry
from rclpy.serialization import deserialize_message, serialize_message


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    evidence = build / "evidence"
    for filename in ("defaults-python.cdr", "defaults-C++.cdr", "defaults-Rust.cdr"):
        assert (
            deserialize_message((evidence / filename).read_bytes(), RosTelemetry) == RosTelemetry()
        ), filename
    reference = RosTelemetry(
        sequence=4294967295, payload=list(range(256)), hops=[-2147483648, 2147483647]
    )
    reference.header.frame_id = "map"
    reference.header.stamp.sec = -1
    reference.header.stamp.nanosec = 999999999
    reference.label = "café"
    reference.position.x = -1.25
    expected = serialize_message(reference)
    for filename in ("python-le.cdr", "python-be.cdr", "C++.cdr", "Rust.cdr"):
        decoded = deserialize_message((evidence / filename).read_bytes(), RosTelemetry)
        # RMW may leave alignment padding unspecified. Compare every field;
        # byte-canonical comparisons use the independent rosbags oracle.
        assert decoded == reference, filename
        print(f"ROS2 Jazzy decoded {filename}: every field matches")
    (evidence / "ros-jazzy.cdr").write_bytes(expected)
    print(
        "ROS2 Jazzy confirms declared defaults, nested fields, bounds, arrays, Unicode, and both byte orders."
    )


if __name__ == "__main__":
    main()
