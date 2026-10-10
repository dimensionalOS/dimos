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

from custom_msgs.msg import Reading as RosReading
from rclpy.serialization import deserialize_message, serialize_message


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    evidence = build / "evidence"
    for filename, value in (("input.cdr", 20.5), ("cpp.cdr", 21.5), ("rust.cdr", 22.5)):
        reference = RosReading(value=value)
        reference.header.frame_id = "map"
        reference.header.stamp.sec = 17
        reference.header.stamp.nanosec = 123456789
        decoded = deserialize_message((evidence / filename).read_bytes(), RosReading)
        assert decoded == reference, filename
        print(f"ROS2 Jazzy decoded {filename}: all Header and value fields match")
    (evidence / "ros-jazzy.cdr").write_bytes(serialize_message(reference))
    print("ROS2 Jazzy independently decoded the supported three-language ownership fixture.")
    print("Bounded Telemetry and declared-default conformance remain documented limitations.")


if __name__ == "__main__":
    main()
