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

"""Exercise the simulation lidar IPC boundary without running MuJoCo or a robot."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.simulation.mujoco.shared_memory import ShmReader, ShmWriter

STAMP = 1_700_000_000_123_456_789


def produce(names: dict[str, str]) -> None:
    producer = ShmReader(names)
    try:
        points = np.array([[1.25, -2.5, 3.75], [4, 5, 6]], dtype=np.float32)
        message = pointcloud_from_xyz(
            points, header=Header(stamp=time_from_nanoseconds(STAMP), frame_id="world")
        )
        producer.write_lidar(message)
    finally:
        producer.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--producer-names", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.producer_names:
        produce(json.loads(args.producer_names))
        return

    consumer = ShmWriter()
    try:
        subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--producer-names",
                json.dumps(consumer.shm.to_names()),
            ],
            check=True,
            timeout=30,
        )
        message, sequence = consumer.read_lidar()
        assert message is not None
        assert sequence > 0
        assert message.header.frame_id == "world"
        assert to_nanoseconds(message.header.stamp) == STAMP
        coordinates = pointcloud_xyz(message).tolist()
        assert coordinates == [[1.25, -2.5, 3.75], [4, 5, 6]]
        print(f"Separate process → shared memory → generated CDR decoder: {coordinates}")
        print(f"frame={message.header.frame_id}, stamp={STAMP}, sequence={sequence}")
        print("PASS: no ROS installation, MuJoCo engine, or robot required")
    finally:
        consumer.cleanup()


if __name__ == "__main__":
    main()
