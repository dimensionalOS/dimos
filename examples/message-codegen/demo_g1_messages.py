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

"""Publish a synthetic G1 low-state sample as generated feedback without hardware."""

from contextlib import ExitStack
from typing import Any

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.time import to_nanoseconds
from dimos.robot.unitree.g1.wholebody_connection import G1LowStateSnapshot, G1WholeBodyConnection


def main() -> None:
    robot = G1WholeBodyConnection()
    joints: list[Any] = []
    imu: list[Any] = []
    with ExitStack() as stack:
        stack.callback(robot.stop)
        stack.callback(robot.motor_states.subscribe(joints.append))
        stack.callback(robot.imu.subscribe(imu.append))
        for index in range(3):
            header = Header(
                stamp=Time(sec=1700000000, nanosec=123456789 + index), frame_id="g1_pelvis"
            )
            sample = G1LowStateSnapshot(
                positions=[index * 0.1] * 29,
                velocities=[0.2] * 29,
                efforts=[0.3] * 29,
                quaternion=(0.8, 0.0, 0.0, 0.6),
                gyroscope=(1.0, 2.0, 3.0),
                accelerometer=(4.0, 5.0, 6.0),
            )
            robot._publish_motor_state_and_imu(header=header, sample=sample)
        assert len(joints) == len(imu) == 3
        for index, (state, orientation) in enumerate(zip(joints, imu, strict=True)):
            assert state.header == orientation.header
            assert to_nanoseconds(state.header.stamp) == 1700000000123456789 + index
            assert list(state.position) == [index * 0.1] * 29
            assert orientation.orientation.w == 0.8 and orientation.orientation.z == 0.6
            for message in (state, orientation):
                assert type(message).decode(message.encode()) == message
            print(
                f"G1 sample {index}: joints={len(state.name)}, q0={state.position[0]:.1f}, IMU xyzw=(0,0,0.6,0.8), stamp={to_nanoseconds(state.header.stamp)}"
            )
    print(
        "PASS: G1 generated joint/IMU feedback, exact shared stamps, unchanged quaternion convention"
    )


if __name__ == "__main__":
    main()
