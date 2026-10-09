# Copyright 2025-2026 Dimensional Inc.
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

"""Drive a synthetic holonomic robot to a target using generated CDR messages."""

import math

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode

from dimos.navigation.dannav.holonomic_tc.command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)
from dimos.navigation.dannav.holonomic_tc.holonomic_tracking_controller import (
    HolonomicTrackingController,
)
from dimos.navigation.dannav.holonomic_tc.types import (
    TrajectoryMeasuredSample,
    TrajectoryReferenceSample,
)


def main() -> None:
    limits = HolonomicCommandLimits(1.0, 1.0, 2.0, 2.0)
    controller = HolonomicTrackingController(k_position_per_s=2.0, k_yaw_per_s=1.0)
    controller.configure(limits)
    target = Pose(
        position=Point(x=2.0, y=1.0, z=0.0), orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    )
    pose = Pose(
        orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0), position=Point(x=0.0, y=0.0, z=0.0)
    )
    command = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    dt = 0.05
    for tick in range(200):
        reference = TrajectoryReferenceSample(
            tick * dt,
            cdr_decode(cdr_encode(target), Pose),
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
        )
        measured = TrajectoryMeasuredSample(tick * dt, cdr_decode(cdr_encode(pose), Pose), command)
        raw = controller.control(reference, measured)
        command = cdr_decode(cdr_encode(clamp_holonomic_cmd_vel(command, raw, limits, dt)), Twist)
        pose.position.x += command.linear.x * dt
        pose.position.y += command.linear.y * dt
        if tick % 40 == 0:
            print(
                f"t={tick * dt:.1f}s position=({pose.position.x:.3f}, {pose.position.y:.3f}) CDR velocity=({command.linear.x:.3f}, {command.linear.y:.3f})"
            )
    error = math.hypot(target.position.x - pose.position.x, target.position.y - pose.position.y)
    assert error < 0.001, error
    print(f"Reached target (2, 1); position error={error:.8f}m")


if __name__ == "__main__":
    main()
