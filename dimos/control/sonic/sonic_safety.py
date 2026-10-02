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

"""Shared SONIC fault response for the policy task and final G1 publisher."""

from collections.abc import Sequence
import math

from dimos.hardware.whole_body.spec import MotorCommand

# Match the original SONIC damping gains and overspeed threshold, checking
# both directions. These are deployment defaults, not certified safety limits.
DAMPING_KD = 8.0
JOINT_VELOCITY_LIMIT = 35.0
FEEDBACK_TIMEOUT_SECONDS = 0.1
COMMAND_TIMEOUT_SECONDS = 0.1
PLANNER_TIMEOUT_SECONDS = 1.0


class SonicSafetyError(RuntimeError):
    """A control failure requiring a latched damping stop."""


def check_joint_velocities(velocities: Sequence[float], limit: float) -> None:
    for index, velocity in enumerate(velocities):
        if not math.isfinite(velocity):
            raise SonicSafetyError(f"non-finite joint velocity: motor {index}")
        if abs(velocity) > limit:
            raise SonicSafetyError(
                f"joint overspeed: motor {index}, {velocity:.3f} rad/s, limit {limit:g}"
            )


def damping_commands(count: int) -> list[MotorCommand]:
    """Zero stiffness/feedforward, zero desired velocity, positive damping."""
    return [MotorCommand(q=0.0, dq=0.0, kp=0.0, kd=DAMPING_KD, tau=0.0) for _ in range(count)]
