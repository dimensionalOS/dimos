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

"""Owner-side bridge to tested checkpoint contracts; never a target generator.

Privileged collision/retention diagnostics remain development supervisor inputs.
Caller-supplied poses and contact intent are not corrected toward task geometry.
"""

from collections.abc import Callable, Mapping, Sequence
from typing import Any

import numpy as np

from dimos.manipulation.manipulation_spec import ExecutionResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.behavior.radio_checkpoint import RadioGraspCheckpoint, rigid

SINGLE_HAND_PHASES = {
    "pregrasp",
    "grasp",
    "departure",
    "lift",
    "reorient",
    "lower",
    "place",
    "retract",
    "table_approach",
    "table_press",
}

# Measured robot tool geometry for the proven 0.08 gripper opening. Not a radio
# annotation or a task-success coordinate. Other tool geometry needs validation.
RIGHT_FINGER2_PAD = (0.0003888239844, 0.0049937094101, -0.0780771998475)


class RadioCheckpointPolicyMotion:
    def __init__(self, checkpoint: RadioGraspCheckpoint, auxiliary_groups: Sequence[str]) -> None:
        if tuple(auxiliary_groups) not in ((), ("torso",)):
            raise ValueError("Checkpoint policy permits only explicit torso assistance")
        self.checkpoint = checkpoint
        self.auxiliary_torso = bool(auxiliary_groups)

    def move_intent(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        timeout: float,
        intent: Mapping[str, Any],
        *,
        dispatch: Callable[[str, float], ExecutionResult],
        cancelled: Callable[[], bool],
    ) -> None:
        phase = intent["phase"]
        if phase not in SINGLE_HAND_PHASES:
            raise ValueError("Choose a single-hand checkpoint phase")
        contact = None
        if phase == "table_press":
            # The world point/normal come from the caller, never the hidden marker.
            observation = self.checkpoint._observation(phase)
            opening = observation["measured_gripper"]
            if opening is None or not np.isfinite(opening) or abs(opening - 0.08) > 0.005:
                raise RuntimeError("Validated press tool requires measured 0.08 gripper opening")
            radio = rigid(observation["radio_pose"])
            surface = np.asarray(intent["surface_world"], dtype=float)
            normal = np.asarray(intent["normal_world"], dtype=float)
            if (
                surface.shape != (3,)
                or normal.shape != (3,)
                or not np.isfinite(surface).all()
                or not np.isfinite(normal).all()
                or not np.isclose(np.linalg.norm(normal), 1, atol=1e-6)
            ):
                raise ValueError("Declare finite sensor-derived contact point and unit normal")
            contact = {
                "source": "caller_sensor_intent",
                "surface_in_radio": (np.linalg.inv(radio) @ np.append(surface, 1))[:3].tolist(),
                "outward_normal_in_radio": (radio[:3, :3].T @ normal).tolist(),
                "pad_in_gripper": list(RIGHT_FINGER2_PAD),
            }
        self.checkpoint.move(
            PoseStamped(frame_id="world", position=position, orientation=orientation),
            phase,
            timeout=timeout,
            auxiliary_torso=self.auxiliary_torso,
            contact=contact,
            dispatch=dispatch,
            cancelled=cancelled,
        )
