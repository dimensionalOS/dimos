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

"""Development comparison harness: common sensor facade and checked motion path.

This supplies neither a button detector nor a success checker. A sensor baseline
must provide its own estimate from the observation; evaluator truth is not an input.
"""

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.simulation.behavior.radio_policy import PolicyAction, RadioPolicy, vector


@dataclass(frozen=True)
class PressIntent:
    """Gripper-link contact pose and inward press direction in base_link."""

    position: tuple[float, ...]
    orientation: tuple[float, ...]
    inward: tuple[float, ...]
    provenance: str

    def __post_init__(self) -> None:
        vector(self.position, 3)
        vector(self.orientation, 4)
        vector(self.inward, 3)
        if abs(math.dist(self.orientation, (0, 0, 0, 0)) - 1) > 1e-6:
            raise ValueError("Use a unit gripper quaternion")
        if abs(math.dist(self.inward, (0, 0, 0)) - 1) > 1e-6:
            raise ValueError("Use a unit inward direction")
        if not self.provenance:
            raise ValueError("Declare target provenance")


@dataclass(frozen=True)
class SensorContact:
    """Detector output: RGB pixel and sensor-estimated approach/orientation.

    finger_offset is known robot geometry from gripper-link origin to contact
    point, not an object pose. No object coordinates can enter this interface.
    """

    u: int
    v: int
    orientation: Sequence[float]
    inward: Sequence[float]
    finger_offset: Sequence[float]


def perception_intent(
    policy: RadioPolicy,
    estimate: Callable[[Mapping[str, Any]], SensorContact],
) -> PressIntent:
    """Require a fresh actual observation and ground the detector's selected pixel.

    estimate must use RGB/depth and robot calibration/geometry. There is no
    default detector or normal estimator: missing perception must fail explicitly.
    """
    observation = policy.observe("left_wrist")
    contact = estimate(observation)
    grounded = policy.ground(observation["id"], contact.u, contact.v)
    if (
        grounded["frame"] != "base_link"
        or grounded["observation_id"] != observation["id"]
        or grounded["pixel"] != [contact.u, contact.v]
    ):
        raise ValueError("Grounded point does not match the selected sensor observation")
    orientation = vector(contact.orientation, 4)
    offset = vector(contact.finger_offset, 3)
    xyz = np.asarray(vector(grounded["position"], 3)) - Rotation.from_quat(orientation).apply(
        offset
    )
    return PressIntent(
        tuple(float(x) for x in xyz),
        orientation,
        vector(contact.inward, 3),
        f"sensor observation {observation['id']}, pixel ({contact.u}, {contact.v}); robot finger geometry",
    )


class PressBaseline:
    """Pollable common runner for a locked intent or a perception-produced intent.

    No sleeps or background worker: tick dispatches at most one action and polls
    the owned ID. Failed/cancelled/uncertain actions halt the sequence. A caller
    abandoning the run must call cancel(); supervisor owns motion timeouts.
    Completion means SDK stages completed, never BDDL success.
    """

    def __init__(self, policy: RadioPolicy, intent: PressIntent) -> None:
        self.policy, self.intent = policy, intent
        pose = policy.pose()
        if pose.frame_id != "base_link":
            raise ValueError("Baseline requires base-frame robot feedback")
        self._precontact = np.asarray(intent.position) - 0.012 * np.asarray(intent.inward)
        initial = pose.position.to_tuple()
        self._stages = (
            ("lift", (initial[0], initial[1], 0.8), pose.orientation.to_tuple()),
            (
                "cross",
                (float(self._precontact[0]), float(self._precontact[1]), 0.75),
                intent.orientation,
            ),
            ("precontact", tuple(float(x) for x in self._precontact), intent.orientation),
            ("press", tuple(0.012 * x for x in intent.inward), None),
        )
        self._index = 0
        self._action: PolicyAction | None = None
        self.halted: PolicyAction | None = None
        self.evidence: list[dict[str, Any]] = []

    @property
    def completed(self) -> bool:
        return self._index == len(self._stages) and self.halted is None

    def tick(self) -> PolicyAction | None:
        if self.halted is not None:
            return self.halted
        if self.completed:
            return None
        if self._action is not None:
            result = self.policy.status(self._action.id)
            if result.state == "running":
                return result
            self.evidence[-1]["result"] = result
            if result.state != "completed":
                self.halted = result
                return result
            self._index += 1
            self._action = None
            # Keep a completed-stage result observable before dispatching another.
            return result
        label, position, orientation = self._stages[self._index]
        result = (
            self.policy.press(position, timeout=10)
            if orientation is None
            else self.policy.move_pose(position, orientation, timeout=20)
        )
        self.evidence.append(
            {"stage": label, "action_id": result.id, "provenance": self.intent.provenance}
        )
        self._action = result
        if result.state in ("failed", "cancelled", "uncertain"):
            self.halted = result
        return result

    def cancel(self) -> PolicyAction | None:
        if self._action is None or self.completed:
            return None
        result = self.policy.cancel(self._action.id)
        # Even a pending cancellation halts this runner. No next stage is sent.
        self.halted = result
        self.evidence[-1]["cancellation"] = result
        return result
