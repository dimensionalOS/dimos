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

"""Absolute dual-hand goals and the thin WebXR-to-goal mapping.

Goals are expressed for the two station-frame TCP sites, which is exactly what
the trained policy observes. An operator-facing WebXR stream is mapped by
anchoring the controller poses at engagement, so a relative hand motion becomes
an absolute station-frame goal. The mapper holds no session authority of its
own: releasing the deadman or a fault drops the anchor, and the caller decides
when a fresh engagement is allowed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Self

import numpy as np
from numpy.typing import NDArray

from dimos.robot.manipulators.dual_makermods_metal.policy.contract import SIDES


class GoalError(ValueError):
    """Raised for malformed hand goals."""


def quaternion_xyzw_to_matrix(quaternion: tuple[float, float, float, float]) -> NDArray[np.float64]:
    """Unit quaternion ``(x, y, z, w)`` to a rotation matrix."""
    x, y, z, w = (float(v) for v in quaternion)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if not np.isfinite(norm) or norm < 1e-8:
        raise GoalError("quaternion must be finite and non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def check_rotation(rotation: NDArray[np.float64], *, atol: float = 1e-5) -> NDArray[np.float64]:
    """Reject a matrix that is not a proper rotation."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise GoalError(f"rotation must be 3x3, got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise GoalError("rotation contains non-finite values")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=atol):
        raise GoalError("rotation is not orthonormal")
    if abs(float(np.linalg.det(matrix)) - 1.0) > atol:
        raise GoalError("rotation determinant must be +1")
    return matrix


@dataclass(frozen=True)
class RigidSample:
    """A validated station-frame pose: finite position and a proper rotation."""

    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]

    def __post_init__(self) -> None:
        position = tuple(float(v) for v in self.position)
        if len(position) != 3 or not np.isfinite(position).all():
            raise GoalError("position must be a finite 3-vector")
        check_rotation(np.asarray(self.rotation, dtype=np.float64))

    @property
    def matrix(self) -> NDArray[np.float64]:
        return np.asarray(self.rotation, dtype=np.float64)

    @classmethod
    def from_quaternion(
        cls,
        position: tuple[float, float, float] | NDArray[np.float64],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> Self:
        matrix = quaternion_xyzw_to_matrix(quaternion_xyzw)
        xyz = np.asarray(position, dtype=np.float64).reshape(3)
        return cls((float(xyz[0]), float(xyz[1]), float(xyz[2])), tuple(map(tuple, matrix)))


class HandGoal(RigidSample):
    """One TCP site goal in the station frame."""


class ControllerPose(RigidSample):
    """One WebXR controller pose in the station frame at the current instant."""

    @classmethod
    def from_pose(
        cls,
        position: tuple[float, float, float] | NDArray[np.float64],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> Self:
        return cls.from_quaternion(position, quaternion_xyzw)


@dataclass(frozen=True)
class DualHandGoal:
    """Absolute goals for both station TCP sites."""

    left: HandGoal
    right: HandGoal
    frame: str = "station"

    def to_pose24(self) -> NDArray[np.float64]:
        """Flatten to the trained layout ``[left_xyz, right_xyz, left_R, right_R]``."""
        out = np.empty(24, dtype=np.float64)
        for index, hand in enumerate((self.left, self.right)):
            out[index * 3 : index * 3 + 3] = np.asarray(hand.position, dtype=np.float64)
        for index, hand in enumerate((self.left, self.right)):
            start = 6 + index * 9
            out[start : start + 9] = hand.matrix.reshape(-1)
        return out

    @classmethod
    def from_pose24(cls, pose24: NDArray[np.float64] | NDArray[np.float32]) -> DualHandGoal:
        values = np.asarray(pose24, dtype=np.float64).reshape(-1)
        if values.shape != (24,):
            raise GoalError(f"goal pose must have 24 values, got {values.shape}")
        if not np.isfinite(values).all():
            raise GoalError("goal pose contains non-finite values")
        hands = []
        for index in range(2):
            position = values[index * 3 : index * 3 + 3]
            rotation = values[6 + index * 9 : 6 + (index + 1) * 9].reshape(3, 3)
            hands.append(
                HandGoal(tuple(position.tolist()), tuple(map(tuple, check_rotation(rotation))))
            )
        return cls(hands[0], hands[1])

    def offset(self, side: str, delta_xyz: tuple[float, float, float]) -> DualHandGoal:
        """Translate one hand's goal, keeping orientation."""
        if side not in SIDES:
            raise GoalError(f"unknown side {side!r}")
        hand = self.left if side == "left" else self.right
        moved = np.asarray(hand.position, dtype=np.float64) + np.asarray(
            delta_xyz, dtype=np.float64
        )
        return DualHandGoal(
            self.left if side == "right" else HandGoal(tuple(moved.tolist()), hand.rotation),
            self.right if side == "left" else HandGoal(tuple(moved.tolist()), hand.rotation),
            frame=self.frame,
        )


class WebXRGoalMapper:
    """Map relative controller motion onto absolute hand goals around an anchor."""

    def __init__(self) -> None:
        self._anchor_goal: DualHandGoal | None = None
        self._anchor_controllers: dict[str, ControllerPose] = {}
        self._engaged = False

    @property
    def engaged(self) -> bool:
        return self._engaged

    def engage(
        self,
        current: DualHandGoal,
        left_controller: ControllerPose,
        right_controller: ControllerPose,
    ) -> None:
        """Anchor the relative mapping at the robot's current TCP goal."""
        self._anchor_goal = current
        self._anchor_controllers = {"left": left_controller, "right": right_controller}
        self._engaged = True

    def release(self) -> None:
        """Drop the anchor; a later engagement needs freshly supplied poses."""
        self._anchor_goal = None
        self._anchor_controllers = {}
        self._engaged = False

    def update(
        self, left_controller: ControllerPose, right_controller: ControllerPose
    ) -> DualHandGoal:
        """Return the absolute goal implied by the controllers' motion since engagement."""
        if not self._engaged or self._anchor_goal is None:
            raise GoalError("WebXR goal mapping is not engaged")
        goals: list[HandGoal] = []
        for side, controller in (("left", left_controller), ("right", right_controller)):
            anchor_controller = self._anchor_controllers[side]
            anchor_hand = self._anchor_goal.left if side == "left" else self._anchor_goal.right
            delta_position = np.asarray(controller.position, dtype=np.float64) - np.asarray(
                anchor_controller.position, dtype=np.float64
            )
            rotation = controller.matrix @ anchor_controller.matrix.T
            position = np.asarray(anchor_hand.position, dtype=np.float64) + delta_position
            goal_rotation = check_rotation(rotation @ anchor_hand.matrix)
            goals.append(HandGoal(tuple(position.tolist()), tuple(map(tuple, goal_rotation))))
        return DualHandGoal(goals[0], goals[1], frame=self._anchor_goal.frame)
