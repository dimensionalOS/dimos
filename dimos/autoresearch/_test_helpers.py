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

"""A robot backend for tests, with no stack behind it.

Not named ``test_*`` so pytest does not collect it; the program subprocess
imports ``fake_backend`` from here by its dotted path.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from dimos.autoresearch.program import Pose, RobotBackend, RobotCallError, SceneObject

# A one-pixel PNG, written wherever a camera frame would go.
_BLANK_PNG = bytes.fromhex(
    "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
    "0000000d49444154789c6360000002000154a24f5d0000000049454e44ae426082"
)


class FakeBackend:
    """Moves only update bookkeeping, every word finds one object and frames
    are blank PNGs. A ``move_pose`` below the table (z < 0) fails planning,
    so a program's handling of a failed call can be tested."""

    _joints: list[float]
    _pose: Pose

    def __init__(self) -> None:
        self.gripper = 0.0
        self.home()

    def joints(self) -> list[float]:
        return list(self._joints)

    def pose(self) -> Pose:
        return self._pose

    def move_joints(self, positions: Sequence[float]) -> None:
        self._joints = [float(value) for value in positions]

    def move_pose(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None:
        if position[2] < 0.0:
            raise RobotCallError("PLANNING_FAILED", f"no plan: {position} is below the table")
        self._pose = Pose(position, self._pose.orientation if orientation is None else orientation)

    def move_linear(self, dx: float, dy: float, dz: float) -> None:
        x, y, z = self._pose.position
        self._pose = Pose((x + dx, y + dy, z + dz), self._pose.orientation)

    def home(self) -> None:
        self._joints = [0.0] * 6
        self._pose = Pose((0.3, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))

    def open_gripper(self) -> None:
        self.gripper = 0.0

    def close_gripper(self) -> None:
        self.gripper = -0.05

    def set_gripper(self, joint_value: float) -> None:
        self.gripper = joint_value

    def gripper_range(self) -> tuple[float, float]:
        return (-0.05, 0.0)

    def objects(self, words: Sequence[str]) -> list[SceneObject]:
        return [
            SceneObject(
                id=str(index), name=word, position=(0.4, 0.1 * index, 0.02), size=(0.04,) * 3
            )
            for index, word in enumerate(words)
        ]

    def capture(self, path: Path) -> None:
        path.write_bytes(_BLANK_PNG)


def fake_backend(trial_dir: Path) -> RobotBackend:
    """The backend factory tests name in ``run_program``."""
    return FakeBackend()
