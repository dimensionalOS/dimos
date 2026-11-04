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

"""Engine-independent dynamics. Reads return detached records for one acquisition.

SimModule calls every operation on its physics thread, between native steps.
"""

from typing import Protocol

from dimos_lcm.control_msgs import CborCommand, JointTarget
from dimos_lcm.sim_msgs import BodyMotion, BodyPose, CborExtension, JointTruth

from dimos.sim2.description import SceneConfig


class PhysicsBackend(Protocol):
    body_ids: tuple[str, ...]
    joint_ids: tuple[str, ...]
    driven_joint_ids: tuple[str, ...]
    root_ids: tuple[str, ...]

    def load(self, scene: SceneConfig, dt_ns: int) -> None: ...
    def read_body_poses(self) -> list[BodyPose]: ...
    def read_joints(self, joint_ids: tuple[str, ...]) -> JointTruth: ...
    def read_extensions(self) -> list[CborExtension]: ...
    def read_motion(self, frame_id: str) -> BodyMotion: ...
    def apply_action(
        self, joint_targets: list[JointTarget], cbor_commands: list[CborCommand]
    ) -> None:
        """Validate the whole action, then apply it; raise ValueError without mutation on rejection."""
        ...

    def hold(self, joint_ids: tuple[str, ...]) -> None: ...
    def step(self) -> None: ...
    def reset(self) -> None: ...
    def place(self, body_id: str, pose: tuple[float, ...]) -> None: ...
    def close(self) -> None: ...
