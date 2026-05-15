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

"""Robot ↔ LeRobot frame contract.

A `RobotContract` is the per-robot adapter that converts:

- An rrd row produced by `RerunDataRecorder` → a LeRobot frame dict
  (used offline by the rrd → LeRobot converter).
- Live `Image` + `JointState` messages → a LeRobot frame dict
  (used online by `PolicyModule`, in a follow-up change).
- A backend-emitted action vector → a coordinator-native `JointState`
  (used online by `PolicyModule`, in a follow-up change).

Concrete implementations live alongside under
`dimos.manipulation.policy.contracts.<robot>` and are looked up by name
via `dimos.manipulation.policy.contracts.registry`.

This module MUST NOT import `lerobot`. The contract describes the LeRobot
dataset feature schema using plain dict/dataclass types so it can be
imported in environments where the `datasets` extra is not installed.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, TypedDict, runtime_checkable

if TYPE_CHECKING:
    import numpy as np

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass
class GripperBinarization:
    """Configuration for converting a continuous gripper position into the
    binary {open, closed} action expected by most LeRobot policies.

    Normalization: `norm = (raw - closed_pos) / (open_pos - closed_pos)`.
    Binarization: `1.0 if norm > threshold else 0.0` (strictly greater —
    a normalized value equal to the threshold is treated as closed).

    Applied to the action slot only by `PiperRobotContract.from_rerun_row`
    and `from_messages`. The state slot is rescaled to fraction-open in
    `[0, 1]` for unit consistency but never thresholded.
    """

    open_pos: float
    closed_pos: float
    enabled: bool = True
    threshold: float = 0.7


class LeRobotFrame(TypedDict, total=False):
    """Shape returned by `RobotContract.from_rerun_row` and `from_messages`.

    Image keys are not enumerable in a TypedDict; they appear under
    `observation.images.<cam>` for each camera the contract declares.
    `total=False` makes every key optional so this same TypedDict can describe
    both training frames (with `action`) and inference frames (without).
    """

    task: str
    # `observation.state` and `action` are declared via the dotted-key
    # pattern below as well, since TypedDict does not accept dots in keys
    # syntactically. Consumers should expect:
    #   "observation.state": np.ndarray  (float32, shape (state_dim,))
    #   "action":            np.ndarray  (float32, shape (action_dim,))
    #   "observation.images.<cam>": np.ndarray  (uint8, shape (H, W, C))


@runtime_checkable
class RobotContract(Protocol):
    """Per-robot adapter between DimOS messages, recorded rrd rows, and a
    LeRobot frame dict.

    Implementations are structural — concrete classes do not need to inherit
    from this Protocol. `isinstance(obj, RobotContract)` works at runtime via
    `@runtime_checkable`.
    """

    cameras: Mapping[str, tuple[int, int, int]]
    """Camera key → ``(height, width, channels)``. The keys appear in the LeRobot
    frame dict as ``observation.images.<key>``."""

    state_joint_names: Sequence[str]
    """Ordered short joint names whose values populate `observation.state`."""

    action_joint_names: Sequence[str]
    """Ordered short joint names whose values populate `action`."""

    gripper_joint: str | None
    """Short name of the gripper joint, or `None` for no-gripper robots.
    When present, must appear in both `state_joint_names` and `action_joint_names`."""

    gripper_binarization: GripperBinarization
    """Gripper rescaling / binarization configuration."""

    def features(self) -> dict[str, dict[str, Any]]:
        """LeRobot dataset feature schema (`observation.images.<cam>`,
        `observation.state`, `action`).

        Returned as plain dicts (no `lerobot` imports) so it is callable in
        environments without the `datasets` extra installed.
        """
        ...

    def rerun_entities(self) -> Sequence[str]:
        """Entity paths to select from a `.rrd` recording — what
        `from_rerun_row` reads."""
        ...

    def from_rerun_row(self, row: Any) -> dict[str, Any]:
        """Translate one Arrow row produced by `rerun.dataframe.view().select()`
        into a LeRobot frame dict. Used offline by the converter."""
        ...

    def from_messages(
        self,
        images: Mapping[str, Image],
        state: JointState,
        *,
        action: JointState | None = None,
        task: str = "",
    ) -> dict[str, Any]:
        """Translate live messages into a LeRobot frame dict matching the
        shape of `from_rerun_row`. Used online by `PolicyModule`.

        When `action is None` (inference), the returned dict omits the
        `"action"` key. Raises `KeyError` naming any missing camera or joint.
        """
        ...

    def to_command(self, action_vec: np.ndarray) -> JointState:
        """Translate a backend action vector into a coordinator-native
        `JointState`. The gripper slot of the action vector — if binarization
        is enabled — is mapped from `{0.0, 1.0}` back to
        `{closed_pos, open_pos}`. Used online by `PolicyModule`."""
        ...


__all__ = [
    "GripperBinarization",
    "LeRobotFrame",
    "RobotContract",
]
