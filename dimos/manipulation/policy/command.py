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

"""Tagged policy command model.

A `PolicyBackend` returns a `PolicyCommand` from `select_action`. The
policy node validates the command against its configuration before
translating it into a coordinator-native message (e.g., `JointState`).

The first reference command family is joint position output. The model
also supports an explicit no-op so backends can decline to act on a step
without raising.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class JointPositionCommand:
    """Joint position targets, named so the node can validate joint mapping
    before publishing."""

    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    kind: Literal["joint_position"] = "joint_position"

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(self.positions):
            raise ValueError(
                f"JointPositionCommand: joint_names ({len(self.joint_names)}) "
                f"and positions ({len(self.positions)}) length mismatch"
            )


@dataclass(frozen=True)
class NoOpCommand:
    """Explicit no-op — the backend has nothing to publish this step.

    `reason` is informational only and surfaced in logs.
    """

    reason: str = ""
    kind: Literal["noop"] = "noop"


PolicyCommand = JointPositionCommand | NoOpCommand
"""Tagged union of supported policy command families."""


__all__ = [
    "JointPositionCommand",
    "NoOpCommand",
    "PolicyCommand",
]
