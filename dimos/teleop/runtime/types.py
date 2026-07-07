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

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Literal, Protocol

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState

TeleopPrimaryOutput = Literal["joint", "cartesian", "twist"]


@dataclass(frozen=True)
class TeleopCommandMetadata:
    """Metadata attached to a teleop command."""

    primary_output: TeleopPrimaryOutput
    timestamp: float = field(default_factory=time.monotonic)


@dataclass(frozen=True)
class TeleopCommand:
    """Command envelope containing an active command or explicit stop."""

    metadata: TeleopCommandMetadata
    joint: JointState | None = None
    cartesian: PoseStamped | None = None
    twist: Twist | None = None
    stop: bool = False

    def __post_init__(self) -> None:
        active_outputs = [
            output
            for output, payload in (
                ("joint", self.joint),
                ("cartesian", self.cartesian),
                ("twist", self.twist),
            )
            if payload is not None
        ]
        if self.stop:
            if len(active_outputs) > 1:
                raise ValueError("TeleopCommand stop envelope must not contain multiple payloads")
            if active_outputs and active_outputs[0] != self.metadata.primary_output:
                raise ValueError("TeleopCommand payload must match metadata.primary_output")
            return
        if len(active_outputs) != 1:
            raise ValueError("TeleopCommand must contain exactly one primary command payload")
        if active_outputs[0] != self.metadata.primary_output:
            raise ValueError("TeleopCommand payload must match metadata.primary_output")


class TeleopAdapter(Protocol):
    """Adapter interface for generic teleop command sources."""

    primary_output: TeleopPrimaryOutput

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_current_command(self) -> TeleopCommand | None: ...
