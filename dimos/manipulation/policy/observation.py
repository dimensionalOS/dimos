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

"""Canonical policy observation model.

`PolicyObservation` is the unified input that `PolicyModule` assembles from
its typed streams (multi-camera images, joint state, task description) and
hands to the configured backend. Backends translate this into
framework-specific tensors inside their own boundaries.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class PolicyObservation:
    """Unified observation a backend receives from `PolicyModule`.

    Attributes:
        images: Camera images keyed by the configured observation camera
            name (e.g., ``"main"``, ``"wrist"``). Empty when no camera input
            has produced a value yet.
        joint_state: Latest robot joint state, or ``None`` if no state has
            been seen yet.
        task: Plain-text task description. Defaults to the empty string when
            no task input has been seen and no default is configured.
        timestamp: Wall-clock time the observation was assembled. Used for
            staleness checks and downstream logging.
        extras: Optional backend-specific extras — left empty by the node
            and reserved for callers that pre-build observations in tests.
    """

    images: Mapping[str, Image] = field(default_factory=dict)
    joint_state: JointState | None = None
    task: str = ""
    timestamp: float = 0.0
    extras: Mapping[str, Any] = field(default_factory=dict)


__all__ = ["PolicyObservation"]
