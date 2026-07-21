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

"""Declarative stream-binding cards for control tasks.

Task manifests (``_registry.py``) stay import-free: they declare bindings as
plain strings, and ``ControlTaskRegistry`` converts them to the typed forms
here at load time. ``ControlCoordinator`` builds its stream route table from
them when tasks are registered.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Routing(str, Enum):
    """How the coordinator matches an input message to a consuming task."""

    CLAIM_OVERLAP = "claim_overlap"  # deliver when msg names joints the task claims
    BY_TASK_NAME = "by_task_name"  # deliver when msg.frame_id == task.name
    BROADCAST = "broadcast"  # deliver to every task consuming this stream


CONSUMABLE_STREAMS = frozenset(
    {
        "joint_command",
        "coordinator_cartesian_command",
        "coordinator_ee_twist_command",
        "gripper_command",
        "twist_command",
        "teleop_buttons",
    }
)
"""Coordinator input ports that cards may bind."""


@dataclass(frozen=True)
class StreamBinding:
    """One input stream a task consumes: coordinator port -> task handler.

    ``handler`` names a method on the task instance with signature
    ``(msg, t_now) -> Any``; the task receives the raw message and owns
    digestion, the coordinator owns only routing.
    """

    stream: str
    handler: str
    routing: Routing


@dataclass(frozen=True)
class TaskBindings:
    """Declared input streams and commands for one task type.

    ``exposes`` is the set of command names the task accepts via
    ``ControlCoordinator.task_invoke``; the task method's own signature
    is the argument schema (validated at dispatch, not here).
    """

    consumes: tuple[StreamBinding, ...] = ()
    exposes: frozenset[str] = frozenset()
