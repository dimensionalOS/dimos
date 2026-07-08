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
here at load time. The coordinator does not consume these yet; a follow-up PR
routes its input streams through them.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType


class Routing(str, Enum):
    """How the coordinator matches an input message to a consuming task."""

    CLAIM_OVERLAP = "claim_overlap"  # deliver when msg names joints the task claims
    BY_TASK_NAME = "by_task_name"  # deliver when msg.frame_id == task.name
    BROADCAST = "broadcast"  # deliver to every task consuming this stream


CONSUMABLE_STREAMS = frozenset({"joint_command", "coordinator_cartesian_command", "teleop_buttons"})
"""Coordinator input ports that cards may bind."""

DEFERRED_STREAMS = frozenset({"twist_command"})
"""Coordinator input ports that exist but are not card-routable yet."""


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


_EMPTY_EXPOSES: Mapping[str, str] = MappingProxyType({})


@dataclass(frozen=True)
class TaskBindings:
    """Declared input streams and commands for one task type.

    ``exposes`` maps a command name to a ``"module:PydanticModel"``
    argument-schema path; it is stored but not consumed yet.
    """

    consumes: tuple[StreamBinding, ...] = ()
    exposes: Mapping[str, str] = _EMPTY_EXPOSES
