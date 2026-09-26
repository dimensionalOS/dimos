# Copyright 2025-2026 Dimensional Inc.
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

"""A robot driver, as the rest of the system sees it: two ports and a set of
calls.

Readings go out on ``control_state`` and commands come in on
``control_command``. Every driver on a robot shares the same two ports, and
each message names the piece of hardware it is about.

The calls take a robot from connected, through getting ready, to obeying
commands, and stop it. Each takes an optional ``source`` naming one piece of
hardware, and returns one answer per piece acted on; leave ``source`` out to
act on all of them. ``ConnectionRpcMixin`` implements all of this.
"""

from typing import Protocol

from dimos.control.connection.status import ConnectionStatus, LifecycleAck
from dimos.control.contract.description import ControlDescription
from dimos.core.stream import In, Out
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.spec.utils import Spec


class ControlSource(Protocol):
    """A robot driver's two ports."""

    control_state: Out[ControlValues]
    control_command: In[ControlValues]


class ConnectionControlSpec(Spec, Protocol):
    """The calls a robot driver answers."""

    def describe_control(self) -> list[ControlDescription]: ...
    def status(self) -> list[ConnectionStatus]: ...
    def prepare_arm(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]: ...
    def commit_arm(
        self, operation_id: int, epoch: int, source: str | None = None
    ) -> list[LifecycleAck]: ...
    def abort_arm(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]: ...
    def safe_stop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]: ...
    def clear_safe_stop(
        self, operation_id: int, source: str | None = None
    ) -> list[LifecycleAck]: ...
    def estop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]: ...
    def clear_estop(self, operation_id: int, source: str | None = None) -> list[LifecycleAck]: ...
