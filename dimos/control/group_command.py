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

"""Local, whole-group command contract (no transport or device semantics)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from dimos.control.contract.convert import JOINT_STATE_FIELDS
from dimos.control.contract.keys import Interface


@dataclass(frozen=True, slots=True)
class GroupCommandInterface:
    """One named interface local to a connection, covering its model joints.

    Declare either an ordered joint payload layout or a coordinate frame and
    TCP. Joint names here are local to the model; tasks use qualified names
    (``source/interface``) to address the group.
    """

    name: str
    payload_type: type
    joints: frozenset[str]
    joint_order: tuple[str, ...] = ()
    frame: str | None = None
    tcp: str | None = None
    feedback_fields: frozenset[Interface] = frozenset()

    def __post_init__(self) -> None:
        if not self.name.strip() or "/" in self.name:
            raise ValueError("group interface name must be a nonempty local name")
        if not isinstance(self.payload_type, type):
            raise ValueError("group interface needs a concrete payload type")
        if not self.joints or any(not name.strip() for name in self.joints):
            raise ValueError("group interface needs covered joints")
        if self.joint_order:
            if len(self.joint_order) != len(self.joints) or set(self.joint_order) != self.joints:
                raise ValueError("group joint order must list every covered joint once")
            if self.frame is not None or self.tcp is not None:
                raise ValueError("joint group cannot also declare a frame/TCP")
        elif not self.frame or not self.frame.strip() or not self.tcp or not self.tcp.strip():
            raise ValueError("coordinate group needs a frame and TCP")
        if not self.feedback_fields <= frozenset(JOINT_STATE_FIELDS):
            raise ValueError("group feedback fields must be joint state fields")


@dataclass(frozen=True, slots=True)
class GroupCommandOutput:
    """One complete, immutable-by-contract payload addressed to a group."""

    interface: str
    payload: object


@dataclass(frozen=True, slots=True)
class GroupCommandLease:
    """Local authority for one payload. Not serializable or renewable.

    Compute outside the guard. Pass the final synchronous, bounded write to
    ``try_write``; revocation and writes share a lock. A false result means
    the write did not run. The write must not reenter the coordinator.
    """

    expires_at: float  # time.monotonic()
    _guard: Callable[[Callable[[], None]], bool] = field(repr=False)

    def try_write(self, write: Callable[[], None]) -> bool:
        return self._guard(write)


GroupCommandHandler = Callable[[GroupCommandOutput, GroupCommandLease], None]
