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

from __future__ import annotations

import time
from typing import Any, TypeAlias

from dimos_lcm.sensor_msgs import JointState as LCMJointState

from dimos.types.timestamped import Timestamped

# Types that can be converted to/from JointState
JointStateConvertable: TypeAlias = dict[str, list[str] | list[float]] | LCMJointState

_FIELDS = ("ts", "frame_id", "name", "position", "velocity", "effort")


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


def _fields_of(source: Any) -> tuple[Any, ...]:
    """Pull the six JointState fields out of a dict, a JointState or an LCM message."""
    if isinstance(source, dict):
        return (
            source.get("ts"),
            source.get("frame_id", ""),
            source.get("name"),
            source.get("position"),
            source.get("velocity"),
            source.get("effort"),
        )
    if isinstance(source, JointState):
        return (source.ts, source.frame_id, *(list(getattr(source, f)) for f in _FIELDS[2:]))
    return (
        source.header.stamp.sec + (source.header.stamp.nsec / 1_000_000_000),
        source.header.frame_id,
        *(list(getattr(source, f) or []) for f in _FIELDS[2:]),
    )


class JointState(Timestamped):
    msg_name = "sensor_msgs.JointState"
    ts: float
    frame_id: str
    name: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]

    def __init__(
        self,
        ts: float | JointStateConvertable | JointState | None = None,
        frame_id: str = "",
        name: list[str] | None = None,
        position: list[float] | None = None,
        velocity: list[float] | None = None,
        effort: list[float] | None = None,
    ) -> None:
        """Initialize a JointState message.

        The first argument doubles as the source for the copy/dict/LCM forms:
        ``JointState(other)``, ``JointState({...})``, ``JointState(lcm_msg)``.

        Args:
            ts: Timestamp in seconds
            frame_id: Frame ID for the message
            name: List of joint names
            position: List of joint positions (rad or m); gripper joints use
                the unit their adapter declares
            velocity: List of joint velocities (rad/s or m/s)
            effort: List of joint efforts (Nm or N)
        """
        stamp: Any = ts
        if isinstance(stamp, dict | JointState | LCMJointState):
            stamp, frame_id, name, position, velocity, effort = _fields_of(stamp)

        self.ts = time.time() if stamp is None else stamp
        self.frame_id = frame_id
        self.name = name if name is not None else []
        self.position = position if position is not None else []
        self.velocity = velocity if velocity is not None else []
        self.effort = effort if effort is not None else []

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMJointState()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.name_length = len(self.name)
        lcm_msg.name = self.name
        lcm_msg.position_length = len(self.position)
        lcm_msg.position = self.position
        lcm_msg.velocity_length = len(self.velocity)
        lcm_msg.velocity = self.velocity
        lcm_msg.effort_length = len(self.effort)
        lcm_msg.effort = self.effort
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> JointState:
        lcm_msg = LCMJointState.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            name=list(lcm_msg.name) if lcm_msg.name else [],
            position=list(lcm_msg.position) if lcm_msg.position else [],
            velocity=list(lcm_msg.velocity) if lcm_msg.velocity else [],
            effort=list(lcm_msg.effort) if lcm_msg.effort else [],
        )

    def __str__(self) -> str:
        return f"JointState({len(self.name)} joints, frame_id='{self.frame_id}')"

    def __repr__(self) -> str:
        return (
            f"JointState(ts={self.ts}, frame_id='{self.frame_id}', "
            f"name={self.name}, position={self.position}, "
            f"velocity={self.velocity}, effort={self.effort})"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two JointState messages are equal."""
        if not isinstance(other, JointState):
            return False
        return (
            self.name == other.name
            and self.position == other.position
            and self.velocity == other.velocity
            and self.effort == other.effort
            and self.frame_id == other.frame_id
        )
