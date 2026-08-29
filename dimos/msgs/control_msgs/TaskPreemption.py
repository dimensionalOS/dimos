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

"""Wire-safe control-task preemption transition."""

from __future__ import annotations

from io import BytesIO
import struct
from typing import ClassVar


class TaskPreemption:
    """A task losing one or more joints to a higher-priority task."""

    msg_name: ClassVar[str] = "control_msgs.TaskPreemption"
    _FINGERPRINT: ClassVar[bytes] = struct.pack(">Q", 0x8B7894B218D03343)

    def __init__(
        self,
        *,
        timestamp: float,
        preempted_task: str,
        preempting_task: str,
        joints: list[str],
    ) -> None:
        self.timestamp = timestamp
        self.preempted_task = preempted_task
        self.preempting_task = preempting_task
        self.joints = list(joints)

    def lcm_encode(self) -> bytes:
        buffer = BytesIO()
        buffer.write(self._FINGERPRINT)
        buffer.write(struct.pack(">d", self.timestamp))
        self._write_string(buffer, self.preempted_task)
        self._write_string(buffer, self.preempting_task)
        buffer.write(struct.pack(">i", len(self.joints)))
        for joint in self.joints:
            self._write_string(buffer, joint)
        return buffer.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes) -> TaskPreemption:
        buffer = BytesIO(data)
        if buffer.read(8) != cls._FINGERPRINT:
            raise ValueError("TaskPreemption fingerprint mismatch")
        timestamp = struct.unpack(">d", buffer.read(8))[0]
        preempted_task = cls._read_string(buffer)
        preempting_task = cls._read_string(buffer)
        joint_count = struct.unpack(">i", buffer.read(4))[0]
        if joint_count < 0:
            raise ValueError("TaskPreemption joint count cannot be negative")
        joints = [cls._read_string(buffer) for _ in range(joint_count)]
        if buffer.read(1):
            raise ValueError("TaskPreemption has trailing bytes")
        return cls(
            timestamp=timestamp,
            preempted_task=preempted_task,
            preempting_task=preempting_task,
            joints=joints,
        )

    @staticmethod
    def _write_string(buffer: BytesIO, value: str) -> None:
        encoded = value.encode("utf-8")
        buffer.write(struct.pack(">i", len(encoded)))
        buffer.write(encoded)

    @staticmethod
    def _read_string(buffer: BytesIO) -> str:
        length_data = buffer.read(4)
        if len(length_data) != 4:
            raise ValueError("TaskPreemption string length is truncated")
        length = struct.unpack(">i", length_data)[0]
        if length < 0:
            raise ValueError("TaskPreemption string length cannot be negative")
        encoded = buffer.read(length)
        if len(encoded) != length:
            raise ValueError("TaskPreemption string is truncated")
        return encoded.decode("utf-8")
