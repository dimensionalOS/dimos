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

"""LCM type for low-level motor commands: q, dq, kp, kd, tau per motor."""

from io import BytesIO
import struct
import time


class MotorCommandArray:
    """Per-motor q/dq/kp/kd/tau. Joint order is implicit (caller-defined)."""

    msg_name = "sensor_msgs.MotorCommandArray"

    __slots__ = ["dq", "kd", "kp", "num_joints", "q", "tau", "timestamp"]

    def __init__(
        self,
        q: list[float] | None = None,
        dq: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        tau: list[float] | None = None,
        timestamp: float | None = None,
    ) -> None:
        if q is None:
            q = []
        if timestamp is None:
            timestamp = time.time()

        n = len(q)
        if dq is None:
            dq = [0.0] * n
        if kp is None:
            kp = [0.0] * n
        if kd is None:
            kd = [0.0] * n
        if tau is None:
            tau = [0.0] * n

        if not (len(dq) == len(kp) == len(kd) == len(tau) == n):
            raise ValueError(
                f"All arrays must have length {n}; got "
                f"dq={len(dq)} kp={len(kp)} kd={len(kd)} tau={len(tau)}"
            )

        self.timestamp = timestamp
        self.num_joints = n
        self.q = list(q)
        self.dq = list(dq)
        self.kp = list(kp)
        self.kd = list(kd)
        self.tau = list(tau)

    def lcm_encode(self) -> bytes:
        return self.encode()

    def encode(self) -> bytes:
        buf = BytesIO()
        buf.write(MotorCommandArray._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf: BytesIO) -> None:
        buf.write(struct.pack(">d", self.timestamp))
        buf.write(struct.pack(">i", self.num_joints))
        for arr in (self.q, self.dq, self.kp, self.kd, self.tau):
            for v in arr:
                buf.write(struct.pack(">d", v))

    @classmethod
    def lcm_decode(cls, data: bytes) -> "MotorCommandArray":
        return cls.decode(data)

    @classmethod
    def decode(cls, data: bytes | BytesIO) -> "MotorCommandArray":
        buf: BytesIO = data if isinstance(data, BytesIO) else BytesIO(data)
        if buf.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._decode_one(buf)

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> "MotorCommandArray":
        self = MotorCommandArray.__new__(MotorCommandArray)
        self.timestamp = struct.unpack(">d", buf.read(8))[0]
        self.num_joints = struct.unpack(">i", buf.read(4))[0]
        n = self.num_joints
        arrays = []
        for _ in range(5):
            arrays.append([struct.unpack(">d", buf.read(8))[0] for _ in range(n)])
        self.q, self.dq, self.kp, self.kd, self.tau = arrays
        return self

    @classmethod
    def _get_hash_recursive(cls, parents: list[type]) -> int:
        if cls in parents:
            return 0
        # Distinct fingerprint from JointCommand (0x8A3D2E1C5F4B6A9D)
        tmphash = (0x9B4E3F2D6A5C7B8E) & 0xFFFFFFFFFFFFFFFF
        tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
        return tmphash

    _packed_fingerprint: bytes | None = None

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        if cls._packed_fingerprint is None:
            cls._packed_fingerprint = struct.pack(">Q", cls._get_hash_recursive([]))
        return cls._packed_fingerprint

    def get_hash(self) -> int:
        return int(struct.unpack(">Q", MotorCommandArray._get_packed_fingerprint())[0])

    def __str__(self) -> str:
        return f"MotorCommandArray(timestamp={self.timestamp:.6f}, num_joints={self.num_joints})"

    def __repr__(self) -> str:
        return (
            f"MotorCommandArray(q={self.q}, dq={self.dq}, kp={self.kp}, "
            f"kd={self.kd}, tau={self.tau}, timestamp={self.timestamp})"
        )
