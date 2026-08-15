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

"""IMU noise-model information, the inertial sibling of ``CameraInfo``.

Where the IMU sits comes from tf (``header.frame_id`` names the frame), just as
camera extrinsics do; this message carries what tf cannot: the continuous-time
noise model consumers such as VIO preintegration need. The values are
Allan-variance constants, measured once per model (or per unit); IMUs do not
report them at runtime, so a driver publishes them from its config the same way
``camera_info`` is published alongside images.

LCM schema (fingerprint-compatible bindings must be generated from this)::

    package sensor_msgs;

    struct ImuInfo {
        std_msgs.Header header;
        double gyro_noise_density;
        double gyro_random_walk;
        double accel_noise_density;
        double accel_random_walk;
        double frequency;
    }
"""

from __future__ import annotations

from io import BytesIO
import struct

from dimos_lcm.std_msgs.Header import Header

from dimos.types.timestamped import Timestamped

# lcm-gen's base hash for the struct above; the wire fingerprint mixes in
# Header's recursively, matching the generated C++ in dimSLAM.
_BASE_HASH = 0x437A196CBD6B4E49


def _packed_fingerprint() -> bytes:
    tmphash = (_BASE_HASH + Header._get_hash_recursive([ImuInfo])) & 0xFFFFFFFFFFFFFFFF
    tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
    return struct.pack(">Q", tmphash)


class ImuInfo(Timestamped):
    """IMU noise model: Allan-variance constants plus the delivered sample rate."""

    msg_name = "sensor_msgs.ImuInfo"

    def __init__(
        self,
        gyro_noise_density: float = 0.0,
        gyro_random_walk: float = 0.0,
        accel_noise_density: float = 0.0,
        accel_random_walk: float = 0.0,
        frequency: float = 0.0,
        frame_id: str = "",
        ts: float | None = None,
    ) -> None:
        """Initialize ImuInfo.

        Args:
            gyro_noise_density: Gyroscope white noise, rad/s/sqrt(Hz)
            gyro_random_walk: Gyroscope bias random walk, rad/s^2/sqrt(Hz)
            accel_noise_density: Accelerometer white noise, m/s^2/sqrt(Hz)
            accel_random_walk: Accelerometer bias random walk, m/s^3/sqrt(Hz)
            frequency: Rate the samples are actually delivered at, Hz
            frame_id: The IMU frame; tf places it against the rest of the rig
            ts: Timestamp (defaults to now)
        """
        import time

        super().__init__(ts if ts is not None else time.time())
        self.gyro_noise_density = gyro_noise_density
        self.gyro_random_walk = gyro_random_walk
        self.accel_noise_density = accel_noise_density
        self.accel_random_walk = accel_random_walk
        self.frequency = frequency
        self.frame_id = frame_id

    def with_ts(self, ts: float) -> ImuInfo:
        """Return a copy of this ImuInfo with the given timestamp."""
        return ImuInfo(
            gyro_noise_density=self.gyro_noise_density,
            gyro_random_walk=self.gyro_random_walk,
            accel_noise_density=self.accel_noise_density,
            accel_random_walk=self.accel_random_walk,
            frequency=self.frequency,
            frame_id=self.frame_id,
            ts=ts,
        )

    def with_frame_id(self, frame_id: str) -> ImuInfo:
        """Return a copy of this ImuInfo stamped with the given frame."""
        copy = self.with_ts(self.ts)
        copy.frame_id = frame_id
        return copy

    def lcm_encode(self) -> bytes:
        header = Header()
        header.seq = 0
        header.frame_id = self.frame_id
        header.stamp.sec = int(self.ts)
        header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)

        buf = BytesIO()
        buf.write(_packed_fingerprint())
        header._encode_one(buf)
        buf.write(
            struct.pack(
                ">ddddd",
                self.gyro_noise_density,
                self.gyro_random_walk,
                self.accel_noise_density,
                self.accel_random_walk,
                self.frequency,
            )
        )
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes) -> ImuInfo:
        buf = BytesIO(data)
        if buf.read(8) != _packed_fingerprint():
            raise ValueError("Decode error")
        header = Header._decode_one(buf)
        values = struct.unpack(">ddddd", buf.read(40))
        return cls(
            gyro_noise_density=values[0],
            gyro_random_walk=values[1],
            accel_noise_density=values[2],
            accel_random_walk=values[3],
            frequency=values[4],
            frame_id=header.frame_id,
            ts=header.stamp.sec + header.stamp.nsec / 1e9,
        )

    def __repr__(self) -> str:
        return (
            f"ImuInfo(gyro_noise_density={self.gyro_noise_density}, "
            f"gyro_random_walk={self.gyro_random_walk}, "
            f"accel_noise_density={self.accel_noise_density}, "
            f"accel_random_walk={self.accel_random_walk}, "
            f"frequency={self.frequency}, frame_id='{self.frame_id}')"
        )
