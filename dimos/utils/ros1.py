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

"""ROS1 binary message deserialization — no ROS1 installation required.

Implements pure-Python deserialization of standard ROS1 message types from their
binary wire format (as used by the Unity ROS-TCP-Connector). These messages use
little-endian encoding with uint32-length-prefixed strings and arrays.

Wire format basics:
  - Primitive types: packed directly (e.g. uint32 = 4 bytes LE)
  - Strings: uint32 length + N bytes (no null terminator in wire format)
  - Arrays: uint32 count + N * element_size bytes
  - Time: uint32 sec + uint32 nsec
  - Nested messages: serialized inline (no length prefix for fixed-size)

Supported types:
  - sensor_msgs/PointCloud2
  - sensor_msgs/CompressedImage
  - geometry_msgs/PoseStamped (serialize + deserialize)
  - geometry_msgs/TwistStamped (serialize)
  - nav_msgs/Odometry (deserialize)
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time

import numpy as np

# ---------------------------------------------------------------------------
# Low-level readers
# ---------------------------------------------------------------------------


class ROS1Reader:
    """Stateful reader for ROS1 binary serialized data."""

    __slots__ = ("data", "off")

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.off = 0

    def u8(self) -> int:
        v = self.data[self.off]
        self.off += 1
        return v

    def bool(self) -> bool:
        return self.u8() != 0

    def u32(self) -> int:
        (v,) = struct.unpack_from("<I", self.data, self.off)
        self.off += 4
        return int(v)

    def i32(self) -> int:
        (v,) = struct.unpack_from("<i", self.data, self.off)
        self.off += 4
        return int(v)

    def f32(self) -> float:
        (v,) = struct.unpack_from("<f", self.data, self.off)
        self.off += 4
        return float(v)

    def f64(self) -> float:
        (v,) = struct.unpack_from("<d", self.data, self.off)
        self.off += 8
        return float(v)

    def string(self) -> str:
        length = self.u32()
        s = self.data[self.off : self.off + length].decode("utf-8", errors="replace")
        self.off += length
        return s

    def time(self) -> float:
        """Read ROS1 time (uint32 sec + uint32 nsec) → float seconds."""
        sec = self.u32()
        nsec = self.u32()
        return sec + nsec / 1e9

    def raw(self, n: int) -> bytes:
        v = self.data[self.off : self.off + n]
        self.off += n
        return v

    def remaining(self) -> int:
        return len(self.data) - self.off


# ---------------------------------------------------------------------------
# Low-level writer
# ---------------------------------------------------------------------------


class ROS1Writer:
    """Stateful writer for ROS1 binary serialized data."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def u8(self, v: int) -> None:
        self.buf.append(v & 0xFF)

    def bool(self, v: bool) -> None:
        self.u8(1 if v else 0)

    def u32(self, v: int) -> None:
        self.buf += struct.pack("<I", v)

    def i32(self, v: int) -> None:
        self.buf += struct.pack("<i", v)

    def f32(self, v: float) -> None:
        self.buf += struct.pack("<f", v)

    def f64(self, v: float) -> None:
        self.buf += struct.pack("<d", v)

    def string(self, s: str) -> None:
        b = s.encode("utf-8")
        self.u32(len(b))
        self.buf += b

    def time(self, t: float | None = None) -> None:
        if t is None:
            t = time.time()
        sec = int(t)
        nsec = int((t - sec) * 1e9)
        self.u32(sec)
        self.u32(nsec)

    def raw(self, data: bytes) -> None:
        self.buf += data

    def bytes(self) -> bytes:
        return bytes(self.buf)


# ---------------------------------------------------------------------------
# Header (std_msgs/Header)
# ---------------------------------------------------------------------------


@dataclass
class ROS1Header:
    seq: int = 0
    stamp: float = 0.0  # seconds
    frame_id: str = ""


def read_header(r: ROS1Reader) -> ROS1Header:
    seq = r.u32()
    stamp = r.time()
    frame_id = r.string()
    return ROS1Header(seq, stamp, frame_id)


def write_header(
    w: ROS1Writer, frame_id: str = "map", stamp: float | None = None, seq: int = 0
) -> None:
    w.u32(seq)
    w.time(stamp)
    w.string(frame_id)


# ---------------------------------------------------------------------------
# sensor_msgs/PointCloud2
# ---------------------------------------------------------------------------


@dataclass
class ROS1PointField:
    name: str
    offset: int
    datatype: int  # 7=FLOAT32, 8=FLOAT64, etc.
    count: int


def deserialize_pointcloud2(data: bytes) -> tuple[np.ndarray, str, float] | None:
    """Deserialize ROS1 sensor_msgs/PointCloud2 → (Nx3 float32 points, frame_id, timestamp).

    Returns None on parse failure.
    """
    try:
        r = ROS1Reader(data)
        header = read_header(r)

        height = r.u32()
        width = r.u32()
        num_points = height * width

        # PointField array
        num_fields = r.u32()
        x_off = y_off = z_off = -1
        for _ in range(num_fields):
            name = r.string()
            offset = r.u32()
            r.u8()
            r.u32()
            if name == "x":
                x_off = offset
            elif name == "y":
                y_off = offset
            elif name == "z":
                z_off = offset

        r.bool()
        point_step = r.u32()
        r.u32()

        # Data array
        data_len = r.u32()
        raw_data = r.raw(data_len)

        # is_dense
        if r.remaining() > 0:
            r.bool()

        if x_off < 0 or y_off < 0 or z_off < 0:
            return None
        if num_points == 0:
            return np.zeros((0, 3), dtype=np.float32), header.frame_id, header.stamp

        # Fast path: standard XYZI layout
        if x_off == 0 and y_off == 4 and z_off == 8 and point_step >= 12:
            if point_step == 12:
                points = (
                    np.frombuffer(raw_data, dtype=np.float32, count=num_points * 3)
                    .reshape(-1, 3)
                    .copy()
                )
            else:
                dt = np.dtype(
                    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{point_step - 12}")]
                )
                structured = np.frombuffer(raw_data, dtype=dt, count=num_points)
                points = np.column_stack((structured["x"], structured["y"], structured["z"])).copy()
        else:
            # Slow path: arbitrary field offsets
            points = np.zeros((num_points, 3), dtype=np.float32)
            for i in range(num_points):
                base = i * point_step
                points[i, 0] = struct.unpack_from("<f", raw_data, base + x_off)[0]
                points[i, 1] = struct.unpack_from("<f", raw_data, base + y_off)[0]
                points[i, 2] = struct.unpack_from("<f", raw_data, base + z_off)[0]

        return points, header.frame_id, header.stamp
    except Exception:
        return None


# ---------------------------------------------------------------------------
# sensor_msgs/CompressedImage
# ---------------------------------------------------------------------------


def deserialize_compressed_image(data: bytes) -> tuple[bytes, str, str, float] | None:
    """Deserialize ROS1 sensor_msgs/CompressedImage → (raw_data, format, frame_id, timestamp).

    The raw_data is JPEG/PNG bytes that can be decoded with cv2.imdecode or PIL.
    Returns None on parse failure.
    """
    try:
        r = ROS1Reader(data)
        header = read_header(r)
        fmt = r.string()  # e.g. "jpeg", "png"
        img_len = r.u32()
        img_data = r.raw(img_len)
        return img_data, fmt, header.frame_id, header.stamp
    except Exception:
        return None


# ---------------------------------------------------------------------------
# geometry_msgs/PoseStamped (serialize)
# ---------------------------------------------------------------------------


def serialize_pose_stamped(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    frame_id: str = "map",
    stamp: float | None = None,
) -> bytes:
    """Serialize geometry_msgs/PoseStamped in ROS1 wire format."""
    w = ROS1Writer()
    write_header(w, frame_id, stamp)
    # Pose: position (3x f64) + orientation (4x f64)
    w.f64(x)
    w.f64(y)
    w.f64(z)
    w.f64(qx)
    w.f64(qy)
    w.f64(qz)
    w.f64(qw)
    return w.bytes()
