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

"""Minimal geometry/message types for the standalone referee.

These replace ``dimos.msgs.*`` so the package runs with numpy + scipy +
pydantic only. Every numeric formula (euler <-> quaternion, Hamilton
product, q*v*q̄ rotation) is copied op-for-op from the dimos originals —
gold-closeness and consistency scores are compared bit-exactly against
the in-repo referee, so "cleaner" reimplementations are not welcome here.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from typing import Protocol, runtime_checkable

import numpy as np
import pydantic


class BaseConfig(pydantic.BaseModel):
    """Pydantic base mirroring dimos.protocol.service.spec.BaseConfig."""

    model_config = pydantic.ConfigDict(arbitrary_types_allowed=True, extra="forbid")


class Vector3:
    """3D vector; .roll/.pitch/.yaw alias x/y/z for euler-angle use."""

    __slots__ = ("x", "y", "z")

    x: float
    y: float
    z: float

    def __init__(
        self,
        x: float | list[float] | tuple[float, ...] | np.ndarray | Vector3 = 0.0,
        y: float = 0.0,
        z: float = 0.0,
    ) -> None:
        if isinstance(x, Vector3):
            self.x, self.y, self.z = x.x, x.y, x.z
        elif isinstance(x, (list, tuple, np.ndarray)):
            self.x, self.y, self.z = float(x[0]), float(x[1]), float(x[2])
        else:
            self.x, self.y, self.z = float(x), float(y), float(z)

    @property
    def roll(self) -> float:
        return self.x

    @property
    def pitch(self) -> float:
        return self.y

    @property
    def yaw(self) -> float:
        return self.z

    def __getitem__(self, idx: int) -> float:
        return (self.x, self.y, self.z)[idx]

    def __add__(self, other: Vector3) -> Vector3:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3) -> Vector3:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __repr__(self) -> str:
        return f"Vector3({self.x}, {self.y}, {self.z})"


class Quaternion:
    """Unit quaternion (x, y, z, w). Formulas verbatim from dimos Quaternion."""

    __slots__ = ("w", "x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)

    @classmethod
    def from_euler(cls, vector: Vector3) -> Quaternion:
        cy = np.cos(vector.yaw * 0.5)
        sy = np.sin(vector.yaw * 0.5)
        cp = np.cos(vector.pitch * 0.5)
        sp = np.sin(vector.pitch * 0.5)
        cr = np.cos(vector.roll * 0.5)
        sr = np.sin(vector.roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return cls(x, y, z, w)

    def to_euler(self) -> Vector3:
        # scipy, exactly as the dimos original — bit-parity over reimplementation
        from scipy.spatial.transform import Rotation

        e = Rotation.from_quat([self.x, self.y, self.z, self.w]).as_euler("xyz")
        return Vector3(e[0], e[1], e[2])

    @property
    def euler(self) -> Vector3:
        return self.to_euler()

    def __mul__(self, other: Quaternion) -> Quaternion:
        # Hamilton product
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(x, y, z, w)

    def conjugate(self) -> Quaternion:
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def rotate_vector(self, vector: Vector3) -> Vector3:
        # q * v * q̄ via pure-quaternion embedding, verbatim
        v_quat = Quaternion(vector.x, vector.y, vector.z, 0)
        rotated = self * v_quat * self.conjugate()
        return Vector3(rotated.x, rotated.y, rotated.z)

    def __repr__(self) -> str:
        return f"Quaternion({self.x:.6f}, {self.y:.6f}, {self.z:.6f}, {self.w:.6f})"


class Pose:
    """Position + orientation. Composition (+) matches dimos Pose.__add__."""

    def __init__(
        self,
        x: float | list[float] | tuple[float, ...] | np.ndarray | Vector3 | None = None,
        y: float | None = None,
        z: float | None = None,
        *,
        position: list[float] | tuple[float, ...] | np.ndarray | Vector3 | None = None,
        orientation: Quaternion | Sequence[float] | None = None,
    ) -> None:
        if position is not None or orientation is not None:
            pos = position if position is not None else Vector3()
            self.position = Vector3(pos)
            if orientation is None:
                self.orientation = Quaternion()
            elif isinstance(orientation, Quaternion):
                self.orientation = Quaternion(
                    orientation.x, orientation.y, orientation.z, orientation.w
                )
            else:
                self.orientation = Quaternion(*orientation)
        elif x is None:
            self.position = Vector3()
            self.orientation = Quaternion()
        elif isinstance(x, (list, tuple, np.ndarray, Vector3)):
            self.position = Vector3(x)
            self.orientation = Quaternion()
        else:
            self.position = Vector3(float(x), float(y or 0.0), float(z or 0.0))
            self.orientation = Quaternion()

    @property
    def x(self) -> float:
        return self.position.x

    @property
    def y(self) -> float:
        return self.position.y

    @property
    def z(self) -> float:
        return self.position.z

    def __add__(self, other: Pose) -> Pose:
        # transform composition, verbatim semantics from dimos Pose.__add__
        new_orientation = self.orientation * other.orientation
        rotated_position = self.orientation.rotate_vector(other.position)
        new_position = self.position + rotated_position
        out = Pose()
        out.position = new_position
        out.orientation = new_orientation
        return out

    def __repr__(self) -> str:
        return f"Pose(position={self.position!r}, orientation={self.orientation!r})"


class PoseStamped(Pose):
    """Pose with a frame id. ``ts`` defaults to 0.0 (nothing here reads it)."""

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        **kwargs: object,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts
        super().__init__(**kwargs)  # type: ignore[arg-type]


class Path(Sequence[PoseStamped]):
    """A sequence of poses; read-only list semantics like dimos Path."""

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "world",
        poses: list[PoseStamped] | None = None,
    ) -> None:
        self.ts = ts
        self.frame_id = frame_id
        self.poses = poses if poses is not None else []

    def __len__(self) -> int:
        return len(self.poses)

    def __bool__(self) -> bool:
        return len(self.poses) > 0

    def __iter__(self) -> Iterator[PoseStamped]:
        return iter(self.poses)

    def __getitem__(self, index):  # type: ignore[no-untyped-def]
        if isinstance(index, slice):
            return Path(ts=self.ts, frame_id=self.frame_id, poses=self.poses[index])
        return self.poses[index]


class PointCloud2:
    """Point positions stored float32; numpy in, numpy out."""

    def __init__(self, points: np.ndarray | None = None, frame_id: str = "world") -> None:
        pts = np.zeros((0, 3), dtype=np.float32) if points is None else points
        self._points = np.ascontiguousarray(pts, dtype=np.float32).reshape(-1, 3)
        self.frame_id = frame_id

    @classmethod
    def from_numpy(cls, points: np.ndarray, frame_id: str = "world") -> PointCloud2:
        return cls(points, frame_id=frame_id)

    def points_f32(self) -> np.ndarray:
        return self._points

    def as_numpy(self) -> tuple[np.ndarray, None]:
        # dimos stores f32 and converts to f64 on the legacy read path;
        # f32 -> f64 is exact, so this matches bit-for-bit.
        return self._points.astype(np.float64), None

    def __len__(self) -> int:
        return len(self._points)


class SolidPrimitive:
    """Box / sphere / cylinder / cone with ROS type codes."""

    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4
    SPHERE_RADIUS = 0

    _NAMES = {1: "box", 2: "sphere", 3: "cylinder", 4: "cone"}

    def __init__(self, type: int = 0, dimensions: list[float] | None = None) -> None:
        self.type = type
        self.dimensions = [float(d) for d in dimensions or []]

    @classmethod
    def box(cls, x: float, y: float, z: float) -> SolidPrimitive:
        return cls(cls.BOX, [x, y, z])

    @classmethod
    def sphere(cls, radius: float) -> SolidPrimitive:
        return cls(cls.SPHERE, [radius])

    @classmethod
    def cylinder(cls, height: float, radius: float) -> SolidPrimitive:
        return cls(cls.CYLINDER, [height, radius])

    @classmethod
    def cone(cls, height: float, radius: float) -> SolidPrimitive:
        return cls(cls.CONE, [height, radius])

    def __repr__(self) -> str:
        dims = ", ".join(f"{d:g}" for d in self.dimensions)
        return f"SolidPrimitive.{self._NAMES.get(self.type, 'unknown')}({dims})"


@runtime_checkable
class PoseLike(Protocol):
    position: Vector3
    orientation: Quaternion


@runtime_checkable
class CloudLike(Protocol):
    def points_f32(self) -> np.ndarray: ...
