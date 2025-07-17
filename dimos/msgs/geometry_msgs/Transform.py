# Copyright 2025 Dimensional Inc.
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

import struct
from io import BytesIO
from typing import BinaryIO

from dimos_lcm.geometry_msgs import Transform as LCMTransform
from plum import dispatch

from .Quaternion import Quaternion
from .Vector3 import Vector3


class Transform(LCMTransform):
    translation: Vector3
    rotation: Quaternion
    msg_name = "geometry_msgs.Transform"

    @dispatch
    def __init__(self) -> None:
        """Initialize an identity transform."""
        self.translation = Vector3()
        self.rotation = Quaternion()

    @dispatch
    def __init__(self, translation: Vector3) -> None:
        """Initialize a transform with only translation (identity rotation)."""
        self.translation = translation
        self.rotation = Quaternion()

    @dispatch
    def __init__(self, rotation: Quaternion) -> None:
        """Initialize a transform with only rotation (zero translation)."""
        self.translation = Vector3()
        self.rotation = rotation

    @dispatch
    def __init__(self, translation: Vector3, rotation: Quaternion) -> None:
        """Initialize a transform from translation and rotation."""
        self.translation = translation
        self.rotation = rotation

    @dispatch
    def __init__(self, transform: "Transform") -> None:
        """Initialize from another Transform (copy constructor)."""
        self.translation = Vector3(transform.translation)
        self.rotation = Quaternion(transform.rotation)

    @dispatch
    def __init__(self, lcm_transform: LCMTransform) -> None:
        """Initialize from an LCM Transform."""
        self.translation = Vector3(lcm_transform.translation)
        self.rotation = Quaternion(lcm_transform.rotation)

    @dispatch
    def __init__(self, **kwargs):
        """Handle keyword arguments for LCM compatibility."""
        # Get values with defaults and let dispatch handle type conversion
        translation = kwargs.get("translation", Vector3())
        rotation = kwargs.get("rotation", Quaternion())

        # Call the appropriate positional init - dispatch will handle the types
        self.__init__(translation, rotation)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):
        if not hasattr(data, "read"):
            data = BytesIO(data)
        if data.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._lcm_decode_one(data)

    @classmethod
    def _lcm_decode_one(cls, buf):
        translation = Vector3._lcm_decode_one(buf)
        rotation = Quaternion._lcm_decode_one(buf)
        return cls(translation=translation, rotation=rotation)

    def lcm_encode(self) -> bytes:
        return super().encode()

    def __repr__(self) -> str:
        return f"Transform(translation={self.translation!r}, rotation={self.rotation!r})"

    def __str__(self) -> str:
        return f"Transform:\n  Translation: {self.translation}\n  Rotation: {self.rotation}"

    def __eq__(self, other) -> bool:
        """Check if two transforms are equal."""
        if not isinstance(other, Transform):
            return False
        return self.translation == other.translation and self.rotation == other.rotation

    @classmethod
    def identity(cls) -> Transform:
        """Create an identity transform."""
        return cls()
