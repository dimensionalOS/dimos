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
import time
from io import BytesIO
from typing import BinaryIO

from dimos_lcm.geometry_msgs import Transform as LCMTransform
from dimos_lcm.geometry_msgs import TransformStamped as LCMTransformStamped
from plum import dispatch

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


class Transform(Timestamped):
    translation: Vector3
    rotation: Quaternion
    ts: float
    frame_id: str
    child_frame_id: str
    msg_name = "tf2_msgs.TFMessage"

    def __init__(
        self,
        translation: Vector3 | None = None,
        rotation: Quaternion | None = None,
        frame_id: str = "world",
        child_frame_id: str = "base_link",
        ts: float = 0.0,
        **kwargs,
    ) -> None:
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.ts = ts if ts != 0.0 else time.time()
        self.translation = translation if translation is not None else Vector3()
        self.rotation = rotation if rotation is not None else Quaternion()

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
