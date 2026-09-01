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

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.perception.detection.type.detection3d.bbox import Detection3DBBox


@dataclass
class Detection3DMarker(Detection3DBBox):
    """Fiducial marker (ArUco / AprilTag) detection with a world-frame pose.

    ``bbox`` is the axis-aligned 2D bbox around the four detected corners so
    crop/draw helpers from ``Detection2DBBox`` work as-is. ``center`` and
    ``orientation`` carry the marker pose in ``frame_id`` (typically
    ``"world"``); ``transform`` is the camera-in-world transform used to
    compose it.
    """

    marker_id: int = -1
    corners_px: np.ndarray = field(default_factory=lambda: np.zeros((4, 2), dtype=np.float32))
    dictionary: str = ""
    reprojection_error: float = 0.0

    def __post_init__(self) -> None:
        self.name = self.marker_label

    @property
    def marker_label(self) -> str:
        """Dictionary-qualified marker label used for display and wire class id."""
        return f"{self.dictionary}:{self.marker_id}"

    def to_detection3d_msg(self) -> Detection3D:
        """Convert to a ROS Detection3D message, preserving marker identity."""
        msg = super().to_detection3d_msg()
        msg.id = str(self.marker_id)
        if msg.results:
            msg.results[0].hypothesis.class_id = self.marker_label
        msg.results_length = len(msg.results)
        return msg

    def to_repr_dict(self) -> dict[str, Any]:
        parent = super().to_repr_dict()
        return {
            **parent,
            "marker_id": str(self.marker_id),
            "dict": self.dictionary,
            "reproj": f"{self.reprojection_error:.3f}px",
        }
