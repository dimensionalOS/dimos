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

"""Live marker detection as a memory2 fan-I/O StreamModule.

The module keeps the same transform chain used by offline marker tooling:
quality-gated images, optional motion gating, marker fan-out, then one
``Detection3DArray`` (world poses for LCM / TF) and one ``Detection2DArray``
(image-plane overlay) per processed frame, both produced from a single marker
detection pass and scattered to their ports in one subscribe.
"""

from __future__ import annotations

import time
from typing import Any, cast

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2.fanio import Bundle
from dimos.memory2.module import StreamModule
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow, SpeedLimit
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_pose import camera_optical_frame_id, is_fisheye_model
from dimos.perception.fiducial.marker_transformer import DetectMarkers, MarkersToBundle
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MarkerModuleConfig(ModuleConfig):
    """Configuration for :class:`MarkerModule`."""

    world_frame: str = "world"
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    marker_length_m: float = Field(
        ..., gt=0.0, description="Physical square marker edge length in meters."
    )
    quality_window_s: float = Field(0.5, gt=0.0)
    smoothing_window: float = Field(0.0, ge=0.0)
    speed_limit_enabled: bool = False
    speed_limit_max_mps: float = Field(0.05, gt=0.0)
    speed_limit_max_dps: float = Field(15.0, gt=0.0)
    tf_lookup_tolerance: float = Field(0.5, ge=0.0)
    camera_info: CameraInfo | None = None


class MarkerModule(StreamModule):
    """Publish fiducial marker detections as both ``Detection3DArray`` (world
    poses for LCM / TF consumers) and ``Detection2DArray`` (image-plane overlays).

    A single detection pass feeds both outputs: ``pipeline()`` ends in a
    :class:`Bundle` keyed by Out port name, and the fan-I/O base scatters it in
    one subscribe (so :class:`DetectMarkers` runs once per frame regardless of
    output count). Per-frame world->camera_optical pose anchoring happens in the
    :meth:`ingest` seam, which replaces the copied ``start()`` the old 1:1 base
    required.
    """

    config: MarkerModuleConfig

    color_image: In[Image]
    detections_3d: Out[Detection3DArray]
    detections_2d: Out[Detection2DArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._warned_distortion_model = False
        if self.config.camera_info is not None:
            self._maybe_warn_distortion(self.config.camera_info)

    def ingest(self, name: str, stream: Stream[Image], image: Image) -> None:
        """Anchor each frame with its world->camera_optical pose before the pipeline.

        Overrides the default append so marker detection has a camera-in-world
        pose at the image timestamp; frames without CameraInfo or TF are dropped
        (the base ``start()`` wiring stays untouched).
        """
        self._append_image_with_pose(stream, image)

    def pipeline(self, stream: Stream[Image]) -> Stream[Bundle]:
        result: Stream[Any] = stream.transform(
            QualityWindow(lambda img: img.sharpness, window=self.config.quality_window_s)
        )
        if self.config.speed_limit_enabled:
            result = result.transform(
                SpeedLimit(
                    max_mps=self.config.speed_limit_max_mps,
                    max_dps=self.config.speed_limit_max_dps,
                )
            )

        markers = cast(
            "Stream[Detection3DMarker | None]",
            result.transform(
                DetectMarkers(
                    camera_info=self.config.camera_info,
                    marker_length_m=self.config.marker_length_m,
                    aruco_dictionary=self.config.aruco_dictionary,
                    world_frame=self.config.world_frame,
                    smoothing_window=self.config.smoothing_window,
                    emit_empty_frames=True,
                )
            ),
        )
        return markers.transform(MarkersToBundle(frame_id=self.config.world_frame))

    def _maybe_warn_distortion(self, camera_info: CameraInfo) -> None:
        model = (camera_info.distortion_model or "").strip().lower()
        if model in ("", "plumb_bob") or is_fisheye_model(model):
            return
        if not self._warned_distortion_model:
            logger.warning(
                "MarkerModule: distortion_model=%r may be unsupported; using D as-is.",
                camera_info.distortion_model,
            )
            self._warned_distortion_model = True

    def _append_image_with_pose(self, stream: Stream[Image], image: Image) -> None:
        info = self.config.camera_info
        if info is None:
            logger.debug("MarkerModule: no CameraInfo yet; skipping frame")
            return

        ts = getattr(image, "ts", None) or time.time()
        optical = camera_optical_frame_id(image, info)
        t_world_optical = self.tf.get(
            self.config.world_frame,
            optical,
            time_point=ts,
            time_tolerance=self.config.tf_lookup_tolerance,
        )
        if t_world_optical is None:
            logger.debug(
                "MarkerModule: no TF %s -> %s at ts=%s",
                self.config.world_frame,
                optical,
                ts,
            )
            return

        stream.append(
            image,
            ts=ts,
            pose=(
                t_world_optical.translation.x,
                t_world_optical.translation.y,
                t_world_optical.translation.z,
                t_world_optical.rotation.x,
                t_world_optical.rotation.y,
                t_world_optical.rotation.z,
                t_world_optical.rotation.w,
            ),
        )
