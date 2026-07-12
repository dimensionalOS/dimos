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

"""Live marker detection as a memory2 StreamModule.

The module keeps the same transform chain used by offline marker tooling:
quality-gated images, optional motion gating, marker fan-out, then one
``Detection3DArray`` per processed frame for LCM consumers.
"""

from __future__ import annotations

import time
from typing import Any, cast

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2.module import StreamModule, stream_to_port
from dimos.memory2.store.null import NullStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow, SpeedLimit
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_pose import camera_optical_frame_id, is_fisheye_model
from dimos.perception.fiducial.marker_transformer import DetectMarkers, MarkersPerFrame
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MarkerDetectionStreamModuleConfig(ModuleConfig):
    """Configuration for :class:`MarkerDetectionStreamModule`."""

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


class MarkerDetectionStreamModule(StreamModule[Image, Detection3DArray]):
    """Publish fiducial marker detections as ``Detection3DArray`` messages."""

    config: MarkerDetectionStreamModuleConfig

    color_image: In[Image]
    detections: Out[Detection3DArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._warned_distortion_model = False

    def pipeline(self, stream: Stream[Image]) -> Stream[Detection3DArray]:
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
        return markers.transform(MarkersPerFrame(frame_id=self.config.world_frame))

    def _maybe_warn_distortion(self, camera_info: CameraInfo) -> None:
        model = (camera_info.distortion_model or "").strip().lower()
        if model in ("", "plumb_bob") or is_fisheye_model(model):
            return
        if not self._warned_distortion_model:
            logger.warning(
                "MarkerDetectionStreamModule: distortion_model=%r may be unsupported; "
                "using D as-is.",
                camera_info.distortion_model,
            )
            self._warned_distortion_model = True

    def _append_image_with_pose(self, stream: Stream[Image], image: Image) -> None:
        info = self.config.camera_info
        if info is None:
            logger.debug("MarkerDetectionStreamModule: no CameraInfo yet; skipping frame")
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
                "MarkerDetectionStreamModule: no TF %s -> %s at ts=%s",
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

    @rpc
    def start(self) -> None:
        Module.start(self)

        if len(self.inputs) != 1 or len(self.outputs) != 1:
            raise TypeError(
                f"{self.__class__.__name__} must have exactly one In and one Out port, "
                f"found {len(self.inputs)} In and {len(self.outputs)} Out"
            )

        store = self.register_disposable(NullStore())
        store.start()
        stream: Stream[Image] = store.stream("color_image", Image)

        if self.config.camera_info is not None:
            self._maybe_warn_distortion(self.config.camera_info)

        unsub_image = self.color_image.subscribe(
            lambda image: self._append_image_with_pose(
                stream, image.decode() if isinstance(image, CompressedImage) else image
            )
        )
        self.register_disposable(Disposable(unsub_image) if callable(unsub_image) else unsub_image)
        self.register_disposable(
            stream_to_port(self._apply_pipeline(stream.live()), self.detections)
        )

    @rpc
    def stop(self) -> None:
        super().stop()


class CompressedMarkerDetectionStreamModule(MarkerDetectionStreamModule):
    """MarkerDetectionStreamModule for CompressedImage camera graphs (#2831)."""

    color_image: In[CompressedImage]  # type: ignore[assignment]  # deliberate retype: go2 graphs wire compressed frames (#2831)
