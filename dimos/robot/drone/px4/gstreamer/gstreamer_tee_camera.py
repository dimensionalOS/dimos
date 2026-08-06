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

"""Single-source PX4 camera Module with raw BGR and Annex-B H.264 outputs."""

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import time
from typing import ClassVar

import numpy as np
import reactivex as rx
from typing_extensions import assert_never

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstApi,
    GstBusEvent,
    GstBusMessage,
    GstPipeline,
    GstPipelineError,
    GstSourceConfig,
    GstTeePipelineSpec,
    H264AccessUnit,
    RawBgrSample,
)
from dimos.robot.drone.px4.gstreamer.pygi_gstreamer import PyGstApi


class CameraLifecycle(str, Enum):
    """Observable state of the owned local source pipeline."""

    CREATED = "created"
    RUNNING = "running"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    """Configured intrinsics and the physical parent-to-camera-link transform."""

    camera_info: CameraInfo
    camera_link: Transform


DEFAULT_SOURCE_CONFIG = GstSourceConfig()


class Px4GstTeeCamera(Module):
    """Own one live source pipeline and publish its raw and encoded tee branches."""

    dedicated_worker: ClassVar[bool] = True

    color_image: Out[Image]
    video_h264: Out[CompressedVideo]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(
        self,
        *,
        gst_api: GstApi | None = None,
        source_config: GstSourceConfig = DEFAULT_SOURCE_CONFIG,
        calibration: CameraCalibration | None = None,
        clock: Callable[[], float] = time.time,
        g: GlobalConfig = global_config,
    ) -> None:
        super().__init__(frame_id="camera_optical", g=g)
        self._gst_api: GstApi | None = gst_api
        self._source_config: GstSourceConfig = source_config
        self._calibration: CameraCalibration | None = calibration
        self._clock: Callable[[], float] = clock
        self._pipeline: GstPipeline | None = None
        self.lifecycle: CameraLifecycle = CameraLifecycle.CREATED
        self.last_bus_error: str | None = None
        self.keyframe_count: int = 0

    @rpc
    def start(self) -> None:
        """Create, bind, and start the one owned tee pipeline."""
        if self.lifecycle is not CameraLifecycle.CREATED:
            raise GstPipelineError(detail=f"cannot start camera from {self.lifecycle}")
        super().start()
        gst_api = self._gst_api or PyGstApi()
        pipeline = gst_api.create_pipeline(self._pipeline_spec())
        pipeline.set_callbacks(self._on_raw_sample, self._on_h264_access_unit, self._on_bus_message)
        self._pipeline = pipeline
        try:
            pipeline.start()
        except GstPipelineError:
            self._release_pipeline()
            self.lifecycle = CameraLifecycle.FAILED
            raise
        self.lifecycle = CameraLifecycle.RUNNING
        self._publish_metadata()
        if self._has_valid_calibration():
            _ = self.register_disposable(
                rx.interval(1.0).subscribe(lambda _: self._publish_metadata())
            )

    @rpc
    def stop(self) -> None:
        """Release the owned Gst pipeline before closing Module resources."""
        self._release_pipeline()
        if self.lifecycle is not CameraLifecycle.FAILED:
            self.lifecycle = CameraLifecycle.STOPPED
        super().stop()

    def _pipeline_spec(self) -> GstTeePipelineSpec:
        return GstTeePipelineSpec(source=self._source_config)

    def _on_raw_sample(self, sample: RawBgrSample) -> None:
        """Publish a copied BGR frame from the raw appsink."""
        data = (
            np.frombuffer(sample.data, dtype=np.uint8)
            .reshape((sample.height, sample.width, 3))
            .copy()
        )
        self.color_image.publish(
            Image(
                data=data,
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=sample.pts_ns / 1_000_000_000,
            )
        )

    def _on_h264_access_unit(self, access_unit: H264AccessUnit) -> None:
        """Publish every encoded access unit in callback order without throttling."""
        if not access_unit.delta_unit:
            self.keyframe_count += 1
        self.video_h264.publish(
            CompressedVideo(
                data=access_unit.data,
                format="h264",
                frame_id=self.frame_id,
                ts=access_unit.pts_ns / 1_000_000_000,
            )
        )

    def _on_bus_message(self, message: GstBusMessage) -> None:
        """Stop the local pipeline exactly once after a terminal bus event."""
        match message.event:
            case GstBusEvent.ERROR | GstBusEvent.EOS:
                self.last_bus_error = message.detail
                self._release_pipeline()
                self.lifecycle = CameraLifecycle.FAILED
            case unreachable:
                assert_never(unreachable)

    def _publish_metadata(self) -> None:
        calibration = self._calibration
        if calibration is None or not self._has_valid_calibration():
            return
        timestamp = self._clock()
        camera_info = calibration.camera_info.with_ts(timestamp)
        self.camera_info.publish(camera_info)
        camera_link = Transform(
            translation=calibration.camera_link.translation,
            rotation=calibration.camera_link.rotation,
            frame_id=calibration.camera_link.frame_id,
            child_frame_id=calibration.camera_link.child_frame_id,
            ts=timestamp,
        )
        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=OPTICAL_ROTATION,
            frame_id=camera_link.child_frame_id,
            child_frame_id=camera_info.frame_id,
            ts=timestamp,
        )
        self.tf.publish(TFMessage(camera_link, camera_optical))

    def _has_valid_calibration(self) -> bool:
        calibration = self._calibration
        if calibration is None:
            return False
        camera_matrix = calibration.camera_info.K
        return len(camera_matrix) >= 5 and camera_matrix[0] > 0.0 and camera_matrix[4] > 0.0

    def _release_pipeline(self) -> None:
        pipeline = self._pipeline
        self._pipeline = None
        if pipeline is not None:
            pipeline.stop()
