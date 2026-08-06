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

from collections.abc import Callable

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.drone.px4.gstreamer.fake_gstreamer import FakeGstApi
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstBusEvent,
    GstPipelineError,
    GstSource,
    GstSourceConfig,
    H264AccessUnit,
    RawBgrSample,
)
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import (
    CameraCalibration,
    CameraLifecycle,
    Px4GstTeeCamera,
)


def test_worker_style_constructor_forwards_global_config(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: a worker-provided GlobalConfig and a spy on Module construction.
    worker_config = GlobalConfig()
    observed: list[GlobalConfig] = []
    original_init = Module.__init__

    def spy(self: Module, *, frame_id: str, g: GlobalConfig) -> None:
        observed.append(g)
        original_init(self, frame_id=frame_id, g=g)

    monkeypatch.setattr(Module, "__init__", spy)

    # When: the PX4 camera receives the worker-style constructor argument.
    camera = Px4GstTeeCamera(g=worker_config)

    # Then: the precise GlobalConfig object reaches the Module base constructor.
    try:
        assert observed == [worker_config]
    finally:
        camera._close_module()


def test_callbacks_publish_bgr_and_h264_access_units_in_source_callback_order(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    # Given: a running camera with local output observers.
    gst, camera = make_camera(GstSource.VIDEOTEST, None)
    images: list[Image] = []
    video_packets: list[CompressedVideo] = []
    _ = camera.color_image.subscribe(images.append)
    _ = camera.video_h264.subscribe(video_packets.append)
    camera.start()

    # When: the two tee branches deliver one BGR sample and ordered access units.
    gst.emit_raw(
        RawBgrSample(data=b"\x01\x02\x03\x04\x05\x06", width=2, height=1, pts_ns=1_500_000_000)
    )
    gst.emit_h264(
        H264AccessUnit(data=b"\x00\x00\x00\x01\x65", pts_ns=2_000_000_000, delta_unit=False)
    )
    gst.emit_h264(
        H264AccessUnit(data=b"\x00\x00\x00\x01\x41", pts_ns=2_100_000_000, delta_unit=True)
    )

    # Then: raw data is BGR, H.264 bytes retain their exact callback order, and keyframes are counted.
    assert images[0].format is ImageFormat.BGR
    assert images[0].data.tolist() == [[[1, 2, 3], [4, 5, 6]]]
    assert images[0].ts == 1.5
    assert [bytes(packet.data) for packet in video_packets] == [
        b"\x00\x00\x00\x01\x65",
        b"\x00\x00\x00\x01\x41",
    ]
    assert [packet.ts for packet in video_packets] == [2.0, 2.1]
    assert camera.keyframe_count == 1


def test_no_calibration_publishes_no_camera_metadata(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    # Given: a camera without a calibrated optical model.
    _, camera = make_camera(GstSource.VIDEOTEST, None)
    camera_infos: list[CameraInfo] = []
    transforms: list[TFMessage] = []
    _ = camera.camera_info.subscribe(camera_infos.append)
    _ = camera.tf.subscribe(transforms.append)

    # When: the camera starts.
    camera.start()

    # Then: it does not fabricate intrinsics or frame metadata.
    assert camera_infos == []
    assert transforms == []


def test_valid_calibration_publishes_camera_info_and_optical_frame_tf(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    # Given: a calibrated camera-link transform and optical-frame intrinsics.
    calibration = CameraCalibration(
        camera_info=CameraInfo.from_intrinsics(
            fx=100.0,
            fy=120.0,
            cx=50.0,
            cy=60.0,
            width=128,
            height=96,
            frame_id="camera_optical",
        ),
        camera_link=Transform(
            translation=Vector3(0.1, 0.0, -0.05),
            frame_id="base_link",
            child_frame_id="camera_link",
        ),
    )
    _, camera = make_camera(GstSource.VIDEOTEST, calibration)
    camera_infos: list[CameraInfo] = []
    transforms: list[TFMessage] = []
    _ = camera.camera_info.subscribe(camera_infos.append)
    _ = camera.tf.subscribe(transforms.append)

    # When: the calibrated camera starts.
    camera.start()

    # Then: exact configured intrinsics and CameraModule-equivalent optical TF are emitted.
    assert camera_infos[0].K == [100.0, 0.0, 50.0, 0.0, 120.0, 60.0, 0.0, 0.0, 1.0]
    assert camera_infos[0].ts == 42.0
    assert [(transform.frame_id, transform.child_frame_id) for transform in transforms[0]] == [
        ("base_link", "camera_link"),
        ("camera_link", "camera_optical"),
    ]
    assert transforms[0][1].rotation == OPTICAL_ROTATION
    assert [transform.ts for transform in transforms[0]] == [42.0, 42.0]


def test_valid_calibration_publishes_metadata_periodically(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Interval:
        class Subscription:
            def dispose(self) -> None:
                return None

        def __init__(self) -> None:
            self.callback: Callable[[int], None] | None = None

        def subscribe(self, callback: Callable[[int], None]) -> Subscription:
            self.callback = callback
            return self.Subscription()

    interval = Interval()
    monkeypatch.setattr(
        "dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera.rx.interval",
        lambda seconds: interval,
    )
    calibration = CameraCalibration(
        camera_info=CameraInfo.from_intrinsics(
            fx=100.0,
            fy=120.0,
            cx=50.0,
            cy=60.0,
            width=128,
            height=96,
            frame_id="camera_optical",
        ),
        camera_link=Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            frame_id="base_link",
            child_frame_id="camera_link",
        ),
    )
    _, camera = make_camera(GstSource.VIDEOTEST, calibration)
    camera_infos: list[CameraInfo] = []
    _ = camera.camera_info.subscribe(camera_infos.append)

    camera.start()
    assert interval.callback is not None
    interval.callback(0)

    assert len(camera_infos) == 2


def test_zero_focal_calibration_publishes_no_camera_metadata(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    calibration = CameraCalibration(
        camera_info=CameraInfo.from_intrinsics(
            fx=0.0,
            fy=120.0,
            cx=50.0,
            cy=60.0,
            width=128,
            height=96,
            frame_id="camera_optical",
        ),
        camera_link=Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            frame_id="base_link",
            child_frame_id="camera_link",
        ),
    )
    _, camera = make_camera(GstSource.VIDEOTEST, calibration)
    camera_infos: list[CameraInfo] = []
    transforms: list[TFMessage] = []
    _ = camera.camera_info.subscribe(camera_infos.append)
    _ = camera.tf.subscribe(transforms.append)

    camera.start()

    assert camera_infos == []
    assert transforms == []


def test_bus_error_stops_the_owned_pipeline_deterministically(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    # Given: a running camera pipeline.
    gst, camera = make_camera(GstSource.VIDEOTEST, None)
    camera.start()

    # When: the pipeline reports a terminal bus event twice.
    gst.emit_bus(GstBusEvent.ERROR, "source terminated")
    gst.emit_bus(GstBusEvent.ERROR, "source terminated")

    # Then: cleanup occurs exactly once and exposes the terminal diagnostic.
    assert camera.lifecycle is CameraLifecycle.FAILED
    assert camera.last_bus_error == "source terminated"
    assert gst.stop_count == 1


def test_pipeline_start_failure_stops_the_created_pipeline() -> None:
    # Given: a Gst API whose pipeline cannot enter PLAYING.
    gst = FakeGstApi(start_error=GstPipelineError(detail="PLAYING failed"))
    camera = Px4GstTeeCamera(
        gst_api=gst,
        source_config=GstSourceConfig(
            source=GstSource.VIDEOTEST,
            device="/dev/video0",
            width=640,
            height=480,
            fps=30,
        ),
    )

    # When: startup requests the failed state transition.
    with pytest.raises(GstPipelineError, match="PLAYING failed"):
        camera.start()

    # Then: the module records failure and releases the pipeline deterministically.
    assert camera.lifecycle is CameraLifecycle.FAILED
    assert gst.stop_count == 1
    camera.stop()
