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

import threading
from typing import Any
from unittest.mock import MagicMock
import uuid

import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.core.transport import LCMTransport
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_detection_stream_module import (
    MarkerDetectionStreamModule,
    deploy_marker_detection,
)
from dimos.perception.fiducial.marker_transformer import MarkersPerFrame
from dimos.perception.fiducial.test_helpers import (
    blank_image,
    camera_info,
    synthetic_marker_image,
)


class CameraInfoSource:
    def __init__(self) -> None:
        self._callbacks: list[Any] = []

    def subscribe(self, callback: Any) -> Any:
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            self._callbacks.remove(callback)

        return unsubscribe

    def publish(self, info: CameraInfo) -> None:
        for callback in list(self._callbacks):
            callback(info)


def _reset_thread_pool() -> None:
    import reactivex.scheduler

    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = reactivex.scheduler.ThreadPoolScheduler(max_workers=tp.get_max_workers())


def _marker(image: Image, marker_id: int) -> Detection3DMarker:
    return Detection3DMarker(
        bbox=(10.0 + marker_id, 20.0, 40.0 + marker_id, 50.0),
        track_id=-1,
        class_id=marker_id,
        confidence=1.0,
        name="",
        ts=image.ts,
        image=image,
        center=Vector3(float(marker_id), 2.0, 3.0),
        size=Vector3(0.18, 0.18, 0.0),
        transform=Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id="camera_optical",
            ts=image.ts,
        ),
        frame_id="world",
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        marker_id=marker_id,
        corners_px=np.array(
            [[10.0, 20.0], [40.0, 20.0], [40.0, 50.0], [10.0, 50.0]],
            dtype=np.float32,
        ),
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=0.01,
    )


def _marker_obs(
    image: Image,
    marker: Detection3DMarker | None,
    *,
    obs_id: int,
    marker_count: int,
    marker_index: int = 0,
) -> Observation[Detection3DMarker | None]:
    tags: dict[str, Any] = {
        "marker_frame_image": image,
        "marker_frame_count": marker_count,
    }
    if marker is not None:
        tags["marker_frame_index"] = marker_index
    return Observation(
        id=obs_id,
        ts=image.ts,
        data_type=Detection3DMarker if marker is not None else type(None),
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        tags=tags,
        _data=marker,
    )


def test_marker_detection_stream_module_exposes_single_stream_input() -> None:
    module = MarkerDetectionStreamModule(marker_length_m=0.18, camera_info=camera_info())
    try:
        assert set(module.inputs) == {"color_image"}
        assert set(module.outputs) == {"detections"}
    finally:
        module.stop()


def test_deploy_passescamera_info_source_via_config_and_wires_only_image_input() -> None:
    dimos = MagicMock()
    proxy = MagicMock()
    dimos.deploy.return_value = proxy
    camera = MagicMock()
    camera.color_image = MagicMock()
    camera.camera_info = MagicMock()

    result = deploy_marker_detection(dimos, camera, marker_length_m=0.18)

    assert result is proxy
    dimos.deploy.assert_called_once_with(
        MarkerDetectionStreamModule,
        marker_length_m=0.18,
        camera_info_source=camera.camera_info,
    )
    proxy.color_image.connect.assert_called_once_with(camera.color_image)
    assert not hasattr(proxy, "camera_info") or not proxy.camera_info.connect.called
    assert proxy.detections.transport.topic.topic == "/marker_detection/detections"
    proxy.start.assert_called_once()


def test_markers_per_frame_groups_markers_and_preserves_empty_frames() -> None:
    image = blank_image(ts=10.0)
    empty_image = blank_image(ts=11.0)
    marker_a = _marker(image, 7)
    marker_b = _marker(image, 42)

    outputs = list(
        MarkersPerFrame(frame_id="world")(
            iter(
                [
                    _marker_obs(image, marker_a, obs_id=1, marker_count=2, marker_index=0),
                    _marker_obs(image, marker_b, obs_id=2, marker_count=2, marker_index=1),
                    _marker_obs(empty_image, None, obs_id=3, marker_count=0),
                ]
            )
        )
    )

    assert len(outputs) == 2
    first = outputs[0].data
    assert first.header.frame_id == "world"
    assert first.ts == pytest.approx(image.ts)
    assert first.detections_length == 2
    assert [det.id for det in first.detections] == ["7", "42"]
    assert outputs[0].pose_tuple == pytest.approx((1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))

    empty = outputs[1].data
    assert empty.header.frame_id == "world"
    assert empty.ts == pytest.approx(empty_image.ts)
    assert empty.detections_length == 0
    assert empty.detections == []


def test_marker_detection_stream_pipeline_outputs_arrays_for_marker_and_empty_frame() -> None:
    marker_id = 7
    marker_length_m = 0.18
    marker_image = synthetic_marker_image(marker_id, ts=10.0)
    empty_image = blank_image(ts=11.0)

    module = MarkerDetectionStreamModule(
        marker_length_m=marker_length_m,
        camera_info=camera_info(marker_image.ts),
        quality_window_s=0.01,
    )
    try:
        with MemoryStore() as store:
            stream = store.stream("color_image", Image)
            stream.append(
                marker_image,
                ts=marker_image.ts,
                pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )
            stream.append(
                empty_image,
                ts=empty_image.ts,
                pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )

            outputs = [obs.data for obs in module.pipeline(stream).to_list()]
    finally:
        module.stop()

    assert len(outputs) == 2
    assert outputs[0].detections_length == 1
    assert outputs[0].detections[0].id == str(marker_id)
    assert outputs[0].detections[0].results[0].hypothesis.class_id == (
        f"DICT_APRILTAG_36h11:{marker_id}"
    )
    assert outputs[0].detections[0].bbox.size.x == pytest.approx(marker_length_m)

    assert outputs[1].ts == pytest.approx(empty_image.ts)
    assert outputs[1].detections_length == 0
    assert outputs[1].detections == []


def test_marker_detection_stream_pipeline_speed_limit_is_config_gated() -> None:
    info = camera_info()
    images = [
        blank_image(ts=10.0),
        blank_image(ts=11.0),
        blank_image(ts=12.0),
    ]
    poses = [
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (100.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]

    def run_pipeline(*, speed_limit_enabled: bool) -> list[Detection3DArray]:
        module = MarkerDetectionStreamModule(
            marker_length_m=0.18,
            camera_info=info,
            quality_window_s=0.01,
            speed_limit_enabled=speed_limit_enabled,
            speed_limit_max_mps=0.05,
        )
        try:
            with MemoryStore() as store:
                stream = store.stream("color_image", Image)
                for image, pose in zip(images, poses, strict=True):
                    stream.append(image, ts=image.ts, pose=pose)
                return [obs.data for obs in module.pipeline(stream).to_list()]
        finally:
            module.stop()

    disabled = run_pipeline(speed_limit_enabled=False)
    enabled = run_pipeline(speed_limit_enabled=True)

    assert [msg.ts for msg in disabled] == pytest.approx([10.0, 11.0, 12.0])
    assert all(msg.detections_length == 0 for msg in disabled)
    assert [msg.ts for msg in enabled] == pytest.approx([12.0])
    assert enabled[0].detections_length == 0


def test_append_image_with_pose_uses_camera_optical_tf_without_recomputing_pose() -> None:
    image = blank_image(ts=12.0)
    info = camera_info(image.ts)
    t_world_optical = Transform(
        translation=Vector3(4.0, 5.0, 6.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=image.ts,
    )

    class FakeTf:
        def __init__(self) -> None:
            self.calls: list[tuple[str, str, float | None, float | None]] = []

        def get(
            self,
            parent_frame: str,
            child_frame: str,
            time_point: float | None = None,
            time_tolerance: float | None = None,
        ) -> Transform:
            self.calls.append((parent_frame, child_frame, time_point, time_tolerance))
            return t_world_optical

        def stop(self) -> None:
            pass

    module = MarkerDetectionStreamModule(
        marker_length_m=0.18,
        camera_info=info,
        tf_lookup_tolerance=0.25,
    )
    fake_tf = FakeTf()
    module._tf = fake_tf
    try:
        with MemoryStore() as store:
            stream = store.stream("color_image", Image)

            module._append_image_with_pose(stream, image)

            observations = list(stream)
    finally:
        module.stop()

    assert fake_tf.calls == [("world", "camera_optical", image.ts, 0.25)]
    assert len(observations) == 1
    assert observations[0].data is image
    assert observations[0].ts == pytest.approx(image.ts)
    assert observations[0].pose_tuple == pytest.approx((4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0))


def test_append_image_with_pose_skips_withoutcamera_info_or_tf() -> None:
    image = blank_image(ts=13.0)

    module = MarkerDetectionStreamModule(marker_length_m=0.18)
    try:
        with MemoryStore() as store:
            stream = store.stream("color_image", Image)
            module._append_image_with_pose(stream, image)
            assert list(stream) == []
    finally:
        module.stop()

    class MissingTf:
        def __init__(self) -> None:
            self.calls = 0

        def get(self, *args: Any, **kwargs: Any) -> None:
            self.calls += 1
            return None

        def stop(self) -> None:
            pass

    missing_tf = MissingTf()
    module = MarkerDetectionStreamModule(marker_length_m=0.18, camera_info=camera_info(image.ts))
    module._tf = missing_tf
    try:
        with MemoryStore() as store:
            stream = store.stream("color_image", Image)
            module._append_image_with_pose(stream, image)
            assert list(stream) == []
    finally:
        module.stop()

    assert missing_tf.calls == 1


def test_marker_detection_stream_module_start_publishes_detection_array_over_lcm() -> None:
    marker_id = 7
    source = CameraInfoSource()
    module = MarkerDetectionStreamModule(
        marker_length_m=0.18,
        camera_info_source=source,
        quality_window_s=0.01,
    )
    suffix = uuid.uuid4().hex[:8]
    module.color_image.transport = LCMTransport(f"/tmd/{suffix}/image", Image)
    module.detections.transport = LCMTransport(
        f"/tmd/{suffix}/detections",
        Detection3DArray,
    )

    module._tf = type(
        "FakeTf",
        (),
        {
            "get": lambda self, *args, **kwargs: Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="camera_optical",
                ts=10.0,
            ),
            "stop": lambda self: None,
        },
    )()

    received: list[Detection3DArray] = []
    done = threading.Event()

    def on_detection(msg: Detection3DArray) -> None:
        received.append(msg)
        if len(received) >= 2:
            done.set()

    unsub = module.detections.transport.subscribe(on_detection)

    module.start()
    try:
        source.publish(camera_info(ts=10.0))
        module.color_image.transport.publish(synthetic_marker_image(marker_id, ts=10.0))
        module.color_image.transport.publish(blank_image(ts=11.0))
        module.color_image.transport.publish(blank_image(ts=12.0))

        assert done.wait(timeout=5.0), f"Timed out waiting for marker detections, got {received}"
        marker_msg = received[0]
        assert marker_msg.detections_length == 1
        assert marker_msg.detections[0].id == str(marker_id)
        assert marker_msg.detections[0].results[0].hypothesis.class_id == (
            f"DICT_APRILTAG_36h11:{marker_id}"
        )
        empty_msg = received[1]
        assert empty_msg.ts == pytest.approx(11.0)
        assert empty_msg.detections_length == 0
        assert empty_msg.detections == []
    finally:
        unsub()
        module.stop()
        _reset_thread_pool()
