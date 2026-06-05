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

from collections.abc import Callable
import time
from typing import Any

import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.core.transport import pLCMTransport
from dimos.memory2.fanio import Bundle
from dimos.memory2.module import StreamModule
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial import marker_transformer
from dimos.perception.fiducial.marker_module import MarkerModule
from dimos.perception.fiducial.marker_transformer import MarkersToBundle
from dimos.perception.fiducial.test_helpers import (
    blank_image,
    camera_info,
    synthetic_marker_image,
)
from dimos.types.timestamped import to_timestamp


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


def test_marker_module_exposes_image_input_and_2d_3d_outputs() -> None:
    module = MarkerModule(marker_length_m=0.18, camera_info=camera_info())
    try:
        assert set(module.inputs) == {"color_image"}
        assert set(module.outputs) == {"detections_3d", "detections_2d"}
    finally:
        module.stop()


def test_markers_to_bundle_emits_parallel_2d_and_3d_arrays() -> None:
    image = blank_image(ts=10.0)
    empty_image = blank_image(ts=11.0)
    marker_a = _marker(image, 7)
    marker_b = _marker(image, 42)

    outputs = list(
        MarkersToBundle(frame_id="world")(
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
    assert all(isinstance(obs.data, Bundle) for obs in outputs)

    frame = outputs[0].data
    d3d = frame["detections_3d"]
    d2d = frame["detections_2d"]
    # One detection pass feeds both arrays: equal count and frame timestamp.
    assert d3d.detections_length == 2
    assert d2d.detections_length == 2
    assert d3d.ts == pytest.approx(image.ts)
    assert to_timestamp(d2d.header.stamp) == pytest.approx(image.ts)
    assert [det.id for det in d3d.detections] == ["7", "42"]

    # 2D overlays must carry the same marker identity as 3D. Markers have
    # track_id=-1, so without a 2D identity override every overlay labelled
    # "id=-1" with the hypothesis dropped (results_length stayed 0).
    assert [det.id for det in d2d.detections] == ["7", "42"]
    assert [det.results_length for det in d2d.detections] == [1, 1]
    assert [det.results[0].hypothesis.class_id for det in d2d.detections] == [
        "DICT_APRILTAG_36h11:7",
        "DICT_APRILTAG_36h11:42",
    ]
    # The image-plane overlay labels read identically to the 3D boxes.
    assert d2d.to_rerun().labels.as_arrow_array().to_pylist() == [
        "DICT_APRILTAG_36h11:7 id=7",
        "DICT_APRILTAG_36h11:42 id=42",
    ]

    empty = outputs[1].data
    # Empty frame still publishes both arrays (empty-but-present, not idle).
    assert empty["detections_3d"].detections_length == 0
    assert empty["detections_2d"].detections_length == 0
    assert empty["detections_3d"].ts == pytest.approx(empty_image.ts)
    assert to_timestamp(empty["detections_2d"].header.stamp) == pytest.approx(empty_image.ts)


def test_marker_module_pipeline_outputs_arrays_for_marker_and_empty_frame() -> None:
    marker_id = 7
    marker_length_m = 0.18
    marker_image = synthetic_marker_image(marker_id, ts=10.0)
    empty_image = blank_image(ts=11.0)

    module = MarkerModule(
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

            bundles = [obs.data for obs in module.pipeline(stream).to_list()]
    finally:
        module.stop()

    assert len(bundles) == 2
    d3d = [bundle["detections_3d"] for bundle in bundles]
    d2d = [bundle["detections_2d"] for bundle in bundles]

    assert d3d[0].detections_length == 1
    assert d3d[0].detections[0].id == str(marker_id)
    assert d3d[0].detections[0].results[0].hypothesis.class_id == (
        f"DICT_APRILTAG_36h11:{marker_id}"
    )
    assert d3d[0].detections[0].bbox.size.x == pytest.approx(marker_length_m)

    # 2D output is the same detection pass repackaged for image-plane overlays,
    # so its identity and overlay label match the 3D box (not the track_id=-1
    # default that the inherited Detection2DBBox.to_ros_detection2d would emit).
    assert d2d[0].detections_length == d3d[0].detections_length
    assert to_timestamp(d2d[0].header.stamp) == pytest.approx(d3d[0].ts)
    assert d2d[0].detections[0].id == str(marker_id)
    assert d2d[0].detections[0].results_length == 1
    assert d2d[0].detections[0].results[0].hypothesis.class_id == (
        f"DICT_APRILTAG_36h11:{marker_id}"
    )
    assert d2d[0].to_rerun().labels.as_arrow_array().to_pylist() == [
        f"DICT_APRILTAG_36h11:{marker_id} id={marker_id}"
    ]
    # Empty frame clears the overlay (empty Boxes2D), not a stale box.
    assert d2d[1].to_rerun().labels.as_arrow_array().to_pylist() == []

    assert d3d[1].ts == pytest.approx(empty_image.ts)
    assert d3d[1].detections_length == 0
    assert d3d[1].detections == []
    assert d2d[1].detections_length == 0


def test_marker_module_pipeline_speed_limit_is_config_gated() -> None:
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
        module = MarkerModule(
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
                return [obs.data["detections_3d"] for obs in module.pipeline(stream).to_list()]
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

    module = MarkerModule(
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

    module = MarkerModule(marker_length_m=0.18)
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
    module = MarkerModule(marker_length_m=0.18, camera_info=camera_info(image.ts))
    module._tf = missing_tf
    try:
        with MemoryStore() as store:
            stream = store.stream("color_image", Image)
            module._append_image_with_pose(stream, image)
            assert list(stream) == []
    finally:
        module.stop()

    assert missing_tf.calls == 1


class _IdentityTf:
    """TF stand-in that always resolves world->camera_optical as identity."""

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform:
        return Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=parent_frame,
            child_frame_id=child_frame,
            ts=time_point or 0.0,
        )

    def stop(self) -> None:
        pass


def _reset_thread_pool() -> None:
    """Shut down and replace the global RxPY thread pool so conftest thread-leak check passes."""
    from reactivex.scheduler import ThreadPoolScheduler

    import dimos.utils.threadpool as tp

    tp.scheduler.executor.shutdown(wait=True)
    tp.scheduler = ThreadPoolScheduler(max_workers=tp.get_max_workers())


def _wait_until(predicate: Callable[[], bool], timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


@pytest.mark.tool
def test_marker_module_inherited_start_feeds_both_ports_from_one_detector_pass(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The live module runs through the inherited fan-I/O ``start()`` (the old
    base required copying it to anchor pose on ingest): frames published on the
    In port come out as parallel 2D/3D arrays, and the detector runs once per
    processed frame - if scatter subscribed each Out separately, the lazy
    pipeline would re-run and double the detection passes."""
    # The 1->2 wiring below is the base class's: MarkerModule customizes only
    # ingest(), it does not carry a copied start() override.
    assert MarkerModule.start is StreamModule.start

    detect_calls: list[float] = []
    real_detect = marker_transformer._detect_markers_in_image

    def counting_detect(image: Image, **kwargs: Any) -> Any:
        detect_calls.append(image.ts)
        return real_detect(image, **kwargs)

    monkeypatch.setattr(marker_transformer, "_detect_markers_in_image", counting_detect)

    marker_id = 7
    frames = [
        synthetic_marker_image(marker_id, ts=10.0),
        blank_image(ts=11.0),
        # The quality window emits a frame when the next one arrives, so this
        # third frame exists only to flush the second; it stays buffered.
        blank_image(ts=12.0),
    ]

    module = MarkerModule(
        marker_length_m=0.18,
        camera_info=camera_info(10.0),
        quality_window_s=0.01,
    )
    module._tf = _IdentityTf()
    module.color_image.transport = pLCMTransport("/test/marker/color_image")
    module.detections_3d.transport = pLCMTransport("/test/marker/detections_3d")
    module.detections_2d.transport = pLCMTransport("/test/marker/detections_2d")

    received_3d: list[Detection3DArray] = []
    received_2d: list[Detection2DArray] = []
    unsub_3d = module.detections_3d.subscribe(received_3d.append)
    unsub_2d = module.detections_2d.subscribe(received_2d.append)

    module.start()
    try:
        for frame in frames:
            # Gaps keep ingest order == capture order over the loopback bus.
            module.color_image.transport.publish(frame)
            time.sleep(0.3)
        assert _wait_until(lambda: len(received_3d) >= 2 and len(received_2d) >= 2), (
            f"timed out: 3d={len(received_3d)} 2d={len(received_2d)}"
        )
        # Snapshot before stop(): closing the live stream flushes the buffered
        # third frame through the detector as part of the drain.
        processed = list(detect_calls)
    finally:
        unsub_3d()
        unsub_2d()
        module.stop()
        _reset_thread_pool()
        _reset_thread_pool()

    # One detection pass per processed frame (two flushed by the quality
    # window), not one per Out port.
    assert processed == [10.0, 11.0]

    # Both ports carry the same frames at the image capture times.
    assert [msg.ts for msg in received_3d] == pytest.approx([10.0, 11.0])
    assert [to_timestamp(msg.header.stamp) for msg in received_2d] == pytest.approx([10.0, 11.0])

    # Marker frame: the 2D overlay mirrors the 3D boxes from the same pass.
    assert received_3d[0].detections_length == 1
    assert received_2d[0].detections_length == 1
    assert received_3d[0].detections[0].id == str(marker_id)
    assert received_2d[0].detections[0].id == str(marker_id)

    # Empty frame still publishes empty-but-present arrays on both ports.
    assert received_3d[1].detections_length == 0
    assert received_2d[1].detections_length == 0
