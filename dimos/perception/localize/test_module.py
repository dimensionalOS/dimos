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

from collections.abc import Iterator
from dataclasses import replace
import threading
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize import Groups
from dimos.perception.localize.module import LiveLocalizeModule, as_detection_array
from dimos.perception.localize.types import Localization, LocalizePolicy


def image(ts: float, frame: str = "wrist") -> Image:
    return Image(np.full((8, 8, 3), 128, dtype=np.uint8), ImageFormat.RGB, ts=ts, frame_id=frame)


def localization(ts: float = 99.0) -> Localization:
    return Localization(
        instance_id="query-0",
        semantic_score=0.9,
        identity_score=0.5,
        ambiguity_margin=1.0,
        position_world_xyz=(0.1, 0.2, 0.3),
        orientation_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        support=None,
        pose_timestamp=ts,
        geometry_timestamp=ts,
        last_seen_timestamp=ts,
        point_cloud=PointCloud2.from_numpy(
            np.array([[0.1, 0.2, 0.3], [0.15, 0.25, 0.35]]), frame_id="world", timestamp=ts
        ),
        coverage=0.5,
        n_views=2,
    )


@pytest.fixture
def module() -> Iterator[LiveLocalizeModule]:
    instance = LiveLocalizeModule(optical_frame="wrist")
    with MemoryStore() as store:
        instance._memory = store
        instance.index = store.stream("index", Image)
        instance._color_feed = store.stream("color", Image)
        instance._depth_feed = store.stream("depth", Image)
        instance._depth_memory = store.stream("depth_memory", Image)
        instance._color_feed.append(image(105.0), ts=105.0)
        for ts in (80.0, 90.0, 100.0):
            instance.index.append(image(ts), ts=ts)
        instance.detector = MagicMock()
        instance.rig = SimpleNamespace(world_frame="world", default_localize_policy=LocalizePolicy)
        instance.detections = MagicMock()
        instance.hit_points = MagicMock()
        instance._ready.set()
        try:
            yield instance
        finally:
            instance.stop()


def test_batched_multi_instance_results_and_memory_are_preserved(
    module: LiveLocalizeModule,
) -> None:
    old, recent = localization(80.0), localization(99.0)
    ambiguous = replace(recent, reason="ambiguous_between_coexisting_candidates")
    module.detector.localize.return_value = [[old, recent], [ambiguous], []]
    group = Groups()
    module._groups["cup"] = group

    results = module.localize_objects(["cup", "bowl", "fork"])

    assert results == [[old, recent], [ambiguous], []]
    call = module.detector.localize.call_args
    assert call.kwargs["groups"]["cup"] is group
    assert call.kwargs["rig"] is module.rig
    assert [obs.ts for obs in call.kwargs["index"]] == [90.0, 100.0]
    assert call.kwargs["require_pose"] is True
    assert results[0][0].point_cloud is old.point_cloud


def test_age_filter_is_opt_in_and_does_not_erase_memory(module: LiveLocalizeModule) -> None:
    old, recent = localization(80.0), localization(103.0)
    module.detector.localize.return_value = [[old, recent]]

    assert module.localize_objects(["cup"], max_age=5.0) == [[recent]]
    assert module.localize_objects(["cup"]) == [[old, recent]]


def test_window_and_policy_overrides(module: LiveLocalizeModule) -> None:
    module.config.policy = {"candidate_floor": 0.07}
    module.detector.localize.return_value = [[]]

    module.localize_objects(["cup"], start=0.0, duration=5.0, policy='{"min_views": 1}')

    call = module.detector.localize.call_args
    assert [obs.ts for obs in call.kwargs["index"]] == [80.0]
    assert call.kwargs["policy"].min_views == 1
    assert call.kwargs["policy"].candidate_floor == 0.07


@pytest.mark.parametrize("prompts", [[], [""], [" cup"], ["cup", "cup"]])
def test_invalid_prompts(module: LiveLocalizeModule, prompts: list[str]) -> None:
    with pytest.raises(ValueError):
        module.localize_objects(prompts)
    module.detector.localize.assert_not_called()


@pytest.mark.parametrize("policy", ["[]", '{"unknown": 1}', "bad json"])
def test_invalid_policy(module: LiveLocalizeModule, policy: str) -> None:
    with pytest.raises(ValueError):
        module.localize_objects(["cup"], policy=policy)


def test_not_ready(module: LiveLocalizeModule) -> None:
    module._ready.clear()
    module._stage = "waiting for camera_info"
    assert "camera_info" in module.state()
    with pytest.raises(RuntimeError, match="camera_info"):
        module.localize_objects(["cup"])


@pytest.mark.parametrize("wrong_cloud", [False, True])
def test_frame_mismatch(module: LiveLocalizeModule, wrong_cloud: bool) -> None:
    hit = localization()
    if wrong_cloud:
        hit.point_cloud.frame_id = "camera"
    else:
        hit.frame_id = "camera"
    module.detector.localize.return_value = [[hit]]
    with pytest.raises(RuntimeError, match="frame mismatch"):
        module.localize_objects(["cup"])


def test_calibration_selects_wrist_and_depth_pair_survives_embedding(
    module: LiveLocalizeModule,
) -> None:
    env = CameraInfo.from_intrinsics(8, 8, 10.0, 10.0, 4.0, 4.0, frame_id="environment")
    wrist = CameraInfo.from_intrinsics(8, 8, 10.0, 10.0, 4.0, 4.0, frame_id="wrist")
    module._on_camera_info(env)
    assert not module._camera_seen.is_set()
    module._on_camera_info(wrist)
    assert module._camera_seen.is_set()

    color = image(110.0)
    depth = Image(np.ones((8, 8), dtype=np.float32), ImageFormat.DEPTH, ts=110.01, frame_id="wrist")
    module._on_frames((image(109.0, "environment"), depth))
    assert module._depth_feed.count() == 0
    module._on_frames((color, depth))
    observation = module._color_feed.last()
    list(module._pair_depth(iter([observation])))

    assert module._depth_memory.last().ts == 110.0
    assert module._depth_memory.last().data is depth


def test_visualization_ids_are_unique_across_prompts() -> None:
    hit = localization()
    msg = as_detection_array(["cup", "bowl"], [[hit, hit], [hit]], "world")
    assert len({d.id for d in msg.detections}) == 3


def test_stop_without_start() -> None:
    module = LiveLocalizeModule()
    module.stop()
    assert module.state() == "not ready: stopped"


def test_live_wrist_memory_keeps_capture_time_poses_and_paired_depth(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = LiveLocalizeModule(optical_frame="wrist")
    scheduler = ThreadPoolScheduler(max_workers=3)
    monkeypatch.setattr("dimos.utils.threadpool.scheduler", scheduler)
    detector = MagicMock()
    subjects = {name: Subject() for name in ("color_image", "depth_image", "camera_info", "tf")}
    for name, subject in subjects.items():
        port = MagicMock()
        port.observable.return_value = subject
        port.subscribe.side_effect = lambda cb, s=subject: s.subscribe(cb).dispose
        setattr(module, name, port)

    def embed(store: Any, *, rig: Any, source: Any) -> Any:
        # Exercise real streams/codecs/TF; only the neural embedding is replaced.
        index = store.stream("color_image_embedded", Image)
        pipeline = (
            source.live()
            .map(lambda obs: obs.derive(data=obs.data, pose=rig.index_pose(obs)))
            .filter(lambda obs: obs.pose is not None)
            .save(index)
        )
        module._workers.add(pipeline.drain_thread())
        return index

    detector.embed_live.side_effect = embed
    monkeypatch.setattr("dimos.perception.localize.module.DanDetector", lambda: detector)
    try:
        module.start()
        subjects["camera_info"].on_next(
            CameraInfo.from_intrinsics(8, 8, 10.0, 10.0, 4.0, 4.0, frame_id="wrist")
        )
        subjects["tf"].on_next(
            TFMessage(
                Transform(ts=10.0, frame_id="world", child_frame_id="wrist"),
                Transform(
                    ts=11.0,
                    frame_id="world",
                    child_frame_id="wrist",
                    translation=Vector3(0.2, 0, 0),
                ),
            )
        )
        depth = np.ones((8, 8), dtype=np.float32)
        subjects["color_image"].on_next(image(10.0))
        subjects["depth_image"].on_next(Image(depth, ImageFormat.DEPTH, ts=10.0, frame_id="wrist"))
        assert module._ready.wait(5.0), module.state()

        paired = threading.Event()
        subscription = module._depth_memory.live().subscribe(
            lambda obs: paired.set() if obs.ts == 11.0 else None
        )
        module._workers.add(subscription)
        subjects["color_image"].on_next(image(11.0))
        subjects["depth_image"].on_next(
            Image(depth * 2, ImageFormat.DEPTH, ts=11.0, frame_id="wrist")
        )
        assert paired.wait(5.0)

        first, second = module.index.to_list()
        assert first.pose_stamped.position.x == pytest.approx(0.0)
        assert second.pose_stamped.position.x == pytest.approx(0.2)
        assert module.rig.camera_pose(10.0).position.x == pytest.approx(0.0)
        np.testing.assert_allclose(module.rig.depth_at(10.0).data, depth)
        np.testing.assert_allclose(module.rig.depth_at(11.0).data, depth * 2)
    finally:
        module.stop()
        for subject in subjects.values():
            subject.dispose()
        scheduler.executor.shutdown(wait=True)
    detector.dispose.assert_called_once()


def test_detector_can_stop_before_models_are_initialized() -> None:
    detector = DanDetector()
    detector.stop()
    detector.stop()
