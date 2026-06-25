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

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

pytest.importorskip("cv2.aruco")

from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.tick import Interpolate, Tick
from dimos.memory2.transform import QualityWindow
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.marker_detection_pure_module import MarkerDetectionPureModule
from dimos.perception.fiducial.test_helpers import blank_image, camera_info, synthetic_marker_image

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.stream import Stream

MARKER_LENGTH_M = 0.18


def identity_pose(ts: float) -> PoseStamped:
    return PoseStamped(ts=ts, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0))


@pytest.fixture
def store() -> Iterator[MemoryStore]:
    with MemoryStore() as s:
        yield s


def fill_poses(
    store: MemoryStore, start: float, stop: float, hz: float = 25.0
) -> Stream[PoseStamped]:
    poses = store.stream("camera_pose", PoseStamped)
    t = start
    while t <= stop:
        poses.append(identity_pose(t), ts=t)
        t += 1.0 / hz
    return poses


def module() -> MarkerDetectionPureModule:
    return MarkerDetectionPureModule.offline(
        marker_length_m=MARKER_LENGTH_M, camera_info=camera_info()
    )


def test_plan_declares_tick_and_interpolated_pose() -> None:
    plan = MarkerDetectionPureModule._plan()
    assert plan.trigger == "color_image"
    assert isinstance(plan.samplers["camera_pose"], Interpolate)
    assert plan.samplers["camera_pose"].tolerance == 0.5  # parity: tf_lookup_tolerance
    assert set(plan.outs) == {"detections"}
    assert not isinstance(plan.samplers.get("color_image"), Tick)  # trigger isn't sampled


def test_detects_marker_and_emits_empty_frames(store: MemoryStore) -> None:
    images = store.stream("color_image", Image)
    marker_image = synthetic_marker_image(7, ts=10.0)
    images.append(marker_image, ts=10.0)
    images.append(blank_image(ts=11.0), ts=11.0)
    poses = fill_poses(store, 9.9, 11.1)

    out = [o.data for o in module().over(color_image=images, camera_pose=poses).to_list()]

    assert len(out) == 2  # one array per frame, empty frames included
    assert out[0].detections_length == 1
    assert out[0].detections[0].id == "7"
    assert out[0].detections[0].results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"
    assert out[0].detections[0].bbox.size.x == pytest.approx(MARKER_LENGTH_M)
    assert out[0].header.frame_id == "world"
    assert out[1].detections_length == 0
    assert out[1].detections == []


def test_quality_gating_composes_upstream(store: MemoryStore) -> None:
    """The stream module's QualityWindow config knob becomes stream composition."""
    images = store.stream("color_image", Image)
    images.append(blank_image(ts=10.0), ts=10.0)  # featureless -> low sharpness
    images.append(synthetic_marker_image(7, ts=10.5), ts=10.5)  # edges -> sharp
    poses = fill_poses(store, 9.9, 11.0)

    gated: Stream[Image] = images.transform(QualityWindow(lambda img: img.sharpness, window=2.0))
    out = module().over(color_image=gated, camera_pose=poses).to_list()

    assert len(out) == 1  # only the best frame in the window ticked
    assert out[0].data.detections_length == 1


def test_frame_without_camera_pose_is_dropped(store: MemoryStore) -> None:
    images = store.stream("color_image", Image)
    images.append(synthetic_marker_image(7, ts=10.0), ts=10.0)
    images.append(blank_image(ts=20.0), ts=20.0)  # far outside pose coverage
    poses = fill_poses(store, 9.9, 10.1)

    out = module().over(color_image=images, camera_pose=poses).to_list()

    assert [o.ts for o in out] == [10.0]  # unposed frame dropped, not mislocated

    with pytest.raises(ValueError, match="missing required inputs"):
        module().over(_strict=True, color_image=images, camera_pose=poses).to_list()


def test_without_camera_info_emits_nothing(store: MemoryStore) -> None:
    images = store.stream("color_image", Image)
    images.append(synthetic_marker_image(7, ts=10.0), ts=10.0)
    poses = fill_poses(store, 9.9, 10.1)

    bare = MarkerDetectionPureModule.offline(marker_length_m=MARKER_LENGTH_M)
    assert bare.over(color_image=images, camera_pose=poses).to_list() == []


def test_step_is_directly_callable() -> None:
    """The pure core needs no streams, store, or ports at all."""
    m = module()
    result = m.step(synthetic_marker_image(7, ts=10.0), identity_pose(10.0), ts=10.0)
    assert result is not None
    assert result.detections_length == 1
    assert result.detections[0].id == "7"
