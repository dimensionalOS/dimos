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

import json
import time
from dataclasses import replace

import numpy as np
import pytest
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from microduck_world.knowledge import CameraView, DuckKnowledge, locate_pixel
from microduck_world.robot_io import Observation


@pytest.fixture
def observation():
    return Observation(
        Image(data=np.zeros((3, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=time.time()),
        Image(data=np.full((3, 3), 2.0, dtype=np.float32), format=ImageFormat.DEPTH),
        CameraInfo.from_intrinsics(2, 2, 1, 1, 3, 3),
        PoseStamped(position=(3, 4, 5), orientation=(0, 0, 0, 1), frame_id="world"),
    )


def test_object_position_uses_pixel_depth_and_own_camera_pose(observation):
    assert locate_pixel(observation, 2, 1) == (4, 4, 7)


@pytest.mark.parametrize("x,y", [(-1, 0), (0, 3), (3, 0)])
def test_annotation_rejects_pixels_outside_own_image(observation, x, y):
    with pytest.raises(ValueError, match="outside"):
        locate_pixel(observation, x, y)


@pytest.mark.parametrize("depth", [float("nan"), float("inf"), 0, 7])
def test_annotation_requires_measured_depth(observation, depth):
    observation.depth.data[1, 1] = depth
    with pytest.raises(ValueError, match="measured depth"):
        locate_pixel(observation, 1, 1)


@pytest.fixture
def knowledge(module_factory, tmp_path):
    def create(generation):
        directory = tmp_path / generation
        directory.mkdir(exist_ok=True)
        return module_factory(
            DuckKnowledge,
            rooms={},
            objects={},
            scene=generation,
            places_db=str(directory / "places.db"),
            knowledge_dir=str(directory),
            places_republish_s=0,
        )

    return create


def test_evidence_and_named_objects_stay_private_and_new_visitors_start_fresh(
    knowledge, observation
):
    first, other = knowledge("first"), knowledge("other")
    first._on_observation(observation)
    view = first.observe()
    assert isinstance(view, CameraView)
    assert "Unknown observation" in other.remember_object("box", view.id, 1, 1)
    assert "Remembered box" in first.remember_object("box", view.id, 1, 1, "a visible box")
    assert "box" in first.list_objects()
    assert "box" not in other.list_objects()
    first.stop()
    resumed = knowledge("first")
    assert "box" in resumed.list_objects()
    fresh = knowledge("new-visitor")
    assert "box" not in fresh.list_objects()
    assert json.loads(fresh.understanding())["observations"] == []


def test_only_images_returned_by_observe_can_be_annotated(knowledge, observation):
    duck = knowledge("duck")
    duck._on_observation(observation)
    assert "Unknown observation" in duck.record_observation(str(observation.image.ts), "a box")
    view = duck.observe()
    assert isinstance(view, CameraView)
    assert (
        duck.record_observation(view.id, "a box") == "Saved this observation in my private memory."
    )
    assert json.loads(duck.understanding())["observations"][0]["description"] == "a box"


def test_stale_camera_is_not_presented_as_current(knowledge, observation):
    duck = knowledge("duck")
    stale = replace(
        observation,
        image=Image(data=observation.image.data, format=ImageFormat.RGB, ts=time.time() - 10),
    )
    duck._on_observation(stale)
    assert not duck.observe().success
