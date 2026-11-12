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

import gc
import weakref

import pytest
import rerun as rr

from dimos.visualization.rerun import scene as scene_module
from dimos.visualization.rerun.scene import Scene


@pytest.fixture
def scene():
    value = Scene()
    yield value
    value.stop()


def refreshed(log):
    return {
        (
            call.args[0],
            call.kwargs["static"],
            str(batch.component_descriptor()),
        ): batch.as_arrow_array().to_pylist()
        for call in log.call_args_list
        for batch in call.args[1]
    }


def test_refresh_merges_partial_updates_and_releases_old_geometry(scene, mocker):
    log = mocker.patch.object(rr, "log")
    initial = rr.Points3D([[0, 0, 0]], colors=[255, 0, 0])
    previous_positions = weakref.ref(initial.positions)
    scene.log("world/map", initial, static=True)
    del initial

    for x in range(1, 101):
        scene.log("world/map", rr.Points3D.from_fields(positions=[[x, 0, 0]]), static=True)
    log.reset_mock()
    gc.collect()
    assert previous_positions() is None

    scene.refresh(0)

    assert refreshed(log) == {
        ("world/map", True, "Points3D:positions"): [[100.0, 0.0, 0.0]],
        ("world/map", True, "Points3D:colors"): [0xFF0000FF],
    }


def test_refreshes_stale_metadata_without_replaying_live_sensor_data(scene, mocker):
    clock = mocker.patch.object(scene_module.time, "monotonic", return_value=0)
    log = mocker.patch.object(rr, "log")
    scene.log("world/camera", rr.Transform3D(parent_frame="tf#/camera"), retain=True)

    clock.return_value = 5
    scene.log("world/camera", rr.Points3D([[1, 2, 3]]))
    log.reset_mock()
    scene.refresh(5)

    assert refreshed(log) == {
        ("world/camera", False, "Transform3D:parent_frame"): ["tf#/camera"],
    }
    log.reset_mock()
    scene.refresh(5)
    log.assert_not_called()


def test_new_frame_replaces_retained_frame_even_without_retain_flag(scene, mocker):
    log = mocker.patch.object(rr, "log")
    scene.log("world/cloud", rr.Transform3D(parent_frame="tf#/old"), retain=True)
    scene.log("world/cloud", rr.Transform3D(parent_frame="tf#/new"))
    log.reset_mock()

    scene.refresh(0)

    assert refreshed(log) == {
        ("world/cloud", False, "Transform3D:parent_frame"): ["tf#/new"],
    }


@pytest.mark.parametrize("static", [False, True])
def test_recursive_clear_does_not_restore_deleted_geometry(scene, mocker, static):
    log = mocker.patch.object(rr, "log")
    scene.log("world/map/one", rr.Points3D([[1, 2, 3]]), static=static, retain=True)
    scene.log("world/map_two", rr.Points3D([[4, 5, 6]]), static=static, retain=True)
    scene.log("world/map", rr.Clear(recursive=True), static=static)
    log.reset_mock()

    scene.refresh(0)

    assert refreshed(log) == {
        ("world/map/one", static, "Points3D:positions"): [],
        ("world/map_two", static, "Points3D:positions"): [[4.0, 5.0, 6.0]],
    }


def test_stop_releases_state_and_prevents_further_logging(scene, mocker):
    log = mocker.patch.object(rr, "log")
    points = rr.Points3D([[1, 2, 3]])
    positions = weakref.ref(points.positions)
    scene.log("world/map", points, static=True)
    del points
    log.reset_mock()

    scene.stop()
    gc.collect()
    assert positions() is None
    scene.refresh(0)
    scene.log("world/map", rr.Points3D([[4, 5, 6]]), static=True)
    log.assert_not_called()
