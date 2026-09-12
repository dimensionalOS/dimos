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

"""Recovery must preserve scene objects and refuse to release an airborne grasp."""

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask
from dimos.robot.galaxea.r1pro.object_recovery import plan_object_recovery
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion, TrayWaypoint

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


@pytest.fixture
def task(tmp_path):
    layout = sample_layout(100000)
    scene = prepare_object_scene(tmp_path / "scene.xml", layout)
    with ObjectPackingTask(scene, layout, images=False) as instance:
        index = next(i for i, o in enumerate(layout.objects) if o.shape == "box")
        assert instance.select_object(index)
        yield instance


@pytest.mark.parametrize("last_phase", ["approach", "grasp"])
def test_supported_release_and_retreat_preserves_objects(task, last_phase):
    for phase, action in task.teacher_actions():
        if phase == ("grasp" if last_phase == "approach" else "lift"):
            break
        task.step(action)
    before = task.inventory()
    qpos = task.data.qpos.copy()
    points = plan_object_recovery(task.model, task.data, task.layout, task.home)
    np.testing.assert_array_equal(task.data.qpos, qpos)
    motion = TrayMotion(
        task.model, task.data, cargo_bodies=tuple(o.name for o in task.layout.objects)
    )
    waypoints = [TrayWaypoint(p["phase"], p["positions"], p["seconds"]) for p in points]
    for _, action in motion.actions(waypoints):
        task.step(action)
    assert np.max(np.abs(task.data.qpos[task.qids] - task.home)) < 0.015
    assert all(r["released"] and r["support_geoms"] and r["upright"] for r in task.inventory())
    for initial, final in zip(before, task.inventory(), strict=True):
        assert np.linalg.norm(np.asarray(initial["position"]) - final["position"]) < 0.02


def test_airborne_object_cannot_be_dropped_for_recovery(task):
    for phase, action in task.teacher_actions():
        if phase == "clear_sources":
            break
        task.step(action)
    assert task.bilateral_grasp
    before = task.data.ctrl.copy()
    with pytest.raises(RuntimeError, match="supported upright"):
        plan_object_recovery(task.model, task.data, task.layout, task.home)
    np.testing.assert_array_equal(task.data.ctrl, before)
