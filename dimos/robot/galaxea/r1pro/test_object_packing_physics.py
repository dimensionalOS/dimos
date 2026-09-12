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

"""Physical task results must remain usable by the JSON evaluation/reporting API."""

import json

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_state import ObjectPackingState
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


@pytest.mark.parametrize("start_jitter", [0.0, 0.02])
def test_random_box_is_physically_placed_and_completion_serializes(tmp_path, start_jitter):
    layout = sample_layout(100000)
    scene = prepare_object_scene(tmp_path / "scene.xml", layout)
    with ObjectPackingTask(scene, layout, images=False) as task:
        task.reset(layout.seed, start_joint_jitter=start_jitter)
        index = next(i for i, obj in enumerate(layout.objects) if obj.shape == "box")
        assert task.select_object(index)
        monitor = ObjectPackingState(task.model, task.data, layout, task.home)
        physical_before = [x.copy() for x in (task.data.qpos, task.data.qvel, task.data.ctrl)]
        assert monitor.select_object(index)
        monitor.observe()
        np.testing.assert_array_equal(
            monitor.goal(), task.observation()["observation.environment_state"]
        )
        for before_array, after_array in zip(
            physical_before, (task.data.qpos, task.data.qvel, task.data.ctrl), strict=True
        ):
            np.testing.assert_array_equal(before_array, after_array)
        before = task.inventory()
        saw_current_hold = False
        for _, action in task.teacher_actions():
            task.step(action)
            task.validate(before)
            monitor.observe()
            saw_current_hold |= monitor.holding()
            np.testing.assert_array_equal(
                monitor.goal(), task.observation()["observation.environment_state"]
            )
            assert monitor.inventory() == task.inventory()
        result = json.loads(
            json.dumps({"complete": task.pick_complete(), "physical": task.result().to_dict()})
        )
        assert saw_current_hold
        assert not monitor.holding()
        assert result["complete"] is True
        assert monitor.pick_complete() is True
        assert result["physical"]["bilateral_grasp"] is True
        assert task.geometry(index)["supported"] is True
