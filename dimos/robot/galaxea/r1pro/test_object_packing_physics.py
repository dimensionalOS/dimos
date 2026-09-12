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

import pytest

from dimos.robot.galaxea.r1pro.object_packing_scene import prepare_object_scene, sample_layout
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


def test_random_box_is_physically_placed_and_completion_serializes(tmp_path):
    layout = sample_layout(100000)
    scene = prepare_object_scene(tmp_path / "scene.xml", layout)
    with ObjectPackingTask(scene, layout, images=False) as task:
        index = next(i for i, obj in enumerate(layout.objects) if obj.shape == "box")
        assert task.select_object(index)
        before = task.inventory()
        for _, action in task.teacher_actions():
            task.step(action)
            task.validate(before)
        result = json.loads(
            json.dumps({"complete": task.pick_complete(), "physical": task.result().to_dict()})
        )
        assert result["complete"] is True
        assert result["physical"]["bilateral_grasp"] is True
        assert task.geometry(index)["supported"] is True
