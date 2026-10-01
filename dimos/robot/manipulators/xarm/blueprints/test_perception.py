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

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.perception.localize.module import LiveLocalizeModule, LiveLocalizeModuleConfig
from dimos.perception.localize.types import LocalizePolicy
from dimos.robot.manipulators.xarm.blueprints import grasp
from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim


def _localize_config(blueprint: Blueprint) -> LiveLocalizeModuleConfig:
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, LiveLocalizeModule)
    )
    # Inspect only this atom: parsing the whole blueprint downloads unrelated LFS assets.
    return LiveLocalizeModuleConfig.model_validate(atom.kwargs)


def test_real_camera_mount_connects_link7_to_camera_link() -> None:
    assert grasp.XARM_WRIST_CAMERA_TRANSFORM.frame_id == "link7"
    assert grasp.XARM_WRIST_CAMERA_TRANSFORM.child_frame_id == "camera_link"


@pytest.mark.parametrize(
    ("simulated", "expected", "floor"),
    [(False, "camera_color_optical_frame", 0.25), (True, "wrist_camera_color_optical_frame", 0.07)],
)
def test_grasp_uses_wrist_camera_with_multiview_memory(
    monkeypatch: pytest.MonkeyPatch, simulated: bool, expected: str, floor: float
) -> None:
    monkeypatch.setattr(grasp, "SIMULATED", simulated)
    config = _localize_config(grasp._scene_registration())
    assert config.optical_frame == expected
    assert config.world_frame == "world"
    assert LocalizePolicy(**config.policy).min_views == 2
    assert LocalizePolicy(**config.policy).candidate_floor == floor


def test_perception_sim_uses_wrist_camera() -> None:
    config = _localize_config(xarm_perception_sim)
    assert config.optical_frame == "wrist_camera_color_optical_frame"
