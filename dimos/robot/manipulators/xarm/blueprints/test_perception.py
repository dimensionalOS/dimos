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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationConfig,
    ObjectSceneRegistrationModule,
)
from dimos.robot.manipulators.xarm.blueprints.perception import (
    XARM_PERCEPTION_CAMERA_TRANSFORM,
    xarm_perception,
)
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    xarm_perception_sim,
    xarm_room_sim,
)


def _osr_config(blueprint) -> ObjectSceneRegistrationConfig:  # type: ignore[no-untyped-def]
    atom = next(
        atom
        for atom in blueprint.active_blueprints
        if issubclass(atom.module, ObjectSceneRegistrationModule)
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    return ObjectSceneRegistrationConfig.model_validate(parsed.module_kwargs(atom.name))


def test_real_camera_mount_connects_link7_to_camera_link() -> None:
    assert XARM_PERCEPTION_CAMERA_TRANSFORM.frame_id == "link7"
    assert XARM_PERCEPTION_CAMERA_TRANSFORM.child_frame_id == "camera_link"


def test_real_and_simulation_use_their_optical_frames() -> None:
    assert _osr_config(xarm_perception).optical_frame == "camera_color_optical_frame"
    assert _osr_config(xarm_perception_sim).optical_frame == "wrist_camera_color_optical_frame"


def test_room_sim_uses_single_view_synthetic_thresholds() -> None:
    atom = next(
        atom
        for atom in xarm_room_sim.active_blueprints
        if issubclass(atom.module, ObjectSceneRegistrationModule)
    )
    # Parsing the complete room blueprint resolves its LFS-backed MuJoCo path.
    config = ObjectSceneRegistrationConfig.model_validate(atom.kwargs)

    assert config.optical_frame == "wrist_camera_color_optical_frame"
    assert config.candidate_floor == 0.07
    assert config.accept_score == 0.07
    assert config.min_views == 1
