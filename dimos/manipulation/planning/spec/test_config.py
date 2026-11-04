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

from dataclasses import replace
import json
from pathlib import Path
from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.assets.model import RobotModel


@pytest.mark.parametrize("in_memory", [False, True])
def test_robot_model_survives_blueprint_config_round_trip(tmp_path: Path, in_memory: bool) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='arm'><link name='base'/></robot>")
    model = RobotModel.from_root_link("base") if in_memory else RobotModel.from_file(urdf)
    loaded = model.load()
    config = RobotModelConfig(
        model=model,
        joint_names=[],
    )
    blueprint = ManipulationModule.blueprint(model=config)

    parsed = BlueprintConfigParser(blueprint).parse(environ={})

    kwargs = decode(encode(parsed.module_kwargs(ManipulationModule.name)))
    assert "_loaded" not in kwargs["model"]["model"]
    restored = RobotModelConfig(**kwargs["model"])
    assert restored.model.source_path == (None if in_memory else urdf)
    assert restored.model.load() == loaded


@pytest.mark.parametrize("source", ["programmatic", "config_file"])
@pytest.mark.parametrize(
    "model_override",
    [
        {"_default_joint_acceleration_limit": 2.0},
        {"_source_path": "replacement.urdf"},
    ],
    ids=["acceleration_limit", "source_path"],
)
def test_partial_robot_model_overrides_preserve_pinned_settings(
    tmp_path: Path, source: str, model_override: dict[str, Any]
) -> None:
    model = RobotModel.from_file(
        tmp_path / "robot.urdf",
        package_paths={"assets": tmp_path},
        xacro_args={"tool": "attached"},
    ).with_fixed_frame("tool", "base")
    blueprint = ManipulationModule.blueprint(model=RobotModelConfig(model=model, joint_names=[]))
    overrides = {ManipulationModule.name: {"model": {"model": model_override}}}
    parser = BlueprintConfigParser(blueprint)
    if source == "config_file":
        config_path = tmp_path / "config.json"
        config_path.write_text(json.dumps(overrides))
        parsed = parser.parse(config_path=config_path, environ={})
    else:
        parsed = parser.parse(overrides=overrides, environ={})

    kwargs = parsed.module_kwargs(ManipulationModule.name)
    restored = RobotModelConfig(**kwargs["model"])
    assert restored.model == replace(model, **model_override)


def test_attached_model_survives_blueprint_config_round_trip(tmp_path: Path) -> None:
    source = tmp_path / "single.urdf"
    source.write_text('<robot name="single"><link name="base"/></robot>')
    model = RobotModel.from_file(source).with_attached_model(
        RobotModel.from_file(source), parent_link="base", joint_name="mount", prefix="other_"
    )
    blueprint = ManipulationModule.blueprint(model=RobotModelConfig(model=model, joint_names=[]))
    parsed = BlueprintConfigParser(blueprint).parse([], environ={})
    restored = RobotModelConfig(**parsed.module_kwargs(ManipulationModule.name)["model"])
    assert restored.model.source_path == source
    assert restored.model._fixed_frames == model._fixed_frames


# Gripper task bindings: one gripper task per pose-capable planning group,
# with the legacy scalar hardware-id binding kept working.


def _groups() -> list[PlanningGroupDefinition]:
    return [
        PlanningGroupDefinition("left_arm", ("j1",), base_link="base", tip_link="left_tip"),
        PlanningGroupDefinition("right_arm", ("j2",), base_link="base", tip_link="right_tip"),
        PlanningGroupDefinition("torso", ("j3",), base_link="base"),
    ]


def _binding_config(**overrides: Any) -> RobotModelConfig:
    kwargs = {"model": RobotModel.from_root_link("base"), "joint_names": []}
    return RobotModelConfig(**{**kwargs, **overrides})


def test_each_group_resolves_to_its_own_gripper_task() -> None:
    config = _binding_config(
        gripper_tasks={"left_arm": "left_gripper", "right_arm": "right_gripper"}
    )
    assert config.resolve_gripper_tasks(_groups()) == {
        "left_arm": "left_gripper",
        "right_arm": "right_gripper",
    }
    # A group without a binding has no gripper at all.
    assert "torso" not in config.resolve_gripper_tasks(_groups())


def test_a_binding_to_an_unknown_group_is_rejected() -> None:
    config = _binding_config(gripper_tasks={"nobody": "gripper"})
    with pytest.raises(ValueError, match="unknown planning groups"):
        config.resolve_gripper_tasks(_groups())


def test_a_binding_to_a_group_without_a_pose_target_is_rejected() -> None:
    config = _binding_config(gripper_tasks={"torso": "gripper"})
    with pytest.raises(ValueError, match="pose-capable"):
        config.resolve_gripper_tasks(_groups())


def test_two_groups_cannot_share_one_gripper_task() -> None:
    with pytest.raises(ValueError, match="distinct gripper task"):
        _binding_config(gripper_tasks={"left_arm": "gripper", "right_arm": "gripper"})


def test_the_two_binding_styles_are_mutually_exclusive() -> None:
    with pytest.raises(ValueError, match="not both"):
        _binding_config(gripper_hardware_id="r1pro", gripper_tasks={"left_arm": "gripper"})


def test_the_legacy_scalar_binding_resolves_against_the_single_pose_group() -> None:
    config = _binding_config(gripper_hardware_id="r1pro")
    single = [PlanningGroupDefinition("arm", ("j1",), base_link="base", tip_link="tip")]
    assert config.resolve_gripper_tasks(single) == {"arm": "r1pro_gripper"}
    # Two pose-capable groups make the scalar binding ambiguous.
    with pytest.raises(ValueError, match="ambiguous|exactly one pose-capable"):
        config.resolve_gripper_tasks(_groups())


def test_declared_planning_groups_are_validated_at_construction() -> None:
    with pytest.raises(ValueError, match="unknown planning groups"):
        _binding_config(
            planning_groups=_groups(),
            gripper_tasks={"left_arm": "left_gripper", "nobody": "gripper"},
        )
