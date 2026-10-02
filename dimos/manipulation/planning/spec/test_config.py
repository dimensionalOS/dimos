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
import pickle
from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel


def test_robot_model_survives_blueprint_config_round_trip(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='arm'><link name='base'/></robot>")
    model = RobotModel.from_file(urdf)
    loaded = model.load()
    config = RobotModelConfig(
        model=model,
        joint_names=[],
    )
    blueprint = ManipulationModule.blueprint(model=config)

    parsed = BlueprintConfigParser(blueprint).parse(environ={})

    kwargs = pickle.loads(pickle.dumps(parsed.module_kwargs(ManipulationModule.name)))
    assert "_loaded" not in kwargs["model"]["model"]
    restored = RobotModelConfig(**kwargs["model"])
    assert restored.model.source_path == urdf
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
