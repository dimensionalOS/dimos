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

from pathlib import Path

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel


def test_robot_model_survives_blueprint_config_round_trip(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='arm'><link name='base'/></robot>")
    config = RobotModelConfig(
        model=RobotModel.from_file(urdf),
        joint_names=[],
    )
    blueprint = ManipulationModule.blueprint(model=config)

    parsed = BlueprintConfigParser(blueprint).parse(environ={})

    kwargs = parsed.module_kwargs(blueprint.blueprints[0].name)
    assert kwargs["model"]["model"]["_source_path"] == urdf
    assert "_loaded" not in kwargs["model"]["model"]
