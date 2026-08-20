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
import pickle

from pydantic import ValidationError
import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.processing import AddFixedFrame


@pytest.mark.parametrize("filename", ["robot.urdf", "robot.xacro", "robot.urdf.xacro"])
def test_robot_model_config_accepts_urdf_descriptions(filename: str) -> None:
    config = RobotModelConfig(name="arm", urdf_path=Path(filename), joint_names=[])

    assert config.urdf_path == Path(filename)


@pytest.mark.parametrize("filename", ["robot.xml", "robot.mjcf", "robot"])
def test_robot_model_config_rejects_non_urdf_descriptions(filename: str) -> None:
    with pytest.raises(ValidationError, match="must reference a .urdf or .xacro file"):
        RobotModelConfig(name="arm", urdf_path=Path(filename), joint_names=[])


def test_robot_model_config_rejects_obsolete_model_path_field() -> None:
    with pytest.raises(ValidationError):
        RobotModelConfig.model_validate(
            {"name": "arm", "model_path": Path("robot.urdf"), "joint_names": []}
        )


def test_robot_model_config_processors_survive_pickle_round_trip() -> None:
    config = RobotModelConfig(
        name="arm",
        urdf_path=Path("robot.urdf"),
        joint_names=[],
        urdf_processors=[AddFixedFrame("tool", "base")],
    )

    restored = pickle.loads(pickle.dumps(config))

    assert restored.urdf_processors == [AddFixedFrame("tool", "base")]
