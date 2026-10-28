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

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.message_utils import geometry
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header
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


@pytest.mark.parametrize("payload_kind", ["flat", "generated", "canonical"])
def test_nonzero_base_pose_survives_launcher_and_json_round_trip(tmp_path, payload_kind):
    expected = PoseStamped(
        Header(Time(123, 125000000), "map"),
        Pose(Point(1.25, -2.5, 0.75), Quaternion(0.0, 0.0, 0.6, 0.8)),
    )
    payload = {
        "flat": {
            "position": [1.25, -2.5, 0.75],
            "orientation": [0.0, 0.0, 0.6, 0.8],
            "ts": 123.125,
            "frame_id": "map",
        },
        "generated": expected,
        "canonical": {
            "header": {"stamp": {"sec": 123, "nanosec": 125000000}, "frame_id": "map"},
            "pose": {
                "position": {"x": 1.25, "y": -2.5, "z": 0.75},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.6, "w": 0.8},
            },
        },
    }[payload_kind]
    config = RobotModelConfig(
        model=RobotModel.from_file(tmp_path / "robot.urdf"), joint_names=[], base_pose=payload
    )
    parsed = BlueprintConfigParser(ManipulationModule.blueprint(model=config)).parse(environ={})
    restored = RobotModelConfig(**parsed.module_kwargs(ManipulationModule.name)["model"])
    restored = RobotModelConfig.model_validate_json(
        restored.model_dump_json(exclude={"rpc_transport"})
    )
    assert (
        restored.base_pose.header.frame_id,
        restored.base_pose.header.stamp.sec,
        restored.base_pose.header.stamp.nanosec,
    ) == ("map", 123, 125000000)
    assert restored.base_pose.pose == expected.pose
    np.testing.assert_allclose(
        geometry.pose_matrix(restored.base_pose.pose),
        [
            [0.28, -0.96, 0.0, 1.25],
            [0.96, 0.28, 0.0, -2.5],
            [0.0, 0.0, 1.0, 0.75],
            [0.0, 0.0, 0.0, 1.0],
        ],
        atol=1e-12,
    )
