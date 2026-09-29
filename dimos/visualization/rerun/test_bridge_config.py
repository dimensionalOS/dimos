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
from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.simulation.scene_assets.spec import SceneMeshAlignment
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule
from dimos.visualization.rerun.scene_package import SceneVisualFactory
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
)


def _identity(value: Any) -> Any:
    return value


def test_parsed_bridge_config_preserves_callable_factories(tmp_path: Path) -> None:
    root = "world/odometry/g1"
    joint_state = UrdfRobotJointStateRerunFactory("g1_urdf/g1.fixed.urdf", root)
    static_robot = UrdfRobotStaticRerunFactory("g1_urdf/g1.fixed.urdf", root)
    scene = SceneVisualFactory(tmp_path / "missing.glb", SceneMeshAlignment())
    blueprint = RerunBridgeModule.blueprint(
        visual_override={"world/g1/joints": joint_state, "world/path": _identity, "hidden": None},
        static={root: static_robot, "world/scene": scene},
        max_hz={"world/g1/joints": 20.0},
    )

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    # Worker deployment transports constructor kwargs over a multiprocessing pipe.
    kwargs = pickle.loads(pickle.dumps(parsed.module_kwargs(RerunBridgeModule.name)))
    config = Config(**kwargs)

    assert config.visual_override == blueprint.blueprints[0].kwargs["visual_override"]
    assert config.static == blueprint.blueprints[0].kwargs["static"]
    assert callable(config.visual_override["world/g1/joints"])
    assert callable(config.static[root])
    assert config.static["world/scene"](None) == []
    assert config.visual_override["world/path"] is _identity
    assert config.visual_override["hidden"] is None
    assert config.max_hz == {"world/g1/joints": 20.0}


@pytest.mark.parametrize("field_name", ["visual_override", "static"])
def test_parsed_bridge_config_rejects_dictionary_callbacks(field_name: str) -> None:
    blueprint = RerunBridgeModule.blueprint(**{field_name: {"world/g1": {"root_path": "g1"}}})

    with pytest.raises(BlueprintConfigError, match="callable"):
        BlueprintConfigParser(blueprint).parse(environ={})
