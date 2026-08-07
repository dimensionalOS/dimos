# Copyright 2025-2026 Dimensional Inc.
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

"""Rerun entity factories must survive a config dump/re-validate round trip.

The blueprint config parser dumps every module config with
``model_dump(mode="python")`` before shipping kwargs to the worker process, and
the worker re-validates them. Pydantic rewrites dataclass instances into plain
dicts on the way out, so a ``@dataclass`` factory arrives at the worker as a
dict and fails validation against the ``Callable`` field type -- the module
never deploys. Keep these factories as plain classes.
"""

from pathlib import Path

from dimos.simulation.scene_assets.spec import SceneMeshAlignment
from dimos.visualization.rerun.bridge import Config
from dimos.visualization.rerun.scene_package import SceneVisualFactory
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
)


def _round_trip(config: Config) -> Config:
    """Mirror what blueprint_config.parser does before worker deployment."""
    dumped = config.model_dump(mode="python", exclude_unset=True)
    return Config.model_validate(dumped)


def test_urdf_factories_survive_config_round_trip() -> None:
    static_factory = UrdfRobotStaticRerunFactory(urdf_path="robot.urdf", root_path="world/robot")
    joint_factory = UrdfRobotJointStateRerunFactory(urdf_path="robot.urdf", root_path="world/robot")
    config = Config.model_validate(
        {
            "static": {"world/robot": static_factory},
            "visual_override": {"world/robot/joints": joint_factory},
        }
    )

    restored = _round_trip(config)

    assert callable(restored.static["world/robot"])
    assert callable(restored.visual_override["world/robot/joints"])


def test_scene_visual_factory_survives_config_round_trip(tmp_path: Path) -> None:
    factory = SceneVisualFactory(tmp_path / "visual.glb", SceneMeshAlignment())
    config = Config.model_validate({"static": {"world/scene": factory}})

    restored = _round_trip(config)

    assert callable(restored.static["world/scene"])
