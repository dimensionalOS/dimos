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

"""Simulation helpers for manipulator blueprints."""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.sim2.module import SimulationModule
from dimos.sim2.spec import WorldConfig
from dimos.sim2.world.scene import describe_scene, scene_path


def arm_scene(default: str | Path = "workbench.xml") -> Path:
    return scene_path(global_config.scene_package, str(default))


def arm_base_pose(scene: Path | None = None, spawn: str = "workbench") -> PoseStamped:
    """Use the same authored support as the robot connection."""
    description = describe_scene(scene if scene is not None else arm_scene())
    if description.spawns and spawn not in description.spawns:
        raise ValueError(f"scene {description.id!r} has no authored {spawn!r} support")
    pose = description.spawns.get(spawn, Pose(0, 0, 0.12))
    return PoseStamped(
        frame_id="world",
        position=pose.position,
        orientation=pose.orientation,
    )


def arm_world(scene: Path) -> Blueprint:
    """World settings only; robot connections are collected at deployment."""
    return SimulationModule.blueprint(world=WorldConfig(scene=scene, robots={}))
