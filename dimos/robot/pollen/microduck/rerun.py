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

"""MicroDuck-specific Rerun visualization helpers."""

from __future__ import annotations

from functools import cache
from typing import Any

from dimos.core.global_config import global_config
from dimos.robot.pollen.microduck.config import (
    MICRODUCK_HOME,
    MICRODUCK_JOINT_SUFFIXES,
    MICRODUCK_ROBOT_MJCF,
)
from dimos.visualization.rerun.mjcf_robot import MjcfRobotRerun
from dimos.visualization.rerun.scene_package import scene_package_static_entities

MICRODUCK_RERUN_ROOT = "world/microduck/odom/model"
MICRODUCK_RERUN_JOINTS = "world/microduck/joints"
MICRODUCK_RERUN_SCENE = "world/scene"


@cache
def _microduck_rerun_robot() -> MjcfRobotRerun:
    return MjcfRobotRerun(
        mjcf_path=MICRODUCK_ROBOT_MJCF,
        root_path=MICRODUCK_RERUN_ROOT,
        root_body_name="trunk_base",
        initial_joint_positions=dict(zip(MICRODUCK_JOINT_SUFFIXES, MICRODUCK_HOME, strict=True)),
    )


def microduck_static_robot(rr: Any) -> list[tuple[str, Any]]:
    """Log the official MicroDuck CAD meshes under its odometry transform."""

    return _microduck_rerun_robot().static(rr)


def microduck_joint_state(msg: Any) -> list[tuple[str, Any]]:
    """Animate the MicroDuck CAD model from its published joint state."""

    return _microduck_rerun_robot().joint_state(msg)


def microduck_static_scene(rr: Any) -> list[Any]:
    """Log the active cooked scene package without passing callable instances."""

    factory = scene_package_static_entities(global_config.scene_package).get(MICRODUCK_RERUN_SCENE)
    return [] if factory is None else factory(rr)
