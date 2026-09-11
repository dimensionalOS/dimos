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

"""Persistent shared physics and visitor lifecycle; robot stacks run independently."""

from dimos.core.coordination.blueprints import autoconnect
from microduck_world.ball_detection import BallPerception
from microduck_world.camera import HEAD_CAMERA
from microduck_world.cockpit import world_cockpit
from microduck_world.robot_io import ROBOT_IDS
from microduck_world.scene import load_world
from microduck_world.supervisor import RobotSupervisor
from microduck_world.world_sim import WorldSimModule

_package, _scene = load_world()
cockpit_world = (
    autoconnect(
        WorldSimModule.blueprint(
            scene_xml=_package.mujoco_scene_path,
            headless=True,
            spawn_xy=_scene.spawn_xy,
            camera_name=HEAD_CAMERA,
            auto_stand=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_mujoco_lidar=False,
            chase_cam=False,
        ),
        world_cockpit("world"),
        RobotSupervisor.blueprint(),
        BallPerception.blueprint(),
    )
    .remappings(
        [
            (WorldSimModule, f"{robot}_{kind}", f"{robot}/hardware_{kind}")
            for robot in ROBOT_IDS
            for kind in ("command", "state", "vision")
        ]
        + [(BallPerception, f"{robot}_vision", f"{robot}/hardware_vision") for robot in ROBOT_IDS]
    )
    .global_config(robot_model="microduck", viewer="none", n_workers=4)
)
