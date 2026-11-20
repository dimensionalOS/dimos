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

"""Drive the simulated xArm7 TCP to a hover above the tabletop cube."""

from __future__ import annotations

import math

from pydantic import Field, FiniteFloat

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.common.blueprints import coordinator, eef_twist_task
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_HOME,
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.mujoco.scene_package_entity_composer import scene_package_entity_body_name
from dimos.teleop.policy.module import PolicyTeleopModule
from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_TCP = "link_tcp"
_CUBE = scene_package_entity_body_name("cube")


class ArmTcpFeedback(Module):
    """Select the coordinator's measured world-to-TCP transform for both control sources."""

    arm_tf: In[TFMessage]
    current_pose: Out[Transform]

    async def handle_arm_tf(self, msg: TFMessage) -> None:
        for pose in msg.transforms:
            if pose.frame_id == "world" and pose.child_frame_id == _TCP:
                self.current_pose.publish(pose)
                return


class CubeHoverConfig(ModuleConfig):
    """Hover height is measured above the centre of the cube, in metres."""

    hover_height_m: FiniteFloat = Field(default=0.10, gt=0)
    position_gain: FiniteFloat = Field(default=1.5, gt=0)
    max_speed_m_s: FiniteFloat = Field(default=0.06, gt=0)
    tolerance_m: FiniteFloat = Field(default=0.005, gt=0)


class CubeHoverPolicy(Module):
    """Use measured arm TF and MuJoCo object poses to approach the cube."""

    config: CubeHoverConfig
    current_pose: In[Transform]
    body_poses: In[TFMessage]
    policy_command: Out[TwistStamped]

    _cube: Transform | None = None
    _at_goal: bool | None = None

    async def handle_body_poses(self, msg: TFMessage) -> None:
        self._cube = next(
            (t for t in msg.transforms if t.frame_id == "world" and t.child_frame_id == _CUBE),
            None,
        )

    async def handle_current_pose(self, tcp: Transform) -> None:
        if self._cube is None:
            return
        goal = self._cube.translation + Vector3(z=self.config.hover_height_m)
        error = goal - tcp.translation
        distance = error.length()
        if not math.isfinite(distance):
            return
        at_goal = distance <= self.config.tolerance_m
        velocity = (
            Vector3()
            if at_goal
            else error * min(self.config.position_gain, self.config.max_speed_m_s / distance)
        )
        # The blender's watchdog must see the age of both observations.
        self.policy_command.publish(
            TwistStamped(ts=min(tcp.ts, self._cube.ts), frame_id="world", linear=velocity)
        )
        if at_goal != self._at_goal:
            logger.info(
                "Cube hover reached" if at_goal else "Approaching cube",
                tcp_m=tcp.translation.to_list(),
                goal_m=goal.to_list(),
                error_m=distance,
            )
            self._at_goal = at_goal


_hw = make_xarm7_sim_hardware()
policy_teleop_sim = autoconnect(
    MujocoSimModule.blueprint(
        robot_mjcf=XARM7_SIM_PATH,
        robot_meshdir=LfsPath("xarm7/assets"),
        robot_id="xarm",
        reset_joint_positions=XARM7_SIM_HOME,
        headless=True,
        camera_name="xarm-wrist_camera",
        base_frame_id="xarm-link7",
        # MuJoCo publishes tracked body poses with each wrist-camera frame.
        width=64,
        height=48,
        enable_depth=False,
        tracked_bodies=[_CUBE],
        scene_entities=[
            {
                "id": "cube",
                "initial_pose": {"x": 0.36, "y": -0.11, "z": 0.15},
                "descriptor": {
                    "kind": "dynamic",
                    "shape_hint": "box",
                    "extents": [0.04] * 3,
                    "mass": 0.05,
                    "rgba": [0.1, 0.5, 0.9, 1.0],
                },
            }
        ],
    ),
    coordinator(
        cls=ArmTwistCoordinator,
        instance_name="ControlCoordinator",
        hardware=[_hw],
        tasks=[eef_twist_task(_hw, robot_model=make_xarm7_sim_robot_config(), target_frame=_TCP)],
        publish_frame_poses=True,
    ),
    ArmTcpFeedback.blueprint(),
    CubeHoverPolicy.blueprint(),
    PolicyTeleopModule.blueprint(tcp_frame=_TCP),
).remappings([(ArmTwistCoordinator, "tf", "arm_tf")])
