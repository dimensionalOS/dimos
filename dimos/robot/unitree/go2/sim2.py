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

"""A simulated Go2: its locomotion policy runs here, on the world's clock, and drives the joints."""

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.pxr import frame_name, layer_from, load_pxr
from dimos.robot.unitree.go2.connection import GO2_MODEL, ConnectionConfig
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    L1_MOUNT,
    L1_XYZ,
    MID360_PITCH_DOWN,
    MID360_XYZ,
    OPTICAL_RPY,
)
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.lfs import pull_model_assets
from dimos.sim2.mount_odometry import MountOdometry
from dimos.sim2.mujoco.sensor_world import rotation_matrix
from dimos.sim2.sim import SimulatedWorldSpec, until_it_stops
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.go2_legged.policy import Go2Policy, Proprioception, load_policy
from dimos.simulation.go2_legged.robot import (
    COMMAND_SLEW,
    CONTROL_DT,
    projected_gravity,
    yaw_of,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GO2_USD = UsdModel.from_file(Path(__file__).parent / "assets" / "go2.usda").with_frames(
    {
        "l1": (
            "base_link",
            L1_XYZ,
            (math.radians(L1_MOUNT[0]), math.radians(L1_MOUNT[1]), math.radians(L1_MOUNT[2])),
        ),
        "front_camera": ("base_link", CAMERA_XYZ, (0.0, 0.0, 0.0)),
        "lidar": ("front_camera", MID360_XYZ, (0.0, MID360_PITCH_DOWN, 0.0)),
        "camera": ("front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
    }
)


# What a real Go2 publishes, for any module simulating one: odom unprefixed, the body its root link.
GO2_TF_FRAMES: dict[str, dict[str, str]] = {
    # The real Go2's root link, which needs no USD read to build a blueprint; GO2_USD's matches it.
    "tf_frames_local": {"body": GO2_MODEL.load().root_link},
    "tf_frames_global": {"odom": ConnectionConfig.model_fields["odom_frame_id"].default},
}


def with_policy_drives(usd: UsdModel, policy: Go2Policy) -> UsdModel:
    """The USD with the policy's gains and default pose on its joint drives, which the world holds it at."""
    pxr = load_pxr()
    layer = layer_from(pxr, usd.usd, "a robot", usd.source_path.parent if usd.source_path else None)
    stage = pxr.Usd.Stage.Open(layer)
    drives = {
        name: (float(kp), float(kd), float(q))
        for name, kp, kd, q in zip(
            policy.joint_names, policy.kp, policy.kd, policy.default_pose, strict=True
        )
    }
    for prim in stage.Traverse():
        if prim.IsA(pxr.UsdPhysics.RevoluteJoint) and frame_name(prim) in drives:
            kp, kd, target = drives.pop(frame_name(prim))
            drive = pxr.UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr().Set(kp)
            drive.CreateDampingAttr().Set(kd)
            drive.CreateTargetPositionAttr().Set(target)
    if drives:
        raise ValueError(f"the policy drives joints the USD does not have: {sorted(drives)}")
    return UsdModel.from_layer(str(layer.ExportToString()), usd.source_path)


class Go2SimConfig(HardwareModuleConfig):
    usd: UsdModel = GO2_USD
    policy: Path | None = None
    # Proportional yaw hold while no turn is commanded, rad/s per rad of drift.
    heading_gain: float = 2.0
    tf_frames_local: dict[str, str] = GO2_TF_FRAMES["tf_frames_local"]
    tf_frames_global: dict[str, str] = GO2_TF_FRAMES["tf_frames_global"]
    tf_edges: list[tuple[str, str]] = [("odom", "body")]


class Go2Sim(HardwareModule):
    """A Go2 in a simulated world; mount it on a frame of the scene, which is where it starts."""

    config: Go2SimConfig
    mounts = Mounts("l1", "lidar", "camera")
    world: SimulatedWorldSpec
    cmd_vel: In[Twist]
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]
    odom: Out[PoseStamped]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Before its drives are written, which opens a stage of every mesh layer it references.
        pull_model_assets(self.config.usd)
        self._policy = load_policy(self.config.policy)
        self.config.usd = with_policy_drives(self.config.usd, self._policy)
        self._desired = np.zeros(3)
        self._desired_expires = 0.0
        self._command = np.zeros(3)
        self._heading: float | None = None
        self._odometry: MountOdometry | None = None

    @property
    def mounted_frame(self) -> str:
        return str(self.frame.odom)

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(
            self.namespace, self.config.usd, held_until_commanded=True, odom_frame=self.frame.odom
        )

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations, self.world.snapshot)
        acting = asyncio.create_task(until_it_stops("go2 policy", self._run()))
        yield
        acting.cancel()

    async def handle_cmd_vel(self, twist: Twist) -> None:
        desired = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        if not np.isfinite(desired).all():
            logger.warning(f"go2 sim: ignoring a non-finite cmd_vel (vx, vy, vyaw) = {desired}")
            return
        self._desired = desired
        self._desired_expires = self._world.stamp() + CMD_VEL_TIMEOUT

    async def _run(self) -> None:
        async for _ in self._world.ticks(1.0 / CONTROL_DT):
            self._act()

    def _act(self) -> None:
        policy, world, robot = self._policy, self._world, self.namespace
        with world.lock:
            base = world.entity(robot)
            names, rows = world.joints(robot)
            stamp = world.stamp()
        if base is None or base.pose is None or base.velocity is None:
            return
        x, y, z, w = base.pose[3:]
        quat = np.array([w, x, y, z])
        desired = self._desired if stamp < self._desired_expires else np.zeros(3)
        self._command += np.clip(desired - self._command, -COMMAND_SLEW, COMMAND_SLEW)
        if desired[2] == 0:
            self._heading = yaw_of(quat) if self._heading is None else self._heading
            turn = self.config.heading_gain * math.remainder(self._heading - yaw_of(quat), math.tau)
        else:
            self._heading, turn = None, 0.0
        joints = dict(zip(names, rows, strict=True))
        observation = Proprioception(
            angular_velocity=rotation_matrix(quat).T @ base.velocity[3:],
            gravity=projected_gravity(quat),
            joint_position=np.array([joints[name][0] for name in policy.joint_names]),
            joint_velocity=np.array([joints[name][1] for name in policy.joint_names]),
        )
        targets = policy.act(observation, self._command + np.array([0.0, 0.0, turn]))
        self.actions.publish(
            world_action(
                world,
                JointCommand(robot, tuple(policy.joint_names), position=tuple(map(float, targets))),
            )
        )
        self.joint_state.publish(
            JointState(
                ts=stamp,
                name=list(policy.joint_names),
                position=observation.joint_position.tolist(),
                velocity=observation.joint_velocity.tolist(),
            )
        )
        self._publish_pose(base.pose, stamp, {name: float(row[0]) for name, row in joints.items()})

    def _odom(self, pose: np.ndarray, stamp: float) -> PoseStamped:
        if self._odometry is None:
            usd = self.config.usd
            self._odometry = MountOdometry.of(usd, usd.root_link)
        relative = self._odometry(pose)
        return PoseStamped(
            ts=stamp,
            frame_id=self.frame.odom,
            position=[relative.translation.x, relative.translation.y, relative.translation.z],
            orientation=[
                relative.rotation.x,
                relative.rotation.y,
                relative.rotation.z,
                relative.rotation.w,
            ],
        )

    def _publish_pose(self, pose: np.ndarray, stamp: float, positions: dict[str, float]) -> None:
        odom = self._odom(pose, stamp)
        self.odom.publish(odom)
        self.tfbuffer.publish(
            Transform.from_pose(self.frame.body, odom), *self.usd_transforms(positions, ts=stamp)
        )
