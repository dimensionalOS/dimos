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

"""G1Sim: the sim twin of G1WholeBodyConnection, with its motor_command in and motor_states and pelvis imu out.

G1Sim.usd is the committed G1 layer with its sensor mounts added as frames: LivoxSim names
G1Sim.lidar_mount_frame, and the same layer answers the simulation, the UsdManager and the viewer.
"""

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
import time

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.g1_tf_publisher import _TORSO_MID360_XYZ, MID360_PITCH
from dimos.robot.unitree.g1.wholebody_connection import G1_JOINT_NAMES
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror
from dimos.utils.logging_config import setup_logger

# Where it starts: pelvis height, and per joint (Menagerie's order, which is the motors' order) the pose it stands in
# and the PD gains that hold it there until the first motor_command.
HOME_HEIGHT = 0.755
_LEG = (
    ("hip_pitch", -0.312, 75, 2),
    ("hip_roll", 0, 75, 2),
    ("hip_yaw", 0, 75, 2),
    ("knee", 0.669, 75, 2),
    ("ankle_pitch", -0.363, 20, 1),
    ("ankle_roll", 0, 2, 0.2),
)
_WAIST = (("waist_yaw", 0, 75, 2), ("waist_roll", 0, 75, 2), ("waist_pitch", 0.073, 75, 2))
_ARM = (
    ("shoulder_pitch", 0.2, 75, 2),
    ("shoulder_roll", 0.2, 75, 2),
    ("shoulder_yaw", 0, 75, 2),
    ("elbow", 0.6, 75, 2),
    ("wrist_roll", 0, 2, 0.2),
    ("wrist_pitch", 0, 2, 0.2),
    ("wrist_yaw", 0, 2, 0.2),
)


def g1_drives() -> dict[str, tuple[float, float, float]]:
    """The gains and home pose the GR00T controller holds the robot at, as `usd_with_drives` wants.

    Written into the committed layer by `scripts/usd_drives.py`, so whatever runs this robot reads
    them from the same place; the test beside it notices if the two drift. Never called at runtime.
    """
    return {
        f"{side}_{name}_joint": (kp, kd, _home(side, name, initial))
        for side in ("left", "right")
        for name, initial, kp, kd in _LEG + _ARM
    } | {f"{name}_joint": (kp, kd, initial) for name, initial, kp, kd in _WAIST}


def _home(side: str, name: str, initial: float) -> float:
    """The arms are mirrored, so the right shoulder's roll is the left's the other way."""
    return -initial if side == "right" and name == "shoulder_roll" else initial


# The joints the whole-body connection sends and expects back, in its order.
SERVO_JOINTS = (
    tuple(f"{side}_{name}_joint" for side in ("left", "right") for name, *_ in _LEG)
    + tuple(f"{name}_joint" for name, *_ in _WAIST)
    + tuple(f"{side}_{name}_joint" for side in ("left", "right") for name, *_ in _ARM)
)


logger = setup_logger()


# The body the layer is rooted at, and so the frame odometry moves.
ROOT_BODY = "pelvis"
# The IMU the whole-body controller is fed, which is the one in the pelvis and not the lidar's.
IMU_FRAME = "imu_in_pelvis"


class G1SimConfig(SceneConfig):
    robot_id: str = "g1"
    frame_id: str = "g1_pelvis"
    # What the robot's pose is given in. The world frame, as the Go2 twin uses.
    odom_frame_id: str = "world"
    # As G1WholeBodyConnection: full damping at once, stiffness and torque fading in over this long.
    soft_start_seconds: float = 3.0
    # As G1WholeBodyConnection's, but per sim second.
    publish_rate_hz: float = 500.0


class G1Sim(Module):
    # The committed layer, converted once from the menagerie MJCF (see assets/g1.usda), with the
    # Mid-360 upside down on the torso, as G1's torso_to_mid360 has it. The mount is added here
    # rather than baked in because a mount is whose robot carries it, not what the robot is.
    usd = UsdModel.from_file(Path(__file__).parent / "assets" / "g1.usda").with_frames(
        {"mid360_link": ("torso_link", _TORSO_MID360_XYZ, (math.pi, MID360_PITCH, 0.0))}
    )
    lidar_mount_frame = "mid360_link"

    config: G1SimConfig
    motor_command: In[MotorCommandArray]
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]
    # Named and ordered as G1WholeBodyConnection's.
    motor_states: Out[JointState]
    imu: Out[Imu]
    # Where the robot is. The sim knows it exactly and nothing else does: without it a viewer
    # cannot place the robot, so the model goes undrawn and every frame hanging off it is reported
    # as missing from the tf tree.
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    _view: WorldView | None = None
    _soft_start: float | None = None

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._publishing = asyncio.create_task(self._publish())
        yield

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        if command.num_joints != len(SERVO_JOINTS):
            return
        now = time.monotonic()
        if self._soft_start is None:
            self._soft_start = now
        duration = self.config.soft_start_seconds
        scale = min(1.0, (now - self._soft_start) / duration) if duration > 0 else 1.0
        self.actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.config.robot_id,
                    SERVO_JOINTS,
                    position=tuple(command.q),
                    velocity=tuple(command.dq),
                    effort=tuple(tau * scale for tau in command.tau),
                    kp=tuple(kp * scale for kp in command.kp),
                    kd=tuple(command.kd),
                ),
            )
        )

    def _publish_pose(self, world: "WorldMirror", robot: str, stamp: float) -> None:
        """The robot's pose in the world, as odometry and as the tf edge to its root body.

        An entity's pose is x, y, z then the quaternion as x, y, z, w -- the order a PoseStamped
        already wants, so it goes straight through.
        """
        base = world.entity(robot)
        if base is None or base.pose is None:
            return
        position, orientation = base.pose[:3], base.pose[3:7]
        odom = PoseStamped(
            ts=stamp,
            frame_id=self.config.odom_frame_id,
            position=position.tolist(),
            orientation=orientation.tolist(),
        )
        # Every module config carries frame_id_prefix and namespace() sets it, so several robots
        # in one world do not all publish the same "pelvis" for tf to choose between.
        prefix = self.config.frame_id_prefix
        root = f"{prefix}/{ROOT_BODY}" if prefix else ROOT_BODY
        self.tf.publish(TFMessage(Transform.from_pose(root, odom)))
        self.odom.publish(odom)

    def _open(self) -> WorldView:
        """This robot's own world, for the IMU in its pelvis: the scene, with a site at that frame.

        Opened on the first frame rather than at start, because the robots in the world are the ones
        the observations announce, and the view has to name its bodies the way they do.
        """
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.config.robot_id}: nothing says what the scene is, so there is no IMU to"
                f" read. Give this robot the scene_manager that composes the world."
            )
        return open_view(
            usd_layer(scene),
            robot_ids(self._world),
            SensorMount("imu", f"{self.config.robot_id}/{IMU_FRAME}", IMU_FRAME),
        )

    async def _publish(self) -> None:
        robot, world = self.config.robot_id, self._world
        view: WorldView | None = None
        async for _ in world.ticks(self.config.publish_rate_hz):
            if view is None:
                view = self._open()
            with world.lock:
                names, rows = world.joints(robot)
                stamp = world.stamp()
            if not names:
                continue
            index = {name: i for i, name in enumerate(names)}
            rows = rows[[index[joint] for joint in SERVO_JOINTS]]
            self.motor_states.publish(
                JointState(
                    ts=stamp,
                    frame_id=self.config.frame_id,
                    name=G1_JOINT_NAMES,
                    position=rows[:, 0].tolist(),
                    velocity=rows[:, 1].tolist(),
                    effort=rows[:, 2].tolist(),
                )
            )
            self._publish_pose(world, robot, stamp)
            view.update(world)
            gyro, accelerometer, (w, x, y, z) = view.imu(IMU_FRAME)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.config.frame_id,
                    ts=stamp,
                )
            )
