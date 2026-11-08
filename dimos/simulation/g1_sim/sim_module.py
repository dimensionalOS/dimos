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


"""One G1's controller: the actor and the frames, for a robot simulated somewhere else.

The world runs in `MujocoSim`. This is what stands in for the robot's own compute: it hears what
the robot feels (`sim_odom` and `sim_state`), runs the walking actor, answers with joint targets,
and publishes that robot's tf, odometry and joint state under its own namespace.

A reply carries the sim time of the state it was computed from, which is how the world knows the
tick has been answered. The actor is this module's own, which is the point -- its gait phase is
per robot, and three robots sharing one all fall over.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.simulation.g1_sim.body import LeggedG1
from dimos.simulation.g1_sim.policy import G1Policy, G1Proprioception
from dimos.simulation.g1_sim.world import MID360, SENSOR_FRAME, WORLD_FRAME
from dimos.simulation.go2_legged.robot import projected_gravity
from dimos.simulation.sim_world.mujoco_sim import BASE_SUFFIX
from dimos.simulation.sim_world.world import CommandHold
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1SimConfig(ModuleConfig):
    # The prefix this robot's names take in the shared world. It is what the module answers
    # under and what the world routes its targets back by, so it must match the world's.
    prefix: str = ""
    policy: Path | None = None


class G1Sim(Module):
    """The actor and the frames of one G1 in a world another module runs."""

    config: G1SimConfig

    cmd_vel: In[Twist]
    sim_odom: In[Odometry]
    sim_state: In[JointState]

    joint_target: Out[JointState]
    tf: Out[TFMessage]
    odometry: Out[Odometry]
    ground_truth: Out[PoseStamped]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._base: Odometry | None = None
        self._names: tuple[str, ...] = ()

    @rpc
    def start(self) -> None:
        super().start()
        path = self.config.policy
        self._policy = G1Policy.load(LeggedG1.dt) if path is None else G1Policy(path, LeggedG1.dt)
        self._hold = CommandHold(CMD_VEL_TIMEOUT)
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self.register_disposable(Disposable(self.sim_odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.sim_state.subscribe(self._on_state)))

    @property
    def _child(self) -> str:
        return f"{self.config.prefix}{BASE_SUFFIX}"

    def _on_cmd_vel(self, msg: Twist) -> None:
        command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not self._hold.update(command, time.monotonic()):
            logger.warning("Ignored non-finite cmd_vel", command=command.tolist())

    def _on_odom(self, msg: Odometry) -> None:
        """The world reports every robot on one topic; this one is ours by its child frame."""
        if msg.child_frame_id == self._child:
            self._base = msg

    def _on_state(self, msg: JointState) -> None:
        """One tick of the world: pick our joints out of it, act, and answer with the targets."""
        base = self._base
        if base is None or base.ts != msg.ts:
            return  # the pose for this tick has not landed yet; the next state will carry on
        mine = [i for i, name in enumerate(msg.name) if name.startswith(self.config.prefix)]
        if not mine:
            return
        self._names = tuple(msg.name[i] for i in mine)
        positions = np.array([msg.position[i] for i in mine])
        velocities = np.array([msg.velocity[i] for i in mine])

        target = self._policy.act(self._observe(base, positions, velocities), self._command())
        self.joint_target.publish(
            JointState(
                ts=msg.ts,
                frame_id=self._child,
                name=list(self._names),
                position=target.tolist(),
                velocity=[0.0] * len(target),
            )
        )
        self._publish_frames(base, positions, velocities)

    def _command(self) -> NDArray[np.float64]:
        return self._hold.current(time.monotonic())

    def _observe(
        self, base: Odometry, positions: NDArray[np.float64], velocities: NDArray[np.float64]
    ) -> G1Proprioception:
        """What the actor was trained on, rebuilt from the two messages the world sends.

        The world reports the base twist with its linear part in the world frame, as odometry
        does; the actor wants it in the base frame, so the orientation it came with is what
        turns it round.
        """
        quaternion = _xyzw(base)
        rotation = Rotation.from_quat(quaternion).as_matrix()
        linear = np.array([base.twist.linear.x, base.twist.linear.y, base.twist.linear.z])
        angular = np.array([base.twist.angular.x, base.twist.angular.y, base.twist.angular.z])
        # projected_gravity reads w, x, y, z, which is MuJoCo's order and not the message's.
        wxyz = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
        return G1Proprioception(
            linear_velocity=rotation.T @ linear,
            angular_velocity=angular,
            gravity=projected_gravity(wxyz),
            joint_position=positions,
            joint_velocity=velocities,
        )

    def _publish_frames(
        self, base: Odometry, positions: NDArray[np.float64], velocities: NDArray[np.float64]
    ) -> None:
        """This robot's own tf, odometry and joint state, under this module's namespace."""
        stamp = base.ts
        position = np.array([base.pose.position.x, base.pose.position.y, base.pose.position.z])
        quaternion = _xyzw(base)
        rotation = Rotation.from_quat(quaternion).as_matrix()
        sensor = position + rotation @ MID360.mount_xyz
        sensor_q = Rotation.from_matrix(rotation @ MID360.mount_r).as_quat()
        child = f"{self.config.prefix}{SENSOR_FRAME}"
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(*map(float, sensor)),
                    rotation=Quaternion(*map(float, sensor_q)),
                    frame_id=WORLD_FRAME,
                    child_frame_id=child,
                    ts=stamp,
                )
            )
        )
        self.odometry.publish(
            Odometry(
                ts=stamp,
                frame_id=WORLD_FRAME,
                child_frame_id=child,
                pose=base.pose,
                twist=base.twist,
            )
        )
        self.ground_truth.publish(
            PoseStamped(
                *map(float, position),
                *map(float, quaternion),
                ts=stamp,
                frame_id=WORLD_FRAME,
            )
        )
        self.joint_state.publish(
            JointState(
                ts=stamp,
                frame_id=self._child,
                name=[name.removeprefix(self.config.prefix) for name in self._names],
                position=positions.tolist(),
                velocity=velocities.tolist(),
            )
        )


def _xyzw(base: Odometry) -> NDArray[np.float64]:
    """The base's orientation as scipy wants it: x, y, z, w, out of the message's own fields."""
    q = base.pose.orientation
    return np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
