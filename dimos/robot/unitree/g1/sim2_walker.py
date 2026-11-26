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

"""GR00T walking a simulated G1: one policy step every GROOT_PERIOD of wall time, on the world as it is then.

Nothing waits for it, as on the real robot: a step that comes late is late, and the world has moved on meanwhile.
"""

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
import time
from typing import Any, ClassVar

from dimos_lcm.world_msgs import WorldObservations
import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]
from scipy.spatial.transform import Rotation

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1GrootWBCTaskConfig,
    g1_joints,
    g1_legs_waist,
)
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.sim2 import usd_joint
from dimos.sim2.sim import SimulatedWorldSpec, until_it_stops
from dimos.sim2.world_observations import WorldMirror
from dimos.utils.data import LfsPath

# GR00T was trained at 50 Hz.
GROOT_PERIOD = 1 / 50


class GrootPolicy:
    """GR00T's balance and walk ONNX policies, with the observation history and last action they read."""

    def __init__(self, model_dir: Path) -> None:
        # The real G1's GR00T task holds the policies' training contract: default pose and scales.
        self.contract = G1GrootWBCTaskConfig(
            balance_onnx=model_dir / "balance.onnx",
            walk_onnx=model_dir / "walk.onnx",
            joint_names=g1_legs_waist,
            all_joint_names=g1_joints,
        )
        self._balance = ort.InferenceSession(str(self.contract.balance_onnx))
        self._walk = ort.InferenceSession(str(self.contract.walk_onnx))
        self.default_pose = np.asarray(self.contract.default_positions_29, dtype=np.float32)
        self._last_action = np.zeros(len(g1_legs_waist), dtype=np.float32)
        self._history: NDArray[np.float32] | None = None

    def step(
        self,
        command: NDArray[np.float32],
        gyro: NDArray[np.float32],
        gravity: NDArray[np.float32],
        position: NDArray[np.float32],
        velocity: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        """The 15 leg and waist targets for one tick, from a (vx, vy, wz) command and the robot's readings."""
        contract = self.contract
        observation = np.concatenate(
            [
                command * np.asarray(contract.cmd_scale, dtype=np.float32),
                [contract.height_cmd, 0.0, 0.0, 0.0],
                gyro * contract.obs_ang_vel_scale,
                gravity,
                (position - self.default_pose) * contract.obs_dof_pos_scale,
                velocity * contract.obs_dof_vel_scale,
                self._last_action,
            ]
        ).astype(np.float32)
        # The input is the last few observations, oldest first; the first tick fills them all with its own.
        if self._history is None:
            input_size = self._balance.get_inputs()[0].shape[1]
            assert isinstance(input_size, int), "GR00T takes a fixed number of observations"
            self._history = np.tile(observation, input_size // observation.size)
        else:
            self._history = np.concatenate([self._history[observation.size :], observation])
        walking = float(np.linalg.norm(command)) > contract.cmd_norm_threshold
        session = self._walk if walking else self._balance
        inputs = {session.get_inputs()[0].name: self._history[None, :]}
        action = session.run(None, inputs)[0][0, : len(g1_legs_waist)].astype(np.float32)
        self._last_action = action
        return np.asarray(action * contract.action_scale + self.default_pose[: action.size])


class G1SimWalkerConfig(ModuleConfig):
    model_path: Path = LfsPath("groot")
    # Seconds a cmd_vel is followed for, as on the real G1.
    cmd_vel_timeout: float = G1GrootWBCTaskConfig.timeout


class G1SimWalker(Module):
    """Walks the simulated G1 of its namespace under GR00T: reads its world, takes cmd_vel, sends position_command."""

    config: G1SimWalkerConfig
    # Its 50 Hz policy step shares no GIL with a sensor, which would make it late.
    dedicated_worker: ClassVar[bool] = True
    world: SimulatedWorldSpec
    world_observations: In[WorldObservations]
    cmd_vel: In[Twist]
    position_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._policy: GrootPolicy | None = None
        self._command = np.zeros(3, dtype=np.float32)
        self._command_until = -math.inf
        self._arms: NDArray[np.float32] | None = None

    async def main(self) -> AsyncIterator[None]:
        self._policy = await asyncio.to_thread(GrootPolicy, Path(self.config.model_path))
        self._world = WorldMirror()
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self._world.subscribe(self.world_observations, self.world.snapshot)
        walking = asyncio.create_task(until_it_stops("g1 walker", self._walk()))
        yield
        walking.cancel()

    async def _walk(self) -> None:
        # A late tick is just late: the ones it missed are skipped, not caught up on.
        async for _ in self._world.ticks(1 / GROOT_PERIOD):
            self.act()

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._command = np.array(
            [twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float32
        )
        self._command_until = time.time() + self.config.cmd_vel_timeout

    def act(self) -> None:
        """One policy step on the world as it is now, sent as a position_command."""
        policy, world = self._policy, self._world
        assert policy is not None
        with world.lock:
            body = world.entity(self.namespace)
            names, rows = world.joints(self.namespace)
        now = time.time()
        if body is None or body.pose is None or body.velocity is None:
            return
        by_name = dict(zip(names, rows, strict=True))
        joints = np.array([by_name[usd_joint(name)] for name in g1_joints], dtype=np.float32)
        # The pelvis IMU sits unrotated on the pelvis, so it reads the pelvis's own turn and spin.
        pelvis_from_world = Rotation.from_quat(body.pose[3:7]).inv()
        gyro = pelvis_from_world.apply(body.velocity[3:]).astype(np.float32)
        gravity = pelvis_from_world.apply([0.0, 0.0, -1.0]).astype(np.float32)
        command = self._command if now < self._command_until else np.zeros(3, dtype=np.float32)
        legs = policy.step(command, gyro, gravity, joints[:, 0], joints[:, 1])
        # The arms are no part of GR00T's action: held where they first were.
        if self._arms is None:
            self._arms = joints[legs.size :, 0]
        self.position_command.publish(
            JointState(ts=now, name=g1_joints, position=[*legs.tolist(), *self._arms.tolist()])
        )
