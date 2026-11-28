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


"""GR00T walking a G1: one policy step every period of wall time, on its latest joint states and IMU.

Nothing waits for it, as on the real robot: a step that comes late is late, and the robot has moved on meanwhile.
"""

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
import time
from typing import Any, ClassVar

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
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
        """The 15 leg and waist targets for one step, from a (vx, vy, wz) command and the robot's readings."""
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
        # The input is the last few observations, oldest first; the first step fills them all with its own.
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


class G1WalkerConfig(ModuleConfig):
    model_path: Path = LfsPath("groot")
    # GR00T was trained stepping at 50 Hz.
    rate_hz: float = 50.0
    # Seconds a cmd_vel is followed for, as on the real G1.
    cmd_vel_timeout: float = G1GrootWBCTaskConfig.timeout


class G1Walker(Module):
    """Walks a G1 under GR00T: joint_state, imu and cmd_vel in, position_command out, a step every period of wall time."""

    config: G1WalkerConfig
    # Its policy step shares no GIL with another module, which would make it late.
    dedicated_worker: ClassVar[bool] = True
    joint_state: In[JointState]
    imu: In[Imu]
    cmd_vel: In[Twist]
    position_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._policy: GrootPolicy | None = None
        self._joints: JointState | None = None
        self._imu: Imu | None = None
        self._command = np.zeros(3, dtype=np.float32)
        self._command_until = -math.inf
        self._arms: NDArray[np.float32] | None = None

    async def main(self) -> AsyncIterator[None]:
        self._policy = await asyncio.to_thread(GrootPolicy, Path(self.config.model_path))
        walking = asyncio.create_task(self._walk())
        yield
        walking.cancel()

    async def handle_joint_state(self, joints: JointState) -> None:
        self._joints = joints

    async def handle_imu(self, imu: Imu) -> None:
        self._imu = imu

    async def handle_cmd_vel(self, twist: Twist) -> None:
        command = np.array([twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float32)
        if not np.isfinite(command).all():
            logger.warning(f"g1 walker: ignoring a non-finite cmd_vel (vx, vy, wz) = {command}")
            return
        self._command = command
        self._command_until = time.time() + self.config.cmd_vel_timeout

    async def _walk(self) -> None:
        async for _ in wall_ticks(self.config.rate_hz):
            self.act()

    def act(self) -> None:
        """One policy step on the latest readings, sent as a position_command; nothing until both have come."""
        policy, joints, imu = self._policy, self._joints, self._imu
        if policy is None or joints is None or imu is None:
            return
        by_name = dict(
            zip(joints.name, zip(joints.position, joints.velocity, strict=True), strict=True)
        )
        missing = [name for name in g1_joints if name not in by_name]
        if missing:
            logger.warning(f"g1 walker: its joint_state lacks {missing}")
            return
        readings = np.array([by_name[name] for name in g1_joints], dtype=np.float32)
        orientation = imu.orientation
        imu_from_world = Rotation.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        ).inv()
        gyro = np.array(
            [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z],
            dtype=np.float32,
        )
        gravity = imu_from_world.apply([0.0, 0.0, -1.0]).astype(np.float32)
        now = time.time()
        command = self._command if now < self._command_until else np.zeros(3, dtype=np.float32)
        legs = policy.step(command, gyro, gravity, readings[:, 0], readings[:, 1])
        # The arms are no part of GR00T's action: held where they first were.
        if self._arms is None:
            self._arms = readings[legs.size :, 0]
        self.position_command.publish(
            JointState(ts=now, name=g1_joints, position=[*legs.tolist(), *self._arms.tolist()])
        )


async def wall_ticks(rate_hz: float) -> AsyncIterator[float]:
    """Wall times ``rate_hz`` apart, each on its due time; one more than a period late skips what it missed."""
    period = 1.0 / rate_hz
    due = time.time()
    while True:
        # In short sleeps: macOS lets a background process's sleep overrun by several times its length.
        while (remaining := due - time.time()) > 0:
            await asyncio.sleep(min(max(remaining / 8, 0.0005), 0.5))
        ticked = time.time()
        yield ticked
        due += period
        if due <= time.time():
            due = time.time() + period
