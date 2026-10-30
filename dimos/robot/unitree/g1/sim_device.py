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

"""G1 device facade using the existing GR00T control task, outside physics.

One policy evaluation per 20 ms simulation sample. GR00T owns the legs/waist;
this single-controller example holds arms at the reference pose. The existing
ControlCoordinator can compose that task with arm tasks; this facade does not
implement another arbiter or copy observation/action mathematics.
"""

from collections.abc import Callable
import threading
import time
from typing import Any

import numpy as np

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.model import G1_MODEL, WALKING_DRIVES
from dimos.sim2.client import SimRobotClient
from dimos.sim2.types import Frame, JointTargets, Layout
from dimos.utils.data import LfsPath

# Hardware-only G1Connection remains importable without the CPU inference extra.
try:
    from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
        G1GrootWBCTask,
        G1GrootWBCTaskConfig,
    )
except ModuleNotFoundError as error:
    if error.name != "onnxruntime":
        raise
    _GROOT_AVAILABLE = False
else:
    _GROOT_AVAILABLE = True


class G1SimDevice:
    def __init__(self, prefix: str, send: Callable[[JointTargets], None]) -> None:
        if not _GROOT_AVAILABLE:
            raise ImportError("GR00T needs onnxruntime; install the cpu extra")
        self.names = tuple(prefix + "/" + d.joint for d in WALKING_DRIVES)
        self._prefix = prefix
        self._client = SimRobotClient(self.names, send)
        self._layout: Layout | None = None
        self._home = np.array([d.initial for d in WALKING_DRIVES])
        joints = {j.name: j for j in G1_MODEL.load().joints}
        self._lower = np.array([joints[d.joint].lower for d in WALKING_DRIVES], dtype=float)
        self._upper = np.array([joints[d.joint].upper for d in WALKING_DRIVES], dtype=float)
        self._task = G1GrootWBCTask(
            "groot_wbc",
            G1GrootWBCTaskConfig(
                balance_onnx=LfsPath("groot/balance.onnx"),
                walk_onnx=LfsPath("groot/walk.onnx"),
                joint_names=list(self.names[:15]),
                all_joint_names=list(self.names),
                auto_arm=True,
                default_ramp_seconds=0.0,
                decimation=1,
            ),
        )
        self._lock = threading.Lock()
        self._velocity = np.zeros(3)
        self._until = 0.0
        self._identity = ("", -1)
        self._last_targets: dict[str, float] = {}
        self.inferences = 0
        self.max_inference_ms = 0.0

    def start(self) -> None:
        self._identity = ("", -1)
        self._last_targets = {}
        self._task.start()

    def stop(self) -> None:
        self._task.stop()

    def bind(self, layout: Layout) -> None:
        if self._layout is not None and self._layout.revision == layout.revision:
            return
        self._indices = [layout.joints.index(n) for n in self.names]
        self._imu = layout.bodies.index(self._prefix + "/imu")
        self._gyro = layout.imu_frames.index(self._prefix + "/imu")
        self._layout = layout

    def move(self, twist: Twist, duration: float = 0.0) -> None:
        velocity = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        if not np.isfinite(velocity).all() or not np.isfinite(duration) or duration < 0:
            raise ValueError("velocity and duration must be finite; duration must be nonnegative")
        with self._lock:
            self._velocity = np.clip(velocity, [-0.5, -0.3, -0.6], [0.5, 0.3, 0.6])
            self._until = time.monotonic() + (duration or 0.3)

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        raise NotImplementedError("the GR00T emulator exposes cmd_vel, not Unitree WebRTC RPCs")

    def status(self) -> dict[str, Any]:
        return {
            "controller": "GR00T WBC (balance + walk)",
            **self._task.state_snapshot(),
            "inferences": self.inferences,
            "max_inference_ms": self.max_inference_ms,
        }

    def observe(self, frame: Frame) -> bool:
        state = frame.state
        if self._layout is None or state.revision != self._layout.revision:
            return False
        identity = (state.epoch, state.tick)
        if state.epoch == self._identity[0] and state.tick < self._identity[1]:
            return False
        self._client.observe(state)
        if identity == self._identity:
            # Retransmit without advancing GR00T's six-frame history.
            self._client.write(self._last_targets, paced=True)
            return False
        if state.epoch != self._identity[0]:
            self._task.reset_runtime_state(reactivate=True)
            with self._lock:
                self._velocity[:] = 0
                self._until = 0.0
        with self._lock:
            velocity = self._velocity.copy() if time.monotonic() < self._until else np.zeros(3)
        # Deadman uses wall time above; the task's state/history clock uses simulation time.
        self._task.set_velocity_command(
            float(velocity[0]), float(velocity[1]), float(velocity[2]), t_now=state.sim_time
        )
        quat = state.poses[self._imu, 3:]
        sample = frame.imu[self._gyro]
        feedback = CoordinatorState(
            joints=JointStateSnapshot(
                joint_positions=dict(zip(self.names, state.position[self._indices], strict=True)),
                joint_velocities=dict(zip(self.names, state.velocity[self._indices], strict=True)),
                timestamp=state.wall_time,
            ),
            imu={
                self._prefix: IMUState(
                    quaternion=(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2])),
                    gyroscope=tuple(sample[3:]),
                    accelerometer=tuple(sample[:3]),
                )
            },
            t_now=state.sim_time,
            dt=0.02,
        )
        started = time.monotonic()
        command = self._task.compute(feedback)
        self.max_inference_ms = max(self.max_inference_ms, (time.monotonic() - started) * 1000)
        if command is None or command.positions is None:
            return False  # physics stays at its control barrier
        self.inferences += 1
        target = self._home.copy()
        target[:15] = command.positions
        target = np.clip(target, self._lower, self._upper)
        self._last_targets = dict(zip(self.names, target.tolist(), strict=True))
        self._client.write(self._last_targets, paced=True)
        self._identity = identity
        return True
