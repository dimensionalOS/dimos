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

"""G1 velocity-command device emulator. ONNX control runs in the connection worker.

No native engine imports or stepping. SimRobotClient sends named motor targets;
physics waits for the response to each 20 ms feedback sample. This emulates a
velocity-controlled G1 using the existing policy, not Unitree's stock firmware.
"""

from collections.abc import Callable
import threading
import time
from types import ModuleType
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.model import G1_MODEL, WALKING_DRIVES
from dimos.sim2.client import SimRobotClient
from dimos.sim2.types import Frame, JointTargets, Layout
from dimos.utils.data import LfsPath

# Hardware G1Connection can be imported without the optional simulation inference extra.
ort: ModuleType | None
try:
    import onnxruntime as ort  # type: ignore[import-untyped]
except ImportError:
    ort = None

# Empirical compensation for this policy with the canonical URDF foot contacts.
# The old MJCF adapter used (-.18, 0, -.09); its contact model differs.
_DRIFT = np.array([-0.195, 0.011, -0.099])


class G1SimDevice:
    def __init__(self, prefix: str, send: Callable[[JointTargets], None]) -> None:
        self.names = tuple(prefix + "/" + d.joint for d in WALKING_DRIVES)
        self._prefix = prefix
        self._client = SimRobotClient(self.names, send)
        self._layout: Layout | None = None
        self._home = np.array([d.initial for d in WALKING_DRIVES])
        joints = {j.name: j for j in G1_MODEL.load().joints}
        self._lower = np.array([joints[d.joint].lower for d in WALKING_DRIVES], dtype=float)
        self._upper = np.array([joints[d.joint].upper for d in WALKING_DRIVES], dtype=float)
        if ort is None:
            raise ImportError("The G1 walking emulator needs onnxruntime; install the cpu extra")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self._policy = ort.InferenceSession(
            str(LfsPath("mujoco_sim/unitree_g1_policy.onnx")),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        self._lock = threading.Lock()
        self._velocity = np.zeros(3)
        self._until = 0.0
        self._identity = ("", -1)
        self._last_action = np.zeros(29)
        self._phase = np.array([0.0, np.pi])
        self._last_targets: dict[str, float] = {}
        self.inferences = 0
        self.max_inference_ms = 0.0

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.move(Twist())

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
        raise NotImplementedError("the walking emulator exposes cmd_vel, not Unitree WebRTC RPCs")

    def observe(self, frame: Frame) -> bool:
        state = frame.state
        if self._layout is None or state.revision != self._layout.revision:
            return False
        self._client.observe(state)
        identity = (state.epoch, state.tick)
        if identity == self._identity:
            # Retransmit without advancing the recurrent policy history.
            self._client.write(self._last_targets, paced=True)
            return False
        if state.epoch != self._identity[0]:
            self._last_action[:] = 0
            self._phase[:] = [0, np.pi]
            with self._lock:
                self._velocity[:] = 0
                self._until = 0.0
        elif state.tick <= self._identity[1]:
            return False
        with self._lock:
            velocity = self._velocity.copy() if time.monotonic() < self._until else np.zeros(3)
        rotation = Rotation.from_quat(state.poses[self._imu, 3:].copy()).as_matrix()
        command = velocity * [2, 2, 1] + _DRIFT
        obs = np.concatenate(
            (
                rotation.T @ state.twists[self._imu, :3],
                frame.imu[self._gyro, 3:],
                rotation.T @ np.array([0, 0, -1]),
                command,
                state.position[self._indices] - self._home,
                state.velocity[self._indices],
                self._last_action,
                np.cos(self._phase),
                np.sin(self._phase),
            )
        ).astype(np.float32)
        started = time.monotonic()
        self._last_action = self._policy.run(None, {"obs": obs[None]})[0][0]
        self.max_inference_ms = max(self.max_inference_ms, (time.monotonic() - started) * 1000)
        self.inferences += 1
        target = np.clip(self._home + 0.5 * self._last_action, self._lower, self._upper)
        self._last_targets = dict(zip(self.names, target.tolist(), strict=True))
        self._client.write(self._last_targets, paced=True)
        self._phase = (self._phase + 2 * np.pi * 1.5 * 0.02 + np.pi) % (2 * np.pi) - np.pi
        self._identity = identity
        return True
