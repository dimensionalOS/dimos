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

"""Read-only worker-local model/data; physics publishes only integration state."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from typing import Any

import mujoco
import numpy as np

from dimos.sim2.messages import StateFrame, StreamDescriptor
from dimos.sim2.streams import SimStream
from dimos.sim2.world.runtime import STATE


class WorldReader:
    def __init__(self, description: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_binary_path(description["model"])
        self.data = mujoco.MjData(self.model)
        self.channel = SimStream.attach(StreamDescriptor.from_dict(description["snapshot"]))
        self.sequence = -1
        self.timestamp = 0.0
        self.episode = -1
        self._history: tuple[StateFrame, ...] = ()
        self._next: mujoco.MjData | None = None

    def update(self) -> bool:
        if self.channel.lifecycle != "ready":
            return False
        frame = self.channel.read_observation()
        if frame is None:
            return False
        changed = frame.metadata.sequence != self.sequence
        self.restore(frame)
        self.sequence = frame.metadata.sequence
        self.episode = frame.metadata.episode_id
        return changed

    def restore(self, frame: StateFrame) -> None:
        """Restore an exact observation, including MuJoCo's mounted IMU readings."""
        mujoco.mj_setState(self.model, self.data, frame.values["state"], STATE)
        mujoco.mj_forward(self.model, self.data)
        self.timestamp = float(frame.values["wall_time"][0])

    def history(self, start: float, end: float) -> bool:
        """Pin a complete interval from this episode; never invent missing motion."""
        frames = self.channel.read_observations()
        self._history = tuple(f for f in frames if f.metadata.episode_id == self.episode)
        times = np.array([f.metadata.sim_time for f in self._history])
        first = max(0, bisect_right(times, start) - 1)
        last = bisect_left(times, end) + 1
        self._history = self._history[first:last]
        times = times[first:last]
        period = self.channel.descriptor.physics_dt * self.channel.descriptor.control_decimation
        if (
            len(times) == 0
            or times[0] > start + 1e-9
            or times[-1] < end - 1e-9
            or np.any(np.diff(times) > period * 1.5)
            or np.any(np.diff(times) < 0)
        ):
            self._history = ()
            return False
        return True

    def restore_at(self, time: float) -> None:
        """Interpolate full scene kinematics, including quaternion joints and mocap bodies."""
        times = [f.metadata.sim_time for f in self._history]
        if not times or time < times[0] - 1e-9 or time > times[-1] + 1e-9:
            raise ValueError(f"snapshot history does not cover {time}")
        index = min(bisect_left(times, time), len(times) - 1)
        after = self._history[index]
        before = self._history[max(0, index - 1)]
        mujoco.mj_setState(self.model, self.data, before.values["state"], STATE)
        duration = after.metadata.sim_time - before.metadata.sim_time
        alpha = (
            0.0 if duration == 0 else np.clip((time - before.metadata.sim_time) / duration, 0, 1)
        )
        if duration > 0:
            if self._next is None:
                self._next = mujoco.MjData(self.model)
            mujoco.mj_setState(self.model, self._next, after.values["state"], STATE)
            velocity = np.empty(self.model.nv)
            mujoco.mj_differentiatePos(
                self.model, velocity, duration, self.data.qpos, self._next.qpos
            )
            mujoco.mj_integratePos(self.model, self.data.qpos, velocity, float(alpha * duration))
            self.data.qvel[:] += alpha * (self._next.qvel - self.data.qvel)
            self.data.mocap_pos[:] += alpha * (self._next.mocap_pos - self.data.mocap_pos)
            q0, q1 = self.data.mocap_quat, self._next.mocap_quat
            q1 = q1 * np.where(np.sum(q0 * q1, axis=1, keepdims=True) < 0, -1, 1)
            q0[:] += alpha * (q1 - q0)
            q0[:] /= np.linalg.norm(q0, axis=1, keepdims=True)
        self.data.time = time
        self.timestamp = float(
            before.values["wall_time"][0]
            + alpha * (after.values["wall_time"][0] - before.values["wall_time"][0])
        )
        mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        self.channel.close()
