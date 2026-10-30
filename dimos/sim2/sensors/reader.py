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

"""Worker-local MuJoCo model and data, restored from the world's snapshot stream."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections import deque
import threading
from typing import Any

import mujoco
import numpy as np

from dimos.sim2.backends.mujoco import STATE
from dimos.sim2.messages import FrameMetadata, StateFrame


class WorldReader:
    """Keeps a bounded snapshot history and restores any instant of it into local data.

    ``push`` is the subscriber callback for the world's ``snapshots`` stream.
    ``update`` restores the newest frame; ``history`` and ``restore_at`` serve
    devices that integrate over an interval, such as a rolling lidar.
    """

    def __init__(self, description: dict[str, Any]) -> None:
        self._model_path = str(description["model"])
        self._model_prefixes = description.get("model_prefixes", {})
        self._root_bodies = description.get("root_bodies", {})
        self.model = mujoco.MjModel.from_binary_path(self._model_path)
        self.data = mujoco.MjData(self.model)
        self.period = float(description["physics_dt"]) * int(description["snapshot_stride"])
        self._frames: deque[StateFrame] = deque(maxlen=int(description["snapshot_slots"]))
        self._lock = threading.Lock()
        self.sequence = -1
        self.timestamp = 0.0
        self.episode = -1
        self.metadata: FrameMetadata | None = None
        self._history: tuple[StateFrame, ...] = ()
        self._next: mujoco.MjData | None = None

    def model_prefix(self, robot_id: str | None) -> str:
        """Resolve upstream model names independently of public stream/frame names."""
        if robot_id is None:
            return ""
        return str(self._model_prefixes.get(robot_id, f"{robot_id}/"))

    def root_body(self, robot_id: str | None) -> str | None:
        """The upstream robot's root, without its namespace prefix."""
        value = self._root_bodies.get(robot_id)
        return str(value) if value is not None else None

    def push(self, frame: StateFrame) -> None:
        """Accept a published snapshot; older sequences and repeated paused ticks are dropped."""
        with self._lock:
            if self._frames:
                last = self._frames[-1]
                if frame.metadata.sequence <= last.metadata.sequence:
                    return
                if (
                    frame.metadata.episode_id != last.metadata.episode_id
                    or frame.model_path != last.model_path
                ):
                    self._frames.clear()
                elif frame.metadata.physics_tick == last.metadata.physics_tick:
                    self._frames.pop()
            self._frames.append(frame)

    def frames(self) -> tuple[StateFrame, ...]:
        with self._lock:
            return tuple(self._frames)

    @property
    def latest_episode(self) -> int:
        """Episode of the newest received frame, which may be newer than the restored one."""
        with self._lock:
            return self._frames[-1].metadata.episode_id if self._frames else -1

    def update(self) -> bool:
        with self._lock:
            frame = self._frames[-1] if self._frames else None
        if frame is None:
            return False
        changed = frame.metadata.sequence != self.sequence
        if frame.model_path is not None and str(frame.model_path) != self._model_path:
            # Reload on the acquisition thread, never in the transport callback.
            self._model_path = str(frame.model_path)
            self.model = mujoco.MjModel.from_binary_path(self._model_path)
            self.data = mujoco.MjData(self.model)
            self._next = None
            self._history = ()
        self.restore(frame)
        self.sequence = frame.metadata.sequence
        self.episode = frame.metadata.episode_id
        self.metadata = frame.metadata
        return changed

    def restore(self, frame: StateFrame) -> None:
        """Restore an exact snapshot, including MuJoCo's mounted IMU readings."""
        mujoco.mj_setState(self.model, self.data, frame.values["state"], STATE)
        mujoco.mj_forward(self.model, self.data)
        self.timestamp = float(frame.values["wall_time"][0])

    def history(self, start: float, end: float) -> bool:
        """Pin a complete interval from this episode; never invent missing motion."""
        self._history = tuple(f for f in self.frames() if f.metadata.episode_id == self.episode)
        times = np.array([f.metadata.sim_time for f in self._history])
        first = max(0, bisect_right(times, start) - 1)
        last = bisect_left(times, end) + 1
        self._history = self._history[first:last]
        times = times[first:last]
        if (
            len(times) == 0
            or times[0] > start + 1e-9
            or times[-1] < end - 1e-9
            or np.any(np.diff(times) > self.period * 1.5)
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
