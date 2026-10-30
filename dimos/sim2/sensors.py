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

"""Read-only native query replica: RGB/depth, timed lidar and solver IMU samples.

This object never calls mj_step or owns dynamics. All native query/render state
belongs to its acquisition thread. Physics sends portable snapshots at 50 Hz.
"""

from collections import deque

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.mujoco import compile_world
from dimos.sim2.types import Array, Camera, Layout, Lidar, Scan, World, WorldState


class MujocoSensors:
    def __init__(self, world: World, layout: Layout) -> None:
        if world.revision != layout.revision:
            raise ValueError("sensor/world description mismatch")
        self.model = compile_world(world)
        self.data = mujoco.MjData(self.model)
        self.layout = layout
        self.history: deque[WorldState] = deque(maxlen=64)
        self.gaps = 0
        self._renderer: mujoco.Renderer | None = None
        self._cameras = {s.frame: s for s in world.sensors if isinstance(s, Camera)}
        self._lidars = {s.frame: s for s in world.sensors if isinstance(s, Lidar)}
        self._rays: dict[str, Array] = {}
        for frame, profile in self._lidars.items():
            if (
                profile.rays <= 0
                or profile.packets <= 0
                or profile.rays % profile.packets
                or not profile.elevations
                or not 0 < profile.min_range < profile.max_range
            ):
                raise ValueError("invalid lidar profile")
            azimuth = np.linspace(-np.pi, np.pi, profile.rays, endpoint=False)
            elevation = np.resize(profile.elevations, profile.rays)
            self._rays[frame] = np.column_stack(
                (
                    np.cos(azimuth) * np.cos(elevation),
                    np.sin(azimuth) * np.cos(elevation),
                    np.sin(elevation),
                )
            )

        self._q = np.array([self.model.joint(n).qposadr[0] for n in layout.joints])
        self._free = [
            (layout.bodies.index(n), int(self.model.body(n).jntadr[0]))
            for n in layout.roots
            if self.model.body(n).jntnum[0] == 1
            and self.model.jnt_type[self.model.body(n).jntadr[0]] == mujoco.mjtJoint.mjJNT_FREE
        ]

    def observe(self, state: WorldState) -> None:
        if state.revision != self.layout.revision:
            raise ValueError("state belongs to a different model revision")
        if self.history and state.epoch != self.history[-1].epoch:
            self.history.clear()
        if self.history and state.tick <= self.history[-1].tick:
            return
        self.history.append(state)

    def _load(self, state: WorldState) -> None:
        self.data.qpos[self._q] = state.position
        for index, joint in self._free:
            pose = state.poses[index]
            q = self.model.jnt_qposadr[joint]
            self.data.qpos[q : q + 7] = (*pose[:3], *pose[[6, 3, 4, 5]])
        self.data.time = state.sim_time
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)

    def camera(
        self, frame: str, state: WorldState
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        self._load(state)
        profile = self._cameras[frame]
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=profile.height, width=profile.width)
        self._renderer.update_scene(self.data, camera=frame)
        self._renderer.disable_depth_rendering()
        rgb = self._renderer.render().copy()
        self._renderer.enable_depth_rendering()
        depth = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()
        return rgb, depth

    def lidar(self, frame: str, duration: float = 0.1) -> Scan | None:
        """192 rays in six timestamped packets, using preceding samples (no extrapolation).

        Reject a scan if history is missing or any packet is more than 30 ms old.
        World-frame output keeps moving-frame scans correct without pretending
        every point was acquired at the final pose. This is ideal ray lidar.
        """
        if not self.history:
            return None
        end = self.history[-1]
        profile = self._lidars[frame]
        times = np.linspace(end.sim_time - duration, end.sim_time, profile.packets)
        if times[0] < self.history[0].sim_time - 1e-9:
            self.gaps += 1
            return None
        points, stamps = [], []
        body = self.model.body(frame).id
        directions = self._rays[frame]
        count = profile.rays // profile.packets
        for batch, stamp in enumerate(times):
            state = next((s for s in reversed(self.history) if s.sim_time <= stamp + 1e-9), None)
            if state is None or stamp - state.sim_time > profile.max_pose_age:
                self.gaps += 1
                return None
            self._load(state)
            origin = self.data.xpos[body].copy()
            rotation = self.data.xmat[body].reshape(3, 3)
            for direction in directions[batch * count : (batch + 1) * count]:
                ray = rotation @ direction
                hit = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(self.model, self.data, origin, ray, None, True, body, hit)
                if profile.min_range <= distance <= profile.max_range:
                    points.append(origin + ray * distance)
                    stamps.append(state.sim_time)
        return Scan(np.asarray(points).reshape((-1, 3)), np.asarray(stamps), end.epoch)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
