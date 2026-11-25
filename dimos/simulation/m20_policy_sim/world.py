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

"""The vendor M20 MJCF in MuJoCo, driven like the real motor bus: 16 x (kp, q, kd, dq, tau) every 1 ms.

Visual meshes are stripped (collision geoms are primitives), so it runs headless and fast. Terrains: flat, a step
(box) ahead, stairs, a charging dock (a back wall with two contact pads) behind the robot, and the reflector-post
charging dock ahead of it ("charge_dock": two retro-reflective posts the lidar finds, which World.lidar_scan sees).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re

import mujoco
import numpy as np

from dimos.simulation.m20_policy_sim.controller import JointCommand
from dimos.simulation.m20_policy_sim.runner import RobotState

WHEEL_RADIUS = 0.09


def m20_mjcf_path() -> str:
    from dimos.robot.deeprobotics.m20.model import M20_MJCF

    return str(M20_MJCF)


@dataclass
class Terrain:
    kind: str = "flat"  # flat | step | stairs | dock | charge_dock
    # step / stairs: first riser this far ahead of the start (m), height and depth of each step
    distance: float = 1.0
    height: float = 0.08
    depth: float = 0.3
    steps: int = 1
    # dock: wall this far behind the start (m)
    dock_distance: float = 1.0
    # charge_dock: the midpoint of the two reflector posts at (post_x, post_y) in the world, the dock's outward axis
    # (pointing from the posts toward an arriving robot) at -x turned by post_yaw (rad); posts post_spacing apart (m)
    post_x: float = 2.6
    post_y: float = 0.0
    post_yaw: float = 0.0
    post_spacing: float = 0.374
    post_radius: float = 0.025
    post_height: float = 0.6


def terrain_xml(t: Terrain) -> str:
    geoms = []
    if t.kind in ("step", "stairs"):
        for k in range(t.steps):
            top = (k + 1) * t.height
            x0 = t.distance + k * t.depth
            length = 20.0 if k == t.steps - 1 else t.depth
            geoms.append(
                f'<geom name="step{k}" type="box" pos="{x0 + length / 2} 0 {top / 2}" size="{length / 2} 2 {top / 2}" '
                f'friction="1 0.01 0.01" rgba="0.6 0.6 0.7 1"/>'
            )
    if t.kind == "dock":
        x = -t.dock_distance
        geoms.append(
            f'<geom name="dock_wall" type="box" pos="{x - 0.1} 0 0.25" size="0.1 0.4 0.25" rgba="0.3 0.3 0.3 1"/>'
        )
        for side in (-1, 1):
            geoms.append(
                f'<geom name="dock_pad{side}" type="box" pos="{x + 0.05} {side * 0.15} 0.35" size="0.05 0.04 0.03" '
                f'contype="0" conaffinity="0" rgba="0.9 0.6 0.1 1"/>'
            )
    if t.kind == "charge_dock":
        out = np.array([-math.cos(t.post_yaw), -math.sin(t.post_yaw)])
        across = np.array([-out[1], out[0]])
        mid = np.array([t.post_x, t.post_y])
        for k, sign in enumerate((-1, 1)):
            x, y = mid + sign * t.post_spacing / 2 * across
            geoms.append(
                f'<geom name="reflector{k}" type="cylinder" pos="{x} {y} {t.post_height / 2}" '
                f'size="{t.post_radius} {t.post_height / 2}" rgba="0.95 0.95 0.95 1"/>'
            )
        # the dock's body behind the posts: plain (low-intensity) surfaces the detector must ignore
        bx, by = mid - 0.25 * out
        qw, qz = math.cos(t.post_yaw / 2), math.sin(t.post_yaw / 2)
        geoms.append(
            f'<geom name="dock_body" type="box" pos="{bx} {by} 0.2" size="0.15 0.5 0.2" quat="{qw} 0 0 {qz}" '
            f'rgba="0.3 0.3 0.3 1"/>'
        )
    return "\n".join(geoms)


# lidar_scan's surfaces: a retro-reflector returns this intensity (Airy's 0-255 reflectivity), anything else REFLECTIVITY
REFLECTOR_INTENSITY = 250.0
REFLECTIVITY = 30.0


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll), as tf's roll-pitch-yaw."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


class World:
    def __init__(
        self, terrain: Terrain | None = None, torque_limit: np.ndarray | None = None
    ) -> None:
        self.terrain = terrain or Terrain()
        xml = open(m20_mjcf_path()).read()
        xml = re.sub(r'<mesh name="[^"]*" file="[^"]*"\s*/>', "", xml)
        xml = re.sub(r'<geom type="mesh"[^>]*/>', "", xml)
        xml = xml.replace('meshdir="../meshes/"', "")
        xml = xml.replace("</worldbody>", terrain_xml(self.terrain) + "\n</worldbody>", 1)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.model.opt.timestep = 0.001
        self.data = mujoco.MjData(self.model)
        self.torque_lo = self.model.actuator_ctrlrange[:, 0].copy()
        self.torque_hi = self.model.actuator_ctrlrange[:, 1].copy()
        if torque_limit is not None:
            self.torque_lo, self.torque_hi = -torque_limit, torque_limit
        self.wheel_bodies = [self.model.body(f"{leg}_wheel").id for leg in ("fl", "fr", "hl", "hr")]
        self.base = self.model.body("base_link").id
        self.time = 0.0

    def reset(
        self, pose: np.ndarray, yaw: float = 0.0, xy: tuple[float, float] = (0.0, 0.0)
    ) -> None:
        """Place the robot with joint pose `pose`, its lowest wheel touching the ground, and settle 0 s."""
        d = self.data
        mujoco.mj_resetData(self.model, d)  # type: ignore[attr-defined]  # missing from stubs/mujoco
        d.qpos[0:2] = xy
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[7:23] = pose
        d.qpos[2] = 1.0
        mujoco.mj_forward(self.model, d)
        lowest = min(
            min(d.xpos[b][2] for b in self.wheel_bodies) - WHEEL_RADIUS,
            d.xpos[self.base][2] - 0.12,
        )
        d.qpos[2] -= lowest - 0.002
        mujoco.mj_forward(self.model, d)
        self.time = 0.0

    def state(self) -> RobotState:
        d = self.data
        return RobotState(
            quat=d.qpos[3:7].copy(),
            gyro=d.qvel[3:6].copy(),
            accel=d.sensordata[4:7].copy(),
            q=d.qpos[7:23].copy(),
            dq=d.qvel[6:22].copy(),
            tau=d.ctrl.copy(),
            height_map=None,
            stamp=self.time,
        )

    def step(self, cmd: JointCommand, substeps: int = 1) -> None:
        d = self.data
        for _ in range(substeps):
            q, dq = d.qpos[7:23], d.qvel[6:22]
            tau = cmd.kp * (cmd.q - q) + cmd.kd * (cmd.dq - dq) + cmd.tau
            d.ctrl[:] = np.clip(tau, self.torque_lo, self.torque_hi)
            mujoco.mj_step(self.model, d)
            self.time += self.model.opt.timestep

    # ground truth for scoring
    def base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()

    def base_velocity_body(self) -> np.ndarray:
        """Body-frame (vx, vy) and world yaw rate."""
        from dimos.simulation.m20_policy_sim.runner import rotation

        R = rotation(self.data.qpos[3:7])
        v = R.T @ self.data.qvel[0:3]
        return np.array([v[0], v[1], self.data.qvel[5]])

    def body_contacts_ground(self) -> bool:
        """True when the trunk (not a wheel) touches the floor or terrain."""
        m, d = self.model, self.data
        for k in range(d.ncon):
            c = d.contact[k]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if self.base in (b1, b2) and 0 in (b1, b2):
                return True
        return False

    def height_map(
        self,
        rows: int,
        cols: int,
        spacing_x: float,
        spacing_y: float,
        offset: float,
        min_value: float,
        fill: float,
        min_row: int,
        front_first: bool,
    ) -> np.ndarray:
        """rl_deploy's HeightGap input from a yaw-aligned grid around the base (HeightsSubCallback): per cell
        v = (base height above the terrain) - offset, kept when v > min_value and row >= min_row, else fill."""
        d = self.data
        base = d.qpos[0:3]
        w, x, y, z = d.qpos[3:7]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, s = np.cos(yaw), np.sin(yaw)
        xs = (np.arange(rows) - (rows - 1) / 2) * spacing_x
        if front_first:
            xs = xs[::-1]
        ys = (np.arange(cols) - (cols - 1) / 2) * spacing_y
        out = np.full((rows, cols), fill)
        geomid = np.zeros(1, np.int32)
        group = np.array([1, 0, 0, 0, 0, 0], np.uint8)  # group 0 = floor and terrain
        for i, px in enumerate(xs):
            if i < min_row:
                continue
            for j, py in enumerate(ys):
                pt = np.array([base[0] + c * px - s * py, base[1] + s * px + c * py, 2.0])
                dist = mujoco.mj_ray(  # type: ignore[attr-defined]  # missing from stubs/mujoco
                    self.model, d, pt, np.array([0, 0, -1.0]), group, 1, -1, geomid
                )
                v = base[2] - (2.0 - dist if dist >= 0 else 0.0) - offset
                if v > min_value:
                    out[i, j] = v
        return out.ravel()

    def lidar_scan(
        self,
        mount_xyz: tuple[float, float, float],
        mount_rpy: tuple[float, float, float],
        elevations_deg: np.ndarray,
        azimuths_deg: np.ndarray,
        range_noise_m: float,
        rng: np.random.Generator,
        max_range_m: float = 30.0,
    ) -> np.ndarray:
        """One sweep of a spinning lidar mounted on base_link at (mount_xyz, mount_rpy), in the lidar's own frame (an
        Airy's: z the spin axis, elevation measured from its x-y plane toward +z): rows of x, y, z, intensity. Only the
        world (geom group 0) is seen, never the robot; ranges get Gaussian noise of range_noise_m."""
        d = self.data
        rot_base = np.asarray(d.xmat[self.base]).reshape(3, 3)
        rot_mount = rotation_rpy(*mount_rpy)
        origin = d.xpos[self.base] + rot_base @ np.asarray(mount_xyz, np.float64)
        el, az = np.meshgrid(np.radians(elevations_deg), np.radians(azimuths_deg), indexing="ij")
        local = np.stack(
            [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1
        ).reshape(-1, 3)
        world_dirs = local @ (rot_base @ rot_mount).T
        n = len(local)
        geomid = np.zeros(n, np.int32)
        dist = np.zeros(n, np.float64)
        group = np.array([1, 0, 0, 0, 0, 0], np.uint8)
        mujoco.mj_multiRay(
            self.model,
            d,
            origin,
            np.ascontiguousarray(world_dirs).ravel(),
            group,
            1,
            -1,
            geomid,
            dist,
            None,
            n,
            max_range_m,
        )
        hit = (geomid >= 0) & (dist > 0)
        names = [self.model.geom(int(g)).name for g in geomid[hit]]
        intensity = np.array(
            [
                REFLECTOR_INTENSITY if name.startswith("reflector") else REFLECTIVITY
                for name in names
            ]
        )
        r = dist[hit] + rng.normal(0.0, range_noise_m, hit.sum())
        points = local[hit] * r[:, None]
        return np.column_stack([points, intensity]) if len(points) else np.zeros((0, 4))
