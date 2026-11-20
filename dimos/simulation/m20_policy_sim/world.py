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
(box) ahead, stairs, and a charging dock (a back wall with two contact pads) behind the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
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
    kind: str = "flat"  # flat | step | stairs | dock
    # step / stairs: first riser this far ahead of the start (m), height and depth of each step
    distance: float = 1.0
    height: float = 0.08
    depth: float = 0.3
    steps: int = 1
    # dock: wall this far behind the start (m)
    dock_distance: float = 1.0


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
    return "\n".join(geoms)


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
        mujoco.mj_resetData(self.model, d)
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
                dist = mujoco.mj_ray(
                    self.model, d, pt, np.array([0, 0, -1.0]), group, 1, -1, geomid
                )
                v = base[2] - (2.0 - dist if dist >= 0 else 0.0) - offset
                if v > min_value:
                    out[i, j] = v
        return out.ravel()
