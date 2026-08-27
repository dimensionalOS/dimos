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

"""The MuJoCo backend: the seam implemented against the CPU engine.

The plant itself is :mod:`~dimos.robot.unitree.go2.sim.engines.model` — the
vendored scene and the knob application, shared with every other MuJoCo-family
backend. What lives here is the engine binding: one ``mj_step`` per tick, and
the rollout that turns a :class:`~dimos.simulation.sysid.backend.RolloutPlan`
into a :class:`~dimos.simulation.sysid.backend.Prediction`.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import mujoco
import numpy as np

# The plant arrives as a MODULE, not as unbound names: a test that zeroes
# gravity patches `model.load`, and a name imported here would not see it.
from dimos.robot.unitree.go2.sim.engines import model as go2_model
from dimos.robot.unitree.go2.sim.engines.model import FOOT_RADIUS, IMU_SITE
from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS
from dimos.robot.unitree.go2.sim.ranges import KNOBS, PHYSICS_KEYS, Knob
from dimos.simulation.sysid.backend import (
    CHANNELS,
    BaseCondition,
    GhostTrack,
    LoopState,
    Prediction,
    RolloutPlan,
    State,
)
from dimos.simulation.sysid.engines.model import ANCHOR_BODY, GHOST_BODY, mocap_index
from dimos.simulation.sysid.plant import TorqueEnvelope, actuator_step
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat


class MujocoSession:
    """One closed-loop episode in a compiled MuJoCo model.

    The engine's half of Mode B and nothing more: state out, torques in, one
    ``mj_step`` per :meth:`step`. Policy, observation build and every loop
    mechanism live in the generic driver
    (:func:`~dimos.robot.unitree.go2.sim.sysid.ground.rollout_policy`).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        ghost: bool = False,
        view: bool = False,
        view_speed: float = 1.0,
    ) -> None:
        self._model = model
        self._data = data
        self._feet = go2_model.foot_geom_ids(model)
        self._gi = mocap_index(model, GHOST_BODY) if ghost else -1
        self._speed = view_speed
        self._wall: float | None = None
        self._viewer_cm = None
        self._viewer = None
        if view:
            from mujoco import viewer as mj_viewer

            self._viewer_cm = mj_viewer.launch_passive(model, data)
            self._viewer = self._viewer_cm.__enter__()

    @property
    def timestep(self) -> float:
        return float(self._model.opt.timestep)

    def state(self) -> LoopState:
        d = self._data
        return LoopState(
            pos=d.qpos[0:3].copy(),
            quat=d.qpos[3:7].copy(),
            q=d.qpos[7:19].copy(),
            dq=d.qvel[6:18].copy(),
            gyro=d.qvel[3:6].copy(),
        )

    def step(self, ctrl: np.ndarray) -> bool:
        self._data.ctrl[:] = ctrl
        mujoco.mj_step(self._model, self._data)
        if self._viewer is not None:
            import time

            if not self._viewer.is_running():
                return False
            self._viewer.sync()
            if self._wall is None:
                self._wall = time.perf_counter()
            self._wall += self.timestep / max(self._speed, 1e-6)
            lag = self._wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                self._wall = time.perf_counter()
        return True

    def snap(self, state: State) -> None:
        """Mode A's re-init placement, preserving the sim's own yaw and x/y.

        ``state.rot`` arrives yaw-stripped (as Mode A's does); composing the
        sim's current world yaw back on keeps the run continuous — flat
        ground is yaw- and translation-symmetric, so the physics is
        identical and the window bookkeeping stays legible.
        """
        d, m = self._data, self._model
        xy = d.qpos[0:2].copy()
        w, x, y, z = d.qpos[3:7]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        cy, sy = np.cos(yaw), np.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rot = rz @ state.rot
        d.qpos[:] = 0.0
        d.qvel[:] = 0.0
        d.qpos[0:2] = xy
        d.qpos[3:7] = mat_to_quat(rot)
        d.qpos[7:19] = state.q
        d.qpos[2] = 0.30
        mujoco.mj_forward(m, d)
        low = float(np.min(d.geom_xpos[self._feet, 2])) - FOOT_RADIUS
        d.qpos[2] -= low  # lowest foot exactly on the floor
        d.qvel[6:18] = state.dq
        d.qvel[3:6] = rot @ state.gyro  # body rates -> world
        if state.v_body is not None:
            d.qvel[0:3] = rot @ state.v_body
        mujoco.mj_forward(m, d)

    def show_ghost(self, pos: np.ndarray, quat: np.ndarray) -> None:
        if self._gi >= 0:
            self._data.mocap_pos[self._gi] = pos
            self._data.mocap_quat[self._gi] = quat

    def close(self) -> None:
        if self._viewer_cm is not None:
            self._viewer_cm.__exit__(None, None, None)
            self._viewer_cm = None
            self._viewer = None


class MujocoBackend:
    """The menagerie Go2 behind the seam.

    Deterministic by construction: a fresh model is compiled per rollout with
    the currently applied knob values, so no state leaks between rollouts and
    the same plan always yields the same prediction.

    THE VIEWER AND THE HEADLESS RUN ARE THE SAME FUNCTION. ``view=True`` only
    attaches a passive viewer to :meth:`rollout` and paces it to wall clock
    (``view_speed`` < 1 is slow motion — the interesting failures are
    20-50 ms long); the physics, the re-initialisation and every number in
    the returned :class:`Prediction` come from the same code either way. A
    ``ghost`` is drawn re-anchored at every snap, exactly as the scorer maps
    the recorded track (open-loop drift is only defined relative to the pose
    a clip started from). A PINNED (suspended) rollout shows the held,
    swinging trunk, because the viewer watches the same weld the physics
    imposes.
    """

    name = "mujoco"

    def __init__(
        self,
        menagerie: Path | None = None,
        *,
        envelope: TorqueEnvelope | None = None,
        view: bool = False,
        view_speed: float = 1.0,
        ghost: GhostTrack | None = None,
    ) -> None:
        self._menagerie = menagerie
        self._envelope = envelope
        self._view = view
        self._view_speed = view_speed
        self._ghost = ghost if view else None
        self._values: dict[str, float] = {}
        self._dt: float | None = None

    @property
    def timestep(self) -> float:
        if self._dt is None:
            model = mujoco.MjModel.from_xml_path(str(go2_model.scene_path(self._menagerie)))
            self._dt = float(model.opt.timestep)
        return self._dt

    def knobs(self) -> Mapping[str, Knob]:
        return KNOBS

    def channels(self) -> frozenset[str]:
        # MuJoCo predicts everything the robot measures: joints from qpos, the
        # virtual IMU at the sensor site, delivered torque from the actuator
        # chain, and the base pose the tracker would see.
        return frozenset(CHANNELS)

    def apply(self, values: Mapping[str, float]) -> None:
        unknown = set(values) - set(KNOBS)
        if unknown:
            raise ValueError(f"unknown knob(s) for {self.name}: {sorted(unknown)}")
        self._values = dict(values)

    def session(
        self,
        pose: np.ndarray,
        *,
        ghost: bool = False,
        view: bool = False,
        view_speed: float = 1.0,
    ) -> MujocoSession:
        """A closed-loop episode: applied physics, keyframe home, joints at ``pose``."""
        model, data = go2_model.load(self._menagerie, ghost=ghost)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            go2_model.apply_physics(model, physics)
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
        data.qpos[7:19] = pose
        mujoco.mj_forward(model, data)
        return MujocoSession(model, data, ghost=ghost, view=view, view_speed=view_speed)

    def rollout(self, plan: RolloutPlan) -> Prediction:
        pinned = plan.base is BaseCondition.PINNED
        ghost = self._ghost
        model, data = go2_model.load(self._menagerie, pinned=pinned, ghost=ghost is not None)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            go2_model.apply_physics(model, physics)
        actuator_tau = self._values.get("actuator_tau", 0.0)
        dt = float(model.opt.timestep)
        feet = go2_model.foot_geom_ids(model)
        track = plan.base_track if pinned else None
        ai = mocap_index(model, ANCHOR_BODY) if pinned else -1
        gi = mocap_index(model, GHOST_BODY) if ghost is not None else -1
        # Rigid map from the ghost's room frame into this rollout's world,
        # recomputed at every snap — the same convention the scorer uses:
        # open-loop drift is only defined relative to the pose a clip
        # started from, and a fixed anchor reads the snap itself as error.
        g_anchor_r = np.eye(3)
        g_anchor_p = np.zeros(3)
        # World-yaw offset between the sim's (yaw-stripped) pose and the
        # measured attitude, refreshed at every snap: anchor @ track.rot(t)
        # then has the measured gravity direction through the legs at EVERY
        # sample, not just at clip starts — the robot swings on the rope.
        anchor = np.eye(3)

        def track_rot(t: float) -> np.ndarray:
            assert track is not None
            k = int(np.clip(np.searchsorted(track.t, t, "right") - 1, 0, len(track.t) - 1))
            out: np.ndarray = track.rot[k]
            return out

        reinit = list(plan.reinit)
        if not reinit or reinit[0].t > plan.t0:
            raise ValueError("plan.reinit must start with the state at t0")

        def snap(state: State) -> None:
            nonlocal anchor, g_anchor_r, g_anchor_p
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[3:7] = mat_to_quat(state.rot)
            data.qpos[7:19] = state.q
            data.qpos[2] = 0.30
            mujoco.mj_forward(model, data)
            low = float(np.min(data.geom_xpos[feet, 2])) - FOOT_RADIUS
            data.qpos[2] -= low  # lowest foot exactly on the floor
            data.qvel[6:18] = state.dq
            data.qvel[3:6] = state.rot @ state.gyro  # body rates -> world
            if state.v_body is not None:
                data.qvel[0:3] = state.rot @ state.v_body
            if pinned:
                data.qpos[2] = max(float(data.qpos[2]), 2.0)  # clear of the floor
                data.mocap_pos[ai] = data.qpos[0:3]
                data.mocap_quat[ai] = data.qpos[3:7]
                if track is not None:
                    anchor = state.rot @ track_rot(state.t).T
            mujoco.mj_forward(model, data)
            if ghost is not None:
                j = int(
                    np.clip(np.searchsorted(ghost.t, state.t, "right") - 1, 0, len(ghost.t) - 1)
                )
                g_anchor_r = quat_to_mat(data.qpos[3:7].copy()) @ ghost.rot[j].T
                g_anchor_p = data.qpos[0:3].copy() - g_anchor_r @ ghost.pos[j]

        snap(reinit[0])

        # The virtual IMU: `mj_objectAcceleration(..., flg_local=1)` returns
        # [angular, linear] in local coordinates and MuJoCo seeds the world
        # body with -gravity, so the linear half IS specific force — ~9.8
        # standing, ~0 in free fall — the same convention as rt/lowstate's
        # accelerometer. Read AT THE `imu` SITE: 49 mm of lever arm is worth
        # 1.8x on the landing residual (15.33 vs 8.61 RMS against a 9.01
        # signal — read at the frame it is worse than predicting zero).
        imu_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, IMU_SITE)
        imu_objtype = mujoco.mjtObj.mjOBJ_SITE
        if imu_id < 0:  # a model without the sensor's mount point: fall back
            imu_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            imu_objtype = mujoco.mjtObj.mjOBJ_BODY
        acc6 = np.zeros(6)

        cmds = plan.commands
        applied = np.zeros(12)
        ts: list[float] = []
        qs: list[np.ndarray] = []
        dqs: list[np.ndarray] = []
        ps: list[np.ndarray] = []
        rs: list[np.ndarray] = []
        ats: list[float] = []
        asim: list[np.ndarray] = []
        wsim: list[np.ndarray] = []
        tsim: list[np.ndarray] = []
        reinit_t: list[float] = [plan.t0]
        reinit_pos: list[np.ndarray] = [data.qpos[0:3].copy()]
        reinit_rot: list[np.ndarray] = [quat_to_mat(data.qpos[3:7].copy())]
        si = 1

        viewer_cm = None
        if self._view:
            from mujoco import viewer as mj_viewer

            viewer_cm = mj_viewer.launch_passive(model, data)
        viewer = viewer_cm.__enter__() if viewer_cm is not None else None

        try:
            import time

            wall = time.perf_counter()
            n = int(plan.duration / dt)
            for step in range(n):
                t = plan.t0 + step * dt
                if si < len(reinit) and t >= reinit[si].t:
                    snap(reinit[si])
                    applied[:] = 0.0
                    reinit_t.append(t)
                    reinit_pos.append(data.qpos[0:3].copy())
                    reinit_rot.append(quat_to_mat(data.qpos[3:7].copy()))
                    si += 1

                k = int(np.clip(np.searchsorted(cmds.t, t, "right") - 1, 0, len(cmds.t) - 1))
                tau = (
                    cmds.kp[k] * (cmds.q[k] - data.qpos[7:19])
                    + cmds.kd[k] * (cmds.dq[k] - data.qvel[6:18])
                    + cmds.tau_ff[k]
                )
                tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
                applied = actuator_step(
                    applied, tau, dt, actuator_tau, dq=data.qvel[6:18], envelope=self._envelope
                )
                data.ctrl[:] = applied
                if track is not None:
                    # The measured trunk attitude, re-anchored at the last snap:
                    # the weld drags the trunk through the swing the robot
                    # actually made, so gravity loads the legs the measured way.
                    data.mocap_quat[ai] = mat_to_quat(anchor @ track_rot(t))
                mujoco.mj_step(model, data)

                # Every step, not every fifth: an impact is ~20 ms wide.
                # Both exist at runtime; the bundled mujoco stubs omit them.
                mujoco.mj_rnePostConstraint(model, data)  # type: ignore[attr-defined]
                mujoco.mj_objectAcceleration(  # type: ignore[attr-defined]
                    model, data, imu_objtype, imu_id, acc6, 1
                )
                ats.append(t)
                asim.append(acc6[3:].copy())
                # Angular rate in the trunk's own frame — the convention
                # rt/lowstate's gyroscope reports in.
                wsim.append(quat_to_mat(data.qpos[3:7]).T @ data.qvel[3:6])
                tsim.append(applied.copy())

                if step % 5 == 0:  # pose-rate log at 100 Hz
                    ts.append(t)
                    qs.append(data.qpos[7:19].copy())
                    dqs.append(data.qvel[6:18].copy())
                    ps.append(data.qpos[0:3].copy())
                    rs.append(quat_to_mat(data.qpos[3:7].copy()))
                    if ghost is not None:
                        j = int(
                            np.clip(np.searchsorted(ghost.t, t, "right") - 1, 0, len(ghost.t) - 1)
                        )
                        data.mocap_pos[gi] = g_anchor_r @ ghost.pos[j] + g_anchor_p
                        data.mocap_quat[gi] = mat_to_quat(g_anchor_r @ ghost.rot[j])

                if viewer is not None:
                    if not viewer.is_running():
                        break
                    viewer.sync()
                    wall += dt / max(self._view_speed, 1e-6)
                    lag = wall - time.perf_counter()
                    if lag > 0:
                        time.sleep(lag)
                    else:
                        wall = time.perf_counter()
        finally:
            if viewer_cm is not None:
                viewer_cm.__exit__(None, None, None)

        return Prediction(
            t=np.array(ts),
            q=np.array(qs).reshape(-1, 12),
            dq=np.array(dqs).reshape(-1, 12),
            body_pos=np.array(ps).reshape(-1, 3),
            body_rot=np.array(rs).reshape(-1, 3, 3),
            at=np.array(ats),
            imu_accel=np.array(asim).reshape(-1, 3),
            imu_gyro=np.array(wsim).reshape(-1, 3),
            tau=np.array(tsim).reshape(-1, 12),
            reinit_t=np.array(reinit_t),
            reinit_pos=np.array(reinit_pos).reshape(-1, 3),
            reinit_rot=np.array(reinit_rot).reshape(-1, 3, 3),
        )
