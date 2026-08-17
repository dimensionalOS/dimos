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

"""The MuJoCo backend: menagerie scene, physics overrides, the seam implemented.

This is the one module that imports the engine. The seam it implements
(:class:`~dimos.robot.unitree.go2.sim.backend.Backend`) deliberately does not.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim.backend import (
    CHANNELS,
    BaseCondition,
    GhostTrack,
    LoopState,
    Prediction,
    RolloutPlan,
    State,
)
from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS, TorqueEnvelope, actuator_step
from dimos.robot.unitree.go2.sim.ranges import KNOBS, PHYSICS_KEYS, Knob
from dimos.robot.unitree.go2.sim.rotations import mat_to_quat, quat_to_mat

# Where the accelerometer actually is: the menagerie model's `imu` site,
# (-0.02557, 0, 0.04232) — 49 mm from the body origin. The virtual IMU must be
# read THERE, not at the trunk frame; see MujocoBackend.rollout below.
IMU_SITE = "imu"

FOOT_GEOMS = ("FL", "FR", "RL", "RR")
FOOT_RADIUS = 0.022

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve joints.
LEG_DOFS = slice(6, 18)

# The rope, as MuJoCo can express it: a mocap body welded to the trunk, so the
# trunk is held DURING mj_step and gravity loads the legs. (A post-step snap
# lets the whole robot free-fall within the step and the legs end up in a
# weightless plant — see BaseCondition.PINNED.) Stiff on purpose: with these
# values the hold error is sub-micrometre / sub-microradian over seconds, so
# the weld is a rigid grip on the measured pose, not a spring to identify.
ANCHOR_BODY = "trunk_anchor"
WELD_SOLREF = (0.004, 1.0)  # timeconst = 2 * the scene's 2 ms step, critical damping
WELD_SOLIMP = (0.999, 0.9999, 0.001, 0.5, 2.0)

# The recorded pose drawn beside the sim under --view. Visual only: its geom
# collides with nothing, so the watched physics is the scored physics.
GHOST_BODY = "ghost"


def scene_path(menagerie: Path | None = None) -> Path:
    """Path to the flat-ground go2 scene (menagerie ``unitree_go2/scene.xml``).

    Assets are not vendored: point ``MUJOCO_MENAGERIE`` at a mujoco_menagerie
    checkout, or install ``mujoco_playground``, which manages one.
    """
    root = menagerie or _menagerie_root()
    scene = root / "unitree_go2" / "scene.xml"
    if not scene.is_file():
        raise FileNotFoundError(
            f"go2 scene not found at {scene}. Point MUJOCO_MENAGERIE at a "
            "mujoco_menagerie checkout."
        )
    return scene


def _menagerie_root() -> Path:
    env = os.environ.get("MUJOCO_MENAGERIE")
    if env:
        return Path(env)
    # mujoco_playground maintains a checkout of its own, but importing it drags
    # in mjx -> mujoco_warp, which fails on an mjtEnableBit the installed mujoco
    # does not have. That surfaces as AttributeError from a third-party module,
    # not ImportError, so catch broadly: this is a PATH LOOKUP, and no failure
    # inside an optional dependency should be able to take the package down.
    try:
        from mujoco_playground._src import mjx_env

        return Path(str(mjx_env.MENAGERIE_PATH))
    except Exception:
        pass
    raise FileNotFoundError(
        "no menagerie checkout: set MUJOCO_MENAGERIE to a mujoco_menagerie "
        "clone, or install mujoco_playground"
    )


def load(
    menagerie: Path | None = None, *, pinned: bool = False, ghost: bool = False
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The compiled scene; ``pinned`` adds the rope (mocap anchor + weld),
    ``ghost`` a translucent mocap box a viewer drives with the recorded pose.

    The plain path compiles the menagerie XML untouched — bit-identical to
    every rollout ever scored on it. The PINNED path adds a mocap body and an
    ``mjEQ_WELD`` holding the trunk to it; the mocap body carries no joints
    and no geoms, so nq/nv and every contact pair are unchanged. The GHOST
    body is mocap with a contype/conaffinity 0 geom — visual only, it can
    touch nothing — so attaching it never moves the physics being watched.
    """
    if not pinned and not ghost:
        model = mujoco.MjModel.from_xml_path(str(scene_path(menagerie)))
        return model, mujoco.MjData(model)
    spec = mujoco.MjSpec.from_file(str(scene_path(menagerie)))
    if pinned:
        spec.worldbody.add_body(name=ANCHOR_BODY, mocap=True)
        weld = spec.add_equality()
        weld.type = mujoco.mjtEq.mjEQ_WELD  # type: ignore[attr-defined]  # absent from the bundled stubs
        weld.objtype = mujoco.mjtObj.mjOBJ_BODY
        weld.name1 = ANCHOR_BODY
        weld.name2 = "base"
        weld.data[:] = 0.0
        weld.data[6] = 1.0  # relpose quat = identity: base coincides with the anchor
        weld.data[10] = 1.0  # torquescale: the rope reacts torque, not just force
        weld.solref = WELD_SOLREF
        weld.solimp = WELD_SOLIMP
    if ghost:
        body = spec.worldbody.add_body(name=GHOST_BODY, mocap=True)
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [0.1881, 0.04675, 0.057]  # the URDF base collision box
        geom.rgba = [0.2, 1.0, 0.2, 0.35]
        geom.contype = 0
        geom.conaffinity = 0
    model = spec.compile()
    return model, mujoco.MjData(model)


def mocap_index(model: mujoco.MjModel, name: str) -> int:
    """The mocap slot of a named mocap body — never assume it is 0."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"no body named {name!r} in this model")
    return int(model.body_mocapid[bid])


def apply_physics(model: mujoco.MjModel, overrides: dict[str, float]) -> None:
    """Patch plant knob values onto a compiled model, in place.

    Keys are :data:`~dimos.robot.unitree.go2.sim.ranges.PHYSICS_KEYS`; anything
    else raises. An ABSENT key is never written, so the menagerie default
    stands and a preset that omits a knob reproduces older scores bit-for-bit.

    The foot geom has contact priority 1 and the floor 0, so MuJoCo takes the
    FOOT's friction/solref/solimp verbatim for the pair and ignores the
    floor's: a "softer floor" is expressed by softening the foot.
    """
    unknown = set(overrides) - PHYSICS_KEYS
    if unknown:
        raise ValueError(f"unknown physics override(s): {sorted(unknown)}")
    if "armature" in overrides:
        model.dof_armature[LEG_DOFS] = overrides["armature"]
    if "damping" in overrides:
        model.dof_damping[LEG_DOFS] = overrides["damping"]
    if "frictionloss" in overrides:
        model.dof_frictionloss[LEG_DOFS] = overrides["frictionloss"]
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    if "trunk_mass_scale" in overrides:
        model.body_mass[trunk] *= overrides["trunk_mass_scale"]
    if "trunk_inertia_scale" in overrides:
        model.body_inertia[trunk] *= overrides["trunk_inertia_scale"]
    if "trunk_com_x" in overrides:
        model.body_ipos[trunk][0] += overrides["trunk_com_x"]
    if "leg_mass_scale" in overrides:
        for prefix in FOOT_GEOMS:
            for part in ("thigh", "calf"):
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_{part}")
                model.body_mass[bid] *= overrides["leg_mass_scale"]
                model.body_inertia[bid] *= overrides["leg_mass_scale"]
    # geom_friction columns are (tangential, torsional, rolling); solref is
    # (timeconst, dampratio); solimp is (dmin, dmax, width, mid, power). Only
    # dmin and width are writable: dmax, mid and power shape the impedance
    # sigmoid without changing what it starts and ends at, and nothing in the
    # data could tell them apart from width.
    for name in FOOT_GEOMS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if "foot_friction" in overrides:
            model.geom_friction[gid, 0] = overrides["foot_friction"]
        if "foot_friction_torsional" in overrides:
            model.geom_friction[gid, 1] = overrides["foot_friction_torsional"]
        if "foot_solref_time" in overrides:
            model.geom_solref[gid, 0] = overrides["foot_solref_time"]
        if "foot_solref_damp" in overrides:
            model.geom_solref[gid, 1] = overrides["foot_solref_damp"]
        if "foot_solimp_dmin" in overrides:
            model.geom_solimp[gid, 0] = overrides["foot_solimp_dmin"]
        if "foot_solimp_width" in overrides:
            model.geom_solimp[gid, 2] = overrides["foot_solimp_width"]


def foot_geom_ids(model: mujoco.MjModel) -> np.ndarray:
    return np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS], dtype=int
    )


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
            model = mujoco.MjModel.from_xml_path(str(scene_path(self._menagerie)))
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
        model, data = load(self._menagerie, ghost=ghost)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            apply_physics(model, physics)
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
        data.qpos[7:19] = pose
        mujoco.mj_forward(model, data)
        return MujocoSession(model, data, ghost=ghost, view=view, view_speed=view_speed)

    def rollout(self, plan: RolloutPlan) -> Prediction:
        pinned = plan.base is BaseCondition.PINNED
        ghost = self._ghost
        model, data = load(self._menagerie, pinned=pinned, ghost=ghost is not None)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            apply_physics(model, physics)
        actuator_tau = self._values.get("actuator_tau", 0.0)
        dt = float(model.opt.timestep)
        feet = foot_geom_ids(model)
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
