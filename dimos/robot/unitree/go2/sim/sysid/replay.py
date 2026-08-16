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

"""Mode A: open-loop plant replay. Recorded joint targets in, residuals out.

Replaying a recording *through a policy* only identifies the plant if the
recorded and simulated policies are the same net; replaying the policy's
OUTPUT (the joint targets it actually sent) removes the policy from the loop,
so every span is plant-excitation data whatever was driving. The cost is
feedback — the replay diverges — hence multiple shooting with short clips
(:func:`~dimos.robot.unitree.go2.sim.sysid.regimes.clip_schedule`).

Everything here is simulator-agnostic: it builds a :class:`RolloutPlan` from
measured streams, hands it to a :class:`Backend`, and differences the
:class:`Prediction` against the recording where the two overlap.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dataclasses_field

import numpy as np

from dimos.robot.unitree.go2.sim.backend import (
    Backend,
    BaseCondition,
    BaseTrack,
    Commands,
    Prediction,
    RolloutPlan,
    State,
)
from dimos.robot.unitree.go2.sim.ranges import Preset
from dimos.robot.unitree.go2.sim.rotations import quat_to_mat, rotation_angle, strip_yaw
from dimos.robot.unitree.go2.sim.sysid.ingest import Streams, mount_matrix
from dimos.robot.unitree.go2.sim.sysid.regimes import clip_schedule


def _step_quantized(t0: float, duration: float, schedule: np.ndarray, dt: float) -> list[float]:
    """Each scheduled snap moved to the first step time at or past it.

    Mirrors the stepping loop exactly, including its one-snap-per-step rule,
    so a plan built here fires bit-identically inside the backend.
    """
    out: list[float] = []
    si = 0
    for step in range(int(duration / dt)):
        t = t0 + step * dt
        if si < len(schedule) and t >= schedule[si]:
            out.append(t)
            si += 1
    return out


def measured_state(
    st: Streams,
    t: float,
    *,
    base_p: np.ndarray | None = None,
    base_r: np.ndarray | None = None,
) -> State:
    """The robot's MEASURED state at ``t``, ready for a backend to snap to.

    Joints from ``rt/lowstate``; attitude from the IMU, yaw-stripped. Base
    linear velocity needs the tracker (central difference on the room-frame
    base track); without one the clip starts at rest.
    """
    i = int(np.clip(int(np.searchsorted(st.lt, t, "right")) - 1, 0, len(st.lt) - 1))
    rot = strip_yaw(quat_to_mat(st.lquat[i]))
    v_body: np.ndarray | None = None
    if st.has_markers and base_p is not None and base_r is not None:
        j = int(np.clip(np.searchsorted(st.vt, t, "right") - 1, 1, len(st.vt) - 2))
        dt = st.vt[j + 1] - st.vt[j - 1]
        v_room = (base_p[j + 1] - base_p[j - 1]) / max(dt, 1e-6)
        v_body = base_r[j].T @ v_room  # room -> body
    return State(t=t, q=st.lq[i], dq=st.ldq[i], rot=rot, gyro=st.lgyro[i], v_body=v_body)


def base_track(st: Streams, t0: float, duration: float) -> BaseTrack:
    """The measured trunk attitude over ``[t0, t0+duration]``, for a PINNED base.

    Raw IMU attitude, NOT yaw-stripped: the backend anchors the track to the
    (yaw-stripped) snap pose at each re-initialisation, which cancels the
    arbitrary yaw as a constant world rotation while keeping every measured
    attitude increment — including real yaw motion, which per-sample stripping
    would corrupt near the 70-85 deg hang. One sample of margin either side so
    zero-order-hold sampling inside the backend never runs off the end.
    """
    i0 = max(int(np.searchsorted(st.lt, t0, "right")) - 1, 0)
    i1 = min(int(np.searchsorted(st.lt, t0 + duration, "right")) + 1, len(st.lt))
    rots = np.stack([quat_to_mat(q) for q in st.lquat[i0:i1]])
    return BaseTrack(t=st.lt[i0:i1].copy(), rot=rots)


def build_plan(
    st: Streams,
    t0: float,
    duration: float,
    *,
    schedule: np.ndarray,
    dt: float,
    suspended: bool = False,
    mount: np.ndarray | None = None,
) -> RolloutPlan:
    """A fully determined rollout: commands, reinit states, base condition.

    ``schedule`` are the mid-run snap times (absolute, from
    :func:`~dimos.robot.unitree.go2.sim.sysid.regimes.clip_schedule`); the
    state at ``t0`` is always prepended.

    Snap times are quantized to the backend's step grid (``dt``) FIRST, and
    the measured state is sampled at the quantized time: a snap can only fire
    on a step boundary, and sampling at the scheduled time instead can land
    one 500 Hz sample off — a silent nondeterminism between backends with
    different steps.
    """
    mount = mount_matrix() if mount is None else mount
    base_p = base_r = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount)
    times = _step_quantized(t0, duration, schedule, dt)
    reinit = [measured_state(st, t, base_p=base_p, base_r=base_r) for t in [t0, *times]]
    return RolloutPlan(
        t0=t0,
        duration=duration,
        commands=Commands(t=st.ct, q=st.cq, dq=st.cdq, kp=st.ckp, kd=st.ckd, tau_ff=st.ctau),
        reinit=reinit,
        base=BaseCondition.PINNED if suspended else BaseCondition.FREE,
        base_track=base_track(st, t0, duration) if suspended else None,
    )


@dataclass
class ReplayResult:
    """Sim prediction and recording, aligned on the prediction's time bases."""

    prediction: Prediction
    q_real: np.ndarray  # (n,12) at prediction.t
    p_real: np.ndarray | None  # (n,3) recorded base track in the SIM world frame
    r_real: np.ndarray | None  # (n,3,3)
    a_real: np.ndarray  # (k,3) at prediction.at
    w_real: np.ndarray  # (k,3)
    tau_real: np.ndarray  # (k,12)
    dq_real: np.ndarray = dataclasses_field(default_factory=lambda: np.zeros((0, 12)))
    # (n,12) at prediction.t — measured joint speeds, the `dq` channel

    def joint_err(self) -> np.ndarray:
        out: np.ndarray = np.abs(self.prediction.q - self.q_real)
        return out

    def body_err(self) -> tuple[np.ndarray, np.ndarray]:
        """Planar position error (m) and orientation error (rad) per sample."""
        if self.p_real is None or self.r_real is None:
            n = len(self.prediction.t)
            return np.zeros(n), np.zeros(n)
        dp = np.linalg.norm(self.prediction.body_pos[:, :2] - self.p_real[:, :2], axis=1)
        return dp, rotation_angle(self.prediction.body_rot, self.r_real)


def _real_track_in_sim_world(
    pred: Prediction, st: Streams, base_p: np.ndarray, base_r: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """The recorded base track mapped into the rollout's world frame.

    Open-loop drift is only defined relative to the pose a clip started from,
    so the rigid room->sim anchor is recomputed at EVERY re-initialisation
    from the sim pose the backend reported — a fixed anchor reads the snap
    itself as an 81 deg orientation error that is pure bookkeeping.
    """
    n = len(pred.t)
    p_real = np.empty((n, 3))
    r_real = np.empty((n, 3, 3))
    # anchors per reinit
    anchors_r = []
    anchors_p = []
    for r in range(len(pred.reinit_t)):
        j = int(np.clip(np.searchsorted(st.vt, pred.reinit_t[r], "right") - 1, 0, len(st.vt) - 1))
        a_r = pred.reinit_rot[r] @ base_r[j].T
        a_p = pred.reinit_pos[r] - a_r @ base_p[j]
        anchors_r.append(a_r)
        anchors_p.append(a_p)
    for i, t in enumerate(pred.t):
        r = int(np.clip(np.searchsorted(pred.reinit_t, t, "right") - 1, 0, len(pred.reinit_t) - 1))
        j = int(np.clip(np.searchsorted(st.vt, t, "right") - 1, 0, len(st.vt) - 1))
        p_real[i] = anchors_r[r] @ base_p[j] + anchors_p[r]
        r_real[i] = anchors_r[r] @ base_r[j]
    return p_real, r_real


def score(pred: Prediction, st: Streams, *, mount: np.ndarray | None = None) -> ReplayResult:
    """Difference the prediction against the recording on their intersection.

    A recording without a tracker simply has no body channels — no separate
    code path, no "VR mode": adding a tracker adds a channel and nothing else
    changes.
    """
    mount = mount_matrix() if mount is None else mount
    idx = np.clip(np.searchsorted(st.lt, pred.t, "right") - 1, 0, len(st.lt) - 1)
    q_real = st.lq[idx]
    aidx = np.clip(np.searchsorted(st.lt, pred.at, "right") - 1, 0, len(st.lt) - 1)
    p_real = r_real = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount)
        p_real, r_real = _real_track_in_sim_world(pred, st, base_p, base_r)
    return ReplayResult(
        prediction=pred,
        q_real=q_real,
        p_real=p_real,
        r_real=r_real,
        a_real=st.lacc[aidx] if len(st.lacc) else np.zeros((len(pred.at), 3)),
        w_real=st.lgyro[aidx] if len(st.lgyro) else np.zeros((len(pred.at), 3)),
        tau_real=st.ltau[aidx] if len(st.ltau) else np.zeros((len(pred.at), 12)),
        dq_real=st.ldq[idx] if len(st.ldq) else np.zeros((len(pred.t), 12)),
    )


def replay(
    st: Streams,
    t0: float,
    duration: float,
    backend: Backend,
    *,
    preset: Preset | None = None,
    window: float | tuple[float, float] | None = 0.4,
    seed: int = 0,
    protect: np.ndarray | None = None,
    suspended: bool = False,
    mount: np.ndarray | None = None,
) -> ReplayResult:
    """THE Mode-A code path: schedule, plan, rollout, score.

    ``preset`` applies a named plant to the backend first; ``None`` leaves
    whatever the caller last applied. Everything that varies between candidate
    plants is in the backend; everything here is a pure function of the
    recording and ``(window, seed, protect)``, so every candidate is scored on
    identical clips.
    """
    if preset is not None:
        backend.apply({**preset.physics, "actuator_tau": preset.actuator_tau})
    sched = clip_schedule(t0, duration, window, seed=seed, protect=protect)
    plan = build_plan(
        st, t0, duration, schedule=sched, dt=backend.timestep, suspended=suspended, mount=mount
    )
    return score(backend.rollout(plan), st, mount=mount)
