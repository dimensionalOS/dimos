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

"""Is this .bin the net that produced this recording? Check, never assume.

    python -m dimos.robot.unitree.go2.sim.sysid.verify_net REC.mcap NET.bin \
        --control OTHER.bin

Loop 2 runs a policy closed loop and compares statistics against the
recording. That is only meaningful when the net in the sim IS the net that was
driving the robot — with the wrong net the grounding produces confident,
meaningless numbers, which is worse than no grounding.

The check is TEACHER-FORCED REPLAY: rebuild the policy's observation at every
recorded ``policy/lowcmd`` tick from the MEASURED lowstate (q, dq, gyro,
projected gravity), the reconstructed slewed operator command, and the
PREVIOUS RECORDED action — then one forward pass, and compare the predicted
joint target with the one the executor actually sent. Teacher forcing keeps
the errors per-tick instead of compounding, so the residual measures net
identity plus observation-timing noise, not divergence.

An identity is only readable against a yardstick, so pass ``--control``: a
DIFFERENT net run through the identical procedure. The right net's residual
sits far below both the signal's own spread and the control's residual; a
wrong net's residual is on the order of the signal itself.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.sysid.ground import (
    COMMAND_SLEW,
    CONTROL_DT,
    NOMINAL_GAIT_HEIGHT,
    projected_gravity,
)
from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
from dimos.robot.unitree.go2.sim.sysid.recording import Streams


def slewed_commands(st: Streams, ticks: np.ndarray) -> np.ndarray:
    """The command the policy saw at each tick: operator target through the ramp.

    Reconstructed from the start of the command stream so the slew state is
    converged long before any scored tick.
    """
    if len(st.wt) == 0:
        return np.zeros((len(ticks), 3))
    out = np.empty((len(ticks), 3))
    vel = st.wcmd[0].astype(float).copy()
    t = float(min(st.wt[0], ticks[0] if len(ticks) else st.wt[0]))
    for i, tk in enumerate(ticks):
        while t <= tk:
            k = int(np.clip(np.searchsorted(st.wt, t, "right") - 1, 0, len(st.wt) - 1))
            vel += np.clip(st.wcmd[k] - vel, -COMMAND_SLEW, COMMAND_SLEW)
            t += CONTROL_DT
        out[i] = vel
    return out


@dataclass
class NetCheck:
    """One net's teacher-forced residual against one recording."""

    name: str
    obs_per_frame: int
    hist: int
    n_ticks: int
    residual_rms: float  # rad, prediction vs recorded target
    residual_p50: float
    residual_p90: float
    signal_rms: float  # rad, recorded target's deviation from its mean
    kp_match: bool  # blob gains == the gains the executor stamped
    kd_match: bool

    @property
    def ratio(self) -> float:
        """Residual over signal: ~0.1-0.3 for the producing net, ~1 for a stranger."""
        return self.residual_rms / self.signal_rms if self.signal_rms > 0 else float("inf")


def teacher_forced(
    st: Streams,
    policy: FreePolicy,
    *,
    name: str = "net",
    t0: float | None = None,
    t1: float | None = None,
    state_offset: float = 0.0,
) -> NetCheck:
    """Replay the net one tick at a time against the recorded targets.

    ``state_offset`` shifts where lowstate is sampled relative to the command
    timestamp (the hardware built its observation a few ms before the command
    was logged); the identity verdict is insensitive to it, the residual floor
    is not.
    """
    lo = float(st.ct[0]) if t0 is None else t0
    hi = float(st.ct[-1]) if t1 is None else t1
    sel = np.where((st.ct >= lo) & (st.ct <= hi))[0]
    # need hist frames of history and one previous action
    sel = sel[sel >= policy.hist]
    st.ct[sel]
    cmds = slewed_commands(st, st.ct)  # per recorded tick, full stream

    def frame(i: int) -> np.ndarray:
        t = st.ct[i] + state_offset
        k = int(np.clip(np.searchsorted(st.lt, t, "right") - 1, 0, len(st.lt) - 1))
        last = np.clip((st.cq[i - 1] - policy.act_mean) / policy.act_scale, -100, 100)
        raw = np.concatenate(
            [
                cmds[i],
                st.lgyro[k],
                projected_gravity(st.lquat[k]),
                st.lq[k],
                st.ldq[k],
                last,
            ]
        )
        extra = policy.obs_per_frame - raw.size
        if extra > 0:
            raw = np.concatenate([raw, [NOMINAL_GAIT_HEIGHT], np.zeros(extra - 1)])
        return policy.normalize(raw)

    errs = np.empty((len(sel), 12))
    for row, i in enumerate(sel):
        stack = np.concatenate([frame(i - h) for h in range(policy.hist)])  # newest first
        _a, target = policy.act(stack, cmds[i])
        errs[row] = target - st.cq[i]

    per_tick = np.sqrt(np.mean(errs * errs, axis=1))
    signal = st.cq[sel] - st.cq[sel].mean(axis=0)
    return NetCheck(
        name=name,
        obs_per_frame=policy.obs_per_frame,
        hist=policy.hist,
        n_ticks=len(sel),
        residual_rms=float(np.sqrt(np.mean(errs * errs))),
        residual_p50=float(np.percentile(per_tick, 50)),
        residual_p90=float(np.percentile(per_tick, 90)),
        signal_rms=float(np.sqrt(np.mean(signal * signal))),
        kp_match=bool(np.allclose(np.median(st.ckp[sel], axis=0), policy.kp, atol=1e-3)),
        kd_match=bool(np.allclose(np.median(st.ckd[sel], axis=0), policy.kd, atol=1e-3)),
    )


def verify(
    recording: str | Path,
    policy_bin: str | Path,
    control_bin: str | Path | None = None,
    *,
    t0: float | None = None,
    t1: float | None = None,
    state_offset: float = 0.0,
) -> tuple[NetCheck, NetCheck | None]:
    st = read_streams(recording)
    check = teacher_forced(
        st,
        FreePolicy.load(policy_bin),
        name=Path(policy_bin).stem,
        t0=t0,
        t1=t1,
        state_offset=state_offset,
    )
    control = None
    if control_bin is not None:
        control = teacher_forced(
            st,
            FreePolicy.load(control_bin),
            name=Path(control_bin).stem,
            t0=t0,
            t1=t1,
            state_offset=state_offset,
        )
    return check, control


def _fmt(c: NetCheck) -> str:
    return (
        f"  {c.name:<20s} obs {c.obs_per_frame} x{c.hist}  ticks {c.n_ticks}  "
        f"residual rms {c.residual_rms:.4f} p50 {c.residual_p50:.4f} p90 {c.residual_p90:.4f} rad"
        f"  signal rms {c.signal_rms:.4f}  ratio {c.ratio:.3f}"
        f"  kp {'match' if c.kp_match else 'MISMATCH'} kd {'match' if c.kd_match else 'MISMATCH'}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.verify_net")
    ap.add_argument("recording")
    ap.add_argument("policy_bin")
    ap.add_argument("--control", default=None, help="a DIFFERENT net, as the yardstick")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument(
        "--state-offset",
        type=float,
        default=0.0,
        help="sample lowstate this many s relative to the command timestamp",
    )
    args = ap.parse_args()
    t1 = None if args.seconds is None or args.start is None else args.start + args.seconds
    check, control = verify(
        args.recording,
        args.policy_bin,
        args.control,
        t0=args.start,
        t1=t1,
        state_offset=args.state_offset,
    )
    print(f"{Path(args.recording).name}: teacher-forced replay vs recorded policy/lowcmd")
    print(_fmt(check))
    if control is not None:
        print(_fmt(control))
        print(
            f"  candidate/control residual: {check.residual_rms / control.residual_rms:.3f} "
            "(well under 1 = the candidate explains the recording far better)"
        )


if __name__ == "__main__":
    main()
