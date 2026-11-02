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

"""Go2 system identification in one file: fit the joints, judge the body.

    python -m dimos.navigation.sysid.example          # both loops, ~30 s
    python -m dimos.navigation.sysid.example --fit    # plus a small fit, ~1 min
    python -m dimos.navigation.sysid.example --view   # watch loop 2 walk

The question: which simulator parameters (joint armature, friction, actuator
lag, leg mass, foot contact) make MuJoCo's Go2 move like the real one?

The data (``data/go2_sysid_sample``, pulled from LFS on first use) is 80 s of
the real robot walking around a room under a learned policy:

* ``rt/lowstate``: measured joint angles, speeds and torques at ~460 Hz.
* ``rt/lowcmd`` / ``policy/lowcmd``: the joint targets and PD gains sent to
  the motors.
* ``control_log``: the operator's velocity commands (vx, vy, yaw rate).
* ``vive/pose``: a VR tracker on the trunk, ground truth for where the body
  went.
* ``freewalk_mcf.bin``: the policy that produced the walk.

Why two loops? Neither one alone has a unique answer, and the right parameters
are where their answers overlap.

Loop 1 (the joints) has many solutions because the parameters trade against
each other: more armature and less friction, or softer feet and a heavier
leg, explain the joint signals equally well. Seeded fits agree on the loss and
disagree on parameters by up to 9x. Loop 2 (the walk) has many solutions
because a whole walk boils down to a handful of statistics, too few to pin a
dozen parameters: plenty of physically wrong plants still walk the right
path. A plant in both sets moves its joints like the robot AND walks like it.

The two sets are found differently. Loop 1 is dense (36 signals at 500 Hz),
repeatable and not chaotic, so it can be searched: an optimiser maps its
region. Loop 2 is chaotic (two walks from almost the same start drift apart)
and slow, so it is only ever evaluated: candidates from loop 1's region are
walked, and the ones that also walk right are the overlap.

**Loop 1: per joint, open loop. This one fits the parameters.**
Feed the recorded joint targets straight into the sim's PD motors (no policy)
and compare every joint's angle, speed and torque with the recording, sample
by sample. Open loop drifts, so the sim is snapped back to the measured state
at random intervals of 0.05 to 0.8 s ("multiple shooting"), which leaves the
plant's short-horizon dynamics as the only thing being compared. That gives 36
dense signals at 500 Hz: cheap, and smooth enough for a black-box optimiser.

**Loop 2: whole body, closed loop. This one judges the result.**
Put the real policy in the loop, drive it with the operator's recorded
commands, and let it walk for 40 s with no snapping. Compare where the sim
robot went with where the tracker says the real one went: mean error along
the path, across it, and in heading. Loop 1 never looked at body motion, so
loop 2 is an independent check. ``sysid.select`` walks candidate sets from
loop 1's region through it and ships the best (``measured`` is one such
draw).

Each loop is run on two plants: ``stock`` (the MuJoCo menagerie model as
shipped) and ``measured`` (the identified plant in ``ranges.py``). ``--fit``
also runs one small seeded search from a naive start and puts its result
through both loops. Expect it to win loop 1 (scored in-sample) and not
necessarily loop 2: a point in loop 1's region, not yet shown to be in the
overlap.

Loop 2 runs a few replicates from slightly perturbed start poses. The spread
between them is the chaos floor: how much the sim disagrees with itself. An
error near the floor is as good as this data can tell.

Everything below calls the real pipeline. The CLIs (``sysid.replay``,
``sysid.fit``, ``sysid.ground``) are these same calls with more options; see
README.md for the module map.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import math
from pathlib import Path

import numpy as np

from dimos.navigation.sysid.backend import ClosedLoopBackend
from dimos.navigation.sysid.fit import (
    Objective,
    base_values,
    default_plan,
    merged,
    run_study,
)
from dimos.navigation.sysid.ground import (
    PERTURB_RAD,
    Tracking,
    rollout_policy,
    tracking_curves,
    tracking_of,
)
from dimos.navigation.sysid.ingest import read_streams, sample
from dimos.navigation.sysid.plant import TORQUE_ENVELOPES
from dimos.navigation.sysid.policy import FreePolicy
from dimos.navigation.sysid.ranges import Preset, load_preset
from dimos.navigation.sysid.recording import Streams, read_declarations
from dimos.navigation.sysid.regimes import protected, regimes, sample_segments
from dimos.navigation.sysid.replay import ghost_track, replay
from dimos.navigation.sysid.rollouts import Rollouts
from dimos.navigation.sysid.score import DEFAULT_WEIGHTS, residual

# Builds a simulator for a plant. Only main() names the engine (MuJoCo); the
# loops below work with any backend, which is how another simulator plugs in.
Engine = Callable[[Preset], ClosedLoopBackend]

# Loop 1 replays 20 s of steady walking.
REPLAY_START, REPLAY_SECONDS = 10.0, 20.0
# Loop 2 starts mid-walk and runs to near the end of the commands.
WALK_START, WALK_SECONDS = 6.0, 40.0
REPLICATES = 3


def loop1(st: Streams, preset: Preset, engine: Engine) -> dict[str, float]:
    """Open-loop replay: mean |sim - real| per joint channel, in physical units."""
    r = replay(
        st,
        REPLAY_START,
        REPLAY_SECONDS,
        engine(preset),
        preset=preset,
        window=(0.05, 0.8),  # random snap-back interval, seconds
        protect=protected(regimes(st)),  # never snap mid-flight
    )
    out = {}
    for channel in ("joint", "dq", "tau"):
        res = residual(r, channel)
        assert res is not None  # joint channels always have a real side
        out[channel] = float(np.mean(np.abs(res)))
    return out


def loop2(st: Streams, policy: FreePolicy, preset: Preset, engine: Engine) -> Tracking:
    """Closed-loop walk, scored against the tracker; replicates give the chaos floor."""
    rng = np.random.default_rng(0)
    curves = []
    for i in range(REPLICATES):
        run = rollout_policy(
            st,
            policy,
            preset,
            engine(preset),
            start=WALK_START,
            seconds=WALK_SECONDS,
            perturb=rng.normal(0.0, PERTURB_RAD, 12) if i else None,
        )
        curves.append(tracking_curves(run, st, start=WALK_START))
    tracking = tracking_of(curves, WALK_SECONDS)
    assert tracking is not None  # the sample has a tracker
    return tracking


def watch(st: Streams, policy: FreePolicy, preset: Preset, engine: Engine) -> None:
    """Loop 2 in the viewer; the translucent ghost is the tracker's pose."""
    rollout_policy(
        st,
        policy,
        preset,
        engine(preset),
        start=WALK_START,
        seconds=WALK_SECONDS,
        view=True,
        ghost=ghost_track(st),
    )


def fit_from_naive(
    recording: Path, engine: Engine, *, trials: int = 60, workers: int = 8
) -> Preset:
    """One seeded CMA-ES study on loop 1, starting from the middle of every range.

    The real fit (``sysid.fit``) runs several such studies and ships the median
    of every near-best trial, because the parameters trade against each other
    and the data only pins down a region. One study is enough to see the loss
    move.
    """
    backend = engine(load_preset("stock"))
    # Pinned knobs (trunk mass, foot friction, ...) come from measurements;
    # the searched ones start at the centre of their declared range.
    plan = default_plan(backend.knobs())
    naive = {
        name: math.sqrt(k.lo * k.hi) if k.log else (k.lo + k.hi) / 2
        for name, k in plan.searched.items()
    }
    base = {**base_values("measured"), **naive}
    with Rollouts(recording, backend, workers=workers) as rollouts:
        st = rollouts.streams
        objective = Objective(
            rollouts,
            # Eight 4-8 s stretches of the walk, the same for every candidate.
            segments=sample_segments(8.0, 70.0, n=8, length=(4.0, 8.0), seed=0),
            spans=regimes(st, read_declarations(recording)),
            weights=DEFAULT_WEIGHTS,  # joint 0.4, dq 0.4, tau 0.2; body channels 0
            backend_channels=backend.channels(),
        )
        # Residuals are normalised by the starting plant's, so its loss is the yardstick.
        start = objective.calibrate(merged(base, plan, {}))
        study = run_study(objective, plan, base, seed=0, trials=trials)
    print(
        f"  loop 1 loss: naive start {start.total:.3f} -> best of {trials} {study.best_total:.3f}"
    )
    for name, value in study.best_params.items():
        print(f"    {name:20s} {naive[name]:10.5f} -> {value:10.5f}")
    values = merged(base, plan, study.best_params)
    tau = values.pop("actuator_tau")
    return Preset(name="fit", physics=values, actuator_tau=tau)


def report(name: str, st: Streams, policy: FreePolicy, preset: Preset, engine: Engine) -> Tracking:
    """Both loops on one plant, printed as one row."""
    j = loop1(st, preset, engine)
    t = loop2(st, policy, preset, engine)
    print(
        f"{name:9s} | loop 1  joint {j['joint']:.4f} rad  dq {j['dq']:.3f} rad/s  "
        f"tau {j['tau']:.2f} Nm"
        f" | loop 2  along {t.median('along'):.3f} m  cross {t.median('cross'):.3f} m  "
        f"yaw {t.median('yaw'):.3f} rad"
    )
    return t


def main() -> None:
    ap = argparse.ArgumentParser(
        prog="navigation.sysid.example",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--fit", action="store_true", help="also run a small fit and judge it")
    ap.add_argument("--view", action="store_true", help="watch loop 2 on the measured plant")
    args = ap.parse_args()

    from dimos.navigation.sysid.engines.mujoco import MujocoBackend

    def mujoco(preset: Preset) -> MujocoBackend:
        # A plant fitted with a torque envelope must always run with it.
        envelope = TORQUE_ENVELOPES[preset.envelope] if preset.envelope else None
        return MujocoBackend(envelope=envelope)

    recording, policy_path = sample()
    st = read_streams(recording)
    policy = FreePolicy.load(policy_path)

    if args.view:
        watch(st, policy, load_preset("measured"), mujoco)
        return

    print("Mean |sim - real|. Loop 1 per joint, open loop. Loop 2 body path, closed loop.\n")
    report("stock", st, policy, load_preset("stock"), mujoco)
    t = report("measured", st, policy, load_preset("measured"), mujoco)
    print(
        f"\nchaos floor, the sim against itself: along {t.floor_of('along'):.3f} m  "
        f"cross {t.floor_of('cross'):.3f} m  "
        f"yaw {t.floor_of('yaw'):.3f} rad"
    )

    if args.fit:
        print("\nFitting on loop 1 from a naive start:")
        report("fit", st, policy, fit_from_naive(recording, mujoco), mujoco)


if __name__ == "__main__":
    main()
