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

"""Loop-2 identifiability: can ANY knob move the grounding's statistics?

    python -m dimos.robot.unitree.go2.sim.sysid.probe REC.mcap NET.bin \
        --preset measured --workers 16

:mod:`~dimos.simulation.sysid.identify` asks what a recording
resolves through Mode-A replay channels. This asks the closed-loop version of
the same prior question: push each knob to the ENDS of its admissible range,
roll the real policy, and see which grounding statistics move — judged
against the sim-perturb chaos floor (below it, the movement is
indistinguishable from rerunning) and against the sim-real gap (the thing a
fit would need to close). Range endpoints rather than derivative nudges,
deliberately: chaos makes small-step derivatives noise, and the endpoints
bound what any admissible value of that knob could ever do.

The question exists because of one specific verdict: on the verified
freewalk net every body-oscillation statistic came out ~2x too small — the
sim walks SMOOTHER than the robot. If no knob endpoint moves those
statistics past the chaos floor, then no fit over the knob table can close
the gap and the deficit is a MISSING MECHANISM, not a wrong number.

Besides the knob table, the same instrument probes the three candidate
mechanisms the ideal closed loop lacks and the real one has, each default-off
in :func:`~dimos.robot.unitree.go2.sim.sysid.ground.rollout_policy`:
action latency (delay eats phase margin: LESS delay = LESS oscillation),
observation noise (the policy's reaction to sensor noise IS body motion, and
the net was trained noisy), and the measured torque envelope (the real drive
delivers ~half its demand at swing speeds).

ONE ROLLOUT PER PROBE POINT. A probe is a spectrum, not a measurement: any
movement worth acting on must be re-established with the full grounding
before it becomes a claim, and everything below the chaos floor is already
labelled as such.
"""

from __future__ import annotations

import argparse
import concurrent.futures
from dataclasses import dataclass, field
import multiprocessing
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.ranges import DEFAULT_PRESET, KNOBS, Preset, load_preset
from dimos.robot.unitree.go2.sim.sysid.ground import (
    PERTURB_RAD,
    ObsNoise,
    Report,
    rollout_policy,
    sim_summary,
    usable_floor,
)
from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
from dimos.robot.unitree.go2.sim.sysid.real import cmd_at, real_summary
from dimos.robot.unitree.go2.sim.sysid.stats import Summary
from dimos.simulation.sysid.backend import ClosedLoopBackend
from dimos.simulation.sysid.recording import Streams

# The probed statistics: the 5b oscillation family, then the 5g speed family
# measured by the stride instrument (`gait_hz` was retired from scoring — it
# measured its own estimator, README 6).
FOCUS = ("roll_std", "pitch_std", "tilt_p99", "height_std", "stride_hz", "stride_len", "speed")


@dataclass(frozen=True)
class Probe:
    """One closed-loop rollout's complete configuration, decided up front."""

    name: str
    physics: dict[str, float] = field(default_factory=dict)
    actuator_tau: float = 0.0
    action_latency: float = 0.0
    noise_scale: float = 0.0
    envelope: str | None = None
    perturb_seed: int | None = None  # None = unperturbed
    # Measured-mechanism fields; each defaults to the pre-measurement shape.
    noise: ObsNoise | None = None  # per-channel levels; overrides noise_scale
    noise_seed: int = 0
    timing: tuple[float, ...] | None = None  # measured control intervals, s


# Worker-process state, set once by the initializer (spawned, never forked —
# a forked MuJoCo is a bug that looks like a speedup). The backend arrives
# PICKLED — a seam requirement (see backend.Backend) — so serial and worker
# rollouts run identically configured engines.
_WORKER: dict[str, object] = {}


def _init_worker(recording: str, policy_bin: str, backend: ClosedLoopBackend) -> None:
    _WORKER["st"] = read_streams(recording)  # cache-warm: the parent read it first
    _WORKER["policy"] = FreePolicy.load(policy_bin)
    _WORKER["backend"] = backend


def _eval(
    st: Streams,
    policy: FreePolicy,
    backend: ClosedLoopBackend,
    probe: Probe,
    start: float,
    seconds: float,
) -> Summary:
    """THE evaluation, serial and parallel alike: one rollout, one Summary."""
    perturb = None
    if probe.perturb_seed is not None:
        rng = np.random.default_rng(probe.perturb_seed)
        perturb = rng.normal(0.0, PERTURB_RAD, 12)
    obs_noise: ObsNoise | None
    if probe.noise is not None:
        obs_noise = probe.noise
    else:
        obs_noise = ObsNoise().scaled(probe.noise_scale) if probe.noise_scale > 0 else None
    run = rollout_policy(
        st,
        policy,
        Preset(name=probe.name, physics=probe.physics, actuator_tau=probe.actuator_tau),
        backend,
        start=start,
        seconds=seconds,
        action_latency=probe.action_latency,
        obs_noise=obs_noise,
        noise_seed=probe.noise_seed,
        control_intervals=np.array(probe.timing) if probe.timing is not None else None,
        envelope=TORQUE_ENVELOPES[probe.envelope] if probe.envelope else None,
        perturb=perturb,
    )
    return sim_summary(run, cmd_at(st, run.t + start))


def _eval_in_worker(probe: Probe, start: float, seconds: float) -> Summary:
    st = _WORKER["st"]
    policy = _WORKER["policy"]
    backend = _WORKER["backend"]
    assert isinstance(st, Streams) and isinstance(policy, FreePolicy)
    return _eval(st, policy, backend, probe, start, seconds)  # type: ignore[arg-type]


def default_probes(
    base_physics: dict[str, float],
    base_tau: float,
    *,
    latencies: tuple[float, ...] = (0.005, 0.01, 0.02, 0.04),
    noises: tuple[float, ...] = (0.5, 1.0, 2.0),
    envelopes: tuple[str, ...] = ("central", "central-signed"),
    knobs: tuple[str, ...] | None = None,
) -> list[Probe]:
    """The spectrum: every knob at both range ends, then the absent mechanisms."""
    out: list[Probe] = []
    for name in knobs if knobs is not None else tuple(KNOBS):
        k = KNOBS[name]
        for label, v in (("lo", k.lo), ("hi", k.hi)):
            if name == "actuator_tau":
                out.append(Probe(f"{name}={v:g} ({label})", dict(base_physics), v))
            else:
                out.append(Probe(f"{name}={v:g} ({label})", {**base_physics, name: v}, base_tau))
    for lat in latencies:
        out.append(Probe(f"action_latency={lat:g}", dict(base_physics), base_tau, lat))
    for s in noises:
        out.append(Probe(f"obs_noise x{s:g}", dict(base_physics), base_tau, noise_scale=s))
    for env in envelopes:
        out.append(Probe(f"envelope={env}", dict(base_physics), base_tau, envelope=env))
    # The mechanisms compound on the robot; probe one plausible stack too.
    out.append(
        Probe(
            "latency=0.01 + noise x1",
            dict(base_physics),
            base_tau,
            action_latency=0.01,
            noise_scale=1.0,
        )
    )
    out.append(
        Probe(
            "latency=0.01 + noise x1 + central",
            dict(base_physics),
            base_tau,
            action_latency=0.01,
            noise_scale=1.0,
            envelope="central",
        )
    )
    return out


@dataclass
class Spectrum:
    """Every probe's statistics beside the base run, the real run and the floor."""

    base: Summary
    real: Summary
    floor: dict[str, float]
    results: list[tuple[Probe, Summary]]
    start: float
    seconds: float
    preset: str

    def loss(self, s: Summary) -> float:
        rep = Report(
            preset="probe",
            sim=s,
            real=self.real,
            noise=self.floor,
            floor_source="sim-perturb (base plant, shared)",
            start=self.start,
            seconds=self.seconds,
        )
        return rep.loss()

    def table(self) -> str:
        b, r = self.base.as_dict(), self.real.as_dict()
        lines = [
            f"LOOP-2 SPECTRUM  base={self.preset}  {self.seconds:.0f}s from t={self.start:.0f}s"
            f"  {len(self.results)} probes",
            "",
            f"{'statistic':>12} {'sim':>8} {'real':>8} {'gap':>8} {'floor':>8}",
        ]
        for k in FOCUS:
            lines.append(
                f"{k:>12} {b[k]:8.3f} {r[k]:8.3f} {r[k] - b[k]:+8.3f} {self.floor[k]:8.3f}"
            )
        lines += [
            "",
            "Cells: signed % of the gap closed; '*' = movement exceeds the chaos floor,",
            "'.' = below it (indistinguishable from a rerun). 'loss' = grounding RMS-SNR.",
            "",
            f"{'probe':<34}" + "".join(f"{k:>11}" for k in FOCUS) + f"{'loss':>8}",
        ]
        base_loss = self.loss(self.base)
        lines.append(f"{'(base)':<34}" + " " * (11 * len(FOCUS)) + f"{base_loss:8.2f}")
        for probe, s in self.results:
            sd = s.as_dict()
            cells = []
            for k in FOCUS:
                delta = sd[k] - b[k]
                gap = r[k] - b[k]
                mark = "*" if abs(delta) > self.floor[k] else "."
                pct = 100.0 * delta / gap if abs(gap) > 1e-9 else float("inf")
                cells.append(f"{pct:+9.0f}%{mark}")
            lines.append(f"{probe.name:<34}" + "".join(cells) + f"{self.loss(s):8.2f}")
        lines += ["", "BEST MOVEMENT toward the real value, per statistic"]
        for k in FOCUS:
            gap = r[k] - b[k]
            if abs(gap) < 1e-9:
                continue
            best_probe, best_pct = None, 0.0
            for probe, s in self.results:
                delta = s.as_dict()[k] - b[k]
                if abs(delta) <= self.floor[k]:
                    continue
                pct = 100.0 * delta / gap
                if pct > best_pct:
                    best_probe, best_pct = probe, pct
            if best_probe is None:
                lines.append(f"  {k:>12}  NOTHING moves it past the chaos floor")
            else:
                lines.append(f"  {k:>12}  {best_probe.name:<34} closes {best_pct:+.0f}%")
        return "\n".join(lines)


def spectrum(
    recording: str | Path,
    policy_bin: str | Path,
    probes: list[Probe],
    backend: ClosedLoopBackend,
    *,
    preset: str = "measured",
    start: float = 6.0,
    seconds: float | None = None,
    floor_seeds: int = 4,
    workers: int = 1,
) -> Spectrum:
    """Run the probes (fanned across processes) against one shared floor.

    Every probe is a pure function of its :class:`Probe`, so the fan-out
    changes nothing but wall time. The floor is measured on the BASE plant
    (perturbed initial poses) and shared by every probe — per-probe floors
    would let a probe buy its result with its own chaos.
    """
    st = read_streams(recording)
    policy = FreePolicy.load(policy_bin)
    p = load_preset(preset)
    base_physics, base_tau = dict(p.physics), p.actuator_tau
    span = float(st.wt[-1]) - start
    seconds = span if seconds is None else seconds

    # The base and floor probes inherit the preset's own envelope: a plant
    # fitted with the envelope on is grounded with it, floor included.
    base_probe = Probe("(base)", base_physics, base_tau, envelope=p.envelope)
    floor_probes = [
        Probe(f"(floor seed {i})", base_physics, base_tau, envelope=p.envelope, perturb_seed=i)
        for i in range(floor_seeds)
    ]
    all_probes = [base_probe, *floor_probes, *probes]

    if workers <= 1:
        summaries = [_eval(st, policy, backend, pr, start, seconds) for pr in all_probes]
    else:
        with concurrent.futures.ProcessPoolExecutor(
            max_workers=workers,
            mp_context=multiprocessing.get_context("spawn"),
            initializer=_init_worker,
            initargs=(str(recording), str(policy_bin), backend),
        ) as pool:
            futures = [pool.submit(_eval_in_worker, pr, start, seconds) for pr in all_probes]
            summaries = [f.result() for f in futures]

    base = summaries[0]
    real = real_summary(st, start=start, seconds=seconds)
    raw = {
        k: max(s.as_dict()[k] for s in summaries[1 : 1 + floor_seeds])
        - min(s.as_dict()[k] for s in summaries[1 : 1 + floor_seeds])
        for k in base.as_dict()
    }
    floor = usable_floor(raw, real.as_dict())
    return Spectrum(
        base=base,
        real=real,
        floor=floor,
        results=list(zip(probes, summaries[1 + floor_seeds :], strict=True)),
        start=start,
        seconds=seconds,
        preset=preset,
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.probe")
    ap.add_argument("recording")
    ap.add_argument("policy_bin", help="the net that PRODUCED the recording (verify_net first)")
    ap.add_argument("--preset", default=DEFAULT_PRESET, help="base plant: preset name or JSON path")
    ap.add_argument("--start", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--floor-seeds", type=int, default=4)
    ap.add_argument("--knobs", nargs="+", default=None, help="probe only these knobs")
    ap.add_argument(
        "--latency", type=float, nargs="*", default=[0.005, 0.01, 0.02, 0.04], metavar="S"
    )
    ap.add_argument("--noise", type=float, nargs="*", default=[0.5, 1.0, 2.0], metavar="SCALE")
    ap.add_argument("--envelopes", nargs="*", default=["central", "central-signed"], metavar="NAME")
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

    p = load_preset(args.preset)
    probes = default_probes(
        dict(p.physics),
        p.actuator_tau,
        latencies=tuple(args.latency),
        noises=tuple(args.noise),
        envelopes=tuple(args.envelopes),
        knobs=tuple(args.knobs) if args.knobs else None,
    )
    spec = spectrum(
        args.recording,
        args.policy_bin,
        probes,
        MujocoBackend(),
        preset=args.preset,
        start=args.start,
        seconds=args.seconds,
        floor_seeds=args.floor_seeds,
        workers=args.workers,
    )
    print(spec.table())


if __name__ == "__main__":
    main()
