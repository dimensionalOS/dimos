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

"""Draw selection: the referee chooses the shipping point from a fit's cloud.

README 5's shipping mechanism, as a tool instead of a ritual re-derived per
shipping decision: the fit's median is NOT what ships — refit stochasticity
moves the referee verdict more than any judge-weighting choice — so every
index-aligned JOINT draw of the pooled cloud (``ranges.json: "cloud"``) is
grounded on the SELECTION recording under one shared floor, alongside the
incumbent, and the ranking this prints is what a reserve pass then quotes
ONCE (:mod:`~dimos.robot.unitree.go2.sim.sysid.meta` ``experiment`` with the
finalist JSONs this writes).

Three-split discipline is the caller's obligation, restated here because the
tool cannot enforce it: the selection recording must be one no fit touched,
and whatever recording the finalists are quoted on afterwards is SPENT.

    python -m dimos.robot.unitree.go2.sim.sysid.select \\
        results/fit.ranges.json REC.mcap NET.bin \\
        --preset results/fitbase.json --replicates 16 --workers 24 \\
        --out results/draws --top 3
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

from dimos.robot.unitree.go2.sim.backend import ClosedLoopBackend
from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.ranges import Preset, load_preset
from dimos.robot.unitree.go2.sim.sysid.fit import base_values
from dimos.robot.unitree.go2.sim.sysid.ground import Report, ground
from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
from dimos.robot.unitree.go2.sim.sysid.meta import MDD_N16, shared_floor


@dataclass(frozen=True)
class Draw:
    """One joint row of the pooled cloud, ready to be grounded."""

    index: int
    values: dict[str, float]  # complete knob values: base, pins, then the row

    def preset(self, name: str, *, envelope: str | None, why: str) -> Preset:
        physics = {k: v for k, v in self.values.items() if k != "actuator_tau"}
        return Preset(
            name=name,
            physics=physics,
            actuator_tau=float(self.values.get("actuator_tau", 0.0)),
            envelope=envelope,
            provenance=dict.fromkeys(physics, why),
        )


def load_draws(ranges_path: str | Path, base_preset: str) -> list[Draw]:
    """The cloud's joint rows over the fit's own base and pins.

    Row ``i`` of every cloud array is ONE harvested trial — the knobs trade
    against each other, so rows are never resampled per knob (that fabricates
    off-valley points the fit never visited). Values the fit pinned ride
    along; everything else comes from the base preset, exactly as
    :func:`~dimos.robot.unitree.go2.sim.sysid.fit.merged` composed each trial.
    """
    d = json.loads(Path(ranges_path).read_text())
    cloud: dict[str, list[float]] = d.get("cloud", {})
    if not cloud:
        raise ValueError(f"{ranges_path}: no cloud — re-run the fit with --out")
    ns = {len(v) for v in cloud.values()}
    if len(ns) != 1:
        raise ValueError(f"{ranges_path}: cloud rows are not index-aligned: lengths {sorted(ns)}")
    base = base_values(base_preset)
    pins = {k: float(p["value"]) for k, p in d.get("pinned", {}).items()}
    n = ns.pop()
    return [
        Draw(index=i, values={**base, **pins, **{k: float(v[i]) for k, v in cloud.items()}})
        for i in range(n)
    ]


def ground_draws(
    draws: list[Draw],
    recording: str | Path,
    policy: FreePolicy,
    backend: ClosedLoopBackend,
    *,
    envelope: str | None,
    incumbent: str = "measured",
    start: float = 6.0,
    seconds: float | None = None,
    replicates: int = 16,
    workers: int = 1,
    on_result: Any = None,
) -> tuple[list[tuple[Draw | None, Report]], dict[str, float], str]:
    """Every draw and the incumbent, grounded under ONE shared floor.

    Returns ``(results, floor, floor_source)`` where the incumbent's report
    carries ``None`` for its draw. A per-candidate floor would let a plant
    buy small SNRs with its own chaos (:func:`shared_floor`'s rule); the
    incumbent rides in the same pass so "the price" is two rows of one table,
    never two runs of two scripts.
    """
    noise, source = shared_floor(
        recording, policy, backend, baseline=incumbent, start=start, seconds=seconds
    )
    st = read_streams(recording)
    why = f"candidate draw, grounded at {replicates} replicates on {Path(str(recording)).name}"
    results: list[tuple[Draw | None, Report]] = []
    todo: list[tuple[Draw | None, Preset]] = [(None, load_preset(incumbent))]
    todo += [(dr, dr.preset(f"draw{dr.index:03d}", envelope=envelope, why=why)) for dr in draws]
    for dr, preset in todo:
        rep = ground(
            st,
            policy,
            preset,
            backend,
            start=start,
            seconds=seconds,
            noise=noise,
            floor_source=source,
            replicates=replicates,
            workers=workers,
            with_ghost=False,
        )
        results.append((dr, rep))
        if on_result is not None:
            on_result(dr, rep)
    return results, noise, source


def ranking_table(results: list[tuple[Draw | None, Report]], mdd: float) -> str:
    order = sorted(results, key=lambda t: t[1].loss())
    lines = [
        f"{'rank':>4} {'candidate':<12} {'loss':>7} {'draws':>13} {'k of M':>7}",
    ]
    for rank, (dr, rep) in enumerate(order):
        lo, hi = rep.loss_range()
        n, of = rep.n_matched()
        name = "incumbent" if dr is None else f"draw{dr.index:03d}"
        lines.append(
            f"{rank:>4} {name:<12} {rep.loss():7.2f} {lo:6.2f}-{hi:<6.2f} {n:>3} of {of}"
        )
    lines.append(
        f"  Losses within the MDD ({mdd:.2f} at these replicates) of each other are TIES."
    )
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.select", description=__doc__)
    ap.add_argument("ranges_json", help="a fit's .ranges.json (must carry the cloud)")
    ap.add_argument("recording", help="the SELECTION recording — one no fit touched")
    ap.add_argument("policy_bin", help="the net that produced it (verify_net first)")
    ap.add_argument("--preset", required=True, help="the fit's base preset (name or JSON)")
    ap.add_argument("--incumbent", default="measured", help="grounded in the same pass")
    ap.add_argument("--start", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--replicates", type=int, default=16)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument(
        "--limit",
        type=int,
        default=None,
        help="ground only every k-th draw to fit a budget — the cut is PRINTED, "
        "never silent (README: no silent caps)",
    )
    ap.add_argument("--top", type=int, default=3, help="finalist preset JSONs to write")
    ap.add_argument("--out", required=True, help="output prefix for finalists and the log")
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

    draws = load_draws(args.ranges_json, args.preset)
    if args.limit is not None and args.limit > 1:
        kept = draws[:: args.limit]
        print(f"LIMIT: grounding {len(kept)} of {len(draws)} draws (every {args.limit}th)")
        draws = kept
    base = load_preset(args.preset)
    policy = FreePolicy.load(args.policy_bin)
    backend = MujocoBackend()
    t0 = time.perf_counter()

    def progress(dr: Draw | None, rep: Report) -> None:
        name = "incumbent" if dr is None else f"draw{dr.index:03d}"
        lo, hi = rep.loss_range()
        print(
            f"  {name:<12} loss {rep.loss():7.2f} (draws {lo:.2f}-{hi:.2f})  "
            f"[{time.perf_counter() - t0:6.0f}s]",
            flush=True,
        )

    print(
        f"SELECTION  {Path(args.recording).name}  {len(draws)} draws + {args.incumbent}  "
        f"replicates {args.replicates}"
    )
    results, _noise, source = ground_draws(
        draws,
        args.recording,
        policy,
        backend,
        envelope=base.envelope,
        incumbent=args.incumbent,
        start=args.start,
        seconds=args.seconds,
        replicates=args.replicates,
        workers=args.workers,
        on_result=progress,
    )
    print(f"\nfloor: {source}")
    print(ranking_table(results, MDD_N16))

    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    ranked = sorted(results, key=lambda t: t[1].loss())
    finalists = [dr for dr, _rep in ranked if dr is not None][: args.top]
    why = (
        f"selected: cloud of {Path(args.ranges_json).name}, grounded at "
        f"{args.replicates} replicates on {Path(args.recording).name}"
    )
    for dr in finalists:
        p = dr.preset(f"{prefix.name}-draw{dr.index:03d}", envelope=base.envelope, why=why)
        path = p.save(f"{prefix}-draw{dr.index:03d}.json")
        print(f"finalist -> {path}")
    log = {
        "ranges_json": str(args.ranges_json),
        "recording": str(args.recording),
        "replicates": args.replicates,
        "floor_source": source,
        "results": [
            {
                "draw": None if dr is None else dr.index,
                "loss": rep.loss(),
                "loss_range": list(rep.loss_range()),
                "matched": list(rep.n_matched()),
                "values": None if dr is None else dr.values,
            }
            for dr, rep in ranked
        ],
    }
    Path(f"{prefix}.selection.json").write_text(json.dumps(log, indent=2))
    print(f"log -> {prefix}.selection.json")


if __name__ == "__main__":
    main()
