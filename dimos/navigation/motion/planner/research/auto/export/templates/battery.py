#!/usr/bin/env python3
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

"""Scored battery with per-world evo traces.

The heavy lifting is the referee's: `judge()`, `score_world()` and
`summarize()` are imported from the lab's local `referee/` package and are
never reimplemented here. This file mirrors the referee CLI's `--score`
path (same core pinning, same per-plan-call time limit) and adds two things
the CLI has no reason to provide: per-world trace emission, and the
*optimization* fitness (see bench/fitness.py -- the weights and their
retune procedure live there, on purpose).

Deviation is tolerated, collisions are not: DQ zeroes the world outright and
the registered gates hard-fail on any DQ, on mean gold below the floor, and
on any single world falling off a cliff. See gates.json.

  bench/run --gen 40 --seed 0                    # the scored battery
  bench/run --gen 0 --max-dq 0 --min-gold 1.0    # curated only, as a gate
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _evo_inline import log_task, write_result  # noqa: E402
from fitness import FITNESS_MAX, fitness_of  # noqa: E402
# quality.py is stdlib-only (it runs as a gate, outside the lab venv), so
# the fingerprints live there and are imported here -- one definition, and
# the gate can never disagree with the run it is validating.
from quality import LAB, evo_root, summary_path, planner_fingerprint, referee_tree_hash  # noqa: E402

from referee import scenarios as _scenarios  # noqa: E402
from referee.geometry import AvoidanceConfig  # noqa: E402
from referee.scenarios import EMBODIMENTS, SCENARIOS, generated  # noqa: E402
from referee.score import SPEED_BUDGET_MS, score_world, summarize  # noqa: E402
from referee.sim import judge  # noqa: E402


def _isolate_world_cache(count: int, seed: int) -> None:
    """Give each (count, seed) its own world cache file, in the SHARED cache
    dir at the origin lab root.

    The referee keeps ONE `.gen_cache.pkl` holding a single fingerprint of
    (count, seed, rules, emb), so alternating between the scored battery
    (40/0) and the held-out set (8/991) would regenerate every world on every
    switch. Pointing the module global at a per-combination path costs
    nothing, changes no world (the fingerprint check is unchanged, so a stale
    file is still rejected), and makes the held-out gate affordable.

    The gold-path cache is deliberately NOT isolated: it is a per-query memo,
    it is correct to share it across every seed and every worktree, and it is
    the reason a run takes minutes instead of an hour. bench/run points it at
    the shared dir via AUTORESEARCH_CACHE_DIR; this only renames the world
    cache within that dir."""
    cache_dir = evo_root() / ".evo" / "cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    _scenarios._CACHE = cache_dir / f"gen_cache_{count}_{seed}.pkl"


# The referee CLI applies this to every scored run. Kept honest by
# bench/parity: drifting it would change scores, and parity cross-checks
# against ./eval.
TIME_LIMIT_MS = 6000.0


def _pin_to_one_core() -> int | None:
    """Scored runs are timing runs -- pin to one core, as the referee does.

    The referee CLI picks min(affinity); this picks max, because core 0 is
    where the kernel lands most IRQ work and it is the busiest core on the
    box."""
    try:
        core = max(os.sched_getaffinity(0))
        os.sched_setaffinity(0, {core})
        return core
    except (AttributeError, OSError):
        return None  # non-Linux: unpinned, timings advisory


def _calibrate() -> float:
    """CPU-time cost of a fixed arithmetic workload, in ms.

    Speed is the pillar with real headroom, so it is also the pillar that can
    be won by luck. CPU-time scoring removed the scheduling component of that
    noise but not the cache/memory-bandwidth component, which tracks how busy
    the machine is. Recording a constant workload's cost in every summary
    lets a later reader tell "this candidate got faster" from "this run
    happened on a quieter box" instead of guessing."""
    t0 = time.process_time()
    acc = 0.0
    for k in range(400_000):
        acc += math.sqrt(k * 1.0000001)
    return round((time.process_time() - t0) * 1e3, 3)


def _referee_provenance() -> dict[str, Any]:
    """Which referee produced these numbers. Scores are only comparable
    across experiments judged by byte-identical referee code. source_commit
    (from .evo/referee.lock) records where that code originally came from."""
    lock_path = evo_root() / ".evo" / "referee.lock"
    source = "unknown"
    if lock_path.exists():
        try:
            source = json.loads(lock_path.read_text())["source_commit"]
        except (OSError, json.JSONDecodeError, KeyError):
            pass
    return {"referee_sha256": referee_tree_hash(), "source_commit": source}


def _finite(x: float) -> float | None:
    """json.dumps emits bare Infinity, which is not valid JSON."""
    return None if (math.isinf(x) or math.isnan(x)) else round(float(x), 6)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gen", type=int, default=40, help="generated worlds to append")
    ap.add_argument("--seed", type=int, default=0, help="base seed for generated worlds")
    ap.add_argument("--emb", default="mix", choices=[*sorted(EMBODIMENTS), "mix"])
    ap.add_argument("--scenario", help="run one world by name (diagnosis; not a scored run)")
    ap.add_argument("--no-curated", action="store_true",
                    help="generated worlds only -- for the held-out gate, where the "
                         "curated 16 are neither unseen nor informative")
    ap.add_argument("--time-limit-ms", type=float, default=TIME_LIMIT_MS)
    ap.add_argument("--summary-json", help="also write the full summary dict here")
    # Gate thresholds. Any breach exits non-zero; absent flags are not checked.
    ap.add_argument("--min-fitness", type=float)
    ap.add_argument("--min-referee-score", type=float)
    ap.add_argument("--min-gold", type=float, help="floor on MEAN gold closeness")
    ap.add_argument("--min-world-gold", type=float, help="floor on the WORST world's gold")
    ap.add_argument("--min-speed", type=float)
    ap.add_argument("--min-consistency", type=float)
    ap.add_argument("--max-dq", type=int)
    args = ap.parse_args()

    core = _pin_to_one_core()
    prov = _referee_provenance()
    calib_ms = _calibrate()
    if args.gen:
        _isolate_world_cache(args.gen, args.seed)
    cfg = AvoidanceConfig()
    curated = {sc.name for sc in SCENARIOS}
    pool = ([] if args.no_curated else list(SCENARIOS)) + (
        generated(args.gen, args.seed, emb=EMBODIMENTS.get(args.emb)) if args.gen else []
    )
    if args.scenario:
        pool = [sc for sc in pool if sc.name == args.scenario]
        if not pool:
            sys.exit(f"no scenario named {args.scenario!r}")

    scores: list[dict[str, Any]] = []
    fits: list[float] = []
    for sc in pool:
        v = judge(sc, cfg, planner="target", time_limit_ms=args.time_limit_ms)
        w = score_world(v)
        scores.append(w)
        fit = fitness_of(w)
        fits.append(fit)
        # Everything a future reader needs to diagnose THIS world without
        # re-running the battery: which pillar lost, and by how much.
        log_task(
            sc.name,
            score=round(fit, 4),
            summary=(
                f"{sc.name}: fitness {fit:.2f}/{FITNESS_MAX:g} (see bench/fitness.py) "
                f"from gold {w['gold']} + consist {w['consistency']} + speed {w['speed']}; "
                f"{v.avoid_ms:.1f} ms vs {SPEED_BUDGET_MS:.0f} ms budget; "
                f"referee total {w['total']}/111"
            ),
            failure_reason=(
                f"dq={w['dq']}" if w["dq"] else
                "; ".join(
                    p for p in (
                        f"gold {w['gold']}" if w["gold"] < 0.999 else "",
                        f"consistency {w['consistency']}" if w["consistency"] < 0.999 else "",
                        f"speed {w['speed']} ({v.avoid_ms:.0f} ms)" if w["speed"] < 0.999 else "",
                    ) if p
                ) or None
            ),
            note=sc.note,
            embodiment=sc.emb.tag,
            split="curated" if sc.name in curated else "generated",
            # Pillars, as the referee scored them.
            gold=w["gold"],
            consistency=w["consistency"],
            speed=w["speed"],
            dq=w["dq"],
            referee_total=w["total"],
            # The raw measurements behind each pillar.
            avoid_ms=round(v.avoid_ms, 3),
            speed_budget_ms=SPEED_BUDGET_MS,
            over_budget_x=round(v.avoid_ms / SPEED_BUDGET_MS, 3),
            timed_out=bool(v.timed_out),
            flicker=round(v.flicker, 6),
            consist=round(v.consist, 6),
            min_truth=_finite(v.min_truth),
            min_scored=_finite(v.min_scored),
            veto=bool(v.veto),
            fans=int(v.fans),
            lat_max=round(v.lat_max, 4),
            path_poses=len(v.final.poses),
            gold_ms=round(v.gold_ms, 3),
            gold_exists=v.gold is not None,
            **prov,
        )

    gen_speeds = [w["speed"] for w, sc in zip(scores, pool) if sc.name not in curated]
    cur_speeds = [w["speed"] for w, sc in zip(scores, pool) if sc.name in curated]
    ref = summarize(scores)
    fitness = round(sum(fits) / len(fits), 4) if fits else 0.0
    worst_gold = min((w["gold"] for w in scores), default=0.0)
    worst_fit = min(zip([w["name"] for w in scores], fits), key=lambda t: t[1], default=None)
    summary = {
        "score": fitness,  # what evo climbs
        "fitness": fitness,
        "fitness_max": FITNESS_MAX,
        "referee_score": ref["score"],  # what review ships on
        "referee_max": 111.0,
        "referee_worst": ref["worst"],
        "worst_fitness": {"name": worst_fit[0], "fitness": round(worst_fit[1], 2)}
        if worst_fit else None,
        "dq": ref["dq"],
        "timeouts": ref.get("timeouts", 0),
        "gold": ref["gold"],
        "worst_world_gold": round(worst_gold, 4),
        # Worlds where gold closeness collapsed entirely -- refused, stalled,
        # vetoed, or deviated past half the gold path's length. The count is
        # gated; the names are what failure analysis starts from.
        "gold_zero": [w["name"] for w in scores if not w["dq"] and w["gold"] <= 0.0],
        "consistency": ref["consistency"],
        "speed": ref["speed"],
        # Split out so the generalization gate can compare generated-world
        # speed against generated-world speed. The curated 16 are far smaller
        # and far faster, so a whole-battery mean is not comparable to a
        # generated-only held-out mean.
        "speed_generated": round(sum(gen_speeds) / len(gen_speeds), 4) if gen_speeds else None,
        "speed_curated": round(sum(cur_speeds) / len(cur_speeds), 4) if cur_speeds else None,
        "worlds": ref["worlds"],
        "gen": args.gen,
        "seed": args.seed,
        "pinned_core": core,
        "calibration_ms": calib_ms,
        "score_scale": (
            f"fitness, 0..{FITNESS_MAX:g} (weights in bench/fitness.py; per-world "
            f"trace scores share this scale)"
        ),
        "time_limit_ms": args.time_limit_ms,
        "scenario_filter": args.scenario,
        "curated_included": not args.no_curated,
        "planner_sha256": planner_fingerprint(),
        **prov,
    }
    if args.summary_json:
        Path(args.summary_json).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    # Breadcrumb for bench/quality.py, so the quality gate can validate the
    # run that just happened rather than re-running the whole battery.
    crumb = summary_path(args.gen, args.seed, not args.no_curated)
    crumb.parent.mkdir(parents=True, exist_ok=True)
    crumb.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary), file=sys.stderr)

    write_result(fitness)

    breaches = []
    checks = [
        ("fitness", args.min_fitness, summary["fitness"], "min"),
        ("referee_score", args.min_referee_score, summary["referee_score"], "min"),
        ("gold", args.min_gold, summary["gold"], "min"),
        ("worst_world_gold", args.min_world_gold, summary["worst_world_gold"], "min"),
        ("speed", args.min_speed, summary["speed"], "min"),
        ("consistency", args.min_consistency, summary["consistency"], "min"),
        ("dq", args.max_dq, summary["dq"], "max"),
    ]
    for name, want, got, direction in checks:
        if want is None:
            continue
        if (direction == "min" and got < want - 1e-9) or (direction == "max" and got > want + 1e-9):
            breaches.append(f"{name}={got} violates {direction}={want}")
    if breaches:
        print("GATE FAILED: " + "; ".join(breaches), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
