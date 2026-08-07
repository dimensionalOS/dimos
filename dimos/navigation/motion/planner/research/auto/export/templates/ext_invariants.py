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

"""Invariants checked against the artifact that is actually scored, plus a
gold-oracle integrity spot-check.

`cargo test --no-default-features` builds an rlib with `feature = "python"`
off, so every pyo3-gated line in the candidate is invisible to it -- and the
pyo3 path is exactly where the interesting cheat lives, because `plan()` runs
inside the referee's own interpreter. This file loads the built `.so` the
battery imports and exercises it through the same entry point the referee
uses, so nothing can hide behind a cfg.

It also verifies the gold oracle has not been tampered with. `se2_path()`
memoizes into a pickle in the shared cache dir, an unsigned file that is
returned before any validation; a poisoned entry would redefine what "gold"
means for a world and both legs of the parity gate would happily agree with
each other. So: recompute two worlds' gold from geometry with the cache
bypassed, and compare.

And it pins the referee's RUNTIME: live code-object hashes and live constant
values against .evo/referee.lock -- a sitecustomize.py or .pth file in the
venv can monkeypatch the judge without touching any file the hash checks see.

Run through bench/extcheck, which supplies the venv and PYTHONPATH.
"""

from __future__ import annotations

import sys
import time
import hashlib
import json
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quality import evo_root, planner_fingerprint  # noqa: E402

import dimos_motion2_target as ext  # noqa: E402
from referee import scenarios as sc_mod  # noqa: E402
from referee.scenarios import SCENARIOS, generated, se2_path  # noqa: E402

GO2 = (0.85, 0.50, -0.01, 0.4, 0.05, 1.8, 1.5, 0.25)
REFEREE_LOCK = evo_root() / ".evo" / "referee.lock"

problems: list[str] = []


def slalom_cloud() -> np.ndarray:
    pts = []
    for cx, cy in ((1.5, 0.6), (3.0, -0.6), (4.5, 0.6), (6.0, -0.6)):
        t = -0.45
        while t <= 0.45:
            for x, y in ((cx - 0.45, cy + t), (cx + 0.45, cy + t),
                         (cx + t, cy - 0.45), (cx + t, cy + 0.45)):
                pts.append([x, y, 0.2])
            t += 0.05
    return np.ascontiguousarray(np.array(pts, dtype=np.float64))


def check_determinism(cloud: np.ndarray) -> None:
    a = ext.plan(cloud, (0.0, 0.0, 0.0), (7.5, 0.0), GO2, 0.1)
    b = ext.plan(cloud, (0.0, 0.0, 0.0), (7.5, 0.0), GO2, 0.1)
    if a is None or b is None:
        problems.append("the built module refused a world that has a route")
        return
    if a.shape != b.shape or not np.array_equal(a.view(np.uint8), b.view(np.uint8)):
        problems.append("the built module is not bit-identical across identical calls")


def check_no_memoization(cloud: np.ndarray) -> None:
    """Same control as the crate's own test, but through the real extension:
    a query already answered, against equivalent ones never seen.

    The control must differ in EVERY input. Repeating one control query lets
    a cache populate on its first call; varying only the goal still lets a
    cache keyed on the CLOUD (a stashed distance field or roadmap) through.
    Translating the whole world keeps the work identical and misses every
    possible key. Both weaker versions shipped in the original lab and both
    were defeated."""
    def timed(pts: np.ndarray, pose: tuple[float, float, float],
              goal: tuple[float, float]) -> float:
        t0 = time.process_time()
        ext.plan(pts, pose, goal, GO2, 0.1)
        return time.process_time() - t0

    timed(cloud, (0.0, 0.0, 0.0), (7.5, 0.0))  # the call a cache would populate
    rep_a = min(timed(cloud, (0.0, 0.0, 0.0), (7.5, 0.0)) for _ in range(3))

    def shifted(k: int) -> float:
        d = 0.37 * (k + 1)
        pts = np.ascontiguousarray(cloud + np.array([d, d, 0.0]))
        return timed(pts, (d, d, 0.0), (7.5 + d, d))

    fresh_b = min(shifted(k) for k in range(3))
    if rep_a < 0.7 * fresh_b:
        problems.append(
            f"repeated query cost {rep_a * 1e3:.2f} ms vs {fresh_b * 1e3:.2f} ms for an "
            f"equivalent fresh one, the same world translated ({100 * rep_a / fresh_b:.0f}%) "
            f"-- the built module reuses work across calls, which fakes the speed pillar"
        )


def check_referee_code(lock: dict) -> None:
    """Pin the referee's RUNTIME, not just its files.

    check_rules pins the referee bytes, but a `sitecustomize.py` or a `.pth`
    in the venv's site-packages auto-executes on every python start and can
    monkeypatch score_world or judge. That lives outside the referee tree
    entirely, so no file hash would ever show it. Hashing the live code
    objects catches it no matter where it came from."""
    import referee.score as score_mod
    import referee.sim as sim_mod

    live = {
        "judge": sim_mod.judge,
        "score_world": score_mod.score_world,
        "summarize": score_mod.summarize,
        "se2_path": se2_path,
    }
    want = lock.get("code_sha256", {})
    for name, fn in sorted(live.items()):
        got = hashlib.sha256(fn.__code__.co_code).hexdigest()[:16]
        if name not in want:
            problems.append(f"referee.lock has no pinned code hash for {name}()")
        elif got != want[name]:
            problems.append(
                f"the referee's live {name}() does not match referee.lock "
                f"(expected {want[name]}, got {got}) -- the judging code has been "
                f"patched at runtime, or the lock is stale"
            )


def check_referee_constants(lock: dict) -> None:
    """Pin the scoring constants BY VALUE. check_referee_code cannot see them.

    SPEED_BUDGET_MS and friends are module globals, so score_world() reads
    them with a LOAD_GLOBAL and its co_code is byte-identical whichever value
    is there. The entire definition of what the referee rewards could
    therefore be rewritten -- by an edit, or at runtime by the same
    sitecustomize.py the code pin exists to catch -- without moving any hash
    above. So compare the live values too.

    `geometry` carries the station geometry -- SCORE_STRIDE_M, and the
    sweep_yaw_step / turn_yaw_eps defaults on AvoidanceConfig. Those are not
    scoring weights, but the candidate's yaw publication schedule is derived
    from them: publish above turn_yaw_eps at a station and the box becomes
    its circumscribing cylinder. A soundness argument resting on an unpinned
    threshold is not a soundness argument."""
    import referee.geometry as geometry_mod
    import referee.score as score_mod
    import referee.sim as sim_mod

    mods = {"score": score_mod, "sim": sim_mod, "geometry": geometry_mod}
    want = lock.get("constants", {})
    if not want:
        problems.append("referee.lock has no `constants` block -- scoring constants unpinned")
        return
    for dotted, expected in sorted(want.items()):
        mod_name, _, rest = dotted.partition(".")
        obj = mods.get(mod_name)
        if obj is None:
            problems.append(f"referee.lock pins {dotted}, but module {mod_name!r} is not known here")
            continue
        # Walk the remaining dotted path, so a field default can be pinned as
        # e.g. geometry.AvoidanceConfig.turn_yaw_eps.
        for part in rest.split("."):
            # AvoidanceConfig is a pydantic model, so its field defaults are
            # not class attributes -- they only materialise on an instance.
            # Default-construct when the walk hits a class that does not
            # carry the attribute directly.
            if not hasattr(obj, part) and isinstance(obj, type):
                try:
                    obj = obj()
                except Exception:  # noqa: BLE001 -- not constructible, report below
                    pass
            if not hasattr(obj, part):
                obj = None
                break
            obj = getattr(obj, part)
        if obj is None:
            problems.append(f"referee.lock pins {dotted}, but the referee no longer defines it")
            continue
        got = obj
        if got != expected:
            problems.append(
                f"the referee's live {dotted} is {got!r}, referee.lock pins {expected!r} -- "
                f"the scoring rules have changed. Scores across this boundary are not "
                f"comparable; if the change was intended, update referee.lock and re-baseline"
            )


def check_gold_cache(tmp: Path, spot: tuple[str, ...]) -> None:
    """Gold must be a function of world geometry, not of what is in the
    pickle. Recompute with the memo bypassed and compare."""
    by_name = {s.name: s for s in list(SCENARIOS) + list(generated(40, 0))}
    real = sc_mod._SE2_CACHE
    for name in spot:
        s = by_name.get(name)
        if s is None:
            problems.append(f"spot-check world {name!r} is missing from SCENARIOS")
            continue
        cached = se2_path(s.boxes, s.start, s.goal, s.emb)
        sc_mod._SE2_CACHE = tmp / f".se2_recompute_{name}.pkl"
        try:
            fresh = se2_path(s.boxes, s.start, s.goal, s.emb)
        finally:
            sc_mod._SE2_CACHE = real
        if (cached is None) != (fresh is None):
            problems.append(f"gold for {name!r} disagrees with a cache-bypassed recompute "
                            f"(one refuses, one routes) -- the gold cache is not trustworthy")
        elif cached is not None and not np.allclose(cached, fresh, atol=1e-9):
            problems.append(f"gold for {name!r} differs from a cache-bypassed recompute -- "
                            f"the gold cache has been tampered with or is stale")


def _use_shared_world_cache() -> None:
    """Read the same world cache the battery writes, in the shared cache dir
    at the lab root (bench/run already points AUTORESEARCH_CACHE_DIR there;
    this only matches battery.py's per-(count,seed) file naming)."""
    cache = evo_root() / ".evo" / "cache"
    cache.mkdir(parents=True, exist_ok=True)
    sc_mod._CACHE = cache / "gen_cache_40_0.pkl"


def pick_spot_check() -> tuple[str, ...]:
    """One curated world and one generated world, rotated.

    A fixed pair is a pair a poisoner can simply leave honest. The index is
    derived from the current candidate sources, so it is deterministic and
    reproducible for a given commit but not choosable in advance."""
    seed = int(planner_fingerprint()[:8], 16)
    curated = sorted(s.name for s in SCENARIOS)
    gen = [s.name for s in generated(40, 0)]
    return (curated[seed % len(curated)], gen[(seed // 7) % len(gen)])


def main() -> None:
    import tempfile

    print(f"exercising {ext.__file__}", file=sys.stderr)
    lock = json.loads(REFEREE_LOCK.read_text(encoding="utf-8")) if REFEREE_LOCK.exists() else {}
    if not lock:
        problems.append(f"referee lock missing at {REFEREE_LOCK}")
    _use_shared_world_cache()
    cloud = slalom_cloud()
    check_determinism(cloud)
    check_no_memoization(cloud)
    check_referee_code(lock)
    check_referee_constants(lock)
    spot = pick_spot_check()
    with tempfile.TemporaryDirectory() as t:
        check_gold_cache(Path(t), spot)

    if problems:
        print(f"EXT INVARIANTS FAILED ({len(problems)}):", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        sys.exit(1)
    print("ext invariants ok: built module deterministic, no cross-call reuse, "
          f"referee code objects and scoring constants match the lock, gold "
          f"oracle matches a cache-bypassed recompute on {spot}")


if __name__ == "__main__":
    main()
