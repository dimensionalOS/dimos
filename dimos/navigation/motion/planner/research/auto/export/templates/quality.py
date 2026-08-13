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

"""The safety gate: deviation is tolerated, collisions are not.

Enforced on the FULL scored battery, because a candidate that trades one
collided world for a large speed win would still come out ahead on fitness
(a DQ costs 1/56th of the score). That trade must be impossible, not
merely unattractive.

Running the battery again just to check it would double the cost of every
experiment, so this reuses the summary `bench/run` left behind -- but only
after proving it describes the candidate sources AND the referee copy that
are checked out right now. If that proof fails for any reason, it runs the
battery itself. Reusing a stale summary is the one outcome this must never
have.

  bench/quality.py --max-dq 0 --min-gold 0.90
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

# Stdlib only, on purpose: gates run under bare `python3`, not inside the
# lab's venv, so this file must not import numpy or the referee -- directly
# or through battery.py.
HERE = Path(__file__).resolve().parent
LAB = HERE.parent  # the worktree root: candidate/, referee/, .ext/ live here


def evo_root(start: Path = LAB) -> Path:
    """The origin lab root -- the directory holding `.evo/` (locks, shared
    caches). For a fresh lab this is the lab itself; for an evo worktree it
    is found by walking up. `EVO_LAB_ROOT` overrides when worktrees live
    outside the lab tree."""
    env = os.environ.get("EVO_LAB_ROOT")
    if env:
        return Path(env)
    d = start
    while True:
        if (d / ".evo" / "referee.lock").exists():
            return d
        if d.parent == d:
            return LAB  # no marker found: assume flat lab layout
        d = d.parent


def summary_path(gen: int, seed: int, curated: bool) -> Path:
    """One breadcrumb per battery configuration.

    A single shared file would mean the curated parity run and the held-out
    run -- both of which invoke bench/run -- overwrite the scored battery's
    summary, silently forcing the safety gate to re-measure or, worse,
    approving the wrong configuration if the gate order ever moves."""
    return LAB / ".ext" / f"summary_g{gen}_s{seed}_c{int(curated)}.json"


# Must equal battery.TIME_LIMIT_MS; a summary taken under a different
# per-call budget is not the measurement this gate was asked to approve.
TIME_LIMIT_MS = 6000.0


def planner_fingerprint() -> str:
    """Everything that determines what the built .so does: the sources, and
    the manifest/lockfile that decide how they are compiled. A summary is
    only reusable if all of it is unchanged."""
    h = hashlib.sha256()
    paths = sorted((LAB / "candidate" / "src").rglob("*.rs"))
    paths += [p for p in (LAB / "candidate" / "Cargo.toml",
                          LAB / "candidate" / "Cargo.lock") if p.exists()]
    for path in paths:
        h.update(path.relative_to(LAB).as_posix().encode())
        h.update(path.read_bytes())
    return h.hexdigest()


def referee_tree_hash() -> str:
    """Content hash of the local referee copy -- the standalone analogue of
    the old lab's `git -C $DIMOS rev-parse HEAD`. Scores are only comparable
    across runs judged by byte-identical referee code."""
    h = hashlib.sha256()
    for path in sorted((LAB / "referee").rglob("*.py")):
        h.update(path.relative_to(LAB).as_posix().encode())
        h.update(path.read_bytes())
    return h.hexdigest()


def fresh_summary(gen: int, seed: int) -> tuple[dict, str]:
    """The benchmark's own summary if it provably matches this checkout,
    otherwise a summary from a battery run we do ourselves."""
    want = planner_fingerprint()
    crumb = summary_path(gen, seed, curated=True)
    why = "no summary on disk"
    if crumb.exists():
        try:
            s = json.loads(crumb.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as e:
            s, why = None, f"unreadable summary ({e})"
        if s is not None:
            # Every input that could change the numbers must match, or the
            # summary describes a different measurement than the one we are
            # being asked to approve.
            if s.get("planner_sha256") != want:
                why = "summary predates the current candidate build inputs"
            elif s.get("scenario_filter"):
                why = f"summary covers only {s['scenario_filter']!r}"
            elif (s.get("gen"), s.get("seed")) != (gen, seed):
                why = f"summary covers gen={s.get('gen')} seed={s.get('seed')}"
            elif s.get("referee_sha256") != referee_tree_hash():
                why = "summary was judged by a different referee copy"
            elif s.get("time_limit_ms") != TIME_LIMIT_MS:
                why = f"summary used a {s.get('time_limit_ms')} ms per-call budget"
            else:
                return s, f"reused the benchmark's own run ({s['worlds']} worlds)"

    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "summary.json"
        try:
            subprocess.run(
                [str(HERE / "run"), "--gen", str(gen), "--seed", str(seed),
                 "--summary-json", str(out)],
                check=True, stdout=subprocess.DEVNULL,
                env={**os.environ, "EVO_RESULT_PATH": "", "EVO_TRACES_DIR": ""},
            )
        except subprocess.CalledProcessError as e:
            # Fail closed, but say what kind of failure it was. A battery that
            # crashes (SIGSEGV/SIGBUS have both been seen here) is an infra
            # problem; reporting it as a quality regression would send the
            # next experiment chasing a defect that does not exist.
            sys.exit(
                f"QUALITY INFRA: the battery did not complete (rc={e.returncode}"
                f"{', killed by signal ' + str(-e.returncode) if e.returncode < 0 else ''}). "
                f"This is not a candidate regression -- re-run the experiment."
            )
        return json.loads(out.read_text(encoding="utf-8")), f"ran the battery ({why})"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gen", type=int, default=40)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-dq", type=int, default=0,
                    help="collisions the referee did not veto; the hard floor")
    ap.add_argument("--max-timeouts", type=int, default=0)
    ap.add_argument("--min-gold", type=float, help="floor on MEAN gold closeness")
    ap.add_argument("--max-gold-zero", type=int,
                    help="worlds allowed to lose gold closeness outright")
    ap.add_argument("--min-consistency", type=float)
    ap.add_argument("--min-fitness", type=float)
    ap.add_argument("--min-referee-score", type=float,
                    help="the ship envelope: a fitness winner that is a big "
                         "referee loser should be a stated decision, not emergent")
    args = ap.parse_args()

    s, provenance = fresh_summary(args.gen, args.seed)
    print(f"quality: {provenance}", file=sys.stderr)

    breaches = []
    for name, want, got, direction in [
        ("dq", args.max_dq, s["dq"], "max"),
        ("timeouts", args.max_timeouts, s.get("timeouts", 0), "max"),
        ("gold", args.min_gold, s["gold"], "min"),
        ("gold_zero", args.max_gold_zero, len(s.get("gold_zero", [])), "max"),
        ("consistency", args.min_consistency, s["consistency"], "min"),
        ("fitness", args.min_fitness, s["fitness"], "min"),
        ("referee_score", args.min_referee_score, s["referee_score"], "min"),
    ]:
        if want is None:
            continue
        if (direction == "min" and got < want - 1e-9) or (direction == "max" and got > want + 1e-9):
            extra = f" ({', '.join(s['gold_zero'])})" if name == "gold_zero" else ""
            breaches.append(f"{name}={got} violates {direction}={want}{extra}")

    line = (f"fitness {s['fitness']}/{s['fitness_max']} · referee {s['referee_score']}/111 · "
            f"gold {s['gold']} · consistency {s['consistency']} · speed {s['speed']} · "
            f"dq {s['dq']} · gold_zero {len(s.get('gold_zero', []))} · {s['worlds']} worlds")
    if breaches:
        print(f"QUALITY FAILED: {'; '.join(breaches)}\n  {line}", file=sys.stderr)
        sys.exit(1)
    print(f"quality ok: {line}")


if __name__ == "__main__":
    main()
