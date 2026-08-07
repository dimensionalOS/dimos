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

"""The anti-memorization gate: speed must generalize, not just exist.

Every other gate scores worlds. This one compares two scorings, because the
attack it exists to stop is invisible in either one alone.

A lookup table keyed on a hash of (points, pose, goal, emb), holding
precomputed paths for the 56 scored worlds and falling back to the honest
planner otherwise, scores ~1.0 speed on the battery and trips nothing: it is
not `static mut`, it is not a cross-CALL cache so the memoization tests pass,
and on held-out worlds it simply plans honestly, so gold and DQ floors are
all satisfied there too. What it cannot fake is the SHAPE: blazing on the
worlds it memorized, ordinary on the ones it did not.

So the signal is the gap. Held-out speed is compared against the scored
battery's speed **on generated worlds only** -- comparing against the full
battery mean would be comparing different world mixes, since the 16 curated
worlds are much smaller and much faster than the generated ones.

Both numbers already exist in the breadcrumbs bench/run leaves behind; this
gate does no measuring of its own.

Stdlib only -- gates run outside the lab's venv.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quality import planner_fingerprint, referee_tree_hash, summary_path  # noqa: E402


def load(gen: int, seed: int, curated: bool, label: str) -> dict:
    path = summary_path(gen, seed, curated)
    if not path.exists():
        sys.exit(
            f"GENERALIZATION INFRA: no {label} summary at {path.name} -- this gate reads what "
            f"the battery and the holdout gate already measured, so both must run before it."
        )
    s = json.loads(path.read_text(encoding="utf-8"))
    if s.get("planner_sha256") != planner_fingerprint():
        sys.exit(f"GENERALIZATION INFRA: the {label} summary predates the current build inputs.")
    if s.get("referee_sha256") != referee_tree_hash():
        sys.exit(f"GENERALIZATION INFRA: the {label} summary was judged by another referee.")
    return s


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gen", type=int, default=40, help="scored battery generated count")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--holdout-gen", type=int, default=8)
    ap.add_argument("--holdout-seed", type=int, default=991)
    ap.add_argument("--min-ratio", type=float, default=0.5,
                    help="held-out speed as a fraction of scored generated-world speed")
    args = ap.parse_args()

    scored = load(args.gen, args.seed, True, "scored battery")
    held = load(args.holdout_gen, args.holdout_seed, False, "held-out")

    # Like for like: generated worlds on both sides.
    seen = scored.get("speed_generated")
    unseen = held["speed"]
    if seen is None:
        sys.exit("GENERALIZATION INFRA: scored summary has no speed_generated field.")

    ratio = unseen / seen if seen > 1e-9 else 1.0
    line = (f"held-out speed {unseen:.4f} vs scored generated-world speed {seen:.4f} "
            f"(ratio {ratio:.2f}, floor {args.min_ratio})")
    if ratio < args.min_ratio:
        print(
            f"GENERALIZATION FAILED: {line}\n"
            f"  The candidate is dramatically faster on the worlds it is scored on than on "
            f"worlds it has never seen. That is the signature of memorized answers, not of a "
            f"faster planner. If this is a real effect -- e.g. the held-out worlds are simply "
            f"larger -- say so explicitly and move the floor on purpose.",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"generalization ok: {line}")


if __name__ == "__main__":
    main()
