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

"""Cache warming for exported labs.

The gold cache (`.se2_cache.pkl`) is `{key-string: ndarray|None}` — no
classes pickled — so the source package's copy transfers as-is (numpy-major
sensitive, which is why the lab pyproject pins numpy). The world cache
(`.gen_cache.pkl`) pickles Scenario instances BY MODULE PATH, so it is never
copied: it is rebuilt under the lab's `referee.*` import name. With a warm
gold cache the rebuild is cheap — generation's cost is SE(2) labeling.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

_WARM_GEN = r"""
import sys
dest = sys.argv[1]
sys.path.insert(0, dest)
from pathlib import Path
from referee import scenarios as sc
cache_dir = Path(dest) / ".evo" / "cache"
for count, seed in ((40, 0), (8, 991)):
    sc._CACHE = cache_dir / f"gen_cache_{count}_{seed}.pkl"
    worlds = sc.generated(count, seed)
    print(f"gen_cache_{count}_{seed}: {len(worlds)} worlds")
"""

# Timing probe: a cache hit answers in milliseconds, a miss takes seconds of
# SE(2) search. Coarse on purpose — this is a warmth diagnostic, not a gate
# (ext_invariants owns cache INTEGRITY, via cache-bypassed recompute).
_PROBE_SE2 = r"""
import sys, time
dest = sys.argv[1]
sys.path.insert(0, dest)
from referee.scenarios import SCENARIOS, se2_path
sc = next(s for s in SCENARIOS if s.name == "zigzag_room")
t0 = time.perf_counter()
se2_path(sc.boxes, sc.start, sc.goal, sc.emb)
print(f"{time.perf_counter() - t0:.3f}")
"""


def warm(dest: Path, python: str, source_pkg: Path) -> str:
    """Copy + probe the gold cache, rebuild the world caches. Returns a
    human-readable status line for PROVENANCE.md."""
    cache_dir = dest / ".evo" / "cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    notes = []

    src_se2 = source_pkg / ".se2_cache.pkl"
    if src_se2.exists():
        shutil.copy2(src_se2, cache_dir / ".se2_cache.pkl")
        env = {"AUTORESEARCH_CACHE_DIR": str(cache_dir), "AUTORESEARCH_CACHE_RO": "1"}
        probe = subprocess.run(
            [python, "-c", _PROBE_SE2, str(dest)],
            capture_output=True,
            text=True,
            check=True,
            env=_merged(env),
        )
        secs = float(probe.stdout.strip())
        if secs < 0.5:
            notes.append(f"gold cache shipped warm (probe {secs * 1e3:.0f} ms)")
        else:
            notes.append(
                f"gold cache PROBED COLD ({secs:.1f} s for one world) — keys drifted; "
                f"first full battery will be slow while it re-warms"
            )
    else:
        notes.append("no gold cache at source — cold start, first battery is slow")

    out = subprocess.run(
        [python, "-c", _WARM_GEN, str(dest)],
        capture_output=True,
        text=True,
        check=True,
        env=_merged({"AUTORESEARCH_CACHE_DIR": str(cache_dir)}),
    )
    notes.append(
        "world caches rebuilt under the lab import name "
        f"({'; '.join(out.stdout.strip().splitlines())})"
    )
    return "; ".join(notes)


def _merged(extra: dict[str, str]) -> dict[str, str]:
    import os

    return {**os.environ, **extra}
