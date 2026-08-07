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

"""Lock-file generation for exported labs.

The trust chain, identical in shape to the source lab's:
`bench/trust.lock` (tracked, inside every worktree) pins the three `.evo/`
integrity files; `.evo/harness.lock` pins the harness bytes;
`.evo/referee.lock` pins the referee copy — by file hash, by live code
object, and by constant value. A one-sided edit fails `bench_guard`; a
matched two-sided edit shows up as a tracked-file change in the experiment
diff. Nothing is unwritable; tampering is visible.
"""

from __future__ import annotations

from datetime import date
import hashlib
import json
from pathlib import Path
import subprocess

HARNESS_FILES = [
    "bench/_evo_inline.py",
    "bench/battery.py",
    "bench/check_rules.py",
    "bench/ext_invariants.py",
    "bench/extcheck",
    "bench/fitness.py",
    "bench/frozen.json",
    "bench/generalization.py",
    "bench/parity",
    "bench/quality.py",
    "bench/run",
    "candidate/Cargo.lock",
    "candidate/tests/invariants.rs",
    "eval",
]

TRUST_FILES = [".evo/harness.lock", ".evo/bench_guard.py", ".evo/referee.lock"]

# The referee's runtime pins, read live from the exported copy. Dotted names
# resolve against {score, sim, geometry} in ext_invariants.check_referee_constants.
_PROBE = r"""
import hashlib, json, sys
sys.path.insert(0, sys.argv[1])
import referee.geometry as geometry
import referee.score as score
import referee.sim as sim
from referee.scenarios import se2_path

code = {
    name: hashlib.sha256(fn.__code__.co_code).hexdigest()[:16]
    for name, fn in {
        "judge": sim.judge,
        "score_world": score.score_world,
        "summarize": score.summarize,
        "se2_path": se2_path,
    }.items()
}
cfg = geometry.AvoidanceConfig()
constants = {
    "score.SPEED_BUDGET_MS": score.SPEED_BUDGET_MS,
    "score.GOLD_SCALE": score.GOLD_SCALE,
    "score.CONSIST_SCALE": score.CONSIST_SCALE,
    "score.STALL_ARC": score.STALL_ARC,
    "sim.REPEATS": sim.REPEATS,
    "sim.SPOT_STEP": sim.SPOT_STEP,
    "sim.CLOUD_STEP": sim.CLOUD_STEP,
    "sim.TRUTH_STEP": sim.TRUTH_STEP,
    "geometry.SCORE_STRIDE_M": geometry.SCORE_STRIDE_M,
    "geometry.AvoidanceConfig.sweep_yaw_step": cfg.sweep_yaw_step,
    "geometry.AvoidanceConfig.turn_yaw_eps": cfg.turn_yaw_eps,
}
print(json.dumps({"code_sha256": code, "constants": constants}))
"""

PINNED_BECAUSE = (
    "Exported standalone: the referee is a local copy, content-hash pinned; "
    "source_commit is provenance only. This referee corresponds to the source "
    "lab's epoch 3: SPEED_BUDGET_MS 20 (was 40), avoid_ms measured as "
    "time.process_time with BLAS pools pinned to 1 thread, and the gold "
    "oracle's densify fix (ceil(|dyaw|/0.045)) in place. Scores judged by any "
    "other referee bytes are not comparable; if a referee change is intended, "
    "re-export, re-baseline, and say so in project.md."
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_files(dest: Path, subdir: str) -> dict[str, str]:
    """relpath -> sha256 for every regular file under dest/subdir."""
    out: dict[str, str] = {}
    for path in sorted((dest / subdir).rglob("*")):
        if path.is_dir() or "__pycache__" in path.parts:
            continue
        out[path.relative_to(dest).as_posix()] = sha256_file(path)
    return out


def write_frozen(dest: Path) -> None:
    """The frozen python surface, hashed from the exported bytes."""
    frozen = {
        rel: sha256_file(dest / rel) for rel in ("candidate/src/python.rs", "candidate/src/lib.rs")
    }
    (dest / "bench" / "frozen.json").write_text(json.dumps(frozen, indent=2) + "\n")


def write_referee_lock(dest: Path, python: str, source_repo: str, source_commit: str) -> None:
    probe = subprocess.run(
        [python, "-c", _PROBE, str(dest)], capture_output=True, text=True, check=True
    )
    live = json.loads(probe.stdout)
    lock = {
        "pinned_at": date.today().isoformat(),
        "source_repo": source_repo,
        "source_commit": source_commit,
        "pinned_because": PINNED_BECAUSE,
        "files": tree_files(dest, "referee"),
        **live,
    }
    (dest / ".evo" / "referee.lock").write_text(json.dumps(lock, indent=2) + "\n")


def write_harness_lock(dest: Path) -> None:
    lock = {
        "pinned_at": date.today().isoformat(),
        "comment": (
            "sha256 of every measurement-harness file. bench_guard.py verifies "
            "these against each worktree; the harness measures the candidate, "
            "it is not part of the candidate. Update deliberately, with a "
            "re-baseline, never as a side effect."
        ),
        "files": {rel: sha256_file(dest / rel) for rel in HARNESS_FILES},
    }
    (dest / ".evo" / "harness.lock").write_text(json.dumps(lock, indent=2) + "\n")


def write_trust_lock(dest: Path) -> None:
    lock = {
        "comment": (
            "sha256 of the .evo/ integrity files, tracked INSIDE the worktree "
            "so bench_guard can verify its verifiers. This does not make "
            "anything unwritable; it makes tampering visible."
        ),
        "files": {rel: sha256_file(dest / rel) for rel in TRUST_FILES},
    }
    (dest / "bench" / "trust.lock").write_text(json.dumps(lock, indent=2) + "\n")
