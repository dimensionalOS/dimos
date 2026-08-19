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

"""Gates for the ``agent_encode`` autoresearch loop. Exit 0 = pass.

    python -m dimos.evals.temp.tool_evo_gate static    # pre: benchmark unedited, encoder honest
    python -m dimos.evals.temp.tool_evo_gate budget    # pre: encoding stays small and fast
    python -m dimos.evals.temp.tool_evo_gate floors    # post: no family regressed
    python -m dimos.evals.temp.tool_evo_gate freeze    # (setup) re-record the hash manifest

``static`` and ``budget`` are pre-gates: they decide from the worktree alone,
so a candidate that edits the benchmark or bloats the encoding fails before a
single model call is paid for. ``floors`` re-reads the artifact the benchmark
already wrote, so it costs nothing.

The manifest cannot defend itself — an optimizer that rewrites this file and
``evo_frozen.json`` together passes. What catches that is the per-experiment
diff evo shows and the review before anything lands; the gate is here to make
accidental drift loud, not to be tamper-proof.
"""

from __future__ import annotations

import argparse
import ast
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any

REPO = Path(__file__).resolve().parents[3]
MANIFEST = Path(__file__).parent / "evo_frozen.json"
FLOORS = Path(__file__).parent / "evo_floors.json"
BENCH_ARTIFACT = Path(".evo_bench/bench.json")

FROZEN_GLOBS = (
    "dimos/evals/*.py",
    "dimos/evals/temp/*.py",
    "dimos/evals/suites/*.py",
    "dimos/evals/suites/*.json",
)
"""Everything the benchmark is made of. The optimizer edits the encoder, not
the questions, the scorers, the slices, or the harness."""

TARGET = Path("dimos/msgs/sensor_msgs/PointCloud2.py")
"""The one file the loop may change."""

BANNED_IMPORTS = (
    # the encoder may not import the answer: route truth is our own planner,
    # clearance and glass truth are the eval generators.
    "dimos.navigation",
    "dimos.mapping",
    "dimos.evals",
    "dimos.perception",
    # nor reach outside the process for one
    "subprocess",
    "socket",
    "requests",
    "urllib",
    "http",
    "importlib",
    "pickle",
)
BANNED_NAMES = ("open", "eval", "exec", "compile", "__import__", "getenv", "environ")

MAX_BYTES = 6_000
"""Ceiling on one frame's encoding, JSON bytes. This runs on a robot and lands
in a context window eight frames at a time; "emit more" is the cheapest way to
win a VQA benchmark and the worst way to win it. The seed encoder measures
2.6-3.3 kB over BUDGET_SAMPLE, so this leaves room for a free-space channel
and not for dumping the cloud."""

MAX_MS = 80.0
"""Ceiling on mean encode time per frame over the same sample, where the seed
encoder takes ~14 ms. Wide enough for real work, narrow enough that nobody
runs a planner in here."""

BUDGET_SAMPLE: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("go2_short", (5.0, 20.0, 40.0, 58.0)),
    ("go2_china_office", (25.0, 62.0, 100.0, 130.0)),
)

TOLERANCE = 0.05
"""How far a frozen family may sag before the floors gate fails. Loose enough
for model sampling noise, tight enough that a real regression cannot hide."""


def _fail(message: str) -> int:
    print(f"GATE FAIL: {message}", file=sys.stderr)
    return 1


def _frozen_files() -> list[Path]:
    return sorted({p for glob in FROZEN_GLOBS for p in REPO.glob(glob)})


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# -- freeze / static -------------------------------------------------------------


def freeze(_: argparse.Namespace) -> int:
    """Record the hash of every benchmark file. Run after any deliberate change."""
    manifest = {str(p.relative_to(REPO)): _sha(p) for p in _frozen_files()}
    MANIFEST.write_text(json.dumps({"files": manifest}, indent=2, sort_keys=True) + "\n")
    print(f"froze {len(manifest)} files into {MANIFEST}")
    return 0


def _banned_reach(source: str) -> list[str]:
    """Imports and builtins the encoder has no business using."""
    tree = ast.parse(source)
    found: list[str] = []
    for node in ast.walk(tree):
        modules: list[str] = []
        if isinstance(node, ast.Import):
            modules = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            modules = [node.module or ""]
        for module in modules:
            if any(module == b or module.startswith(f"{b}.") for b in BANNED_IMPORTS):
                found.append(f"line {node.lineno}: imports {module}")
        if isinstance(node, ast.Name) and node.id in BANNED_NAMES:
            found.append(f"line {node.lineno}: uses {node.id}")
        if isinstance(node, ast.Attribute) and node.attr in BANNED_NAMES:
            found.append(f"line {node.lineno}: uses .{node.attr}")
    return found


def static(_: argparse.Namespace) -> int:
    """The benchmark is byte-identical, and the encoder reaches nowhere it shouldn't."""
    if not MANIFEST.exists():
        return _fail(f"no {MANIFEST.name}; run `tool_evo_gate freeze` on the baseline first")
    recorded: dict[str, str] = json.loads(MANIFEST.read_text())["files"]
    live = {str(p.relative_to(REPO)): _sha(p) for p in _frozen_files()}
    changed = sorted(k for k in recorded.keys() & live.keys() if recorded[k] != live[k])
    missing = sorted(recorded.keys() - live.keys())
    stray = sorted(live.keys() - recorded.keys())
    if changed or missing or stray:
        return _fail(
            f"benchmark edited — changed {changed}, missing {missing}, added {stray}. "
            "The encoder is the only thing under optimization."
        )

    target = REPO / TARGET
    if not target.exists():
        return _fail(f"{TARGET} is gone")
    reach = _banned_reach(target.read_text())
    if reach:
        return _fail(f"{TARGET} reaches outside the encoding: " + "; ".join(reach))
    print(f"static: {len(recorded)} benchmark files unchanged, {TARGET} clean")
    return 0


# -- budget ----------------------------------------------------------------------


def _encode_sample() -> list[tuple[str, int, float]]:
    """``(label, json bytes, ms)`` for each sampled frame."""
    from dimos.evals.generate import _dataset, _frame_at

    out: list[tuple[str, int, float]] = []
    for name, timestamps in BUDGET_SAMPLE:
        with _dataset(name) as store:
            for t in timestamps:
                cloud, _ = _frame_at(store, t)
                cloud.points_f32()  # warm the decode; we time the encoding
                start = time.perf_counter()
                encoded = cloud.agent_encode()
                ms = (time.perf_counter() - start) * 1000.0
                out.append((f"{name}@{t:g}", len(json.dumps(encoded)), ms))
    return out


def budget(args: argparse.Namespace) -> int:
    """One frame's encoding stays small enough to read and fast enough to ship."""
    try:
        sample = _encode_sample()
    except Exception as e:
        return _fail(f"agent_encode raised on the sample: {e!r}")
    sizes = sorted(size for _, size, _ in sample)
    p95 = sizes[min(len(sizes) - 1, int(0.95 * len(sizes)))]
    mean_ms = sum(ms for _, _, ms in sample) / len(sample)
    if args.report:
        for label, size, ms in sample:
            print(f"{label:26} {size:7} B  {ms:7.1f} ms")
    print(f"budget: p95 {p95} B (max {MAX_BYTES}), mean {mean_ms:.1f} ms (max {MAX_MS})")
    if p95 > MAX_BYTES:
        return _fail(f"encoding p95 {p95} B over the {MAX_BYTES} B budget")
    if mean_ms > MAX_MS:
        return _fail(f"encoding {mean_ms:.1f} ms over the {MAX_MS} ms budget")
    return 0


# -- floors ----------------------------------------------------------------------


def _read(path: Path, what: str) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"no {what} at {path}")
    return json.loads(path.read_text())


def floors(args: argparse.Namespace) -> int:
    """No family fell below its recorded floor. Reads the benchmark's own artifact."""
    try:
        recorded = _read(Path(args.floors), "floors")["families"]
        payload = _read(Path(args.artifact), "benchmark artifact")
    except (FileNotFoundError, KeyError) as e:
        return _fail(f"{e} — record floors with `tool_evo_bench --write-floors` on the baseline")
    measured: dict[str, float] = payload["families"]
    sagged = [
        f"{family} {measured[family]:.3f} < {floor:.3f}-{args.tolerance}"
        for family, floor in recorded.items()
        if family in measured and measured[family] < floor - args.tolerance
    ]
    absent = sorted(set(recorded) - set(measured))
    if absent:
        return _fail(f"families missing from the run: {absent}")
    if sagged:
        return _fail("regressed: " + "; ".join(sagged))
    print(f"floors: {len(recorded)} families at or above baseline (tolerance {args.tolerance})")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="gate", required=True)
    sub.add_parser("freeze", help="record the benchmark hash manifest")
    sub.add_parser("static", help="benchmark unedited and encoder honest")
    budget_parser = sub.add_parser("budget", help="encoding size and latency")
    budget_parser.add_argument("--report", action="store_true", help="print every sampled frame")
    floors_parser = sub.add_parser("floors", help="no family regressed")
    floors_parser.add_argument("--artifact", default=str(BENCH_ARTIFACT))
    floors_parser.add_argument("--floors", default=str(FLOORS))
    floors_parser.add_argument("--tolerance", type=float, default=TOLERANCE)
    args = parser.parse_args(argv)

    gates: dict[str, Any] = {"freeze": freeze, "static": static, "budget": budget, "floors": floors}
    return int(gates[args.gate](args))


if __name__ == "__main__":
    sys.exit(main())
