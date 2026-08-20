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

import re as _re

# -- prose gate ------------------------------------------------------------------

DATA_RE = _re.compile(r"^[-0-9A-U.:@,;| ]*$")
"""A string value that is encoded data: digits, the raster alphabet 0-9A-U, and
the separators the encoding uses. Anything else is prose and must be short."""

MAX_PROSE_STR = 40
"""A string value that is not pure data may still be a short label (frame_id)."""

MAX_PROSE_BYTES = 400
"""Non-data string bytes allowed in one frame's encoding, over BUDGET_SAMPLE.
Round 2 carried ~3.6 kB of description per frame; the seed carries a handful."""

BANNED_VOCAB = (
    # semantic nouns the encoder must not name -- it renders geometry, the
    # reader names the thing
    "door",
    "doorway",
    "room",
    "stair",
    "step",
    "platform",
    "landing",
    "ramp",
    # classifications the movement controller owns, not the encoder
    "free",
    "clear",
    "navigable",
    "reachable",
    "route",
    "unknown",
    # scorer / eval words -- naming the question is answering it
    "answer",
    "question",
    "count",
    # imperatives -- telling the model what to do with a field
    "never",
    "always",
    "use",
    "take",
    "instead",
    "already",
    "only",
    "do not",
    "not a",
)
"""Words that turn a rendering into a lookup table. Crude on purpose: an
optimizer that learns to dodge the list shows up in the per-experiment diff."""

LEGEND_ATTR = "AGENT_ENCODE_LEGEND"


def _banned_in(text: str) -> list[str]:
    low = text.lower()
    hits = []
    for word in BANNED_VOCAB:
        pattern = _re.escape(word) if " " in word else rf"\b{_re.escape(word)}\b"
        if _re.search(pattern, low):
            hits.append(word)
    return hits


def _string_values(node: Any) -> list[str]:
    """Every string value in a JSON-like structure (keys are structure, skipped)."""
    if isinstance(node, str):
        return [node]
    if isinstance(node, dict):
        return [v for value in node.values() for v in _string_values(value)]
    if isinstance(node, list):
        return [v for item in node for v in _string_values(item)]
    return []


def _reachable_strings(source: str) -> tuple[list[str], ast.AST | None]:
    """String literals reachable from ``agent_encode`` within the PointCloud2 class.

    Follows ``self.<name>`` / ``cls.<name>`` references from ``agent_encode`` to
    the methods and class constants it uses, transitively, and collects their
    string constants. Scoped so unrelated docstrings elsewhere in the file
    (``__add__``, ``filter_by_height``) are not swept in -- they are not part
    of the encoding.
    """
    tree = ast.parse(source)
    cls = next(
        (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "PointCloud2"),
        None,
    )
    if cls is None:
        return [], tree
    methods = {
        n.name: n for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    consts: dict[str, ast.AST] = {}
    for n in cls.body:
        if isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Name):
                    consts[t.id] = n.value
        elif (
            isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name) and n.value is not None
        ):
            consts[n.target.id] = n.value
    seen: set[str] = set()
    strings: list[str] = []
    queue = ["agent_encode", LEGEND_ATTR]
    while queue:
        name = queue.pop()
        if name in seen:
            continue
        seen.add(name)
        node = methods.get(name) or consts.get(name)
        if node is None:
            continue
        for sub in ast.walk(node):
            if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                strings.append(sub.value)
            if (
                isinstance(sub, ast.Attribute)
                and isinstance(sub.value, ast.Name)
                and sub.value.id in ("self", "cls")
            ):
                if sub.attr in methods or sub.attr in consts:
                    queue.append(sub.attr)
    return strings, tree


def _legend_is_one_constant(tree: ast.AST) -> bool:
    """AGENT_ENCODE_LEGEND is assigned exactly once, at class level, to a
    literal string expression (a name or a concatenation of string constants)."""
    assigns = []
    for cls in ast.walk(tree):
        if isinstance(cls, ast.ClassDef) and cls.name == "PointCloud2":
            for n in cls.body:
                if isinstance(n, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == LEGEND_ATTR for t in n.targets
                ):
                    assigns.append(n.value)
    if len(assigns) != 1:
        return False

    def literal(node: ast.AST) -> bool:
        if isinstance(node, ast.Constant):
            return isinstance(node.value, str)
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            return literal(node.left) and literal(node.right)
        return False

    return literal(assigns[0])


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
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    legend = hashlib.sha256(PointCloud2.AGENT_ENCODE_LEGEND.encode()).hexdigest()
    MANIFEST.write_text(
        json.dumps({"files": manifest, "agent_encode_legend_sha": legend}, indent=2, sort_keys=True)
        + "\n"
    )
    print(f"froze {len(manifest)} files + the encoder legend into {MANIFEST}")
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
    recorded_legend = json.loads(MANIFEST.read_text()).get("agent_encode_legend_sha")
    if recorded_legend is not None:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        live_legend = hashlib.sha256(PointCloud2.AGENT_ENCODE_LEGEND.encode()).hexdigest()
        if live_legend != recorded_legend:
            return _fail(
                "AGENT_ENCODE_LEGEND changed since freeze -- the format legend is frozen; "
                "re-freeze deliberately if this was intended"
            )
    print(f"static: {len(recorded)} benchmark files unchanged, {TARGET} clean, legend frozen")
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


# -- prose -----------------------------------------------------------------------


def prose(_: argparse.Namespace) -> int:
    """The encoding renders geometry and names nothing. Runs from the worktree.

    Six checks (round3.md #2): every output string is encoded data or a short
    label; non-data bytes per frame stay under the prose cap; no banned word
    appears in the output or in a string literal reachable from agent_encode;
    the legend is one class constant; every key is present on every frame; and
    the two output caps derive from one constant with the encoder no larger.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    source = (REPO / TARGET).read_text()
    strings, tree = _reachable_strings(source)

    # legend is one class constant
    if tree is None or not _legend_is_one_constant(tree):
        return _fail(f"{LEGEND_ATTR} must be exactly one class-level string constant")

    # banned vocabulary in the source reachable from agent_encode (legend included)
    for literal in strings:
        hits = _banned_in(literal)
        if hits:
            return _fail(f"banned word(s) {hits} in an encoder string literal: {literal[:60]!r}")

    # sample real frames
    try:
        from dimos.evals.generate import _dataset, _frame_at

        frames = []
        for name, timestamps in BUDGET_SAMPLE:
            with _dataset(name) as store:
                for t in timestamps:
                    frames.append(_frame_at(store, t)[0].agent_encode())
    except Exception as e:
        return _fail(f"agent_encode raised on the sample: {e!r}")
    frames.append(PointCloud2.from_numpy(__import__("numpy").zeros((0, 3))).agent_encode())

    reference_keys = _key_set(frames[-1])  # the empty cloud carries every key by construction
    for encoded in frames:
        # key stability
        if _key_set(encoded) != reference_keys:
            missing = reference_keys - _key_set(encoded)
            extra = _key_set(encoded) - reference_keys
            return _fail(f"frame key set differs: missing {missing}, extra {extra}")
        # output strings and prose bytes
        prose_bytes = 0
        for value in _string_values(encoded):
            hits = _banned_in(value)
            if hits:
                return _fail(f"banned word(s) {hits} in the output: {value[:60]!r}")
            if not DATA_RE.match(value):
                if len(value) > MAX_PROSE_STR:
                    return _fail(
                        f"non-data output string over {MAX_PROSE_STR} chars: {value[:60]!r}"
                    )
                prose_bytes += len(value.encode())
        if prose_bytes > MAX_PROSE_BYTES:
            return _fail(f"{prose_bytes} prose bytes in one frame, over {MAX_PROSE_BYTES}")

    # cap tie: encoder cap derives from one constant, encoder <= skill
    from dimos.agents.skills.memory_query import MemoryQuerySkillConfig

    skill_cap = MemoryQuerySkillConfig().max_output_chars
    if PointCloud2.ENCODE_SOFT_CAP > skill_cap:
        return _fail(f"encoder cap {PointCloud2.ENCODE_SOFT_CAP} exceeds skill cap {skill_cap}")

    print(
        f"prose: {len(frames)} frames render only data, legend frozen, "
        f"caps tied ({PointCloud2.ENCODE_SOFT_CAP} <= {skill_cap})"
    )
    return 0


def _key_set(encoded: dict[str, Any]) -> frozenset[str]:
    keys: set[str] = set()
    for k, v in encoded.items():
        keys.add(k)
        if isinstance(v, dict):
            keys |= {f"{k}.{kk}" for kk in v}
    return frozenset(keys)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="gate", required=True)
    sub.add_parser("freeze", help="record the benchmark hash manifest")
    sub.add_parser("static", help="benchmark unedited and encoder honest")
    sub.add_parser("prose", help="encoder renders geometry, names nothing")
    budget_parser = sub.add_parser("budget", help="encoding size and latency")
    budget_parser.add_argument("--report", action="store_true", help="print every sampled frame")
    floors_parser = sub.add_parser("floors", help="no family regressed")
    floors_parser.add_argument("--artifact", default=str(BENCH_ARTIFACT))
    floors_parser.add_argument("--floors", default=str(FLOORS))
    floors_parser.add_argument("--tolerance", type=float, default=TOLERANCE)
    args = parser.parse_args(argv)

    gates: dict[str, Any] = {
        "freeze": freeze,
        "static": static,
        "budget": budget,
        "prose": prose,
        "floors": floors,
    }
    return int(gates[args.gate](args))


if __name__ == "__main__":
    sys.exit(main())
