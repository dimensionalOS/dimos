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
"""evo gate: the experiment changed only the TypeSafe state and question text.

Exits 1 when anything outside the agent package changed, when answer decoding, steering, config
or the module's streams differ from BASE, when `_tick` stops passing the model's answers straight
to `decode` and `_steer`, when the ground truth changed, or when the agent package reaches for
planner output or names a scored scene.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
from pathlib import Path
import re
import subprocess
import sys

BASE = "f8dedf451"
GROUND_TRUTH_SHA = "31f47c58851dfe10"
PACKAGE = "dimos/agents/typesafe"
MAY_CHANGE = re.compile(
    rf"^({PACKAGE}/(world_state|drive|agent|demo_objects|test_\w+)\.py|misc/evals/ts_state_\w+\.(py|json))$"
)
# Top-level names and Class.member names that must match BASE, per file.
FROZEN_NAMES = {
    f"{PACKAGE}/drive.py": {"AXES", "Drive", "_choice", "_noul", "decode"},
    f"{PACKAGE}/agent.py": {
        "PUBLISH_HZ", "SLOW_WITHIN_M", "TURN_FULL_AT_DEG", "TypeSafeAgentConfig",
        "TypeSafeAgent._steer", "TypeSafeAgent._set_target", "TypeSafeAgent._publish_loop",
        "TypeSafeAgent._infer_loop", "TypeSafeAgent.odom", "TypeSafeAgent.odometry",
        "TypeSafeAgent.detections_3d", "TypeSafeAgent.detections_2d", "TypeSafeAgent.lidar",
        "TypeSafeAgent.human_input", "TypeSafeAgent.cmd_vel", "TypeSafeAgent.agent",
        "TypeSafeAgent.agent_idle", "TypeSafeAgent.finished",
    },
}  # fmt: skip
PLANNER_LEAK = re.compile(
    r"navmesh|pathfinder|find_path|mls_planner|replanning_a_star|geodesic|shortest_path"
    r"|habitat_sim|nav_msgs\.Path|In\[Path\]|reference_paths|misc/evals"
)
SCENE_ID = re.compile(r"\b(00861|10[2-8]\d{6})")


def named_nodes(source: str) -> dict[str, str]:
    out: dict[str, str] = {}

    def names(node: ast.stmt) -> list[str]:
        if isinstance(node, ast.Assign):
            return [t.id for t in node.targets if isinstance(t, ast.Name)]
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            return [node.target.id]
        return [getattr(node, "name", "")]

    for node in ast.parse(source).body:
        for name in names(node):
            out[name] = ast.dump(node)
        if isinstance(node, ast.ClassDef):
            for sub in node.body:
                for name in names(sub):
                    out[f"{node.name}.{name}"] = ast.dump(sub)
    return out


def tick_problems(source: str) -> list[str]:
    """`answers` comes from the client once, goes to decode; `drive` from decode once, to _steer."""
    cls = next(
        n
        for n in ast.parse(source).body
        if isinstance(n, ast.ClassDef) and n.name == "TypeSafeAgent"
    )
    tick = next(n for n in cls.body if getattr(n, "name", "") == "_tick")
    assigned: dict[str, list[str]] = {"answers": [], "drive": []}
    for node in ast.walk(tick):
        targets = (
            node.targets if isinstance(node, ast.Assign)
            else [node.target] if isinstance(node, (ast.AnnAssign, ast.AugAssign, ast.NamedExpr))
            else []
        )  # fmt: skip
        for t in targets:
            for name in ast.walk(t):
                if isinstance(name, ast.Name) and name.id in assigned:
                    assigned[name.id].append(ast.unparse(node.value))  # type: ignore[attr-defined]
    out = []
    if assigned["answers"] != ["self._client(state, qs)"]:
        out.append(f"_tick: answers must be assigned once from the client: {assigned['answers']}")
    if len(assigned["drive"]) != 1 or not assigned["drive"][0].startswith("decode(answers,"):
        out.append(
            f"_tick: drive must be assigned once from decode(answers, ...): {assigned['drive']}"
        )
    if "self._steer(state, drive)" not in ast.unparse(tick):
        out.append("_tick: drive must go to self._steer(state, drive)")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parents[2]))
    root = Path(ap.parse_args().root).resolve()

    def git(*args: str) -> str:
        return subprocess.run(["git", *args], cwd=root, capture_output=True, text=True).stdout

    problems: list[str] = []
    touched = git("diff", "--name-only", BASE).split() + [
        line[3:] for line in git("status", "--porcelain").splitlines()
    ]
    problems += [
        f"outside the allowed surface: {p}" for p in sorted(set(touched)) if not MAY_CHANGE.match(p)
    ]
    for path, names in FROZEN_NAMES.items():
        base, now = (
            named_nodes(git("show", f"{BASE}:{path}")),
            named_nodes((root / path).read_text()),
        )
        problems += [
            f"{path}: {n} differs from {BASE}" for n in sorted(names) if base.get(n) != now.get(n)
        ]
    problems += tick_problems((root / PACKAGE / "agent.py").read_text())
    truth = sorted(
        p
        for p in (root / "misc/habitat/ground_truth/hssd").glob("*.json")
        if "top_down" not in p.name
    )
    digest = hashlib.sha256(b"".join(p.read_bytes() for p in truth)).hexdigest()[:16]
    if digest != GROUND_TRUTH_SHA:
        problems.append(f"ground truth changed: {digest}")
    for py in (root / PACKAGE).glob("*.py"):
        text = py.read_text()
        if not py.name.startswith("test_"):
            if PLANNER_LEAK.search(text):
                problems.append(f"planner output referenced in {py.name}")
            if SCENE_ID.search(text):
                problems.append(f"scene id hardcoded in {py.name}")
    if problems:
        print("\n".join(problems))
        sys.exit(1)
    print("frozen surface intact")


if __name__ == "__main__":
    main()
