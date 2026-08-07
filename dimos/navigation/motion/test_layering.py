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

"""The one-way rule that makes a shared judge mean something.

``research/*`` imports the referee; the referee imports nothing from research.
The moment a judge knows which candidate it is scoring, the numbers stop being
comparable across candidates -- and that comparison is the whole point of
having one referee for the shipped law, an autoresearch lab's and a learned
one. Cheap to state, easy to violate by accident, so it is a test.
"""

from __future__ import annotations

import ast
from pathlib import Path

MOTION = Path(__file__).resolve().parent
# lab-convention code copied into exported labs verbatim; not ours to shape
SKIP_PARTS = {"__pycache__", "templates"}


def _sources() -> list[Path]:
    return [
        p
        for p in sorted(MOTION.rglob("*.py"))
        if not SKIP_PARTS & set(p.parts) and "research" not in p.relative_to(MOTION).parts
    ]


def _imported_modules(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.extend(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            # level > 0 is a relative import; only absolute ones can name a
            # research package from outside it, and outside is all we scan.
            if node.module and node.level == 0:
                names.append(node.module)
    return names


def test_nothing_outside_research_imports_a_candidate() -> None:
    """Referee, production and adapter never import ``motion.*.research.*``."""
    offenders = [
        f"{p.relative_to(MOTION)} -> {mod}"
        for p in _sources()
        for mod in _imported_modules(p)
        if ".research." in f"{mod}." and "navigation.motion" in mod
    ]
    assert not offenders, "the referee must not know which candidate it scores: " + "; ".join(
        offenders
    )


def test_referee_is_reachable_from_research() -> None:
    """The scaffolds exist and the referee packages they import are importable."""
    for side in ("planner", "control"):
        for kind in ("auto", "ml"):
            assert (MOTION / side / "research" / kind).is_dir(), f"{side}/research/{kind} missing"
        assert (MOTION / side / "referee").is_dir(), f"{side}/referee missing"
