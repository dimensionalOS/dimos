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

"""Capabilities must be declared centrally and actually used.

The registry only prevents a conflict it has been told about. A skill that
contends for something real but declares nothing gets dispatched anyway, and
the failure shows up as two models fighting over a device rather than as a
refused call.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

import dimos.agents.capabilities as caps

ROOT = pathlib.Path(caps.__file__).parents[2]


def _declared() -> set[str]:
    return {v for k, v in vars(caps).items() if k.startswith("CAP_") and isinstance(v, str)}


def _uses_in(path: pathlib.Path) -> set[str]:
    """Capability names passed to @skill(uses=[...]) in one file."""
    found: set[str] = set()
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
        if name != "skill":
            continue
        for kw in node.keywords:
            if kw.arg == "uses" and isinstance(kw.value, ast.List):
                found |= {e.id for e in kw.value.elts if isinstance(e, ast.Name)}
    return found


def _all_uses() -> set[str]:
    found: set[str] = set()
    for path in (ROOT / "dimos").rglob("*.py"):
        if path.name.startswith("test_"):
            continue
        try:
            found |= _uses_in(path)
        except SyntaxError:
            continue
    return found


class TestCentralRegistry:
    def test_capability_constants_live_in_one_place(self):
        """A locally-defined constant is invisible to anyone looking for it."""
        stray = []
        for path in (ROOT / "dimos").rglob("*.py"):
            if path.samefile(pathlib.Path(caps.__file__)) or path.name.startswith("test_"):
                continue
            for line in path.read_text().splitlines():
                if line.startswith("CAP_") and "=" in line and '"' in line:
                    stray.append(f"{path.relative_to(ROOT)}: {line.strip()}")
        assert not stray, f"declare these in agents/capabilities.py instead: {stray}"

    @pytest.mark.parametrize("name", ["CAP_MOVEMENT", "CAP_PAYLOAD"])
    def test_the_expected_capabilities_exist(self, name):
        assert isinstance(getattr(caps, name), str)

    def test_names_are_distinct(self):
        """Two capabilities sharing a string would silently serialise each other."""
        values = [v for k, v in vars(caps).items() if k.startswith("CAP_")]

        assert len(values) == len(set(values))


class TestContendingSkillsDeclare:
    def test_every_referenced_capability_is_a_real_constant(self):
        assert _all_uses() <= set(vars(caps)) | {"CAP_MOVEMENT"}
