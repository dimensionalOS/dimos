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

"""Generate or verify ``constraints.txt``, the tested resolution a release ships.

Locally the test rewrites the file (and fails when that leaves uncommitted
changes, so the regenerated file gets committed). In CI it compares, exactly
like ``test_catalog_generation.py``. The comparison is by requirement (name,
specifier, marker) so uv's output formatting can never fail it.
"""

import os
from pathlib import Path
import subprocess

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.lock import LockIndex
from dimos.deps.policy import SHIPPED_CONSTRAINTS_FILE, load_constraints
from dimos.deps.uv import UvNotFoundError, find_uv

EXPORT = (
    "export",
    "--frozen",
    "--offline",
    "--format",
    "requirements.txt",
    "--all-extras",
    "--no-default-groups",
    "--no-dev",
    "--no-emit-project",
    "--no-editable",
    "--no-hashes",
    "--no-annotate",
    "--no-header",
)


def _requirements(text: str) -> set[tuple[str, str, str]]:
    found = set()
    for raw in text.splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            requirement = Requirement(line)
            found.add(
                (
                    canonicalize_name(requirement.name),
                    str(requirement.specifier),
                    str(requirement.marker or ""),
                )
            )
    return found


@pytest.fixture(scope="module")
def exported() -> str:
    try:
        uv = find_uv()
    except UvNotFoundError:
        pytest.skip("uv is not installed")
    completed = subprocess.run(
        [*uv, *EXPORT], capture_output=True, text=True, cwd=DIMOS_PROJECT_ROOT, check=False
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


def test_constraints_are_current(exported: str) -> None:
    if "CI" in os.environ:
        assert SHIPPED_CONSTRAINTS_FILE.exists(), f"{SHIPPED_CONSTRAINTS_FILE} does not exist"
        if _requirements(SHIPPED_CONSTRAINTS_FILE.read_text()) != _requirements(exported):
            pytest.fail(
                "constraints.txt is out of date. Run "
                "`pytest dimos/deps/test_constraints_generation.py` locally to update."
            )
        return
    current = SHIPPED_CONSTRAINTS_FILE.read_text() if SHIPPED_CONSTRAINTS_FILE.exists() else ""
    if _requirements(current) != _requirements(exported):
        SHIPPED_CONSTRAINTS_FILE.write_text(exported)
    result = subprocess.run(
        ["git", "diff", "--quiet", str(SHIPPED_CONSTRAINTS_FILE)],
        capture_output=True,
        cwd=DIMOS_PROJECT_ROOT,
    )
    if result.returncode != 0:
        pytest.fail("constraints.txt was updated and has uncommitted changes. Commit it.")


def test_constraints_cover_every_direct_requirement() -> None:
    constraints = load_constraints(SHIPPED_CONSTRAINTS_FILE)
    names = {canonicalize_name(Requirement(line).name) for line in constraints}
    index = LockIndex.load(DIMOS_PROJECT_ROOT)
    declared = set(index.core_direct)
    for info in index.extras.values():
        declared |= info.own_direct
    assert declared <= names, sorted(declared - names)
    assert "dimos" not in names
    assert all("==" in line for line in constraints)
    assert Path(SHIPPED_CONSTRAINTS_FILE).stat().st_size < 50_000
