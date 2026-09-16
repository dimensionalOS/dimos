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

"""Generate or verify ``blueprint_catalog.json``.

Locally the test rewrites the file (and fails when that leaves uncommitted
changes, so the regenerated file gets committed). In CI it compares and
fails with a diff, exactly like ``test_all_blueprints_generation.py``.
"""

import difflib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.analysis import Analyzer
from dimos.deps.build_catalog import CatalogBuild, build_catalog, render
from dimos.deps.catalog import CATALOG_PATH, CATALOG_VERSION
from dimos.deps.imports import is_excluded_from_check, iter_source_files
from dimos.deps.lock import LockIndex
from dimos.robot.all_blueprints import all_blueprints, all_modules

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"
SIZE_LIMIT = 120_000


@pytest.fixture(scope="module")
def build() -> CatalogBuild:
    return build_catalog(DIMOS_PROJECT_ROOT)


def test_no_findings(build: CatalogBuild) -> None:
    listing = "\n".join(f"  - {finding}" for finding in build.findings)
    assert not build.findings, f"unexplained imports in blueprint closures:\n{listing}"


def test_catalog_is_current(build: CatalogBuild) -> None:
    generated = render(build.catalog)
    if "CI" in os.environ:
        assert CATALOG_PATH.exists(), f"{CATALOG_PATH} does not exist"
        current = CATALOG_PATH.read_text()
        if current != generated:
            diff = "".join(
                difflib.unified_diff(
                    current.splitlines(keepends=True),
                    generated.splitlines(keepends=True),
                    fromfile="blueprint_catalog.json (current)",
                    tofile="blueprint_catalog.json (generated)",
                )
            )
            pytest.fail(
                "blueprint_catalog.json is out of date. Run "
                "`pytest dimos/deps/test_catalog_generation.py` locally to update.\n\n"
                f"Diff:\n{diff}"
            )
        return
    CATALOG_PATH.write_text(generated)
    result = subprocess.run(
        ["git", "diff", "--quiet", str(CATALOG_PATH)], capture_output=True, cwd=DIMOS_PROJECT_ROOT
    )
    if result.returncode != 0:
        pytest.fail("blueprint_catalog.json was updated and has uncommitted changes. Commit it.")


def test_catalog_keys_match_registry(build: CatalogBuild) -> None:
    catalog = build.catalog
    assert set(catalog["blueprints"]) == set(all_blueprints)  # type: ignore[arg-type]
    assert set(catalog["modules"]) == set(all_modules)  # type: ignore[arg-type]


def test_catalog_size(build: CatalogBuild) -> None:
    size = len(render(build.catalog).encode())
    assert size < SIZE_LIMIT, f"catalog is {size} bytes; keep it under the large-file limit"


def test_every_production_file_is_explained() -> None:
    """Standalone check: each file's imports, eager and lazy, are covered by its declarations."""
    analyzer = Analyzer(DIMOS_PROJECT_ROOT, LockIndex.load(DIMOS_PROJECT_ROOT))
    findings = []
    for path in iter_source_files(DIMOS_DIR):
        if is_excluded_from_check(path.relative_to(DIMOS_DIR).as_posix()):
            continue
        _requirements, file_findings, _warnings = analyzer.check_file(analyzer.scan(path))
        findings.extend(file_findings)
    listing = "\n".join(f"  - {finding}" for finding in findings)
    assert not findings, f"files importing what they do not declare:\n{listing}"


def test_registries_cover_every_manifest_name(build: CatalogBuild) -> None:
    """Every registry manifest name has an entry, and every declared selector family exists."""
    registries = build.catalog["registries"]
    assert isinstance(registries, dict)
    analyzer = Analyzer(DIMOS_PROJECT_ROOT, LockIndex.load(DIMOS_PROJECT_ROOT))
    for family, name in analyzer.manifests():
        assert name in registries[family], (family, name)
    assert registries["adapter"]["xarm"]["extras"] == {
        "control": ["hardware.manipulators.xarm.adapter"]
    }
    assert "sim" in registries["connection"]["mujoco"]["extras"]
    assert registries["simulation"]["mujoco"]["extras"]["sim"]
    for section in ("blueprints", "modules"):
        entries = build.catalog[section]
        assert isinstance(entries, dict)
        for entry in entries.values():
            for instance, table in entry.get("selectors", {}).items():
                for family in table.values():
                    assert family in registries, (instance, family)


def test_generation_ignores_the_environment(
    build: CatalogBuild, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The catalog must not depend on env vars, .env or the installed packages."""
    monkeypatch.setenv("SIMULATION", "mujoco")
    monkeypatch.setenv("VIEWER", "none")
    monkeypatch.setenv("ROBOT_IP", "mujoco")
    monkeypatch.setattr("importlib.metadata.packages_distributions", lambda: {})
    assert render(build_catalog(DIMOS_PROJECT_ROOT).catalog) == render(build.catalog)


def test_generation_needs_only_bootstrap_imports() -> None:
    """Building the catalog must not import the runtime framework or heavy libraries."""
    script = (
        "import sys\n"
        "from dimos.deps.build_catalog import build_catalog\n"
        "build_catalog()\n"
        "names = {m.split('.')[0] for m in sys.modules}\n"
        "print(sorted(n for n in names if n.startswith('dimos.') or n == 'dimos'))\n"
        "banned = {'torch', 'cv2', 'open3d', 'rerun', 'scipy', 'numpy', 'pydantic', 'zenoh'}\n"
        "assert not banned & names, banned & names\n"
        "dimos_modules = {m for m in sys.modules if m.startswith('dimos.')}\n"
        "allowed = {'dimos.constants', 'dimos.robot', 'dimos.robot.all_blueprints'}\n"
        "extra = {m for m in dimos_modules if not m.startswith('dimos.deps') and m not in allowed}\n"
        "assert not extra, extra\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, cwd=DIMOS_PROJECT_ROOT
    )
    assert result.returncode == 0, result.stderr


def test_committed_catalog_parses() -> None:
    data = json.loads(CATALOG_PATH.read_text())
    assert data["version"] == CATALOG_VERSION
    assert Path(CATALOG_PATH).stat().st_size < SIZE_LIMIT


def test_cli_entry_point_is_core_only() -> None:
    """`dimos --help`, deps, doctor and prepare must work in a core-only installation."""
    analyzer = Analyzer(DIMOS_PROJECT_ROOT, LockIndex.load(DIMOS_PROJECT_ROOT))
    from dimos.deps.rules import DEFAULTS

    closure = analyzer.closure([DIMOS_DIR / "cli" / "dimos.py"], DEFAULTS)
    requirements, findings, _warnings = analyzer.requirements(closure)
    assert not findings
    assert not requirements.extras, (
        "the CLI entry point eagerly needs extras; import them inside the command: "
        f"{dict(requirements.extras)}"
    )
    assert not requirements.backends
