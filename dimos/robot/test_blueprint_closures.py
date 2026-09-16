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

"""Importing a blueprint loads only files its static closure predicted.

The dependency catalog is built from the static import closure of each
blueprint (``dimos/deps``). This test imports every cataloged blueprint the
running interpreter can satisfy, in a subprocess per configuration scenario,
and checks that each ``dimos`` module the import loaded is in that closure.
Together with the catalog audit (every file's imports are declared) that
shows the plan covers what a blueprint imports at run time. A blueprint whose
plan this interpreter satisfies but that fails to import means the plan
misses a requirement.

Modules a previous blueprint already loaded are not attributed again, so
each module is checked against the first blueprint that loaded it.
"""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.analysis import Analyzer, target_file
from dimos.deps.catalog import default_catalog
from dimos.deps.environment import check_environment, installed_versions, requires_of
from dimos.deps.lock import LockIndex
from dimos.deps.rules import DEFAULTS, SCENARIOS
from dimos.robot.all_blueprints import all_blueprints, all_modules

CONFIG_VARIABLES = ("SIMULATION", "ROBOT_IP", "VIEWER", "REPLAY")
IMPORTER = """
import json, sys
from dimos.core.global_config import global_config
request = json.load(sys.stdin)
sys.stdout = sys.stderr  # blueprint modules log at import; keep stdout for nothing
global_config.update(**request["overrides"])
from dimos.robot.get_all_blueprints import get_by_name
loaded = {}
for name in request["names"]:
    before = set(sys.modules)
    try:
        get_by_name(name)
    except Exception as error:
        module = sys.modules.get(getattr(error, "name", "") or "")
        where = getattr(module, "__file__", None) or getattr(module, "__path__", None)
        loaded[name] = {"error": f"{type(error).__name__}: {error} (found at {where}; sys.path={sys.path})"}
        continue
    files = []
    for module_name in set(sys.modules) - before:
        module = sys.modules.get(module_name)
        file = getattr(module, "__file__", None)
        if module_name.startswith("dimos.") and file:
            files.append(file)
    loaded[name] = {"files": sorted(files)}
with open(request["result"], "w") as handle:
    json.dump(loaded, handle)
"""


def _scenarios() -> list[tuple[str, dict[str, object]]]:
    return [("defaults", {}), *((s.name, dict(s.overrides)) for s in SCENARIOS.values())]


@pytest.fixture(scope="module")
def satisfiable() -> dict[str, list[str]]:
    """Scenario -> cataloged names whose packages, host modules and native modules are present."""
    catalog = default_catalog()
    versions = installed_versions()
    requires = requires_of("dimos") or []
    accelerator = "cpu"
    result: dict[str, list[str]] = {}
    for scenario, overrides in _scenarios():
        names = []
        for name in sorted(catalog.names):
            plan = catalog.plan_for([name], {**DEFAULTS, **overrides}, accelerator=accelerator)
            report = check_environment(
                plan, checks=("packages", "system", "native"), requires=requires, versions=versions
            )
            if report.ok:
                names.append(name)
        result[scenario] = names
    return result


@pytest.mark.parametrize("scenario", [name for name, _overrides in _scenarios()])
def test_blueprint_imports_stay_within_their_static_closure(
    scenario: str, satisfiable: dict[str, list[str]], tmp_path: Path
) -> None:
    overrides = dict(_scenarios())[scenario]
    names = satisfiable[scenario]
    assert names, f"no cataloged blueprint is importable here under {scenario}"
    env = {key: value for key, value in os.environ.items() if key not in CONFIG_VARIABLES}
    result = tmp_path / "loaded.json"
    completed = subprocess.run(
        [sys.executable, "-c", IMPORTER],
        input=json.dumps({"overrides": overrides, "names": names, "result": str(result)}),
        capture_output=True,
        text=True,
        env=env,
        cwd=DIMOS_PROJECT_ROOT,
        timeout=600,
        check=False,
    )
    assert completed.returncode == 0 and result.is_file(), completed.stderr[-4000:]
    loaded = json.loads(result.read_text())
    analyzer = Analyzer(DIMOS_PROJECT_ROOT, LockIndex.load(DIMOS_PROJECT_ROOT))
    config = {**DEFAULTS, **overrides}
    problems: list[str] = []
    for name in names:
        outcome = loaded[name]
        if "error" in outcome:
            problems.append(
                f"{name}: the plan is satisfied but the import failed: {outcome['error']}"
            )
            continue
        target = all_blueprints.get(name) or all_modules[name]
        closure = analyzer.closure([target_file(DIMOS_PROJECT_ROOT, target)], config)
        predicted = {str(path) for path in closure.files}
        unexpected = sorted(
            Path(file).relative_to(DIMOS_PROJECT_ROOT).as_posix()
            for file in outcome["files"]
            if str(Path(file).resolve()) not in predicted
        )
        if unexpected:
            problems.append(f"{name}: imported files outside its static closure: {unexpected}")
    listing = "\n".join(f"  - {problem}" for problem in problems)
    assert not problems, f"under {scenario}:\n{listing}"
