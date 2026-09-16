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

"""Prepare real managed environments from this checkout with uv.

Installs requirements of the repository lock into a temporary environment
store, so it needs uv, the lock's wheels (network or a warm uv cache) and a
C++ toolchain for the native extension. Runs on the self-hosted CI job and by
hand: ``uv run pytest -m self_hosted dimos/deps/test_managed_integration.py``.
Set ``DIMOS_TEST_ENVS_DIR`` to a directory on the same file system as the uv
cache so installations hardlink instead of copying.
"""

from collections.abc import Iterator
import os
from pathlib import Path
import shutil
import subprocess
import sys
import textwrap

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps import managed
from dimos.deps.catalog import Plan, default_catalog
from dimos.deps.environment import EnvironmentReport
from dimos.deps.managed import (
    Stamp,
    Validation,
    detect_source,
    ensure_environment,
    environment_key,
)
from dimos.deps.probe import ProbeRequest, run_probe
from dimos.deps.profiles import resolve_profile
from dimos.deps.selectors import collect_selector_inputs
from dimos.deps.uv import UvRunner

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(shutil.which("uv") is None, reason="uv is not installed"),
]
PREPARE_TIMEOUT_S = 1800
PREPARER = """
import sys
from pathlib import Path
from dimos.deps import managed
from dimos.deps.catalog import Plan
from dimos.deps.managed import detect_source, ensure_environment, environment_key
from dimos.deps.profiles import resolve_profile

managed.ENVS_DIR = Path(sys.argv[1])
extras = frozenset(sys.argv[2].split(",")) - {""}
profile, _overridden = resolve_profile(sys.argv[3] or None)
plan = Plan(extras=extras)
key = environment_key(plan, profile, detect_source())
_stamp, lease = ensure_environment(
    key, plan, profile, overridden_profile=False, blueprints=("coordinator-mock",),
    global_config={}, offline=False, echo=lambda message: print(message, flush=True),
)
lease.release()
print("done", key.name)
"""


def _profile_name() -> str | None:
    return "linux-x86_64-cpu" if sys.platform == "linux" else None


@pytest.fixture
def envs_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Path]:
    configured = os.environ.get("DIMOS_TEST_ENVS_DIR")
    root = (Path(configured) / tmp_path.name if configured else tmp_path) / "envs"
    monkeypatch.setattr(managed, "ENVS_DIR", root)
    yield root
    shutil.rmtree(root, ignore_errors=True)


class CountingUv(UvRunner):
    def __init__(self) -> None:
        super().__init__()
        self.calls = 0

    def run(self, args, *, env, cwd=None):  # type: ignore[no-untyped-def]
        self.calls += 1
        return super().run(args, env=env, cwd=cwd)


def test_prepare_core_only_environment_from_the_checkout(envs_dir: Path) -> None:
    profile, overridden = resolve_profile(_profile_name())
    source = detect_source()
    assert source.mode == "checkout" and source.root == DIMOS_PROJECT_ROOT.resolve()
    plan = Plan()  # core only: coordinator-mock needs nothing else
    key = environment_key(plan, profile, source)
    uv = CountingUv()
    probes: list[ProbeRequest] = []

    def counting_probe(python: Path, request: ProbeRequest) -> EnvironmentReport:
        probes.append(request)
        return run_probe(python, request)

    stamp, lease = ensure_environment(
        key,
        plan,
        profile,
        overridden_profile=overridden,
        blueprints=("coordinator-mock",),
        global_config={},
        offline=False,
        uv=uv,
        probe=counting_probe,
        echo=lambda message: None,
    )
    with lease:
        assert uv.calls == 1 and len(probes) == 2
        assert key.dimos_executable.is_file() and key.stamp_path.is_file()
        assert Stamp.load(key.stamp_path) == stamp
        (record_path,) = (key.path / "validations").glob("*.json")
        record = Validation.load(record_path)
        assert record is not None and record.material["blueprints"] == ["coordinator-mock"]
        assert record.report["blueprints"] == {
            "coordinator-mock": {"status": "satisfied", "detail": None}
        }
        assert not record.report["missing"]

    again, lease = ensure_environment(
        key,
        plan,
        profile,
        overridden_profile=overridden,
        blueprints=("coordinator-mock",),
        global_config={},
        offline=True,
        prepare_missing=False,
        uv=uv,
        probe=counting_probe,
        echo=lambda message: None,
    )
    with lease:
        assert again == stamp and uv.calls == 1 and len(probes) == 2

    # The same packages serve a blueprint with a new prerequisite: it validates again
    # instead of trusting the first stamp, and reports what it cannot check.
    native = default_catalog().plan_for(["local-planner-native"], accelerator=profile.accelerator)
    assert environment_key(native, profile, source) == key and native.native == {"local_planner"}
    _stamp, lease = ensure_environment(
        key,
        native,
        profile,
        overridden_profile=overridden,
        blueprints=("local-planner-native",),
        global_config={},
        offline=True,
        prepare_missing=False,
        uv=uv,
        probe=counting_probe,
        echo=lambda message: None,
    )
    with lease:
        assert uv.calls == 1 and len(probes) == 3
        records = sorted((key.path / "validations").glob("*.json"))
        assert len(records) == 2
        (fresh,) = [
            record
            for record in map(Validation.load, records)
            if record is not None and record.material["blueprints"] == ["local-planner-native"]
        ]
        assert fresh.report["native"]["local_planner"]["status"] == "unchecked"

    completed = subprocess.run(
        [str(key.dimos_executable), "--help"], capture_output=True, text=True, timeout=120
    )
    assert completed.returncode == 0, completed.stderr
    assert "Dimensional CLI" in completed.stdout


def test_selected_adapter_is_installed(envs_dir: Path) -> None:
    """A hardware override selecting the xArm adapter plans and installs its SDK."""
    profile, overridden = resolve_profile(_profile_name())
    catalog = default_catalog()
    inputs = collect_selector_inputs(
        ["--controlcoordinator.hardware", '[{"adapter_type": "xarm"}]'],
        {},
        {},
        catalog.selector_fields(),
    )
    plan = catalog.plan_for(["coordinator-mock"], accelerator=profile.accelerator, inputs=inputs)
    assert plan.complete and "control" in plan.extras
    key = environment_key(plan, profile, detect_source())
    _stamp, lease = ensure_environment(
        key,
        plan,
        profile,
        overridden_profile=overridden,
        blueprints=("coordinator-mock",),
        global_config={},
        offline=False,
        echo=lambda message: None,
    )
    with lease:
        completed = subprocess.run(
            [str(key.python_executable), "-c", "import xarm"], capture_output=True, text=True
        )
        assert completed.returncode == 0, completed.stderr


def _prepare_in_process(envs_dir: Path, extras: str) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            sys.executable,
            "-c",
            textwrap.dedent(PREPARER),
            str(envs_dir),
            extras,
            _profile_name() or "",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        cwd=DIMOS_PROJECT_ROOT,
    )


@pytest.fixture
def preparers() -> Iterator[list[subprocess.Popen[str]]]:
    started: list[subprocess.Popen[str]] = []
    yield started
    for process in started:
        if process.poll() is None:
            process.kill()
        process.wait()


def test_preparations_of_different_environments_run_in_parallel(
    envs_dir: Path, preparers: list[subprocess.Popen[str]]
) -> None:
    core = _prepare_in_process(envs_dir, "")
    drone = _prepare_in_process(envs_dir, "drone")
    preparers.extend([core, drone])
    outputs = [process.communicate(timeout=PREPARE_TIMEOUT_S)[0] for process in (core, drone)]
    assert [process.returncode for process in (core, drone)] == [0, 0], outputs
    assert all("waiting" not in output for output in outputs), outputs
    assert sum(output.count("Preparing environment") for output in outputs) == 2
    assert len(list(envs_dir.glob("*/dimos-env.json"))) == 2


def test_concurrent_preparations_of_one_environment_prepare_once(
    envs_dir: Path, preparers: list[subprocess.Popen[str]]
) -> None:
    first = _prepare_in_process(envs_dir, "")
    second = _prepare_in_process(envs_dir, "")
    preparers.extend([first, second])
    outputs = [process.communicate(timeout=PREPARE_TIMEOUT_S)[0] for process in (first, second)]
    assert [process.returncode for process in (first, second)] == [0, 0], outputs
    assert sum(output.count("Preparing environment") for output in outputs) == 1, outputs
    assert sum(output.count("Reusing prepared environment") for output in outputs) == 1, outputs
    assert len(list(envs_dir.glob("*/dimos-env.json"))) == 1
