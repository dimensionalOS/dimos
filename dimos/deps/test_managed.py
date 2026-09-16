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

from collections.abc import Callable, Iterator, Sequence
from pathlib import Path
import sys
import threading
import time
from typing import Any

from packaging.version import Version
import pytest

from dimos.deps import managed
from dimos.deps.catalog import Plan
from dimos.deps.environment import EnvironmentReport, Outcome, RequirementIssue
from dimos.deps.lease import EnvironmentBusyError, EnvironmentLease, prepare_lock, use_lock
from dimos.deps.managed import (
    PreparationError,
    Source,
    Stamp,
    Validation,
    ensure_environment,
    environment_key,
    environment_state,
    list_environments,
    managed_environment_of,
    managed_path,
    prepare_environment,
    prune_environments,
    remove_environment,
    use_lease,
)
from dimos.deps.profiles import PROFILES, HostFacts

PYTHON = f"{sys.version_info.major}.{sys.version_info.minor}"
WAIT_S = 10.0
FACTS = HostFacts(system="linux", machine="x86_64", hardware="generic", cuda_major=0)


class FakeUv:
    """Records uv invocations and materializes what a real sync would create."""

    def __init__(
        self, returncodes: Sequence[int] = (), lock_content: str = "version = 1\n"
    ) -> None:
        self.command = ["uv"]
        self.calls: list[tuple[list[str], dict[str, str], Path | None]] = []
        self._returncodes = list(returncodes)
        self._lock_content = lock_content

    def run(self, args: Sequence[str], *, env: dict[str, str], cwd: Path | None = None) -> int:
        self.calls.append((list(args), dict(env), cwd))
        code = self._returncodes.pop(0) if self._returncodes else 0
        if code == 0 and args[0] == "sync":
            venv = Path(env["UV_PROJECT_ENVIRONMENT"])
            (venv / "bin").mkdir(parents=True, exist_ok=True)
            (venv / "bin" / "python").write_text("#!/bin/sh\n")
            (venv / "bin" / "dimos").write_text("#!/bin/sh\n")
        if code == 0 and args[0] == "lock":
            project = Path(args[args.index("--project") + 1])
            (project / "uv.lock").write_text(self._lock_content)
        return code


class FakeProbe:
    def __init__(self, reports: Sequence[EnvironmentReport] = ()) -> None:
        self.reports = list(reports)
        self.requests: list[Any] = []

    def __call__(self, python: Path, request: Any, **kwargs: Any) -> EnvironmentReport:
        self.requests.append((python, request))
        if self.reports:
            return self.reports.pop(0)
        return ok_report()


def ok_report(**changes: Any) -> EnvironmentReport:
    report = EnvironmentReport(python=PYTHON, prefix="/env", dimos_version="0.0.14")
    for key, value in changes.items():
        setattr(report, key, value)
    return report


@pytest.fixture
def checkout(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    root = tmp_path / "checkout"
    root.mkdir()
    (root / "pyproject.toml").write_text('[project]\nname = "dimos"\n')
    (root / "uv.lock").write_text("version = 1\n")
    monkeypatch.setattr(managed, "ENVS_DIR", tmp_path / "envs")
    monkeypatch.setattr(managed, "uv_version", lambda command: Version("0.11.15"))
    return root


@pytest.fixture
def constraints(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "constraints.txt"
    path.write_text("numpy==2.3.5\ntorch==2.7.1+cu128 ; sys_platform == 'linux'\n")
    monkeypatch.setattr(managed, "SHIPPED_CONSTRAINTS_FILE", path)
    return path


@pytest.fixture
def leases() -> Iterator[list[EnvironmentLease]]:
    held: list[EnvironmentLease] = []
    yield held
    for lease in held:
        lease.release()


def source_for(root: Path) -> Source:
    return Source("checkout", root, "0.0.14")


PLAN = Plan(
    extras=frozenset({"web", "unitree"}), backends={"onnxruntime": "cpu"}, tools=frozenset()
)
CPU = PROFILES["linux-x86_64-cpu"]


def prepare(
    root: Path, uv: FakeUv, probe: FakeProbe, plan: Plan = PLAN, profile: Any = CPU, **kwargs: Any
) -> Stamp:
    key = environment_key(plan, profile, source_for(root))
    return prepare_environment(
        key,
        plan,
        profile,
        offline=kwargs.pop("offline", False),
        uv=uv,  # type: ignore[arg-type]
        probe=probe,
        echo=lambda message: None,
    )


def ensure(
    root: Path, uv: FakeUv, probe: FakeProbe, plan: Plan = PLAN, **kwargs: Any
) -> tuple[Stamp, EnvironmentLease]:
    key = environment_key(plan, CPU, source_for(root))
    return ensure_environment(
        key,
        plan,
        CPU,
        overridden_profile=kwargs.pop("overridden", False),
        blueprints=kwargs.pop("blueprints", ("unitree-go2",)),
        global_config=kwargs.pop("global_config", {"simulation": ""}),
        offline=kwargs.pop("offline", False),
        uv=uv,  # type: ignore[arg-type]
        probe=probe,
        echo=kwargs.pop("echo", lambda message: None),
        facts=FACTS,
        **kwargs,
    )


def validations(key: Any) -> list[Validation]:
    records = [Validation.load(path) for path in sorted((key.path / "validations").glob("*.json"))]
    assert all(record is not None for record in records)
    return [record for record in records if record is not None]


def _poll(condition: Callable[[], bool]) -> None:
    deadline = time.monotonic() + WAIT_S
    while not condition():
        assert time.monotonic() < deadline, "condition not met in time"
        time.sleep(0.01)


def test_identity_is_stable_and_sensitive(
    checkout: Path, tmp_path: Path, constraints: Path
) -> None:
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert (
        key.name
        == environment_key(
            Plan(extras=frozenset({"unitree", "web"})), CPU, source_for(checkout)
        ).name
    )
    assert key.name.startswith(f"linux-x86_64-cpu-py{PYTHON}-")
    assert key.path == tmp_path / "envs" / key.name and key.venv == key.path / ".venv"
    assert key.lock_hash is not None and key.lock_hash.startswith("sha256:")
    assert (
        key.name != environment_key(Plan(extras=frozenset({"web"})), CPU, source_for(checkout)).name
    )
    assert (
        key.name != environment_key(PLAN, PROFILES["linux-x86_64-cuda"], source_for(checkout)).name
    )
    (checkout / "uv.lock").write_text("version = 2\n")
    assert key.name != environment_key(PLAN, CPU, source_for(checkout)).name
    wheel = environment_key(PLAN, CPU, Source("wheel", None, "0.0.14"))
    assert wheel.lock_hash is not None and wheel.name != key.name
    assert wheel.name == environment_key(PLAN, CPU, Source("wheel", None, "0.0.14", "dist/")).name
    constraints.write_text("numpy==2.4.0\n")
    assert wheel.name != environment_key(PLAN, CPU, Source("wheel", None, "0.0.14")).name
    constraints.unlink()
    assert environment_key(PLAN, CPU, Source("wheel", None, "0.0.14")).lock_hash is None


def test_prepare_checkout_runs_the_exact_uv_command(checkout: Path) -> None:
    uv, probe = FakeUv(), FakeProbe()
    stamp = prepare(checkout, uv, probe)
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert len(uv.calls) == 1
    args, env, cwd = uv.calls[0]
    assert args == [
        "sync",
        "--frozen",
        "--no-default-groups",
        "--project",
        str(checkout),
        "--python",
        sys.executable,
        "--extra",
        "unitree",
        "--extra",
        "web",
    ]
    assert env["UV_PROJECT_ENVIRONMENT"] == str(key.venv) and "VIRTUAL_ENV" not in env
    assert "UV_OFFLINE" not in env and cwd == checkout
    assert key.stamp_path.is_file() and Stamp.load(key.stamp_path) == stamp
    assert stamp.extras == ["unitree", "web"] and stamp.profile == "linux-x86_64-cpu"
    assert stamp.lock_hash == key.lock_hash
    assert stamp.uv_version == "0.11.15" and stamp.fixups == []
    (request,) = [request for _python, request in probe.requests]
    assert request.checks == ("packages", "providers") and request.blueprints == ()


def test_package_failure_after_installation_writes_no_stamp(checkout: Path) -> None:
    lacking = ok_report(missing=[RequirementIssue("fastapi>=0.115", "web", None, "missing")])
    uv = FakeUv()
    with pytest.raises(PreparationError, match="lacks packages after installation"):
        prepare(checkout, uv, FakeProbe([lacking]))
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert key.path.is_dir() and not key.stamp_path.exists()
    (key.path / "leftover").write_text("x")
    prepare(checkout, uv, FakeProbe())
    assert key.stamp_path.is_file() and not (key.path / "leftover").exists()
    assert len(uv.calls) == 2


def test_ensure_validates_the_run_once_per_shape(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    uv, probe = FakeUv(), FakeProbe()
    messages: list[str] = []
    _stamp, lease = ensure(checkout, uv, probe, echo=messages.append)
    leases.append(lease)
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert f"Validating {key.name} for unitree-go2" in messages
    request = probe.requests[-1][1]
    assert "blueprints" in request.checks and request.blueprints == ("unitree-go2",)
    assert request.global_config == {"simulation": ""}
    (record,) = validations(key)
    assert record.material["blueprints"] == ["unitree-go2"]
    assert record.material["config"]["simulation"] == "" and record.material["overridden"] is False
    assert record.material["host"] == {
        "system": "linux",
        "machine": "x86_64",
        "hardware": "generic",
        "cuda_major": 0,
    }
    assert record.report["blueprints"] == {}
    probed = len(probe.requests)

    _stamp, lease = ensure(checkout, uv, probe)
    leases.append(lease)
    assert len(probe.requests) == probed and len(uv.calls) == 1

    for changed in (
        {"blueprints": ("unitree-go2", "unitree-go2-detection")},
        {"overridden": True},
        {"global_config": {"simulation": "mujoco"}},
    ):
        _stamp, lease = ensure(checkout, uv, probe, **changed)
        leases.append(lease)
    assert len(probe.requests) == probed + 3 and len(validations(key)) == 4

    _stamp, lease = ensure(checkout, uv, probe, global_config={"simulation": "", "log_level": "x"})
    leases.append(lease)
    assert len(probe.requests) == probed + 3


def test_validation_failure_keeps_the_stamp_and_writes_no_record(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    failing = ok_report(blueprints={"unitree-go2": Outcome("missing", "ImportError: no x")})
    uv = FakeUv()
    with pytest.raises(PreparationError, match="does not satisfy this run"):
        ensure(checkout, uv, FakeProbe([ok_report(), failing]))
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert key.stamp_path.is_file() and validations(key) == []
    assert environment_state(key.path) == "idle"
    probe = FakeProbe()
    _stamp, lease = ensure(checkout, uv, probe)
    leases.append(lease)
    assert len(uv.calls) == 1 and len(probe.requests) == 1 and len(validations(key)) == 1


def test_reuse_makes_no_uv_calls(checkout: Path, leases: list[EnvironmentLease]) -> None:
    uv, probe = FakeUv(), FakeProbe()
    stamp = prepare(checkout, uv, probe)
    key = environment_key(PLAN, CPU, source_for(checkout))
    messages: list[str] = []
    again, lease = ensure(
        checkout, uv, probe, offline=True, prepare_missing=False, echo=messages.append
    )
    leases.append(lease)
    assert again == stamp and len(uv.calls) == 1
    assert messages[0] == f"Reusing prepared environment {key.name} at {key.path}"


def test_offline_run_refuses_to_prepare(checkout: Path) -> None:
    with pytest.raises(PreparationError, match="--offline forbids preparing"):
        ensure(checkout, FakeUv(), FakeProbe(), offline=True, prepare_missing=False)


def test_offline_preparation_installs_from_the_cache(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    uv = FakeUv()
    _stamp, lease = ensure(checkout, uv, FakeProbe(), offline=True)
    leases.append(lease)
    assert "--offline" in uv.calls[0][0] and uv.calls[0][1]["UV_OFFLINE"] == "1"


def test_ensure_holds_a_shared_lease_on_the_environment(checkout: Path) -> None:
    _stamp, lease = ensure(checkout, FakeUv(), FakeProbe())
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert lease.path == use_lock(key.path) and environment_state(key.path) == "in use"
    with pytest.raises(EnvironmentBusyError, match="in use by another dimos process"):
        remove_environment(key.name)
    lease.release()
    assert environment_state(key.path) == "idle"
    assert remove_environment(key.name) == key.path and not key.path.exists()
    assert use_lock(key.path).is_file() and prepare_lock(key.path).is_file()


def test_ensure_waits_for_a_concurrent_preparation(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    key = environment_key(PLAN, CPU, source_for(checkout))
    holder = EnvironmentLease.acquire(
        prepare_lock(key.path), shared=False, blocking=False, busy="held by the test"
    )
    leases.append(holder)
    messages: list[str] = []
    results: list[tuple[Stamp, EnvironmentLease]] = []
    uv = FakeUv()

    def ensure_in_thread() -> None:
        results.append(ensure(checkout, uv, FakeProbe(), echo=messages.append))

    worker = threading.Thread(target=ensure_in_thread)
    worker.start()
    _poll(lambda: bool(messages))
    assert messages == [
        f"{key.name} is being prepared or removed by another dimos process; waiting..."
    ]
    assert not uv.calls and environment_state(key.path) == "preparing"
    holder.release()
    worker.join(timeout=WAIT_S)
    assert results and results[0][0] == Stamp.load(key.stamp_path)
    leases.append(results[0][1])
    assert messages[1] == f"Preparing environment {key.name} at {key.path}"


def test_prepare_refuses_an_incomplete_environment_in_use(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    key = environment_key(PLAN, CPU, source_for(checkout))
    key.path.mkdir(parents=True)
    leases.append(use_lease(key.path))
    with pytest.raises(PreparationError, match="incomplete but in use by another dimos process"):
        ensure(checkout, FakeUv(), FakeProbe())


def test_deno_is_ensured_for_a_reused_environment(
    checkout: Path, leases: list[EnvironmentLease], monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[int] = []
    monkeypatch.setattr("dimos.utils.deno.ensure_deno", lambda: calls.append(1))
    plan = Plan(tools=frozenset({"deno"}))
    uv = FakeUv()
    for _ in range(2):
        _stamp, lease = ensure(checkout, uv, FakeProbe(), plan=plan)
        leases.append(lease)
    assert len(calls) == 2 and len(uv.calls) == 1


def test_uv_failure_is_reported(checkout: Path) -> None:
    with pytest.raises(PreparationError, match="C\\+\\+ toolchain"):
        prepare(checkout, FakeUv([1]), FakeProbe())


def test_cuda_relayers_onnxruntime_gpu(checkout: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(managed, "_installed_version", lambda python, dist: "1.24.1")
    conflicted = ok_report(conflicts=[("onnxruntime", "onnxruntime-gpu")])
    uv = FakeUv()
    stamp = prepare(
        checkout, uv, FakeProbe([conflicted, ok_report()]), profile=PROFILES["linux-x86_64-cuda"]
    )
    assert stamp.fixups == ["onnxruntime-gpu-relayer"]
    key = environment_key(PLAN, PROFILES["linux-x86_64-cuda"], source_for(checkout))
    assert uv.calls[1][0] == [
        "pip",
        "install",
        "--python",
        str(key.python_executable),
        "--no-deps",
        "--reinstall",
        "onnxruntime-gpu==1.24.1",
    ]


def test_backend_failure_is_a_warning_for_an_overridden_profile(
    checkout: Path, leases: list[EnvironmentLease]
) -> None:
    failing = ok_report(backends={"onnxruntime": Outcome("missing", "no CUDA")})
    uv = FakeUv()
    with pytest.raises(PreparationError, match="Backend onnxruntime: missing \\(no CUDA\\)"):
        ensure(checkout, uv, FakeProbe([ok_report(), failing]))
    messages: list[str] = []
    _stamp, lease = ensure(
        checkout, uv, FakeProbe([failing]), overridden=True, echo=messages.append
    )
    leases.append(lease)
    assert any(message.startswith("Warning: backend probes failed") for message in messages)
    key = environment_key(PLAN, CPU, source_for(checkout))
    (record,) = validations(key)
    assert record.material["overridden"] is True
    assert record.report["backends"] == {"onnxruntime": {"status": "missing", "detail": "no CUDA"}}


def test_wheel_mode_renders_a_project_and_locks_it(
    checkout: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, constraints: Path
) -> None:
    policy = tmp_path / "policy.toml"
    policy.write_text(
        '[project]\nname = "dimos"\n[tool.uv]\noverride-dependencies = ["pillow>=12"]\n'
        '[[tool.uv.index]]\nname = "pytorch-cu128"\nurl = "https://download.pytorch.org/whl/cu128"\nexplicit = true\n'
    )
    monkeypatch.setattr(managed, "find_project_file", lambda root: policy)
    source = Source("wheel", None, "0.0.14", find_links=str(tmp_path / "dist"))
    key = environment_key(PLAN, CPU, source)
    uv = FakeUv()
    stamp = prepare_environment(
        key,
        PLAN,
        CPU,
        offline=False,
        uv=uv,
        probe=FakeProbe(),
        echo=lambda m: None,  # type: ignore[arg-type]
    )
    project = (key.project / "pyproject.toml").read_text()
    assert 'dependencies = ["dimos[unitree,web]==0.0.14"]' in project
    assert 'override-dependencies = ["pillow>=12"]' in project and "pytorch-cu128" in project
    assert (
        '"numpy==2.3.5",' in project and "torch==2.7.1+cu128 ; sys_platform == 'linux'" in project
    )
    assert [call[0][:3] for call in uv.calls] == [
        ["lock", "--project", str(key.project)],
        ["sync", "--frozen", "--project"],
    ]
    assert uv.calls[0][1]["UV_FIND_LINKS"] == str((tmp_path / "dist").resolve())
    assert stamp.lock_hash == key.lock_hash and stamp.lock_hash.startswith("sha256:")
    assert stamp.source["find_links"] == str(tmp_path / "dist")


def test_wheel_mode_refuses_packages_outside_the_tested_set(
    checkout: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, constraints: Path
) -> None:
    policy = tmp_path / "policy.toml"
    policy.write_text('[project]\nname = "dimos"\n')
    monkeypatch.setattr(managed, "find_project_file", lambda root: policy)
    key = environment_key(PLAN, CPU, Source("wheel", None, "0.0.14"))
    resolved = 'version = 1\n[[package]]\nname = "numpy"\nversion = "2.3.5"\n[[package]]\nname = "surprise"\nversion = "1"\n'
    with pytest.raises(PreparationError, match="outside the tested set: surprise"):
        prepare_environment(
            key,
            PLAN,
            CPU,
            offline=False,
            uv=FakeUv(lock_content=resolved),  # type: ignore[arg-type]
            probe=FakeProbe(),
            echo=lambda m: None,
        )
    assert not key.stamp_path.exists()
    constraints.unlink()
    with pytest.raises(PreparationError, match="ships no tested lock"):
        prepare_environment(
            key,
            PLAN,
            CPU,
            offline=False,
            uv=FakeUv(),  # type: ignore[arg-type]
            probe=FakeProbe(),
            echo=lambda m: None,
        )


@pytest.mark.parametrize(
    "name",
    [
        "../outside",
        "/tmp/outside",
        ".locks",
        "a/b",
        "linux-x86_64-cpu-py3.12-zzzzzzzz",
        "LINUX-x86_64-cpu-py3.12-deadbeef",
    ],
)
def test_managed_path_rejects_names_outside_the_store(checkout: Path, name: str) -> None:
    with pytest.raises(ValueError, match="not a managed environment name"):
        managed_path(name)


def test_managed_path_requires_an_existing_environment(checkout: Path, tmp_path: Path) -> None:
    name = "linux-x86_64-cpu-py3.12-deadbeef"
    with pytest.raises(FileNotFoundError, match="no managed environment named"):
        managed_path(name)
    (tmp_path / "envs" / name).mkdir(parents=True)
    assert managed_path(name) == tmp_path / "envs" / name
    outside = tmp_path / "outside"
    outside.mkdir()
    with pytest.raises(ValueError):
        remove_environment(str(outside))
    assert outside.is_dir()


def test_remove_and_prune(checkout: Path, tmp_path: Path, leases: list[EnvironmentLease]) -> None:
    stamp = prepare(checkout, FakeUv(), FakeProbe())
    key = environment_key(PLAN, CPU, source_for(checkout))
    unfinished = tmp_path / "envs" / "linux-x86_64-cpu-py3.12-deadbeef"
    unfinished.mkdir(parents=True)
    lease = use_lease(key.path)
    leases.append(lease)
    with pytest.raises(
        EnvironmentBusyError, match=f"{key.name} is in use by another dimos process"
    ):
        remove_environment(stamp.name)
    assert prune_environments() == ([unfinished], [])
    assert prune_environments(all_unused=True) == (
        [],
        [f"{key.name} is in use by another dimos process"],
    )
    lease.release()
    assert prune_environments() == ([], [])
    assert prune_environments(all_unused=True) == ([key.path], [])
    with pytest.raises(FileNotFoundError):
        remove_environment(stamp.name)


def test_prune_skips_an_environment_being_prepared(
    checkout: Path, tmp_path: Path, leases: list[EnvironmentLease]
) -> None:
    unfinished = tmp_path / "envs" / "linux-x86_64-cpu-py3.12-deadbeef"
    unfinished.mkdir(parents=True)
    leases.append(
        EnvironmentLease.acquire(
            prepare_lock(unfinished), shared=False, blocking=False, busy="held by the test"
        )
    )
    assert environment_state(unfinished) == "preparing"
    assert prune_environments() == (
        [],
        [f"{unfinished.name} is being prepared or removed by another dimos process"],
    )
    assert unfinished.is_dir()


def test_prune_removes_environments_whose_checkout_vanished(checkout: Path, tmp_path: Path) -> None:
    stamp = prepare(checkout, FakeUv(), FakeProbe())
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert stamp.name == key.name
    for path in (checkout / "uv.lock", checkout / "pyproject.toml"):
        path.unlink()
    checkout.rmdir()
    assert prune_environments() == ([key.path], [])


def test_store_layout_helpers(checkout: Path, tmp_path: Path) -> None:
    prepare(checkout, FakeUv(), FakeProbe())
    key = environment_key(PLAN, CPU, source_for(checkout))
    assert (tmp_path / "envs" / ".locks").is_dir()
    assert [path.name for path, _stamp in list_environments()] == [key.name]
    assert managed_environment_of(key.venv) == key.path
    assert managed_environment_of(key.path) is None
    assert managed_environment_of(tmp_path / "elsewhere" / ".venv") is None
