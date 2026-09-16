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

from collections.abc import Iterator
import os
from pathlib import Path
import sys
from typing import Any

import pytest

from dimos.deps import launch, managed
from dimos.deps.catalog import Plan
from dimos.deps.environment import EnvironmentReport, Outcome, RequirementIssue
from dimos.deps.launch import (
    LAUNCHED_ENV,
    Decision,
    LaunchError,
    exec_into,
    hold_current_lease,
    replace_option,
    select_environment,
)
from dimos.deps.lease import LEASE_FD_ENV, EnvironmentLease, is_held, use_lock
from dimos.deps.managed import Source, Stamp
from dimos.deps.planning import RunPlan
from dimos.deps.profiles import PROFILES, UnsupportedProfileError

PLAN = Plan(extras=frozenset({"web"}))
SOURCE = Source("wheel", None, "0.0.14")


def planned(profile: Any = PROFILES["linux-x86_64-cpu"], unsupported: Any = None) -> RunPlan:
    return RunPlan(("unitree-go2",), PLAN, profile, False, unsupported)


def report(satisfied: bool) -> EnvironmentReport:
    result = EnvironmentReport(
        python="3.12", prefix="/venv", dimos_version="0.0.14", checks=["packages"]
    )
    if not satisfied:
        result.missing = [RequirementIssue("fastapi>=0.115", "web", None, "missing")]
    return result


def stamp(name: str = "env") -> Stamp:
    return Stamp(name, "linux-x86_64-cpu", "3.12", "/x", ["web"], {}, {}, None, "0.11", "now")


@pytest.fixture
def no_launched_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(LAUNCHED_ENV, raising=False)


@pytest.fixture
def lease(tmp_path: Path) -> Iterator[EnvironmentLease]:
    held = EnvironmentLease.acquire(
        tmp_path / "env.lock", shared=True, blocking=False, busy="held by the test"
    )
    yield held
    held.release()


def test_current_environment(no_launched_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
    decision = select_environment(
        planned(),
        environment="current",
        offline=False,
        blueprints=("unitree-go2",),
        global_config={},
        checker=lambda plan: report(True),
    )
    assert decision == Decision(None, None, "current environment satisfies the plan")
    with pytest.raises(LaunchError, match="fastapi>=0.115 \\(extra web\\): not installed") as info:
        select_environment(
            planned(),
            environment="current",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(False),
        )
    assert info.value.exit_code == 1
    monkeypatch.setenv(LAUNCHED_ENV, "/envs/x")
    with pytest.raises(LaunchError, match="dimos envs remove"):
        select_environment(
            planned(),
            environment="current",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(False),
        )


def test_current_and_auto_refuse_a_missing_prerequisite(no_launched_env: None) -> None:
    def lacking_host_module(plan: Any) -> EnvironmentReport:
        result = report(True)
        result.system = {
            "rclpy": Outcome("missing", "ModuleNotFoundError: No module named 'rclpy'")
        }
        return result

    for environment in ("current", "auto"):
        with pytest.raises(LaunchError, match="Host module rclpy: missing") as info:
            select_environment(
                planned(),
                environment=environment,
                offline=False,
                blueprints=(),
                global_config={},
                checker=lacking_host_module,
            )
        assert info.value.exit_code == 1


def test_managed_and_auto_use_the_prepared_environment(
    no_launched_env: None, monkeypatch: pytest.MonkeyPatch, lease: EnvironmentLease
) -> None:
    calls: list[dict[str, Any]] = []

    def fake_ensure(
        key: Any, plan: Any, profile: Any, **kwargs: Any
    ) -> tuple[Stamp, EnvironmentLease]:
        calls.append({"key": key, **kwargs})
        return stamp(key.name), lease

    monkeypatch.setattr(launch, "ensure_environment", fake_ensure)
    monkeypatch.setattr(launch, "detect_source", lambda: SOURCE)
    messages: list[str] = []
    decision = select_environment(
        planned(),
        environment="managed",
        offline=True,
        blueprints=("unitree-go2",),
        global_config={"a": 1},
        checker=lambda plan: report(False),
        uv_factory=lambda: object(),  # type: ignore[arg-type]
        echo=messages.append,
    )
    key = calls[0]["key"]
    assert decision.dimos_executable == key.dimos_executable and decision.env_dir == key.path
    assert decision.lease is lease
    assert calls[0]["offline"] is True and calls[0]["prepare_missing"] is False
    assert calls[0]["global_config"] == {"a": 1} and calls[0]["blueprints"] == ("unitree-go2",)
    decision = select_environment(
        planned(),
        environment="auto",
        offline=False,
        blueprints=("unitree-go2",),
        global_config={},
        checker=lambda plan: report(False),
        uv_factory=lambda: object(),  # type: ignore[arg-type]
        echo=messages.append,
    )
    assert decision.dimos_executable == key.dimos_executable
    assert calls[1]["offline"] is False and calls[1]["prepare_missing"] is True
    assert messages == ["Current environment lacks fastapi>=0.115; using a managed environment"]
    assert (
        select_environment(
            planned(),
            environment="auto",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(True),
        ).dimos_executable
        is None
    )


def test_auto_refuses_when_launched_or_unsupported(
    no_launched_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    unsupported = UnsupportedProfileError("orin", "NVIDIA Jetson (orin) has no tested profile")
    with pytest.raises(LaunchError, match="no tested profile") as info:
        select_environment(
            planned(None, unsupported),
            environment="auto",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(False),
        )
    assert (info.value.exit_code == 1 and "--extra web" in str(info.value)) or "dimos[web]" in str(
        info.value
    )
    with pytest.raises(LaunchError, match="no tested profile") as info:
        select_environment(
            planned(None, unsupported),
            environment="managed",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(False),
        )
    assert info.value.exit_code == 2
    monkeypatch.setenv(LAUNCHED_ENV, "/envs/x")
    with pytest.raises(LaunchError, match="managed environment /envs/x lacks"):
        select_environment(
            planned(),
            environment="auto",
            offline=False,
            blueprints=(),
            global_config={},
            checker=lambda plan: report(False),
        )


def test_explicit_path(no_launched_env: None, tmp_path: Path) -> None:
    with pytest.raises(LaunchError, match="not a virtualenv") as info:
        select_environment(
            planned(), environment=str(tmp_path), offline=False, blueprints=(), global_config={}
        )
    assert info.value.exit_code == 2
    (tmp_path / "bin").mkdir()
    (tmp_path / "bin" / "python").write_text("")
    (tmp_path / "bin" / "dimos").write_text("")
    decision = select_environment(
        planned(),
        environment=str(tmp_path),
        offline=False,
        blueprints=(),
        global_config={},
        probe=lambda python, request: report(True),
    )
    assert decision.dimos_executable == tmp_path / "bin" / "dimos"
    with pytest.raises(LaunchError, match="not installed"):
        select_environment(
            planned(),
            environment=str(tmp_path),
            offline=False,
            blueprints=(),
            global_config={},
            probe=lambda python, request: report(False),
        )


def test_replace_option() -> None:
    argv = [
        "--replay",
        "run",
        "unitree-go2",
        "--environment",
        "managed",
        "--daemon",
        "--environment=x",
    ]
    assert replace_option(argv, "--environment", "current") == [
        "--replay",
        "run",
        "unitree-go2",
        "--daemon",
        "--environment",
        "current",
    ]


def test_exec_into(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, lease: EnvironmentLease
) -> None:
    captured: dict[str, Any] = {}

    def fake_execve(path: str, args: list[str], env: dict[str, str]) -> None:
        captured.update(path=path, args=args, env=env)

    monkeypatch.setattr(os, "execve", fake_execve)
    exe = tmp_path / "env" / ".venv" / "bin" / "dimos"
    # The fake execve returns; the real one never does.
    exec_into(
        exe,
        tmp_path / "env",
        ["dimos", "run", "x", "--environment", "managed"],
        offline=True,
        lease=lease,
    )
    assert captured["path"] == str(exe)
    assert captured["args"] == [str(exe), "run", "x", "--environment", "current"]
    env = captured["env"]
    assert env[LAUNCHED_ENV] == str(tmp_path / "env") and env["VIRTUAL_ENV"] == str(
        exe.parent.parent
    )
    assert env["PATH"].startswith(str(exe.parent) + os.pathsep)
    assert env["HF_HUB_OFFLINE"] == "1" and env["UV_OFFLINE"] == "1"
    assert env[LEASE_FD_ENV] == str(lease.fd) and os.get_inheritable(lease.fd)

    def failing_execve(path: str, args: list[str], env: dict[str, str]) -> None:
        raise OSError("nope")

    monkeypatch.setattr(os, "execve", failing_execve)
    with pytest.raises(LaunchError, match="could not start"):
        exec_into(exe, tmp_path / "env", ["dimos"], offline=False)


def test_hold_current_lease(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(managed, "ENVS_DIR", tmp_path / "envs")
    monkeypatch.delenv(LEASE_FD_ENV, raising=False)
    monkeypatch.setattr(sys, "prefix", str(tmp_path / "elsewhere" / ".venv"))
    assert hold_current_lease() is None
    env = tmp_path / "envs" / "linux-x86_64-cpu-py3.12-aaaaaaaa"
    (env / ".venv").mkdir(parents=True)
    monkeypatch.setattr(sys, "prefix", str(env / ".venv"))
    held = hold_current_lease()
    assert held is not None and is_held(use_lock(env))
    exported: dict[str, str] = {}
    held.export(exported)
    monkeypatch.setenv(LEASE_FD_ENV, exported[LEASE_FD_ENV])
    adopted = hold_current_lease()
    assert adopted is not None and adopted.fd == held.fd and LEASE_FD_ENV not in os.environ
    held.release()
    assert not is_held(use_lock(env))


def test_incomplete_plans_never_switch_automatically(no_launched_env: None, tmp_path: Path) -> None:
    reason = "external blueprint 'my.stack' publishes no dependency metadata"
    incomplete = RunPlan(
        ("unitree-go2", "my.stack"),
        Plan(extras=frozenset({"web"}), external=("my.stack",), incomplete=(reason,)),
        PROFILES["linux-x86_64-cpu"],
        False,
        None,
    )
    messages: list[str] = []
    decision = select_environment(
        incomplete,
        environment="auto",
        offline=False,
        blueprints=("unitree-go2",),
        global_config={},
        checker=lambda plan: report(True),
        echo=messages.append,
    )
    assert decision == Decision(None, None, "current environment satisfies the plan")
    assert messages == [
        f"Automatic planning is incomplete; staying in the current environment:\n  - {reason}"
    ]
    with pytest.raises(LaunchError, match="not installed"):
        select_environment(
            incomplete,
            environment="auto",
            offline=False,
            blueprints=("unitree-go2",),
            global_config={},
            checker=lambda plan: report(False),
            echo=messages.append,
        )
    with pytest.raises(LaunchError, match="automatic planning is incomplete") as info:
        select_environment(
            incomplete,
            environment="managed",
            offline=False,
            blueprints=("unitree-go2",),
            global_config={},
            checker=lambda plan: report(True),
        )
    assert info.value.exit_code == 2 and reason in str(info.value)
    (tmp_path / "bin").mkdir()
    (tmp_path / "bin" / "python").write_text("")
    (tmp_path / "bin" / "dimos").write_text("")
    decision = select_environment(
        incomplete,
        environment=str(tmp_path),
        offline=False,
        blueprints=("unitree-go2",),
        global_config={},
        probe=lambda python, request: report(True),
    )
    assert decision.dimos_executable == tmp_path / "bin" / "dimos"
