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

import json
from pathlib import Path
import sys

import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.core.global_config import global_config
from dimos.deps.lease import EnvironmentLease, is_held

runner = CliRunner()


@pytest.fixture(autouse=True)
def _restore_global_config() -> None:
    snapshot = global_config.model_dump()
    yield
    global_config.update(**snapshot)


def test_deps_explains_a_blueprint() -> None:
    result = runner.invoke(main, ["deps", "unitree-go2"])
    assert result.exit_code == 0, result.output
    assert "Blueprints:  unitree-go2" in result.output
    assert "unitree" in result.output and "<- robot.unitree" in result.output
    assert "Checkout:    uv sync --extra" in result.output
    assert "Release:     pip install 'dimos[" in result.output
    assert "sim" not in result.output.split("Extras:")[1].splitlines()[0]


def test_deps_follows_global_flags() -> None:
    result = runner.invoke(main, ["--simulation", "mujoco", "deps", "unitree-go2"])
    assert result.exit_code == 0, result.output
    extras_line = result.output.split("Extras:")[1].splitlines()[0]
    assert "sim" in extras_line


def test_deps_json() -> None:
    result = runner.invoke(main, ["deps", "unitree-go2", "--json"])
    assert result.exit_code == 0, result.output
    payload = json.loads(result.output)
    assert payload["blueprints"] == ["unitree-go2"]
    assert "unitree" in payload["extras"]
    assert payload["reasons"]["unitree"] == ["robot.unitree.connection"]
    assert payload["incomplete"] == []
    assert payload["environment"]["prefix"] == sys.prefix


def test_deps_why_shows_an_import_chain() -> None:
    result = runner.invoke(main, ["deps", "unitree-go2-agentic", "--why", "perception"])
    assert result.exit_code == 0, result.output
    assert "Why perception:" in result.output
    assert "dimos/perception/experimental/" in result.output
    # unitree-go2 stopped needing perception when SemanticSearch left memory.module.
    result = runner.invoke(main, ["deps", "unitree-go2", "--why", "perception"])
    assert result.exit_code == 0, result.output
    assert "unitree-go2: no eager import path needs perception" in result.output


def test_deps_unknown_blueprint() -> None:
    result = runner.invoke(main, ["deps", "no-such-blueprint"])
    assert result.exit_code == 2
    assert "no entry for 'no-such-blueprint'" in result.output


def test_deps_external_name_is_listed_not_planned() -> None:
    result = runner.invoke(main, ["deps", "unitree-go2", "my-stack.teleop"])
    assert result.exit_code == 0, result.output
    assert "External:    my-stack.teleop" in result.output


@pytest.mark.skipif(sys.platform != "linux", reason="asserts a Linux host")
def test_deps_rejects_a_profile_for_another_platform() -> None:
    result = runner.invoke(main, ["deps", "unitree-go2", "--profile", "macos-arm64-cpu"])
    assert result.exit_code == 2
    assert "targets darwin/arm64" in result.output


def test_doctor_rejects_a_non_virtualenv_path(tmp_path) -> None:  # type: ignore[no-untyped-def]
    result = runner.invoke(main, ["doctor", "demo-mcp-stress-test", "--environment", str(tmp_path)])
    assert result.exit_code == 2
    assert "not a virtualenv" in result.output


def test_doctor_runs_the_probe_in_the_current_environment() -> None:
    # coordinator-mock needs core only, so any development environment satisfies it.
    result = runner.invoke(main, ["doctor", "coordinator-mock"])
    assert result.exit_code == 0, result.output
    assert "Requirements: satisfied" in result.output
    assert "Blueprint coordinator-mock: satisfied" in result.output


def _stamp(name: str, root: str | None = None) -> "Stamp":
    from dimos.deps.managed import Stamp

    source = {"mode": "checkout", "root": root or "/repo", "version": "0.0.14", "find_links": None}
    return Stamp(
        name,
        "linux-x86_64-cpu",
        "3.12",
        "/x/bin/python",
        ["web"],
        {},
        source,
        "sha256:1",
        "0.11.15",
        "2026-09-16T10:00:00+00:00",
    )


def test_prepare_reuses_an_already_prepared_environment(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from dimos.cli.commands import deps as deps_module
    from dimos.deps import managed

    monkeypatch.setattr(managed, "ENVS_DIR", tmp_path / "envs")
    monkeypatch.setattr(deps_module.Stamp, "load", classmethod(lambda cls, path: _stamp("env-a")))
    validated: list[tuple[str, ...]] = []
    monkeypatch.setattr(
        managed,
        "validate_run",
        lambda key, plan, profile, **kwargs: validated.append(kwargs["blueprints"]),
    )
    result = runner.invoke(main, ["prepare", "demo-mcp-stress-test"])
    assert result.exit_code == 0, result.output
    assert "Reusing prepared environment" in result.output
    assert validated == [("demo-mcp-stress-test",)]
    locks = list((tmp_path / "envs" / ".locks").iterdir())
    assert len(locks) == 2 and not any(is_held(path) for path in locks)


def test_prepare_prepares_when_no_stamp_exists(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from dimos.cli.commands import deps as deps_module

    calls: list[dict[str, object]] = []

    def fake_ensure(
        key: object, plan: object, profile: object, **kwargs: object
    ) -> tuple["Stamp", EnvironmentLease]:
        calls.append(kwargs)
        lease = EnvironmentLease.acquire(
            tmp_path / "env.lock", shared=True, blocking=False, busy="held by the test"
        )
        return _stamp("env-b"), lease

    monkeypatch.setattr(deps_module, "ensure_environment", fake_ensure)
    result = runner.invoke(
        main, ["--simulation", "mujoco", "prepare", "demo-mcp-stress-test", "--offline"]
    )
    assert result.exit_code == 0, result.output
    assert (
        calls
        and calls[0]["offline"] is True
        and calls[0]["blueprints"] == ("demo-mcp-stress-test",)
    )
    assert "prepare_missing" not in calls[0]
    assert calls[0]["global_config"]["simulation"] == "mujoco"  # type: ignore[index]
    assert "dimos_api_key" not in calls[0]["global_config"]  # type: ignore[operator]
    assert not is_held(tmp_path / "env.lock")


@pytest.mark.skipif(sys.platform != "linux", reason="asserts a checkout on Linux")
def test_prepare_rejects_find_links_in_a_checkout(tmp_path) -> None:  # type: ignore[no-untyped-def]
    result = runner.invoke(main, ["prepare", "demo-mcp-stress-test", "--find-links", str(tmp_path)])
    assert result.exit_code == 2
    assert "--find-links applies to an installed dimos" in result.output


def test_envs_list_remove_and_prune(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:  # type: ignore[no-untyped-def]
    from dimos.deps import managed
    from dimos.deps.managed import use_lease

    envs = tmp_path / "envs"
    monkeypatch.setattr(managed, "ENVS_DIR", envs)
    complete = envs / "linux-x86_64-cpu-py3.12-aaaaaaaa"
    complete.mkdir(parents=True)
    _stamp(complete.name, root=str(tmp_path)).save(complete / "dimos-env.json")
    (envs / "linux-x86_64-cpu-py3.12-bbbbbbbb").mkdir()
    result = runner.invoke(main, ["envs", "list"])
    assert result.exit_code == 0, result.output
    assert "aaaaaaaa: linux-x86_64-cpu, python 3.12, extras [web]" in result.output
    assert "bbbbbbbb: incomplete (idle)" in result.output
    assert runner.invoke(main, ["envs", "remove", "nope"]).exit_code == 2
    result = runner.invoke(main, ["envs", "remove", "../outside"])
    assert result.exit_code == 2 and "not a managed environment name" in result.output
    with use_lease(complete):
        assert ", in use" in runner.invoke(main, ["envs", "list"]).output
        result = runner.invoke(main, ["envs", "remove", complete.name])
        assert result.exit_code == 1 and "in use by another dimos process" in result.output
        result = runner.invoke(main, ["envs", "prune", "--all"])
        assert result.exit_code == 0 and "Removed" in result.output and "bbbbbbbb" in result.output
        assert f"Skipped: {complete.name} is in use by another dimos process" in result.output
    assert complete.is_dir()
    result = runner.invoke(main, ["envs", "remove", complete.name])
    assert result.exit_code == 0 and not complete.exists()
    assert "No managed environments" in runner.invoke(main, ["envs", "list"]).output


@pytest.fixture
def never_start(monkeypatch: pytest.MonkeyPatch) -> None:
    """A run test must never reach the coordinator; that would start a real robot stack."""
    from dimos.cli.commands import lifecycle

    def boom(*args: object, **kwargs: object) -> None:
        raise AssertionError("run reached _start; the environment decision did not stop it")

    monkeypatch.setattr(lifecycle, "_start", boom)


def test_run_current_environment_refuses_when_unsatisfied(
    never_start: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.deps import launch
    from dimos.deps.environment import EnvironmentReport, RequirementIssue

    def unsatisfied(plan: object, **kwargs: object) -> EnvironmentReport:
        report = EnvironmentReport(
            python="3.12", prefix=sys.prefix, dimos_version="0.0.14", checks=["packages"]
        )
        report.missing = [RequirementIssue("fastapi>=0.115", "web", None, "missing")]
        return report

    monkeypatch.setattr(launch, "check_environment", unsatisfied)
    result = runner.invoke(main, ["run", "demo-mcp-stress-test", "--environment", "current"])
    assert result.exit_code == 1, result.output
    assert "fastapi>=0.115 (extra web): not installed" in result.output
    assert "--extra web --inexact" in result.output


def test_run_execs_into_the_selected_environment(
    never_start: None, monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:  # type: ignore[no-untyped-def]
    from dimos.cli.commands import lifecycle
    from dimos.deps.launch import Decision

    exe = tmp_path / ".venv" / "bin" / "dimos"
    monkeypatch.setattr(
        lifecycle, "select_environment", lambda planned, **kwargs: Decision(exe, tmp_path, "test")
    )
    executed: list[tuple[object, ...]] = []

    def fake_exec(
        executable: object, env_dir: object, argv: object, *, offline: bool, lease: object
    ) -> None:
        executed.append((executable, env_dir, offline, lease))
        raise SystemExit(0)

    monkeypatch.setattr(lifecycle, "exec_into", fake_exec)
    result = runner.invoke(
        main, ["run", "demo-mcp-stress-test", "--environment", "managed", "--offline"]
    )
    assert result.exit_code == 0, result.output
    assert executed == [(exe, tmp_path, True, None)]


def test_run_rejects_an_unknown_profile(never_start: None) -> None:
    result = runner.invoke(main, ["run", "demo-mcp-stress-test", "--profile", "jetson"])
    assert result.exit_code == 2
    assert "unknown profile 'jetson'" in result.output


def test_run_help_does_not_select_an_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.cli.commands import lifecycle

    def boom(*args: object, **kwargs: object) -> None:
        raise AssertionError("environment selection must not run for --help")

    monkeypatch.setattr(lifecycle, "select_environment", boom)
    result = runner.invoke(main, ["run", "demo-mcp-stress-test", "--help"])
    assert result.exit_code == 0, result.output
    assert "Blueprint configuration options" in result.output


def test_status_prints_the_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.cli.commands import lifecycle
    from dimos.core.run_registry import RunEntry

    entry = RunEntry(
        "run-1", 1, "demo", "2026-09-16T10:00:00+00:00", "/logs", environment="/envs/x/.venv"
    )
    monkeypatch.setattr(lifecycle, "get_most_recent", lambda alive_only=True: entry)
    result = runner.invoke(main, ["status"])
    assert result.exit_code == 0 and "Env:       /envs/x/.venv" in result.output


def test_run_entry_loads_files_without_environment(tmp_path) -> None:  # type: ignore[no-untyped-def]
    from dimos.core.run_registry import RunEntry

    path = tmp_path / "old.json"
    path.write_text(
        json.dumps({"run_id": "old", "pid": 1, "blueprint": "b", "started_at": "t", "log_dir": "l"})
    )
    assert RunEntry.load(path).environment == ""


def test_deps_reads_selections_from_the_request(tmp_path: Path) -> None:
    result = runner.invoke(
        main,
        ["deps", "coordinator-mock", "--controlcoordinator.hardware", '[{"adapter_type": "xarm"}]'],
    )
    assert result.exit_code == 0, result.output
    assert "Planning:    complete" in result.output
    assert "control      <- hardware.manipulators.xarm.adapter" in result.output
    result = runner.invoke(
        main,
        [
            "deps",
            "coordinator-mock",
            "--controlcoordinator.hardware",
            '[{"adapter_type": "xarm7"}]',
        ],
    )
    assert result.exit_code == 0, result.output
    assert "Planning:    incomplete" in result.output
    assert "selects a adapter the planner does not know: 'xarm7'" in result.output
    config = tmp_path / "config"
    config.write_text(json.dumps({"controlcoordinator": {"tasks": [{"type": "g1_groot_wbc"}]}}))
    result = runner.invoke(main, ["deps", "coordinator-mock", "--config", str(config)])
    assert result.exit_code == 0, result.output
    assert "Backend:     onnxruntime" in result.output


def test_prepare_refuses_an_incomplete_plan() -> None:
    result = runner.invoke(main, ["prepare", "coordinator-mock", "my.stack"])
    assert result.exit_code == 2
    assert "automatic planning is incomplete" in result.output
    assert "external blueprint 'my.stack'" in result.output


def test_run_stays_current_for_an_external_composition(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.cli.commands import lifecycle
    from dimos.deps import launch
    from dimos.deps.environment import EnvironmentReport

    monkeypatch.setattr(
        launch,
        "check_environment",
        lambda plan, **kwargs: EnvironmentReport(
            python="3.12", prefix=sys.prefix, dimos_version="0.0.14", checks=["packages"]
        ),
    )
    started: list[tuple[str, ...]] = []
    monkeypatch.setattr(
        lifecycle, "_start", lambda ctx, request, **kwargs: started.append(request.blueprint_names)
    )
    result = runner.invoke(main, ["run", "my-plugin.teleop", "coordinator-mock"])
    assert result.exit_code == 0, result.output
    assert "Automatic planning is incomplete; staying in the current environment" in result.output
    assert "external blueprint 'my-plugin.teleop'" in result.output
    assert started == [("my-plugin.teleop", "coordinator-mock")]
    result = runner.invoke(
        main, ["run", "my-plugin.teleop", "coordinator-mock", "--environment", "managed"]
    )
    assert result.exit_code == 2
    assert "cannot prepare a managed environment: automatic planning is incomplete" in result.output
