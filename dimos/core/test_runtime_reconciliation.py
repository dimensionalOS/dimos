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
from __future__ import annotations

from dataclasses import replace
import os
from pathlib import Path
import subprocess
from types import MappingProxyType

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import ModuleBase
from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    RuntimePlacement,
    UnknownRuntimeEnvironmentError,
)
from dimos.core.runtime_reconciliation import (
    RuntimeReconciliationCommand,
    RuntimeReconciliationError,
    RuntimeReconciliationItem,
    RuntimeReconciliationPlan,
    SubprocessRuntimeCommandRunner,
    active_runtime_placements,
    reconcile_blueprint_runtimes,
    reconcile_runtime_plan,
    select_runtime_reconciliation_plan,
)


class RuntimeModule(ModuleBase):
    pass


class DisabledRuntimeModule(ModuleBase):
    pass


class UnrelatedRuntimeModule(ModuleBase):
    pass


class FakeRunner(SubprocessRuntimeCommandRunner):
    def __init__(self, failing_command: tuple[str, ...] | None = None) -> None:
        self.commands: list[RuntimeReconciliationCommand] = []
        self.failing_command = failing_command

    def run(self, command: RuntimeReconciliationCommand) -> None:
        self.commands.append(command)
        if command.argv == self.failing_command:
            raise subprocess.CalledProcessError(returncode=17, cmd=command.argv)


def _write_project_files(tmp_path: Path, *, pixi: bool = False) -> PythonProjectRuntimeEnvironment:
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'example'\n", encoding="utf-8")
    (tmp_path / "uv.lock").write_text("version = 1\n", encoding="utf-8")
    if pixi:
        (tmp_path / "pixi.toml").write_text("[workspace]\n", encoding="utf-8")
        (tmp_path / "pixi.lock").write_text("version = 1\n", encoding="utf-8")
    return PythonProjectRuntimeEnvironment(name="project", project=tmp_path, env={"A": "B"})


def test_runtime_placement_validation_rejects_unknown_runtime(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path)
    blueprint = RuntimeModule.blueprint().runtime_environments(runtime)

    with pytest.raises(
        UnknownRuntimeEnvironmentError, match="Unknown runtime environment 'missing'"
    ):
        reconcile_blueprint_runtimes(
            replace(
                blueprint,
                runtime_placement_map=MappingProxyType(
                    {RuntimeModule: RuntimePlacement(runtime="missing", implementation="impl.py")}
                ),
            ),
            runner=FakeRunner(),
        )


def test_runtime_placement_validation_rejects_non_blueprint_module(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path)
    blueprint = RuntimeModule.blueprint().runtime_environments(runtime)

    with pytest.raises(RuntimeReconciliationError, match="does not match a blueprint module"):
        reconcile_blueprint_runtimes(
            replace(
                blueprint,
                runtime_placement_map=MappingProxyType(
                    {
                        UnrelatedRuntimeModule: RuntimePlacement(
                            runtime="project", implementation="impl.py"
                        )
                    }
                ),
            ),
            runner=FakeRunner(),
        )


def test_active_placements_exclude_disabled_modules(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path)
    blueprint = autoconnect(RuntimeModule.blueprint(), DisabledRuntimeModule.blueprint())
    blueprint = blueprint.runtime_environments(runtime).runtime_placements(
        {
            RuntimeModule: RuntimePlacement(runtime="project", implementation="enabled.py"),
            DisabledRuntimeModule: RuntimePlacement(
                runtime="project", implementation="disabled.py"
            ),
        }
    )
    blueprint = blueprint.disabled_modules(DisabledRuntimeModule)

    assert active_runtime_placements(blueprint) == {
        RuntimeModule: RuntimePlacement(runtime="project", implementation="enabled.py")
    }
    assert [item.runtime_name for item in select_runtime_reconciliation_plan(blueprint).items] == [
        "project"
    ]


def test_disabled_runtime_placement_does_not_require_runtime_registration(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path)
    blueprint = autoconnect(RuntimeModule.blueprint(), DisabledRuntimeModule.blueprint())
    blueprint = blueprint.runtime_environments(runtime).runtime_placements(
        {
            RuntimeModule: RuntimePlacement(runtime="project", implementation="enabled.py"),
            DisabledRuntimeModule: RuntimePlacement(
                runtime="missing", implementation="disabled.py"
            ),
        }
    )
    blueprint = blueprint.disabled_modules(DisabledRuntimeModule)

    plan = reconcile_blueprint_runtimes(blueprint, runner=FakeRunner())

    assert [item.runtime_name for item in plan.items] == ["project"]


def test_reconciliation_command_selection_for_uv(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path)
    blueprint = (
        RuntimeModule.blueprint()
        .runtime_environments(runtime)
        .runtime_placements(
            {RuntimeModule: RuntimePlacement(runtime="project", implementation="impl.py")}
        )
    )

    plan = select_runtime_reconciliation_plan(blueprint)

    assert [command.argv for command in plan.items[0].commands] == [
        ("uv", "venv", "--seed", "--allow-existing"),
        ("uv", "sync", "--locked"),
    ]
    assert all(command.cwd == tmp_path for command in plan.items[0].commands)


def test_subprocess_runner_merges_command_env_with_parent_env(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    captured_env: dict[str, str] = {}

    def fake_run(
        argv: tuple[str, ...],
        *,
        cwd: Path,
        env: dict[str, str],
        check: bool,
    ) -> None:
        assert argv == ("uv", "sync", "--locked")
        assert cwd == tmp_path
        assert check is True
        captured_env.update(env)

    monkeypatch.setenv("DIMOS_PARENT_ENV", "parent")
    monkeypatch.setattr(subprocess, "run", fake_run)

    SubprocessRuntimeCommandRunner().run(
        RuntimeReconciliationCommand(
            argv=("uv", "sync", "--locked"),
            cwd=tmp_path,
            env={"DIMOS_COMMAND_ENV": "command"},
            description="sync uv project in locked mode",
        )
    )

    assert captured_env["DIMOS_PARENT_ENV"] == "parent"
    assert captured_env["DIMOS_COMMAND_ENV"] == "command"
    assert captured_env["PATH"] == os.environ["PATH"]


def test_reconciliation_command_selection_for_pixi_backed_uv(tmp_path: Path) -> None:
    runtime = _write_project_files(tmp_path, pixi=True)
    blueprint = (
        RuntimeModule.blueprint()
        .runtime_environments(runtime)
        .runtime_placements(
            {RuntimeModule: RuntimePlacement(runtime="project", implementation="impl.py")}
        )
    )

    plan = select_runtime_reconciliation_plan(blueprint)

    assert [command.argv for command in plan.items[0].commands] == [
        ("pixi", "install", "--locked"),
        (
            "pixi",
            "run",
            "uv",
            "venv",
            "-p",
            ".pixi/envs/default/bin/python",
            "--seed",
            "--allow-existing",
        ),
        ("pixi", "run", "uv", "sync", "--locked"),
    ]


def test_fake_runner_records_commands_and_grouped_failure_surfaces_before_worker_launch(
    tmp_path: Path,
) -> None:
    runtime = _write_project_files(tmp_path)
    commands = (
        RuntimeReconciliationCommand(
            argv=("uv", "venv", "--seed"),
            cwd=tmp_path,
            env={},
            description="ensure uv virtualenv",
        ),
        RuntimeReconciliationCommand(
            argv=("uv", "sync", "--locked"),
            cwd=tmp_path,
            env={},
            description="sync uv project in locked mode",
        ),
    )
    plan = RuntimeReconciliationPlan(
        items=(
            RuntimeReconciliationItem(runtime_name="project", runtime=runtime, commands=commands),
        )
    )
    runner = FakeRunner(failing_command=("uv", "sync", "--locked"))

    with pytest.raises(RuntimeReconciliationError) as error:
        reconcile_runtime_plan(plan, runner=runner)

    assert [command.argv for command in runner.commands] == [command.argv for command in commands]
    message = str(error.value)
    assert "Runtime reconciliation failed before worker launch" in message
    assert "project" in message
    assert "exited 17" in message
