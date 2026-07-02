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

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
import subprocess

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    RuntimePlacement,
)
from dimos.utils.safe_thread_map import safe_thread_map


class RuntimeReconciliationError(RuntimeError):
    pass


@dataclass(frozen=True)
class RuntimeReconciliationCommand:
    argv: tuple[str, ...]
    cwd: Path
    env: Mapping[str, str]
    description: str


@dataclass(frozen=True)
class RuntimeReconciliationItem:
    runtime_name: str
    runtime: PythonProjectRuntimeEnvironment
    commands: tuple[RuntimeReconciliationCommand, ...]


@dataclass(frozen=True)
class RuntimeReconciliationPlan:
    items: tuple[RuntimeReconciliationItem, ...]


class SubprocessRuntimeCommandRunner:
    def run(self, command: RuntimeReconciliationCommand) -> None:
        subprocess.run(
            command.argv,
            cwd=command.cwd,
            env={**os.environ, **command.env},
            check=True,
        )


def select_runtime_reconciliation_plan(blueprint: Blueprint) -> RuntimeReconciliationPlan:
    active_modules = {bp.module for bp in blueprint.active_blueprints}
    active_runtime_names = {
        placement.runtime
        for module, placement in blueprint.runtime_placement_map.items()
        if module in active_modules
    }
    items: list[RuntimeReconciliationItem] = []
    for runtime_name in sorted(active_runtime_names):
        runtime = blueprint.runtime_environment_registry.resolve(runtime_name)
        if isinstance(runtime, PythonProjectRuntimeEnvironment):
            runtime.validate_project_files()
            items.append(
                RuntimeReconciliationItem(
                    runtime_name=runtime_name,
                    runtime=runtime,
                    commands=tuple(_commands_for_project_runtime(runtime)),
                )
            )
    return RuntimeReconciliationPlan(items=tuple(items))


def reconcile_blueprint_runtimes(
    blueprint: Blueprint,
    *,
    runner: SubprocessRuntimeCommandRunner | None = None,
    output: Callable[[str], None] | None = None,
) -> RuntimeReconciliationPlan:
    _validate_runtime_placements(blueprint)
    plan = select_runtime_reconciliation_plan(blueprint)
    reconcile_runtime_plan(plan, runner=runner, output=output)
    return plan


def reconcile_runtime_plan(
    plan: RuntimeReconciliationPlan,
    *,
    runner: SubprocessRuntimeCommandRunner | None = None,
    output: Callable[[str], None] | None = None,
) -> None:
    command_runner = runner or SubprocessRuntimeCommandRunner()

    def _run_item(item: RuntimeReconciliationItem) -> None:
        for command in item.commands:
            if output is not None:
                output(f"[{item.runtime_name}] {command.description}: {' '.join(command.argv)}")
            try:
                command_runner.run(command)
            except subprocess.CalledProcessError as e:
                raise RuntimeReconciliationError(
                    f"Runtime reconciliation failed for {item.runtime_name!r} at "
                    f"{item.runtime.project_path}: {' '.join(command.argv)} exited {e.returncode}. "
                    "Lockfile mutation belongs to a manual package-manager command or "
                    "future DimOS build/update command."
                ) from e
            except FileNotFoundError as e:
                raise RuntimeReconciliationError(
                    f"Runtime reconciliation failed for {item.runtime_name!r}: "
                    f"command {command.argv[0]!r} was not found"
                ) from e

    def _raise_grouped(
        outcomes: list[tuple[RuntimeReconciliationItem, None | Exception]],
        _successes: list[None],
        errors: list[Exception],
    ) -> None:
        lines = ["Runtime reconciliation failed before worker launch:"]
        for item, result in outcomes:
            if isinstance(result, Exception):
                lines.append(f"- {item.runtime_name} ({item.runtime.project_path}): {result}")
        raise RuntimeReconciliationError("\n".join(lines)) from errors[0]

    safe_thread_map(list(plan.items), _run_item, on_errors=_raise_grouped)


def active_runtime_placements(
    blueprint: Blueprint,
) -> Mapping[type[ModuleBase], RuntimePlacement]:
    active_modules = {bp.module for bp in blueprint.active_blueprints}
    return {
        module: placement
        for module, placement in blueprint.runtime_placement_map.items()
        if module in active_modules
    }


def _validate_runtime_placements(blueprint: Blueprint) -> None:
    blueprint_modules = {bp.module for bp in blueprint.blueprints}
    active_modules = {bp.module for bp in blueprint.active_blueprints}
    for module, placement in blueprint.runtime_placement_map.items():
        if module not in blueprint_modules:
            raise RuntimeReconciliationError(
                f"Runtime placement for {module.__name__} does not match a blueprint module"
            )
        if module in active_modules:
            blueprint.runtime_environment_registry.resolve(placement.runtime)


def _commands_for_project_runtime(
    runtime: PythonProjectRuntimeEnvironment,
) -> Sequence[RuntimeReconciliationCommand]:
    project = runtime.project_path
    env = dict(runtime.env)
    if runtime.convention == "pixi-backed-uv":
        return (
            RuntimeReconciliationCommand(
                argv=("pixi", "install", "--locked"),
                cwd=project,
                env=env,
                description="reconcile Pixi environment in locked mode",
            ),
            RuntimeReconciliationCommand(
                argv=(
                    "pixi",
                    "run",
                    "uv",
                    "venv",
                    "-p",
                    ".pixi/envs/default/bin/python",
                    "--seed",
                    "--allow-existing",
                ),
                cwd=project,
                env=env,
                description="ensure uv virtualenv from Pixi Python",
            ),
            RuntimeReconciliationCommand(
                argv=("pixi", "run", "uv", "sync", "--locked"),
                cwd=project,
                env=env,
                description="sync uv project in locked mode",
            ),
        )
    return (
        RuntimeReconciliationCommand(
            argv=("uv", "venv", "--seed", "--allow-existing"),
            cwd=project,
            env=env,
            description="ensure uv virtualenv",
        ),
        RuntimeReconciliationCommand(
            argv=("uv", "sync", "--locked"),
            cwd=project,
            env=env,
            description="sync uv project in locked mode",
        ),
    )
