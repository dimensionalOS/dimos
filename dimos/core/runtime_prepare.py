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

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import subprocess

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    PythonProjectRuntimeEnvironmentError,
    PythonVenvRuntimeEnvironment,
)


class RuntimePrepareError(RuntimeError):
    """Raised when runtime preparation cannot select or prepare a runtime."""


@dataclass(frozen=True)
class RuntimePrepareCommand:
    runtime_name: str
    project: Path
    convention: str
    argv: list[str]


@dataclass(frozen=True)
class RuntimePreparePlan:
    commands: list[RuntimePrepareCommand]
    no_op_runtime_names: list[str]


SubprocessRunner = Callable[..., subprocess.CompletedProcess[object]]


def select_runtime_prepare_plan(
    blueprint: Blueprint, runtime_name: str | None = None
) -> RuntimePreparePlan:
    active_module_classes = {atom.module for atom in blueprint.active_blueprints}
    active_placements = {
        module_class: placed_runtime_name
        for module_class, placed_runtime_name in blueprint.runtime_placement_map.items()
        if module_class in active_module_classes
    }
    active_runtime_names = sorted(set(active_placements.values()))
    registry = blueprint.runtime_environment_registry

    if runtime_name is not None:
        if runtime_name in active_runtime_names and runtime_name not in registry.environments:
            raise _missing_active_runtime_error(runtime_name, active_placements)
        try:
            registry.resolve(runtime_name)
        except KeyError as exc:
            raise RuntimePrepareError(str(exc)) from exc
        if runtime_name not in active_runtime_names:
            used = ", ".join(active_runtime_names) or "<none>"
            raise RuntimePrepareError(
                f"Runtime environment '{runtime_name}' is registered but is not used by the "
                f"active blueprint configuration. Active placed runtimes: {used}"
            )
        active_runtime_names = [runtime_name]

    commands: list[RuntimePrepareCommand] = []
    no_op_runtime_names: list[str] = []
    for active_runtime_name in active_runtime_names:
        try:
            environment = registry.resolve(active_runtime_name)
        except KeyError as exc:
            raise _missing_active_runtime_error(active_runtime_name, active_placements) from exc
        if isinstance(environment, PythonProjectRuntimeEnvironment):
            try:
                commands.extend(_commands_for_project_runtime(environment))
            except PythonProjectRuntimeEnvironmentError as exc:
                raise RuntimePrepareError(str(exc)) from exc
        elif isinstance(environment, PythonVenvRuntimeEnvironment):
            no_op_runtime_names.append(active_runtime_name)
        else:
            # Current-process/native/etc. placements are not Python project runtimes and have no
            # preparation contract in this change.
            no_op_runtime_names.append(active_runtime_name)
    return RuntimePreparePlan(commands=commands, no_op_runtime_names=no_op_runtime_names)


def _missing_active_runtime_error(
    runtime_name: str,
    active_placements: dict[type, str],
) -> RuntimePrepareError:
    modules = sorted(
        module_class.__name__
        for module_class, placed_runtime_name in active_placements.items()
        if placed_runtime_name == runtime_name
    )
    module_list = ", ".join(modules) or "<unknown>"
    return RuntimePrepareError(
        f"Active runtime environment '{runtime_name}' is used by module(s) "
        f"{module_list}, but it is not registered on the blueprint."
    )


def prepare_runtime_plan(
    plan: RuntimePreparePlan,
    *,
    runner: SubprocessRunner | None = None,
    output: Callable[[str], None] = print,
) -> None:
    run_command = runner or subprocess.run
    for runtime_name in plan.no_op_runtime_names:
        output(
            f"Runtime '{runtime_name}' is an active direct/external environment; no prepare step is required."
        )

    for command in plan.commands:
        output(
            f"Preparing runtime '{command.runtime_name}' "
            f"(convention: {command.convention}, project: {command.project})"
        )
        output(f"Running: {' '.join(command.argv)}")
        try:
            run_command(command.argv, check=True, cwd=command.project)
        except (OSError, subprocess.CalledProcessError) as exc:
            raise RuntimePrepareError(
                f"Failed preparing runtime '{command.runtime_name}' at '{command.project}' "
                f"with command {command.argv!r}"
            ) from exc


def prepare_blueprint_runtimes(
    blueprint: Blueprint,
    runtime_name: str | None = None,
    *,
    runner: SubprocessRunner | None = None,
    output: Callable[[str], None] = print,
) -> RuntimePreparePlan:
    plan = select_runtime_prepare_plan(blueprint, runtime_name)
    prepare_runtime_plan(plan, runner=runner, output=output)
    return plan


def _commands_for_project_runtime(
    environment: PythonProjectRuntimeEnvironment,
) -> list[RuntimePrepareCommand]:
    convention = environment.convention()
    if convention == "pixi-backed-uv":
        argv_by_step = [
            ["pixi", "install"],
            [
                "pixi",
                "run",
                "uv",
                "venv",
                "-p",
                ".pixi/envs/default/bin/python",
                "--seed",
            ],
            ["pixi", "run", "uv", "sync"],
        ]
    else:
        argv_by_step = [["uv", "venv", "--seed"], ["uv", "sync"]]
    return [
        RuntimePrepareCommand(
            runtime_name=environment.name,
            project=environment.project,
            convention=convention,
            argv=argv,
        )
        for argv in argv_by_step
    ]
