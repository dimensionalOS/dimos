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

"""Choose the interpreter a run executes in, and hand the process over to it."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
import sys
from typing import Any, NoReturn

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.environment import (
    CHEAP_CHECKS,
    EnvironmentReport,
    check_environment,
    format_report,
    is_checkout,
)
from dimos.deps.lease import EnvironmentLease, use_lock
from dimos.deps.managed import (
    PreparationError,
    Source,
    detect_source,
    ensure_environment,
    environment_key,
    is_running_inside,
    managed_environment_of,
    use_lease,
)
from dimos.deps.planning import RunPlan, install_recipes
from dimos.deps.probe import ProbeError, ProbeRequest, run_probe
from dimos.deps.uv import UvNotFoundError, UvRunner

LAUNCHED_ENV = "DIMOS_LAUNCHED_ENVIRONMENT"
OFFLINE_ENV = {"UV_OFFLINE": "1", "HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1"}
Echo = Callable[[str], None]


class LaunchError(RuntimeError):
    def __init__(self, exit_code: int, message: str) -> None:
        super().__init__(message)
        self.exit_code = exit_code


@dataclass(frozen=True)
class Decision:
    dimos_executable: Path | None
    """Console script to exec, or ``None`` to continue in this interpreter."""
    env_dir: Path | None
    reason: str
    lease: EnvironmentLease | None = None
    """Use lease on the environment to exec into, handed over to the new process."""


def select_environment(
    planned: RunPlan,
    *,
    environment: str,
    offline: bool,
    blueprints: Sequence[str],
    global_config: Mapping[str, Any],
    source: Source | None = None,
    checker: Callable[..., EnvironmentReport] | None = None,
    probe: Callable[..., EnvironmentReport] | None = None,
    uv_factory: Callable[[], UvRunner] | None = None,
    echo: Echo = print,
) -> Decision:
    """Apply the ``--environment`` policy: auto, current, managed or a virtualenv path."""
    # Late binding keeps the module attributes patchable in tests.
    checker = checker or check_environment
    probe = probe or run_probe
    uv_factory = uv_factory or UvRunner
    plan = planned.plan
    checkout = is_checkout(DIMOS_PROJECT_ROOT)
    if not plan.complete:
        reasons = "\n".join(f"  - {reason}" for reason in plan.incomplete)
        if environment == "managed":
            raise LaunchError(
                2,
                "cannot prepare a managed environment: automatic planning is incomplete\n"
                f"{reasons}\nRun with --environment current, or pass the path of a virtualenv "
                "that has every requirement installed.",
            )
        if environment == "auto":
            echo(
                f"Automatic planning is incomplete; staying in the current environment:\n{reasons}"
            )
            environment = "current"
    if environment == "current":
        report = checker(plan)
        if not report.satisfied_for_launch:
            hint = ""
            if LAUNCHED_ENV in os.environ:
                hint = (
                    f"\nThe managed environment {os.environ[LAUNCHED_ENV]} does not match its "
                    "plan; remove it with `dimos envs remove <name>` and run again."
                )
            raise LaunchError(1, format_report(report, plan, checkout=checkout) + hint)
        _require_prerequisites(report, plan, checkout)
        return Decision(None, None, "current environment satisfies the plan")

    if environment == "managed":
        return _managed(
            planned, offline, blueprints, global_config, source, probe, uv_factory, echo
        )

    if environment == "auto":
        report = checker(plan)
        if report.satisfied_for_launch:
            _require_prerequisites(report, plan, checkout)
            return Decision(None, None, "current environment satisfies the plan")
        lacking = [issue.requirement for issue in (*report.missing, *report.mismatched)]
        if LAUNCHED_ENV in os.environ:
            raise LaunchError(
                1,
                f"managed environment {os.environ[LAUNCHED_ENV]} lacks {', '.join(lacking)}; "
                "remove it with `dimos envs remove <name>` and run again",
            )
        if planned.profile is None:
            recipes = install_recipes(plan)
            recipe = recipes.get("checkout" if checkout else "release", "")
            raise LaunchError(
                1,
                f"{format_report(report, plan, checkout=checkout)}\nThis host has no tested "
                f"profile ({planned.unsupported}), so no managed environment can be prepared."
                + (f" Install the requirements yourself: {recipe}" if recipe else ""),
            )
        summary = ", ".join(lacking[:3]) + (
            f" (+{len(lacking) - 3} more)" if len(lacking) > 3 else ""
        )
        echo(f"Current environment lacks {summary}; using a managed environment")
        return _managed(
            planned, offline, blueprints, global_config, source, probe, uv_factory, echo
        )

    return _explicit_path(environment, plan, checkout, probe, echo)


def _require_prerequisites(report: EnvironmentReport, plan: Any, checkout: bool) -> None:
    """A missing host module, in-tree native module or tool fails here: packages cannot fix it."""
    if not report.ok:
        raise LaunchError(1, format_report(report, plan, checkout=checkout))


def _managed(
    planned: RunPlan,
    offline: bool,
    blueprints: Sequence[str],
    global_config: Mapping[str, Any],
    source: Source | None,
    probe: Callable[..., EnvironmentReport],
    uv_factory: Callable[[], UvRunner],
    echo: Echo,
) -> Decision:
    if planned.profile is None:
        raise LaunchError(2, str(planned.unsupported))
    source = source or detect_source()
    key = environment_key(planned.plan, planned.profile, source)
    if is_running_inside(key):
        return Decision(None, None, f"already running inside {key.name}")
    try:
        stamp, lease = ensure_environment(
            key,
            planned.plan,
            planned.profile,
            overridden_profile=planned.overridden,
            blueprints=blueprints,
            global_config=global_config,
            offline=offline,
            prepare_missing=not offline,
            uv=uv_factory(),
            probe=probe,
            echo=echo,
        )
    except (PreparationError, UvNotFoundError, ProbeError) as error:
        raise LaunchError(1, str(error)) from error
    return Decision(key.dimos_executable, key.path, f"managed environment {stamp.name}", lease)


def _explicit_path(
    environment: str,
    plan: Any,
    checkout: bool,
    probe: Callable[..., EnvironmentReport],
    echo: Echo,
) -> Decision:
    root = Path(environment).expanduser()
    python = root / "bin" / "python"
    dimos = root / "bin" / "dimos"
    if not python.is_file() or not dimos.is_file():
        raise LaunchError(
            2, f"{root} is not a virtualenv with dimos installed (needs bin/python and bin/dimos)"
        )
    try:
        report = probe(python, ProbeRequest.from_plan(plan, CHEAP_CHECKS))
    except ProbeError as error:
        raise LaunchError(1, str(error)) from error
    if not report.satisfied_for_launch:
        raise LaunchError(1, format_report(report, plan, checkout=checkout))
    managed = managed_environment_of(root)
    lease = use_lease(managed, echo=echo) if managed is not None else None
    return Decision(dimos, root, f"environment {root} satisfies the plan", lease)


def hold_current_lease(*, echo: Echo = print) -> EnvironmentLease | None:
    """The use lease of the managed environment this interpreter runs from, if any.

    A launcher that exec'd into the environment exported its lease; otherwise
    (``dimos restart``, ``--environment current`` inside the environment) take one.
    """
    managed = managed_environment_of(Path(sys.prefix))
    if managed is None:
        return None
    return EnvironmentLease.adopt(os.environ, use_lock(managed)) or use_lease(managed, echo=echo)


def replace_option(argv: Sequence[str], option: str, value: str) -> list[str]:
    """``argv`` with every ``option X`` / ``option=X`` removed and ``option value`` appended."""
    result: list[str] = []
    skip = False
    for item in argv:
        if skip:
            skip = False
            continue
        if item == option:
            skip = True
            continue
        if item.startswith(option + "="):
            continue
        result.append(item)
    result.extend([option, value])
    return result


def exec_into(
    dimos_executable: Path,
    env_dir: Path,
    argv: Sequence[str],
    *,
    offline: bool,
    lease: EnvironmentLease | None = None,
) -> NoReturn:
    """Replace this process with the environment's ``dimos`` running the same command."""
    env = dict(os.environ)
    env[LAUNCHED_ENV] = str(env_dir)
    env["VIRTUAL_ENV"] = str(dimos_executable.parent.parent)
    env["PATH"] = f"{dimos_executable.parent}{os.pathsep}{env.get('PATH', '')}"
    env.pop("PYTHONHOME", None)
    if offline:
        env.update(OFFLINE_ENV)
    if lease is not None:
        lease.export(env)
    arguments = [str(dimos_executable), *replace_option(list(argv)[1:], "--environment", "current")]
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        os.execve(str(dimos_executable), arguments, env)
    except OSError as error:
        raise LaunchError(1, f"could not start {dimos_executable}: {error}") from error
