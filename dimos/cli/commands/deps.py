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

"""Dependency commands: explain a blueprint's requirements and diagnose an environment."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any

import typer

from dimos.cli.commands.lifecycle import (
    DEFAULT_CONFIG_PATH,
    RunRequest,
    plan_request,
    public_config,
    resolve_run_request,
)
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.catalog import CatalogError
from dimos.deps.environment import ALL_CHECKS, check_environment, format_report, is_checkout
from dimos.deps.lease import EnvironmentBusyError
from dimos.deps.managed import (
    PreparationError,
    Stamp,
    detect_source,
    ensure_environment,
    environment_key,
    environment_state,
    list_environments,
    prune_environments,
    remove_environment,
)
from dimos.deps.planning import RunPlan, install_recipes
from dimos.deps.policy import find_project_file
from dimos.deps.probe import ProbeError, ProbeRequest, run_probe
from dimos.deps.uv import UvNotFoundError
from dimos.utils.cache import cache_usage_guard

RUN_CONTEXT_SETTINGS = {"allow_extra_args": True, "ignore_unknown_options": True}
WHY_CHAINS = 3


def _plan_or_exit(request: RunRequest, profile: str | None) -> RunPlan:
    try:
        return plan_request(request, profile)
    except (CatalogError, ValueError) as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error


def _planning_lines(planned: RunPlan) -> list[str]:
    if planned.plan.complete:
        return ["Planning:    complete"]
    return ["Planning:    incomplete", *(f"  - {reason}" for reason in planned.plan.incomplete)]


def _profile_line(planned: RunPlan) -> str:
    if planned.profile is None:
        return f"unsupported host ({planned.unsupported}); backends left unresolved"
    origin = "--profile" if planned.overridden else "detected"
    return f"{planned.profile.name} ({origin})"


def deps(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to explain"),
    profile: str | None = typer.Option(None, "--profile", help="Hardware profile to plan for"),
    why: str | None = typer.Option(
        None, "--why", help="Show the import chain that needs this extra, distribution or import"
    ),
    json_output: bool = typer.Option(False, "--json", help="Machine-readable output"),
    config_path: Path = typer.Option(
        DEFAULT_CONFIG_PATH, "--config", "-c", help="Path to config file"
    ),
) -> None:
    """Explain which extras, backends and prerequisites a blueprint needs."""
    request = resolve_run_request(ctx, robot_types, config_path)
    planned = _plan_or_exit(request, profile)
    plan = planned.plan
    is_checkout(DIMOS_PROJECT_ROOT)
    report = check_environment(plan)
    recipes = install_recipes(plan)

    if json_output:
        payload: dict[str, Any] = {
            "blueprints": list(planned.builtin_names),
            "external": list(plan.external),
            "profile": planned.profile.name if planned.profile else None,
            "unsupported": str(planned.unsupported) if planned.unsupported else None,
            "incomplete": list(plan.incomplete),
            "extras": sorted(plan.extras),
            "reasons": {extra: sorted(plan.reasons[extra]) for extra in sorted(plan.reasons)},
            "backends": dict(plan.backends),
            "native": sorted(plan.native),
            "system": sorted(plan.system),
            "tools": sorted(plan.tools),
            "recipes": recipes,
            "environment": report.to_json(),
        }
        typer.echo(json.dumps(payload, indent=2))
        return

    typer.echo(f"Blueprints:  {', '.join(planned.builtin_names) or '(none built in)'}")
    for name in plan.external:
        typer.echo(f"External:    {name} (requirements unknown: no dependency metadata)")
    for line in _planning_lines(planned):
        typer.echo(line)
    typer.echo(f"Profile:     {_profile_line(planned)}")
    typer.echo(f"Extras:      {', '.join(sorted(plan.extras)) or 'none (core only)'}")
    for extra in sorted(plan.reasons):
        typer.echo(f"  {extra:<12} <- {', '.join(sorted(plan.reasons[extra]))}")
    for backend, accelerator in sorted(plan.backends.items()):
        resolved = f"-> {accelerator}" if accelerator else "-> unresolved"
        typer.echo(f"Backend:     {backend} {resolved}")
    if plan.native:
        typer.echo(f"Native:      {', '.join(sorted(plan.native))}")
    if plan.system:
        typer.echo(f"System:      {', '.join(sorted(plan.system))} (host-provided modules)")
    if plan.tools:
        typer.echo(f"Tools:       {', '.join(sorted(plan.tools))}")
    if recipes:
        typer.echo(f"Checkout:    {recipes['checkout']}")
        typer.echo(f"Release:     {recipes['release']}")
    name = ", ".join(planned.builtin_names)
    if report.satisfied_for_launch:
        typer.echo(f"Environment: {report.prefix}: satisfied")
    else:
        count = len(report.missing) + len(report.mismatched)
        typer.echo(
            f"Environment: {report.prefix}: {count} requirement(s) missing or mismatched "
            f"(run `dimos doctor {name}` for details)"
        )
    for excluded in report.excluded:
        typer.echo(f"Excluded:    {excluded} is not installed on this platform")
    if why:
        _explain(request, planned, why)


def _explain(request: RunRequest, planned: RunPlan, needle: str) -> None:
    """Recompute import chains from the shipped sources (no catalog reasons are stored)."""
    from dimos.deps.analysis import Analyzer, target_file
    from dimos.deps.lock import LockIndex
    from dimos.deps.rules import DEFAULTS
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    # The checkout's pyproject.toml, or the copy a wheel ships; the lock is optional.
    project_file = find_project_file(DIMOS_PROJECT_ROOT)
    lock = LockIndex.load(project_file.parent) if project_file is not None else None
    analyzer = Analyzer(DIMOS_PROJECT_ROOT, lock)
    config = {**DEFAULTS, **request.global_values}
    typer.echo(f"Why {needle}:")
    for name in planned.builtin_names:
        target = all_blueprints.get(name) or all_modules.get(name)
        if target is None:
            continue
        chains = analyzer.why([target_file(DIMOS_PROJECT_ROOT, target)], needle, config)
        if not chains:
            typer.echo(f"  {name}: no eager import path needs {needle}")
            continue
        for chain in chains[:WHY_CHAINS]:
            rendered = " -> ".join(
                path.relative_to(DIMOS_PROJECT_ROOT).as_posix() for path in chain
            )
            typer.echo(f"  {name}: {rendered}")


def doctor(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to diagnose"),
    profile: str | None = typer.Option(None, "--profile", help="Hardware profile to plan for"),
    environment: str = typer.Option(
        "current", "--environment", help="current, managed, or the path of a virtualenv"
    ),
    config_path: Path = typer.Option(
        DEFAULT_CONFIG_PATH, "--config", "-c", help="Path to config file"
    ),
) -> None:
    """Diagnose packages, providers, native modules, tools and backends for a blueprint."""
    request = resolve_run_request(ctx, robot_types, config_path)
    planned = _plan_or_exit(request, profile)
    python = _environment_python(environment, planned)
    for line in _planning_lines(planned):
        typer.echo(line)
    typer.echo(f"Profile:     {_profile_line(planned)}")
    typer.echo(f"Extras:      {', '.join(sorted(planned.plan.extras)) or 'none (core only)'}")
    probe = ProbeRequest.from_plan(
        planned.plan,
        ALL_CHECKS,
        blueprints=planned.builtin_names,
        global_config=public_config(request.global_values),
    )
    try:
        report = run_probe(python, probe)
    except ProbeError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    typer.echo(format_report(report, planned.plan, checkout=is_checkout(DIMOS_PROJECT_ROOT)))
    if not report.ok:
        raise typer.Exit(1)


def _environment_python(environment: str, planned: RunPlan) -> Path:
    if environment == "current":
        return Path(sys.executable)
    if environment == "managed":
        if planned.profile is None:
            typer.echo(f"Error: {planned.unsupported}", err=True)
            raise typer.Exit(2)
        key = environment_key(planned.plan, planned.profile, detect_source())
        stamp = Stamp.load(key.stamp_path)
        if stamp is None:
            names = " ".join(planned.builtin_names)
            typer.echo(
                f"Error: managed environment {key.name} is not prepared; run `dimos prepare {names}`",
                err=True,
            )
            raise typer.Exit(1)
        typer.echo(
            f"Managed:     {key.name} at {key.path} (prepared {stamp.created_at}, "
            f"uv {stamp.uv_version})"
        )
        return key.python_executable
    python = Path(environment).expanduser() / "bin" / "python"
    if not python.is_file():
        typer.echo(f"Error: {environment} is not a virtualenv (no bin/python)", err=True)
        raise typer.Exit(2)
    return python


def prepare(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to prepare for"),
    profile: str | None = typer.Option(None, "--profile", help="Hardware profile to prepare"),
    offline: bool = typer.Option(False, "--offline", help="Fail instead of downloading anything"),
    find_links: Path | None = typer.Option(
        None,
        "--find-links",
        help="Directory with locally built dimos wheels (installed dimos only)",
    ),
    config_path: Path = typer.Option(
        DEFAULT_CONFIG_PATH, "--config", "-c", help="Path to config file"
    ),
) -> None:
    """Prepare the managed runtime environment for a blueprint without running it."""
    request = resolve_run_request(ctx, robot_types, config_path)
    planned = _plan_or_exit(request, profile)
    if planned.profile is None:
        typer.echo(f"Error: {planned.unsupported}", err=True)
        raise typer.Exit(2)
    if not planned.plan.complete:
        reasons = "\n".join(f"  - {reason}" for reason in planned.plan.incomplete)
        typer.echo(f"Error: automatic planning is incomplete\n{reasons}", err=True)
        raise typer.Exit(2)
    source = detect_source(find_links=str(find_links) if find_links else None)
    if source.mode == "checkout" and find_links is not None:
        typer.echo("Error: --find-links applies to an installed dimos, not a checkout", err=True)
        raise typer.Exit(2)
    key = environment_key(planned.plan, planned.profile, source)
    try:
        with cache_usage_guard():
            _stamp, lease = ensure_environment(
                key,
                planned.plan,
                planned.profile,
                overridden_profile=planned.overridden,
                blueprints=planned.builtin_names,
                global_config=public_config(request.global_values),
                offline=offline,
                echo=lambda message: typer.echo(message, err=True),
            )
    except (PreparationError, UvNotFoundError, ProbeError) as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    lease.release()
    typer.echo(str(key.path))


envs_app = typer.Typer(help="Manage dimos-managed runtime environments", no_args_is_help=True)


@envs_app.command("list")
def envs_list() -> None:
    """List managed environments and whether a dimos process is preparing or using them."""
    environments = list_environments()
    if not environments:
        typer.echo("No managed environments.")
        return
    for path, stamp in environments:
        status = environment_state(path)
        if stamp is None:
            typer.echo(f"{path.name}: incomplete ({status})")
            continue
        source = stamp.source
        origin = (
            source.get("root")
            if source.get("mode") == "checkout"
            else f"dimos {source.get('version')} (tested lock)"
        )
        typer.echo(
            f"{stamp.name}: {stamp.profile}, python {stamp.python}, "
            f"extras [{', '.join(stamp.extras) or 'core only'}], from {origin}, "
            f"prepared {stamp.created_at[:19]}, {status}"
        )


@envs_app.command("remove")
def envs_remove(
    name: str = typer.Argument(..., help="Environment name from `dimos envs list`"),
) -> None:
    """Remove one managed environment (refused while a dimos process prepares or uses it)."""
    try:
        path = remove_environment(name)
    except (ValueError, FileNotFoundError) as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    except EnvironmentBusyError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    typer.echo(f"Removed {path}")


@envs_app.command("prune")
def envs_prune(
    all_unused: bool = typer.Option(
        False, "--all", help="Also remove environments that still match the current sources"
    ),
) -> None:
    """Remove unfinished and outdated managed environments no dimos process uses."""
    removed, skipped = prune_environments(all_unused=all_unused)
    for path in removed:
        typer.echo(f"Removed {path}")
    for reason in skipped:
        typer.echo(f"Skipped: {reason}")
    if not removed and not skipped:
        typer.echo("Nothing to prune.")
