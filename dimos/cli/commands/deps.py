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

"""Dependency commands: ``dimos deps`` explains, ``dimos prepare`` installs."""

from __future__ import annotations

import importlib.metadata
from typing import NoReturn

import typer

from dimos.deps import install
from dimos.deps.backend import (
    BACKEND_CHOICES,
    DEPENDENCIES_DOC,
    Backend,
    UnsupportedError,
    check_supported,
    limitations,
    python_version,
    resolve_backend,
)
from dimos.deps.bundles import (
    BundleMetadataError,
    UnknownNameError,
    bundles_for,
    is_external_name,
    load_assignments,
)

NAMES_ARGUMENT = typer.Argument(..., help="Blueprint or module names, as listed by `dimos list`")
BACKEND_OPTION = typer.Option(
    "auto", "--backend", help="Inference provider extra to install: auto, cpu or cuda"
)


def _fail(message: str, code: int) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(code)


def _split_names(names: list[str]) -> tuple[list[str], list[str]]:
    builtin = [name for name in names if not is_external_name(name)]
    external = [name for name in names if is_external_name(name)]
    return builtin, external


def _assignments() -> dict[str, str]:
    try:
        return load_assignments()
    except BundleMetadataError as e:
        _fail(str(e), 2)


def _bundles(names: list[str], assignments: dict[str, str]) -> list[str]:
    try:
        return bundles_for(names, assignments)
    except UnknownNameError as e:
        _fail(str(e), 1)
    except BundleMetadataError as e:
        _fail(str(e), 2)


def _backend(choice: str) -> tuple[Backend, str]:
    if choice not in BACKEND_CHOICES:
        _fail(f"--backend must be one of {', '.join(BACKEND_CHOICES)}, not {choice!r}", 2)
    try:
        return resolve_backend(choice)
    except UnsupportedError as e:
        _fail(str(e), 2)


def _source_line() -> str:
    root = install.checkout_root()
    if root is not None:
        return f"checkout {root} (uv.lock)"
    return f"packaged lock artifacts of dimos {importlib.metadata.version('dimos')}"


def _external_source(name: str) -> str:
    # Entry-point discovery imports the blueprint framework; only pay for it when asked.
    from dimos.robot.external_blueprints import list_external_blueprints

    for entry in list_external_blueprints():
        if entry.qualified_name == name:
            return (
                f"external, provided by distribution {entry.distribution_name!r} "
                "(its dependencies come from that package)"
            )
    return "external, not found among installed `dimos.blueprints` entry points"


def deps(names: list[str] = NAMES_ARGUMENT, backend: str = BACKEND_OPTION) -> None:
    """Show which dependency bundle each name needs and how to install it (read-only)."""
    builtin, external = _split_names(names)
    assignments = _assignments()
    bundles = _bundles(builtin, assignments)
    width = max(len(name) for name in names)
    for name in names:
        label = _external_source(name) if name in external else assignments[name]
        typer.echo(f"  {name:<{width}}  {label}")
    chosen, reason = _backend(backend)
    target = install.Target.current()
    environment = (
        f"virtualenv {target.prefix}"
        if target.is_virtualenv
        else f"system Python {target.prefix}; create a virtualenv before preparing"
    )
    typer.echo("")
    typer.echo(f"Bundles:      {', '.join(bundles) or '(none: external names only)'}")
    typer.echo(f"Backend:      {chosen} ({reason})")
    typer.echo(f"Python:       {target.python} ({python_version()}, {environment})")
    typer.echo(f"Source:       {_source_line()}")
    if builtin:
        typer.echo(f"Prepare:      dimos prepare {' '.join(builtin)} --backend {chosen}")
    unsupported = None
    try:
        check_supported(bundles, chosen)
    except UnsupportedError as e:
        unsupported = str(e)
        typer.echo(f"Unsupported:  {unsupported}")
    typer.echo("Limitations:")
    for note in limitations(bundles, chosen):
        typer.echo(f"  - {note}")
    typer.echo(f"Setup guide:  {DEPENDENCIES_DOC}")
    if unsupported:
        raise typer.Exit(2)


def prepare(
    names: list[str] = NAMES_ARGUMENT,
    backend: str = BACKEND_OPTION,
    offline: bool = typer.Option(False, "--offline", help="Install only from uv's cache"),
) -> None:
    """Install the Python dependencies of the named blueprints or modules into this venv."""
    builtin, external = _split_names(names)
    if external:
        _fail(
            "external blueprints get their dependencies from their own package, not from "
            f"dimos prepare: {', '.join(external)}",
            2,
        )
    bundles = _bundles(builtin, _assignments())
    chosen, reason = _backend(backend)
    try:
        check_supported(bundles, chosen)
    except UnsupportedError as e:
        _fail(str(e), 2)
    target = install.Target.current()
    typer.echo(f"Environment:  {target.prefix}")
    typer.echo(f"Python:       {target.python} ({python_version()})")
    typer.echo(f"Bundles:      {', '.join(bundles)}")
    typer.echo(f"Backend:      {chosen} ({reason})")
    typer.echo(f"Source:       {_source_line()}")
    try:
        install.prepare(bundles, chosen, offline)
    except install.PrepareError as e:
        _fail(str(e), 1)
    typer.echo(
        f"Installed Python dependencies for {', '.join(bundles)} ({chosen}) into {target.prefix}."
    )
    typer.echo(
        f"Native modules, hardware and vendor SDKs are not verified; see {DEPENDENCIES_DOC}."
    )
