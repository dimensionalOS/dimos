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
import sys
from typing import TypedDict

from rich.console import Console
from rich.table import Table
from rich.text import Text
import typer

from dimos.cli import theme
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo
from dimos.core.run_registry import get_most_recent
from dimos.porcelain.dimos import DescribeTarget, Dimos
from dimos.porcelain.module_handle import ModuleHandle

QUICK_START = """\
[bold]Quick start[/bold]
  guide()                           Show this guide again
  modules()                         List deployed module instances
  rpcs()                            List every RPC
  rpcs("ModuleName")                Filter RPCs by module
  describe("ModuleName.method")     Show a signature and documentation
  app.ModuleName.method(...)        Invoke an RPC
  app.ModuleName.method?            Inspect an RPC with IPython"""


class ShellNamespace(TypedDict):
    app: Dimos
    guide: Callable[[], None]
    modules: Callable[[], None]
    rpcs: Callable[[str | ModuleHandle | None], None]
    describe: Callable[[DescribeTarget], None]


def _is_interactive_terminal() -> bool:
    return sys.stdin.isatty() and sys.stdout.isatty()


def _rpc_signature(rpc: RpcInfo) -> str:
    parameters = ", ".join(repr(param) for param in rpc.params)
    returns = f" -> {rpc.return_type}" if rpc.return_type is not None else ""
    return f"{rpc.name}({parameters}){returns}"


def _format_description(info: ModuleInfo | RpcInfo) -> str:
    if isinstance(info, RpcInfo):
        qualified_name = (
            f"{info.module_name}.{info.name}" if info.module_name is not None else info.name
        )
        return "\n".join(
            [
                f"RPC: {qualified_name}",
                f"Signature: {_rpc_signature(info)}",
                "",
                "Documentation:",
                info.documentation or "  (none)",
            ]
        )

    instance_name = info.instance_name or info.name
    class_name = info.qualified_path or info.class_name or info.name
    lines = [
        f"Module: {instance_name}",
        f"Class: {class_name}",
        "",
        "Documentation:",
        info.documentation or "  (none)",
        "",
        "RPCs:",
    ]
    lines.extend(f"  {_rpc_signature(rpc)}" for rpc in info.rpcs)
    if not info.rpcs:
        lines.append("  (none)")
    return "\n".join(lines)


def _shell_namespace(app: Dimos) -> ShellNamespace:
    def guide() -> None:
        """Print the DimOS RPC shell quick-start guide."""
        Console(highlight=False).print(QUICK_START)

    def modules() -> None:
        """Print deployed module instances."""
        table = Table("Instance", "Class", "RPCs", header_style=theme.ACCENT)
        for info in app.list_modules():
            table.add_row(
                info.instance_name or info.name,
                info.class_name or info.name,
                str(len(info.rpcs)),
            )
        Console().print(table)

    def rpcs(module: str | ModuleHandle | None = None) -> None:
        """Print advertised RPC signatures and summaries."""
        table = Table("RPC", "Signature", "Description", header_style=theme.ACCENT)
        for info in app.list_rpcs(module):
            qualified_name = (
                f"{info.module_name}.{info.name}" if info.module_name is not None else info.name
            )
            summary = info.documentation.splitlines()[0] if info.documentation else ""
            table.add_row(qualified_name, _rpc_signature(info), summary)
        Console().print(table)

    def describe(target: DescribeTarget) -> None:
        """Print a module or RPC's signature and documentation."""
        Console().print(Text(_format_description(app.describe(target))))

    return {
        "app": app,
        "guide": guide,
        "modules": modules,
        "rpcs": rpcs,
        "describe": describe,
    }


def _print_shell_banner() -> None:
    entry = get_most_recent(alive_only=True)
    run = (
        f"run {entry.run_id} ({entry.blueprint})"
        if entry is not None
        else "unregistered coordinator"
    )
    console = Console(highlight=False)
    console.print(Text(theme.ascii_logo.rstrip(), style=theme.ACCENT), soft_wrap=True)
    console.print(Text("RPC shell", style=f"bold {theme.ACCENT}"))
    console.print(f"Attached to {run}.")
    console.print()
    console.print(QUICK_START)
    console.print(
        Text(
            "WARNING: RPC calls execute immediately against the running system.",
            style=theme.WARNING,
        )
    )


def shell() -> None:
    """Open an IPython shell attached to the running DimOS coordinator."""
    if not _is_interactive_terminal():
        typer.echo(
            "Error: `dimos shell` requires an interactive terminal. "
            "Use the Dimos Python interface for automation.",
            err=True,
        )
        raise typer.Exit(1)

    try:
        with Console().status("Waiting for the DimOS coordinator...", spinner="dots"):
            app = Dimos.connect()
    except RuntimeError as exc:
        typer.echo(f"Error: {exc}", err=True)
        raise typer.Exit(1) from exc

    # Import here, not at the top: IPython pulls ~1700 modules (~1.3s) and must
    # load only when the shell actually opens, not on every CLI startup.
    from IPython import start_ipython

    try:
        _print_shell_banner()
        start_ipython(  # type: ignore[no-untyped-call]
            argv=["--no-banner", "--no-tip"],
            user_ns=_shell_namespace(app),
        )
    finally:
        app.stop()
