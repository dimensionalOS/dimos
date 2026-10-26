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

"""`dimos agent` commands: the on-robot agent."""

from __future__ import annotations

import fcntl
import getpass
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import NoReturn

import click
from dotenv import dotenv_values, find_dotenv
import typer
from typer.core import TyperCommand, TyperGroup

from dimos.constants import CONFIG_DIR, DIMOS_PROJECT_ROOT, STATE_DIR
from dimos.core.run_registry import REGISTRY_DIR

SERVICE = "dimos-agent.service"
FORWARD = "forward"
"""The hidden command for a command line that names no command, such as `--conversation 3`."""


class _NodeFallback(TyperGroup):
    """Sends a command line that names none of this group's commands to FORWARD, unparsed."""

    def resolve_command(
        self, ctx: click.Context, args: list[str]
    ) -> tuple[str | None, click.Command | None, list[str]]:
        if args[0] in self.commands:
            return super().resolve_command(ctx, args)
        return FORWARD, self.commands[FORWARD], args


class _Verbatim(TyperCommand):
    """Parses nothing, so the Node CLI receives every argument, `--` and `--help` included."""

    def parse_args(self, ctx: click.Context, args: list[str]) -> list[str]:
        ctx.args = list(args)
        return []


agent_app = typer.Typer(
    cls=_NodeFallback,
    invoke_without_command=True,
    context_settings={"ignore_unknown_options": True, "allow_extra_args": True},
)
service_app = typer.Typer(
    help=f"The systemd user service {SERVICE}, which runs `dimos agent gateway`."
)
agent_app.add_typer(service_app, name="service")


@agent_app.callback()
def agent_main(ctx: typer.Context) -> None:
    """The on-robot agent.

    With no command, opens a terminal on a new conversation, starting the gateway if needed.

    Every command but load and service runs in the Node CLI with the same arguments.
    """
    if ctx.invoked_subcommand is None:
        _exec_node([])


@agent_app.command(FORWARD, cls=_Verbatim, hidden=True)
def forward_cmd(ctx: typer.Context) -> None:
    _exec_node(ctx.args)


def _forwarded(name: str, help: str) -> None:
    def command(ctx: typer.Context) -> None:
        _exec_node([name, *ctx.args])

    agent_app.command(name, cls=_Verbatim, help=help)(command)


_forwarded("gateway", "Run the gateway in the foreground, or in the background with --daemon.")
_forwarded(
    "run", "Send PROMPT to a new conversation, or to --conversation ID, and print the answer."
)
_forwarded("stop", "Stop the gateway.")
_forwarded("status", "Show whether the gateway runs.")
_forwarded("conversations", "List the conversations, newest first.")
_forwarded("version", "Print the agent's version and build.")


@agent_app.command("load")
def load_cmd() -> None:
    """Ensure the relay bridge, HarnessRuntime and McpServer are running.

    Prints one JSON line: {"loaded": [...], "already": [...]}.
    """
    # The runtime imports the porcelain and module stack; keep `dimos --help` fast.
    from dimos.agents.harness.runtime.load import main

    raise typer.Exit(main())


NO_SYSTEMCTL = typer.Option(
    False, "--no-systemctl", help="Only touch the unit file; do not run systemctl."
)


@service_app.command("install")
def service_install(no_systemctl: bool = NO_SYSTEMCTL) -> None:
    """Write the unit, stop a gateway that `dimos agent` started, and enable the service now."""
    unit = _unit_path()
    if no_systemctl:
        _write_unit(unit)
        return
    stop = _node_argv(["stop"])
    _write_unit(unit)
    _systemctl("daemon-reload")
    if subprocess.run(stop, env=_node_environment()).returncode != 0:
        _fail("`dimos agent stop` failed, so the service could not take over the gateway's socket.")
    _systemctl("enable", "--now", SERVICE)
    if not _lingers():
        user = getpass.getuser()
        typer.echo(
            f"{SERVICE} runs while {user} is logged in. To run it from boot, run: loginctl enable-linger {user}"
        )


@service_app.command("status")
def service_status(no_systemctl: bool = NO_SYSTEMCTL) -> None:
    """Show the unit file, whether it is current, and the service's state."""
    unit = _unit_path()
    if not unit.exists():
        typer.echo(f"{SERVICE} is not installed ({unit} does not exist).")
        raise typer.Exit(1)
    if unit.read_text() == _unit_text():
        typer.echo(f"{unit} is current.")
    else:
        typer.echo(f"{unit} differs from this checkout's unit. Run `dimos agent service install`.")
    if no_systemctl:
        return
    if not _lingers():
        typer.echo(f"{SERVICE} runs only while {getpass.getuser()} is logged in.")
    raise typer.Exit(
        subprocess.run(["systemctl", "--user", "status", "--no-pager", SERVICE]).returncode
    )


@service_app.command("uninstall")
def service_uninstall(no_systemctl: bool = NO_SYSTEMCTL) -> None:
    """Stop and disable the service and remove its unit."""
    unit = _unit_path()
    if not unit.exists():
        typer.echo(f"{SERVICE} is not installed.")
        return
    if not no_systemctl:
        _systemctl("disable", "--now", SERVICE)
    unit.unlink()
    typer.echo(f"Removed {unit}")
    if not no_systemctl:
        _systemctl("daemon-reload")


def _exec_node(args: list[str]) -> NoReturn:
    argv = _node_argv(args)
    os.execve(argv[0], argv, _node_environment())


def _node_argv(args: list[str]) -> list[str]:
    """`node <checkout>/dimos/agents/dist/cli.js ARGS`, once Node and the build can run it.

    First installs and builds whatever changed, except for gateway, stop and status, which run the
    existing build: the service starts the gateway at boot, where nothing should download
    packages, and stop and status must work while a gateway runs.
    """
    package = DIMOS_PROJECT_ROOT / "dimos" / "agents"
    if not (package / "package.json").exists():
        _fail(f"dimos agent runs from a DimOS source checkout, and {package} has no package.json.")
    node = shutil.which("node")
    if node is None:
        _fail("dimos agent needs Node.js 24 or 26 on PATH, and there is no `node` on PATH.")
    version = subprocess.run(
        [node, "--version"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if version.lstrip("v").split(".")[0] not in ("24", "26"):
        _fail(
            f"dimos agent needs Node.js 24 or 26, and {node} is {version}. Put Node 24 or 26 first on PATH."
        )
    cli = package / "dist" / "cli.js"
    if not args or args[0] not in ("gateway", "stop", "status"):
        _build(package, node)
    else:
        if args[0] == "gateway":
            _wait_for_build(package)
        if not cli.exists() or not (package / "node_modules").is_dir():
            _fail("The agent is not built. Run: dimos agent version")
    return [node, str(cli), *args]


def _build(package: Path, node: str) -> None:
    """`npm ci` when package.json or package-lock.json changed since the last install, and
    `npm run build` when a file the last build read changed. One launch builds at a time."""
    if _installed(package) and _built(package):
        return
    try:
        (package / "dist").mkdir(exist_ok=True)
        with (package / "dist" / ".build.lock").open("w") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                typer.echo("Waiting for another `dimos agent` to finish building.", err=True)
                fcntl.flock(lock, fcntl.LOCK_EX)
            environment = _node_environment()
            if not _installed(package):
                _refuse_while_serving(package, node, environment)
                _install(package, node, environment)
            if not _built(package):
                _npm(node, ["run", "build"], package, environment)
    except OSError as error:
        _fail(f"Could not build the agent in {package}: {error}")


def _wait_for_build(package: Path) -> None:
    """Waits out a build in progress, so a gateway never starts on a half-installed node_modules."""
    try:
        with (package / "dist" / ".build.lock").open() as lock:
            fcntl.flock(lock, fcntl.LOCK_SH)
    except OSError:
        pass


def _dependencies(package: Path) -> str:
    files = (package / "package.json", package / "package-lock.json")
    return hashlib.sha256(b"\0".join(file.read_bytes() for file in files)).hexdigest()


def _installed(package: Path) -> bool:
    stamp = package / "node_modules" / ".dimos-stamp"
    return stamp.is_file() and stamp.read_text() == _dependencies(package)


def _built(package: Path) -> bool:
    """Whether every file the last build read, as build.mjs recorded in dist/inputs.json, is
    unchanged."""
    try:
        read = json.loads((package / "dist" / "inputs.json").read_text())
        return (
            isinstance(read, dict)
            and (package / "dist" / "cli.js").is_file()
            and all(
                hashlib.sha256((package / name).read_bytes()).hexdigest() == digest
                for name, digest in read.items()
            )
        )
    except (FileNotFoundError, ValueError):
        return False


def _install(package: Path, node: str, environment: dict[str, str]) -> None:
    """`npm ci`, keeping the previous node_modules until it succeeds: npm empties node_modules
    before it downloads anything, and a robot is often offline."""
    modules = package / "node_modules"
    previous = package / "dist" / "node_modules.previous"
    shutil.rmtree(previous, ignore_errors=True)
    if modules.is_dir():
        modules.rename(previous)
    try:
        _npm(node, ["ci"], package, environment)
        (modules / ".dimos-stamp").write_text(_dependencies(package))
    except BaseException:
        shutil.rmtree(modules, ignore_errors=True)
        if previous.is_dir():
            previous.rename(modules)
        raise
    shutil.rmtree(previous, ignore_errors=True)


def _refuse_while_serving(package: Path, node: str, environment: dict[str, str]) -> None:
    """Fails while a gateway runs this checkout's agent, whose node_modules `npm ci` would
    replace. A gateway of another checkout imports its own, and a build that cannot run `status`
    serves nothing."""
    try:
        status = subprocess.run(
            [node, str(package / "dist" / "cli.js"), "status", "--json"],
            capture_output=True,
            text=True,
            env=environment,
            timeout=20,
        )
        state = json.loads(status.stdout)
    except subprocess.TimeoutExpired:
        state = None
    except ValueError:
        return
    match state:
        case {"kind": "stopped" | "stale_socket"}:
            return
        case {"process": {"pid": pid, "build": {"executable": str(executable)}}}:
            if not Path(executable).resolve().is_relative_to(package.resolve()):
                return
            reason = f"The gateway (pid {pid}) runs this checkout's agent"
        case _:
            reason = "A gateway may be running this checkout's agent"
    _fail(
        f"{reason}, and installing would replace the packages it loads. Stop it with "
        "`dimos agent stop`, then run `dimos agent version`. As the service: systemctl --user stop "
        f"{SERVICE} && dimos agent version && systemctl --user start {SERVICE}"
    )


def _npm(node: str, args: list[str], package: Path, environment: dict[str, str]) -> None:
    """Runs the npm installed with `node`, with its output on stderr, since stdout carries what
    the command prints."""
    npm = shutil.which("npm", path=str(Path(node).parent))
    if npm is None:
        _fail(f"dimos agent builds itself with the npm installed with {node}, and there is none.")
    command = f"npm {' '.join(args)}"
    typer.echo(f"Running `{command}` in {package}", err=True)
    if subprocess.run([npm, *args], cwd=package, env=environment, stdout=sys.stderr).returncode:
        _fail(f"`{command}` failed.")


def _node_environment() -> dict[str, str]:
    """This process's environment, plus where DimOS is. Of what `dimos` loaded from .env,
    only GlobalConfig settings stay, such as the transport and the robot's address.

    bash inherits the gateway's environment, and .env holds API keys.
    """
    # The resolver imports the module stack; keep `dimos --help` fast.
    from dimos.agents.harness.runtime.load import mcp_server_url
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.sources import global_environment_names

    dotenv = dotenv_values(find_dotenv())
    settings = global_environment_names()
    environment = {
        name: value
        for name, value in os.environ.items()
        if dotenv.get(name) != value or name.lower() in settings
    }
    try:
        mcp_url = mcp_server_url()
    except BlueprintConfigError as error:
        _fail(str(error))
    environment.update(
        DIMOS_ROOT=str(DIMOS_PROJECT_ROOT),
        DIMOS_PYTHON=sys.executable,
        DIMOS_CLI=str(_dimos_executable()),
        DIMOS_MCP_URL=mcp_url,
        DIMOS_RUNS_DIR=str(REGISTRY_DIR),
        DIMOS_STATE_DIR=str(STATE_DIR),
    )
    return environment


def _dimos_executable() -> Path:
    """The `dimos` script beside this interpreter, unresolved so that a venv stays a venv."""
    return Path(sys.executable).parent / "dimos"


def _unit_path() -> Path:
    return CONFIG_DIR.parent / "systemd" / "user" / SERVICE


def _unit_text() -> str:
    """`dimos agent gateway` from this checkout, with the installing shell's PATH and XDG dirs."""
    captured = ["PATH", "XDG_CONFIG_HOME", "XDG_STATE_HOME", "XDG_CACHE_HOME"]
    environment = [
        f"Environment={_quoted(f'{name}={os.environ[name]}')}"
        for name in captured
        if name in os.environ
    ]
    executable = _quoted(str(_dimos_executable())).replace("$", "$$")
    return "\n".join(
        [
            "[Unit]",
            "Description=DimOS agent gateway",
            "",
            "[Service]",
            "Type=simple",
            f"WorkingDirectory={str(DIMOS_PROJECT_ROOT).replace('%', '%%')}",
            *environment,
            f"ExecStart={executable} agent gateway",
            "Restart=on-failure",
            "RestartSec=2",
            "UMask=0077",
            "",
            "[Install]",
            "WantedBy=default.target",
            "",
        ]
    )


def _quoted(value: str) -> str:
    """A double-quoted unit-file value that systemd reads back unchanged."""
    return '"' + value.replace("%", "%%").replace("\\", "\\\\").replace('"', '\\"') + '"'


def _write_unit(unit: Path) -> None:
    unit.parent.mkdir(parents=True, exist_ok=True)
    unit.write_text(_unit_text())
    typer.echo(f"Wrote {unit}")


def _systemctl(*args: str) -> None:
    if subprocess.run(["systemctl", "--user", *args]).returncode != 0:
        _fail(f"`systemctl --user {' '.join(args)}` failed.")


def _lingers() -> bool:
    """Whether systemd keeps this user's services running without a login session."""
    return (Path("/var/lib/systemd/linger") / getpass.getuser()).exists()


def _fail(message: str) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(1)
