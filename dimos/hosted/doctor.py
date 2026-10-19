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

"""Health checks for this machine's Host setup, in the order ``--fix`` should apply them."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
import getpass
import re
import socket
import subprocess
import time

from dimos.hosted import machine
from dimos.hosted.config import HostConfig
from dimos.hosted.discovery import probe
from dimos.protocol.service.zenohservice import split_csv


@dataclass(frozen=True, slots=True)
class Check:
    description: str
    # None when fine, else what is wrong.
    check: Callable[[], str | None]
    fix: Callable[[], None] | None = None
    # A failing warning is reported but does not fail the doctor run.
    warning: bool = False


@dataclass(frozen=True, slots=True)
class Result:
    description: str
    ok: bool
    error: str | None = None
    # None: not attempted; otherwise what the fix did.
    fix_note: str | None = None
    warning: bool = False


def _config() -> str | None:
    HostConfig()
    return None


def _identity() -> str | None:
    return None if machine.load_host_id() else "empty Host identity"


def port_free(host: str, port: int) -> bool:
    bind_host = host.strip("[]") or "0.0.0.0"
    with socket.socket(socket.AF_INET6 if ":" in bind_host else socket.AF_INET) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((bind_host, port))
        except OSError:
            return False
    return True


def _port() -> str | None:
    """The daemon is the machine's router on its port; anything else there blocks it."""
    host, _, port = split_csv(HostConfig().listen)[0].partition("/")[2].rpartition(":")
    local = machine.local_host()
    if local is not None and any(e.endswith(f":{port}") for e in local.listen):
        return None
    if port_free(host, int(port)):
        return None
    out = subprocess.run(
        ["ss", "-ltnpH", f"sport = :{port}"], capture_output=True, text=True
    ).stdout
    owner = re.search(r'users:\(\("([^"]+)",pid=(\d+)', out)
    who = f"{owner.group(1)} (pid {owner.group(2)})" if owner else "another user's process"
    return f"port {port} is held by {who}, not a dimos Host"


def _revision() -> str | None:
    """Hosts match on the commit, plus the digest of the uncommitted diff, so clean is best."""
    revision = machine.code_revision()
    if re.fullmatch(r"[0-9a-f]{40}", revision):
        return None
    if "+" in revision:
        return f"uncommitted changes ({revision[-8:]}); every Host needs the same ones"
    return f"not a commit: {revision}"


def _unit_installed() -> str | None:
    return None if machine.installed() and machine.enabled() else "not installed"


def _unit_current() -> str | None:
    return None if not machine.installed() or machine.unit_current() else "stale unit file"


def _linger() -> str | None:
    if not machine.installed():
        return None
    user = getpass.getuser()
    out = subprocess.run(
        ["loginctl", "show-user", user, "-p", "Linger"], capture_output=True, text=True
    ).stdout
    return None if "Linger=yes" in out else f"run: sudo loginctl enable-linger {user}"


def _running() -> str | None:
    local = machine.local_host()
    if local is None:
        return "no local Host"
    return None if probe(local.client_endpoint) is not None else "the local Host does not answer"


def _running_revision() -> str | None:
    local = machine.local_host()
    found = probe(local.client_endpoint) if local else None
    if local is None or found is None:
        return None
    mine = [h for h in found.hosts if h.host_id == local.host_id]
    revision = machine.code_revision()
    if mine and mine[0].revision == revision:
        return None
    return (
        f"Host serves {mine[0].revision[:10] if mine else 'unknown'}, checkout is {revision[:10]}"
    )


def _wait(check: Callable[[], str | None], timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while check() is not None and time.monotonic() < deadline:
        time.sleep(1.0)


def _install() -> None:
    machine.install()


def _start() -> None:
    machine.start()
    _wait(_running)


def _restart() -> None:
    machine.restart()
    _wait(_running_revision)


CHECKS: tuple[Check, ...] = (
    Check("Host config (HOST__* in the environment or .env) is valid", _config),
    Check("Host identity is readable", _identity),
    Check("The configured Host port is free or held by this Host", _port),
    Check("Code revision is a clean git commit, so Hosts can match on it", _revision, warning=True),
    Check("The dimos-host systemd user unit is installed and enabled", _unit_installed, _install),
    Check("The installed systemd unit matches this checkout", _unit_current, _install),
    Check("Lingering is on, so the unit starts at boot without a login", _linger, warning=True),
    Check("The local Host is running and answers on its router", _running, _start),
    Check("The running Host serves this checkout's code revision", _running_revision, _restart),
)


def _run_check(check: Check) -> str | None:
    try:
        return check.check()
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"


def run(checks: Iterable[Check] = CHECKS, fix: bool = False) -> list[Result]:
    """Run every check; with ``fix``, fix the failing ones that can and check again."""
    results = []
    for check in checks:
        error = _run_check(check)
        note = None
        if error is not None and fix:
            if check.fix is None:
                note = "no automatic fix"
            else:
                try:
                    check.fix()
                    note = "fixed"
                except Exception as exc:
                    note = f"fix failed: {type(exc).__name__}: {exc}"
                error = _run_check(check)
        results.append(Result(check.description, error is None, error, note, check.warning))
    return results
