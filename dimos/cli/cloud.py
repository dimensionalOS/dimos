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

"""Dimensional cloud auth: `dimos login` / `dimos logout` / `dimos whoami`.

Device-code flow (RFC 8628 shaped) against login.dimensional.org — built for robots:
no browser or clipboard needed on this machine. The CLI prints an 8-character code,
you approve it from any signed-in browser (laptop, phone), and the minted API key is
stored in the system keyring, falling back to a 0600 file (`CREDENTIALS_PATH`) on
headless machines with no keyring backend. `DIMOS_API_KEY` (via GlobalConfig)
overrides any stored login.
"""

import json
import os
import socket
import time
from types import ModuleType
from typing import Any, cast
import urllib.error
import urllib.parse
import urllib.request

from rich.console import Console
from rich.panel import Panel
from rich.text import Text
import typer

from dimos.constants import CREDENTIALS_PATH
from dimos.core.global_config import global_config

_KEYRING_SERVICE = "dimos-cloud"
_KEYRING_USER = "default"

_console = Console(highlight=False)

_LOGO = """\
██████╗ ██╗███╗   ███╗ ██████╗ ███████╗
██╔══██╗██║████╗ ████║██╔═══██╗██╔════╝
██║  ██║██║██╔████╔██║██║   ██║███████╗
██║  ██║██║██║╚██╔╝██║██║   ██║╚════██║
██████╔╝██║██║ ╚═╝ ██║╚██████╔╝███████║
╚═════╝ ╚═╝╚═╝     ╚═╝ ╚═════╝ ╚══════╝"""

_GRADIENT = ["#00e5ff", "#00c4ff", "#3d9bff", "#5c7cff", "#7a5cff", "#9b40ff"]


def _logo() -> Text:
    art = Text()
    for line, color in zip(_LOGO.splitlines(), _GRADIENT, strict=True):
        art.append(line + "\n", style=f"bold {color}")
    return art


def _base() -> str:
    return global_config.dimos_cloud_url.rstrip("/")


def _post(path: str, **params: str | int) -> dict[str, Any]:
    url = f"{_base()}{path}?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(urllib.request.Request(url, method="POST")) as r:
        return cast("dict[str, Any]", json.load(r))


def _keyring() -> ModuleType | None:
    """The OS keyring, or None on machines without a usable backend (headless robots)."""
    try:
        import keyring

        keyring.get_password(_KEYRING_SERVICE, "probe")
        return keyring
    except Exception:
        return None


def _store(creds: dict[str, str]) -> str:
    """Persist credentials; returns a human-readable location for the login message."""
    blob = json.dumps(creds)
    if kr := _keyring():
        kr.set_password(_KEYRING_SERVICE, _KEYRING_USER, blob)
        return "system keyring"
    CREDENTIALS_PATH.parent.mkdir(parents=True, exist_ok=True)
    # No keyring backend (typical on robots): owner-only file, the same convention
    # gh / aws / kubectl use for exactly this situation.
    fd = os.open(CREDENTIALS_PATH, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(blob)
    return str(CREDENTIALS_PATH)


def _load() -> dict[str, str] | None:
    if kr := _keyring():
        if blob := kr.get_password(_KEYRING_SERVICE, _KEYRING_USER):
            return cast("dict[str, str]", json.loads(blob))
    try:
        return cast("dict[str, str]", json.loads(CREDENTIALS_PATH.read_text()))
    except (OSError, ValueError):
        return None


def _forget() -> bool:
    found = False
    if kr := _keyring():
        if kr.get_password(_KEYRING_SERVICE, _KEYRING_USER):
            kr.delete_password(_KEYRING_SERVICE, _KEYRING_USER)
            found = True
    if CREDENTIALS_PATH.exists():
        CREDENTIALS_PATH.unlink()
        found = True
    return found


def api_key() -> str | None:
    """The credential for cloud calls: DIMOS_API_KEY first, then the stored login."""
    if global_config.dimos_api_key:
        return global_config.dimos_api_key
    creds = _load()
    return creds.get("api_key") if creds else None


def login() -> None:
    """Sign this machine in to Dimensional cloud."""
    d = _post("/auth/device", label=socket.gethostname())
    _console.print(
        Panel(
            Text.assemble(
                ("Open        ", "dim"),
                (d["verification_uri"], "bold cyan underline"),
                "\n",
                ("Enter code  ", "dim"),
                (f" {d['user_code']} ", "bold black on cyan"),
            ),
            title="[bold]Dimensional Cloud[/]",
            subtitle="[dim]approve from any signed-in browser[/]",
            border_style="cyan",
            padding=(1, 3),
            expand=False,
        )
    )
    deadline = time.time() + d["expires_in"]
    with _console.status("", spinner="dots") as status:
        while time.time() < deadline:
            left = int(deadline - time.time())
            status.update(f"[cyan]Waiting for approval…[/] [dim]{left // 60}:{left % 60:02d}[/]")
            time.sleep(d["interval"])
            r = _post("/auth/token", device_code=d["device_code"])
            if r["status"] == "ok":
                where = _store({"api_key": r["api_key"], "email": r["email"]})
                _console.print(_logo())
                _console.print(
                    f"[bold green]✔ Logged in[/] as [bold]{r['email']}[/] "
                    f"[dim](key {r['key_id']}…, stored in {where})[/]"
                )
                return
            if r["status"] in ("denied", "expired"):
                typer.echo(f"Login {r['status']}.", err=True)
                raise typer.Exit(1)
    typer.echo("Login timed out.", err=True)
    raise typer.Exit(1)


def logout() -> None:
    """Forget the stored key. Revoke it fully at login.dimensional.org/keys."""
    if _forget():
        _console.print(
            f"[bold cyan]⏻[/] [bold]Logged out.[/] [dim]Revoke the key at {_base()}/keys.[/]"
        )
    else:
        typer.echo("Not logged in.")


def whoami() -> None:
    """Show which account this machine's key belongs to."""
    key = api_key()
    if not key:
        typer.echo("Not logged in — run `dimos login`.", err=True)
        raise typer.Exit(1)
    req = urllib.request.Request(
        f"{_base()}/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    try:
        with urllib.request.urlopen(req) as r:
            who = json.load(r)
    except urllib.error.HTTPError as e:
        typer.echo(
            "Key invalid or revoked — run `dimos login`."
            if e.code == 401
            else f"Cloud error: {e.code}",
            err=True,
        )
        raise typer.Exit(1) from e
    _console.print(f"[bold]{who['email']}[/] [dim](scopes: {who['scopes']})[/]")
