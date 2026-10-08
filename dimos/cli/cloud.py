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

Device-code flow (RFC 8628 shaped) against api.dimensional.org — built for robots:
no browser or clipboard needed on this machine. The CLI prints an 8-character code,
you approve it from any signed-in browser (laptop, phone), and the minted API key is
stored in the system keyring, falling back to a plain-text 0600 file
(`CREDENTIALS_PATH`, just the key) on headless machines with no keyring
backend. `DIMOS_API_KEY` (via GlobalConfig) overrides any stored login.
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
    """DIMENSIONAL wordmark when the terminal fits it, the compact DIMOS mark below."""
    from dimos.cli import theme

    wide = [ln for ln in theme.ascii_logo.splitlines() if ln.strip()]
    lines = wide if _console.width >= max(map(len, wide)) else _LOGO.splitlines()
    art = Text()
    for line, color in zip(lines, _GRADIENT, strict=True):
        art.append(line + "\n", style=f"bold {color}")
    return art


def _base() -> str:
    return global_config.dimos_cloud_url.rstrip("/")


def _post(path: str, **params: str | int) -> dict[str, Any]:
    url = f"{_base()}{path}?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(
        urllib.request.Request(url, method="POST"), timeout=global_config.dimos_http_timeout
    ) as r:
        return cast("dict[str, Any]", json.load(r))


def _keyring() -> ModuleType | None:
    """The OS keyring, or None on machines without a usable backend (headless robots)."""
    try:
        import keyring

        keyring.get_password(_KEYRING_SERVICE, "probe")
        return keyring
    except Exception:
        return None


def _store(key: str) -> str:
    """Persist the API key; returns a human-readable location for the login message."""
    if kr := _keyring():
        kr.set_password(_KEYRING_SERVICE, _KEYRING_USER, key)
        return "system keyring"
    CREDENTIALS_PATH.parent.mkdir(parents=True, exist_ok=True)
    # No keyring backend (typical on robots): owner-only file, the same convention
    # gh / aws / kubectl use for exactly this situation.
    fd = os.open(CREDENTIALS_PATH, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key + "\n")
    return str(CREDENTIALS_PATH)


def _load() -> str | None:
    if kr := _keyring():
        if key := kr.get_password(_KEYRING_SERVICE, _KEYRING_USER):
            return cast("str", key)
    try:
        return CREDENTIALS_PATH.read_text().strip() or None
    except OSError:
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
    return _load()


def _key_owner(key: str) -> dict[str, Any] | None:
    """Who the key belongs to, or None if it is invalid or the cloud is unreachable."""
    req = urllib.request.Request(
        f"{_base()}/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    try:
        with urllib.request.urlopen(req, timeout=global_config.dimos_http_timeout) as r:
            return cast("dict[str, Any]", json.load(r))
    except (urllib.error.URLError, TimeoutError, OSError):
        return None


def login() -> None:
    """Sign this machine in to Dimensional cloud."""
    if (key := api_key()) and (who := _key_owner(key)):
        _console.print(
            f"[bold green]✔ Already logged in[/] as [bold]{who['email']}[/] "
            "[dim]— `dimos logout` first to switch accounts[/]"
        )
        return
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
                where = _store(r["api_key"])
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
    """Forget the stored key. The key itself stays valid until revoked in the console."""
    if _forget():
        _console.print(
            "[bold cyan]⏻[/] [bold]Logged out.[/]"
            " [dim]The key stays valid until you revoke it in the console.[/]"
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
        with urllib.request.urlopen(req, timeout=global_config.dimos_http_timeout) as r:
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
