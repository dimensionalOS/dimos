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

import typer

from dimos.constants import CREDENTIALS_PATH
from dimos.core.global_config import global_config

_KEYRING_SERVICE = "dimos-cloud"
_KEYRING_USER = "default"


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


def login() -> None:
    """Sign this machine in to Dimensional cloud."""
    d = _post("/auth/device", label=socket.gethostname())
    typer.echo(f"\n  Open        {d['verification_uri']}")
    typer.echo(f"  Enter code  {d['user_code']}\n")
    deadline = time.time() + d["expires_in"]
    while time.time() < deadline:
        time.sleep(d["interval"])
        r = _post("/auth/token", device_code=d["device_code"])
        if r["status"] == "ok":
            where = _store(r["api_key"])
            typer.echo(f"Logged in as {r['email']} (key {r['key_id']}…, stored in {where})")
            return
        if r["status"] in ("denied", "expired"):
            typer.echo(f"Login {r['status']}.", err=True)
            raise typer.Exit(1)
    typer.echo("Login timed out.", err=True)
    raise typer.Exit(1)


def logout() -> None:
    """Forget the stored key. The key itself stays valid until revoked in the console."""
    if _forget():
        typer.echo("Logged out. The key stays valid until you revoke it in the console.")
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
    typer.echo(f"{who['email']} (scopes: {who['scopes']})")
