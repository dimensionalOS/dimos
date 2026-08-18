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

"""Dimensional cloud auth: `dimos login` / `logout` / `whoami`.

Device-code flow (RFC 8628 shaped) against login.dimensional.org — built for robots:
no browser or clipboard needed on this machine. The CLI prints an 8-character code,
you approve it from any signed-in browser (laptop, phone), and the minted API key
lands in the XDG config dir. `DIMOS_API_KEY` overrides the stored key everywhere.
"""

import json
import os
import socket
import stat
import time
import urllib.error
import urllib.parse
import urllib.request

import typer

from dimos.constants import CONFIG_DIR

BASE = os.environ.get("DIMOS_CLOUD_URL", "https://login.dimensional.org")
CRED_PATH = CONFIG_DIR / "dimos" / "credentials.json"

app = typer.Typer(help="Dimensional cloud account")


def _post(path: str, **params: str | int) -> dict:
    url = f"{BASE}{path}?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(urllib.request.Request(url, method="POST")) as r:
        return json.load(r)


def api_key() -> str | None:
    """The credential for cloud calls: env var first, then the stored login."""
    if key := os.environ.get("DIMOS_API_KEY"):
        return key
    try:
        return json.loads(CRED_PATH.read_text())["api_key"]
    except (OSError, KeyError, ValueError):
        return None


@app.command()
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
            CRED_PATH.parent.mkdir(parents=True, exist_ok=True)
            CRED_PATH.write_text(json.dumps({"api_key": r["api_key"], "email": r["email"]}))
            CRED_PATH.chmod(stat.S_IRUSR | stat.S_IWUSR)
            typer.echo(f"Logged in as {r['email']} (key {r['key_id']}…)")
            return
        if r["status"] in ("denied", "expired"):
            typer.echo(f"Login {r['status']}.", err=True)
            raise typer.Exit(1)
    typer.echo("Login timed out.", err=True)
    raise typer.Exit(1)


@app.command()
def logout() -> None:
    """Forget the stored key. Revoke it fully at login.dimensional.org/keys."""
    if CRED_PATH.exists():
        CRED_PATH.unlink()
        typer.echo(f"Removed {CRED_PATH}. Revoke the key at {BASE}/keys.")
    else:
        typer.echo("Not logged in.")


@app.command()
def whoami() -> None:
    """Show which account this machine's key belongs to."""
    key = api_key()
    if not key:
        typer.echo("Not logged in — run `dimos login`.", err=True)
        raise typer.Exit(1)
    req = urllib.request.Request(f"{BASE}/auth/whoami", headers={"Authorization": f"Bearer {key}"})
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
    typer.echo(f"{who['email']} (scopes: {who['scopes']})")
