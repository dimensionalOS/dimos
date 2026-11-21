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
backend. `DIMOS_CLOUD_API_KEY` overrides any stored login.
"""

from collections.abc import Callable
import json
import os
import socket
import threading
import time
from types import ModuleType
from typing import Any, TypeVar, cast
import urllib.error
import urllib.parse
import urllib.request

import typer

from dimos.cloud.config import CloudConfig
from dimos.constants import CREDENTIALS_PATH
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_KEYRING_SERVICE = "dimos-cloud"
_KEYRING_USER = "default"

T = TypeVar("T")

# The keyring backend, memoized: the module when it answers, None when it can't be used.
# A backend that stalls is re-probed after KEYRING_RETRY_S, so a login stored after this
# process started is still found, without paying the stall on every single lookup.
KEYRING_RETRY_S = 60.0
_backend: tuple[float, ModuleType | None] | None = None
_backend_lock = threading.Lock()


class _KeyringStalledError(Exception):
    """The OS keyring did not answer within CloudConfig().keyring_timeout."""


def _base() -> str:
    return CloudConfig().url.rstrip("/")


def _post(path: str, **params: str | int) -> dict[str, Any]:
    url = f"{_base()}{path}?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(
        urllib.request.Request(url, method="POST"), timeout=CloudConfig().http_timeout
    ) as r:
        return cast("dict[str, Any]", json.load(r))


def _bounded(what: str, call: Callable[[], T]) -> T:
    """Run one keyring call with a deadline, on a thread we are willing to abandon.

    The OS keyring answers over IPC (D-Bus SecretService on Linux). A *locked* collection
    answers an ordinary read with an unlock **prompt** and then waits for whoever is at the
    screen — and in a service with nobody there, that wait never ends. The call blocks inside
    the socket, where no flag or signal of ours can reach it, so the only way to put a
    deadline on it is to stop waiting for the thread that made it. One stalled lookup used to
    take the whole gateway down with it: every poll of the cloud account queued another
    thread behind the same prompt until the pool was gone, and the upload worker went with it.

    The abandoned thread leaks until the prompt is answered or the process ends; memoizing the
    stall keeps that to one thread per KEYRING_RETRY_S rather than one per request.
    """
    out: list[T] = []
    failed: list[BaseException] = []

    def run() -> None:
        try:
            out.append(call())
        except BaseException as error:  # re-raised on the calling thread
            failed.append(error)

    worker = threading.Thread(target=run, name=f"keyring-{what}", daemon=True)
    worker.start()
    worker.join(CloudConfig().keyring_timeout)
    if failed:
        raise failed[0]
    if not out:
        raise _KeyringStalledError(what)
    return out[0]


def _import_keyring() -> ModuleType:
    import keyring

    return keyring


def _probe() -> ModuleType | None:
    try:
        kr = _import_keyring()
        _bounded("probe", lambda: kr.get_password(_KEYRING_SERVICE, "probe"))
        return kr
    except _KeyringStalledError:
        logger.warning(
            "the system keyring didn't answer, so it is being skipped: is it locked, "
            "waiting to be unlocked by someone at the screen?",
            timeout_s=CloudConfig().keyring_timeout,
            falling_back_to=str(CREDENTIALS_PATH),
        )
        return None
    except Exception:
        return None


def _keyring() -> ModuleType | None:
    """The OS keyring, or None on machines without a usable backend (headless robots)."""
    global _backend
    with _backend_lock:
        if _backend is None or (
            _backend[1] is None and time.monotonic() - _backend[0] > KEYRING_RETRY_S
        ):
            _backend = (time.monotonic(), _probe())
        return _backend[1]


def _forget_backend() -> None:
    """Skip the keyring until the next retry window: it stalled on a real lookup."""
    global _backend
    with _backend_lock:
        _backend = (time.monotonic(), None)


def _keyring_call(what: str, call: Callable[[ModuleType], T]) -> T | None:
    """One keyring operation, or None when there is no usable keyring to do it with."""
    kr = _keyring()
    if kr is None:
        return None
    try:
        return _bounded(what, lambda: call(kr))
    except _KeyringStalledError:
        _forget_backend()
        return None


def _store(key: str) -> str:
    """Persist the API key; returns a human-readable location for the login message."""
    if _keyring_call(
        "set", lambda kr: kr.set_password(_KEYRING_SERVICE, _KEYRING_USER, key) or True
    ):
        return "system keyring"
    CREDENTIALS_PATH.parent.mkdir(parents=True, exist_ok=True)
    # No keyring backend (typical on robots): owner-only file, the same convention
    # gh / aws / kubectl use for exactly this situation.
    fd = os.open(CREDENTIALS_PATH, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key + "\n")
    return str(CREDENTIALS_PATH)


def _load() -> str | None:
    if key := _keyring_call("get", lambda kr: kr.get_password(_KEYRING_SERVICE, _KEYRING_USER)):
        return cast("str", key)
    try:
        return CREDENTIALS_PATH.read_text().strip() or None
    except OSError:
        return None


def _forget() -> bool:
    found = False
    if _keyring_call("get", lambda kr: kr.get_password(_KEYRING_SERVICE, _KEYRING_USER)):
        _keyring_call("delete", lambda kr: kr.delete_password(_KEYRING_SERVICE, _KEYRING_USER))
        found = True
    if CREDENTIALS_PATH.exists():
        CREDENTIALS_PATH.unlink()
        found = True
    return found


def api_key(config: CloudConfig | None = None) -> str | None:
    """The configured cloud credential, falling back to the stored login."""
    settings = config if config is not None else CloudConfig()
    return settings.api_key or _load()


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
        with urllib.request.urlopen(req, timeout=CloudConfig().http_timeout) as r:
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
