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


"""Dimensional cloud for the dimos gateway, run in a child process (`python -m dimos.gateway.cloud_worker ...`) so a
cancel is a kill. It only calls dimos's own code (dimos.cli.cloud's key store and endpoints for the login,
dimos.cloud.data for uploads), unmodified: what `dimos login` prints and waits for, it does step by step here.
Every result is a line MARKER + JSON on stdout (importing dimos can print); a traceback goes to stderr.

    account                         -> {"loggedIn", "email", "scopes", "source", "cloudUrl", "error"}
    login                           -> {"event": "code", url, urlComplete, code, expiresIn, interval}
                                       ... then {"event": "done", "status": "ok"|"denied"|"expired", "email"}
    logout                          -> {"loggedOut": bool}
    upload <path> [robot_id] [kind] -> {"event": "progress", phase, done, total} ...
                                       then {"event": "result", uploadId, state, skipped, quota, link}
    any failure                     -> {"event": "error", "code": not_logged_in|network|quota|file_missing|failed, "message"}
"""

from __future__ import annotations

from collections.abc import Iterator
import json
import signal
import sys
import time
import traceback
from typing import Any

# the same answer marker as the gateway's other child (introspect.py)
from dimos.gateway.introspect import MARKER


def emit(value: dict[str, Any]) -> None:
    sys.stdout.write("\n" + MARKER + json.dumps(value, default=str) + "\n")
    sys.stdout.flush()


def causes(error: BaseException | None) -> Iterator[BaseException]:
    while error is not None:
        yield error
        error = error.__cause__ or error.__context__


def http_status(chain: list[BaseException]) -> int | None:
    import urllib.error

    return next((e.code for e in chain if isinstance(e, urllib.error.HTTPError)), None)


class NotLoggedInError(RuntimeError):
    """No key: checked here before any cloud call, so it's never read from dimos's wording."""


def classify(error: BaseException) -> tuple[str, str]:
    """(code, readable message) for an exception from dimos's cloud code, by its type and HTTP status (never its
    wording, which dimos is free to change)."""
    import urllib.error

    text = str(error)
    chain = list(causes(error))
    status = http_status(chain)
    if isinstance(error, FileNotFoundError):
        return "file_missing", f"The file is gone: {error.filename or text}"
    if any(isinstance(e, NotLoggedInError) for e in chain) or status == 401:
        return (
            "not_logged_in",
            "Not logged in to Dimensional cloud (or the login was revoked). Log in, then retry.",
        )
    if status in (402, 413, 507):
        return "quota", f"Your Dimensional cloud storage quota doesn't allow this upload: {text}"
    if any(
        isinstance(e, (TimeoutError, ConnectionError))
        or (isinstance(e, urllib.error.URLError) and not isinstance(e, urllib.error.HTTPError))
        for e in chain
    ):
        return (
            "network",
            f"Couldn't reach Dimensional cloud (check the network, then retry): {text}",
        )
    return "failed", text or type(error).__name__


def account() -> dict[str, Any]:
    import urllib.error

    from dimos.cli import cloud
    from dimos.core.global_config import global_config

    key = cloud.api_key()
    result: dict[str, Any] = {
        "loggedIn": False,
        "email": None,
        "scopes": None,
        "source": None,
        "cloudUrl": global_config.dimos_cloud_url.rstrip("/"),
        "error": None,
    }
    if not key:
        return result
    result["source"] = "env" if global_config.dimos_api_key else "stored"
    try:
        who = whoami(key)
        result.update(loggedIn=True, email=who.get("email"), scopes=who.get("scopes"))
    except urllib.error.HTTPError as error:
        if error.code == 401:
            result["error"] = "The saved login was revoked or is invalid: log in again."
        else:
            result.update(loggedIn=True, error=f"Dimensional cloud answered {error.code}")
    except Exception as error:
        # a key is there; we just can't check it now
        result.update(loggedIn=True, error=f"Couldn't reach Dimensional cloud: {error}")
    return result


def whoami(key: str) -> dict[str, Any]:
    """The account `key` belongs to (`email`, `scopes`); urllib's HTTPError on a refusal (401: invalid or revoked).
    The request `dimos whoami` makes."""
    import urllib.request

    from dimos.cli import cloud
    from dimos.core.global_config import global_config

    request = urllib.request.Request(
        f"{cloud._base()}/auth/whoami", headers={"Authorization": f"Bearer {key}"}
    )
    with urllib.request.urlopen(request, timeout=global_config.dimos_http_timeout) as response:
        answer: dict[str, Any] = json.load(response)
        return answer


def login() -> dict[str, Any]:
    """`dimos login`'s device flow, through its own endpoints and key store, with the code emitted instead of printed."""
    import socket

    from dimos.cli import cloud

    device = cloud._post("/auth/device", label=socket.gethostname())
    emit(
        {
            "event": "code",
            "url": device["verification_uri"],
            "urlComplete": device.get("verification_uri_complete"),
            "code": device["user_code"],
            "expiresIn": device["expires_in"],
            "interval": device["interval"],
        }
    )
    deadline = time.time() + device["expires_in"]
    while time.time() < deadline:
        time.sleep(device["interval"])
        answer = cloud._post("/auth/token", device_code=device["device_code"])
        if answer["status"] == "ok":
            cloud._store(answer["api_key"])
            return {"event": "done", "status": "ok", "email": answer.get("email")}
        if answer["status"] in ("denied", "expired"):
            return {"event": "done", "status": answer["status"], "email": None}
    return {"event": "done", "status": "expired", "email": None}


def logout() -> dict[str, Any]:
    from dimos.cli import cloud

    return {"loggedOut": cloud._forget()}


def console_datasets_url() -> str | None:
    """The web console's page listing the account's datasets: dimos_cloud_url's api.X -> console.X (None for a cloud
    URL without an api. host). The console has no per-dataset URL yet."""
    from dimos.core.global_config import global_config

    base = global_config.dimos_cloud_url.rstrip("/")
    if "://api." not in base:
        return None
    return base.replace("://api.", "://console.", 1) + "/console/data"


def upload(path: str, robot_id: str | None = None, kind: str | None = None) -> dict[str, Any]:
    from pathlib import Path

    from dimos.cli import cloud
    from dimos.cloud.data import CloudData

    last: list[Any] = [0.0, None]

    def tick(phase: str, done: int, total: int) -> None:
        # at most ~10 lines a second, but every phase change
        now = time.monotonic()
        if phase != last[1] or now - last[0] >= 0.1 or (total and done >= total):
            last[0], last[1] = now, phase
            emit({"event": "progress", "phase": phase, "done": done, "total": total})

    if not Path(path).is_file():
        raise FileNotFoundError(2, "No such file", path)
    if not cloud.api_key():
        raise NotLoggedInError("not logged in")
    tick("preparing", 0, 0)
    result = CloudData().upload(
        Path(path), robot_id=robot_id or None, kind=kind or None, progress=tick
    )
    return {
        "event": "result",
        "link": console_datasets_url() if result.get("upload_id") else None,
        "uploadId": result.get("upload_id"),
        "state": result.get("state"),
        "skipped": bool(result.get("skipped")),
        "quota": result.get("quota") or {},
    }


def main() -> None:
    # the gateway cancels with SIGTERM: unwind, so dimos's staging folder is removed
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    command, args = sys.argv[1], sys.argv[2:]
    try:
        if command == "account":
            result = account()
        elif command == "login":
            result = login()
        elif command == "logout":
            result = logout()
        elif command == "upload":
            result = upload(*args)
        else:
            result = {"event": "error", "code": "failed", "message": f"unknown command {command}"}
    except SystemExit:
        raise
    except BaseException as error:
        traceback.print_exc()
        code, message = classify(error)
        result = {"event": "error", "code": code, "message": message}
    emit(result)


if __name__ == "__main__":
    main()
