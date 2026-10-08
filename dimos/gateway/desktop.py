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

"""Running shell commands through dimOS Desktop (its `POST /api/desktop/shell`, Desktop's docs/shell.md): Desktop shows
them over the asking app with a note for each, nothing runs until the user presses Run, they run in one terminal (sudo
asks once), and when one fails the user or Desktop's agent fixes it there and retries it. The gateway finds Desktop at
$DIMOS_GATEWAY's `desktopUrl`, else at config.yaml's `desktop.port` on this machine."""

from __future__ import annotations

import asyncio
import json
from typing import Any
import urllib.error
import urllib.request

from dimos.gateway import config

DEFAULT_PORT = 5555
FINISHED = ("succeeded", "failed", "cancelled")


class DesktopUnavailableError(Exception):
    """Desktop doesn't answer (the gateway runs without it)."""


def desktop_url() -> str:
    given = str(config.gateway_env().get("desktopUrl") or "").strip().rstrip("/")
    if given:
        return given
    desktop = config.load_desktop_config().get("desktop")
    port = desktop.get("port") if isinstance(desktop, dict) else None
    return f"http://127.0.0.1:{port or DEFAULT_PORT}"


def _call(method: str, path: str, body: Any = None, timeout: float = 10) -> Any:
    request = urllib.request.Request(
        desktop_url() + path,
        method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={"content-type": "application/json"} if body is not None else {},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read() or b"{}")
    except urllib.error.HTTPError as error:
        detail = error.read().decode(errors="replace")
        raise RuntimeError(
            f"Desktop answered {error.code} for {method} {path}: {detail}"
        ) from error
    except (urllib.error.URLError, OSError) as error:
        raise DesktopUnavailableError(
            f"Desktop isn't answering at {desktop_url()}: {error}"
        ) from error


async def request_shell(
    title: str, message: str, commands: list[dict[str, Any]], app: str | None = None
) -> str:
    """Asks Desktop to run `commands` ([{run, note, needsStdout?, cwd?, env?}]); answers the session's id."""
    body = {"title": title, "message": message, "commands": commands, "app": app}
    answer = await asyncio.to_thread(_call, "POST", "/api/desktop/shell", body)
    return str(answer["id"])


async def wait_shell(session: str) -> dict[str, Any]:
    """The session once it ends: `status` succeeded / failed / cancelled and each command's result."""
    while True:
        try:
            answer = await asyncio.to_thread(
                _call, "GET", f"/api/desktop/shell/{session}?wait=25", None, 40
            )
        except DesktopUnavailableError:
            # Desktop restarting: its sessions are gone with it
            await asyncio.sleep(3)
            try:
                answer = await asyncio.to_thread(_call, "GET", f"/api/desktop/shell/{session}")
            except (DesktopUnavailableError, RuntimeError) as error:
                return {"status": "failed", "reason": str(error), "commands": []}
        if answer.get("status") in FINISHED:
            return dict(answer)
