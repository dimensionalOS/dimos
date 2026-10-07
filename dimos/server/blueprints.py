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

"""Blueprints and GlobalConfig as JSON.

The blueprint list and GlobalConfig's schema are read in this process (names and a pydantic class: cheap). Anything
that imports a blueprint's code runs in a child process (introspect.py) with a timeout, so a blueprint that hangs,
crashes or loads a GPU stack can't take the server down. Answers are cached, since each child takes seconds.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
import json
from pathlib import Path
import sys
import time
from typing import Any

from dimos.server.introspect import MARKER


class IntrospectError(Exception):
    pass


class Cache:
    """Values by key, each good for `ttl` seconds; one computation per key at a time."""

    def __init__(self) -> None:
        self.values: dict[str, tuple[float, Any]] = {}
        self.locks: dict[str, asyncio.Lock] = {}

    def forget(self, key: str) -> None:
        self.values.pop(key, None)

    async def get(self, key: str, ttl: float, compute: Callable[[], Awaitable[Any]]) -> Any:
        async with self.locks.setdefault(key, asyncio.Lock()):
            hit = self.values.get(key)
            if hit and time.monotonic() - hit[0] < ttl:
                return hit[1]
            value = await compute()
            self.values[key] = (time.monotonic(), value)
            return value


def blueprint_list() -> list[dict[str, str]]:
    """What `dimos list` prints: built-ins (not demo-*, its rule) then external blueprints."""
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.external_blueprints import list_external_blueprint_names

    builtin = sorted(name for name in all_blueprints if not name.startswith("demo-"))
    return [{"name": name, "kind": "builtin"} for name in builtin] + [
        {"name": name, "kind": "external"} for name in list_external_blueprint_names()
    ]


def global_config_schema() -> dict[str, Any]:
    """GlobalConfig's JSON schema and each field's default."""
    from dimos.core.global_config import GlobalConfig

    defaults = {}
    for key, field in GlobalConfig.model_fields.items():
        if field.default_factory is None:
            try:
                defaults[key] = json.loads(json.dumps(field.default, default=str))
            except Exception:
                defaults[key] = None
    return {"schema": GlobalConfig.model_json_schema(), "defaults": defaults}


def valid_name(name: str) -> bool:
    return bool(name) and not name.startswith("-") and not any(c.isspace() for c in name)


async def introspect(
    dimos_dir: Path, args: list[str], timeout: float = 180, python: str | None = None
) -> dict[str, Any]:
    """`python -m dimos.server.introspect <args>` in the checkout; its JSON answer, or IntrospectError."""
    child = await asyncio.create_subprocess_exec(
        python or sys.executable,
        "-m",
        "dimos.server.introspect",
        *args,
        cwd=dimos_dir,
        stdin=asyncio.subprocess.DEVNULL,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        # its own group, so a timeout kills whatever it started too
        start_new_session=True,
    )
    try:
        stdout, stderr = await asyncio.wait_for(child.communicate(), timeout)
    except asyncio.TimeoutError:
        import os
        import signal

        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        await child.wait()
        raise IntrospectError(f"dimos introspection of {' '.join(args)} took over {timeout:.0f} s")
    text = stdout.decode("utf-8", "replace")
    _, found, answer = text.rpartition(MARKER)
    if not found:
        tail = (stderr.decode("utf-8", "replace").strip().splitlines() or ["no output"])[-1]
        raise IntrospectError(f"dimos introspection failed (exit {child.returncode}): {tail}")
    value: dict[str, Any] = json.loads(answer.strip().splitlines()[0])
    if isinstance(value.get("error"), str):
        raise IntrospectError(value["error"])
    return value


def shown_config(name: str, value: dict[str, Any]) -> dict[str, Any]:
    """A blueprint's introspected config as the API answers it: Desktop's saved module config for it (`overrides`),
    each arg marked `secret`, every secret value as •••."""
    from dimos.server import config, overrides

    saved = config.module_config(name)
    _, shown = overrides.redact({}, saved, overrides.secret_paths({}, saved))
    modules = []
    for module in value.get("modules", []):
        args = []
        for arg in module.get("args", []):
            secret = overrides.is_secret_name(arg["name"])
            arg = {**arg, "secret": secret}
            if secret and arg.get("value") is not None:
                arg["value"] = overrides.HIDDEN
            args.append(arg)
        modules.append({**module, "args": args})
    return {**value, "modules": modules, "overrides": shown}
