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

from __future__ import annotations

from collections.abc import Callable
import importlib
import inspect
import threading
from typing import Any

from fastapi.encoders import jsonable_encoder
import psutil

from dimos.core.run_registry import RunEntry
from experimental.gateway.errors import ApiError


class NothingRunningError(ApiError):
    def __init__(self) -> None:
        super().__init__(409, "nothing running")


def instance_id(remote_name: str) -> str:
    """The coordinator's key for an instance, `/` written as `:` so it is one path segment: `robot0:go2connection`."""
    # a namespaced instance's RPC name is its key; an un-namespaced one's is its class name (key = lowercase class)
    # TODO(core): list_modules could publish the coordinator's key itself, instead of the gateway rebuilding it
    return remote_name.replace("/", ":") if "/" in remote_name else remote_name.lower()


def flag_value(argv: list[str], flag: str) -> str | None:
    """The last `--flag=value` or `--flag value` in a command line."""
    found = None
    for index, arg in enumerate(argv):
        if arg.startswith(flag + "="):
            found = arg.partition("=")[2]
        elif arg == flag and index + 1 < len(argv):
            found = argv[index + 1]
    return found


def run_transport(entry: RunEntry) -> str | None:
    """The transport a run said it uses: its registry entry's global overrides, its command line, or its env."""
    from experimental.gateway.launches import current_launch

    if transport := entry.config_overrides.get("transport"):
        return str(transport)
    if transport := flag_value(list(entry.original_argv), "--transport"):
        return transport
    launch = current_launch() or {}
    if int(launch.get("pid") or -1) == entry.pid:
        if transport := launch.get("config", {}).get("global", {}).get("transport"):
            return str(transport)
    try:
        environment: dict[str, str] = psutil.Process(entry.pid).environ()
    except psutil.Error:
        environment = {}
    return environment.get("DIMOS_TRANSPORT")


def connect_dimos() -> Any:
    """`Dimos.connect()` on the transport the active run uses, since connect finds the coordinator on the gateway's
    own GlobalConfig transport."""
    from dimos.core.global_config import GlobalConfig, global_config
    from dimos.core.run_registry import get_most_recent
    from dimos.porcelain.dimos import Dimos

    entry = get_most_recent(alive_only=True)
    transport = run_transport(entry) if entry is not None else None
    # TODO(core): Dimos.connect could take a transport= instead of reading the process-wide GlobalConfig
    global_config.update(transport=transport or GlobalConfig().transport)
    return Dimos.connect()


def load_class(path: str) -> Any:
    """A module class from its class path (what Dimos.restart and Dimos.unload take)."""
    module, name = path.rsplit(".", 1)
    return getattr(importlib.import_module(module), name)


def live_run_id() -> str | None:
    from dimos.core.run_registry import get_most_recent

    entry = get_most_recent(alive_only=True)
    return None if entry is None else str(entry.run_id)


class Live:
    """The running blueprint through one kept-open `Dimos.connect()`, reconnected when the run changes. Its RPCs are
    called directly (no MCP, so no capability holds): this API is for buttons, agents use the python API."""

    def __init__(
        self,
        connect: Callable[[], Any] = connect_dimos,
        run_id: Callable[[], str | None] = live_run_id,
    ) -> None:
        self._connect, self._run_id = connect, run_id
        self._app: Any = None
        self._connected_to: str | None = None
        self._lock = threading.Lock()

    def app(self) -> Any:
        run = self._run_id()
        with self._lock:
            if self._app is not None and run != self._connected_to:
                self._app.stop()
                self._app = None
            if run is None:
                raise NothingRunningError()
            if self._app is None:
                try:
                    self._app = self._connect()
                except (RuntimeError, TimeoutError):
                    raise NothingRunningError() from None
                self._connected_to = run
            return self._app

    def modules(self, scanned: dict[str, Any] | None) -> list[dict[str, Any]]:
        """Each instance with its RPCs (doc, params and skill flag as the last scan found them) and streams."""
        from experimental.gateway.introspect import base_rpcs

        app = self.app()
        by_class = {
            module["class"]: module for module in (scanned or {}).get("modules", {}).values()
        }
        streams = self.streams()
        found = []
        for instance in app.list_modules():
            module_id = instance_id(instance.instance_name)
            known = {
                rpc["name"]: rpc
                for rpc in by_class.get(instance.qualified_path, {}).get("rpcs", [])
            }
            rpcs = [
                {
                    **known.get(rpc.name, {"params": None, "return_type": None, "skill": None}),
                    "name": rpc.name,
                    "module": module_id,
                    "doc": known.get(rpc.name, {}).get("doc") or rpc.documentation or "",
                }
                for rpc in sorted(instance.rpcs, key=lambda rpc: rpc.name)
                if rpc.name in known or rpc.name not in base_rpcs()
            ]
            found.append(
                {
                    "id": module_id,
                    "class": instance.class_name,
                    "catalogId": instance.qualified_path,
                    "namespace": module_id.rsplit(":", 1)[0] if ":" in module_id else None,
                    "doc": instance.documentation or "",
                    "rpcs": rpcs,
                    "streams": [stream for stream in streams if stream["module"] == module_id],
                }
            )
        return sorted(found, key=lambda module: module["id"])

    def module(self, scanned: dict[str, Any] | None, module_id: str) -> dict[str, Any]:
        modules = self.modules(scanned)
        found = next((listed for listed in modules if listed["id"] == module_id), None)
        if found is None:
            running = ", ".join(listed["id"] for listed in modules) or "none"
            raise ApiError(404, f"no running module {module_id} (running: {running})")
        return found

    def streams(self) -> list[dict[str, Any]]:
        return [
            {
                "name": stream.name,
                "type": stream.type_name,
                "direction": stream.direction,
                "channel": stream.channel,
                "module": instance_id(str(stream.module_name or "")),
            }
            for stream in self.app().list_streams()
        ]

    def call(
        self,
        scanned: dict[str, Any] | None,
        module_id: str,
        name: str,
        args: list[Any],
        kwargs: dict[str, Any],
    ) -> dict[str, Any]:
        module = self.module(scanned, module_id)
        if name not in {rpc["name"] for rpc in module["rpcs"]}:
            listed = ", ".join(rpc["name"] for rpc in module["rpcs"])
            raise ApiError(404, f"{module_id} has no RPC {name} (its RPCs: {listed})")
        # porcelain takes a namespaced instance's key, or an un-namespaced one's class name
        handle = self.app().get_module(
            module_id.replace(":", "/") if ":" in module_id else module["class"]
        )
        try:
            result = getattr(handle, name)(*args, **kwargs)
        except TimeoutError:
            raise ApiError(500, f"{module_id}/{name} didn't answer in time")
        except Exception as error:
            return {"ok": False, "result": None, "error": f"{type(error).__name__}: {error}"}
        try:
            encoded = jsonable_encoder(result)
        except ValueError:  # jsonable_encoder's error for a value with no fields it can read (no __dict__, not iterable)
            encoded = str(result)
        return {"ok": True, "result": encoded, "error": None}

    # TODO(core): Dimos.restart and Dimos.unload take a module class, so with several namespaced copies of one class
    # (robot0:go2connection, robot1:go2connection) they refuse ("Multiple instances"). Doing this cleanly needs them
    # to take an instance name: the coordinator already has restart_module_by_name, but unloading by name has no RPC.
    def restart(self, module_id: str) -> dict[str, Any]:
        return self._try(lambda app, module_class: app.restart(module_class), module_id)

    def unload(self, module_id: str) -> dict[str, Any]:
        return self._try(lambda app, module_class: app.unload(module_class), module_id)

    def _try(self, action: Callable[[Any, Any], None], module_id: str) -> dict[str, Any]:
        module = self.module(None, module_id)
        try:
            action(self.app(), load_class(module["catalogId"]))
        except Exception as error:
            return {"ok": False, "error": f"{type(error).__name__}: {error}"}
        return {"ok": True, "error": None}

    def peek(self, name: str, timeout: float | None = None) -> dict[str, Any]:
        from dimos.porcelain.dimos import Dimos

        if timeout is None:
            timeout = inspect.signature(Dimos.peek_stream).parameters["timeout"].default
        stream = next((listed for listed in self.streams() if listed["name"] == name), None)
        if stream is None:
            raise ApiError(404, f"no running stream {name}")
        try:
            value = self.app().peek_stream(name, timeout)
        except LookupError as error:
            raise ApiError(404, str(error))
        if value is None:
            raise ApiError(504, f"{name} sent nothing within {timeout} s")
        try:
            encoded = jsonable_encoder(value)
        except ValueError:  # jsonable_encoder's error for a value with no fields it can read (no __dict__, not iterable)
            encoded = str(value)
        return {
            "name": name,
            "type": stream["type"],
            "value": encoded,
            "ts": getattr(value, "ts", None),
        }
