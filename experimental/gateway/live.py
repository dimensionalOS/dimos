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
import typing
from typing import Any

from fastapi.encoders import jsonable_encoder
import psutil

from dimos.core.run_registry import RunEntry
from experimental.gateway.errors import ApiError

# ModuleConfig fields the coordinator sets for its own wiring, not for editing
INFRASTRUCTURE_FIELDS = frozenset({"rpc_transport", "g", "instance_name", "rpc_name"})


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


def config_fields(class_path: str) -> dict[str, Any]:
    """The pydantic fields of a module class's config, by name (none when the class can't be loaded)."""
    try:
        module, name = class_path.rsplit(".", 1)
        config_cls = typing.get_type_hints(getattr(importlib.import_module(module), name))["config"]
        return dict(config_cls.model_fields)
    except Exception:  # the class is the module's own code
        return {}


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
        raw: bool = False,
    ) -> dict[str, Any]:
        """The RPC's result as JSON, or with `raw` the object itself (for a CBOR reply)."""
        module = self.module(scanned, module_id)
        if name not in {rpc["name"] for rpc in module["rpcs"]}:
            listed = ", ".join(rpc["name"] for rpc in module["rpcs"])
            raise ApiError(404, f"{module_id} has no RPC {name} (its RPCs: {listed})")
        handle = self._handle(module_id, module)
        try:
            result = getattr(handle, name)(*args, **kwargs)
        except TimeoutError:
            raise ApiError(500, f"{module_id}/{name} didn't answer in time")
        except Exception as error:
            return {"ok": False, "result": None, "error": f"{type(error).__name__}: {error}"}
        if raw:
            return {"ok": True, "result": result, "error": None}
        try:
            encoded = jsonable_encoder(result)
        except ValueError:  # jsonable_encoder's error for a value with no fields it can read (no __dict__, not iterable)
            encoded = str(result)
        return {"ok": True, "result": encoded, "error": None}

    def _handle(self, module_id: str, module: dict[str, Any]) -> Any:
        # porcelain takes a namespaced instance's key, or an un-namespaced one's class name
        return self.app().get_module(
            module_id.replace(":", "/") if ":" in module_id else module["class"]
        )

    def config(self, module_id: str) -> dict[str, Any]:
        """The instance's config fields (its `get_config` RPC); a secret, or a field JSON can't carry, is readonly."""
        from experimental.gateway.introspect import field_schema, is_secret_name, secret_annotation
        from experimental.gateway.launches import HIDDEN

        module = self.module(None, module_id)
        instance = next(
            found
            for found in self.app().list_modules()
            if instance_id(found.instance_name) == module_id
        )
        if "get_config" not in {rpc.name for rpc in instance.rpcs}:
            raise ApiError(
                404, f"{module_id} has no get_config RPC (its dimos is older than the gateway)"
            )
        try:
            result = self._handle(module_id, module).get_config()
        except TimeoutError:
            raise ApiError(500, f"{module_id}/get_config didn't answer in time")
        fields = config_fields(module["catalogId"])
        config: dict[str, Any] = {}
        readonly: list[str] = []
        for field, value in dict(result).items():
            if field in INFRASTRUCTURE_FIELDS:
                continue
            kind = fields[field].annotation if field in fields else None
            if is_secret_name(field) or secret_annotation(kind):
                config[field] = HIDDEN
                readonly.append(field)
                continue
            try:
                config[field] = jsonable_encoder(value)
            except ValueError:  # jsonable_encoder's error for a value with no fields it can read
                config[field] = str(value)
                readonly.append(field)
                continue
            # get_config sends what JSON can't carry as its repr: the field's type says which those are
            if field in fields and field_schema(kind) is None:
                readonly.append(field)
        return {"config": config, "readonly": readonly}

    def module_ids(self) -> set[str]:
        """The running blueprint's instance ids (empty when nothing runs)."""
        try:
            app = self.app()
        except NothingRunningError:
            return set()
        return {instance_id(instance.instance_name) for instance in app.list_modules()}

    def restart(
        self, module_id: str, config: dict[str, Any] | None = None, reload_source: bool = True
    ) -> dict[str, Any]:
        """Restart that one instance (a namespaced robot's, not every copy of its class), with `config` (field ->
        value) merged over its config first, so a changed setting takes effect."""
        self.module(None, module_id)
        try:
            self.app().restart_instance(
                module_id.replace(":", "/"), reload_source=reload_source, config=config
            )
        except Exception as error:
            return {"ok": False, "error": f"{type(error).__name__}: {error}"}
        return {"ok": True, "error": None}

    def unload(self, module_id: str) -> dict[str, Any]:
        """Stop and remove that one instance (a namespaced robot's, not every copy of its class)."""
        self.module(None, module_id)
        try:
            self.app().unload_instance(module_id.replace(":", "/"))
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
