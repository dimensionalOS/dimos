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

import atexit
from collections.abc import Callable
import importlib
import inspect
import threading
from typing import Any, TypeAlias, TypeVar, cast, overload

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator, ModuleDescriptor
from dimos.core.global_config import global_config
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo, extract_rpc_info
from dimos.core.module import ModuleBase, PeekNotFound
from dimos.core.rpc_client import RpcCall
from dimos.porcelain.local_module_source import LocalModuleSource
from dimos.porcelain.module_handle import ModuleHandle
from dimos.porcelain.module_source import ModuleSource
from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.porcelain.skills_proxy import SkillsProxy
from dimos.robot.all_blueprints import all_modules
from dimos.robot.get_all_blueprints import class_name_to_registry_key, get_by_name
from dimos.spec.utils import get_protocol_method_signatures, is_spec, spec_annotation_compliance

DescribeTarget: TypeAlias = str | ModuleHandle | RpcCall | ModuleInfo | RpcInfo
S = TypeVar("S")


class Dimos:
    def __init__(self, **config_overrides: Any) -> None:
        self._config_overrides = config_overrides
        self._coordinator: ModuleCoordinator | None = None
        self._source: ModuleSource | None = None
        self._lock = threading.RLock()
        self._stopped = False
        atexit.register(self.stop)

    def run(self, target: str | Blueprint | type[ModuleBase]) -> None:
        """Start a blueprint, module, or named configuration.

        Args:
            target: One of:
                - A string name from the blueprint/module registry
                  (e.g. `"unitree-go2-basic"` or `"camera-module"`).
                - A `Blueprint` object.
                - A `Module` class (calls `.blueprint()` automatically).

        The first call creates the coordinator and starts the system.
        Subsequent calls add modules to the already-running system.

        On a connected Dimos, `target` is sent to the daemon: strings and
        registered module classes take a name-based fast path, while Module
        classes and Blueprint objects are pickled and unpickled on the daemon.
        """
        with self._lock:
            if self._stopped:
                raise RuntimeError("This Dimos instance has been stopped")

            if self._source is not None and self._source.is_remote:
                assert isinstance(self._source, RemoteModuleSource)
                _run_remote(target, self._source)
                return

            blueprint = _resolve_target(target)
            if self._coordinator is None:
                if self._config_overrides:
                    global_config.update(**self._config_overrides)
                self._coordinator = ModuleCoordinator.build(blueprint)
                self._source = LocalModuleSource(self._coordinator)
            else:
                self._coordinator.load_blueprint(blueprint)

    def restart(self, module_class: type[ModuleBase], *, reload_source: bool = True) -> None:
        """Restart a running module, optionally reloading its source.

        Args:
            module_class: The module class to restart.
            reload_source: If True (default), reload the module's source file
                so code changes are picked up.
        """
        with self._lock:
            if self._source is None:
                raise RuntimeError("No modules are running")
            if self._source.is_remote:
                assert isinstance(self._source, RemoteModuleSource)
                self._source.restart_module_by_class_name(
                    module_class.__name__, reload_source=reload_source
                )
                return
            assert isinstance(self._source, LocalModuleSource)
            assert self._coordinator is not None
            self._source.invalidate(module_class.__name__)
            self._coordinator.restart_module(module_class, reload_source=reload_source)

    @classmethod
    def connect(cls, *, timeout: float = 5.0) -> Dimos:
        """Connect to the running DimOS coordinator on the current transport bus.

        One coordinator serves the configured bus. This works for both
        `dimos run` processes and coordinators launched directly from Python;
        a run-registry entry is not required.

        Returns a `Dimos` instance in read/call mode: `skills`, attribute
        access, `__repr__` and `__dir__` work, but only methods marked with
        `@rpc` (and `@skill`, which implies `@rpc`) on a module are callable.
        `stop()` closes the connection without terminating the remote process.
        """
        source = RemoteModuleSource(timeout=timeout)
        instance = cls()
        instance._source = source
        return instance

    def list_modules(self) -> list[ModuleInfo]:
        """Return structured information for the currently deployed module instances.

        Discovery is live: every call asks the coordinator for fresh descriptors.
        """
        with self._lock:
            source = self._require_source()
            descriptors = source.list_module_descriptors()

        return [_describe_module(descriptor) for descriptor in descriptors]

    @overload
    def get_module(self, name: Callable[..., S], *, instance_name: str | None = None) -> S: ...

    @overload
    def get_module(self, name: str) -> ModuleHandle: ...

    def get_module(
        self, name: str | Callable[..., S], *, instance_name: str | None = None
    ) -> S | ModuleHandle:
        """Resolve a typed Spec, exact instance name, or unique module class name.

        A Spec matches deployed modules whose advertised RPCs implement its
        method signatures. Exactly one match is required. Use ``instance_name``
        with a Spec to select one instance when several implement it. Matching
        requires the deployed module class to be importable in the client.
        """
        with self._lock:
            source = self._require_source()
            if isinstance(name, str):
                if instance_name is not None:
                    raise TypeError("instance_name is only supported with a Spec")
                return source.get_module(name)
            if not is_spec(name):
                raise TypeError("get_module() expects a module name or a Spec Protocol")
            # Callable admits abstract Protocol classes to type checkers; the
            # Spec is inspected here, never constructed or called.
            spec = cast("type[S]", name)
            required = set(get_protocol_method_signatures(spec))
            if not required:
                raise TypeError("A client Spec must declare at least one RPC method")
            matches: list[str] = []
            unavailable: list[str] = []
            for descriptor in source.list_module_descriptors():
                deployed_name = descriptor.rpc_name or descriptor.class_name
                if instance_name is not None and deployed_name != instance_name:
                    continue
                if not required.issubset(descriptor.rpc_names):
                    continue
                module_class = _load_module_class(descriptor)
                if module_class is None:
                    unavailable.append(deployed_name)
                elif spec_annotation_compliance(module_class, spec):
                    matches.append(deployed_name)
            if not matches:
                detail = f" for instance {instance_name!r}" if instance_name is not None else ""
                if unavailable:
                    detail += f"; cannot inspect module classes for {sorted(unavailable)}"
                raise LookupError(
                    f"No deployed module matches {spec.__name__} RPC signatures{detail}"
                )
            if len(matches) != 1:
                raise ValueError(
                    f"Multiple modules match {spec.__name__}: {sorted(matches)}; "
                    "select one with instance_name="
                )
            return cast("S", source.get_module(matches[0]))

    def list_rpcs(self, module: str | ModuleHandle | None = None) -> list[RpcInfo]:
        """Return structured metadata for all advertised RPCs.

        Args:
            module: Optional exact instance name, unique class name, or module proxy.
        """
        modules = self.list_modules()
        if module is None:
            return [rpc_info for module_info in modules for rpc_info in module_info.rpcs]

        instance_name = self._module_instance_name(module)
        for module_info in modules:
            if module_info.instance_name == instance_name:
                return module_info.rpcs
        raise KeyError(instance_name)

    def describe(self, target: DescribeTarget) -> ModuleInfo | RpcInfo:
        """Describe a module or RPC proxy, or a qualified ``module.rpc`` string."""
        if isinstance(target, (ModuleInfo, RpcInfo)):
            return target
        if isinstance(target, RpcCall):
            return self._find_rpc(target.remote_name, target.rpc_name)
        if not isinstance(target, str):
            return self._find_module(self._module_instance_name(target))

        try:
            proxy = self.get_module(target)
        except KeyError:
            proxy = None
        if proxy is not None:
            return self._find_module(self._module_instance_name(proxy))

        if "." in target:
            module_name, rpc_name = target.rsplit(".", 1)
            return self._find_rpc(self._module_instance_name(module_name), rpc_name)

        matching_modules = sorted(
            {
                rpc_info.module_name
                for rpc_info in self.list_rpcs()
                if rpc_info.name == target and rpc_info.module_name is not None
            }
        )
        if matching_modules:
            choices = ", ".join(f"{name}.{target}" for name in matching_modules)
            raise ValueError(f"RPC name {target!r} is not qualified; use one of: {choices}")
        raise KeyError(target)

    def _find_module(self, instance_name: str) -> ModuleInfo:
        for module_info in self.list_modules():
            if module_info.instance_name == instance_name:
                return module_info
        raise KeyError(instance_name)

    def _find_rpc(self, instance_name: str, rpc_name: str) -> RpcInfo:
        for rpc_info in self.list_rpcs(instance_name):
            if rpc_info.name == rpc_name:
                return rpc_info
        raise KeyError(f"{instance_name}.{rpc_name}")

    def _module_instance_name(self, module: str | ModuleHandle) -> str:
        proxy = self.get_module(module) if isinstance(module, str) else module
        return proxy.remote_name

    def _require_source(self) -> ModuleSource:
        if self._source is None:
            raise RuntimeError("No modules are running")
        return self._source

    @property
    def skills(self) -> SkillsProxy:
        """Access skills from all running modules.

        Returns a proxy that supports attribute access and pretty-printing::

            app.skills.move_to(x=2.0, relative=True)
            print(app.skills)
        """
        with self._lock:
            if self._source is None:
                raise RuntimeError("No modules are running")
            return SkillsProxy(self._source)

    def peek_stream(self, name: str, timeout: float = 1.0) -> Any:
        """Fetch the next message from a named stream on any running module.

        Blocks up to `timeout` seconds for the next emission. Returns
        None if no value arrives in time.

        Args:
            name: Stream attribute name (e.g. "color_image").
            timeout: Max seconds to wait.
        """

        with self._lock:
            source = self._source
            if source is None:
                raise RuntimeError("No modules are running")

        module_names = source.list_module_names()
        for module_name in module_names:
            try:
                module = source.get_module(module_name)
                peek = getattr(module, "peek_stream", None)
                if peek is None:
                    continue
                result = peek(name, timeout)
            except Exception:
                continue
            if isinstance(result, PeekNotFound):
                continue
            return result

        raise LookupError(
            f"No running module exposes a stream named {name!r}. Running modules: {module_names}"
        )

    def stop(self) -> None:
        """Stop all modules and clean up resources.

        On a locally-driven `Dimos`, stops the coordinator and workers.
        On a connected `Dimos` (from `Dimos.connect()`), closes the LCM RPC
        client without terminating the remote process.
        """
        with self._lock:
            if self._stopped:
                return
            self._stopped = True

            if self._source is not None:
                try:
                    self._source.close()
                except Exception:
                    pass
                self._source = None

            if self._coordinator is not None:
                self._coordinator.stop()
                self._coordinator = None

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._source is not None and not self._stopped

    def __getattr__(self, name: str) -> ModuleHandle:
        if name.startswith("_"):
            raise AttributeError(name)

        with self._lock:
            source = self._source
            if source is None:
                raise RuntimeError("No modules are running")

        try:
            return source.get_module(name)
        except KeyError:
            pass

        known_names = _all_module_class_names()
        if name in known_names:
            raise AttributeError(
                f"{name} exists but is not running. Start it with app.run({name}) "
                f"or add it to your blueprint."
            )
        raise AttributeError(f"No module named {name!r}")

    def __repr__(self) -> str:
        with self._lock:
            if self._source is None:
                return "<Dimos(stopped)>"
            modules = self._source.list_module_names()
            return f"<Dimos(remote={self._source.is_remote}, modules={modules})>"

    def __dir__(self) -> list[str]:
        base = list(super().__dir__())
        with self._lock:
            if self._source is not None:
                base.extend(self._source.list_module_names())
        return base


def _run_remote(target: str | Blueprint | type[ModuleBase], source: RemoteModuleSource) -> None:
    if isinstance(target, str):
        source.load_blueprint_by_name(target)
        return
    if inspect.isclass(target) and issubclass(target, ModuleBase):
        key = class_name_to_registry_key(target.__name__)
        if key in all_modules:
            source.load_blueprint_by_name(key)
        else:
            source.load_blueprint(target.blueprint())
        return
    if isinstance(target, Blueprint):
        source.load_blueprint(target)
        return
    raise TypeError(
        f"run() expects a blueprint name (str), Blueprint, or Module class, "
        f"got {type(target).__name__}"
    )


def _load_module_class(descriptor: ModuleDescriptor) -> type[ModuleBase] | None:
    try:
        module_path, class_name = descriptor.qualified_path.rsplit(".", 1)
        candidate = getattr(importlib.import_module(module_path), class_name)
        if inspect.isclass(candidate) and issubclass(candidate, ModuleBase):
            return candidate
    except (ImportError, AttributeError, ValueError):
        pass
    return None


def _describe_module(descriptor: ModuleDescriptor) -> ModuleInfo:
    instance_name = descriptor.rpc_name or descriptor.class_name
    module_class = _load_module_class(descriptor)

    rpc_infos: list[RpcInfo] = []
    for rpc_name in descriptor.rpc_names:
        original = getattr(module_class, rpc_name, None) if module_class is not None else None
        if callable(original):
            rpc_info = extract_rpc_info(original)
        else:
            rpc_info = RpcInfo(name=rpc_name)
        rpc_info.module_name = instance_name
        rpc_infos.append(rpc_info)

    return ModuleInfo(
        name=instance_name,
        rpcs=rpc_infos,
        instance_name=instance_name,
        class_name=descriptor.class_name,
        qualified_path=descriptor.qualified_path,
        documentation=inspect.getdoc(module_class) if module_class is not None else None,
    )


def _resolve_target(target: str | Blueprint | type[ModuleBase]) -> Blueprint:
    """Convert a run() argument into a Blueprint."""

    if isinstance(target, str):
        return get_by_name(target)
    if isinstance(target, Blueprint):
        return target
    if inspect.isclass(target) and issubclass(target, ModuleBase):
        return target.blueprint()  # type: ignore[no-any-return]
    raise TypeError(
        f"run() expects a blueprint name (str), Blueprint, or Module class, "
        f"got {type(target).__name__}"
    )


def _all_module_class_names() -> set[str]:
    """Return the set of all known module class names from the registry."""
    return {path.rsplit(".", 1)[-1] for path in all_modules.values()}
