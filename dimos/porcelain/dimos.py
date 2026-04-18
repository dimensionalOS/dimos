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
import inspect
import threading
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.run_registry import get_most_recent_rpyc_port
from dimos.porcelain.local_module_source import LocalModuleSource
from dimos.porcelain.module_source import ModuleSource
from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.porcelain.skills_proxy import SkillsProxy
from dimos.robot.all_blueprints import all_modules
from dimos.robot.get_all_blueprints import class_name_to_registry_key, get_by_name


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
    def connect(
        cls,
        *,
        run_id: str | None = None,
        host: str | None = None,
        port: int | None = None,
    ) -> Dimos:
        """Connect to an already-running DimOS instance.

        With no arguments, finds the most recent alive `RunEntry` in the
        registry and connects to its coordinator RPyC endpoint. Use `run_id=` to
        select a specific run, or `host=` + `port=` to bypass the registry.

        Returns a `Dimos` instance in read/call mode: `skills`, attribute
        access, `__repr__` and `__dir__` work, but `run()` and `restart()` raise
        `NotImplementedError`. `stop()` closes the connection without
        terminating the remote process.
        """
        if host is not None and port is not None:
            source: ModuleSource = RemoteModuleSource(host, port)
        else:
            rpyc_port = get_most_recent_rpyc_port(run_id=run_id)
            source = RemoteModuleSource("localhost", rpyc_port)

        instance = cls()
        instance._source = source
        return instance

    @property
    def skills(self) -> SkillsProxy:
        """Access skills from all running modules.

        Returns a proxy that supports attribute access and pretty-printing::

            app.skills.relative_move(forward=2.0)
            print(app.skills)
        """
        with self._lock:
            if self._source is None:
                raise RuntimeError("No modules are running")
            return SkillsProxy(self._source)

    def stop(self) -> None:
        """Stop all modules and clean up resources.

        On a locally-driven `Dimos`, stops the coordinator and workers.
        On a connected `Dimos` (from `Dimos.connect()`), closes RPyC
        connections without terminating the remote process.
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

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)

        with self._lock:
            source = self._source
            if source is None:
                raise RuntimeError("No modules are running")

        try:
            return source.get_rpyc_module(name)
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
            source.load_blueprint_pickled(target.blueprint())
        return
    if isinstance(target, Blueprint):
        source.load_blueprint_pickled(target)
        return
    raise TypeError(
        f"run() expects a blueprint name (str), Blueprint, or Module class, "
        f"got {type(target).__name__}"
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
