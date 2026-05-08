# Copyright 2025-2026 Dimensional Inc.
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

import asyncio
from collections import defaultdict
from collections.abc import Mapping, MutableMapping
import dataclasses
import importlib
import inspect
from pathlib import Path
import shutil
import sys
import threading
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, cast

from dimos.core.coordination.rpyc_server import RpycServer
from dimos.core.coordination.worker_manager import WorkerManager
from dimos.core.coordination.worker_manager_docker import WorkerManagerDocker
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.record.record_replay import RecordReplay
from dimos.spec.utils import is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol

logger = setup_logger()


class ModuleCoordinator(Resource):
    _managers: dict[str, WorkerManager]
    _global_config: GlobalConfig
    _deployed_modules: dict[type[ModuleBase], ModuleProxyProtocol]

    def __init__(
        self,
        g: GlobalConfig = global_config,
    ) -> None:
        self._global_config = g
        manager_types: list[type[WorkerManager]] = [WorkerManagerDocker, WorkerManagerPython]
        self._managers = {cls.deployment_identifier: cls(g=g) for cls in manager_types}
        self._deployed_modules = {}
        self._deployed_atoms: dict[type[ModuleBase], BlueprintAtom] = {}
        self._resolved_module_refs: dict[tuple[type[ModuleBase], str], type[ModuleBase]] = {}
        self._transport_registry: dict[tuple[str, type], PubSubTransport[Any]] = {}
        self._class_aliases: dict[type[ModuleBase], type[ModuleBase]] = {}
        self._module_transports: dict[type[ModuleBase], dict[str, PubSubTransport[Any]]] = {}
        self._started = False
        self._modules_lock = threading.RLock()
        self._rpyc = RpycServer(self)

    def start(self) -> None:
        from dimos.core.o3dpickle import register_picklers

        register_picklers()
        for m in self._managers.values():
            m.start()
        self._started = True

    def stop(self) -> None:
        self._rpyc.stop()

        for module_class, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=module_class.__name__)
            try:
                module.stop()
            except Exception:
                logger.error("Error stopping module", module=module_class.__name__, exc_info=True)
            logger.info("Module stopped.", module=module_class.__name__)

        def _stop_manager(m: WorkerManager) -> None:
            try:
                m.stop()
            except Exception:
                logger.error("Error stopping manager", manager=type(m).__name__, exc_info=True)

        safe_thread_map(tuple(self._managers.values()), _stop_manager)

    def start_rpyc_service(self) -> int:
        return self._rpyc.start()

    def list_module_names(self) -> list[str]:
        with self._modules_lock:
            return [cls.__name__ for cls in self._deployed_modules]

    def get_module_endpoint(self, class_name: str) -> tuple[str, int, int]:
        """Return (host, worker_rpyc_port, module_id) for the given class name.

        Lazily starts the worker-side RPyC server on first use.
        """
        with self._modules_lock:
            for cls, proxy in self._deployed_modules.items():
                if cls.__name__ == class_name:
                    actor = cast("ModuleProxy", proxy).actor_instance
                    port = actor.start_rpyc()
                    return ("localhost", int(port), int(actor._module_id))
        raise KeyError(class_name)

    def health_check(self) -> bool:
        return all(m.health_check() for m in self._managers.values())

    @property
    def n_modules(self) -> int:
        return len(self._deployed_modules)

    def suppress_console(self) -> None:
        for m in self._managers.values():
            m.suppress_console()

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> ModuleProxy:
        if not self._managers:
            raise ValueError("Trying to dimos.deploy before the client has started")

        deployed_module = self._managers[module_class.deployment].deploy(
            module_class, global_config, kwargs
        )
        with self._modules_lock:
            self._deployed_modules[module_class] = deployed_module
        return deployed_module  # type: ignore[return-value]

    def deploy_parallel(
        self, module_specs: list[ModuleSpec], blueprint_args: Mapping[str, Mapping[str, Any]]
    ) -> list[ModuleProxy]:
        if not self._managers:
            raise ValueError("Not started")

        # Group specs by deployment type, tracking original indices for reassembly
        indices_by_deployment: dict[str, list[int]] = {}
        specs_by_deployment: dict[str, list[ModuleSpec]] = {}
        for index, spec in enumerate(module_specs):
            # spec = (module_class, global_config, kwargs)
            dep = spec[0].deployment
            indices_by_deployment.setdefault(dep, []).append(index)
            specs_by_deployment.setdefault(dep, []).append(spec)

        results: list[Any] = [None] * len(module_specs)

        def _deploy_group(dep: str) -> None:
            deployed = self._managers[dep].deploy_parallel(specs_by_deployment[dep], blueprint_args)
            for index, module in zip(indices_by_deployment[dep], deployed, strict=True):
                results[index] = module

        try:
            safe_thread_map(list(specs_by_deployment.keys()), _deploy_group)
        except:
            self.stop()
            raise

        with self._modules_lock:
            self._deployed_modules.update(
                {
                    cls: mod
                    for (cls, _, _), mod in zip(module_specs, results, strict=True)
                    if mod is not None
                }
            )
        return results

    def build_all_modules(self) -> None:
        """Call build() on all deployed modules in parallel.

        build() handles heavy one-time work (docker builds, LFS downloads, etc.)
        with a very long timeout. Must be called after deploy and stream wiring
        but before start_all_modules().
        """
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before build_all_modules().")

        try:
            safe_thread_map(modules, lambda m: m.build())
        except:
            self.stop()
            raise

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        if not modules:
            raise ValueError("No modules deployed. Call deploy() before start_all_modules().")

        safe_thread_map(modules, lambda m: m.start())

        self._send_on_system_modules()

    def _resolve_class(self, cls: type[ModuleBase]) -> type[ModuleBase]:
        return self._class_aliases.get(cls, cls)

    def get_instance(self, module: type[ModuleBase]) -> ModuleProxy:
        return self._deployed_modules.get(self._resolve_class(module))  # type: ignore[return-value]

    def _send_on_system_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        for module in modules:
            if hasattr(module, "on_system_modules"):
                module.on_system_modules(modules)

    def _connect_streams(self, blueprint: Blueprint) -> None:
        streams: dict[tuple[str, type], list[tuple[type, str]]] = defaultdict(list)

        for bp in blueprint.active_blueprints:
            for conn in bp.streams:
                remapped_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    streams[remapped_name, conn.type].append((bp.module, conn.name))

        for remapped_name, stream_type in streams.keys():
            key = (remapped_name, stream_type)
            if key in self._transport_registry:
                transport = self._transport_registry[key]
            else:
                transport = _get_transport_for(blueprint, remapped_name, stream_type)
            self._transport_registry[key] = transport
            for module, original_name in streams[key]:
                instance = self.get_instance(module)  # type: ignore[assignment]
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]
                self._module_transports.setdefault(module, {})[original_name] = transport
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{stream_type.__module__}.{stream_type.__qualname__}",
                    module=module.__name__,
                    transport=transport.__class__.__name__,
                )

    @classmethod
    def build(
        cls,
        blueprint: Blueprint,
        blueprint_args: MutableMapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        logger.info("Building the blueprint")
        global_config.update(**dict(blueprint.global_config_overrides))
        blueprint_args = blueprint_args or {}
        if "g" in blueprint_args:
            global_config.update(**blueprint_args.pop("g"))

        # Auto-replay if --replay-file is set
        replay_file = global_config.replay_file
        if replay_file:
            logger.info("Auto-replay from %s", replay_file)
            # Strip replay_file from all override sources so the nested build()
            # inside replay() does not re-enter this branch.
            global_config.replay_file = None
            clean_bp = dataclasses.replace(
                blueprint,
                global_config_overrides=MappingProxyType(
                    {
                        k: v
                        for k, v in blueprint.global_config_overrides.items()
                        if k != "replay_file"
                    }
                ),
            )
            return cls.replay(clean_bp, replay_file, blueprint_args=dict(blueprint_args))

        _run_configurators(blueprint)
        _check_requirements(blueprint)
        _verify_no_name_conflicts(blueprint)

        logger.info("Starting the modules")
        coordinator = cls(g=global_config)
        coordinator.start()

        _deploy_all_modules(blueprint, coordinator, global_config, blueprint_args)
        coordinator._connect_streams(blueprint)
        _connect_module_refs(blueprint, coordinator)

        coordinator.build_all_modules()
        coordinator.start_all_modules()

        if global_config.record_path:
            # Delete existing file, don't append to it.
            Path(global_config.record_path).unlink(missing_ok=True)
            record_modules = blueprint.record_modules
            for bp in blueprint.active_blueprints:
                if bp.module in record_modules:
                    instance = coordinator.get_instance(bp.module)
                    if instance is not None:
                        instance.start_recording_outputs(global_config.record_path)

        _log_blueprint_graph(blueprint, coordinator)

        return coordinator

    @classmethod
    def replay(
        cls,
        blueprint: Blueprint,
        recording_path: str,
        *,
        speed: float = 1.0,
        blueprint_args: MutableMapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        """Build with a recording replacing some module outputs."""
        recording = RecordReplay(recording_path)
        recorded_streams = set(recording.store.list_streams())
        if not recorded_streams:
            raise ValueError("Recording is empty — no streams to replay")

        modules_to_disable: list[type[ModuleBase]] = []
        for bp in blueprint.blueprints:
            out_names = {c.name for c in bp.streams if c.direction == "out"}
            if not out_names:
                continue
            covered = out_names & recorded_streams
            if covered:
                modules_to_disable.append(bp.module)
                uncovered = out_names - covered
                if uncovered:
                    logger.warning(
                        "Replay: disabling %s (partial: replaying %s, missing %s)",
                        bp.module.__name__,
                        covered,
                        uncovered,
                    )
                else:
                    logger.info("Replay: disabling %s (all OUTs covered)", bp.module.__name__)

        if not modules_to_disable:
            logger.warning(
                "Replay: no modules disabled - recording streams %s"
                "don't match any module OUT names",
                recorded_streams,
            )

        patched = blueprint.disabled_modules(*modules_to_disable)
        coordinator = cls.build(patched, blueprint_args)

        recording.play(speed=speed)

        return coordinator

    def load_blueprint(
        self,
        blueprint: Blueprint,
        blueprint_args: MutableMapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        """Load a blueprint into an already-running coordinator.

        Deploys, wires, builds and starts the modules described by *blueprint*.
        Workers are added automatically based on the blueprint's ``n_workers``
        global-config override (additive).
        """
        if not self._started:
            raise RuntimeError("ModuleCoordinator not started; call start() first")

        with self._modules_lock:
            self._load_blueprint(blueprint, blueprint_args)

    def _load_blueprint(
        self,
        blueprint: Blueprint,
        blueprint_args: MutableMapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        # Apply config overrides.
        self._global_config.update(**dict(blueprint.global_config_overrides))
        blueprint_args = blueprint_args or {}
        if "g" in blueprint_args:
            self._global_config.update(**blueprint_args.pop("g"))

        # Scale worker pool.
        n_extra = int(blueprint.global_config_overrides.get("n_workers", 0))
        python_wm = cast("WorkerManagerPython", self._managers["python"])
        if n_extra:
            python_wm.add_workers(n_extra)
        if not python_wm.workers and blueprint.active_blueprints:
            python_wm.add_workers(1)

        _run_configurators(blueprint)
        _check_requirements(blueprint)
        _verify_no_name_conflicts(blueprint)
        _verify_no_conflicts_with_existing(blueprint, self._transport_registry)

        # Reject duplicate modules.
        for bp in blueprint.active_blueprints:
            if bp.module in self._deployed_modules:
                raise ValueError(
                    f"{bp.module.__name__} is already deployed; cannot load the same module twice"
                )

        before = set(self._deployed_modules)

        _deploy_all_modules(blueprint, self, self._global_config, blueprint_args)
        self._connect_streams(blueprint)
        _connect_module_refs(blueprint, self, existing_modules=before)

        new_modules = [proxy for cls, proxy in self._deployed_modules.items() if cls not in before]

        if new_modules:
            safe_thread_map(new_modules, lambda m: m.build())
            safe_thread_map(new_modules, lambda m: m.start())

        self._send_on_system_modules()

    def load_module(
        self,
        module_class: type[ModuleBase],
        blueprint_args: MutableMapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        self.load_blueprint(module_class.blueprint(**blueprint_args or {}))

    def unload_module(self, module_class: type[ModuleBase]) -> None:
        """Stop and tear down a single deployed module.

        Removes the module from coordinator state, stops its worker-side
        instance, and shuts down the worker process if it becomes empty.
        Stream transports and other modules' references are left intact —
        callers that expect the module to come back (e.g. ``restart_module``)
        are responsible for rewiring.
        """
        with self._modules_lock:
            self._unload_module(module_class)

    def _unload_module(self, module_class: type[ModuleBase]) -> None:
        module_class = self._resolve_class(module_class)
        if module_class not in self._deployed_modules:
            raise ValueError(f"{module_class.__name__} is not deployed")
        if module_class.deployment != "python":
            raise NotImplementedError(
                f"unload_module only supports python deployment, got {module_class.deployment!r}"
            )

        proxy = self._deployed_modules[module_class]

        try:
            proxy.stop()
        except Exception:
            logger.error(
                "Error stopping module during unload",
                module=module_class.__name__,
                exc_info=True,
            )

        python_wm = cast("WorkerManagerPython", self._managers["python"])
        try:
            python_wm.undeploy(proxy)
        except Exception:
            logger.error(
                "Error undeploying module from worker",
                module=module_class.__name__,
                exc_info=True,
            )

        del self._deployed_modules[module_class]
        self._deployed_atoms.pop(module_class, None)
        self._module_transports.pop(module_class, None)
        self._class_aliases = {
            k: v for k, v in self._class_aliases.items() if v is not module_class
        }
        self._resolved_module_refs = {
            key: target
            for key, target in self._resolved_module_refs.items()
            if key[0] is not module_class and target is not module_class
        }

    def restart_module_by_class_name(
        self,
        class_name: str,
        *,
        reload_source: bool = True,
    ) -> ModuleProxyProtocol:
        with self._modules_lock:
            for cls in self._deployed_modules:
                if cls.__name__ == class_name:
                    return self._restart_module(cls, reload_source=reload_source)
        raise ValueError(f"No deployed module with class name {class_name!r}")

    def restart_module(
        self,
        module_class: type[ModuleBase],
        *,
        reload_source: bool = True,
    ) -> ModuleProxyProtocol:
        """Restart a single deployed module in place.

        Unloads *module_class*, optionally reloads its source file via
        ``importlib.reload`` so edited code is picked up, then redeploys it
        onto a fresh worker process, reconnects its streams to the existing
        transports, and re-injects the new proxy into every other module that
        held a reference to it.
        """
        with self._modules_lock:
            return self._restart_module(module_class, reload_source=reload_source)

    def _restart_module(
        self,
        module_class: type[ModuleBase],
        *,
        reload_source: bool = True,
    ) -> ModuleProxyProtocol:
        module_class = self._resolve_class(module_class)
        if module_class not in self._deployed_modules:
            raise ValueError(f"{module_class.__name__} is not deployed")
        if module_class.deployment != "python":
            raise NotImplementedError(
                f"restart_module only supports python deployment, got {module_class.deployment!r}"
            )

        old_atom = self._deployed_atoms[module_class]
        kwargs = dict(old_atom.kwargs)
        saved_transports = dict(self._module_transports.get(module_class, {}))
        inbound_refs = [
            (consumer, ref_name)
            for (consumer, ref_name), target in self._resolved_module_refs.items()
            if target is module_class
        ]
        outbound_refs = [
            (ref_name, target)
            for (consumer, ref_name), target in self._resolved_module_refs.items()
            if consumer is module_class
        ]

        self.unload_module(module_class)

        if reload_source:
            source_mod = sys.modules.get(module_class.__module__)
            if source_mod is None:
                source_mod = importlib.import_module(module_class.__module__)
            importlib.reload(source_mod)
            new_class = cast("type[ModuleBase]", getattr(source_mod, module_class.__name__))
        else:
            new_class = module_class

        if new_class is not module_class:
            for old_cls in list(self._class_aliases):
                if self._class_aliases[old_cls] is module_class:
                    self._class_aliases[old_cls] = new_class
            self._class_aliases[module_class] = new_class

        python_wm = cast("WorkerManagerPython", self._managers["python"])
        new_proxy = python_wm.deploy_fresh(new_class, self._global_config, kwargs)
        self._deployed_modules[new_class] = new_proxy

        new_bp = new_class.blueprint(**kwargs)
        new_atom = new_bp.active_blueprints[0]
        self._deployed_atoms[new_class] = new_atom

        for stream_ref in new_atom.streams:
            transport = saved_transports.get(stream_ref.name)
            if transport is not None:
                new_proxy.set_transport(stream_ref.name, transport)
        self._module_transports[new_class] = {
            s.name: t for s in new_atom.streams if (t := saved_transports.get(s.name)) is not None
        }

        for consumer_class, ref_name in inbound_refs:
            consumer_proxy = self._deployed_modules.get(consumer_class)
            if consumer_proxy is None:
                continue
            setattr(consumer_proxy, ref_name, new_proxy)
            consumer_proxy.set_module_ref(ref_name, new_proxy)  # type: ignore[attr-defined]
            self._resolved_module_refs[consumer_class, ref_name] = new_class

        for ref_name, target_class in outbound_refs:
            target_proxy = self._deployed_modules.get(target_class)
            if target_proxy is None:
                continue
            setattr(new_proxy, ref_name, target_proxy)
            new_proxy.set_module_ref(ref_name, target_proxy)  # type: ignore[attr-defined]
            self._resolved_module_refs[new_class, ref_name] = target_class

        new_proxy.build()
        new_proxy.start()

        self._send_on_system_modules()

        return new_proxy

    async def loop(self) -> None:
        stop = asyncio.Event()
        try:
            await stop.wait()
        except (KeyboardInterrupt, asyncio.CancelledError):
            return
        finally:
            self.stop()


def _all_name_types(blueprint: Blueprint) -> set[tuple[str, type]]:
    result = set()
    for bp in blueprint.active_blueprints:
        for conn in bp.streams:
            remapped_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
            if isinstance(remapped_name, str):
                result.add((remapped_name, conn.type))
    return result


def _is_name_unique(blueprint: Blueprint, name: str) -> bool:
    return sum(1 for n, _ in _all_name_types(blueprint) if n == name) == 1


def _get_transport_for(blueprint: Blueprint, name: str, stream_type: type) -> PubSubTransport[Any]:
    transport = blueprint.transport_map.get((name, stream_type), None)
    if transport:
        return transport

    use_pickled = getattr(stream_type, "lcm_encode", None) is None
    topic = f"/{name}" if _is_name_unique(blueprint, name) else f"/{short_id()}"
    transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)

    return transport


def _verify_no_name_conflicts(blueprint: Blueprint) -> None:
    name_to_types: dict[Any, set[type]] = defaultdict(set)
    name_to_modules: dict[Any, list[tuple[type, type]]] = defaultdict(list)

    for bp in blueprint.active_blueprints:
        for conn in bp.streams:
            stream_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
            name_to_types[stream_name].add(conn.type)
            name_to_modules[stream_name].append((bp.module, conn.type))

    conflicts: dict[Any, dict[type, list[type]]] = {}
    for conn_name, types in name_to_types.items():
        if len(types) > 1:
            modules_by_type: dict[type, list[type]] = defaultdict(list)
            for module, conn_type in name_to_modules[conn_name]:
                modules_by_type[conn_type].append(module)
            conflicts[conn_name] = modules_by_type

    if not conflicts:
        return

    error_lines = ["Blueprint cannot start because there are conflicting streams."]
    for name, modules_by_type in conflicts.items():
        type_entries = []
        for conn_type, modules in modules_by_type.items():
            for module in modules:
                type_str = f"{conn_type.__module__}.{conn_type.__name__}"
                module_str = module.__name__
                type_entries.append((type_str, module_str))
        if len(type_entries) >= 2:
            locations = ", ".join(f"{type_} in {module}" for type_, module in type_entries)
            error_lines.append(f"    - '{name}' has conflicting types. {locations}")

    raise ValueError("\n".join(error_lines))


def _verify_no_conflicts_with_existing(
    blueprint: Blueprint,
    existing_registry: dict[tuple[str, type], PubSubTransport[Any]],
) -> None:
    """Check that a new blueprint's streams don't conflict with already-registered transports."""
    if not existing_registry:
        return

    existing_names: dict[str, set[type]] = defaultdict(set)
    for name, stream_type in existing_registry:
        existing_names[name].add(stream_type)

    for bp in blueprint.active_blueprints:
        for conn in bp.streams:
            remapped_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
            if isinstance(remapped_name, str) and remapped_name in existing_names:
                for existing_type in existing_names[remapped_name]:
                    if existing_type != conn.type:
                        raise ValueError(
                            f"Stream '{remapped_name}' in {bp.module.__name__} has type "
                            f"{conn.type.__module__}.{conn.type.__name__} but an existing "
                            f"transport uses {existing_type.__module__}.{existing_type.__name__}"
                        )


def _run_configurators(blueprint: Blueprint) -> None:
    from dimos.protocol.service.system_configurator.base import configure_system
    from dimos.protocol.service.system_configurator.lcm_config import lcm_configurators

    configurators = [*lcm_configurators(), *blueprint.configurator_checks]

    try:
        configure_system(configurators)
    except SystemExit:
        labels = [type(c).__name__ for c in configurators]
        print(
            f"Required system configuration was declined: {', '.join(labels)}",
            file=sys.stderr,
        )
        sys.exit(1)


def _check_requirements(blueprint: Blueprint) -> None:
    errors = []
    red = "\033[31m"
    reset = "\033[0m"

    for check in blueprint.requirement_checks:
        error = check()
        if error:
            errors.append(error)

    if errors:
        for error in errors:
            print(f"{red}Error: {error}{reset}", file=sys.stderr)
        sys.exit(1)


def _deploy_all_modules(
    blueprint: Blueprint,
    module_coordinator: ModuleCoordinator,
    gc: GlobalConfig,
    blueprint_args: Mapping[str, Mapping[str, Any]],
) -> None:
    module_specs: list[ModuleSpec] = []
    for bp in blueprint.active_blueprints:
        module_specs.append((bp.module, gc, bp.kwargs.copy()))

    module_coordinator.deploy_parallel(module_specs, blueprint_args)

    for bp in blueprint.active_blueprints:
        module_coordinator._deployed_atoms[bp.module] = bp


def _ref_msg(module_name: str, ref: object, spec_name: str, detail: str) -> str:
    return (
        f"{module_name} has a module reference ({ref}) requesting a module that "
        f"satisfies the {spec_name} spec. {detail}"
    )


def _resolve_single_ref(
    bp: Any,
    module_ref: Any,
    spec: Any,
    blueprint: Blueprint,
    disabled_set: set[type],
    existing_modules: set[type[ModuleBase]] | None = None,
) -> Any:
    """Resolve a module ref to its provider.

    Returns a module type, a ``DisabledModuleProxy``, or *None* (skip).
    """
    from dimos.core.coordination.blueprints import DisabledModuleProxy

    m = bp.module.__name__
    s = module_ref.spec.__name__

    possible = [
        other.module
        for other in blueprint.active_blueprints
        if other != bp and spec_structural_compliance(other.module, spec)
    ]
    if existing_modules:
        bp_module_set = {o.module for o in blueprint.active_blueprints}
        for mod_cls in existing_modules:
            if (
                mod_cls != bp.module
                and mod_cls not in bp_module_set
                and spec_structural_compliance(mod_cls, spec)
            ):
                possible.append(mod_cls)
    valid = [c for c in possible if spec_annotation_compliance(c, spec)]

    if not possible:
        if module_ref.optional:
            return None
        disabled = next(
            (
                other.module
                for other in blueprint.blueprints
                if other.module in disabled_set and spec_structural_compliance(other.module, spec)
            ),
            None,
        )
        if disabled is not None:
            logger.warning(
                "Module ref unsatisfied because provider is disabled; installing no-op proxy",
                ref=module_ref.name,
                consumer=m,
                disabled_provider=disabled.__name__,
                spec=s,
            )
            return DisabledModuleProxy(s)
        raise Exception(_ref_msg(m, module_ref, s, "No module met that spec."))

    if len(possible) == 1:
        if not valid:
            logger.warning(
                _ref_msg(
                    m,
                    module_ref,
                    s,
                    f"{possible[0].__name__} met the spec structurally but had "
                    f"annotation mismatches.\nPlease either change the {s} spec "
                    f"or the {possible[0].__name__} module.",
                )
            )
        return possible[0]

    if len(valid) == 1:
        return valid[0]

    if len(valid) > 1:
        raise Exception(
            _ref_msg(
                m,
                module_ref,
                s,
                f"Multiple modules met that spec: {valid}.\n"
                f"To fix this use .remappings, for example:\n"
                f"    autoconnect(...).remappings([ ({m}, {module_ref.name!r}, "
                f"<ModuleThatHasTheRpcCalls>) ])",
            )
        )

    names = ", ".join(c.__name__ for c in possible)
    raise Exception(
        _ref_msg(
            m,
            module_ref,
            s,
            f"Some modules ({names}) met the spec structurally but had annotation mismatches.",
        )
    )


def _connect_module_refs(
    blueprint: Blueprint,
    module_coordinator: ModuleCoordinator,
    existing_modules: set[type[ModuleBase]] | None = None,
) -> None:
    from dimos.core.coordination.blueprints import DisabledModuleProxy
    from dimos.core.module import is_module_type
    from dimos.core.rpc_client import AsyncSpecProxy

    mod_and_mod_ref_to_proxy = {
        (module, name): replacement
        for (module, name), replacement in blueprint.remapping_map.items()
        if is_spec(replacement) or is_module_type(replacement)
    }

    # Track the consumer's declared spec for each ref so we can wrap the proxy
    # below if the spec contains async-declared methods.
    declared_spec: dict[tuple[type[ModuleBase], str], Any] = {}

    disabled_ref_proxies: dict[tuple[type[ModuleBase], str], DisabledModuleProxy] = {}
    disabled_set = set(blueprint.disabled_modules_tuple)

    for bp in blueprint.active_blueprints:
        for module_ref in bp.module_refs:
            declared_spec[bp.module, module_ref.name] = module_ref.spec
            spec = mod_and_mod_ref_to_proxy.get((bp.module, module_ref.name), module_ref.spec)

            if is_module_type(spec):
                mod_and_mod_ref_to_proxy[bp.module, module_ref.name] = spec
                continue

            result = _resolve_single_ref(
                bp, module_ref, spec, blueprint, disabled_set, existing_modules
            )
            if result is None:
                continue
            if isinstance(result, DisabledModuleProxy):
                disabled_ref_proxies[bp.module, module_ref.name] = result
            else:
                mod_and_mod_ref_to_proxy[bp.module, module_ref.name] = result

    for (base_module, ref_name), target_module in mod_and_mod_ref_to_proxy.items():
        base_instance = module_coordinator.get_instance(base_module)
        target_instance: Any = module_coordinator.get_instance(target_module)  # type: ignore[arg-type]
        async_methods = _async_methods_of_spec(declared_spec.get((base_module, ref_name)))
        if async_methods:
            target_instance = AsyncSpecProxy(target_instance, async_methods)
        setattr(base_instance, ref_name, target_instance)
        base_instance.set_module_ref(ref_name, target_instance)
        module_coordinator._resolved_module_refs[base_module, ref_name] = cast(
            "type[ModuleBase]", target_module
        )

    for (base_module, ref_name), proxy in disabled_ref_proxies.items():
        base_instance = module_coordinator.get_instance(base_module)
        setattr(base_instance, ref_name, proxy)
        base_instance.set_module_ref(ref_name, cast("Any", proxy))


def _async_methods_of_spec(spec: Any) -> frozenset[str]:
    if not is_spec(spec):
        return frozenset()
    names: set[str] = set()
    for cls in spec.__mro__:
        if cls is object:
            continue
        for attr_name, value in vars(cls).items():
            if attr_name.startswith("_"):
                continue
            if inspect.iscoroutinefunction(value):
                names.add(attr_name)
    return frozenset(names)


def _log_blueprint_graph(blueprint: Blueprint, module_coordinator: ModuleCoordinator) -> None:
    """Log the module graph to Rerun if a RerunBridgeModule is active."""
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    if not any(bp.module is RerunBridgeModule for bp in blueprint.active_blueprints):
        return

    if not shutil.which("dot"):
        logger.info(
            "graphviz not found, skipping blueprint graph. Install: sudo apt install graphviz"
        )
        return

    try:
        from dimos.core.introspection.blueprint.dot import render

        dot_code = render(blueprint)
        module_names = [bp.module.__name__ for bp in blueprint.active_blueprints]
        bridge = module_coordinator.get_instance(RerunBridgeModule)  # type: ignore[arg-type]
        bridge.log_blueprint_graph(dot_code, module_names)
    except Exception:
        logger.error("Failed to log blueprint graph to Rerun", exc_info=True)
