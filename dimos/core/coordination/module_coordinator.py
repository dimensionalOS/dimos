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

from collections import defaultdict
from collections.abc import Callable, Mapping, MutableMapping
import importlib
import inspect
import shutil
import sys
import threading
from typing import TYPE_CHECKING, Any, NamedTuple, cast

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.worker_manager import WorkerManager
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.spec.utils import is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol

logger = setup_logger()

class InstanceKey(NamedTuple):
    namespace: str | None
    module: type[ModuleBase]


class ModuleDescriptor(NamedTuple):
    """Returned by `Coordinator/list_modules` so a remote client can build a proxy."""

    class_name: str
    qualified_path: str
    rpc_names: list[str]


class ModuleCoordinator(Resource):
    _managers: dict[str, WorkerManager]
    _global_config: GlobalConfig

    _deployed_modules: dict[InstanceKey, ModuleProxyProtocol]

    def __init__(
        self,
        g: GlobalConfig = global_config,
    ) -> None:
        self._global_config = g
        manager_types: list[type[WorkerManager]] = [WorkerManagerPython]
        self._managers = {cls.deployment_identifier: cls(g=g) for cls in manager_types}
        self._deployed_modules = {}
        self._deployed_atoms: dict[InstanceKey, BlueprintAtom] = {}
        self._resolved_module_refs: dict[tuple[InstanceKey, str], InstanceKey] = {}
        self._transport_registry: dict[tuple[str | None, str, type], PubSubTransport[Any]] = {}
        self._class_aliases: dict[InstanceKey, InstanceKey] = {}
        self._module_transports: dict[InstanceKey, dict[str, PubSubTransport[Any]]] = {}
        self._started = False
        self._modules_lock = threading.RLock()
        self._coordinator_rpc: CoordinatorRPC | None = None

    def start(self) -> None:
        from dimos.core.o3dpickle import register_picklers

        register_picklers()
        for m in self._managers.values():
            m.start()
        self._started = True

    def stop(self) -> None:
        if self._coordinator_rpc is not None:
            self._coordinator_rpc.stop()
            self._coordinator_rpc = None

        for key, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=key.module.__name__)
            try:
                module.stop()
            except Exception:
                logger.error("Error stopping module", module=key.module.__name__, exc_info=True)
            logger.info("Module stopped.", module=key.module.__name__)

        def _stop_manager(m: WorkerManager) -> None:
            try:
                m.stop()
            except Exception:
                logger.error("Error stopping manager", manager=type(m).__name__, exc_info=True)

        safe_thread_map(tuple(self._managers.values()), _stop_manager)

    def start_rpc_service(self) -> None:
        """Expose the coordinator's API as @rpc methods over LCM."""
        if self._coordinator_rpc is not None:
            return
        self._coordinator_rpc = CoordinatorRPC.serve(self)

    @property
    def rpcs(self) -> dict[str, Callable[..., Any]]:
        """Methods exposed via the Coordinator @rpc service."""
        return {
            "ping": self.ping,
            "list_modules": self.list_modules,
            "load_blueprint_by_name": self.load_blueprint_by_name,
            "load_blueprint": self.load_blueprint,
            "restart_module_by_class_name": self.restart_module_by_class_name,
        }

    def ping(self) -> str:
        """Used by clients to check if the coordinator is alive and responsive."""
        return "pong"

    def list_modules(self) -> list[ModuleDescriptor]:
        with self._modules_lock:
            descriptors: list[ModuleDescriptor] = []
            for key in self._deployed_modules:
                cls = key.module
                qualified = f"{cls.__module__}.{cls.__name__}"
                ns_prefix = f"{key.namespace}/" if key.namespace else ""
                descriptors.append(
                    ModuleDescriptor(
                        class_name=f"{ns_prefix}{cls.__name__}",
                        qualified_path=qualified,
                        rpc_names=list(cls.rpcs.keys()),
                    )
                )
            return descriptors

    def load_blueprint_by_name(self, name: str) -> None:
        # Avoid circular import.
        from dimos.robot.get_all_blueprints import get_by_name

        self.load_blueprint(get_by_name(name))

    def list_module_names(self) -> list[str]:
        with self._modules_lock:
            return [
                f"{k.namespace}/{k.module.__name__}" if k.namespace else k.module.__name__
                for k in self._deployed_modules
            ]

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

        # patch the proxy hardcoded remote name so it correctly targets the namespace
        rpc_name = kwargs.get("__dimos_rpc_name__")
        if rpc_name and hasattr(deployed_module, "remote_name"):
            deployed_module.remote_name = rpc_name

        with self._modules_lock:
            ns = kwargs.get("__dimos_namespace__")
            key = InstanceKey(ns, module_class)
            self._deployed_modules[key] = deployed_module
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
            for i, (original_index, module) in enumerate(
                zip(indices_by_deployment[dep], deployed, strict=True)
            ):
                if module is not None:
                    spec_kwargs = specs_by_deployment[dep][i][2]
                    rpc_name = spec_kwargs.get("__dimos_rpc_name__")
                    if rpc_name and hasattr(module, "remote_name"):
                        module.remote_name = rpc_name

                results[original_index] = module

        try:
            safe_thread_map(list(specs_by_deployment.keys()), _deploy_group)
        except:
            self.stop()
            raise

        with self._modules_lock:
            self._deployed_modules.update(
                {
                    InstanceKey(kwargs.get("__dimos_namespace__"), cls): mod
                    for (cls, _, kwargs), mod in zip(module_specs, results, strict=True)
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

    def _resolve_key(self, key: InstanceKey) -> InstanceKey:
        return self._class_aliases.get(key, key)

    def get_instance_by_key(self, key_or_cls: InstanceKey | type[ModuleBase]) -> ModuleProxy:
        key = key_or_cls if isinstance(key_or_cls, InstanceKey) else InstanceKey(None, key_or_cls)
        return self._deployed_modules.get(self._resolve_key(key))  # type: ignore[return-value]

    def get_instance(self, module_or_key: type[ModuleBase] | InstanceKey) -> ModuleProxy:
        if isinstance(module_or_key, InstanceKey):
            return self.get_instance_by_key(module_or_key)
        return self.get_instance_by_key(InstanceKey(None, module_or_key))

    def _send_on_system_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        for module in modules:
            if hasattr(module, "on_system_modules"):
                module.on_system_modules(modules)

    def _connect_streams(self, blueprint: Blueprint) -> None:
        streams: dict[tuple[str | None, str, type], list[tuple[InstanceKey, str]]] = defaultdict(
            list
        )

        for bp in blueprint.active_blueprints:
            key = InstanceKey(bp.namespace, bp.module)
            for conn in bp.streams:
                remapped_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    streams[bp.namespace, remapped_name, conn.type].append((key, conn.name))

        for namespace, remapped_name, stream_type in streams.keys():
            map_key = (namespace, remapped_name, stream_type)
            if map_key in self._transport_registry:
                transport = self._transport_registry[map_key]
            else:
                transport = _get_transport_for(blueprint, remapped_name, stream_type, namespace)
            self._transport_registry[map_key] = transport
            for key, original_name in streams[namespace, remapped_name, stream_type]:
                instance = self.get_instance_by_key(key)  # type: ignore[assignment]
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]
                self._module_transports.setdefault(key, {})[original_name] = transport
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{stream_type.__module__}.{stream_type.__qualname__}",
                    module=key.module.__name__,
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

        _log_blueprint_graph(blueprint, coordinator)

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
            key = InstanceKey(bp.namespace, bp.module)
            if key in self._deployed_modules:
                raise ValueError(
                    f"{bp.module.__name__} in namespace {bp.namespace} is already deployed; cannot load the same module twice"
                )

        before = set(self._deployed_modules)

        _deploy_all_modules(blueprint, self, self._global_config, blueprint_args)
        self._connect_streams(blueprint)
        _connect_module_refs(blueprint, self, existing_modules=before)

        new_modules = [proxy for key, proxy in self._deployed_modules.items() if key not in before]

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
            self._unload_module(InstanceKey(None, module_class))

    def _unload_module(self, key_or_cls: InstanceKey | type[ModuleBase]) -> None:
        key = key_or_cls if isinstance(key_or_cls, InstanceKey) else InstanceKey(None, key_or_cls)
        key = self._resolve_key(key)
        module_class = key.module
        if key not in self._deployed_modules:
            raise ValueError(f"{module_class.__name__} is not deployed")
        if module_class.deployment != "python":
            raise NotImplementedError(
                f"unload_module only supports python deployment, got {module_class.deployment!r}"
            )

        proxy = self._deployed_modules[key]

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

        del self._deployed_modules[key]
        self._deployed_atoms.pop(key, None)
        self._module_transports.pop(key, None)
        self._class_aliases = {k: v for k, v in self._class_aliases.items() if v != key}
        self._resolved_module_refs = {
            k: target
            for k, target in self._resolved_module_refs.items()
            if k[0] != key and target != key
        }

    def restart_module_by_class_name(
        self,
        class_name: str,
        *,
        namespace: str | None = None,
        reload_source: bool = True,
    ) -> None:
        with self._modules_lock:
            for key in self._deployed_modules:
                if key.module.__name__ == class_name and key.namespace == namespace:
                    self._restart_module(key, reload_source=reload_source)
                    return
        ns_prefix = f"{namespace}/" if namespace else ""
        raise ValueError(f"No deployed module with name {ns_prefix}{class_name!r}")

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
            return self._restart_module(
                InstanceKey(None, module_class), reload_source=reload_source
            )

    def _restart_module(
        self,
        key_or_cls: InstanceKey | type[ModuleBase],
        *,
        reload_source: bool = True,
    ) -> ModuleProxyProtocol:
        key = key_or_cls if isinstance(key_or_cls, InstanceKey) else InstanceKey(None, key_or_cls)
        key = self._resolve_key(key)
        module_class = key.module
        if key not in self._deployed_modules:
            raise ValueError(f"{module_class.__name__} is not deployed")
        if module_class.deployment != "python":
            raise NotImplementedError(
                f"restart_module only supports python deployment, got {module_class.deployment!r}"
            )

        old_atom = self._deployed_atoms[key]
        kwargs = dict(old_atom.kwargs)
        saved_transports = dict(self._module_transports.get(key, {}))

        inbound_refs = [
            (consumer_key, ref_name)
            for (consumer_key, ref_name), target in self._resolved_module_refs.items()
            if target == key
        ]
        outbound_refs = [
            (ref_name, target)
            for (consumer_key, ref_name), target in self._resolved_module_refs.items()
            if consumer_key == key
        ]

        self._unload_module(key)

        if reload_source:
            source_mod = sys.modules.get(module_class.__module__)
            if source_mod is None:
                source_mod = importlib.import_module(module_class.__module__)
            importlib.reload(source_mod)
            new_class = cast("type[ModuleBase]", getattr(source_mod, module_class.__name__))
        else:
            new_class = module_class

        new_key = InstanceKey(key.namespace, new_class)
        if new_key != key:
            for old_key in list(self._class_aliases):
                if self._class_aliases[old_key] == key:
                    self._class_aliases[old_key] = new_key
            self._class_aliases[key] = new_key

        kwargs["__dimos_namespace__"] = key.namespace
        kwargs["__dimos_rpc_name__"] = (
            f"{key.namespace}/{new_class.__name__}" if key.namespace else new_class.__name__
        )

        python_wm = cast("WorkerManagerPython", self._managers["python"])
        new_proxy = python_wm.deploy_fresh(new_class, self._global_config, kwargs)
        if hasattr(new_proxy, "remote_name"):
            new_proxy.remote_name = kwargs["__dimos_rpc_name__"]

        self._deployed_modules[new_key] = new_proxy

        # Strip internal framework keys before calling blueprint() so they don't pollute kwargs
        clean_kwargs = {k: v for k, v in kwargs.items() if not k.startswith("__dimos_")}
        new_bp = new_class.blueprint(**clean_kwargs)
        new_atom = new_bp.active_blueprints[0]
        self._deployed_atoms[new_key] = new_atom

        for stream_ref in new_atom.streams:
            transport = saved_transports.get(stream_ref.name)
            if transport is not None:
                new_proxy.set_transport(stream_ref.name, transport)
        self._module_transports[new_key] = {
            s.name: t for s in new_atom.streams if (t := saved_transports.get(s.name)) is not None
        }

        for consumer_key, ref_name in inbound_refs:
            consumer_proxy = self._deployed_modules.get(consumer_key)
            if consumer_proxy is None:
                continue
            setattr(consumer_proxy, ref_name, new_proxy)
            consumer_proxy.set_module_ref(ref_name, new_proxy)  # type: ignore[attr-defined]
            self._resolved_module_refs[consumer_key, ref_name] = new_key

        for ref_name, target_key in outbound_refs:
            target_proxy = self._deployed_modules.get(target_key)
            if target_proxy is None:
                continue
            setattr(new_proxy, ref_name, target_proxy)
            new_proxy.set_module_ref(ref_name, target_proxy)  # type: ignore[attr-defined]
            self._resolved_module_refs[new_key, ref_name] = target_key

        new_proxy.build()
        new_proxy.start()

        self._send_on_system_modules()

        return new_proxy

    def loop(self) -> None:
        stop = threading.Event()
        try:
            stop.wait()
        except KeyboardInterrupt:
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


def _is_name_unique(blueprint: Blueprint, name: str, namespace: str | None = None) -> bool:
    count = 0
    for bp in blueprint.active_blueprints:
        if bp.namespace == namespace:
            for conn in bp.streams:
                remapped = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
                if remapped == name:
                    count += 1
    return count == 1


def _get_transport_for(
    blueprint: Blueprint, name: str, stream_type: type, namespace: str | None = None
) -> PubSubTransport[Any]:
    transport = blueprint.transport_map.get((name, stream_type), None)
    if transport:
        return transport

    use_pickled = getattr(stream_type, "lcm_encode", None) is None

    topic_base = f"/{namespace}/{name}" if namespace else f"/{name}"
    topic = topic_base if _is_name_unique(blueprint, name, namespace) else f"/{short_id()}"

    transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)
    return transport


def _verify_no_name_conflicts(blueprint: Blueprint) -> None:
    name_to_types: dict[Any, set[type]] = defaultdict(set)
    name_to_modules: dict[Any, list[tuple[type, type]]] = defaultdict(list)

    for bp in blueprint.active_blueprints:
        for conn in bp.streams:
            stream_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
            isolated_name = f"{bp.namespace}/{stream_name}" if bp.namespace else stream_name
            name_to_types[isolated_name].add(conn.type)
            name_to_modules[isolated_name].append((bp.module, conn.type))

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
    existing_registry: dict[tuple[str | None, str, type], PubSubTransport[Any]],
) -> None:
    """Check that a new blueprint's streams don't conflict with already-registered transports."""
    if not existing_registry:
        return

    existing_names: dict[tuple[str | None, str], set[type]] = defaultdict(set)
    for namespace, name, stream_type in existing_registry:
        existing_names[(namespace, name)].add(stream_type)

    for bp in blueprint.active_blueprints:
        for conn in bp.streams:
            remapped_name = blueprint.remapping_map.get((bp.module, conn.name), conn.name)
            if isinstance(remapped_name, str) and (bp.namespace, remapped_name) in existing_names:
                for existing_type in existing_names[(bp.namespace, remapped_name)]:
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
        kwargs = bp.kwargs.copy()
        kwargs["__dimos_namespace__"] = bp.namespace
        kwargs["__dimos_rpc_name__"] = (
            f"{bp.namespace}/{bp.module.__name__}" if bp.namespace else bp.module.__name__
        )
        module_specs.append((bp.module, gc, kwargs))

    module_coordinator.deploy_parallel(module_specs, blueprint_args)

    for bp in blueprint.active_blueprints:
        key = InstanceKey(bp.namespace, bp.module)
        module_coordinator._deployed_atoms[key] = bp


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
    existing_modules: set[InstanceKey] | None = None,
) -> Any:
    """Resolve a module ref to its provider.

    Returns a module type, a ``DisabledModuleProxy``, or *None* (skip).
    """
    from dimos.core.coordination.blueprints import DisabledModuleProxy

    m = bp.module.__name__
    s = module_ref.spec.__name__

    possible = [
        InstanceKey(other.namespace, other.module)
        for other in blueprint.active_blueprints
        if other != bp and spec_structural_compliance(other.module, spec)
    ]
    if existing_modules:
        bp_module_set = {InstanceKey(o.namespace, o.module) for o in blueprint.active_blueprints}
        for key in existing_modules:
            if (
                key != InstanceKey(bp.namespace, bp.module)
                and key not in bp_module_set
                and spec_structural_compliance(key.module, spec)
            ):
                possible.append(key)
    valid = [c for c in possible if spec_annotation_compliance(c.module, spec)]

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
                    f"{possible[0].module.__name__} met the spec structurally but had "
                    f"annotation mismatches.\nPlease either change the {s} spec "
                    f"or the {possible[0].module.__name__} module.",
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

    names = ", ".join(c.module.__name__ for c in possible)
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
    existing_modules: set[InstanceKey] | None = None,
) -> None:
    from dimos.core.coordination.blueprints import DisabledModuleProxy
    from dimos.core.module import is_module_type
    from dimos.core.rpc_client import AsyncSpecProxy

    mod_and_mod_ref_to_proxy: dict[tuple[InstanceKey, str], InstanceKey] = {}
    declared_spec: dict[tuple[InstanceKey, str], Any] = {}
    disabled_ref_proxies: dict[tuple[InstanceKey, str], DisabledModuleProxy] = {}
    disabled_set = set(blueprint.disabled_modules_tuple)

    for bp in blueprint.active_blueprints:
        base_key = InstanceKey(bp.namespace, bp.module)
        for module_ref in bp.module_refs:
            declared_spec[base_key, module_ref.name] = module_ref.spec
            spec = blueprint.remapping_map.get((bp.module, module_ref.name), module_ref.spec)

            if is_module_type(spec):
                target_key = InstanceKey(bp.namespace, cast("type[ModuleBase]", spec))
                if target_key not in module_coordinator._deployed_modules:
                    target_key = InstanceKey(None, cast("type[ModuleBase]", spec))
                mod_and_mod_ref_to_proxy[base_key, module_ref.name] = target_key
                continue

            result = _resolve_single_ref(
                bp, module_ref, spec, blueprint, disabled_set, existing_modules
            )
            if result is None:
                continue
            if isinstance(result, DisabledModuleProxy):
                disabled_ref_proxies[base_key, module_ref.name] = result
            else:
                target_key = result
                if target_key not in module_coordinator._deployed_modules:
                    target_key = InstanceKey(None, target_key.module)
                mod_and_mod_ref_to_proxy[base_key, module_ref.name] = target_key

    for bp in blueprint.active_blueprints:
        base_key = InstanceKey(bp.namespace, bp.module)
        for module_ref in bp.module_refs:
            ref_name = module_ref.name
            if (base_key, ref_name) not in mod_and_mod_ref_to_proxy:
                continue

            target_key = mod_and_mod_ref_to_proxy[base_key, ref_name]

            base_instance = module_coordinator.get_instance_by_key(base_key)
            target_instance: Any = module_coordinator.get_instance_by_key(target_key)

            if target_instance is None:
                logger.error(
                    f"Failed to wire {base_key} -> {ref_name}. Target {target_key} not found."
                )
                continue

            async_methods = _async_methods_of_spec(declared_spec.get((base_key, ref_name)))
            if async_methods:
                target_instance = AsyncSpecProxy(target_instance, async_methods)

            setattr(base_instance, ref_name, target_instance)
            base_instance.set_module_ref(ref_name, target_instance)
            module_coordinator._resolved_module_refs[base_key, ref_name] = target_key

    for (base_key, ref_name), proxy in disabled_ref_proxies.items():
        base_instance = module_coordinator.get_instance_by_key(base_key)
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