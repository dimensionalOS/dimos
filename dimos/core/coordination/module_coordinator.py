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
from dataclasses import dataclass
import importlib
import inspect
import shutil
import sys
import threading
from typing import TYPE_CHECKING, Any, NamedTuple, cast

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.coordination.worker_launcher import CommandWorkerLauncher, VenvWorkerLauncher
from dimos.core.coordination.worker_manager import WorkerManager
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleBase, ModuleConfig, ModuleIOContract, ModuleSpec, StreamDecl
from dimos.core.resource import Resource
from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    PythonProjectRuntimeEnvironmentError,
    RuntimeEnvironmentRegistry,
)
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.spec.utils import is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
    from dimos.core.rpc_client import ModuleProxy, ModuleProxyProtocol

logger = setup_logger()


class ModuleDescriptor(NamedTuple):
    """Returned by `Coordinator/list_modules` so a remote client can build a proxy."""

    class_name: str
    qualified_path: str
    rpc_names: list[str]


@dataclass(frozen=True)
class ResolvedModulePlan:
    atom: BlueprintAtom
    module: type[ModuleBase]
    final_kwargs: dict[str, Any]
    config: ModuleConfig
    io_contract: ModuleIOContract

    @property
    def streams(self) -> tuple[StreamDecl, ...]:
        return self.io_contract.streams


class ModuleCoordinator(Resource):
    _managers: dict[str, WorkerManager]
    _global_config: GlobalConfig
    _deployed_modules: dict[type[ModuleBase], ModuleProxyProtocol]

    def __init__(
        self,
        g: GlobalConfig = global_config,
    ) -> None:
        self._global_config = g
        manager_types: list[type[WorkerManager]] = [WorkerManagerPython]
        self._managers = {cls.deployment_identifier: cls(g=g) for cls in manager_types}
        self._deployed_modules = {}
        self._deployed_atoms: dict[type[ModuleBase], BlueprintAtom] = {}
        self._resolved_module_plans: dict[type[ModuleBase], ResolvedModulePlan] = {}
        self._resolved_module_refs: dict[tuple[type[ModuleBase], str], type[ModuleBase]] = {}
        self._transport_registry: dict[tuple[str, type], PubSubTransport[Any]] = {}
        self._class_aliases: dict[type[ModuleBase], type[ModuleBase]] = {}
        self._module_transports: dict[type[ModuleBase], dict[str, PubSubTransport[Any]]] = {}
        self._runtime_environment_registry = RuntimeEnvironmentRegistry.with_current_process()
        self._runtime_placement_map: dict[type[ModuleBase], str] = {}
        self._module_manager_keys: dict[type[ModuleBase], str] = {}
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
            for cls in self._deployed_modules:
                qualified = f"{cls.__module__}.{cls.__name__}"
                descriptors.append(
                    ModuleDescriptor(
                        class_name=cls.__name__,
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
            return [cls.__name__ for cls in self._deployed_modules]

    def health_check(self) -> bool:
        return all(m.health_check() for m in self._active_managers().values())

    def _active_managers(self) -> dict[str, WorkerManager]:
        return {
            key: manager
            for key, manager in self._managers.items()
            if key in self._module_manager_keys.values()
            or (key == "python" and bool(cast("WorkerManagerPython", manager).workers))
        }

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

        manager_key = self._manager_key_for_module(module_class)
        kwargs = self._inject_native_runtime_registry(module_class, kwargs)
        deployed_module = self._managers[manager_key].deploy(module_class, global_config, kwargs)
        with self._modules_lock:
            self._deployed_modules[module_class] = deployed_module
            self._module_manager_keys[module_class] = manager_key
        return deployed_module  # type: ignore[return-value]

    def _manager_key_for_module(self, module_class: type[ModuleBase]) -> str:
        if module_class.deployment != "python":
            return module_class.deployment
        env_name = self._runtime_placement_map.get(module_class)
        if env_name is None:
            return "python"
        return self._ensure_venv_manager(env_name)

    def _ensure_venv_manager(self, env_name: str) -> str:
        manager_key = f"python:{env_name}"
        if manager_key in self._managers:
            return manager_key
        try:
            environment = self._runtime_environment_registry.resolve(env_name)
        except Exception as exc:
            raise RuntimeError(
                f"Module placement references runtime environment {env_name!r}, but it could not "
                "be resolved as a Python runtime capability. Register a Python runtime environment on the "
                "blueprint with .runtime_environments(...)."
            ) from exc
        if isinstance(environment, PythonProjectRuntimeEnvironment):
            return self._ensure_project_manager(env_name, environment, manager_key)
        try:
            material = environment.resolve_python()
        except Exception as exc:
            raise RuntimeError(
                f"Module placement references runtime environment {env_name!r}, but it could not "
                "be resolved as a Python runtime capability. Register a Python runtime environment on the "
                "blueprint with .runtime_environments(...)."
            ) from exc
        executable = str(material.python_executable)
        if material.python_executable.is_absolute():
            executable_found = material.python_executable.exists()
        else:
            executable_found = shutil.which(executable) is not None
        if not executable_found:
            raise RuntimeError(
                f"Runtime environment {env_name!r} Python capability references missing "
                f"executable {executable!r}."
            )
        manager = WorkerManagerPython(
            g=self._global_config,
            worker_launcher=VenvWorkerLauncher(python_executable=executable, env=material.env),
        )
        if self._started:
            try:
                manager.start()
            except Exception as exc:
                manager.stop()
                raise RuntimeError(
                    f"Failed to start runtime environment {env_name!r} Python capability "
                    f"with executable {executable!r}."
                ) from exc
        self._managers[manager_key] = manager
        return manager_key

    def _ensure_project_manager(
        self,
        env_name: str,
        environment: PythonProjectRuntimeEnvironment,
        manager_key: str,
    ) -> str:
        try:
            material = environment.resolve_python_project()
        except PythonProjectRuntimeEnvironmentError:
            raise
        except Exception as exc:
            raise RuntimeError(
                f"Module placement references runtime environment {env_name!r}, but it could not "
                "be resolved as a Python project runtime capability. Register a Python project runtime "
                "environment on the blueprint with .runtime_environments(...)."
            ) from exc
        manager = WorkerManagerPython(
            g=self._global_config,
            worker_launcher=CommandWorkerLauncher(material=material),
        )
        if self._started:
            try:
                manager.start()
            except Exception as exc:
                manager.stop()
                raise RuntimeError(
                    f"Failed to start runtime environment {env_name!r} Python project capability "
                    f"with command prefix {material.argv_prefix!r} in project {material.cwd!s}."
                ) from exc
        self._managers[manager_key] = manager
        return manager_key

    def _inject_native_runtime_registry(
        self, module_class: type[ModuleBase], kwargs: Mapping[str, Any]
    ) -> dict[str, Any]:
        from dimos.core.native_module import NativeModule

        result = dict(kwargs)
        if issubclass(module_class, NativeModule):
            result["runtime_environment_registry"] = self._runtime_environment_registry
        return result

    def deploy_parallel(
        self,
        module_specs: list[ModuleSpec],
        runtime_environment_registry: RuntimeEnvironmentRegistry | None = None,
        runtime_placement_map: Mapping[type[ModuleBase], str] | None = None,
    ) -> list[ModuleProxy]:
        if not self._managers:
            raise ValueError("Not started")

        if runtime_environment_registry is not None:
            self._runtime_environment_registry = self._runtime_environment_registry.merge(
                runtime_environment_registry
            )
        previous_placements = dict(self._runtime_placement_map)
        existing_manager_keys = set(self._managers)
        module_classes = {spec[0] for spec in module_specs}
        active_runtime_placement_map = (
            {
                module_class: env_name
                for module_class, env_name in runtime_placement_map.items()
                if module_class in module_classes
            }
            if runtime_placement_map is not None
            else None
        )
        if active_runtime_placement_map is not None:
            self._runtime_placement_map.update(active_runtime_placement_map)

        try:
            # Group specs by deployment manager, tracking original indices for reassembly
            indices_by_deployment: dict[str, list[int]] = {}
            specs_by_deployment: dict[str, list[ModuleSpec]] = {}
            effective_module_specs: list[ModuleSpec] = []
            for index, spec in enumerate(module_specs):
                module_class, spec_global_config, kwargs = spec
                dep = self._manager_key_for_module(module_class)
                effective_kwargs = dict(kwargs)
                effective_kwargs = self._inject_native_runtime_registry(
                    module_class, effective_kwargs
                )
                spec = (module_class, spec_global_config, effective_kwargs)
                effective_module_specs.append(spec)
                indices_by_deployment.setdefault(dep, []).append(index)
                specs_by_deployment.setdefault(dep, []).append(spec)
        except Exception:
            self._runtime_placement_map = previous_placements
            for key in set(self._managers) - existing_manager_keys:
                self._stop_and_remove_manager(key)
            raise

        results: list[Any] = [None] * len(module_specs)

        def _deploy_group(dep: str) -> None:
            try:
                deployed = self._managers[dep].deploy_parallel(specs_by_deployment[dep])
            except Exception as exc:
                if dep.startswith("python:"):
                    env_name = dep.removeprefix("python:")
                    try:
                        environment = self._runtime_environment_registry.resolve(env_name)
                        if isinstance(environment, PythonProjectRuntimeEnvironment):
                            executable = " ".join(environment.resolve_python_project().argv_prefix)
                        else:
                            executable = str(environment.resolve_python().python_executable)
                    except Exception:
                        executable = "<unresolved>"
                    raise RuntimeError(
                        f"Failed to deploy with runtime environment {env_name!r} Python "
                        f"capability using executable {executable!r}."
                    ) from exc
                raise
            for index, module in zip(indices_by_deployment[dep], deployed, strict=True):
                results[index] = module

        try:
            safe_thread_map(list(specs_by_deployment.keys()), _deploy_group)
        except:
            self.stop()
            self._runtime_placement_map = previous_placements
            for key in set(self._managers) - existing_manager_keys:
                self._stop_and_remove_manager(key)
            raise

        with self._modules_lock:
            self._deployed_modules.update(
                {
                    cls: mod
                    for (cls, _, _), mod in zip(effective_module_specs, results, strict=True)
                    if mod is not None
                }
            )
            for (cls, _, _), mod in zip(effective_module_specs, results, strict=True):
                if mod is not None:
                    self._module_manager_keys[cls] = self._manager_key_for_module(cls)
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

    def _connect_streams(self, blueprint: Blueprint, plans: tuple[ResolvedModulePlan, ...]) -> None:
        streams: dict[tuple[str, type], list[tuple[type, str]]] = defaultdict(list)

        for plan in plans:
            for conn in plan.streams:
                remapped_name = blueprint.remapping_map.get((plan.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    streams[remapped_name, conn.type].append((plan.module, conn.name))

        for remapped_name, stream_type in streams.keys():
            key = (remapped_name, stream_type)
            if key in self._transport_registry:
                transport = self._transport_registry[key]
            else:
                transport = _get_transport_for(blueprint, plans, remapped_name, stream_type)
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
            global_config.update(**dict(blueprint_args["g"]))

        _run_configurators(blueprint)
        _check_requirements(blueprint)
        plans = _resolve_module_plans(blueprint, global_config, blueprint_args)
        _verify_stream_remappings(blueprint, plans)
        _verify_no_name_conflicts(blueprint, plans)

        logger.info("Starting the modules")
        coordinator = cls(g=global_config)
        coordinator.start()

        try:
            _deploy_all_modules(plans, coordinator, global_config, blueprint)
            coordinator._connect_streams(blueprint, plans)
            _connect_module_refs(blueprint, coordinator)

            coordinator.build_all_modules()
            coordinator.start_all_modules()

            _log_blueprint_graph(blueprint, coordinator, plans)
        except Exception:
            coordinator.stop()
            raise

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
            self._global_config.update(**dict(blueprint_args["g"]))

        # Scale worker pool.
        n_extra = int(blueprint.global_config_overrides.get("n_workers", 0))
        python_wm = cast("WorkerManagerPython", self._managers["python"])
        if n_extra:
            python_wm.add_workers(n_extra)
        if not python_wm.workers and blueprint.active_blueprints:
            python_wm.add_workers(1)

        _run_configurators(blueprint)
        _check_requirements(blueprint)
        plans = _resolve_module_plans(blueprint, self._global_config, blueprint_args)
        _verify_stream_remappings(blueprint, plans)
        _verify_no_name_conflicts(blueprint, plans)
        _verify_no_conflicts_with_existing(blueprint, self._transport_registry, plans)

        # Reject duplicate modules.
        for bp in blueprint.active_blueprints:
            if bp.module in self._deployed_modules:
                raise ValueError(
                    f"{bp.module.__name__} is already deployed; cannot load the same module twice"
                )

        before = set(self._deployed_modules)

        _deploy_all_modules(plans, self, self._global_config, blueprint)
        self._connect_streams(blueprint, plans)
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
        self._unload_module_impl(module_class, preserve_placement=False)

    def _unload_module_impl(
        self, module_class: type[ModuleBase], *, preserve_placement: bool
    ) -> None:
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

        manager_key = self._module_manager_keys.get(
            module_class, self._manager_key_for_module(module_class)
        )
        python_wm = cast("WorkerManagerPython", self._managers[manager_key])
        try:
            python_wm.undeploy(proxy)
        except Exception:
            logger.error(
                "Error undeploying module from worker",
                module=module_class.__name__,
                exc_info=True,
            )

        del self._deployed_modules[module_class]
        self._module_manager_keys.pop(module_class, None)
        if not preserve_placement:
            self._clear_runtime_placement_aliases(module_class)
        self._deployed_atoms.pop(module_class, None)
        self._resolved_module_plans.pop(module_class, None)
        self._module_transports.pop(module_class, None)
        self._class_aliases = {
            k: v for k, v in self._class_aliases.items() if v is not module_class
        }
        self._resolved_module_refs = {
            key: target
            for key, target in self._resolved_module_refs.items()
            if key[0] is not module_class and target is not module_class
        }
        if (
            manager_key.startswith("python:")
            and manager_key not in self._module_manager_keys.values()
        ):
            self._stop_and_remove_manager(manager_key)

    def _clear_runtime_placement_aliases(self, module_class: type[ModuleBase]) -> None:
        """Clear placement for a class and stale handles that alias to it."""
        self._runtime_placement_map.pop(module_class, None)
        for alias_class, resolved_class in list(self._class_aliases.items()):
            if resolved_class is module_class:
                self._runtime_placement_map.pop(alias_class, None)

    def _stop_and_remove_manager(self, manager_key: str) -> None:
        manager = self._managers.pop(manager_key, None)
        if manager is None:
            return
        try:
            manager.stop()
        except Exception:
            logger.error("Error stopping manager", manager=manager_key, exc_info=True)

    def restart_module_by_class_name(
        self,
        class_name: str,
        *,
        reload_source: bool = True,
    ) -> None:
        with self._modules_lock:
            for cls in self._deployed_modules:
                if cls.__name__ == class_name:
                    self._restart_module(cls, reload_source=reload_source)
                    return
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

        old_plan = self._resolved_module_plans[module_class]
        kwargs = dict(old_plan.final_kwargs)
        kwargs["g"] = self._global_config
        runtime_env_name = self._runtime_placement_map.get(module_class)
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

        self._unload_module_impl(module_class, preserve_placement=True)

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
            if runtime_env_name is not None:
                self._runtime_placement_map.pop(module_class, None)
                self._runtime_placement_map[new_class] = runtime_env_name

        manager_key = self._module_manager_keys.get(new_class) or self._manager_key_for_module(
            new_class
        )
        python_wm = cast("WorkerManagerPython", self._managers[manager_key])
        deployment_kwargs = self._inject_native_runtime_registry(new_class, kwargs)
        new_proxy = python_wm.deploy_fresh(new_class, self._global_config, deployment_kwargs)
        self._deployed_modules[new_class] = new_proxy
        self._module_manager_keys[new_class] = manager_key

        blueprint_kwargs = {k: v for k, v in kwargs.items() if k != "g"}
        new_bp = new_class.blueprint(**blueprint_kwargs)
        new_atom = new_bp.active_blueprints[0]
        self._deployed_atoms[new_class] = new_atom
        config = new_class.resolve_config(kwargs)
        new_plan = ResolvedModulePlan(
            atom=new_atom,
            module=new_class,
            final_kwargs=kwargs,
            config=config,
            io_contract=new_class.io_contract(config),
        )
        self._resolved_module_plans[new_class] = new_plan

        for stream_ref in new_plan.streams:
            transport = saved_transports.get(stream_ref.name)
            if transport is not None:
                new_proxy.set_transport(stream_ref.name, transport)
        self._module_transports[new_class] = {
            s.name: t for s in new_plan.streams if (t := saved_transports.get(s.name)) is not None
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

    def loop(self) -> None:
        stop = threading.Event()
        try:
            stop.wait()
        except KeyboardInterrupt:
            return
        finally:
            self.stop()


def _resolve_module_plans(
    blueprint: Blueprint,
    gc: GlobalConfig,
    blueprint_args: Mapping[str, Any],
) -> tuple[ResolvedModulePlan, ...]:
    plans: list[ResolvedModulePlan] = []
    for bp in blueprint.active_blueprints:
        module_overrides = blueprint_args.get(bp.module.name, {})
        if module_overrides is None:
            module_overrides = {}
        if not isinstance(module_overrides, Mapping):
            raise TypeError(
                f"Blueprint args for {bp.module.name} must be a mapping, got "
                f"{type(module_overrides).__name__}"
            )
        final_kwargs = {**bp.kwargs, **dict(module_overrides)}
        final_kwargs["g"] = gc
        config = bp.module.resolve_config(final_kwargs)
        io_contract = bp.module.io_contract(config)
        plans.append(
            ResolvedModulePlan(
                atom=bp,
                module=bp.module,
                final_kwargs=final_kwargs,
                config=config,
                io_contract=io_contract,
            )
        )
    return tuple(plans)


def _verify_stream_remappings(blueprint: Blueprint, plans: tuple[ResolvedModulePlan, ...]) -> None:
    plans_by_module = {plan.module: plan for plan in plans}
    for (module, name), replacement in blueprint.remapping_map.items():
        if not isinstance(replacement, str):
            continue
        plan = plans_by_module.get(module)
        if plan is None:
            continue
        resolved_names = {stream.name for stream in plan.streams}
        if name not in resolved_names:
            raise ValueError(
                f"Stream remapping for {module.__name__}.{name} references a stream absent "
                "from the resolved IO contract"
            )


def _all_name_types(
    blueprint: Blueprint, plans: tuple[ResolvedModulePlan, ...] | None = None
) -> set[tuple[str, type]]:
    plans = plans or _resolve_module_plans(blueprint, global_config, {})
    result = set()
    for plan in plans:
        for conn in plan.streams:
            remapped_name = blueprint.remapping_map.get((plan.module, conn.name), conn.name)
            if isinstance(remapped_name, str):
                result.add((remapped_name, conn.type))
    return result


def _is_name_unique(blueprint: Blueprint, plans: tuple[ResolvedModulePlan, ...], name: str) -> bool:
    return sum(1 for n, _ in _all_name_types(blueprint, plans) if n == name) == 1


def _get_transport_for(
    blueprint: Blueprint,
    plans: tuple[ResolvedModulePlan, ...],
    name: str,
    stream_type: type,
) -> PubSubTransport[Any]:
    transport = blueprint.transport_map.get((name, stream_type), None)
    if transport:
        return transport

    use_pickled = getattr(stream_type, "lcm_encode", None) is None
    topic = f"/{name}" if _is_name_unique(blueprint, plans, name) else f"/{short_id()}"
    transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)

    return transport


def _verify_no_name_conflicts(
    blueprint: Blueprint, plans: tuple[ResolvedModulePlan, ...] | None = None
) -> None:
    plans = plans or _resolve_module_plans(blueprint, global_config, {})
    name_to_types: dict[Any, set[type]] = defaultdict(set)
    name_to_modules: dict[Any, list[tuple[type, type]]] = defaultdict(list)

    for plan in plans:
        for conn in plan.streams:
            stream_name = blueprint.remapping_map.get((plan.module, conn.name), conn.name)
            if not isinstance(stream_name, str):
                continue
            name_to_types[stream_name].add(conn.type)
            name_to_modules[stream_name].append((plan.module, conn.type))

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
    plans: tuple[ResolvedModulePlan, ...] | None = None,
) -> None:
    """Check that a new blueprint's streams don't conflict with already-registered transports."""
    if not existing_registry:
        return

    existing_names: dict[str, set[type]] = defaultdict(set)
    for name, stream_type in existing_registry:
        existing_names[name].add(stream_type)

    plans = plans or _resolve_module_plans(blueprint, global_config, {})
    for plan in plans:
        for conn in plan.streams:
            remapped_name = blueprint.remapping_map.get((plan.module, conn.name), conn.name)
            if isinstance(remapped_name, str) and remapped_name in existing_names:
                for existing_type in existing_names[remapped_name]:
                    if existing_type != conn.type:
                        raise ValueError(
                            f"Stream '{remapped_name}' in {plan.module.__name__} has type "
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
    plans: tuple[ResolvedModulePlan, ...],
    module_coordinator: ModuleCoordinator,
    gc: GlobalConfig,
    blueprint: Blueprint,
) -> None:
    module_specs: list[ModuleSpec] = []
    for plan in plans:
        module_specs.append((plan.module, gc, plan.final_kwargs.copy()))

    module_coordinator.deploy_parallel(
        module_specs,
        runtime_environment_registry=blueprint.runtime_environment_registry,
        runtime_placement_map=blueprint.runtime_placement_map,
    )

    for plan in plans:
        module_coordinator._deployed_atoms[plan.module] = plan.atom
        module_coordinator._resolved_module_plans[plan.module] = plan


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


def _log_blueprint_graph(
    blueprint: Blueprint,
    module_coordinator: ModuleCoordinator,
    plans: tuple[ResolvedModulePlan, ...],
) -> None:
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

        dot_code = render(blueprint, plans)
        module_names = [plan.module.__name__ for plan in plans]
        bridge = module_coordinator.get_instance(RerunBridgeModule)  # type: ignore[arg-type]
        bridge.log_blueprint_graph(dot_code, module_names)
    except Exception:
        logger.error("Failed to log blueprint graph to Rerun", exc_info=True)
