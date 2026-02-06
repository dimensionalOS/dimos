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

from abc import ABC
from collections import defaultdict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from functools import cached_property, reduce
import inspect
import operator
import sys
from types import MappingProxyType
from typing import Any, Literal, get_args, get_origin, get_type_hints

import rerun as rr
import rerun.blueprint as rrb

from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, is_module_type
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, PubSubTransport, pLCMTransport
from dimos.spec.utils import Spec, is_spec, spec_annotation_compliance, spec_structural_compliance
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class StreamRef:
    name: str
    type: type
    direction: Literal["in", "out"]


@dataclass(frozen=True)
class ModuleRef:
    name: str
    spec: type[Spec] | type[Module]


@dataclass(frozen=True)
class _BlueprintAtom:
    module: type[Module]
    streams: tuple[StreamRef, ...]
    module_refs: tuple[ModuleRef, ...]
    args: tuple[Any, ...]
    kwargs: dict[str, Any]

    @classmethod
    def create(
        cls, module: type[Module], args: tuple[Any, ...], kwargs: dict[str, Any]
    ) -> "_BlueprintAtom":
        streams: list[StreamRef] = []
        module_refs: list[ModuleRef] = []

        # Use get_type_hints() to properly resolve string annotations.
        try:
            all_annotations = get_type_hints(module)
        except Exception:
            # Fallback to raw annotations if get_type_hints fails.
            all_annotations = {}
            for base_class in reversed(module.__mro__):
                if hasattr(base_class, "__annotations__"):
                    all_annotations.update(base_class.__annotations__)

        for name, annotation in all_annotations.items():
            origin = get_origin(annotation)
            # Streams
            if origin in (In, Out):
                direction = "in" if origin == In else "out"
                type_ = get_args(annotation)[0]
                streams.append(
                    StreamRef(name=name, type=type_, direction=direction)  # type: ignore[arg-type]
                )
            # linking to unknown module via Spec
            elif is_spec(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))
            # linking to specific/known module directly
            elif is_module_type(annotation):
                module_refs.append(ModuleRef(name=name, spec=annotation))

        return cls(
            module=module,
            streams=tuple(streams),
            module_refs=tuple(module_refs),
            args=args,
            kwargs=kwargs,
        )


@dataclass(frozen=True)
class Blueprint:
    blueprints: tuple[_BlueprintAtom, ...]
    transport_map: Mapping[tuple[str, type], PubSubTransport[Any]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    remapping_map: Mapping[tuple[type[Module], str], str | type[Module] | type[Spec]] = field(
        default_factory=lambda: MappingProxyType({})
    )
    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)

    @classmethod
    def create(cls, module: type[Module], *args: Any, **kwargs: Any) -> "Blueprint":
        blueprint = _BlueprintAtom.create(module, args, kwargs)
        return cls(blueprints=(blueprint,))

    def transports(self, transports: dict[tuple[str, type], Any]) -> "Blueprint":
        return Blueprint(
            blueprints=self.blueprints,
            transport_map=MappingProxyType({**self.transport_map, **transports}),
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def global_config(self, **kwargs: Any) -> "Blueprint":
        return Blueprint(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def remappings(
        self, remappings: list[tuple[type[Module], str, str | type[Module] | type[Spec]]]
    ) -> "Blueprint":
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(module, old)] = new

        return Blueprint(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=MappingProxyType(remappings_dict),
            requirement_checks=self.requirement_checks,
        )

    def requirements(self, *checks: Callable[[], str | None]) -> "Blueprint":
        return Blueprint(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks + tuple(checks),
        )

    def _check_ambiguity(
        self,
        requested_method_name: str,
        interface_methods: Mapping[str, list[tuple[type[Module], Callable[..., Any]]]],
        requesting_module: type[Module],
    ) -> None:
        if (
            requested_method_name in interface_methods
            and len(interface_methods[requested_method_name]) > 1
        ):
            modules_str = ", ".join(
                impl[0].__name__ for impl in interface_methods[requested_method_name]
            )
            raise ValueError(
                f"Ambiguous RPC method '{requested_method_name}' requested by "
                f"{requesting_module.__name__}. Multiple implementations found: "
                f"{modules_str}. Please use a concrete class name instead."
            )

    def _get_transport_for(self, name: str, stream_type: type) -> PubSubTransport[Any]:
        transport = self.transport_map.get((name, stream_type), None)
        if transport:
            return transport

        use_pickled = getattr(stream_type, "lcm_encode", None) is None
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, stream_type)

        return transport

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        # Apply remappings to get the actual names that will be used
        result = set()
        for blueprint in self.blueprints:
            for conn in blueprint.streams:
                # Check if this stream should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    result.add((remapped_name, conn.type))
        return result

    def _is_name_unique(self, name: str) -> bool:
        return sum(1 for n, _ in self._all_name_types if n == name) == 1

    def _check_requirements(self) -> None:
        errors = []
        red = "\033[31m"
        reset = "\033[0m"

        for check in self.requirement_checks:
            error = check()
            if error:
                errors.append(error)

        if errors:
            for error in errors:
                print(f"{red}Error: {error}{reset}", file=sys.stderr)
            sys.exit(1)

    def _verify_no_name_conflicts(self) -> None:
        name_to_types = defaultdict(set)
        name_to_modules = defaultdict(list)

        for blueprint in self.blueprints:
            for conn in blueprint.streams:
                stream_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                name_to_types[stream_name].add(conn.type)
                name_to_modules[stream_name].append((blueprint.module, conn.type))

        conflicts = {}
        for conn_name, types in name_to_types.items():
            if len(types) > 1:
                modules_by_type = defaultdict(list)
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

    def _deploy_all_modules(
        self, module_coordinator: ModuleCoordinator, global_config: GlobalConfig
    ) -> None:
        module_specs: list[tuple[type[Module], tuple[Any, ...], dict[str, Any]]] = []
        for blueprint in self.blueprints:
            kwargs = {**blueprint.kwargs}
            sig = inspect.signature(blueprint.module.__init__)
            if "global_config" in sig.parameters:
                kwargs["global_config"] = global_config
            module_specs.append((blueprint.module, blueprint.args, kwargs))

        module_coordinator.deploy_parallel(module_specs)

    def _connect_streams(self, module_coordinator: ModuleCoordinator) -> None:
        # dict when given (final/remapped) stream name+type, provides a list of modules + original (non-remapped) stream names
        streams = defaultdict(list)

        for blueprint in self.blueprints:
            for conn in blueprint.streams:
                # Check if this stream should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                if isinstance(remapped_name, str):
                    # Group by remapped name and type
                    streams[remapped_name, conn.type].append((blueprint.module, conn.name))

        # Connect all In/Out streams by remapped name and type.
        for remapped_name, stream_type in streams.keys():
            transport = self._get_transport_for(remapped_name, stream_type)
            for module, original_name in streams[(remapped_name, stream_type)]:
                instance = module_coordinator.get_instance(module)  # type: ignore[assignment]
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]
                logger.info(
                    "Transport",
                    name=remapped_name,
                    original_name=original_name,
                    topic=str(getattr(transport, "topic", None)),
                    type=f"{stream_type.__module__}.{stream_type.__qualname__}",
                    module=module.__name__,
                    transport=transport.__class__.__name__,
                )

    def _connect_module_refs(self, module_coordinator: ModuleCoordinator) -> None:
        # partly fill out the mod_and_mod_ref_to_proxy
        mod_and_mod_ref_to_proxy = {
            (module, name): replacement
            for (module, name), replacement in self.remapping_map.items()
            if is_spec(replacement) or is_module_type(replacement)
        }

        # after this loop we should have an exact module for every module_ref on every blueprint
        for blueprint in self.blueprints:
            for each_module_ref in blueprint.module_refs:
                # we've got to find a another module that implements this spec
                spec = mod_and_mod_ref_to_proxy.get(
                    (blueprint.module, each_module_ref.name), each_module_ref.spec
                )

                # if the spec is actually module, use that (basically a user override)
                if is_module_type(spec):
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = spec
                    continue

                # find all available candidates
                possible_module_candidates = [
                    each_other_blueprint.module
                    for each_other_blueprint in self.blueprints
                    if (
                        each_other_blueprint != blueprint
                        and spec_structural_compliance(each_other_blueprint.module, spec)
                    )
                ]
                # we keep valid separate from invalid to provide a better error message for "almost" valid cases
                valid_module_candidates = [
                    each_candidate
                    for each_candidate in possible_module_candidates
                    if spec_annotation_compliance(each_candidate, spec)
                ]
                # none
                if len(possible_module_candidates) == 0:
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I couldn't find a module that met that spec.\n"""
                    )
                # exactly one structurally valid candidate
                elif len(possible_module_candidates) == 1:
                    if len(valid_module_candidates) == 0:
                        logger.warning(
                            f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. I found a module ({possible_module_candidates[0].__name__}) that met that spec structurally, but it had a mismatch in type annotations.\nPlease either change the {each_module_ref.spec.__name__} spec or the {possible_module_candidates[0].__name__} module.\n"""
                        )
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = (
                        possible_module_candidates[0]
                    )
                    continue
                # more than one
                elif len(valid_module_candidates) > 1:
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. But I found multiple modules that met that spec: {possible_module_candidates}.\nTo fix this use .remappings, for example:\n    autoconnect(...).remappings([ ({blueprint.module.__name__}, {each_module_ref.name!r}, <ModuleThatHasTheRpcCalls>) ])\n"""
                    )
                # structural candidates, but no valid candidates
                elif len(valid_module_candidates) == 0:
                    possible_module_candidates_str = ", ".join(
                        [each_candidate.__name__ for each_candidate in possible_module_candidates]
                    )
                    raise Exception(
                        f"""The {blueprint.module.__name__} has a module reference ({each_module_ref}) which requested a module that fills out the {each_module_ref.spec.__name__} spec. Some modules ({possible_module_candidates_str}) met the spec structurally but had a mismatch in type annotations\n"""
                    )
                # one valid candidate (and more than one structurally valid candidate)
                else:
                    mod_and_mod_ref_to_proxy[blueprint.module, each_module_ref.name] = (
                        valid_module_candidates[0]
                    )

        # now that we know the streams, we mutate the RPCClient objects
        for (base_module, module_ref_name), target_module in mod_and_mod_ref_to_proxy.items():
            base_module_proxy = module_coordinator.get_instance(base_module)
            target_module_proxy = module_coordinator.get_instance(target_module)  # type: ignore[type-var,arg-type]
            setattr(
                base_module_proxy,
                module_ref_name,
                target_module_proxy,
            )
            # Ensure the remote module instance can use the module ref inside its own RPC handlers.
            base_module_proxy.set_module_ref(module_ref_name, target_module_proxy)

    def _connect_rpc_methods(self, module_coordinator: ModuleCoordinator) -> None:
        # Gather all RPC methods.
        rpc_methods = {}
        rpc_methods_dot = {}

        # Track interface methods to detect ambiguity.
        interface_methods: defaultdict[str, list[tuple[type[Module], Callable[..., Any]]]] = (
            defaultdict(list)
        )  # interface_name_method -> [(module_class, method)]
        interface_methods_dot: defaultdict[str, list[tuple[type[Module], Callable[..., Any]]]] = (
            defaultdict(list)
        )  # interface_name.method -> [(module_class, method)]

        for blueprint in self.blueprints:
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                module_proxy = module_coordinator.get_instance(blueprint.module)  # type: ignore[assignment]
                method_for_rpc_client = getattr(module_proxy, method_name)
                # Register under concrete class name (backward compatibility)
                rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method_for_rpc_client
                rpc_methods_dot[f"{blueprint.module.__name__}.{method_name}"] = (
                    method_for_rpc_client
                )

                # Also register under any interface names
                for base in blueprint.module.mro():
                    # Check if this base is an abstract interface with the method
                    if (
                        base is not Module
                        and issubclass(base, ABC)
                        and hasattr(base, method_name)
                        and getattr(base, method_name, None) is not None
                    ):
                        interface_key = f"{base.__name__}.{method_name}"
                        interface_methods_dot[interface_key].append(
                            (blueprint.module, method_for_rpc_client)
                        )
                        interface_key_underscore = f"{base.__name__}_{method_name}"
                        interface_methods[interface_key_underscore].append(
                            (blueprint.module, method_for_rpc_client)
                        )

        # Check for ambiguity in interface methods and add non-ambiguous ones
        for interface_key, implementations in interface_methods_dot.items():
            if len(implementations) == 1:
                rpc_methods_dot[interface_key] = implementations[0][1]
        for interface_key, implementations in interface_methods.items():
            if len(implementations) == 1:
                rpc_methods[interface_key] = implementations[0][1]

        # Fulfil method requests (so modules can call each other).
        for blueprint in self.blueprints:
            instance = module_coordinator.get_instance(blueprint.module)  # type: ignore[assignment]

            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                if not method_name.startswith("set_"):
                    continue

                linked_name = method_name.removeprefix("set_")

                self._check_ambiguity(linked_name, interface_methods, blueprint.module)

                if linked_name not in rpc_methods:
                    continue

                getattr(instance, method_name)(rpc_methods[linked_name])

            for requested_method_name in instance.get_rpc_method_names():  # type: ignore[union-attr]
                self._check_ambiguity(
                    requested_method_name, interface_methods_dot, blueprint.module
                )

                if requested_method_name not in rpc_methods_dot:
                    continue

                instance.set_rpc_method(  # type: ignore[union-attr]
                    requested_method_name, rpc_methods_dot[requested_method_name]
                )

    def _init_rerun_blueprint(self, module_coordinator: ModuleCoordinator) -> None:
        """Compose and send Rerun blueprint from module contributions.

        Collects rerun_views() from all modules and composes them into a unified layout.
        """
        # Collect view contributions from all modules
        side_panels = []
        for blueprint in self.blueprints:
            if hasattr(blueprint.module, "rerun_views"):
                views = blueprint.module.rerun_views()
                if views:
                    side_panels.extend(views)

        # Always include latency panel if we have any panels
        if side_panels:
            side_panels.append(
                rrb.TimeSeriesView(
                    name="Latency (ms)",
                    origin="/metrics",
                    contents=[
                        "+ /metrics/voxel_map/latency_ms",
                        "+ /metrics/costmap/latency_ms",
                    ],
                )
            )

        # Compose final layout
        if side_panels:
            composed_blueprint = rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(
                        name="3D View",
                        origin="world",
                        background=[0, 0, 0],
                    ),
                    rrb.Vertical(*side_panels, row_shares=[2] + [1] * (len(side_panels) - 1)),
                    column_shares=[3, 1],
                ),
                rrb.TimePanel(state="collapsed"),
                rrb.SelectionPanel(state="collapsed"),
                rrb.BlueprintPanel(state="collapsed"),
            )
            rr.send_blueprint(composed_blueprint)

    def _start_rerun(self, global_config: GlobalConfig) -> None:
        # Initialize Rerun server before deploying modules (if backend is Rerun)
        if global_config.rerun_enabled and global_config.viewer_backend.startswith("rerun"):
            try:
                from dimos.dashboard.rerun_init import init_rerun_server

                server_addr = init_rerun_server(viewer_mode=global_config.viewer_backend)
                global_config.model_copy(update={"rerun_server_addr": server_addr})
                logger.info("Rerun server initialized", addr=server_addr)
            except Exception as e:
                logger.warning(f"Failed to initialize Rerun server: {e}")

    def build(
        self,
        global_config: GlobalConfig | None = None,
        cli_config_overrides: Mapping[str, Any] | None = None,
    ) -> ModuleCoordinator:
        if global_config is None:
            global_config = GlobalConfig()
        global_config = global_config.model_copy(update=dict(self.global_config_overrides))
        if cli_config_overrides:
            global_config = global_config.model_copy(update=dict(cli_config_overrides))

        self._check_requirements()
        self._verify_no_name_conflicts()
        self._start_rerun(global_config)

        module_coordinator = ModuleCoordinator(global_config=global_config)
        module_coordinator.start()

        # all module constructors are called here (each of them setup their own)
        self._deploy_all_modules(module_coordinator, global_config)
        self._connect_streams(module_coordinator)
        self._connect_rpc_methods(module_coordinator)
        self._connect_module_refs(module_coordinator)

        module_coordinator.start_all_modules()

        # Compose and send Rerun blueprint from module contributions
        if global_config.viewer_backend.startswith("rerun"):
            self._init_rerun_blueprint(module_coordinator)

        return module_coordinator


def autoconnect(*blueprints: Blueprint) -> Blueprint:
    all_blueprints = tuple(_eliminate_duplicates([bp for bs in blueprints for bp in bs.blueprints]))
    all_transports = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.transport_map.items()) for x in blueprints], [])
    )
    all_config_overrides = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.global_config_overrides.items()) for x in blueprints], [])
    )
    all_remappings = dict(  # type: ignore[var-annotated]
        reduce(operator.iadd, [list(x.remapping_map.items()) for x in blueprints], [])
    )
    all_requirement_checks = tuple(check for bs in blueprints for check in bs.requirement_checks)

    return Blueprint(
        blueprints=all_blueprints,
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
        requirement_checks=all_requirement_checks,
    )


def _eliminate_duplicates(blueprints: list[_BlueprintAtom]) -> list[_BlueprintAtom]:
    # The duplicates are eliminated in reverse so that newer blueprints override older ones.
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))
