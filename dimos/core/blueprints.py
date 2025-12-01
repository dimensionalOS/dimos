# Copyright 2025 Dimensional Inc.
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

"""Declarative module composition and automatic connection wiring.

Instead of manually connecting modules, define blueprints that specify each module's
data dependencies (`In[T]`, `Out[T]` streams) and RPC method requirements. The blueprint
system automatically wires streams, selects transports, and links RPC methods between
modules, when building the blueprint.

Core components
---------------
`ModuleBlueprint`
    Immutable specification for instantiating a single module with its stream
    connections. Created via `Module.blueprint()`.

`ModuleBlueprintSet`
    Container for multiple blueprints with builder methods for configuration:
    `transports()`, `global_config()`, `remappings()`, and `build()`.

`autoconnect()`
    Combine multiple `ModuleBlueprintSet` instances into one composed system.
    Deduplicates blueprints, merging configuration with last-wins semantics.

Basic usage
-----------
Streams match by name and type. Use `.remappings()` when names differ:

    blueprint = autoconnect(
        CameraModule.blueprint(),
        ProcessorModule.blueprint()
    ).remappings([
        (CameraModule, "color_image", "rgb"),
        (ProcessorModule, "rgb_input", "rgb"),
    ])
    coordinator = blueprint.build()
    coordinator.loop()  # Run until interrupted

For detailed explanation of connection matching, composition patterns, transport
selection, and RPC wiring, see `/docs/concepts/blueprints.md`.

See also
--------
`dimos.core.module`
    Module base class with `In[T]`/`Out[T]` stream declarations.

`dimos.core.module_coordinator`
    Runtime manager for deployed modules. Returned by `ModuleBlueprintSet.build()`.

`dimos.core.transport`
    Transport implementations (`LCMTransport`, `pLCMTransport`) for inter-module
    communication.
"""

from abc import ABC
from collections import defaultdict
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from functools import cached_property, reduce
import inspect
import operator
import sys
from types import MappingProxyType
from typing import Annotated, Any, Literal, get_args, get_origin

from annotated_doc import Doc

from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.utils.generic import short_id


@dataclass(frozen=True)
class ModuleConnection:
    name: str
    type: type
    direction: Literal["in", "out"]


@dataclass(frozen=True)
class ModuleBlueprint:
    """Declarative specification for instantiating and wiring a module.

    A ModuleBlueprint captures everything needed to instantiate and connect a module
    in a distributed system without actually creating the module instance. This separation
    enables composition, configuration, and deployment to be handled independently.

    Blueprints are immutable and serve as the specification layer between
    high-level system design and runtime deployment. They are typically created using
    `Module.blueprint()`, combined into `ModuleBlueprintSet` containers via `autoconnect()`,
    and deployed by `ModuleCoordinator.build()`.
    """

    module: Annotated[
        type[Module],
        Doc(
            "The module class to instantiate. This is the constructor, not an instance, allowing late binding during deployment."
        ),
    ]
    connections: Annotated[
        tuple[ModuleConnection, ...],
        Doc(
            "Typed stream connections extracted from the module's type annotations. These specify how this module's streams should be wired to other modules during deployment."
        ),
    ]
    args: Annotated[
        tuple[Any], Doc("Positional arguments to pass to the module's `__init__` method.")
    ]
    kwargs: Annotated[
        dict[str, Any], Doc("Keyword arguments to pass to the module's `__init__` method.")
    ]


@dataclass(frozen=True)
class ModuleBlueprintSet:
    blueprints: tuple[ModuleBlueprint, ...]
    # TODO: Replace Any
    transport_map: Mapping[tuple[str, type], Any] = field(
        default_factory=lambda: MappingProxyType({})
    )
    global_config_overrides: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))
    remapping_map: Mapping[tuple[type[Module], str], str] = field(
        default_factory=lambda: MappingProxyType({})
    )
    requirement_checks: tuple[Callable[[], str | None], ...] = field(default_factory=tuple)

    def transports(
        self,
        transports: Annotated[
            dict[tuple[str, type], Any],
            Doc(
                """Dictionary mapping (connection_name, data_type) to transport instances.
                Both the connection name and data type must match for the override to apply."""
            ),
        ],
    ) -> Annotated[
        "ModuleBlueprintSet", Doc("New ModuleBlueprintSet with merged transport overrides.")
    ]:
        """Register explicit transport overrides for specific connections.

        By default, dimos auto-selects transports based on whether the data type
        has an `lcm_encode` method. Use this to override those defaults when you need:
        - Shared memory (SHM) transports for high-bandwidth data like images
        - Custom topic names
        - Specific transport implementations for performance or compatibility
        """
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=MappingProxyType({**self.transport_map, **transports}),
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def global_config(
        self,
        **kwargs: Annotated[
            Any,
            Doc(
                """Key-value pairs to override in GlobalConfig (e.g., n_dask_workers, log_level).
                Values are validated during build()."""
            ),
        ],
    ) -> Annotated[
        "ModuleBlueprintSet", Doc("New ModuleBlueprintSet with merged configuration overrides.")
    ]:
        """Override GlobalConfig parameters for modules in this blueprint set.

        These overrides take precedence over configuration from .env files or
        environment variables. Useful for deployment-specific settings, debugging,
        or testing without changing global configuration.
        """
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=MappingProxyType({**self.global_config_overrides, **kwargs}),
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks,
        )

    def remappings(
        self,
        remappings: Annotated[
            list[tuple[type[Module], str, str]],
            Doc(
                """List of (module_class, old_name, new_name) tuples specifying
                that the module's connection 'old_name' should be treated as 'new_name'."""
            ),
        ],
    ) -> Annotated[
        "ModuleBlueprintSet", Doc("New ModuleBlueprintSet with updated connection remappings.")
    ]:
        """Rename module connections to enable interoperability between modules.

        Allows modules with different naming conventions to communicate. Remapping
        is transparent to modules—they use their original names internally. Remapped
        names are used only for connection matching (connections match if remapped
        names and types both match) and topic generation.

        Examples:
            Connect modules with different naming conventions:

            >>> class CameraModule(Module):
            ...     color_image: Out[str] = None
            >>> class ProcessorModule(Module):
            ...     rgb_input: In[str] = None
            >>>
            >>> blueprint = autoconnect(
            ...     CameraModule.blueprint(),
            ...     ProcessorModule.blueprint()
            ... )
            >>> blueprint = blueprint.remappings([
            ...     (CameraModule, "color_image", "rgb_image"),
            ...     (ProcessorModule, "rgb_input", "rgb_image")
            ... ])
            >>> # Now both connections use "rgb_image" and will be connected

            Broadcast to multiple consumers:

            >>> class SensorModule(Module):
            ...     output: Out[str] = None
            >>> class ProcessorA(Module):
            ...     input: In[str] = None
            >>> class ProcessorB(Module):
            ...     input_stream: In[str] = None
            >>>
            >>> blueprint = autoconnect(
            ...     SensorModule.blueprint(),
            ...     ProcessorA.blueprint(),
            ...     ProcessorB.blueprint()
            ... )
            >>> blueprint = blueprint.remappings([
            ...     (SensorModule, "output", "shared_data"),
            ...     (ProcessorA, "input", "shared_data"),
            ...     (ProcessorB, "input_stream", "shared_data")
            ... ])
            >>> # All three connections now share the same transport
        """
        remappings_dict = dict(self.remapping_map)
        for module, old, new in remappings:
            remappings_dict[(module, old)] = new

        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=MappingProxyType(remappings_dict),
            requirement_checks=self.requirement_checks,
        )

    def requirements(self, *checks: Callable[[], str | None]) -> "ModuleBlueprintSet":
        return ModuleBlueprintSet(
            blueprints=self.blueprints,
            transport_map=self.transport_map,
            global_config_overrides=self.global_config_overrides,
            remapping_map=self.remapping_map,
            requirement_checks=self.requirement_checks + tuple(checks),
        )

    def _get_transport_for(
        self,
        name: Annotated[str, Doc("Connection name after remapping.")],
        type: Annotated[type, Doc("Data type from the module's type annotations.")],
    ) -> Annotated[Any, Doc("Transport instance for the connection.")]:
        """Determine and create the appropriate transport for a connection.

        Selection priority:
        1. Explicit transport override in transport_map
        2. Auto-select based on type: LCMTransport if type has lcm_encode, else pLCMTransport
        3. Topic naming: /{name} if unique, else random ID

        Connections with identical (remapped_name, type) pairs share the same
        transport instance, enabling pub/sub communication.
        """
        transport = self.transport_map.get((name, type), None)
        if transport:
            return transport

        use_pickled = getattr(type, "lcm_encode", None) is None
        topic = f"/{name}" if self._is_name_unique(name) else f"/{short_id()}"
        transport = pLCMTransport(topic) if use_pickled else LCMTransport(topic, type)

        return transport

    @cached_property
    def _all_name_types(self) -> set[tuple[str, type]]:
        # Apply remappings to get the actual names that will be used
        result = set()
        for blueprint in self.blueprints:
            for conn in blueprint.connections:
                # Check if this connection should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
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
            for conn in blueprint.connections:
                connection_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                name_to_types[connection_name].add(conn.type)
                name_to_modules[connection_name].append((blueprint.module, conn.type))

        conflicts = {}
        for conn_name, types in name_to_types.items():
            if len(types) > 1:
                modules_by_type = defaultdict(list)
                for module, conn_type in name_to_modules[conn_name]:
                    modules_by_type[conn_type].append(module)
                conflicts[conn_name] = modules_by_type

        if not conflicts:
            return

        error_lines = ["Blueprint cannot start because there are conflicting connections."]
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
        for blueprint in self.blueprints:
            kwargs = {**blueprint.kwargs}
            sig = inspect.signature(blueprint.module.__init__)
            if "global_config" in sig.parameters:
                kwargs["global_config"] = global_config
            module_coordinator.deploy(blueprint.module, *blueprint.args, **kwargs)

    def _connect_transports(
        self,
        module_coordinator: Annotated[
            ModuleCoordinator,
            Doc(
                """Coordinator containing deployed module instances.
                All modules in self.blueprints must have been deployed first."""
            ),
        ],
    ) -> None:
        """Establish transport connections between deployed modules.

        Processes all stream connections, applies name remappings, groups connections
        by (remapped_name, type), and assigns the same transport instance to all
        connections within each group.

        Key design decision: Connections with matching (remapped_name, type) share
        the same transport instance, enabling pub/sub communication. Type mismatches
        prevent sharing even with matching names, preserving type safety. Remapping
        is transparent—modules use their original connection names internally.
        """
        # Gather all the In/Out connections with remapping applied.
        connections = defaultdict(list)
        # Track original name -> remapped name for each module
        module_conn_mapping = defaultdict(dict)  # type: ignore[var-annotated]

        for blueprint in self.blueprints:
            for conn in blueprint.connections:
                # Check if this connection should be remapped
                remapped_name = self.remapping_map.get((blueprint.module, conn.name), conn.name)
                # Store the mapping for later use
                module_conn_mapping[blueprint.module][conn.name] = remapped_name
                # Group by remapped name and type
                connections[remapped_name, conn.type].append((blueprint.module, conn.name))

        # Connect all In/Out connections by remapped name and type.
        for remapped_name, type in connections.keys():
            transport = self._get_transport_for(remapped_name, type)
            for module, original_name in connections[(remapped_name, type)]:
                instance = module_coordinator.get_instance(module)
                instance.set_transport(original_name, transport)  # type: ignore[union-attr]

    def _connect_rpc_methods(
        self,
        module_coordinator: Annotated[
            ModuleCoordinator,
            Doc("Coordinator containing deployed module instances with initialized RPC servers."),
        ],
    ) -> None:
        """Wire up inter-module RPC method calls.

        Processes two independent wiring mechanisms:
        - Implicit: Calls `set_ClassName_method()` setters with bound methods
        - Explicit: Populates `rpc_calls` requests via `set_rpc_method()`

        Interface-based requests (e.g., `NavigationInterface.get_state`) resolve to a
        single implementation or fail at build time if ambiguous. Missing methods are
        skipped (error raised at call time via `get_rpc_calls()`).
        """
        # Gather all RPC methods.
        rpc_methods = {}
        rpc_methods_dot = {}
        # Track interface methods to detect ambiguity
        interface_methods = defaultdict(list)  # interface_name.method -> [(module_class, method)]

        for blueprint in self.blueprints:
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                method = getattr(module_coordinator.get_instance(blueprint.module), method_name)
                # Register under concrete class name (backward compatibility)
                rpc_methods[f"{blueprint.module.__name__}_{method_name}"] = method
                rpc_methods_dot[f"{blueprint.module.__name__}.{method_name}"] = method

                # Also register under any interface names
                for base in blueprint.module.__bases__:
                    # Check if this base is an abstract interface with the method
                    if (
                        base is not Module
                        and issubclass(base, ABC)
                        and hasattr(base, method_name)
                        and getattr(base, method_name, None) is not None
                    ):
                        interface_key = f"{base.__name__}.{method_name}"
                        interface_methods[interface_key].append((blueprint.module, method))

        # Check for ambiguity in interface methods and add non-ambiguous ones
        for interface_key, implementations in interface_methods.items():
            if len(implementations) == 1:
                rpc_methods_dot[interface_key] = implementations[0][1]

        # Fulfil method requests (so modules can call each other).
        for blueprint in self.blueprints:
            instance = module_coordinator.get_instance(blueprint.module)
            for method_name in blueprint.module.rpcs.keys():  # type: ignore[attr-defined]
                if not method_name.startswith("set_"):
                    continue
                linked_name = method_name.removeprefix("set_")
                if linked_name not in rpc_methods:
                    continue
                getattr(instance, method_name)(rpc_methods[linked_name])
            for requested_method_name in instance.get_rpc_method_names():  # type: ignore[union-attr]
                # Check if this is an ambiguous interface method
                if (
                    requested_method_name in interface_methods
                    and len(interface_methods[requested_method_name]) > 1
                ):
                    modules_str = ", ".join(
                        impl[0].__name__ for impl in interface_methods[requested_method_name]
                    )
                    raise ValueError(
                        f"Ambiguous RPC method '{requested_method_name}' requested by "
                        f"{blueprint.module.__name__}. Multiple implementations found: "
                        f"{modules_str}. Please use a concrete class name instead."
                    )

                if requested_method_name not in rpc_methods_dot:
                    continue
                instance.set_rpc_method(  # type: ignore[union-attr]
                    requested_method_name, rpc_methods_dot[requested_method_name]
                )

    def build(
        self,
        global_config: Annotated[
            GlobalConfig | None,
            Doc(
                """Base configuration for the system. Defaults to GlobalConfig().
                Blueprint overrides take precedence."""
            ),
        ] = None,
    ) -> Annotated[
        ModuleCoordinator,
        Doc(
            """Fully initialized ModuleCoordinator. Call coordinator.loop() to run
            or coordinator.stop() to shut down cleanly."""
        ),
    ]:
        """Transform this blueprint specification into a running distributed system.

        Terminal operation in the blueprint builder pattern. Creates a fully initialized
        ModuleCoordinator with all modules deployed, connected, and ready to run.

        Build process:
        1. Merge global_config with blueprint-level overrides (overrides take precedence)
        2. Create and start ModuleCoordinator with Dask cluster
        3. Deploy all modules
        4. Connect stream transports (In/Out with matching remapped names and types)
        5. Link RPC methods between modules
        6. Start all deployed modules

        Raises:
            ValueError: If an RPC method request is ambiguous (multiple modules
              implement the same interface method). Use concrete class name instead.

        Examples:
            >>> class CameraModule(Module):
            ...     color_image: Out[str] = None
            >>> class ProcessorModule(Module):
            ...     image_in: In[str] = None
            >>>
            >>> blueprint = (
            ...     autoconnect(
            ...         CameraModule.blueprint(),
            ...         ProcessorModule.blueprint()
            ...     )
            ...     .remappings([
            ...         (CameraModule, "color_image", "rgb_input"),
            ...         (ProcessorModule, "image_in", "rgb_input"),
            ...     ])
            ...     .global_config(n_dask_workers=2)
            ... )
            >>> coordinator = blueprint.build()
            >>> # ...do whatever you want to do with the coordinator
            >>> coordinator.stop()
        """
        if global_config is None:
            global_config = GlobalConfig()
        global_config = global_config.model_copy(update=self.global_config_overrides)

        self._check_requirements()
        self._verify_no_name_conflicts()

        module_coordinator = ModuleCoordinator(global_config=global_config)
        module_coordinator.start()

        self._deploy_all_modules(module_coordinator, global_config)
        self._connect_transports(module_coordinator)
        self._connect_rpc_methods(module_coordinator)

        module_coordinator.start_all_modules()

        return module_coordinator


def _make_module_blueprint(
    module: type[Module], args: tuple[Any], kwargs: dict[str, Any]
) -> ModuleBlueprint:
    connections: list[ModuleConnection] = []

    all_annotations = {}
    for base_class in reversed(module.__mro__):
        if hasattr(base_class, "__annotations__"):
            all_annotations.update(base_class.__annotations__)

    for name, annotation in all_annotations.items():
        origin = get_origin(annotation)
        if origin not in (In, Out):  # type: ignore[comparison-overlap]
            continue
        direction = "in" if origin == In else "out"  # type: ignore[comparison-overlap]
        type_ = get_args(annotation)[0]
        connections.append(ModuleConnection(name=name, type=type_, direction=direction))  # type: ignore[arg-type]

    return ModuleBlueprint(module=module, connections=tuple(connections), args=args, kwargs=kwargs)


def create_module_blueprint(module: type[Module], *args: Any, **kwargs: Any) -> ModuleBlueprintSet:
    blueprint = _make_module_blueprint(module, args, kwargs)
    return ModuleBlueprintSet(blueprints=(blueprint,))


def autoconnect(*blueprints: ModuleBlueprintSet) -> ModuleBlueprintSet:
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

    return ModuleBlueprintSet(
        blueprints=all_blueprints,
        transport_map=MappingProxyType(all_transports),
        global_config_overrides=MappingProxyType(all_config_overrides),
        remapping_map=MappingProxyType(all_remappings),
        requirement_checks=all_requirement_checks,
    )


def _eliminate_duplicates(blueprints: list[ModuleBlueprint]) -> list[ModuleBlueprint]:
    # The duplicates are eliminated in reverse so that newer blueprints override older ones.
    seen = set()
    unique_blueprints = []
    for bp in reversed(blueprints):
        if bp.module not in seen:
            seen.add(bp.module)
            unique_blueprints.append(bp)
    return list(reversed(unique_blueprints))
