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

"""Compile one resolved Blueprint into immutable per-Host fragments."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping
from types import MappingProxyType
from typing import Any, cast

from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, ModuleRef, TransportSpec
from dimos.core.module import is_module_type
from dimos.core.transport import ZenohTransport, pZenohTransport
from dimos.hosted.fragment import (
    BoundaryStream,
    HostFragment,
    PythonFragmentPayload,
    run_stream_base_topic,
    run_stream_key,
)
from dimos.spec.utils import is_spec, spec_annotation_compliance, spec_structural_compliance

HOSTED_GLOBAL_OVERRIDES: Mapping[str, Any] = MappingProxyType({"transport": "zenoh"})

StreamKey = tuple[str, type]


def compile_fragments(
    blueprint: Blueprint,
    config: ParsedBlueprintConfig,
    assignments: Mapping[str, str],
    *,
    run_id: str,
    generation: int,
    application_name: str,
    application_revision: str,
) -> dict[str, HostFragment]:
    """Compile a Blueprint into one immutable fragment per assigned Host."""
    _validate_metadata(run_id, generation, application_name, application_revision)
    config.assert_matches(blueprint)

    atoms = tuple(blueprint.active_blueprints)
    atom_names = {atom.name for atom in atoms}
    assignment_names = set(assignments)
    if missing := atom_names - assignment_names:
        raise ValueError(f"Placement is missing modules: {', '.join(sorted(missing))}")
    if unknown := assignment_names - atom_names:
        raise ValueError(f"Placement contains unknown modules: {', '.join(sorted(unknown))}")
    if invalid_hosts := sorted(
        name for name, host_id in assignments.items() if not isinstance(host_id, str) or not host_id
    ):
        raise ValueError(f"Placement has empty Host IDs for: {', '.join(invalid_hosts)}")

    host_ids = sorted(set(assignments.values()))
    if len(host_ids) > 1 and (blueprint.requirement_checks or blueprint.configurator_checks):
        raise ValueError(
            "Multi-Host fragments do not yet support Blueprint requirements or configurators"
        )

    _validate_module_references(blueprint, assignments)
    stream_endpoints = _stream_endpoints(blueprint)
    boundary_keys = {
        key
        for key, endpoint_names in stream_endpoints.items()
        if len({assignments[name] for name in endpoint_names}) > 1
    }
    disabled_atoms = tuple(
        atom for atom in blueprint.blueprints if atom not in blueprint.active_blueprints
    )

    fragments: dict[str, HostFragment] = {}
    for host_id in host_ids:
        local_atoms = tuple(atom for atom in atoms if assignments[atom.name] == host_id)
        local_names = {atom.name for atom in local_atoms}
        local_stream_keys = {
            key
            for key, endpoint_names in stream_endpoints.items()
            if local_names.intersection(endpoint_names)
        }
        local_boundaries = tuple(
            _boundary_stream(run_id, key)
            for key in sorted(
                boundary_keys.intersection(local_stream_keys),
                key=lambda item: (item[0], item[1].__module__, item[1].__qualname__),
            )
        )
        local_blueprint = _local_blueprint(
            blueprint,
            run_id,
            local_atoms,
            disabled_atoms,
            local_names,
            local_stream_keys,
            local_boundaries,
            single_host=len(host_ids) == 1,
        )
        local_config = config.subset_for(
            local_blueprint,
            global_overrides=HOSTED_GLOBAL_OVERRIDES,
        )
        fragments[host_id] = HostFragment.create(
            run_id=run_id,
            generation=generation,
            host_id=host_id,
            application_name=application_name,
            application_revision=application_revision,
            payload=PythonFragmentPayload(
                blueprint=local_blueprint,
                config=local_config,
                boundary_streams=local_boundaries,
            ),
        )

    return fragments


def _validate_metadata(
    run_id: str,
    generation: int,
    application_name: str,
    application_revision: str,
) -> None:
    if not run_id:
        raise ValueError("run_id must not be empty")
    if generation < 1:
        raise ValueError("generation must be at least 1")
    if not application_name:
        raise ValueError("application_name must not be empty")
    if not application_revision:
        raise ValueError("application_revision must not be empty")


def _stream_endpoints(blueprint: Blueprint) -> dict[StreamKey, tuple[str, ...]]:
    endpoints: dict[StreamKey, list[str]] = defaultdict(list)
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            name = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
            if isinstance(name, str):
                endpoints[name, stream.type].append(atom.name)
    return {key: tuple(names) for key, names in endpoints.items()}


def _boundary_stream(run_id: str, stream_key: StreamKey) -> BoundaryStream:
    name, message_type = stream_key
    return BoundaryStream(
        name=name,
        message_type=message_type,
        key_expr=run_stream_key(run_id, name, message_type),
    )


def _boundary_transport(run_id: str, boundary: BoundaryStream) -> TransportSpec:
    if getattr(boundary.message_type, "lcm_encode", None) is None:
        return pZenohTransport.spec(boundary.key_expr)
    return ZenohTransport.spec(
        run_stream_base_topic(run_id, boundary.name),
        boundary.message_type,
    )


def _local_blueprint(
    source: Blueprint,
    run_id: str,
    local_atoms: tuple[BlueprintAtom, ...],
    disabled_atoms: tuple[BlueprintAtom, ...],
    local_names: set[str],
    local_stream_keys: set[StreamKey],
    boundaries: tuple[BoundaryStream, ...],
    *,
    single_host: bool,
) -> Blueprint:
    transports = {
        key: transport
        for key, transport in source.transport_map.items()
        if key in local_stream_keys
    }
    for boundary in boundaries:
        transports[boundary.name, boundary.message_type] = _boundary_transport(
            run_id,
            boundary,
        )

    remappings = {
        key: value for key, value in source.remapping_map.items() if key[0] in local_names
    }
    global_overrides = {**source.global_config_overrides, **HOSTED_GLOBAL_OVERRIDES}
    return Blueprint(
        blueprints=local_atoms + disabled_atoms,
        disabled_modules_tuple=source.disabled_modules_tuple,
        transport_map=MappingProxyType(transports),
        global_config_overrides=MappingProxyType(global_overrides),
        remapping_map=MappingProxyType(remappings),
        requirement_checks=source.requirement_checks if single_host else (),
        configurator_checks=source.configurator_checks if single_host else (),
    )


def _atom_namespace(instance_name: str) -> str:
    return instance_name.rsplit("/", 1)[0] if "/" in instance_name else ""


def _namespace_levels(instance_name: str) -> tuple[str, ...]:
    levels: list[str] = []
    namespace = _atom_namespace(instance_name)
    while namespace:
        levels.append(namespace)
        namespace = _atom_namespace(namespace)
    levels.append("")
    return tuple(levels)


def _matches_reference(candidate: BlueprintAtom, requested: type) -> bool:
    if is_module_type(requested):
        return issubclass(candidate.module, requested)
    return spec_structural_compliance(candidate.module, requested)


def _resolve_reference_target(
    blueprint: Blueprint,
    consumer: BlueprintAtom,
    reference: ModuleRef,
) -> str | None:
    replacement = blueprint.remapping_map.get((consumer.name, reference.name))
    requested = (
        cast("type", replacement)
        if is_module_type(replacement) or is_spec(replacement)
        else reference.spec
    )

    possible: list[BlueprintAtom] = []
    for namespace in _namespace_levels(consumer.name):
        possible = [
            candidate
            for candidate in blueprint.active_blueprints
            if candidate is not consumer
            and _atom_namespace(candidate.name) == namespace
            and _matches_reference(candidate, requested)
        ]
        if possible:
            break

    if not possible:
        disabled = any(
            atom.module in blueprint.disabled_modules_tuple and _matches_reference(atom, requested)
            for atom in blueprint.blueprints
        )
        if reference.optional or disabled:
            return None
        raise ValueError(
            f"Module reference {consumer.name}.{reference.name} has no provider in the Blueprint"
        )

    valid = (
        possible
        if is_module_type(requested)
        else [
            candidate
            for candidate in possible
            if spec_annotation_compliance(candidate.module, requested)
        ]
    )
    if len(possible) == 1:
        return possible[0].name
    if len(valid) == 1:
        return valid[0].name
    candidates = ", ".join(sorted(candidate.name for candidate in possible))
    raise ValueError(
        f"Module reference {consumer.name}.{reference.name} is ambiguous: {candidates}"
    )


def _validate_module_references(
    blueprint: Blueprint,
    assignments: Mapping[str, str],
) -> None:
    for consumer in blueprint.active_blueprints:
        for reference in consumer.module_refs:
            provider = _resolve_reference_target(blueprint, consumer, reference)
            if provider is None:
                continue
            consumer_host = assignments[consumer.name]
            provider_host = assignments[provider]
            if consumer_host != provider_host:
                raise ValueError(
                    f"Module reference {consumer.name}.{reference.name} crosses Hosts: "
                    f"{consumer_host} -> {provider_host}"
                )
