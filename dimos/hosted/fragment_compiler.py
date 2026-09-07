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
from collections.abc import Iterable, Mapping
from dataclasses import replace
from types import MappingProxyType
from typing import Any, cast

from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintAtom,
    HostedPlacement,
    ModuleRef,
    TransportSpec,
)
from dimos.core.module import is_module_type
from dimos.core.transport import ZenohTransport, pZenohTransport
from dimos.hosted.daemon import HostDescriptor
from dimos.hosted.fragment import (
    BoundaryStream,
    HostFragment,
    PythonFragmentPayload,
    RemoteModuleReference,
    run_module_rpc_name,
    run_stream_base_topic,
    run_stream_key,
)
from dimos.spec.utils import is_spec, spec_annotation_compliance, spec_structural_compliance

HOSTED_GLOBAL_OVERRIDES: Mapping[str, Any] = MappingProxyType({"transport": "zenoh"})

StreamKey = tuple[str, type]
_SCHEDULABLE_HOST_STATES = frozenset({"available", "running"})


def compile_fragments(
    blueprint: Blueprint,
    config: ParsedBlueprintConfig,
    assignments: Mapping[str, str] | None = None,
    *,
    run_id: str,
    generation: int,
    application_name: str,
    application_revision: str,
    hosts: Iterable[HostDescriptor] = (),
    local_host_id: str | None = None,
) -> dict[str, HostFragment]:
    """Compile a Blueprint into one immutable fragment per assigned Host.

    Explicit ``assignments`` remain available for callers that already resolved
    placement. Otherwise, the Blueprint's ``hosted()`` metadata is resolved
    automatically against ``hosts``; modules without that metadata stay on the
    controlling Host.
    """
    _validate_metadata(run_id, generation, application_name, application_revision)
    config.assert_matches(blueprint)

    host_descriptors = tuple(hosts)
    resolved_assignments: Mapping[str, str]
    if assignments is None:
        resolved_assignments = resolve_hosted_assignments(
            blueprint,
            host_descriptors,
            local_host_id=local_host_id if local_host_id is not None else f"local-{run_id}",
            application_revision=application_revision,
        )
    else:
        if host_descriptors or local_host_id is not None:
            raise ValueError("Pass either explicit assignments or hosts, not both")
        resolved_assignments = assignments

    atoms = tuple(blueprint.active_blueprints)
    atom_names = {atom.name for atom in atoms}
    assignment_names = set(resolved_assignments)
    if missing := atom_names - assignment_names:
        raise ValueError(f"Placement is missing modules: {', '.join(sorted(missing))}")
    if unknown := assignment_names - atom_names:
        raise ValueError(f"Placement contains unknown modules: {', '.join(sorted(unknown))}")
    if invalid_hosts := sorted(
        name
        for name, host_id in resolved_assignments.items()
        if not isinstance(host_id, str) or not host_id
    ):
        raise ValueError(f"Placement has empty Host IDs for: {', '.join(invalid_hosts)}")

    host_ids = sorted(set(resolved_assignments.values()))
    if len(host_ids) > 1 and (blueprint.requirement_checks or blueprint.configurator_checks):
        raise ValueError(
            "Multi-Host fragments do not yet support Blueprint requirements or configurators"
        )

    resolved_references = _resolve_module_references(blueprint)
    stream_endpoints = _stream_endpoints(blueprint)
    boundary_keys = {
        key
        for key, endpoint_names in stream_endpoints.items()
        if len({resolved_assignments[name] for name in endpoint_names}) > 1
    }
    disabled_atoms = tuple(
        atom for atom in blueprint.blueprints if atom not in blueprint.active_blueprints
    )

    fragments: dict[str, HostFragment] = {}
    for host_id in host_ids:
        local_atoms = tuple(
            _with_rpc_name(atom, run_id, host_id)
            for atom in atoms
            if resolved_assignments[atom.name] == host_id
        )
        local_names = {atom.name for atom in local_atoms}
        remote_references = tuple(
            _remote_module_reference(
                run_id,
                consumer_name,
                reference_name,
                provider,
                resolved_assignments[provider.name],
            )
            for (consumer_name, reference_name), provider in sorted(resolved_references.items())
            if resolved_assignments[consumer_name] == host_id
            and resolved_assignments[provider.name] != host_id
        )
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
                remote_module_references=remote_references,
            ),
        )

    return fragments


def resolve_hosted_assignments(
    blueprint: Blueprint,
    hosts: Iterable[HostDescriptor],
    *,
    local_host_id: str,
    application_revision: str,
) -> dict[str, str]:
    """Resolve Blueprint placement metadata to one Host ID per active module."""
    if not local_host_id:
        raise ValueError("local_host_id must not be empty")
    if not application_revision:
        raise ValueError("application_revision must not be empty")

    atoms = tuple(blueprint.active_blueprints)
    active_names = {atom.name for atom in atoms}
    descriptors = tuple(hosts)
    _validate_host_descriptors(descriptors, local_host_id)

    parents = {name: name for name in active_names}

    def find(name: str) -> str:
        while parents[name] != name:
            parents[name] = parents[parents[name]]
            name = parents[name]
        return name

    def union(names: Iterable[str]) -> None:
        members = tuple(name for name in names if name in active_names)
        if not members:
            return
        root = find(members[0])
        for name in members[1:]:
            other = find(name)
            if other != root:
                parents[other] = root

    placements = tuple(
        placement
        for placement in blueprint.hosted_placements
        if active_names.intersection(placement.module_names)
    )
    for placement in placements:
        union(placement.module_names)

    for (consumer_name, _reference_name), provider in _resolve_module_references(blueprint).items():
        union((consumer_name, provider.name))

    units: dict[str, list[str]] = defaultdict(list)
    for atom in atoms:
        units[find(atom.name)].append(atom.name)

    constraints_by_name: dict[str, list[HostedPlacement]] = defaultdict(list)
    for placement in placements:
        for name in placement.module_names:
            if name in active_names:
                constraints_by_name[name].append(placement)

    placement_units = [tuple(sorted(names)) for names in units.values()]
    placement_units.sort(
        key=lambda names: (
            not any(
                constraint.host is not None
                for name in names
                for constraint in constraints_by_name[name]
            ),
            names,
        )
    )

    commitments = {descriptor.host_id: len(descriptor.active_run_ids) for descriptor in descriptors}
    assignments: dict[str, str] = {}
    for names in placement_units:
        constraints = tuple(
            constraint for name in names for constraint in constraints_by_name[name]
        )
        host_id = _select_host_for_unit(
            names,
            constraints,
            descriptors,
            local_host_id=local_host_id,
            application_revision=application_revision,
            commitments=commitments,
        )
        assignments.update(dict.fromkeys(names, host_id))
        if host_id in commitments:
            commitments[host_id] += 1

    return assignments


def _validate_host_descriptors(descriptors: tuple[HostDescriptor, ...], local_host_id: str) -> None:
    ids = [descriptor.host_id for descriptor in descriptors]
    if any(not host_id for host_id in ids):
        raise ValueError("Discovered Hosts must have non-empty Host IDs")
    if len(set(ids)) != len(ids):
        raise ValueError("Discovered Hosts contain duplicate Host IDs")
    if local_host_id in ids:
        raise ValueError(f"Local Host ID {local_host_id!r} collides with a discovered Host")


def _select_host_for_unit(
    module_names: tuple[str, ...],
    constraints: tuple[HostedPlacement, ...],
    descriptors: tuple[HostDescriptor, ...],
    *,
    local_host_id: str,
    application_revision: str,
    commitments: Mapping[str, int],
) -> str:
    if not constraints:
        return local_host_id

    local = any(constraint.local for constraint in constraints)
    remote = any(not constraint.local for constraint in constraints)
    exact_hosts = {constraint.host for constraint in constraints if constraint.host is not None}
    required_tags = frozenset(tag for constraint in constraints for tag in constraint.tags)

    unit_label = ", ".join(module_names)
    if local and remote:
        raise ValueError(
            f"Placement unit {unit_label} combines local and remote hosted constraints"
        )
    if len(exact_hosts) > 1:
        raise ValueError(
            f"Placement unit {unit_label} has conflicting exact Hosts: "
            f"{', '.join(sorted(exact_hosts))}"
        )
    if local:
        return local_host_id

    exact_host = next(iter(exact_hosts), None)
    candidates = descriptors
    if exact_host is not None:
        id_matches = tuple(host for host in descriptors if host.host_id == exact_host)
        if id_matches:
            candidates = id_matches
        else:
            name_matches = tuple(host for host in descriptors if host.name == exact_host)
            if len(name_matches) > 1:
                ids = ", ".join(sorted(host.host_id for host in name_matches))
                raise ValueError(
                    f"Exact Host name {exact_host!r} for placement unit {unit_label} "
                    f"is ambiguous: {ids}"
                )
            candidates = name_matches

    accepted = tuple(
        descriptor
        for descriptor in candidates
        if not _host_rejection_reasons(descriptor, required_tags, application_revision)
    )
    if not accepted:
        selector = f"host={exact_host!r}" if exact_host is not None else "any Host"
        tags = f", tags={sorted(required_tags)}" if required_tags else ""
        discovered = "; ".join(
            f"{host.name} ({host.host_id}): "
            f"{', '.join(_host_rejection_reasons(host, required_tags, application_revision)) or 'selector mismatch'}"
            for host in descriptors
        )
        raise ValueError(
            f"No Host satisfies placement for {unit_label} ({selector}{tags}). "
            f"Discovered: {discovered or 'none'}"
        )

    return min(accepted, key=lambda host: (commitments[host.host_id], host.host_id)).host_id


def _host_rejection_reasons(
    descriptor: HostDescriptor,
    required_tags: frozenset[str],
    application_revision: str,
) -> tuple[str, ...]:
    reasons: list[str] = []
    if descriptor.state not in _SCHEDULABLE_HOST_STATES:
        reasons.append(f"state is {descriptor.state}")
    if missing_tags := required_tags - descriptor.tags:
        reasons.append(f"missing tags {sorted(missing_tags)}")
    host_revision = descriptor.versions.get(
        "application_revision", descriptor.versions.get("dimos")
    )
    if host_revision is not None and str(host_revision) != application_revision:
        reasons.append(
            f"application revision is {host_revision!s}, expected {application_revision}"
        )
    return tuple(reasons)


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


def _with_rpc_name(
    atom: BlueprintAtom,
    run_id: str,
    host_id: str,
) -> BlueprintAtom:
    kwargs = dict(atom.kwargs)
    kwargs["rpc_name"] = run_module_rpc_name(run_id, host_id, atom.name)
    return replace(atom, kwargs=kwargs)


def _remote_module_reference(
    run_id: str,
    consumer_name: str,
    reference_name: str,
    provider: BlueprintAtom,
    provider_host_id: str,
) -> RemoteModuleReference:
    return RemoteModuleReference(
        consumer_name=consumer_name,
        reference_name=reference_name,
        provider_name=provider.name,
        provider_host_id=provider_host_id,
        provider_type=provider.module,
        rpc_name=run_module_rpc_name(run_id, provider_host_id, provider.name),
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
) -> BlueprintAtom | None:
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
        return possible[0]
    if len(valid) == 1:
        return valid[0]
    candidates = ", ".join(sorted(candidate.name for candidate in possible))
    raise ValueError(
        f"Module reference {consumer.name}.{reference.name} is ambiguous: {candidates}"
    )


def _resolve_module_references(
    blueprint: Blueprint,
) -> dict[tuple[str, str], BlueprintAtom]:
    resolved: dict[tuple[str, str], BlueprintAtom] = {}
    for consumer in blueprint.active_blueprints:
        for reference in consumer.module_refs:
            provider = _resolve_reference_target(blueprint, consumer, reference)
            if provider is not None:
                resolved[consumer.name, reference.name] = provider
    return resolved
