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

"""Place one resolved Blueprint on Hosts and compile it into immutable per-Host fragments."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Mapping
from dataclasses import replace
from types import MappingProxyType
from typing import Any

from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintAtom,
    HostedPlacement,
)
from dimos.core.coordination.module_coordinator import (
    resolve_module_refs,
    stream_name_types,
    verify_no_name_conflicts,
)
from dimos.hosted.fragment import HostFragment, PythonFragmentPayload, RemoteModuleReference
from dimos.hosted.protocol import HostDescriptor, run_module_rpc_name
from dimos.hosted.tags import missing

# Config sections every fragment keeps; the others are module names.
_SHARED_SECTIONS = ("g", "transports")


def compile_fragments(
    blueprint: Blueprint,
    assignments: Mapping[str, str],
    *,
    run_id: str,
    revision: str,
    overrides: Mapping[str, Any] | None = None,
    global_overrides: Mapping[str, Any] | None = None,
) -> dict[str, HostFragment]:
    """One immutable fragment per Host in ``assignments`` (module name -> Host ID).

    ``overrides`` is the controller's explicit configuration by section; each fragment
    gets its own modules' sections plus the shared ones.
    """
    if not run_id:
        raise ValueError("run_id must not be empty")
    if not revision:
        raise ValueError("revision must not be empty")
    verify_no_name_conflicts(blueprint)

    atoms = tuple(blueprint.active_blueprints)
    atom_names = {atom.name for atom in atoms}
    assignment_names = set(assignments)
    if unplaced := atom_names - assignment_names:
        raise ValueError(f"Placement is missing modules: {', '.join(sorted(unplaced))}")
    if unknown := assignment_names - atom_names:
        raise ValueError(f"Placement contains unknown modules: {', '.join(sorted(unknown))}")
    host_ids = sorted(set(assignments.values()))
    atoms_by_name = {atom.name: atom for atom in atoms}
    resolved_references = resolve_module_refs(blueprint)
    disabled_atoms = tuple(
        atom for atom in blueprint.blueprints if atom not in blueprint.active_blueprints
    )

    fragments: dict[str, HostFragment] = {}
    for host_id in host_ids:
        local_atoms = tuple(
            _with_run_identity(atom, run_id, host_id)
            for atom in atoms
            if assignments[atom.name] == host_id
        )
        local_names = {atom.name for atom in local_atoms}
        remote_references = tuple(
            RemoteModuleReference(
                consumer_name=consumer_name,
                reference_name=reference_name,
                provider_name=provider_name,
                provider_host_id=assignments[provider_name],
                rpcs=frozenset(atoms_by_name[provider_name].module.rpcs),
                rpc_name=run_module_rpc_name(run_id, assignments[provider_name], provider_name),
            )
            for (consumer_name, reference_name), provider_name in sorted(
                resolved_references.items()
            )
            if assignments[consumer_name] == host_id and assignments[provider_name] != host_id
        )
        local_blueprint = _local_blueprint(blueprint, local_atoms, disabled_atoms)
        local_overrides = {
            section: values
            for section, values in (overrides or {}).items()
            if section in local_names or section in _SHARED_SECTIONS
        }
        fragments[host_id] = HostFragment.create(
            run_id=run_id,
            host_id=host_id,
            revision=revision,
            payload=PythonFragmentPayload(
                blueprint=local_blueprint,
                overrides=local_overrides,
                global_overrides=dict(global_overrides or {}),
                remote_module_references=remote_references,
            ),
        )

    return fragments


def resolve_hosted_assignments(
    blueprint: Blueprint,
    hosts: Iterable[HostDescriptor],
    *,
    local_host_id: str | None,
    revision: str,
) -> dict[str, str]:
    """One Host ID per active module; unplaced and ``local`` ones go to ``local_host_id``."""
    if not revision:
        raise ValueError("revision must not be empty")

    atoms = tuple(blueprint.active_blueprints)
    active_names = {atom.name for atom in atoms}
    descriptors = tuple(hosts)
    _validate_host_descriptors(descriptors)

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
            not any(constraint.host for name in names for constraint in constraints_by_name[name]),
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
            revision=revision,
            commitments=commitments,
        )
        assignments.update(dict.fromkeys(names, host_id))
        if host_id in commitments:
            commitments[host_id] += 1

    return assignments


def _validate_host_descriptors(descriptors: tuple[HostDescriptor, ...]) -> None:
    ids = [descriptor.host_id for descriptor in descriptors]
    if any(not host_id for host_id in ids):
        raise ValueError("Discovered Hosts must have non-empty Host IDs")
    if len(set(ids)) != len(ids):
        raise ValueError("Discovered Hosts contain duplicate Host IDs")


def _local_host(
    unit_label: str,
    descriptors: tuple[HostDescriptor, ...],
    local_host_id: str | None,
    revision: str,
) -> str:
    """The local Host's ID for a unit that stays with the controller, if it can take it."""
    local = next((d for d in descriptors if d.host_id == local_host_id), None)
    if local is None:
        raise ValueError(f"Placement unit {unit_label} needs a live local Host, found none")
    if reasons := _host_rejection_reasons(local, frozenset(), revision):
        raise ValueError(f"Local Host {local.name} cannot take {unit_label}: {', '.join(reasons)}")
    return local.host_id


def _select_host_for_unit(
    module_names: tuple[str, ...],
    constraints: tuple[HostedPlacement, ...],
    descriptors: tuple[HostDescriptor, ...],
    *,
    local_host_id: str | None,
    revision: str,
    commitments: Mapping[str, int],
) -> str:
    unit_label = ", ".join(module_names)
    if not constraints:
        return _local_host(unit_label, descriptors, local_host_id, revision)

    local = any(constraint.local for constraint in constraints)
    remote = any(not constraint.local for constraint in constraints)
    exact_hosts = {constraint.host for constraint in constraints if constraint.host}
    required_tags = frozenset(tag for constraint in constraints for tag in constraint.tags)

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
        return _local_host(unit_label, descriptors, local_host_id, revision)

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
        if not _host_rejection_reasons(descriptor, required_tags, revision)
    )
    if not accepted:
        selector = f"host={exact_host!r}" if exact_host is not None else "any Host"
        tags = f", tags={sorted(required_tags)}" if required_tags else ""
        discovered = "; ".join(
            f"{host.name} ({host.host_id}): "
            f"{', '.join(_host_rejection_reasons(host, required_tags, revision)) or 'selector mismatch'}"
            for host in descriptors
        )
        raise ValueError(
            f"No Host satisfies placement for {unit_label} ({selector}{tags}). "
            f"Discovered: {discovered or 'none'}"
        )

    prefer = next((c.prefer for c in constraints if c.prefer), None)
    return min(
        accepted,
        key=lambda host: (
            -_numeric_tag(host, prefer),
            commitments[host.host_id],
            host.host_id,
        ),
    ).host_id


def _numeric_tag(host: HostDescriptor, key: str | None) -> float:
    """The tag as a number for ``prefer`` ranking; absent or non-numeric ranks last."""
    if key is None:
        return 0.0
    try:
        return float(host.tags[key])
    except (KeyError, ValueError):
        return float("-inf")


def _host_rejection_reasons(
    descriptor: HostDescriptor,
    required_tags: frozenset[str],
    revision: str,
) -> tuple[str, ...]:
    reasons: list[str] = []
    if missing_tags := missing(descriptor.tags, required_tags):
        reasons.append(f"missing tags {missing_tags}")
    if descriptor.revision != revision:
        reasons.append(f"revision is {descriptor.revision}, expected {revision}")
    return tuple(reasons)


def _with_run_identity(atom: BlueprintAtom, run_id: str, host_id: str) -> BlueprintAtom:
    """The atom with its run-scoped RPC name."""
    kwargs = {**atom.kwargs, "rpc_name": run_module_rpc_name(run_id, host_id, atom.name)}
    return replace(atom, kwargs=kwargs)


def _local_blueprint(
    source: Blueprint,
    local_atoms: tuple[BlueprintAtom, ...],
    disabled_atoms: tuple[BlueprintAtom, ...],
) -> Blueprint:
    """The source Blueprint cut down to ``local_atoms``; disabled atoms and checks travel with every fragment."""
    local_names = {atom.name for atom in local_atoms}
    local = Blueprint(
        blueprints=local_atoms + disabled_atoms,
        disabled_modules_tuple=source.disabled_modules_tuple,
        transport_map=MappingProxyType({}),
        global_config_overrides=source.global_config_overrides,
        remapping_map=MappingProxyType(
            {key: value for key, value in source.remapping_map.items() if key[0] in local_names}
        ),
        requirement_checks=source.requirement_checks,
        configurator_checks=source.configurator_checks,
    )
    local_streams = stream_name_types(local)
    transports = {k: t for k, t in source.transport_map.items() if k in local_streams}
    return replace(local, transport_map=MappingProxyType(transports))
