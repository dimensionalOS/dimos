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

"""Immutable deployment payloads accepted by a DimOS Host."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import pickle

from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
from dimos.core.coordination.blueprints import Blueprint, TransportSpec
from dimos.core.module import ModuleBase
from dimos.core.transport import ZenohTransport, pZenohTransport

FRAGMENT_SCHEMA_VERSION = 1
FRAGMENT_FORMAT = "python-blueprint"
RUN_STREAM_KEY = "dimos/runs/{run_id}/streams/{stream}/{message_type}"
RUN_MODULE_RPC_NAME = "runs/{run_id}/hosts/{host_id}/modules/{module}"


def message_type_name(message_type: type) -> str:
    """Return the stable name used as the final Zenoh topic segment."""
    return str(
        getattr(
            message_type,
            "msg_name",
            f"{message_type.__module__}.{message_type.__qualname__}",
        )
    )


def run_stream_base_topic(run_id: str, stream_name: str) -> str:
    """Return the run-scoped base topic for a hosted stream."""
    return f"dimos/runs/{run_id}/streams/{stream_name.lstrip('/')}"


def run_stream_key(run_id: str, stream_name: str, message_type: type) -> str:
    """Return the complete hosted Zenoh key expression for a typed stream."""
    return RUN_STREAM_KEY.format(
        run_id=run_id,
        stream=stream_name.lstrip("/"),
        message_type=message_type_name(message_type),
    )


def run_module_rpc_name(run_id: str, host_id: str, module_name: str) -> str:
    """Return the run- and Host-scoped logical RPC name for a module."""
    return RUN_MODULE_RPC_NAME.format(
        run_id=run_id,
        host_id=host_id,
        module=module_name.lstrip("/"),
    )


@dataclass(frozen=True, slots=True)
class BoundaryStream:
    """A stream whose endpoints are deployed on more than one Host."""

    name: str
    message_type: type
    key_expr: str


@dataclass(frozen=True, slots=True)
class RemoteModuleReference:
    """A local module attribute backed by a provider on another Host."""

    consumer_name: str
    reference_name: str
    provider_name: str
    provider_host_id: str
    provider_type: type[ModuleBase]
    rpc_name: str


@dataclass(frozen=True, slots=True)
class PythonFragmentPayload:
    """The local Blueprint and its identity-bound parsed configuration."""

    blueprint: Blueprint
    config: ParsedBlueprintConfig
    boundary_streams: tuple[BoundaryStream, ...] = ()
    remote_module_references: tuple[RemoteModuleReference, ...] = ()


@dataclass(frozen=True, slots=True)
class HostFragment:
    """Versioned, immutable deployment unit sent to one Host."""

    run_id: str
    generation: int
    host_id: str
    application_name: str
    application_revision: str
    payload_digest: str
    payload: bytes
    schema_version: int = FRAGMENT_SCHEMA_VERSION
    format: str = FRAGMENT_FORMAT

    @classmethod
    def create(
        cls,
        *,
        run_id: str,
        generation: int,
        host_id: str,
        application_name: str,
        application_revision: str,
        payload: PythonFragmentPayload,
    ) -> HostFragment:
        """Serialize and digest a Python fragment payload exactly once."""
        payload_bytes = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
        return cls(
            run_id=run_id,
            generation=generation,
            host_id=host_id,
            application_name=application_name,
            application_revision=application_revision,
            payload_digest=hashlib.sha256(payload_bytes).hexdigest(),
            payload=payload_bytes,
        )

    def validate_digest(self) -> None:
        """Reject payload bytes that do not match the accepted digest."""
        if hashlib.sha256(self.payload).hexdigest() != self.payload_digest:
            raise ValueError("Fragment payload digest does not match its payload")

    def load_payload(self) -> PythonFragmentPayload:
        """Validate and deserialize the trusted Python payload."""
        self.validate_digest()
        payload = pickle.loads(self.payload)
        if not isinstance(payload, PythonFragmentPayload):
            raise TypeError("Fragment payload is not a PythonFragmentPayload")
        payload.config.assert_matches(payload.blueprint)
        validate_boundary_streams(self.run_id, payload)
        validate_remote_module_references(self, payload)
        return payload


def validate_boundary_streams(run_id: str, payload: PythonFragmentPayload) -> None:
    """Validate boundary manifests and their pinned Blueprint transports."""
    for boundary in payload.boundary_streams:
        expected_key = run_stream_key(run_id, boundary.name, boundary.message_type)
        if boundary.key_expr != expected_key:
            raise ValueError(
                f"Boundary stream {boundary.name!r} uses {boundary.key_expr!r}, "
                f"expected {expected_key!r}"
            )

        stream_key = (boundary.name, boundary.message_type)
        transport = payload.blueprint.transport_map.get(stream_key)
        if not isinstance(transport, TransportSpec):
            raise ValueError(f"Boundary stream {boundary.name!r} has no pinned transport")

        if getattr(boundary.message_type, "lcm_encode", None) is None:
            valid = transport.cls is pZenohTransport and transport.args == (expected_key,)
        else:
            expected_base = run_stream_base_topic(run_id, boundary.name)
            valid = transport.cls is ZenohTransport and transport.args == (
                expected_base,
                boundary.message_type,
            )
        if not valid:
            raise ValueError(f"Boundary stream {boundary.name!r} is not pinned to Zenoh")


def validate_remote_module_references(
    fragment: HostFragment,
    payload: PythonFragmentPayload,
) -> None:
    """Validate that every remote reference targets its canonical RPC address."""
    local_atoms = {atom.name: atom for atom in payload.blueprint.active_blueprints}
    seen: set[tuple[str, str]] = set()
    for reference in payload.remote_module_references:
        key = (reference.consumer_name, reference.reference_name)
        if key in seen:
            raise ValueError(
                f"Remote module reference {reference.consumer_name}.{reference.reference_name} "
                "is duplicated"
            )
        seen.add(key)

        consumer = local_atoms.get(reference.consumer_name)
        if consumer is None or reference.reference_name not in {
            module_ref.name for module_ref in consumer.module_refs
        }:
            raise ValueError(
                f"Remote module reference {reference.consumer_name}.{reference.reference_name} "
                "does not exist in the fragment Blueprint"
            )
        if reference.provider_host_id == fragment.host_id:
            raise ValueError(
                f"Remote module reference {reference.consumer_name}.{reference.reference_name} "
                "targets its local Host"
            )

        expected_name = run_module_rpc_name(
            fragment.run_id,
            reference.provider_host_id,
            reference.provider_name,
        )
        if reference.rpc_name != expected_name:
            raise ValueError(
                f"Remote module reference {reference.consumer_name}.{reference.reference_name} "
                f"uses {reference.rpc_name!r}, expected {expected_name!r}"
            )
