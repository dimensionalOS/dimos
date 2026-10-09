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

"""Controller-side discovery and lifecycle calls for DimOS Hosts."""

from __future__ import annotations

from typing import TypeVar

from dimos.hosted.daemon import (
    HOST_CONTROL_RPC_NAME,
    DeploymentStatus,
    HostDescriptor,
)
from dimos.hosted.fragment import HostFragment
from dimos.protocol.rpc.spec import Args
from dimos.protocol.rpc.zenohrpc import ZenohRPC

HOST_DISCOVERY_KEY = "dimos/hosts/*/live"

ResponseT = TypeVar("ResponseT")


def discover_host_ids(rpc: ZenohRPC, timeout: float) -> tuple[str, ...]:
    """Return the stable IDs of all Hosts with live tokens on this fabric."""
    replies = rpc.session.liveliness().get(HOST_DISCOVERY_KEY, timeout=timeout)
    host_ids: set[str] = set()
    for reply in replies:
        sample = reply.ok
        if sample is None:
            continue
        key = str(sample.key_expr)
        parts = key.split("/")
        if len(parts) == 4 and parts[:2] == ["dimos", "hosts"] and parts[3] == "live":
            host_ids.add(parts[2])
    return tuple(sorted(host_ids))


def get_host_descriptor(rpc: ZenohRPC, host_id: str, timeout: float) -> HostDescriptor:
    """Fetch and validate one live Host's current descriptor."""
    control_name = HOST_CONTROL_RPC_NAME.format(host_id=host_id)
    result, unsubscribe = rpc.call_sync(
        f"{control_name}/describe",
        ([], {}),
        rpc_timeout=timeout,
    )
    try:
        if not isinstance(result, HostDescriptor):
            raise TypeError(f"Host {host_id} returned an invalid descriptor")
        return result
    finally:
        unsubscribe()


def discover_hosts(rpc: ZenohRPC, timeout: float) -> tuple[HostDescriptor, ...]:
    """Discover live Hosts and fetch their authoritative descriptors."""
    return tuple(
        get_host_descriptor(rpc, host_id, timeout) for host_id in discover_host_ids(rpc, timeout)
    )


class HostClient:
    """Lifecycle proxy bound to one Host identity and process epoch."""

    def __init__(self, rpc: ZenohRPC, descriptor: HostDescriptor, *, timeout: float) -> None:
        self._rpc = rpc
        self._descriptor = descriptor
        self._timeout = timeout
        self._control_name = HOST_CONTROL_RPC_NAME.format(host_id=descriptor.host_id)

    @property
    def descriptor(self) -> HostDescriptor:
        return self._descriptor

    def start(self, fragment: HostFragment) -> DeploymentStatus:
        """Start or retrieve the idempotent deployment represented by a fragment."""
        return self._call(
            "start",
            ([self._descriptor.epoch, fragment], {}),
            DeploymentStatus,
        )

    def status(self, run_id: str) -> DeploymentStatus:
        """Return this Host's deployment status for a run."""
        return self._call(
            "status",
            ([self._descriptor.epoch, run_id], {}),
            DeploymentStatus,
        )

    def stop(self, fragment: HostFragment) -> DeploymentStatus:
        """Stop the exact fragment generation previously accepted by this Host."""
        return self._call(
            "stop",
            (
                [
                    self._descriptor.epoch,
                    fragment.run_id,
                    fragment.generation,
                    fragment.payload_digest,
                ],
                {},
            ),
            DeploymentStatus,
        )

    def _call(
        self,
        operation: str,
        arguments: Args,
        response_type: type[ResponseT],
    ) -> ResponseT:
        result, unsubscribe = self._rpc.call_sync(
            f"{self._control_name}/{operation}",
            arguments,
            rpc_timeout=self._timeout,
        )
        try:
            if not isinstance(result, response_type):
                raise TypeError(
                    f"Host {self._descriptor.host_id} returned an invalid {operation} response"
                )
            return result
        finally:
            unsubscribe()
