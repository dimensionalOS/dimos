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

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager, suppress
from typing import TypeVar

from dimos.hosted.fragment import HostFragment
from dimos.hosted.protocol import (
    HOST_CONTROL_RPC_NAME,
    HOST_LIVELINESS_KEY,
    DeploymentStatus,
    HostDescriptor,
)
from dimos.protocol.rpc.spec import Args
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.zenohservice import ZenohSessionPool

ResponseT = TypeVar("ResponseT")


@contextmanager
def controller_rpc(connect: list[str]) -> Iterator[ZenohRPC]:
    """An RPC client of the router at ``connect``, on its own session, for the block."""
    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool, mode="client", connect=connect, multicast=False)
    with ExitStack() as cleanup:
        cleanup.callback(pool.close_all)
        rpc.start()
        cleanup.callback(rpc.stop)
        yield rpc


def discover_host_ids(rpc: ZenohRPC, timeout: float) -> tuple[str, ...]:
    """The stable IDs of all Hosts with live tokens on this fabric."""
    prefix, _, suffix = HOST_LIVELINESS_KEY.partition("{host_id}")
    replies = rpc.session.liveliness().get(HOST_LIVELINESS_KEY.format(host_id="*"), timeout=timeout)
    host_ids: set[str] = set()
    for reply in replies:
        sample = reply.ok
        if sample is None:
            continue
        key = str(sample.key_expr)
        host_id = key.removeprefix(prefix).removesuffix(suffix)
        if key == f"{prefix}{host_id}{suffix}" and host_id and "/" not in host_id:
            host_ids.add(host_id)
    return tuple(sorted(host_ids))


def discover(rpc: ZenohRPC, timeout: float, rpc_timeout: float | None = None) -> list[HostClient]:
    """A client per live Host, none described yet; ``timeout`` bounds the liveliness query."""
    return [
        HostClient(rpc, host_id, timeout=rpc_timeout or timeout)
        for host_id in discover_host_ids(rpc, timeout)
    ]


def describe_all(
    rpc: ZenohRPC, timeout: float, rpc_timeout: float | None = None
) -> tuple[HostDescriptor, ...]:
    """Every live Host that describes itself in time."""
    descriptors = []
    for client in discover(rpc, timeout, rpc_timeout):
        with suppress(Exception):
            descriptors.append(client.describe())
    return tuple(descriptors)


class HostClient:
    """Lifecycle proxy bound to one Host."""

    def __init__(self, rpc: ZenohRPC, host_id: str, *, timeout: float) -> None:
        self._rpc = rpc
        self.host_id = host_id
        self._timeout = timeout
        self._control_name = HOST_CONTROL_RPC_NAME.format(host_id=host_id)

    def describe(self) -> HostDescriptor:
        return self._call("describe", ([], {}), HostDescriptor)

    def start(self, fragment: HostFragment) -> DeploymentStatus:
        """Start, or retrieve the status of, the idempotent deployment of ``fragment``."""
        return self._call("start", ([fragment], {}), DeploymentStatus)

    def status(self, run_id: str) -> DeploymentStatus:
        return self._call("status", ([run_id], {}), DeploymentStatus)

    def stop(self, run_id: str) -> DeploymentStatus:
        return self._call("stop", ([run_id], {}), DeploymentStatus)

    def _call(self, operation: str, arguments: Args, response_type: type[ResponseT]) -> ResponseT:
        result, unsubscribe = self._rpc.call_sync(
            f"{self._control_name}/{operation}", arguments, rpc_timeout=self._timeout
        )
        try:
            if not isinstance(result, response_type):
                raise TypeError(f"Host {self.host_id} returned an invalid {operation} response")
            return result
        finally:
            unsubscribe()
