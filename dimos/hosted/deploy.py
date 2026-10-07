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

"""Controller: place a hosted Blueprint on live HostDaemons and run it until exit."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
import time
import uuid

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hosted.client import HostClient, discover_hosts
from dimos.hosted.daemon import HostDescriptor, code_revision
from dimos.hosted.fragment import HostFragment
from dimos.hosted.fragment_compiler import compile_fragments
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_RPC_TIMEOUT = 120.0


def _local_descriptor(descriptors: tuple[HostDescriptor, ...], local_host: str) -> HostDescriptor:
    matches = [d for d in descriptors if local_host in (d.host_id, d.name)]
    if len(matches) != 1:
        found = ", ".join(sorted(f"{d.name} ({d.host_id})" for d in descriptors)) or "none"
        raise RuntimeError(f"Need exactly one local Host named {local_host!r}; found: {found}")
    return matches[0]


def wait_for_hosts(
    rpc: ZenohRPC, names: set[str], timeout: float, poll: float = 0.2
) -> tuple[HostDescriptor, ...]:
    """Discover until every named Host is live, or raise with what was found."""
    deadline = time.monotonic() + timeout
    descriptors: tuple[HostDescriptor, ...] = ()
    while time.monotonic() < deadline:
        try:
            descriptors = discover_hosts(rpc, timeout=1.0)
        except TimeoutError:
            descriptors = ()
        if names <= {d.name for d in descriptors} | {d.host_id for d in descriptors}:
            return descriptors
        time.sleep(poll)
    found = ", ".join(sorted(d.name for d in descriptors)) or "none"
    raise TimeoutError(f"Hosts {sorted(names)} not all live; found: {found}")


@contextmanager
def deployed(
    blueprint: Blueprint,
    rpc: ZenohRPC,
    *,
    local_host: str,
    application_name: str,
    descriptors: tuple[HostDescriptor, ...],
    rpc_timeout: float = DEFAULT_RPC_TIMEOUT,
) -> Iterator[Mapping[str, str]]:
    """Start one fragment per placed Host; yield module -> Host name; stop them on exit.

    Unplaced modules go to ``local_host``, the daemon serving this machine.
    """
    local = _local_descriptor(descriptors, local_host)
    remote = tuple(d for d in descriptors if d.host_id != local.host_id)
    run_id = f"{application_name}-{uuid.uuid4().hex[:8]}"
    config = BlueprintConfigParser(blueprint).parse()
    fragments = compile_fragments(
        blueprint,
        config,
        run_id=run_id,
        generation=1,
        application_name=application_name,
        application_revision=code_revision(),
        hosts=remote,
        local_host_id=local.host_id,
    )
    names = {d.host_id: d.name for d in descriptors}
    placement = {
        atom.name: names[host_id]
        for host_id, fragment in fragments.items()
        for atom in fragment.load_payload().blueprint.active_blueprints
    }
    logger.info("Placement", run_id=run_id, placement=placement)

    clients = {d.host_id: HostClient(rpc, d, timeout=rpc_timeout) for d in (local, *remote)}
    started: list[tuple[HostClient, HostFragment]] = []
    try:
        for host_id, fragment in fragments.items():
            status = clients[host_id].start(fragment)
            started.append((clients[host_id], fragment))
            if status.state != "running":
                raise RuntimeError(f"Host {names[host_id]} failed to start {run_id}: {status}")
        yield placement
    finally:
        for client, fragment in reversed(started):
            try:
                client.stop(fragment)
            except Exception:
                logger.error("Failed to stop fragment", host=client.descriptor.name, exc_info=True)
