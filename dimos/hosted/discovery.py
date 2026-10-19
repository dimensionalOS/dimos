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

"""Find dimos Hosts: a router endpoint counts when a live Host names that router as its own."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import json
import threading

from dimos.hosted.config import HostConfig
from dimos.hosted.protocol import HostDescriptor
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_TIMEOUT = 1.0


@dataclass(frozen=True, slots=True)
class Probe:
    """What answered at one endpoint: its router and every Host on that fabric."""

    endpoint: str
    router_zid: str
    hosts: tuple[HostDescriptor, ...]


@dataclass(frozen=True, slots=True)
class Found:
    """One discovered Host and the probed endpoints where its own router answered."""

    descriptor: HostDescriptor
    endpoints: tuple[str, ...]


def scouted_endpoints(config: HostConfig, timeout: float = DEFAULT_TIMEOUT) -> list[str]:
    """Every locator of every router answering a scout on the Host scout group."""
    import zenoh

    zconfig = zenoh.Config()
    zconfig.insert_json5("scouting/multicast/address", json.dumps(config.scout_addr))
    zconfig.insert_json5(
        "scouting/multicast/interface", json.dumps(config.scout_interface or "auto")
    )
    locators: dict[str, None] = {}
    lock = threading.Lock()

    def on_hello(hello: zenoh.Hello) -> None:
        with lock:
            locators.update(dict.fromkeys(str(loc) for loc in hello.locators))

    scout = zenoh.scout(on_hello, what="router", config=zconfig)  # type: ignore[arg-type]
    threading.Event().wait(timeout)
    scout.stop()
    with lock:
        return list(locators)


def probe(endpoint: str, timeout: float = DEFAULT_TIMEOUT) -> Probe | None:
    """Join ``endpoint`` as a client; the Probe if a dimos Host's router answers there."""
    from dimos.hosted.client import describe_all
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    pool = ZenohSessionPool()
    rpc = ZenohRPC(
        session_pool=pool,
        mode="client",
        connect=[endpoint],
        multicast=False,
        connect_timeout=timeout,
    )
    try:
        rpc.start()
        routers = [str(zid) for zid in rpc.session.info.routers_zid()]
        hosts = describe_all(rpc, timeout) if routers else ()
    except Exception:
        return None
    finally:
        try:
            rpc.stop()
        finally:
            pool.close_all()
    if not any(h.router_zid == routers[0] for h in hosts):
        return None
    return Probe(endpoint, routers[0], hosts)


def probe_all(endpoints: Iterable[str], timeout: float = DEFAULT_TIMEOUT) -> list[Probe]:
    """Probe every endpoint side by side."""
    results: list[Probe] = []

    def one(endpoint: str) -> None:
        if (found := probe(endpoint, timeout)) is not None:
            results.append(found)

    threads = [threading.Thread(target=one, args=(e,), daemon=True) for e in endpoints]
    for t in threads:
        t.start()
    # A dial, a liveliness query and a describe each.
    for t in threads:
        t.join(3 * timeout + 1)
    return results


def merge(probes: Iterable[Probe]) -> list[Found]:
    """One row per Host, with the probed endpoints where its own router answered."""
    descriptors: dict[str, HostDescriptor] = {}
    endpoints: dict[str, list[str]] = {}
    for p in probes:
        for host in p.hosts:
            descriptors[host.host_id] = host
            mine = endpoints.setdefault(host.host_id, [])
            if host.router_zid == p.router_zid and p.endpoint not in mine:
                mine.append(p.endpoint)
    rows = [Found(d, tuple(endpoints[i])) for i, d in descriptors.items()]
    return sorted(rows, key=lambda r: r.descriptor.name)
