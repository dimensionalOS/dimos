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

"""Find dimos Hosts on the network: scouting, the Go2 LAN probe, and seed endpoints.

A candidate endpoint counts only when a Host answers there: a client session to it
reaches a router whose zid some live Host descriptor names as its own.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import json
import socket
import threading
import time
from typing import TYPE_CHECKING

from dimos.hosted.daemon import DIMOS_SCOUT_ADDR, HostDescriptor
from dimos.protocol.service.zenohservice import ROBOT_ZENOH_PORT
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

logger = setup_logger()

DEFAULT_TIMEOUT = 1.0


@dataclass(frozen=True, slots=True)
class Probe:
    """What answered at one endpoint: its router and every Host on that fabric."""

    endpoint: str
    router_zid: str
    hosts: tuple[HostDescriptor, ...]
    # Round trip of each Host's describe call through this router, in ms.
    rtt_ms: Mapping[str, float] = field(default_factory=dict)


def is_loopback(endpoint: str) -> bool:
    host = endpoint.partition("/")[2].rpartition(":")[0].strip("[]")
    return host.startswith("127.") or host in ("::1", "localhost")


def _local_prefixes() -> set[str]:
    """The /24 prefixes of this machine's IPv4 addresses."""
    import psutil

    return {
        a.address.rsplit(".", 1)[0]
        for addrs in psutil.net_if_addrs().values()
        for a in addrs
        if a.family == socket.AF_INET and not a.address.startswith("127.")
    }


def _nearest_first(locators: Iterable[str]) -> tuple[str, ...]:
    """Locators on one of our /24s first: a router also advertises bridges we cannot reach."""
    prefixes = _local_prefixes()

    def far(locator: str) -> bool:
        host = locator.partition("/")[2].rpartition(":")[0]
        return host.rsplit(".", 1)[0] not in prefixes

    return tuple(sorted(locators, key=far))


def scouted_endpoints(
    scout_addr: str = DIMOS_SCOUT_ADDR,
    interface: str = "auto",
    timeout: float = DEFAULT_TIMEOUT,
    exclude: Collection[str] = (),
) -> list[tuple[str, ...]]:
    """Each router answering on the dimos scouting group, but those in ``exclude``, as its
    locators nearest first."""
    import zenoh

    config = zenoh.Config()
    config.insert_json5("scouting/multicast/address", json.dumps(scout_addr))
    config.insert_json5("scouting/multicast/interface", json.dumps(interface or "auto"))
    routers: dict[str, list[str]] = {}
    lock = threading.Lock()

    def on_hello(hello: zenoh.Hello) -> None:
        if str(hello.zid) in exclude:
            return
        with lock:
            seen = routers.setdefault(str(hello.zid), [])
            seen += [str(loc) for loc in hello.locators if str(loc) not in seen]

    scout = zenoh.scout(on_hello, what="router", config=config)  # type: ignore[arg-type]
    threading.Event().wait(timeout)
    scout.stop()
    with lock:
        return [_nearest_first(locators) for locators in routers.values()]


def go2_endpoints(timeout: float = DEFAULT_TIMEOUT) -> list[str]:
    """The zenoh port of every Go2 answering the LAN probe; the Go2 forwards it onboard."""
    from dimos.robot.unitree.go2.cli.landiscovery import discover

    return [f"tcp/{device.ip}:{ROBOT_ZENOH_PORT}" for device in discover(timeout=timeout)]


Group = tuple[str, ...]
# The Go2 answers its LAN probe slower than routers answer a scout.
GO2_PROBE_TIMEOUT = 2.0


def candidates(
    seeds: Sequence[str] = (),
    *,
    scout: bool = True,
    go2: bool = True,
    scout_addr: str = DIMOS_SCOUT_ADDR,
    scout_interface: str = "",
    timeout: float = DEFAULT_TIMEOUT,
    exclude: Collection[str] = (),
) -> list[Group]:
    """Endpoint groups to probe, one per router: a group's endpoints are alternatives.

    Seeds come as given, discovered endpoints only off loopback: another machine is never
    there, and other zenoh routers on this one are not ours to dial.
    """
    sources: list[Callable[[], list[Group]]] = []
    if scout:
        sources.append(lambda: scouted_endpoints(scout_addr, scout_interface, timeout, exclude))
    if go2:
        sources.append(lambda: [(e,) for e in go2_endpoints(max(timeout, GO2_PROBE_TIMEOUT))])
    results: list[list[Group]] = [[] for _ in sources]

    def collect(index: int) -> None:
        try:
            groups = (tuple(e for e in g if not is_loopback(e)) for g in sources[index]())
            results[index] = [g for g in groups if g]
        except Exception:
            logger.warning("Host discovery source failed", exc_info=True)

    # Sources run side by side: each waits out its own timeout.
    threads = [
        threading.Thread(target=collect, args=(i,), daemon=True) for i in range(len(sources))
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    groups: list[Group] = [(seed,) for seed in dict.fromkeys(seeds)]
    seen = set(seeds)
    for group in (g for result in results for g in result):
        fresh = tuple(e for e in group if e not in seen)
        seen.update(fresh)
        if fresh:
            groups.append(fresh)
    return groups


def probe(endpoint: str, timeout: float = DEFAULT_TIMEOUT) -> Probe | None:
    """Join ``endpoint`` as a client; the Probe if a dimos Host's router answers there."""
    from dimos.hosted.client import discover_host_ids
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
        if not routers:
            return None
        timed = [_describe(rpc, host_id, timeout) for host_id in discover_host_ids(rpc, timeout)]
        hosts = tuple(host for host, _ in timed)
        rtt_ms = {host.host_id: ms for host, ms in timed if ms is not None}
    except Exception:
        return None
    finally:
        try:
            rpc.stop()
        finally:
            pool.close_all()
    # A router whose own Host is too busy to describe itself still counts as a dimos Host.
    if not any(h.router_zid in (routers[0], "") for h in hosts):
        return None
    return Probe(endpoint, routers[0], hosts, rtt_ms)


def _describe(rpc: ZenohRPC, host_id: str, timeout: float) -> tuple[HostDescriptor, float | None]:
    """The Host's descriptor and its round trip, or a stand-in saying it did not answer."""
    from dimos.hosted.client import get_host_descriptor

    start = time.perf_counter()
    try:
        return get_host_descriptor(rpc, host_id, timeout), (time.perf_counter() - start) * 1e3
    except Exception:
        return HostDescriptor(host_id, "", host_id[:12], {}, {}, "unresponsive", ()), None


def rtts(probes: Iterable[Probe]) -> dict[str, float]:
    """Each Host's fastest describe round trip over every probed router, in ms."""
    best: dict[str, float] = {}
    for p in probes:
        for host_id, ms in p.rtt_ms.items():
            best[host_id] = min(ms, best.get(host_id, ms))
    return best


def probe_all(
    groups: Iterable[str | Sequence[str]], timeout: float = DEFAULT_TIMEOUT
) -> list[Probe]:
    """Probe groups side by side; within a group, endpoints in order until one answers."""
    results: list[Probe] = []

    def first(group: Sequence[str]) -> None:
        for endpoint in group:
            if (found := probe(endpoint, timeout)) is not None:
                results.append(found)
                return

    threads = [
        threading.Thread(target=first, args=((g,) if isinstance(g, str) else g,), daemon=True)
        for g in groups
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout * 8 + 5)
    return results


def merge(probes: Iterable[Probe]) -> list[tuple[HostDescriptor, tuple[str, ...]]]:
    """One row per Host, with the probed endpoints where its own router answered."""
    hosts: dict[str, HostDescriptor] = {}
    endpoints: dict[str, list[str]] = {}
    for p in probes:
        for host in p.hosts:
            if hosts.get(host.host_id, host).state == "unresponsive" or host.host_id not in hosts:
                hosts[host.host_id] = host
            mine = endpoints.setdefault(host.host_id, [])
            if host.router_zid == p.router_zid and p.endpoint not in mine:
                mine.append(p.endpoint)
    return [(hosts[i], tuple(endpoints[i])) for i in sorted(hosts, key=lambda i: hosts[i].name)]


def endpoints_to_link(
    probes: Iterable[Probe], *, own_zid: str, linked: Iterable[str], connect: Sequence[str]
) -> list[str]:
    """One endpoint per router not yet linked, not ourselves, not already dialed.

    Both sides may find each other; zenoh keeps one transport per router pair, so a
    second dial in the other direction is harmless and nothing flaps.
    """
    skip = {own_zid, *linked}
    dialed = {p.router_zid for p in probes if p.endpoint in connect}
    out: dict[str, str] = {}
    for p in probes:
        if p.router_zid not in skip and p.router_zid not in dialed:
            out.setdefault(p.router_zid, p.endpoint)
    return list(out.values())
