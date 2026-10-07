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

from collections.abc import Callable, Collection, Iterable, Sequence
from dataclasses import dataclass
import json
import threading

from dimos.hosted.daemon import DIMOS_SCOUT_ADDR, HostDescriptor
from dimos.protocol.service.zenohservice import ROBOT_ZENOH_PORT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_TIMEOUT = 1.0


@dataclass(frozen=True, slots=True)
class Probe:
    """What answered at one endpoint: its router and every Host on that fabric."""

    endpoint: str
    router_zid: str
    hosts: tuple[HostDescriptor, ...]


def is_loopback(endpoint: str) -> bool:
    host = endpoint.partition("/")[2].rpartition(":")[0].strip("[]")
    return host.startswith("127.") or host in ("::1", "localhost")


def scouted_endpoints(
    scout_addr: str = DIMOS_SCOUT_ADDR,
    interface: str = "auto",
    timeout: float = DEFAULT_TIMEOUT,
    exclude: Collection[str] = (),
) -> list[str]:
    """Locators of the routers answering on the dimos scouting group, but those in ``exclude``."""
    import zenoh

    config = zenoh.Config()
    config.insert_json5("scouting/multicast/address", json.dumps(scout_addr))
    config.insert_json5("scouting/multicast/interface", json.dumps(interface or "auto"))
    hellos: list[str] = []
    lock = threading.Lock()

    def on_hello(hello: zenoh.Hello) -> None:
        if str(hello.zid) in exclude:
            return
        with lock:
            hellos.extend(str(locator) for locator in hello.locators)

    scout = zenoh.scout(on_hello, what="router", config=config)  # type: ignore[arg-type]
    threading.Event().wait(timeout)
    scout.stop()
    with lock:
        return list(dict.fromkeys(hellos))


def go2_endpoints(timeout: float = DEFAULT_TIMEOUT) -> list[str]:
    """The zenoh port of every Go2 answering the LAN probe; the Go2 forwards it onboard."""
    from dimos.robot.unitree.go2.cli.landiscovery import discover

    return [f"tcp/{device.ip}:{ROBOT_ZENOH_PORT}" for device in discover(timeout=timeout)]


def candidates(
    seeds: Sequence[str] = (),
    *,
    scout: bool = True,
    go2: bool = True,
    scout_addr: str = DIMOS_SCOUT_ADDR,
    scout_interface: str = "",
    timeout: float = DEFAULT_TIMEOUT,
    exclude: Collection[str] = (),
) -> list[str]:
    """Seeds as given, then discovered endpoints off loopback, without duplicates.

    Loopback is never discovered: another machine is never there, and other zenoh
    routers on this one are not ours to dial.
    """
    sources: list[Callable[[], list[str]]] = []
    if scout:
        sources.append(lambda: scouted_endpoints(scout_addr, scout_interface, timeout, exclude))
    if go2:
        sources.append(lambda: go2_endpoints(timeout))
    results: list[list[str]] = [[] for _ in sources]

    def collect(index: int) -> None:
        try:
            results[index] = [e for e in sources[index]() if not is_loopback(e)]
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
    found = [e for result in results for e in result]
    return list(dict.fromkeys([*seeds, *found]))


def probe(endpoint: str, timeout: float = DEFAULT_TIMEOUT) -> Probe | None:
    """Join ``endpoint`` as a client; the Probe if a dimos Host's router answers there."""
    from dimos.hosted.client import discover_hosts
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
        hosts = discover_hosts(rpc, timeout)
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
    results: list[Probe | None] = []
    threads = [
        threading.Thread(target=lambda e=e: results.append(probe(e, timeout)), daemon=True)
        for e in endpoints
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout * 4 + 5)
    return [p for p in results if p is not None]


def merge(probes: Iterable[Probe]) -> list[tuple[HostDescriptor, tuple[str, ...]]]:
    """One row per Host, with the probed endpoints where its own router answered."""
    hosts: dict[str, HostDescriptor] = {}
    endpoints: dict[str, list[str]] = {}
    for p in probes:
        for host in p.hosts:
            hosts.setdefault(host.host_id, host)
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
