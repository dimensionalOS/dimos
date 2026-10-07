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

import socket

import pytest

from dimos.hosted import discovery
from dimos.hosted.daemon import HostDescriptor, free_listen
from dimos.hosted.discovery import Probe, candidates, endpoints_to_link, is_loopback, merge


def _host(host_id: str, router: str) -> HostDescriptor:
    return HostDescriptor(host_id, "e", host_id, frozenset(), {}, "available", (), router)


def test_candidates_keep_seeds_and_drop_discovered_loopback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        discovery, "scouted_endpoints", lambda *_: ["tcp/127.0.0.1:7447", "tcp/10.0.0.2:7447"]
    )
    monkeypatch.setattr(discovery, "go2_endpoints", lambda *_: ["tcp/10.0.0.2:7447"])
    assert candidates(["tcp/127.0.0.1:7448"]) == ["tcp/127.0.0.1:7448", "tcp/10.0.0.2:7447"]


def test_candidates_survive_a_failing_source(monkeypatch: pytest.MonkeyPatch) -> None:
    def broken(*_: object) -> list[str]:
        raise OSError("no multicast")

    monkeypatch.setattr(discovery, "scouted_endpoints", broken)
    monkeypatch.setattr(discovery, "go2_endpoints", lambda *_: ["tcp/10.0.0.9:7447"])
    assert candidates() == ["tcp/10.0.0.9:7447"]


def test_is_loopback() -> None:
    assert is_loopback("tcp/127.0.0.1:7447") and is_loopback("tcp/[::1]:7447")
    assert not is_loopback("tcp/10.55.1.4:7447")


def test_merge_dedups_hosts_seen_through_several_routers() -> None:
    a, b = _host("a", "za"), _host("b", "zb")
    probes = [
        Probe("tcp/10.0.0.1:7447", "za", (a, b)),
        Probe("tcp/10.0.0.2:7447", "zb", (a, b)),
        Probe("tcp/192.168.1.2:7447", "zb", (b, a)),
    ]
    assert merge(probes) == [
        (a, ("tcp/10.0.0.1:7447",)),
        (b, ("tcp/10.0.0.2:7447", "tcp/192.168.1.2:7447")),
    ]


def test_link_one_endpoint_per_router_skipping_self_linked_and_dialed() -> None:
    probes = [
        Probe("tcp/10.0.0.1:7447", "self", ()),
        Probe("tcp/10.0.0.2:7447", "linked", ()),
        Probe("tcp/10.0.0.3:7447", "dialed", ()),
        Probe("tcp/10.0.0.4:7447", "new", ()),
        Probe("tcp/192.168.1.4:7447", "new", ()),
    ]
    assert endpoints_to_link(
        probes, own_zid="self", linked=["linked"], connect=["tcp/10.0.0.3:7447"]
    ) == ["tcp/10.0.0.4:7447"]


def test_free_listen_falls_back_past_a_taken_port() -> None:
    with socket.socket() as holder:
        holder.bind(("127.0.0.1", 0))
        holder.listen()
        port = holder.getsockname()[1]
        endpoint = free_listen(f"tcp/127.0.0.1:{port}")
    assert endpoint != f"tcp/127.0.0.1:{port}"
    assert int(endpoint.rpartition(":")[2]) > port
