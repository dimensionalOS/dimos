#!/usr/bin/env python3
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

"""Network scouting is optional, and start() waits for its endpoints.

A robot advertising unroutable interfaces makes zenoh retry scouted locators
forever, so scouting can be dropped to loopback. Opening a session returns
before its endpoints are dialled, so start() blocks until they link.
"""

import time

import pytest

from dimos.core.global_config import global_config
from dimos.protocol.service.zenohservice import (
    ALL_INTERFACES,
    LOOPBACK_INTERFACE,
    ZenohConfig,
    ZenohService,
    endpoint_addresses,
)


@pytest.fixture
def zenoh_transport(monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", None)
    monkeypatch.setattr(global_config, "robot_ips", None)
    monkeypatch.setattr(global_config, "transport", "zenoh")
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    monkeypatch.setattr(global_config, "zenoh_interface", "")
    monkeypatch.setattr(global_config, "zenoh_connect_timeout", 10.0)


def test_scouting_defaults_on(zenoh_transport):
    assert ZenohConfig().scouting is True


def test_scouting_follows_global_config(zenoh_transport, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_scouting", False)
    assert ZenohConfig().scouting is False


def test_scouting_separates_pooled_sessions(zenoh_transport):
    """Sessions differing only by scouting must not be shared."""
    assert ZenohConfig(scouting=True).session_key != ZenohConfig(scouting=False).session_key


def test_scouting_on_reaches_every_interface(zenoh_transport):
    assert ZenohConfig().multicast_interface == ALL_INTERFACES


def test_scouting_off_stays_on_loopback(zenoh_transport, monkeypatch):
    """Off narrows the interface rather than disabling discovery outright."""
    monkeypatch.setattr(global_config, "zenoh_scouting", False)
    assert ZenohConfig().multicast_interface == LOOPBACK_INTERFACE


def test_named_interface_overrides_scouting(zenoh_transport, monkeypatch):
    """A named link is how two robots scout each other without scouting everything."""
    monkeypatch.setattr(global_config, "zenoh_interface", "wlan0")
    assert ZenohConfig().multicast_interface == "wlan0"
    monkeypatch.setattr(global_config, "zenoh_scouting", False)
    assert ZenohConfig().multicast_interface == "wlan0"


def test_interface_separates_pooled_sessions(zenoh_transport):
    """Two interfaces are two discovery scopes, so they cannot share a session."""
    assert (
        ZenohConfig(scouting_interface="wlan0").session_key
        != ZenohConfig(scouting_interface="eth0").session_key
    )


def test_loopback_interface_is_a_real_name():
    assert LOOPBACK_INTERFACE in {"lo", "lo0"}


def test_endpoint_addresses_keeps_literal_host_and_port():
    assert "192.0.2.10:7447" in endpoint_addresses("tcp/192.0.2.10:7447")


def test_endpoint_addresses_resolves_names():
    """A link reports the resolved address, never the name that was dialled."""
    assert "127.0.0.1:7447" in endpoint_addresses("tcp/localhost:7447")


class _FakeLink:
    def __init__(self, dst: str) -> None:
        self.dst = dst


class _FakeInfo:
    def __init__(self, links: list[_FakeLink]) -> None:
        self._links = links

    def links(self) -> list[_FakeLink]:
        return self._links


class _FakeSession:
    def __init__(self, links: list[str]) -> None:
        self.info = _FakeInfo([_FakeLink(dst) for dst in links])


def _await(session: _FakeSession, **config) -> float:
    """Seconds _await_connect blocks against a session with the given links."""
    service = ZenohService(**config)
    started = time.monotonic()
    service._await_connect(session)
    return time.monotonic() - started


def test_await_returns_once_endpoint_is_linked(zenoh_transport):
    session = _FakeSession(["tcp/192.0.2.10:7447"])
    assert _await(session, connect=["tcp/192.0.2.10:7447"], connect_timeout=5.0) < 1.0


def test_await_waits_for_every_endpoint(zenoh_transport):
    """One robot linked is not enough when two were configured."""
    session = _FakeSession(["tcp/192.0.2.10:7447"])
    elapsed = _await(
        session,
        connect=["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"],
        connect_timeout=0.3,
    )
    assert elapsed >= 0.3


def test_await_gives_up_after_timeout(zenoh_transport):
    elapsed = _await(_FakeSession([]), connect=["tcp/192.0.2.199:7447"], connect_timeout=0.3)
    assert 0.3 <= elapsed < 3.0


def test_await_is_skipped_without_connect_endpoints(zenoh_transport):
    assert _await(_FakeSession([]), connect=[], connect_timeout=30.0) < 1.0


def test_zero_timeout_disables_the_wait(zenoh_transport):
    assert _await(_FakeSession([]), connect=["tcp/192.0.2.199:7447"], connect_timeout=0.0) < 1.0
