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

"""Network scouting is optional, and start() waits for its connect endpoints."""

import time

import pytest

from dimos.core.global_config import ZenohMode, global_config
from dimos.protocol.service.zenohservice import (
    ALL_INTERFACES,
    LOOPBACK_INTERFACE,
    LOOPBACK_LISTEN,
    ZenohConfig,
    ZenohService,
    endpoint_addresses,
)


def test_scouting_follows_global_config(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert ZenohConfig().scouting is False
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().scouting is True


def test_scouting_separates_pooled_sessions(zenoh_defaults: None) -> None:
    assert ZenohConfig(scouting=True).session_key != ZenohConfig(scouting=False).session_key


def test_scouting_on_reaches_every_interface(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().multicast_interface == ALL_INTERFACES


def test_scouting_off_stays_on_loopback(zenoh_defaults: None) -> None:
    assert ZenohConfig().multicast_interface == LOOPBACK_INTERFACE


def test_named_interface_overrides_scouting(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(global_config, "zenoh_interface", "wlan0")
    assert ZenohConfig().multicast_interface == "wlan0"
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().multicast_interface == "wlan0"


def test_interface_separates_pooled_sessions(zenoh_defaults: None) -> None:
    assert (
        ZenohConfig(scouting_interface="wlan0").session_key
        != ZenohConfig(scouting_interface="eth0").session_key
    )


def test_unscouted_session_listens_only_on_loopback(zenoh_defaults: None) -> None:
    assert ZenohConfig().listen_endpoints == [LOOPBACK_LISTEN]


def test_scouting_on_restores_the_default_listener(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A peer the LAN can scout has to be a peer the LAN can link to."""
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().listen_endpoints == []


def test_named_interface_restores_the_default_listener(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(global_config, "zenoh_interface", "wlan0")
    assert ZenohConfig().listen_endpoints == []


def test_explicit_listen_is_never_overridden(zenoh_defaults: None) -> None:
    assert ZenohConfig(listen=["tcp/0.0.0.0:7447"]).listen_endpoints == ["tcp/0.0.0.0:7447"]


def test_loopback_pin_reaches_the_wire(zenoh_defaults: None) -> None:
    """Native modules read the same pinned listener off stdin."""
    assert ZenohConfig().to_wire()["listen"] == [LOOPBACK_LISTEN]


def test_dialing_a_robot_restores_the_default_listener(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Peers behind the robot's router learn this locator via gossip and dial it."""
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    config = ZenohConfig()
    assert config.connect == ["tcp/192.0.2.10:7447"]
    assert config.listen_endpoints == []


def test_dialing_a_hub_restores_the_default_listener(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Two peers on one hub mesh via gossip, which needs a reachable listener."""
    monkeypatch.setattr(global_config, "zenoh_connect", "tcp/hub.example:7447")
    assert ZenohConfig().listen_endpoints == []


def test_pinned_and_unpinned_sessions_are_pooled_apart(
    zenoh_defaults: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    pinned = ZenohConfig().session_key
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().session_key != pinned


def test_endpoint_addresses_keeps_literal_host_and_port() -> None:
    assert "192.0.2.10:7447" in endpoint_addresses("tcp/192.0.2.10:7447")


def test_endpoint_addresses_resolves_names() -> None:
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


def _await_elapsed(
    session: _FakeSession, connect: list[str], connect_timeout: float, mode: ZenohMode = "peer"
) -> float:
    """Seconds _await_connect blocks against a session with the given links."""
    service = ZenohService(mode=mode, connect=connect, connect_timeout=connect_timeout)
    started = time.monotonic()
    service._await_connect(session)
    return time.monotonic() - started


def test_await_returns_once_endpoint_is_linked(zenoh_defaults: None) -> None:
    session = _FakeSession(["tcp/192.0.2.10:7447"])
    assert _await_elapsed(session, connect=["tcp/192.0.2.10:7447"], connect_timeout=5.0) < 1.0


def test_await_waits_for_every_endpoint(zenoh_defaults: None) -> None:
    session = _FakeSession(["tcp/192.0.2.10:7447"])
    elapsed = _await_elapsed(
        session,
        connect=["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"],
        connect_timeout=0.3,
    )
    assert elapsed >= 0.3


def test_client_mode_await_is_satisfied_by_one_link(zenoh_defaults: None) -> None:
    """A client session holds a single link, so one linked alternative is done."""
    session = _FakeSession(["tcp/192.0.2.10:7447"])
    elapsed = _await_elapsed(
        session,
        connect=["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"],
        connect_timeout=5.0,
        mode="client",
    )
    assert elapsed < 3.0


def test_duplicate_endpoints_do_not_satisfy_the_wait(zenoh_defaults: None) -> None:
    """One endpoint listed twice is still one link to wait for."""
    duplicate = "tcp/192.0.2.199:7447"
    elapsed = _await_elapsed(_FakeSession([]), connect=[duplicate, duplicate], connect_timeout=0.3)
    assert elapsed >= 0.3


def test_await_gives_up_after_timeout(zenoh_defaults: None) -> None:
    elapsed = _await_elapsed(
        _FakeSession([]), connect=["tcp/192.0.2.199:7447"], connect_timeout=0.3
    )
    assert elapsed >= 0.3


def test_await_is_skipped_without_connect_endpoints(zenoh_defaults: None) -> None:
    assert _await_elapsed(_FakeSession([]), connect=[], connect_timeout=30.0) < 3.0


def test_zero_timeout_disables_the_wait(zenoh_defaults: None) -> None:
    assert (
        _await_elapsed(_FakeSession([]), connect=["tcp/192.0.2.199:7447"], connect_timeout=0.0)
        < 1.0
    )
