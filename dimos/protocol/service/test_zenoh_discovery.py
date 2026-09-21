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

"""Multicast and gossip are knobs. Gossip is on unless a caller turns it off."""

import json

import pytest
import zenoh

from dimos.core.global_config import GlobalConfig, global_config
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import _ZENOH_KEYS, ZenohConfig, ZenohSessionPool


def test_multicast_defaults_on(zenoh_defaults):
    assert ZenohConfig().multicast is True


def test_gossip_does_not_follow_scouting(zenoh_defaults, monkeypatch):
    """A peer that cannot gossip drops the data its links send it."""
    assert ZenohConfig().gossip_enabled is True
    monkeypatch.setattr(global_config, "zenoh_scouting", False)
    assert ZenohConfig().gossip_enabled is True


def test_explicit_gossip_wins(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_gossip", False)
    assert ZenohConfig().gossip_enabled is False
    monkeypatch.setattr(global_config, "zenoh_scouting", True)
    assert ZenohConfig().gossip_enabled is False


def test_caller_override_wins(zenoh_defaults):
    assert ZenohConfig(gossip=True).gossip_enabled is True


def test_global_config_turns_multicast_off(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_multicast", False)
    assert ZenohConfig().multicast is False


def test_env_vars_set_the_knobs(monkeypatch):
    monkeypatch.setenv("ZENOH_MULTICAST", "false")
    monkeypatch.setenv("ZENOH_GOSSIP", "true")
    config = GlobalConfig()
    assert config.zenoh_multicast is False
    assert config.zenoh_gossip is True


def test_unset_env_keeps_the_defaults(monkeypatch):
    monkeypatch.delenv("ZENOH_MULTICAST", raising=False)
    monkeypatch.delenv("ZENOH_GOSSIP", raising=False)
    config = GlobalConfig()
    assert config.zenoh_multicast is True
    assert config.zenoh_gossip is True


def test_multicast_separates_pooled_sessions(zenoh_defaults):
    assert ZenohConfig(multicast=True).session_key != ZenohConfig(multicast=False).session_key


def test_gossip_separates_pooled_sessions(zenoh_defaults):
    assert ZenohConfig(gossip=True).session_key != ZenohConfig(gossip=False).session_key


def test_unset_gossip_pools_with_its_resolved_value(zenoh_defaults):
    assert ZenohConfig(gossip=None).session_key == ZenohConfig(gossip=True).session_key


def test_zenoh_knows_every_key_we_write():
    """Zenoh rejects an unknown config key, so a typo in _ZENOH_KEYS fails here."""
    config = zenoh.Config()
    wire = ZenohConfig().to_wire()
    assert set(wire) == set(_ZENOH_KEYS)
    for name, value in wire.items():
        config.insert_json5(_ZENOH_KEYS[name], json.dumps(value))


@pytest.mark.parametrize("multicast", [True, False])
@pytest.mark.parametrize("gossip", [True, False])
def test_pool_inserts_the_discovery_config(zenoh_defaults, monkeypatch, multicast, gossip):
    captured = {}

    def fake_open(zconfig):
        captured["multicast"] = zconfig.get_json("scouting/multicast/enabled")
        captured["gossip"] = zconfig.get_json("scouting/gossip/enabled")
        captured["interface"] = zconfig.get_json("scouting/multicast/interface")
        captured["connect_timeout"] = zconfig.get_json("connect/timeout_ms")
        return object()

    monkeypatch.setattr(zenohservice.zenoh, "open", fake_open)
    ZenohSessionPool().acquire(ZenohConfig(multicast=multicast, gossip=gossip))

    assert captured["multicast"] == str(multicast).lower()
    assert captured["gossip"] == str(gossip).lower()
    assert captured["interface"] == f'"{zenohservice.LOOPBACK_INTERFACE}"'
    assert captured["connect_timeout"] == "1000"
