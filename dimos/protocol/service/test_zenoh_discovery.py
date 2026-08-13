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

"""Multicast and gossip are knobs, and both stay on unless a deployment says no.

Gossip is what turns one dialled endpoint into the whole mesh, so switching it
off is only ever a router-topology choice: clients link the router and nothing
else, instead of gossip-learning their way straight past it.
"""

import pytest
import zenoh

from dimos.core.global_config import GlobalConfig, global_config
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


@pytest.fixture
def clean_config(monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_multicast", True)
    monkeypatch.setattr(global_config, "zenoh_gossip", True)


@pytest.fixture
def session_pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


def test_multicast_defaults_on(clean_config):
    assert ZenohConfig().multicast is True


def test_gossip_defaults_on(clean_config):
    """The default must stay on: with it off a peer knows only what it dialled."""
    assert ZenohConfig().gossip is True


def test_global_config_turns_multicast_off(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_multicast", False)
    assert ZenohConfig().multicast is False


def test_global_config_turns_gossip_off(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_gossip", False)
    assert ZenohConfig().gossip is False


def test_caller_override_wins(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_gossip", False)
    assert ZenohConfig(gossip=True).gossip is True


def test_env_vars_set_the_knobs(monkeypatch):
    monkeypatch.setenv("ZENOH_MULTICAST", "false")
    monkeypatch.setenv("ZENOH_GOSSIP", "false")
    config = GlobalConfig()
    assert config.zenoh_multicast is False
    assert config.zenoh_gossip is False


def test_unset_env_leaves_both_on(monkeypatch):
    monkeypatch.delenv("ZENOH_MULTICAST", raising=False)
    monkeypatch.delenv("ZENOH_GOSSIP", raising=False)
    config = GlobalConfig()
    assert config.zenoh_multicast is True
    assert config.zenoh_gossip is True


def test_multicast_separates_pooled_sessions(clean_config):
    """Two discovery scopes cannot share one session."""
    assert ZenohConfig(multicast=True).session_key != ZenohConfig(multicast=False).session_key


def test_gossip_separates_pooled_sessions(clean_config):
    assert ZenohConfig(gossip=True).session_key != ZenohConfig(gossip=False).session_key


def test_zenoh_knows_both_discovery_keys():
    """The key names are the whole contract; zenoh rejects an unknown one."""
    config = zenoh.Config()
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "false")
    assert config.get_json("scouting/multicast/enabled") == "false"
    assert config.get_json("scouting/gossip/enabled") == "false"


@pytest.mark.parametrize("multicast", [True, False])
@pytest.mark.parametrize("gossip", [True, False])
def test_every_combination_opens_a_session(clean_config, session_pool, multicast, gossip):
    """Zenoh accepts both keys: a rejected one raises out of zenoh.open()."""
    config = ZenohConfig(multicast=multicast, gossip=gossip)
    assert session_pool.acquire(config) is not None
