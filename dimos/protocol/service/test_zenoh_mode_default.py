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

"""Session mode comes from global config, so a robot can be put behind a router.

Every robot-local session in ``client`` mode routes through one zenohd instead
of gossip-meshing straight to the laptop, which is what keeps a heavy stream to
a single copy over the wifi link.
"""

import pytest

from dimos.core.global_config import GlobalConfig, global_config
from dimos.protocol.service.zenohservice import ZenohConfig


@pytest.fixture
def clean_config(monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "peer")


def test_mode_defaults_to_peer(clean_config):
    assert ZenohConfig().mode == "peer"


def test_global_config_sets_mode(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "client")
    assert ZenohConfig().mode == "client"


def test_caller_override_wins(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "client")
    assert ZenohConfig(mode="peer").mode == "peer"


def test_env_var_sets_mode(monkeypatch):
    monkeypatch.setenv("ZENOH_MODE", "client")
    assert GlobalConfig().zenoh_mode == "client"


def test_unknown_mode_rejected(monkeypatch):
    with pytest.raises(ValueError, match="'peer', 'client' or 'router'"):
        ZenohConfig(mode="mesh")


def test_router_is_a_valid_mode(clean_config, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "router")
    assert ZenohConfig().mode == "router"


def test_mode_separates_pooled_sessions(clean_config):
    assert ZenohConfig(mode="client").session_key != ZenohConfig(mode="peer").session_key
