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

"""Session mode comes from global config, so a robot can be put behind a router."""

from pydantic import ValidationError
import pytest
import zenoh

from dimos.core.global_config import GlobalConfig, global_config
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohService


def test_mode_defaults_to_peer(zenoh_defaults):
    assert ZenohConfig().mode == "peer"


def test_global_config_sets_mode(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "client")
    assert ZenohConfig().mode == "client"


def test_caller_override_wins(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_mode", "client")
    assert ZenohConfig(mode="peer").mode == "peer"


def test_env_var_sets_mode(monkeypatch):
    monkeypatch.setenv("ZENOH_MODE", "client")
    assert GlobalConfig().zenoh_mode == "client"


def test_unknown_mode_rejected():
    with pytest.raises(ValueError, match="'peer', 'client' or 'router'"):
        ZenohConfig(mode="mesh")


def test_a_session_can_be_opened_as_a_router(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    config = ZenohConfig(mode="router", listen=["tcp/127.0.0.1:17450"], connect=[])
    assert config.mode == "router"
    assert config.listen == ["tcp/127.0.0.1:17450"]
    assert config.connect == []


def test_router_without_a_listen_endpoint_rejected(zenoh_defaults):
    """A bare router falls back to 7447, the robot bridge's own port."""
    with pytest.raises(ValueError, match="needs an explicit listen endpoint"):
        ZenohConfig(mode="router")


def test_router_is_not_a_whole_process_mode(monkeypatch):
    """Every process would bind the same port, and the second one would fail."""
    monkeypatch.setenv("ZENOH_MODE", "router")
    with pytest.raises(ValidationError, match="'peer' or 'client'"):
        GlobalConfig()


def test_listen_is_never_derived_from_global_config(zenoh_defaults, monkeypatch):
    """A listen port belongs to one session, so nothing hands it to all of them."""
    monkeypatch.setattr(global_config, "zenoh_connect", "tcp/127.0.0.1:17450")
    assert ZenohConfig().listen == []


def test_rebased_rederives_only_the_fields_the_caller_left_unset(zenoh_defaults, monkeypatch):
    """Unset fields must follow the global config values in force at spawn time."""
    pinned = ZenohConfig(mode="client")
    assert pinned.connect == []

    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    rebased = pinned.rebased()
    assert rebased.mode == "client"
    assert rebased.connect == ["tcp/192.0.2.10:7447"]


def test_rebased_keeps_an_explicit_empty_list(zenoh_defaults, monkeypatch):
    pinned = ZenohConfig(mode="router", listen=["tcp/127.0.0.1:17450"], connect=[])
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    assert pinned.rebased().connect == []


class _RaisingPool:
    def acquire(self, config: ZenohConfig) -> zenoh.Session:
        raise zenoh.ZError("Unable to connect to any of [tcp/192.0.2.1:7447]")


def test_client_mode_open_failure_names_the_router(zenoh_defaults):
    service = ZenohService(
        session_pool=_RaisingPool(), mode="client", connect=["tcp/192.0.2.1:7447"]
    )
    with pytest.raises(RuntimeError, match="needs a reachable router"):
        service.start()


def test_peer_mode_open_failure_passes_through(zenoh_defaults):
    service = ZenohService(session_pool=_RaisingPool(), mode="peer", connect=[])
    with pytest.raises(zenoh.ZError):
        service.start()
