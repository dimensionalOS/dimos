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
    with pytest.raises(ValueError, match="'peer' or 'client'"):
        ZenohConfig(mode="mesh")


def test_router_mode_rejected():
    """Routers are external zenohd processes. A second router-mode session on a
    host would fail binding zenoh's fixed router listen port."""
    with pytest.raises(ValueError, match="'peer' or 'client'"):
        ZenohConfig(mode="router")


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
