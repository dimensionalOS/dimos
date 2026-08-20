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

import pytest

from dimos.core.global_config import global_config


@pytest.fixture
def zenoh_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pin every zenoh-relevant global config field to its stock default."""
    monkeypatch.setattr(global_config, "transport", "zenoh")
    monkeypatch.setattr(global_config, "robot_ip", None)
    monkeypatch.setattr(global_config, "robot_ips", None)
    monkeypatch.setattr(global_config, "zenoh_mode", "peer")
    monkeypatch.setattr(global_config, "zenoh_connect", "")
    monkeypatch.setattr(global_config, "zenoh_scouting", False)
    monkeypatch.setattr(global_config, "zenoh_interface", "")
    monkeypatch.setattr(global_config, "zenoh_multicast", True)
    monkeypatch.setattr(global_config, "zenoh_gossip", None)
    monkeypatch.setattr(global_config, "zenoh_connect_timeout", 1.0)
