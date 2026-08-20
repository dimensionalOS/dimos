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

"""Native modules read the session off the launch line, fully resolved."""

import json
from pathlib import Path

from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ALL_INTERFACES, ZenohConfig

# One owner for the wire shape. The Rust suite parses these same files into
# SessionSettings, so a field either side renames fails one of the two suites.
_GOLDENS = Path(__file__).parents[3] / "native" / "rust" / "dimos-module" / "tests" / "fixtures"


def _golden(name: str) -> dict[str, object]:
    return json.loads((_GOLDENS / name).read_text())


def test_wire_matches_the_client_golden(zenoh_defaults: None) -> None:
    config = ZenohConfig(
        mode="client",
        connect=["tcp/192.0.2.10:7447"],
        listen=[],
        scouting=False,
        scouting_interface="lo",
        multicast=True,
        gossip=False,
        connect_timeout=2.0,
    )
    assert config.to_wire() == _golden("session_wire_client.json")


def test_wire_matches_the_router_golden(zenoh_defaults: None) -> None:
    config = ZenohConfig(
        mode="router",
        connect=[],
        listen=["tcp/127.0.0.1:7447"],
        scouting=False,
        scouting_interface="auto",
        multicast=False,
        gossip=True,
        connect_timeout=0.0,
    )
    assert config.to_wire() == _golden("session_wire_router.json")


def test_derived_settings_are_resolved(zenoh_defaults: None) -> None:
    """The native side applies what it is given and derives nothing itself."""
    assert ZenohConfig(scouting=True).to_wire()["interface"] == ALL_INTERFACES
    assert ZenohConfig(scouting_interface="wlan0").to_wire()["interface"] == "wlan0"
    assert ZenohConfig(gossip=None, scouting=True).to_wire()["gossip"] is True


def test_lcm_sends_no_session_settings() -> None:
    """LCM natives take their settings from the environment LCM itself reads."""
    assert LCMConfig().to_wire() == {}
