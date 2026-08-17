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

from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ALL_INTERFACES, LOOPBACK_INTERFACE, ZenohConfig


def test_wire_carries_every_setting(zenoh_defaults):
    config = ZenohConfig(
        mode="client",
        connect=["tcp/192.0.2.10:7447"],
        listen=[],
        scouting=False,
        scouting_interface="",
        multicast=True,
        gossip=False,
        connect_timeout=2.0,
    )
    assert config.to_wire() == {
        "mode": "client",
        "connect": ["tcp/192.0.2.10:7447"],
        "listen": [],
        "multicast": True,
        "gossip": False,
        "interface": LOOPBACK_INTERFACE,
        "connect_timeout_ms": 2000,
    }


def test_derived_settings_are_resolved(zenoh_defaults):
    """The native side applies what it is given and derives nothing itself."""
    assert ZenohConfig(scouting=True).to_wire()["interface"] == ALL_INTERFACES
    assert ZenohConfig(scouting_interface="wlan0").to_wire()["interface"] == "wlan0"
    assert ZenohConfig(gossip=None, scouting=True).to_wire()["gossip"] is True


def test_lcm_sends_no_session_settings():
    """LCM natives take their settings from the environment LCM itself reads."""
    assert LCMConfig().to_wire() == {}
