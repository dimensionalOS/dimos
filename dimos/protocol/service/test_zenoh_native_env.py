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

"""Native modules receive the effective zenoh config as DIMOS_ZENOH_* env vars."""

from dimos.protocol.service.zenohservice import (
    ALL_INTERFACES,
    LOOPBACK_INTERFACE,
    ZenohConfig,
    native_env,
)


def test_native_env_mirrors_the_whole_config():
    config = ZenohConfig(
        mode="client",
        connect=["tcp/192.0.2.10:7447"],
        listen=[],
        scouting=False,
        scouting_interface="",
        multicast=True,
        gossip=False,
    )
    assert native_env(config) == {
        "DIMOS_ZENOH_CONNECT": "tcp/192.0.2.10:7447",
        "DIMOS_ZENOH_LISTEN": "",
        "DIMOS_ZENOH_MODE": "client",
        "DIMOS_ZENOH_MULTICAST": "true",
        "DIMOS_ZENOH_GOSSIP": "false",
        "DIMOS_ZENOH_INTERFACE": LOOPBACK_INTERFACE,
    }


def test_endpoints_join_with_commas():
    config = ZenohConfig(connect=["tcp/192.0.2.10:7447", "tcp/192.0.2.11:7447"])
    assert native_env(config)["DIMOS_ZENOH_CONNECT"] == "tcp/192.0.2.10:7447,tcp/192.0.2.11:7447"


def test_interface_is_passed_resolved():
    assert native_env(ZenohConfig(scouting=True))["DIMOS_ZENOH_INTERFACE"] == ALL_INTERFACES
    assert native_env(ZenohConfig(scouting_interface="wlan0"))["DIMOS_ZENOH_INTERFACE"] == "wlan0"
