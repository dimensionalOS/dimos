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

"""Zenoh session settings shared with native and standalone services.

No GlobalConfig, coordinator, typed messages or codec imports belong here.
"""

from collections.abc import Mapping
import json
import platform
from typing import Any

import zenoh

LOOPBACK_INTERFACE = "lo0" if platform.system() == "Darwin" else "lo"
ALL_INTERFACES = "auto"
LOOPBACK_LISTEN = "tcp/127.0.0.1:0"

ZENOH_KEYS = {
    "mode": "mode",
    "connect": "connect/endpoints",
    "listen": "listen/endpoints",
    "multicast": "scouting/multicast/enabled",
    "scout_addr": "scouting/multicast/address",
    "interface": "scouting/multicast/interface",
    "gossip": "scouting/gossip/enabled",
    "connect_timeout_ms": "connect/timeout_ms",
}
_DEFAULTED_WHEN_EMPTY = ("connect", "listen", "scout_addr", "connect_timeout_ms")


def make_zenoh_config(settings: Mapping[str, Any]) -> zenoh.Config:
    """Translate the same wire settings accepted by native DimOS modules."""
    config = zenoh.Config()
    for name, value in settings.items():
        if name not in ZENOH_KEYS:
            raise ValueError(f"unknown Zenoh setting: {name}")
        if not value and name in _DEFAULTED_WHEN_EMPTY:
            continue
        config.insert_json5(ZENOH_KEYS[name], json.dumps(value))
    # Shared memory silently loses >3 kB payloads between some sessions.
    config.insert_json5("transport/shared_memory/enabled", "false")
    return config
