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

"""Live tuning client for a running :class:`LidarShield`.

Usage:
    # Terminal 1: run the blueprint as usual:
    #   dimos --robot-ip <ip> --no-obstacle-avoidance run unitree-go2-mls-htc ...
    #
    # Terminal 2 (from the repo root):
    #   uv run python -i -m dimos.navigation.lidar_shield.client

Available functions:
    status()                    Engaged? points in band, nearest obstacle, sensor ages
    params()                    Show the current effective config
    tune(shield_radius_m=0.5)   Live-update any config field(s), validated
    disable() / enable()        Bypass / re-arm the shield (pass-through when disabled)
"""

# mypy: disable-error-code=no-any-return
from __future__ import annotations

import json
from typing import Any

from dimos.core.rpc_client import RPCClient
from dimos.navigation.lidar_shield.module import LidarShield

_client = RPCClient.remote(LidarShield)


def status() -> dict[str, Any]:
    """Current shield state (engaged, points in band, nearest obstacle, sensor ages)."""
    return _client.get_status()


def params() -> dict[str, Any]:
    """Current effective config."""
    return _client.set_params()


def tune(**kwargs: Any) -> dict[str, Any]:
    """Live-update config fields, e.g. ``tune(shield_radius_m=0.5, allow_escape=False)``."""
    return _client.set_params(**kwargs)


def enable() -> None:
    _client.set_enabled(True)


def disable() -> None:
    """Full pass-through: commands and map flow untouched until ``enable()``."""
    _client.set_enabled(False)


if __name__ == "__main__":
    print(__doc__)
    print("Connecting to running LidarShield...")
    print(json.dumps(status(), indent=2))
