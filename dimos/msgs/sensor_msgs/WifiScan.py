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

from __future__ import annotations

from dataclasses import dataclass

from dimos.types.timestamped import Timestamped


@dataclass(frozen=True)
class WifiAccessPoint:
    """One radio observation, located at the receiver, not at the access point.

    ``ts`` is when the signal was observed, which can precede the scan's collection
    time. Bitrates are negotiated PHY rates, not measured available throughput.
    """

    ts: float
    bssid: str
    ssid: str
    frequency_mhz: int
    signal_dbm: float
    connected: bool = False
    tx_bitrate_mbps: float | None = None
    rx_bitrate_mbps: float | None = None


@dataclass(frozen=True)
class WifiScan(Timestamped):
    ts: float
    interface: str
    access_points: list[WifiAccessPoint]
