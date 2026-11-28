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

import asyncio
import subprocess
from unittest.mock import AsyncMock

import pytest

from dimos.hardware.sensors.wifi.iw import parse_link, parse_scan, read_scan, run_iw
from dimos.protocol.encode.cbor import decode, encode

LINK = """Connected to AA:BB:CC:DD:EE:01 (on wlp2s0)
    SSID: office network
    freq: 5500
    signal: -52 dBm
    rx bitrate: 468.0 MBit/s VHT-MCS 5 80MHz VHT-NSS 2
    tx bitrate: 526.6 MBit/s VHT-MCS 6 80MHz VHT-NSS 2
"""

SCAN = """BSS aa:bb:cc:dd:ee:01(on wlp2s0) -- associated
    freq: 5500
    signal: -70.00 dBm
    last seen: 1200000 ms ago
    SSID: office network
BSS aa:bb:cc:dd:ee:02(on wlp2s0)
    freq: 5640
    signal: -69.00 dBm
    last seen: 200 ms ago
    SSID: office network
BSS aa:bb:cc:dd:ee:03(on wlp2s0)
    freq: 2412
    signal: -85.00 dBm
    SSID:
"""


def test_link_reports_bssid_band_and_phy_rates():
    ap = parse_link(LINK, 100)
    assert ap is not None
    assert (ap.bssid, ap.ssid, ap.frequency_mhz, ap.signal_dbm) == (
        "aa:bb:cc:dd:ee:01",
        "office network",
        5500,
        -52,
    )
    assert (ap.ts, ap.connected, ap.tx_bitrate_mbps, ap.rx_bitrate_mbps) == (100, True, 526.6, 468)


def test_cache_uses_observation_time_and_rejects_old_or_unknown_age():
    aps = parse_scan(SCAN, 100, 5)
    assert [(ap.bssid, ap.ts, ap.signal_dbm) for ap in aps] == [("aa:bb:cc:dd:ee:02", 99.8, -69)]
    assert not aps[0].connected


def test_disconnect_and_incomplete_signal_do_not_invent_observations():
    assert parse_link("Not connected.\n", 100) is None
    assert parse_link(LINK.replace("signal: -52 dBm", "signal: unknown"), 100) is None


@pytest.mark.asyncio
async def test_live_link_overrides_cached_record_and_cbor_roundtrips(mocker):
    fresh = SCAN.replace("1200000 ms", "100 ms")
    mocker.patch("dimos.hardware.sensors.wifi.iw.run_iw", side_effect=[fresh, LINK])
    mocker.patch("dimos.hardware.sensors.wifi.iw.time.time", return_value=100.0)
    scan = await read_scan("wlp2s0", 5)
    assert [(ap.bssid, ap.signal_dbm) for ap in scan.access_points] == [
        ("aa:bb:cc:dd:ee:02", -69),
        ("aa:bb:cc:dd:ee:01", -52),
    ]
    assert decode(encode(scan)) == scan


@pytest.mark.asyncio
async def test_cancelled_iw_command_is_killed_and_reaped(mocker):
    process = mocker.Mock(returncode=None)
    process.communicate = AsyncMock(side_effect=asyncio.CancelledError)
    process.wait = AsyncMock()
    mocker.patch(
        "dimos.hardware.sensors.wifi.iw.asyncio.create_subprocess_exec", return_value=process
    )
    with pytest.raises(asyncio.CancelledError):
        await run_iw("wlp2s0", "link")
    process.kill.assert_called_once_with()
    process.wait.assert_awaited_once_with()


@pytest.mark.asyncio
async def test_iw_failure_is_reported_instead_of_an_empty_scan(mocker):
    process = mocker.Mock(returncode=19)
    process.communicate = AsyncMock(return_value=(b"", b"No such device"))
    mocker.patch(
        "dimos.hardware.sensors.wifi.iw.asyncio.create_subprocess_exec", return_value=process
    )
    with pytest.raises(subprocess.CalledProcessError, match="19"):
        await run_iw("missing", "link")
