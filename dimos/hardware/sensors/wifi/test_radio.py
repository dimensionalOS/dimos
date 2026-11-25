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

import json

from dimos.hardware.sensors.wifi.radio import RadioHistory, darwin_radio, linux_radio


def test_linux_link_fallback_preserves_unavailable_retry_and_airtime_counters():
    radio = linux_radio(
        [
            {
                "command": ["iw", "dev", "wlan0", "link"],
                "returncode": 0,
                "stdout": "Connected to 11:22:33:44:55:66 (on wlan0)\n freq: 5200\n signal: -45 dBm\n TX: 2000 bytes (100 packets)\n tx bitrate: 351.0 MBit/s\n",
            }
        ],
        "wlan0",
    )
    assert radio["bssid"] == "11:22:33:44:55:66"
    assert radio["signal_dbm"] == -45
    assert radio["tx_rate_mbps"] == 351
    assert radio["counters"] == {"tx_bytes": 2000, "tx_packets": 100}
    assert radio["coverage"]["retry_counters"] is False
    assert radio["coverage"]["channel_airtime"] is False


def test_radio_intervals_measure_retries_and_busy_airtime_without_crossing_roam():
    history = RadioHistory()
    before = {
        "interface": "wlan0",
        "bssid": "a",
        "counters": {"tx_packets": 100, "tx_retries": 10},
        "survey_ms": {"active": 1000, "busy": 100},
    }
    after = {
        **before,
        "counters": {"tx_packets": 120, "tx_retries": 40},
        "survey_ms": {"active": 3000, "busy": 1100},
    }
    assert history.observe(before, 1_000_000_000)["interval_status"] == "baseline"
    measured = history.observe(after, 3_000_000_000)
    assert measured["delta"] == {"tx_packets": 20, "tx_retries": 30}
    assert measured["tx_retries_per_packet"] == 1.5
    assert measured["channel_busy_fraction"] == 0.5
    changed = history.observe({**after, "bssid": "b"}, 4_000_000_000)
    assert changed["association_changed"] is True
    assert changed["delta"] == {}
    reset = history.observe({**before, "bssid": "b"}, 5_000_000_000)
    assert reset["interval_status"] == "counter_reset"
    assert reset["delta"] == {}


def test_macos_reports_os_radio_measurements_without_inventing_bssid():
    payload = {
        "SPAirPortDataType": [
            {
                "spairport_airport_interfaces": [
                    {
                        "_name": "en0",
                        "spairport_current_network_information": {
                            "spairport_network_channel": "117 (6GHz, 160MHz)",
                            "spairport_network_rate": 576,
                            "spairport_signal_noise": "-63 dBm / -93 dBm",
                        },
                    }
                ]
            }
        ]
    }
    radio = darwin_radio(
        [
            {
                "command": ["system_profiler", "SPAirPortDataType", "-json"],
                "returncode": 0,
                "stdout": json.dumps(payload),
            }
        ],
        "en0",
    )
    assert radio["associated"] is True
    assert (radio["signal_dbm"], radio["noise_dbm"], radio["tx_rate_mbps"]) == (-63, -93, 576)
    assert radio["bssid"] is None
    assert radio["coverage"]["association_identity"] is False
