# Copyright 2025-2026 Dimensional Inc.
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

"""The vendor's MID360_config.json, read for Point-LIO."""

import json
from pathlib import Path

import pytest

from dimos.robot.galaxea.r1pro.vendor_lidar import (
    ENV_CONFIG_PATH,
    VENDOR_MID360_CONFIG,
    LidarNetwork,
    VendorLidarConfigError,
    read_vendor_lidar_network,
    vendor_config_path,
)

# The shape livox_ros_driver2 ships, trimmed to what matters.
VENDOR_FILE = {
    "lidar_summary_info": {"lidar_type": 8},
    "MID360": {
        "lidar_net_info": {"cmd_data_port": 56100, "point_data_port": 56300},
        "host_net_info": {
            "cmd_data_ip": "192.168.2.150",
            "cmd_data_port": 56101,
            "point_data_ip": "192.168.2.150",
        },
    },
    "lidar_configs": [{"ip": "192.168.2.100", "pcl_data_type": 1, "pattern_mode": 0}],
}


def _write(tmp_path: Path, data: object) -> Path:
    path = tmp_path / "MID360_config.json"
    path.write_text(json.dumps(data) if not isinstance(data, str) else data)
    return path


def test_reads_both_addresses_from_the_vendor_file(tmp_path: Path) -> None:
    got = read_vendor_lidar_network(_write(tmp_path, VENDOR_FILE))
    assert got == LidarNetwork(lidar_ip="192.168.2.100", host_ip="192.168.2.150")


def test_missing_file_says_where_it_looked_and_what_to_set(tmp_path: Path) -> None:
    missing = tmp_path / "nowhere" / "MID360_config.json"
    with pytest.raises(VendorLidarConfigError) as error:
        read_vendor_lidar_network(missing)
    message = str(error.value)
    assert str(missing) in message
    assert "DIMOS_POINTLIO_LIDAR_IP" in message
    assert ENV_CONFIG_PATH in message


def test_missing_key_names_the_key(tmp_path: Path) -> None:
    without_host = {"lidar_configs": [{"ip": "192.168.2.100"}], "MID360": {}}
    with pytest.raises(VendorLidarConfigError, match="MID360.host_net_info.cmd_data_ip"):
        read_vendor_lidar_network(_write(tmp_path, without_host))

    without_lidar = {"lidar_configs": [], "MID360": {"host_net_info": {"cmd_data_ip": "1.2.3.4"}}}
    with pytest.raises(VendorLidarConfigError, match=r"lidar_configs\[0\].ip"):
        read_vendor_lidar_network(_write(tmp_path, without_lidar))


def test_malformed_json_is_reported_as_such(tmp_path: Path) -> None:
    with pytest.raises(VendorLidarConfigError, match="not valid JSON"):
        read_vendor_lidar_network(_write(tmp_path, "{not json"))


def test_env_var_overrides_the_vendor_location(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(ENV_CONFIG_PATH, raising=False)
    assert vendor_config_path() == VENDOR_MID360_CONFIG
    monkeypatch.setenv(ENV_CONFIG_PATH, "/somewhere/else.json")
    assert vendor_config_path() == Path("/somewhere/else.json")
