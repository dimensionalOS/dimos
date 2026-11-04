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

"""The python config and the rust struct/registry agree field for field."""

import json
from pathlib import Path
import sys

from pydantic import ValidationError
import pytest

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.core.native_module import NativeModuleConfig
from dimos.hardware.sensors.lidar.robosense.module import (
    RoboSenseLidar,
    RoboSenseLidarConfig,
    _resolved_interface,
)

_RUST_MANIFEST = Path(__file__).parent / "rust" / "Cargo.toml"
_NULLABLE = (
    "lidar_ip",
    "interface",
    "multicast_ip",
    "imu_port",
    "min_distance",
    "max_distance",
    "pcap",
    "replay_rate",
)


def _wire_fields():
    own = set(RoboSenseLidarConfig.model_fields) - set(NativeModuleConfig.model_fields)
    return own | {"frame_id"}


def test_config_dict_is_the_wire_config():
    config = RoboSenseLidarConfig(imu_port=None, replay_rate=None)
    wire = config.to_config_dict()

    assert set(wire) == _wire_fields()
    # Nones cross as nulls, never as missing keys.
    assert all(wire[key] is None for key in _NULLABLE)
    assert json.loads(json.dumps(wire)) == wire


def test_rust_struct_has_every_config_key():
    src = (_RUST_MANIFEST.parent / "src" / "module.rs").read_text()
    struct = src.split("struct Config {", 1)[1].split("}", 1)[0]
    declared = {
        line.split(":")[0].strip()
        for line in struct.splitlines()
        if ": " in line and not line.strip().startswith(("//", "#"))
    }

    assert declared == _wire_fields()


def test_ports_match_registry():
    manifest = tomllib.loads(_RUST_MANIFEST.read_text())
    registry = manifest["package"]["metadata"]["dimos"]["module"]["robosense_lidar"]

    assert set(registry["outputs"]) == {"lidar", "imu"}
    assert set(registry["outputs"]) <= set(RoboSenseLidar.__annotations__)
    assert registry["python"] == "dimos.hardware.sensors.lidar.robosense.module:RoboSenseLidar"


def test_empty_pcap_is_rejected():
    with pytest.raises(ValidationError, match="DIMOS_ROBOSENSE_PCAP"):
        RoboSenseLidarConfig(pcap="")


def test_unknown_model_is_rejected():
    with pytest.raises(ValidationError):
        RoboSenseLidarConfig(model="rs128")


def test_interface_comes_from_the_lidar_subnet(monkeypatch):
    monkeypatch.setattr(
        "dimos.hardware.sensors.lidar.robosense.module.get_local_ips",
        lambda: [("192.168.1.5", "wlan0"), ("10.21.33.103", "eth0")],
    )

    assert _resolved_interface(RoboSenseLidarConfig(lidar_ip="10.21.33.201")) == "10.21.33.103"


@pytest.mark.parametrize(
    "config",
    [
        RoboSenseLidarConfig(lidar_ip="10.21.33.201", interface="eth1"),
        RoboSenseLidarConfig(lidar_ip="10.21.33.201", pcap="capture.pcap"),
        RoboSenseLidarConfig(lidar_ip=None),
        RoboSenseLidarConfig(lidar_ip="172.16.0.9"),
    ],
    ids=["explicit", "replay", "no-lidar-ip", "no-local-subnet"],
)
def test_interface_is_left_alone(monkeypatch, config):
    monkeypatch.setattr(
        "dimos.hardware.sensors.lidar.robosense.module.get_local_ips",
        lambda: [("10.21.33.103", "eth0")],
    )

    assert _resolved_interface(config) == config.interface
