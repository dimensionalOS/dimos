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

import tomllib

from dimos.core.native_module import NativeModuleConfig
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera, RealSenseCameraConfig

_RUST_MANIFEST = Path(__file__).parent / "rust" / "Cargo.toml"


def _camera_fields() -> set[str]:
    own = set(RealSenseCameraConfig.model_fields) - set(NativeModuleConfig.model_fields)
    # The frame stem and its namespace are base fields that cross the wire too.
    return own | {"frame_id", "frame_id_prefix"}


def test_config_dict_is_the_camera_config() -> None:
    config = RealSenseCameraConfig(serial_number=None)
    wire = config.to_config_dict()
    assert set(wire) == _camera_fields()
    # Nones cross as nulls, never as missing keys.
    assert wire["serial_number"] is None
    assert wire["imu_info"]["gyro_noise_density"] == 2.0e-4
    json.dumps(wire)


def test_rust_struct_has_every_config_key() -> None:
    src = (_RUST_MANIFEST.parent / "src" / "main.rs").read_text()
    struct = src.split("struct Config {", 1)[1].split("}", 1)[0]
    declared = {line.split(":")[0].strip() for line in struct.splitlines() if ": " in line}
    assert declared == _camera_fields()


def test_ports_match_registry() -> None:
    def ports(cls: type) -> set[str]:
        return {
            name
            for name, hint in cls.__annotations__.items()
            if getattr(hint, "__origin__", None) is not None or "Out[" in str(hint)
        } - {"config"}

    manifest = tomllib.loads(_RUST_MANIFEST.read_text())
    registry = manifest["package"]["metadata"]["dimos"]["module"]["realsense"]
    # The registry files a #[tf] port under inputs, as it subscribes too.
    assert set(registry["outputs"]) | set(registry["inputs"]) == ports(RealSenseCamera)
