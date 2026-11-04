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

"""The python config and ports agree with the rust struct and registry."""

import json
import math
from pathlib import Path
import sys

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.core.native_module import NativeModuleConfig
from dimos.robot.deeprobotics.m20.camera.module import M20CameraRelay, M20CameraRelayConfig
from dimos.robot.deeprobotics.m20.constants import FRONT_CAMERA_RTSP_URL, REAR_CAMERA_RTSP_URL

_RUST = Path(__file__).parent / "rust"


def _config_fields() -> set[str]:
    return set(M20CameraRelayConfig.model_fields) - set(NativeModuleConfig.model_fields)


def test_config_dict_is_the_camera_config() -> None:
    wire = json.loads(json.dumps(M20CameraRelayConfig().to_config_dict()))
    assert set(wire) == _config_fields()
    assert wire["front_rtsp_url"] == FRONT_CAMERA_RTSP_URL
    assert wire["rear_rtsp_url"] == REAR_CAMERA_RTSP_URL
    assert wire["rear_camera_xyz"] == [-0.37646, 0.0, 0.03738]
    assert wire["rear_camera_yaw_rad"] == math.pi


def test_rust_struct_has_every_config_key() -> None:
    src = (_RUST / "src" / "module.rs").read_text()
    struct = src.split("pub struct Config {", 1)[1].split("}", 1)[0]
    declared = {
        line.split(":")[0].strip()
        for line in struct.splitlines()
        if ": " in line and not line.strip().startswith(("//", "#"))
    }
    assert declared == _config_fields()


def test_ports_match_registry() -> None:
    ports = {
        name
        for name, hint in M20CameraRelay.__annotations__.items()
        if "Out[" in str(hint) or "In[" in str(hint)
    }
    manifest = tomllib.loads((_RUST / "Cargo.toml").read_text())
    registry = manifest["package"]["metadata"]["dimos"]["module"]["m20_camera"]
    # The registry files a #[tf] port under inputs, as it subscribes too.
    assert set(registry["outputs"]) | set(registry["inputs"]) == ports
