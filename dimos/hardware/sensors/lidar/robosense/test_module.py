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
from dimos.hardware.sensors.lidar.robosense.module import Airy, AiryConfig, _resolved_host_ip

_RUST_DIR = Path(__file__).parent / "rust"


def _wire_fields() -> set[str]:
    own = set(AiryConfig.model_fields) - set(NativeModuleConfig.model_fields)
    return own | {"frame_id"}


def test_config_dict_is_the_wire_config() -> None:
    wire = AiryConfig().to_config_dict()
    assert set(wire) == _wire_fields()
    # Nones cross as nulls, never as missing keys.
    for key in ("group", "host_ip", "lidar_ip", "recv_buffer_bytes", "mount_parent_frame_id"):
        assert wire[key] is None
    assert json.loads(json.dumps(wire)) == wire


def test_rust_struct_has_every_config_key() -> None:
    src = (_RUST_DIR / "src" / "module.rs").read_text()
    struct = src.split("pub struct Config {", 1)[1].split("\n}", 1)[0]
    declared = {
        line.split(":")[0].strip()
        for line in struct.splitlines()
        if ": " in line and not line.strip().startswith(("//", "#"))
    }
    assert declared == _wire_fields()


@pytest.mark.parametrize("field", ["decimate_points", "decimate_scans", "scan_queue", "imu_queue"])
def test_counts_must_be_positive(field: str) -> None:
    with pytest.raises(ValidationError):
        AiryConfig(**{field: 0})


def test_negative_voxel_is_rejected() -> None:
    with pytest.raises(ValidationError):
        AiryConfig(voxel=-0.1)


def test_host_ip_resolution(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "dimos.hardware.sensors.lidar.robosense.module.resolve_host_ip",
        lambda lidar_ip, host_ip, label: "10.21.33.103",
    )
    assert _resolved_host_ip(AiryConfig(host_ip="10.0.0.9", lidar_ip="10.21.33.201")) == "10.0.0.9"
    assert _resolved_host_ip(AiryConfig(lidar_ip="10.21.33.201")) == "10.21.33.103"
    assert _resolved_host_ip(AiryConfig()) is None


def test_ports_match_registry() -> None:
    def ports(cls: type) -> set[str]:
        return {
            name
            for name, hint in cls.__annotations__.items()
            if getattr(hint, "__origin__", None) is not None
            or str(hint).startswith(("Out[", "In["))
        } - {"config"}

    manifest = tomllib.loads((_RUST_DIR / "Cargo.toml").read_text())
    registry = manifest["package"]["metadata"]["dimos"]["module"]["airy"]
    assert set(registry["outputs"]) | set(registry["inputs"]) == ports(Airy)
