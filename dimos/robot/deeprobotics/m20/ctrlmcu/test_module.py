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
from pathlib import Path
import sys

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.core.native_module import NativeModuleConfig
from dimos.robot.deeprobotics.m20.ctrlmcu.module import (
    VENDOR_REPLY_TIMEOUT_MS,
    M20CtrlMcu,
    M20CtrlMcuConfig,
)

_RUST = Path(__file__).parent / "rust"


def _config_fields() -> set[str]:
    return set(M20CtrlMcuConfig.model_fields) - set(NativeModuleConfig.model_fields)


def _registry() -> dict:
    manifest = tomllib.loads((_RUST / "Cargo.toml").read_text())
    return manifest["package"]["metadata"]["dimos"]["module"]["m20_ctrlmcu"]


def test_config_dict_is_the_vendor_setup() -> None:
    wire = json.loads(json.dumps(M20CtrlMcuConfig().to_config_dict()))
    assert set(wire) == _config_fields()
    assert wire["device"] == "/dev/ttyS3"
    assert wire["baud"] == 115200
    assert wire["startup_commands"] == []
    assert wire["allow_power"] is False
    assert wire["exclusive_with"] == ["ctrlmcu_node"]


def test_every_command_input_has_a_reply_timeout() -> None:
    commands = set(_registry()["inputs"])
    assert set(VENDOR_REPLY_TIMEOUT_MS) == commands


def test_rust_struct_has_every_config_key() -> None:
    src = (_RUST / "src" / "module.rs").read_text()
    struct = src.split("pub struct Config {", 1)[1].split("\n}", 1)[0]
    declared = {
        line.split(":")[0].strip()
        for line in struct.splitlines()
        if ": " in line and not line.strip().startswith(("//", "#"))
    }
    assert declared == _config_fields()


def test_ports_match_registry() -> None:
    hints = M20CtrlMcu.__annotations__
    inputs = {name for name, hint in hints.items() if "In[" in str(hint)}
    outputs = {name for name, hint in hints.items() if "Out[" in str(hint)}
    assert inputs == set(_registry()["inputs"])
    assert outputs == set(_registry()["outputs"])
