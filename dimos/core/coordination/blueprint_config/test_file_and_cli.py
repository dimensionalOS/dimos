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
from pathlib import Path

import pytest

from dimos.cli.commands.lifecycle import _hosted_config_overrides
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import Module, ModuleConfig


class MountConfig(ModuleConfig):
    mount: str = "SF"
    iface: str = "eth0"


class Mounted(Module):
    config: MountConfig


def test_explicit_values_are_what_the_config_file_and_cli_set(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("MOUNTED__MOUNT", "SF")
    parser = BlueprintConfigParser(Mounted.blueprint())
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"mounted": {"mount": "ATHENS"}, "g": {"n_workers": 2}}))

    parsed = parser.parse(["--mounted.iface", "wlan0", "--viewer=none"], config_path=config)

    # The environment and the blueprint's own pins stay out: each Host has its own.
    assert parsed.file_and_cli == {
        "mounted": {"mount": "ATHENS", "iface": "wlan0"},
        "g": {"n_workers": 2, "viewer": "none"},
    }
    assert parser.parse(environ={}).file_and_cli == {}


def test_hosted_diagnostics_session_keeps_host_local_storage():
    blueprint = Mounted.blueprint()
    parser = BlueprintConfigParser(blueprint)
    root = parser.parse(
        environ={
            "DIMOS_DIAGNOSTICS": "trace",
            "DIMOS_DIAGNOSTICS_RUN": "shared-session",
            "DIMOS_DIAGNOSTICS_DIR": "/mac/capture",
        }
    )
    forwarded = _hosted_config_overrides(root)
    assert forwarded == {"g": {"diagnostics": "trace", "diagnostics_run": "shared-session"}}
    host = parser.parse(
        overrides=forwarded,
        environ={
            "DIMOS_DIAGNOSTICS": "off",
            "DIMOS_DIAGNOSTICS_DIR": "/robot/capture",
        },
    )
    assert host.global_config["diagnostics"] == "trace"
    assert host.global_config["diagnostics_run"] == "shared-session"
    assert host.global_config["diagnostics_dir"] == "/robot/capture"


def test_hosted_diagnostics_explicit_flags_win_and_uncoordinated_env_stays_local():
    parser = BlueprintConfigParser(Mounted.blueprint())
    root = parser.parse(
        ["--diagnostics=off"],
        environ={
            "DIMOS_DIAGNOSTICS": "trace",
            "DIMOS_DIAGNOSTICS_RUN": "shared-session",
        },
    )
    assert _hosted_config_overrides(root)["g"]["diagnostics"] == "off"
    local = parser.parse(environ={"DIMOS_DIAGNOSTICS": "trace"})
    assert _hosted_config_overrides(local) == {}
