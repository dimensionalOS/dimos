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

from pathlib import Path

import pytest
from structlog.testing import capture_logs

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprint_config.sources import read_config_file
from dimos.core.global_config import global_config as process_global_config
from dimos.core.module import Module, ModuleConfig


class PrimaryConfig(ModuleConfig):
    speed: float = 1.0


class PrimaryModule(Module):
    config: PrimaryConfig


def test_explicit_empty_environment_does_not_leak_process_global_mutations() -> None:
    previous_robot_ip = process_global_config.robot_ip
    process_global_config.update(robot_ip="198.51.100.7")
    try:
        parsed = BlueprintConfigParser(PrimaryModule.blueprint()).parse(environ={})
    finally:
        process_global_config.update(robot_ip=previous_robot_ip)

    assert parsed.global_config_values()["robot_ip"] is None


def test_default_environment_reads_dotenv_at_environment_precedence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    (tmp_path / ".env").write_text("ROBOT_IP=192.0.2.17\nPRIMARYMODULE__SPEED=3.5\n")
    config_path = tmp_path / "config.json"
    config_path.write_text('{"g":{"robot_ip":"config"},"primarymodule":{"speed":2}}')
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("dimos.core.coordination.blueprint_config.sources.ENV_FILE", ".env")
    monkeypatch.delenv("ROBOT_IP", raising=False)

    parsed = BlueprintConfigParser(PrimaryModule.blueprint()).parse(
        config_path=config_path,
    )

    assert parsed.global_config_values()["robot_ip"] == "192.0.2.17"
    assert parsed.module_kwargs("primarymodule")["speed"] == 3.5


def test_preparse_environment_null_coerces_to_none() -> None:
    values = BlueprintConfigParser.preparse_global_config(
        environ={"ROBOT_IP": "null", "G__RELAY_URL": "null"},
    )

    assert values["robot_ip"] is None
    assert values["relay_url"] is None


def test_config_file_errors_are_clear_but_missing_file_is_optional(tmp_path: Path) -> None:
    parser = BlueprintConfigParser(PrimaryModule.blueprint())
    parser.parse(config_path=tmp_path / "missing.json", environ={})

    invalid = tmp_path / "invalid.json"
    invalid.write_text("{")
    with pytest.raises(BlueprintConfigError, match="Invalid JSON"):
        parser.parse(config_path=invalid, environ={})

    not_an_object = tmp_path / "array.json"
    not_an_object.write_text("[]")
    with pytest.raises(BlueprintConfigError, match="must contain a JSON object"):
        parser.parse(config_path=not_an_object, environ={})


def test_config_path_that_is_a_directory_reads_as_absent(tmp_path: Path) -> None:
    assert read_config_file(tmp_path) == {}


@pytest.mark.parametrize("parse_method", ["parse", "preparse_global_config"])
@pytest.mark.parametrize(
    ("name", "replacement"),
    [
        ("UNITREE_AES_128_KEY", "GO2CONNECTION__AES_128_KEY"),
        ("XARM7_IP", "CONTROLCOORDINATOR__ADDRESSES"),
        ("XARM6_IP", "CONTROLCOORDINATOR__ADDRESSES"),
        ("LITE6_IP", "CONTROLCOORDINATOR__ADDRESSES"),
        ("CAN_PORT", "CONTROLCOORDINATOR__ADDRESSES"),
        ("DEVICE_PATH", "CONTROLCOORDINATOR__ADDRESSES"),
        ("RELAY_KEY", "RELAYBRIDGEMODULE__RELAY_KEY"),
        ("typesafe_api_key", "TYPESAFENAVIGATIONAGENT__API_KEY"),
        ("DIMOS_API_KEY", "DIMOS_CLOUD_API_KEY"),
    ],
)
def test_removed_credentials_and_device_addresses_require_migration(
    parse_method: str, name: str, replacement: str
) -> None:
    parser = BlueprintConfigParser(PrimaryModule.blueprint())
    parse = getattr(parser, parse_method)

    with pytest.raises(BlueprintConfigError, match=f"{name} -> {replacement}") as error:
        parse(environ={name: "private-value"})
    assert "private-value" not in str(error.value)


@pytest.mark.parametrize(
    ("name", "replacement"),
    [
        ("NERF_SPEED", "REPLANNINGASTARPLANNER__NERF_SPEED"),
        ("ROBOT_ID", "RELAYBRIDGEMODULE__ROBOT_ID"),
        ("DIMSIM_PORT", "GO2CONNECTION__DIMSIM__PORT"),
        ("rerun_open", "RERUNBRIDGEMODULE__RERUN_OPEN"),
    ],
)
def test_other_removed_environment_names_warn_without_blocking_runs(
    name: str, replacement: str
) -> None:
    parser = BlueprintConfigParser(PrimaryModule.blueprint())

    with capture_logs() as logs:
        parsed = parser.parse(environ={name: "old-value", "PRIMARYMODULE__SPEED": "2"})

    assert parsed.module_kwargs("primarymodule") == {"speed": 2.0}
    assert len(logs) == 1
    assert logs[0]["log_level"] == "warning"
    assert logs[0]["replacements"] == {name: replacement}
    assert "old-value" not in str(logs)


def test_preparse_warns_for_removed_noncritical_environment() -> None:
    with capture_logs() as logs:
        values = BlueprintConfigParser.preparse_global_config(
            environ={"MCP_PORT": "9992", "ROBOT_IP": "192.0.2.17"}
        )

    assert values["robot_ip"] == "192.0.2.17"
    assert len(logs) == 1
    assert logs[0]["log_level"] == "warning"
    assert logs[0]["replacements"] == {"MCP_PORT": "MCPSERVER__PORT"}


def test_unrelated_mcp_timeout_environment_is_ignored() -> None:
    parser = BlueprintConfigParser(PrimaryModule.blueprint())
    with capture_logs() as logs:
        parsed = parser.parse(environ={"MCP_TIMEOUT": "120", "PRIMARYMODULE__SPEED": "2"})
        global_values = BlueprintConfigParser.preparse_global_config(environ={"MCP_TIMEOUT": "120"})

    assert parsed.module_kwargs("primarymodule") == {"speed": 2.0}
    assert "mcp_timeout" not in global_values
    assert logs == []


@pytest.mark.parametrize("field", ["mcp_port", "obstacle_avoidance", "mujoco_room"])
def test_removed_global_fields_are_rejected_in_config_files(tmp_path: Path, field: str) -> None:
    config_path = tmp_path / "old-config.json"
    config_path.write_text('{"g":{"' + field + '":1}}')

    with pytest.raises(BlueprintConfigError, match="Unknown global configuration.*" + field):
        BlueprintConfigParser.preparse_global_config(config_path=config_path, environ={})


def test_removed_global_fields_are_rejected_in_blueprints() -> None:
    blueprint = PrimaryModule.blueprint().global_config(obstacle_avoidance=False)

    with pytest.raises(
        BlueprintConfigError, match="Unknown global configuration.*obstacle_avoidance"
    ):
        BlueprintConfigParser(blueprint).parse(environ={})
