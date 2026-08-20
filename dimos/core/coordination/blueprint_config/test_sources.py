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
