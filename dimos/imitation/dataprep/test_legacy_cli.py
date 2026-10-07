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

"""Regression coverage for the retained legacy DataPrep CLI during the split."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.imitation.dataprep import cli
from dimos.imitation.dataprep.schema import (
    DataPrepConfig,
)


@pytest.mark.parametrize("format_name", [None, "hdf5", "lerobot"])
def test_legacy_inspect_preserves_automatic_and_explicit_formats(
    tmp_path: Path, format_name: str | None, mocker: MockerFixture, capsys: Any
) -> None:
    path = tmp_path / "dataset-without-extension"
    result = {"episodes": 2}
    autodetect = mocker.patch("dimos.imitation.dataprep.build.inspect_dataset", return_value=result)
    explicit = mocker.Mock(return_value=result)
    target = (
        "dimos.imitation.dataprep.lerobot.inspect_lerobot_dataset"
        if format_name == "lerobot"
        else "dimos.imitation.dataprep.formats.hdf5.reader.inspect"
    )
    mocker.patch(target, explicit)

    cli.inspect(path, format_name)

    assert json.loads(capsys.readouterr().out) == result
    if format_name is None:
        autodetect.assert_called_once_with(path)
        explicit.assert_not_called()
    else:
        explicit.assert_called_once_with(path)
        autodetect.assert_not_called()


def test_retained_example_has_an_explicit_feature_schema() -> None:
    path = Path(__file__).with_name("example_config.json")
    config = DataPrepConfig.model_validate_json(path.read_text())
    assert config.observation["joint_state"].names[-1] == "arm/gripper"
    assert config.action["joint_target"].shape == (8,)
