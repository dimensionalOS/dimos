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

"""The config module's stereo calibration entry point."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.robot.galaxea.r1pro import config, stereo_calibration
from dimos.robot.galaxea.r1pro.stereo_calibration import (
    R1PRO_HEAD_CALIBRATION,
    R1StereoCalibration,
    write_stereo_calibration,
)


def test_the_names_agree_with_the_loader() -> None:
    # config.py repeats them as plain strings so nothing has to import the
    # loader to know where the file goes; they must not drift apart.
    assert config.R1PRO_STEREO_CALIBRATION_ENV == stereo_calibration.ENV_VAR
    assert Path(config.R1PRO_STEREO_CALIBRATION_DEFAULT_PATH) == stereo_calibration.DEFAULT_PATH


def test_stereo_calibration_falls_back_when_nothing_is_fitted(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv(config.R1PRO_STEREO_CALIBRATION_ENV, raising=False)
    assert config.stereo_calibration() is R1PRO_HEAD_CALIBRATION


def test_stereo_calibration_reads_the_env_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fitted = R1StereoCalibration(
        baseline_m=0.121, right_roll_rad=0.001, right_pitch_rad=-0.002, right_yaw_rad=0.003
    )
    path = write_stereo_calibration(tmp_path / "cal.json", fitted)
    monkeypatch.setenv(config.R1PRO_STEREO_CALIBRATION_ENV, str(path))
    assert config.stereo_calibration() == fitted
