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

"""The calibration file: written, read back, missing, and wrong."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloudConfig
from dimos.robot.galaxea.r1pro import stereo_calibration
from dimos.robot.galaxea.r1pro.stereo_calibration import (
    DEFAULT_PATH,
    ENV_VAR,
    R1PRO_HEAD_CALIBRATION,
    EyeIntrinsics,
    R1StereoCalibration,
    StereoCalibrationError,
    load_stereo_calibration,
    stereo_cloud_kwargs,
    write_stereo_calibration,
)


@pytest.fixture()
def isolated_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A HOME with no calibration in it and no env override."""
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv(ENV_VAR, raising=False)
    return tmp_path


def _fitted() -> R1StereoCalibration:
    return R1StereoCalibration(
        baseline_m=0.1195,
        right_roll_rad=-0.0021,
        right_pitch_rad=0.0034,
        right_yaw_rad=-0.0127,
        left=EyeIntrinsics(
            width=1920, height=1536, fx=1050.0, fy=1050.0, cx=960.0, cy=768.0, distortion=[0.0] * 5
        ),
        right=EyeIntrinsics(width=1920, height=1536, fx=1051.0, fy=1049.5, cx=961.0, cy=767.0),
        source_recording="20260919-120000-r1pro-calibration-recorder",
        fitted_at="2026-09-19T12:30:00-07:00",
        pairs_used=412,
        score={"row_residual_px": 0.31, "lidar_agreement": 0.94},
        notes="a test fit",
    )


def test_round_trip_through_the_file(tmp_path: Path) -> None:
    path = tmp_path / "nested" / "calibration.json"
    write_stereo_calibration(path, _fitted())
    assert load_stereo_calibration(path) == _fitted()
    # The on-disk name is `schema`, and the file is the pretty kind.
    text = path.read_text()
    assert json.loads(text)["schema"] == 1
    assert "\n  " in text


def test_the_committed_numbers_are_the_rig_fit() -> None:
    assert R1PRO_HEAD_CALIBRATION.baseline_m == pytest.approx(0.120195)
    assert R1PRO_HEAD_CALIBRATION.right_roll_rad == pytest.approx(-0.002)
    assert R1PRO_HEAD_CALIBRATION.right_pitch_rad == pytest.approx(0.0035)
    assert R1PRO_HEAD_CALIBRATION.right_yaw_rad == pytest.approx(-0.01275)
    assert "20260915-103437-r1pro-kronknav" in (R1PRO_HEAD_CALIBRATION.notes or "")


def test_absent_default_file_falls_back_to_the_committed_numbers_and_says_so(
    isolated_home: Path, mocker
) -> None:
    assert not (isolated_home / ".dimos" / "r1pro" / "calibration.json").exists()
    # The dimos logger does not propagate to caplog, so watch the call itself.
    info = mocker.spy(stereo_calibration.logger, "info")
    warning = mocker.spy(stereo_calibration.logger, "warning")

    assert load_stereo_calibration() is R1PRO_HEAD_CALIBRATION

    assert info.call_count == 1
    assert warning.call_count == 0
    message = info.call_args.args[0] % info.call_args.args[1:]
    assert "R1PRO_HEAD_CALIBRATION" in message
    assert str(DEFAULT_PATH.expanduser()) in message


def test_the_default_path_is_read_when_present(isolated_home: Path) -> None:
    write_stereo_calibration(isolated_home / ".dimos" / "r1pro" / "calibration.json", _fitted())
    assert load_stereo_calibration() == _fitted()


def test_env_var_names_the_file(
    isolated_home: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    elsewhere = tmp_path / "elsewhere.json"
    write_stereo_calibration(elsewhere, _fitted())
    monkeypatch.setenv(ENV_VAR, str(elsewhere))
    assert load_stereo_calibration() == _fitted()


def test_env_var_pointing_nowhere_falls_back(
    isolated_home: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(ENV_VAR, str(tmp_path / "missing.json"))
    assert load_stereo_calibration() is R1PRO_HEAD_CALIBRATION


def test_explicit_missing_path_raises_naming_it(tmp_path: Path) -> None:
    missing = tmp_path / "nope.json"
    with pytest.raises(FileNotFoundError, match=str(missing)):
        load_stereo_calibration(missing)


def test_malformed_json_names_the_file(tmp_path: Path) -> None:
    broken = tmp_path / "broken.json"
    broken.write_text("{ not json")
    with pytest.raises(StereoCalibrationError) as error:
        load_stereo_calibration(broken)
    assert str(broken) in str(error.value)
    assert "JSON" in str(error.value)


def test_out_of_range_yaw_names_the_field(tmp_path: Path) -> None:
    path = tmp_path / "yaw.json"
    payload = _fitted().model_dump(by_alias=True)
    payload["right_yaw_rad"] = 0.5
    path.write_text(json.dumps(payload))
    with pytest.raises(StereoCalibrationError) as error:
        load_stereo_calibration(path)
    assert str(path) in str(error.value)
    assert "right_yaw_rad" in str(error.value)


def test_unknown_schema_version_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "future.json"
    payload = _fitted().model_dump(by_alias=True)
    payload["schema"] = 2
    path.write_text(json.dumps(payload))
    with pytest.raises(StereoCalibrationError, match="schema"):
        load_stereo_calibration(path)


def test_unknown_keys_are_refused(tmp_path: Path) -> None:
    # A typo'd field name would otherwise be read as "use the default", which
    # for a calibration means silently the wrong number.
    path = tmp_path / "typo.json"
    payload = _fitted().model_dump(by_alias=True)
    payload["right_yaw_deg"] = 1.0
    path.write_text(json.dumps(payload))
    with pytest.raises(StereoCalibrationError, match="right_yaw_deg"):
        load_stereo_calibration(path)


def test_stereo_cloud_kwargs_are_real_config_fields() -> None:
    kwargs = stereo_cloud_kwargs(_fitted())
    assert set(kwargs) == {"baseline_m", "right_roll_rad", "right_pitch_rad", "right_yaw_rad"}
    for key in kwargs:
        assert key in StereoCloudConfig.model_fields, key
    # And the config takes them as they are.
    config = StereoCloudConfig(**kwargs)
    assert config.baseline_m == pytest.approx(0.1195)
    assert config.right_yaw_rad == pytest.approx(-0.0127)
