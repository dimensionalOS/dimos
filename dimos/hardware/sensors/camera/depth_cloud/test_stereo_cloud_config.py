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

"""The stereo half of the calibration, on the Python side of the module.

The Rust side has its own tests that the angles reach the rectifying maps. These
are about the surface a blueprint actually sets: a field that silently does not
exist reads exactly like a rig that happens to be parallel.
"""

from __future__ import annotations

from pydantic import ValidationError
import pytest

from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloudConfig


def test_the_eyes_are_assumed_parallel_until_someone_measures_them():
    # Zero is the only defensible default: two monocular calibrations say
    # nothing about the relative orientation, and inventing one would bend a
    # rig that might be straight.
    config = StereoCloudConfig()
    assert config.right_roll_rad == 0.0
    assert config.right_pitch_rad == 0.0
    assert config.right_yaw_rad == 0.0
    assert config.diagonal_paths is False


def test_the_measured_r1_rotation_survives_a_round_trip():
    config = StereoCloudConfig(right_pitch_rad=0.0035, right_yaw_rad=-0.01275)
    restored = StereoCloudConfig(**config.model_dump())
    assert restored.right_pitch_rad == pytest.approx(0.0035)
    assert restored.right_yaw_rad == pytest.approx(-0.01275)


@pytest.mark.parametrize("field", ["right_roll_rad", "right_pitch_rad", "right_yaw_rad"])
def test_an_angle_far_too_large_to_be_a_residual_is_refused(field):
    # These are the difference between two calibrations of one rigid head, not a
    # pose. A tenth of a radian would be six degrees of squint; anything past
    # that is a units mistake, and it would be absorbed silently as depth.
    with pytest.raises(ValidationError):
        StereoCloudConfig(**{field: 1.5})


def test_the_shipped_denoise_chain_is_the_default():
    # The chain that a week of scoring against the lidar settled on. A module
    # started without naming one gets it, not nothing.
    assert StereoCloudConfig().denoise == "median:8+plane:16:1+fill:8"


@pytest.mark.parametrize(
    "chain",
    [
        "none",
        "",
        "median:8",
        "median:8+plane:16:1+fill:8",
        "plane:8",
        "steep:4+speckle:0.1",
        "mean",
    ],
)
def test_a_chain_the_rust_parser_accepts_is_accepted(chain):
    assert StereoCloudConfig(denoise=chain).denoise == chain


@pytest.mark.parametrize(
    ("chain", "token"),
    [
        ("median:8+nonsense:3", "nonsense"),
        ("median:eight", "eight"),
        ("plane:8:1:2", "too many"),
        ("median:8:2", "too many"),
    ],
)
def test_a_bad_chain_fails_at_blueprint_time_naming_the_token(chain, token):
    # The Rust side would log an error and run without a chain; that is a
    # robot silently publishing thorns, which is what this exists to prevent.
    with pytest.raises(ValidationError, match=token):
        StereoCloudConfig(denoise=chain)


def test_no_height_bounds_is_the_default_and_still_crosses_the_wire():
    # native_config forbids absent keys, so "no bound" must be sent as an
    # explicit null rather than omitted; base_fields is what makes that happen.
    config = StereoCloudConfig()
    assert config.min_height_m is None
    assert config.max_height_m is None
    sent = config.to_config_dict()
    assert "min_height_m" in sent and sent["min_height_m"] is None
    assert "max_height_m" in sent and sent["max_height_m"] is None


def test_a_height_gate_and_camera_pose_survive_a_round_trip():
    config = StereoCloudConfig(
        min_height_m=0.05,
        max_height_m=1.8,
        base_from_camera_xyz_m=(0.1, 0.0, 1.4),
        base_from_camera_rpy_rad=(-1.5708, 0.0, -1.5708),
    )
    restored = StereoCloudConfig(**config.model_dump())
    assert restored.min_height_m == pytest.approx(0.05)
    assert restored.max_height_m == pytest.approx(1.8)
    assert restored.base_from_camera_xyz_m == pytest.approx((0.1, 0.0, 1.4))
    assert restored.base_from_camera_rpy_rad == pytest.approx((-1.5708, 0.0, -1.5708))
    # And they reach the stdin blob as three-element lists, which serde reads
    # as [f64; 3].
    sent = config.to_config_dict()
    assert list(sent["base_from_camera_xyz_m"]) == pytest.approx([0.1, 0.0, 1.4])


def test_a_pose_needs_exactly_three_numbers():
    with pytest.raises(ValidationError):
        StereoCloudConfig(base_from_camera_xyz_m=(0.0, 0.0))
