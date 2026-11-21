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

"""Geometric calibration checks independent of camera and robot hardware."""

import numpy as np
import pytest

from dimos.calibration.hand_eye import (
    HandEyeSample,
    checked_transform,
    detect_sample,
    solve_hand_eye,
    validate_hand_eye,
)


def test_recovers_camera_to_ee_and_validates_unseen_poses(observations):
    samples, expected_camera, expected_target = observations
    result = solve_hand_eye(samples[:15])

    np.testing.assert_allclose(result["camera_to_ee"], expected_camera, atol=1e-8)
    np.testing.assert_allclose(result["base_from_target"], expected_target, atol=1e-8)
    report = validate_hand_eye(
        samples[15:], np.asarray(result["camera_to_ee"]), np.asarray(result["base_from_target"])
    )
    assert report["passed"]
    assert report["sample_count"] == 5
    assert report["max_translation_error_m"] < 1e-8
    assert report["max_rotation_error_deg"] < 1e-6


def test_noisy_training_and_holdout_still_pass(observations, make_transform):
    samples, expected_camera, _ = observations
    rng = np.random.default_rng(23)
    noisy = [
        HandEyeSample(
            sample.base_from_ee,
            sample.camera_from_target
            @ make_transform(rng.normal(0, 0.0005, 3), rng.normal(0, 0.0003, 3)),
            0.6,
        )
        for sample in samples
    ]
    result = solve_hand_eye(noisy[:15])
    np.testing.assert_allclose(
        np.asarray(result["camera_to_ee"])[:3, 3], expected_camera[:3, 3], atol=0.003
    )
    assert validate_hand_eye(
        noisy[15:], np.asarray(result["camera_to_ee"]), np.asarray(result["base_from_target"])
    )["passed"]


@pytest.mark.parametrize("fault", ["wrong_pair", "moved_target", "wrong_transform", "reprojection"])
def test_validation_rejects_bad_observations_without_refitting(observations, fault):
    samples, camera_to_ee, reference = observations
    holdout = samples[15:]
    if fault == "wrong_pair":
        holdout = [
            HandEyeSample(sample.base_from_ee, holdout[(i + 1) % 5].camera_from_target, 0.3)
            for i, sample in enumerate(holdout)
        ]
    elif fault == "moved_target":
        moved = reference.copy()
        moved[0, 3] += 0.04
        holdout = [
            HandEyeSample(
                sample.base_from_ee, np.linalg.inv(sample.base_from_ee @ camera_to_ee) @ moved, 0.3
            )
            for sample in holdout
        ]
    elif fault == "wrong_transform":
        camera_to_ee = np.linalg.inv(camera_to_ee)
    else:
        holdout = [HandEyeSample(s.base_from_ee, s.camera_from_target, 3.0) for s in holdout]
    assert not validate_hand_eye(holdout, camera_to_ee, reference)["passed"]


@pytest.mark.parametrize("rotating", [False, True])
def test_rejects_translation_only_and_single_rotation_axis(rotating, make_transform):
    samples = []
    for index in range(10):
        robot = make_transform([0, 0, 0.1 * index if rotating else 0], [index * 0.01, 0, 0])
        samples.append(HandEyeSample(robot, np.linalg.inv(robot), 0.0))
    with pytest.raises(ValueError, match="two axes"):
        solve_hand_eye(samples)


def test_reports_worst_target_errors_in_metres_and_degrees(observations, make_transform):
    samples, camera_to_ee, reference = observations
    holdout = samples[15:]
    tilted = holdout[-1].camera_from_target @ make_transform([0, 0, np.deg2rad(3)], [0.004, 0, 0])
    holdout[-1] = HandEyeSample(holdout[-1].base_from_ee, tilted, 0.3)

    report = validate_hand_eye(holdout, camera_to_ee, reference)

    assert report["max_rotation_error_deg"] == pytest.approx(3.0)
    assert report["max_translation_error_m"] == pytest.approx(0.004)
    assert not report["passed"]


def test_minimum_counts_and_invalid_transforms(observations):
    samples, camera, reference = observations
    with pytest.raises(ValueError, match="8 training"):
        solve_hand_eye(samples[:7])
    with pytest.raises(ValueError, match="4 validation"):
        validate_hand_eye(samples[:3], camera, reference)
    with pytest.raises(ValueError, match="finite 4x4"):
        checked_transform(np.full((4, 4), np.nan))
    reflection = np.eye(4)
    reflection[0, 0] = -1
    with pytest.raises(ValueError, match="proper rotation"):
        checked_transform(reflection)


def test_reuses_marker_detector_and_pose_estimation(marker_image):
    image, info, expected = marker_image
    sample = detect_sample(image, info, np.eye(4))
    np.testing.assert_allclose(sample.camera_from_target[:3, 3], expected[:3, 3], atol=0.005)
    assert sample.reprojection_error_px < 2.0
    np.testing.assert_allclose(sample.base_from_ee, np.eye(4))


def test_low_contrast_marker_keeps_its_pose_and_duplicate_ids_are_rejected(marker_image):
    image, info, expected = marker_image
    image.data[:] = 120 + image.data // 64
    sample = detect_sample(image, info, np.eye(4))
    np.testing.assert_allclose(sample.camera_from_target[:3, 3], expected[:3, 3], atol=0.005)

    image.data[:] = np.minimum(image.data, np.roll(image.data, 200, axis=1))
    with pytest.raises(ValueError, match="detected 2"):
        detect_sample(image, info, np.eye(4))


def test_rejects_missing_marker_wrong_resolution_and_distortion(marker_image):
    image, info, _ = marker_image
    with pytest.raises(ValueError, match="exactly one"):
        detect_sample(image, info, np.eye(4), marker_id=1)
    info.width = 1280
    with pytest.raises(ValueError, match="resolution"):
        detect_sample(image, info, np.eye(4))
    info.width = 640
    info.distortion_model = "inverse_brown_conrady"
    with pytest.raises(ValueError, match="plumb_bob"):
        detect_sample(image, info, np.eye(4))
