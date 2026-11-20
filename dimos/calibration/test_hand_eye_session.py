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

import cv2
import numpy as np
import pytest

from dimos.calibration.hand_eye_session import (
    load_session,
    open_session,
    record_sample,
    solve_session,
    validate_session,
)
from dimos.msgs.geometry_msgs.Transform import Transform


def test_records_raw_image_pose_and_resumes_without_losing_samples(tmp_path, marker_image):
    image, info, _ = marker_image
    open_session(tmp_path, info, ee_frame="arm_link6")
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6", ts=image.ts)

    entry = record_sample(tmp_path, image, pose)
    resumed = open_session(tmp_path, info, ee_frame="arm_link6")

    assert resumed["samples"] == [entry]
    assert entry["role"] == "train"
    assert entry["image_timestamp"] == image.ts
    assert resumed["camera_frame"] == image.frame_id
    np.testing.assert_array_equal(
        cv2.imread(str(tmp_path / entry["image"]), cv2.IMREAD_UNCHANGED), image.to_opencv()
    )
    np.testing.assert_array_equal(entry["base_from_ee"], np.eye(4))
    with pytest.raises(ValueError, match="settings changed"):
        open_session(tmp_path, info, ee_frame="arm_link6", marker_size_m=0.12)


def test_rejected_sample_does_not_write_an_image_or_manifest_entry(tmp_path, marker_image):
    image, info, _ = marker_image
    open_session(tmp_path, info, ee_frame="arm_link6", marker_id=1)
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6")

    with pytest.raises(ValueError, match="exactly one"):
        record_sample(tmp_path, image, pose)

    assert load_session(tmp_path)["samples"] == []
    assert list(tmp_path.glob("*.png")) == []


def test_resumes_after_an_interrupted_image_save(tmp_path, marker_image):
    image, info, _ = marker_image
    open_session(tmp_path, info, ee_frame="arm_link6")
    leftover = tmp_path / "0001.png"
    leftover.write_bytes(b"interrupted capture")
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6", ts=image.ts)

    entry = record_sample(tmp_path, image, pose)

    assert leftover.read_bytes() == b"interrupted capture"
    assert entry["image"] == "0002.png"
    assert entry["id"] == "0002"
    assert load_session(tmp_path)["samples"] == [entry]


def test_cannot_reuse_training_image_as_validation(tmp_path, marker_image):
    image, info, _ = marker_image
    open_session(tmp_path, info, ee_frame="arm_link6")
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6")
    record_sample(tmp_path, image, pose)

    with pytest.raises(ValueError, match="new, finite image timestamp"):
        record_sample(tmp_path, image, pose, validation=True)

    assert len(load_session(tmp_path)["samples"]) == 1


def test_new_validation_image_clears_old_metrics_and_can_be_validated(saved_session, marker_image):
    solve_session(saved_session)
    assert validate_session(saved_session)["validated"]
    image, _, _ = marker_image
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6")

    entry = record_sample(saved_session, image, pose, validation=True)

    pending = json.loads((saved_session / "hand_eye.json").read_text())
    assert not pending["validated"]
    assert "validation_metrics" not in pending
    assert "validation_sample_ids" not in pending
    assert entry["id"] in validate_session(saved_session)["validation_sample_ids"]


def test_new_training_image_requires_a_new_solve(saved_session, marker_image):
    solve_session(saved_session)
    assert validate_session(saved_session)["validated"]
    image, _, _ = marker_image
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6")
    entry = record_sample(saved_session, image, pose)

    with pytest.raises(ValueError, match="solve again"):
        validate_session(saved_session)

    assert not json.loads((saved_session / "hand_eye.json").read_text())["validated"]
    solve_session(saved_session)
    assert entry["id"] in validate_session(saved_session)["training_sample_ids"]


def test_result_records_frame_direction_units_and_requires_validation(saved_session, observations):
    _, camera_to_ee, _ = observations
    result = solve_session(saved_session)

    assert not result["validated"]
    assert (result["parent_frame"], result["child_frame"]) == (
        "arm_link6",
        "camera_color_optical_frame",
    )
    assert result["translation_unit"] == "metre"
    np.testing.assert_allclose(result["camera_to_ee"], camera_to_ee, atol=1e-8)

    validated = validate_session(saved_session)
    assert validated["validated"]
    assert set(validated["training_sample_ids"]).isdisjoint(validated["validation_sample_ids"])
    assert json.loads((saved_session / "hand_eye.json").read_text()) == validated
    assert not solve_session(saved_session)["validated"]


def test_incomplete_validation_clears_an_earlier_pass(saved_session):
    solve_session(saved_session)
    validate_session(saved_session)
    data = load_session(saved_session)
    data["samples"] = data["samples"][:17]
    (saved_session / "session.json").write_text(json.dumps(data))

    with pytest.raises(ValueError, match="4 validation"):
        validate_session(saved_session)

    assert not json.loads((saved_session / "hand_eye.json").read_text())["validated"]
