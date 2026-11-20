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
import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.calibration.hand_eye import HandEyeSample, MarkerObservationError
from dimos.calibration.hand_eye_run import run_hand_eye
from dimos.calibration.hand_eye_session import load_session
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform

HOME = (0.0, 1.0, -1.0, 1.1, 0.0, 0.0)
A1Z_RETURN = (0.0, 0.05, -0.05, 0.0, 0.0, 0.0)


@pytest.fixture
def home_tool_pose():
    return PoseStamped(
        frame_id="world",
        position=(0.45, -0.15, 0.35),
        orientation=Rotation.from_euler("xyz", [0.3, 0.55, -0.4]).as_quat(),
    )


@pytest.fixture
def automatic_run(tmp_path, marker_image, observations, mocker, home_tool_pose):
    """Simulate a planner moving its TCP with a camera mounted on a different link."""
    template, info, _ = marker_image
    _, camera_to_mount, base_from_target = observations
    tcp_from_mount = np.eye(4)
    tcp_from_mount[0, 3] = -0.075
    commands = []
    images = []

    def plan_joint_move(joints):
        commands.append(tuple(joints))
        return home_tool_pose

    def capture():
        tcp = commands[-1] if isinstance(commands[-1], PoseStamped) else home_tool_pose
        image = template.copy()
        image.ts += len(images) + 1
        image.data[0, 0] = len(images)
        images.append(image)
        measured = Transform.from_matrix(
            Transform.from_pose("tip", tcp).to_matrix() @ tcp_from_mount,
            ts=image.ts,
            frame_id="base",
            child_frame_id="mount",
        )
        return image, measured

    def detect(image, info, measured, marker_size, marker_id):
        return HandEyeSample(
            measured, np.linalg.inv(measured @ camera_to_mount) @ base_from_target, 0.1
        )

    mocker.patch("dimos.calibration.hand_eye_session.detect_sample", side_effect=detect)

    def run(
        review,
        *,
        home=HOME,
        joint_count=6,
        move_joints=plan_joint_move,
        **settings,
    ):
        return run_hand_eye(
            tmp_path,
            info,
            home=home,
            joint_names=[f"joint{i}" for i in range(joint_count)],
            base_frame="base",
            ee_frame="mount",
            tip_frame="tip",
            move_joints=move_joints,
            move_pose=commands.append,
            capture=capture,
            review=review,
            **settings,
        )

    return run, commands, images


@pytest.mark.parametrize(
    ("home", "training", "validation"),
    [(HOME, 8, 4), ((*HOME, 0.25), 9, 5)],
)
def test_moves_two_local_tool_rings_and_validates_reviewed_images(
    tmp_path, automatic_run, observations, mocker, home_tool_pose, home, training, validation
):
    run, commands, _ = automatic_run
    review = mocker.Mock(return_value="accept")
    _, camera_to_mount, _ = observations

    result = run(
        review,
        home=home,
        joint_count=len(home),
        count=training,
        validation_count=validation,
    )

    assert result["validated"]
    assert result["parent_frame"] == "mount"
    np.testing.assert_allclose(result["camera_to_ee"], camera_to_mount, atol=1e-8)
    assert json.loads((tmp_path / "hand_eye.json").read_text()) == result
    assert commands[0] == home
    assert commands[-1] == (0.0,) * len(home)
    rotation_at_home = Rotation.from_quat(home_tool_pose.orientation.to_list())
    for role, ring in (("train", commands[1:training]), ("validation", commands[training:-1])):
        radius = math.radians(11 if role == "train" else 7)
        count = training - 1 if role == "train" else validation
        assert len(ring) == count
        for index, target in enumerate(ring):
            assert target.frame_id == "world"
            np.testing.assert_array_equal(
                target.position.to_list(), home_tool_pose.position.to_list()
            )
            local = rotation_at_home.inv() * Rotation.from_quat(target.orientation.to_list())
            phase = math.tau * (index + (0.5 if role == "validation" else 0)) / count
            np.testing.assert_allclose(
                local.as_rotvec(),
                radius * np.array([math.cos(phase), math.sin(phase), 0]),
                atol=1e-12,
            )
    session = load_session(tmp_path)
    samples = session["samples"]
    assert [s["role"] for s in samples] == ["train"] * training + ["validation"] * validation
    for sample, call in zip(samples, review.call_args_list, strict=True):
        reviewed, _ = call.args
        assert sample["image_timestamp"] == reviewed.ts
        np.testing.assert_array_equal(
            cv2.imread(str(tmp_path / sample["image"]), cv2.IMREAD_UNCHANGED), reviewed.to_opencv()
        )


def test_retake_quit_and_resume_reuses_saved_home_and_approved_views(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, images = automatic_run
    choices = mocker.Mock(side_effect=["retry", "accept", "quit"])

    assert run(choices) is None

    home, next_goal = commands
    assert home == HOME
    approved = load_session(tmp_path)["samples"]
    assert [s["image_timestamp"] for s in approved] == [images[1].ts]
    assert [p.name for p in tmp_path.glob("*.png")] == [approved[0]["image"]]
    with pytest.raises(ValueError, match="settings changed"):
        run(choices, count=9)
    assert len(commands) == 2

    # Changing the reading the planner would return cannot shift the saved rings.
    home_tool_pose.position.x += 0.0003
    original_x = load_session(tmp_path)["home_tool_pose"]["position"][0]
    resumed_review = mocker.Mock(return_value="accept")
    result = run(resumed_review, return_joints=A1Z_RETURN)

    assert commands[2].position.x == original_x
    np.testing.assert_array_equal(
        commands[2].orientation.to_list(), next_goal.orientation.to_list()
    )
    assert result["validated"]
    assert resumed_review.call_count == 11
    assert commands[-1] == A1Z_RETURN
    samples = load_session(tmp_path)["samples"]
    assert samples[:1] == approved
    assert [s["image_timestamp"] for s in samples] == [
        images[1].ts,
        *(image.ts for image in images[3:]),
    ]
    commands.clear()
    assert run(resumed_review)["validated"]
    assert commands == []


def test_unreviewed_run_collects_and_validates_both_rings(tmp_path, automatic_run, mocker):
    run, commands, images = automatic_run
    park = mocker.Mock()

    # A home equal to the return target must still use the capture tolerance on the first move.
    result = run(None, home=A1Z_RETURN, return_joints=A1Z_RETURN, return_move=park)

    assert commands[0] == A1Z_RETURN
    assert len(commands) == len(images) == 12
    park.assert_called_once_with(A1Z_RETURN)
    assert result["validated"]
    assert json.loads((tmp_path / "hand_eye.json").read_text()) == result
    roles = [entry["role"] for entry in load_session(tmp_path)["samples"]]
    assert roles == ["train"] * 8 + ["validation"] * 4


@pytest.mark.parametrize(
    ("view", "label"),
    [(1, "Train 1/8 (home)"), (2, "Train 2/8"), (10, "Validation 2/4")],
)
def test_unreviewed_run_stops_at_first_rejected_view(
    tmp_path, automatic_run, observations, mocker, view, label
):
    run, commands, images = automatic_run
    samples, _, _ = observations
    error = MarkerObservationError("Expected exactly one tag36h11 ID 0; detected 0")
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        side_effect=[*samples[: view - 1], error],
    )

    with pytest.raises(MarkerObservationError) as caught:
        run(None)

    assert str(caught.value) == f"{label}: {error}"
    assert len(commands) == len(images) == view
    assert len(load_session(tmp_path)["samples"]) == view - 1
    assert len(list(tmp_path.glob("*.png"))) == view - 1


def test_stop_during_stationary_capture_prevents_save_and_next_move(tmp_path, automatic_run):
    run, commands, images = automatic_run

    def check_cancelled():
        if images:
            raise InterruptedError("Stop requested")

    with pytest.raises(InterruptedError, match="Stop requested"):
        run(None, check_cancelled=check_cancelled)

    assert commands == [HOME]
    assert load_session(tmp_path)["samples"] == []


def test_resume_before_first_accepted_image_uses_remeasured_home(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, _ = automatic_run
    assert run(mocker.Mock(return_value="quit")) is None
    home_tool_pose.position.x += 0.003

    assert run(mocker.Mock(side_effect=["accept", "quit"])) is None

    assert load_session(tmp_path)["home_tool_pose"]["position"] == home_tool_pose.position.to_list()
    assert commands[-1].position.x == home_tool_pose.position.x


@pytest.mark.parametrize("bad_sample", [8, 12], ids=["failed_training", "failed_validation"])
def test_returns_to_zero_when_quality_checks_fail(automatic_run, observations, bad_sample):
    run, commands, images = automatic_run
    _, _, base_from_target = observations

    def review(image, label):
        if len(images) == bad_sample:
            base_from_target[0, 3] += 0.1
        return "accept"

    result = run(review)

    assert not result["validated"]
    assert len(images) == bad_sample
    assert commands[-1] == (0.0,) * 6


def test_failed_return_reports_error_and_preserves_result(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, _ = automatic_run

    def move_joints(joints):
        commands.append(joints)
        if joints != HOME:
            raise RuntimeError("Return plan rejected")
        return home_tool_pose

    with pytest.raises(RuntimeError, match="Return plan rejected") as error:
        run(mocker.Mock(return_value="accept"), move_joints=move_joints, return_joints=A1Z_RETURN)

    assert f"return to joint positions {list(A1Z_RETURN)} rad" in str(error.value)
    assert commands[-1] == A1Z_RETURN
    assert json.loads((tmp_path / "hand_eye.json").read_text())["validated"]


@pytest.mark.parametrize(
    ("home", "joint_count", "settings"),
    [
        (HOME, 7, {}),
        ((*HOME[:-1], math.nan), 6, {}),
        (HOME, 6, {"count": 7}),
        (HOME, 6, {"validation_count": 3}),
        (HOME, 6, {"return_joints": (0.0,)}),
        (HOME, 6, {"return_joints": (*HOME[:-1], math.inf)}),
    ],
)
def test_invalid_run_settings_rejected_before_motion(
    tmp_path, automatic_run, mocker, home, joint_count, settings
):
    run, commands, _ = automatic_run

    with pytest.raises(ValueError, match="finite positions|at least"):
        run(mocker.Mock(), home=home, joint_count=joint_count, **settings)

    assert commands == []
    assert not (tmp_path / "session.json").exists()


def test_old_joint_schedule_requires_a_new_session_before_motion(tmp_path, automatic_run, mocker):
    from_session = {
        "version": 1,
        "settings": {"automatic_views": {"train": [list(HOME)], "validation": []}},
    }
    (tmp_path / "session.json").write_text(json.dumps(from_session))
    run, commands, _ = automatic_run

    with pytest.raises(ValueError, match="settings changed.*new session"):
        run(mocker.Mock())

    assert commands == []
    assert load_session(tmp_path) == from_session


def test_motion_failure_stops_before_review_or_further_motion(tmp_path, marker_image, mocker):
    _, info, _ = marker_image
    move_joints = mocker.Mock(side_effect=RuntimeError("Joint feedback timed out"))
    move_pose, capture, review = mocker.Mock(), mocker.Mock(), mocker.Mock()

    with pytest.raises(RuntimeError, match="Joint feedback timed out"):
        run_hand_eye(
            tmp_path,
            info,
            home=HOME,
            joint_names=["a", "b", "c", "d", "e", "f"],
            base_frame="base",
            ee_frame="mount",
            tip_frame="tip",
            move_joints=move_joints,
            move_pose=move_pose,
            capture=capture,
            review=review,
        )

    move_joints.assert_called_once_with(HOME)
    move_pose.assert_not_called()
    capture.assert_not_called()
    review.assert_not_called()
    assert load_session(tmp_path)["samples"] == []


def test_missing_tag_halves_tilt_once_and_saves_fresh_image(
    tmp_path, marker_image, home_tool_pose, mocker
):
    template, info, _ = marker_image
    images = [template.copy() for _ in range(5)]
    for index, image in enumerate(images):
        image.ts += index
    images[1].data.fill(255)
    images[2].data.fill(255)
    capture = mocker.Mock(
        side_effect=[
            (image, Transform(frame_id="base", child_frame_id="mount", ts=image.ts))
            for image in images
        ]
    )
    review = mocker.Mock(side_effect=["accept", "accept", "accept", "accept", "quit"])
    move_pose = mocker.Mock()

    result = run_hand_eye(
        tmp_path,
        info,
        home=HOME,
        joint_names=["a", "b", "c", "d", "e", "f"],
        base_frame="base",
        ee_frame="mount",
        tip_frame="tip",
        move_joints=mocker.Mock(return_value=home_tool_pose),
        move_pose=move_pose,
        capture=capture,
        review=review,
    )

    assert result is None
    missed, recovered, next_view = [call.args[0] for call in move_pose.call_args_list]
    assert home_tool_pose.orientation.angle_to(missed.orientation) == pytest.approx(
        math.radians(11)
    )
    assert home_tool_pose.orientation.angle_to(recovered.orientation) == pytest.approx(
        math.radians(5.5)
    )
    assert next_view.orientation != recovered.orientation
    assert [s["image_timestamp"] for s in load_session(tmp_path)["samples"]] == [
        images[0].ts,
        images[3].ts,
    ]
