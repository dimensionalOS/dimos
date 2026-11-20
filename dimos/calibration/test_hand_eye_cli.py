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

from itertools import count
import json
from pathlib import Path

import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler
from typer.testing import CliRunner
import yaml

from dimos.calibration import hand_eye_run
from dimos.calibration.hand_eye_session import load_session, solve_session, validate_session
from dimos.cli.commands.handeyecalibrate import handeyecalibrate_app
from dimos.manipulation.manipulation_spec import PlanningGroupInfo
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform


def test_cli_validation_fails_when_target_moves(saved_session):
    runner = CliRunner()
    assert (
        runner.invoke(handeyecalibrate_app, ["solve", "--session", str(saved_session)]).exit_code
        == 0
    )
    assert (
        runner.invoke(handeyecalibrate_app, ["validate", "--session", str(saved_session)]).exit_code
        == 0
    )
    data = load_session(saved_session)
    for sample in data["samples"]:
        if sample["role"] == "validation":
            sample["camera_from_target"][0][3] += 0.04
    (saved_session / "session.json").write_text(json.dumps(data))

    failed = runner.invoke(handeyecalibrate_app, ["validate", "--session", str(saved_session)])

    assert failed.exit_code == 1
    assert "FAIL" in failed.output
    assert not json.loads((saved_session / "hand_eye.json").read_text())["validated"]


def test_capture_cli_requires_a_mount_frame(tmp_path, mocker):
    read_info = mocker.patch("dimos.calibration.hand_eye_capture.read_camera_info")

    result = CliRunner().invoke(handeyecalibrate_app, ["capture", "--session", str(tmp_path)])

    assert result.exit_code == 2
    assert "--ee-frame" in result.output
    read_info.assert_not_called()


def test_capture_cli_retries_then_saves_a_held_out_pair(tmp_path, marker_image, topic_capture):
    image, info, _ = marker_image
    intrinsics = tmp_path / "intrinsics.yaml"
    intrinsics.write_text(
        yaml.safe_dump(
            {
                "image_width": info.width,
                "image_height": info.height,
                "distortion_model": info.distortion_model,
                "camera_matrix": {"data": info.K},
                "distortion_coefficients": {"data": info.D},
            }
        )
    )
    source = topic_capture.return_value.__enter__.return_value
    pose = Transform(frame_id="base_link", child_frame_id="arm_link6", ts=image.ts)
    source.capture.side_effect = [RuntimeError("No fresh TF"), (image, pose)]
    directory = tmp_path / "capture"

    result = CliRunner().invoke(
        handeyecalibrate_app,
        [
            "capture",
            "--session",
            str(directory),
            "--ee-frame",
            "arm_link6",
            "--intrinsics",
            str(intrinsics),
            "--count",
            "1",
            "--validation",
        ],
        input="\n\n",
    )

    assert result.exit_code == 0, result.output
    assert "Rejected: No fresh TF" in result.output
    assert "Saved 0001.png" in result.output
    assert load_session(directory)["samples"][0]["role"] == "validation"
    topic_capture.return_value.__exit__.assert_called_once()


@pytest.fixture
def topic_capture(mocker):
    return mocker.patch("dimos.calibration.hand_eye_capture.TopicCapture")


def _image_pairs(template, base_frame, ee_frame):
    for index in count():
        image = template.copy()
        image.ts += index
        yield image, Transform(frame_id=base_frame, child_frame_id=ee_frame, ts=image.ts)


@pytest.fixture
def live_color_stream(marker_image, mocker, topic_capture):
    image, info, _ = marker_image
    info.frame_id = image.frame_id
    info.D[0] = -0.05
    bus = mocker.patch("dimos.core.transport.Zenoh", autospec=True).return_value
    source = topic_capture.return_value.__enter__.return_value
    source.tip_from_mount.return_value = Transform(frame_id="tcp", child_frame_id="wrist")

    def send_info(topic, callback):
        callback(info, topic)
        return bus.subscribe.return_value

    bus.subscribe.side_effect = send_info

    def open_topics(image_topic, tf_topic, base_frame, ee_frame):
        source.capture.side_effect = _image_pairs(image, base_frame, ee_frame)
        return topic_capture.return_value

    topic_capture.side_effect = open_topics
    scheduler = ThreadPoolScheduler(max_workers=1)
    mocker.patch("dimos.utils.reactive.get_scheduler", return_value=scheduler)
    with scheduler.executor:
        yield info, bus


def test_capture_cli_saves_live_intrinsics_and_checks_them_on_resume(tmp_path, live_color_stream):
    info, bus = live_color_stream
    runner = CliRunner()
    args = ["capture", "--session", str(tmp_path), "--ee-frame", "wrist", "--count", "1"]

    result = runner.invoke(handeyecalibrate_app, args, input="\n")

    assert result.exit_code == 0, result.output
    session = load_session(tmp_path)
    assert session["settings"]["intrinsics"] == info.to_dict()
    assert session["samples"][0]["image"] == "0001.png"
    assert bus.subscribe.call_args.args[0].key_expr == "dimos/camera_info/sensor_msgs.CameraInfo"
    bus.subscribe.return_value.assert_called_once_with()
    bus.stop.assert_called_once_with()

    info.K[0] += 1.0
    resumed = runner.invoke(handeyecalibrate_app, [*args, "--validation"], input="q\n")

    assert resumed.exit_code == 1
    assert "Session settings changed" in resumed.output
    assert load_session(tmp_path) == session


def test_capture_cli_exits_when_no_camera_info_arrives(tmp_path, live_color_stream):
    _, bus = live_color_stream
    bus.subscribe.side_effect = None
    topic = "zenoh:dimos/wrist/camera_info"

    result = CliRunner().invoke(
        handeyecalibrate_app,
        [
            "capture",
            "--session",
            str(tmp_path),
            "--ee-frame",
            "wrist",
            "--camera-info-topic",
            topic,
            "--timeout-sec",
            "0.1",
        ],
    )

    assert result.exit_code == 1
    assert f"Could not read CameraInfo from {topic!r}" in result.output
    assert (
        bus.subscribe.call_args.args[0].key_expr == "dimos/wrist/camera_info/sensor_msgs.CameraInfo"
    )
    bus.subscribe.return_value.assert_called_once_with()
    bus.stop.assert_called_once_with()
    assert not (tmp_path / "session.json").exists()


def test_capture_cli_rejects_intrinsics_from_another_optical_frame(tmp_path, live_color_stream):
    info, _ = live_color_stream
    info.frame_id = "other_camera_optical"

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["capture", "--session", str(tmp_path), "--ee-frame", "wrist", "--count", "1"],
        input="\n",
    )

    assert result.exit_code == 1
    assert "CameraInfo frame 'other_camera_optical' differs from image frame" in result.output
    assert load_session(tmp_path)["samples"] == []


def test_capture_cli_reports_write_failure_without_prompting_for_another_pose(
    tmp_path, live_color_stream, mocker
):
    mocker.patch("dimos.msgs.sensor_msgs.Image.Image.save", return_value=False)

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["capture", "--session", str(tmp_path), "--ee-frame", "wrist", "--count", "1"],
        input="\n",
    )

    assert result.exit_code == 1
    assert "Could not save" in result.output
    assert "Reposition and retry" not in result.output
    assert load_session(tmp_path)["samples"] == []


@pytest.fixture
def connected_arm(mocker):
    arm = mocker.patch("dimos.calibration.hand_eye_motion.connect_calibration_arm")
    connected = arm.return_value.__enter__.return_value
    connected.info = PlanningGroupInfo(
        id="tool",
        joint_names=tuple(f"j{i}" for i in range(7)),
        base_frame="robot_base",
        tip_frame="tcp",
        has_gripper=False,
    )
    connected.return_joints = (0.0, 0.05, -0.05, 0.0, 0.0, 0.0, 0.0)
    connected.move_joints.return_value = PoseStamped(frame_id="world", position=[0.2, 0, 0.3])
    connected.capture_at_view.side_effect = lambda capture: capture()
    return connected


def test_run_cli_uses_planner_group_and_supplied_mount(
    tmp_path, live_color_stream, connected_arm, topic_capture, marker_image, mocker
):
    info, _ = live_color_stream
    run = mocker.spy(hand_eye_run, "run_hand_eye")
    preview = mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")
    rejected = mocker.patch("dimos.cli.hand_eye_preview.review_rejected")
    review = preview.return_value.__enter__.return_value
    review.side_effect = ["accept", "quit"]
    home = [0, 1, -1, 1.1, 0, 0, -0.1]

    result = CliRunner().invoke(
        handeyecalibrate_app,
        [
            "run",
            "--session",
            str(tmp_path),
            "--home",
            *map(str, home),
            "--ee-frame",
            "wrist",
            "--count",
            "10",
            "--validation-count",
            "6",
        ],
    )

    assert result.exit_code == 0, result.output
    session = load_session(tmp_path)
    settings = session["settings"]
    assert settings["intrinsics"] == info.to_dict()
    assert (settings["base_frame"], settings["ee_frame"]) == ("robot_base", "wrist")
    views = settings["automatic_views"]
    assert (len(views["train"]), len(views["validation"])) == (10, 6)
    assert views["home_joints"] == home
    assert views["tip_frame"] == "tcp"
    source = topic_capture.return_value.__enter__.return_value
    source.tip_from_mount.assert_called_once_with("tcp", marker_image[0].ts)
    assert run.call_args.kwargs["tip_from_mount"] is source.tip_from_mount.return_value
    assert run.call_args.kwargs["orbit_camera"] is None
    assert run.call_args.kwargs["review_rejected"] is rejected
    assert run.call_args.kwargs["return_joints"] == connected_arm.return_joints
    connected_arm.move_joints.assert_called_once()
    assert list(connected_arm.move_joints.call_args.args[0]) == home
    assert connected_arm.move_pose.call_count == 1
    assert [entry["image"] for entry in session["samples"]] == ["0001.png"]


def test_run_cli_resumes_an_older_tilt_schedule_without_tip_to_mount_tf(
    tmp_path, live_color_stream, connected_arm, marker_image, topic_capture, mocker
):
    info, _ = live_color_stream
    image = marker_image[0]
    home = [0] * 7
    arm = connected_arm.info
    pairs = _image_pairs(image, arm.base_frame, "wrist")
    hand_eye_run.run_hand_eye(
        tmp_path,
        info,
        home=home,
        joint_names=arm.joint_names,
        base_frame=arm.base_frame,
        ee_frame="wrist",
        tip_frame=arm.tip_frame,
        move_joints=connected_arm.move_joints,
        move_pose=connected_arm.move_pose,
        capture=lambda: next(pairs),
        review=mocker.Mock(side_effect=["accept", "quit"]),
    )
    initial = load_session(tmp_path)
    assert initial["settings"]["automatic_views"]["space"] == "tool_tilt_xy_rad"
    first_tilt = connected_arm.move_pose.call_args.args[0]
    connected_arm.reset_mock()
    source = topic_capture.return_value.__enter__.return_value
    source.tip_from_mount.side_effect = RuntimeError("No tip to mount TF")
    preview = mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")
    preview.return_value.__enter__.return_value.return_value = "quit"

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["run", "--session", str(tmp_path), "--ee-frame", "wrist", "--home", *map(str, home)],
    )

    assert result.exit_code == 0, result.output
    source.tip_from_mount.assert_not_called()
    connected_arm.move_joints.assert_not_called()
    resumed_tilt = connected_arm.move_pose.call_args.args[0]
    np.testing.assert_allclose(
        Transform.from_pose("tcp", resumed_tilt).to_matrix(),
        Transform.from_pose("tcp", first_tilt).to_matrix(),
    )
    assert load_session(tmp_path) == initial


@pytest.fixture
def orbit_command(saved_session):
    solve_session(saved_session)
    validate_session(saved_session)
    prior = saved_session / "hand_eye.json"
    cmd = [
        "run",
        "--session",
        str(saved_session / "orbit"),
        "--ee-frame",
        "arm_link6",
        "--home",
        *(["0"] * 7),
        "--orbit-calibration",
        str(prior),
    ]
    return cmd, prior


@pytest.mark.parametrize(
    ("change", "error"),
    [
        ({"validated": False}, "passed training and validation"),
        ({"child_frame": "other_camera"}, "camera_color_optical_frame"),
        ({"parent_frame": "other_mount"}, "arm_link6"),
    ],
)
def test_run_cli_rejects_unsuitable_orbit_calibration_before_motion(
    orbit_command, live_color_stream, connected_arm, change, error
):
    command, prior = orbit_command
    prior.write_text(json.dumps(json.loads(prior.read_text()) | change))

    result = CliRunner().invoke(handeyecalibrate_app, command)

    assert result.exit_code == 1
    assert error in result.output
    connected_arm.move_joints.assert_not_called()
    connected_arm.move_pose.assert_not_called()


def test_run_cli_composes_live_tcp_from_mount_with_saved_mount_from_camera(
    orbit_command,
    live_color_stream,
    connected_arm,
    topic_capture,
    marker_image,
    make_transform,
    mocker,
):
    command, prior = orbit_command
    image, _, _ = marker_image
    tip_from_mount = make_transform([0.5, -0.4, 0.1], [0.1, 0.2, -0.05])
    source = topic_capture.return_value.__enter__.return_value
    source.tip_from_mount.return_value = Transform.from_matrix(
        tip_from_mount, frame_id="tcp", child_frame_id="arm_link6"
    )
    mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")
    run = mocker.patch.object(hand_eye_run, "run_hand_eye", autospec=True, return_value=None)

    result = CliRunner().invoke(handeyecalibrate_app, command)

    assert result.exit_code == 0, result.output
    source.tip_from_mount.assert_called_once_with("tcp", image.ts)
    composed = run.call_args.kwargs["orbit_camera"]
    assert (composed.frame_id, composed.child_frame_id) == ("tcp", image.frame_id)
    np.testing.assert_allclose(
        composed.to_matrix(), tip_from_mount @ json.loads(prior.read_text())["camera_to_ee"]
    )


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("frame_id", "another_camera", "differs from image frame"),
        ("distortion_model", "unknown_model", "Unsupported color distortion model"),
    ],
)
def test_run_cli_stops_before_motion_for_unusable_camera_info(
    tmp_path, live_color_stream, connected_arm, mocker, field, value, error
):
    info, _ = live_color_stream
    setattr(info, field, value)
    mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["run", "--session", str(tmp_path), "--ee-frame", "wrist", "--home", *(["0"] * 7)],
    )

    assert result.exit_code == 1, result.output
    assert error in result.output
    connected_arm.move_joints.assert_not_called()
    assert not (tmp_path / "session.json").exists()


@pytest.mark.parametrize(
    ("option", "field", "expected"),
    [
        ("--orbit-axis", "orbit_axis", "-z"),
        ("--orbit-calibration", "orbit_calibration", Path("prior/hand_eye.json")),
    ],
)
def test_run_cli_passes_orbit_option_to_live_workflow(tmp_path, mocker, option, field, expected):
    run = mocker.patch.object(hand_eye_run, "run_live_hand_eye", autospec=True, return_value=None)

    result = CliRunner().invoke(
        handeyecalibrate_app,
        [
            "run",
            "--session",
            str(tmp_path),
            "--home",
            "0",
            "0.1",
            option,
            str(expected),
            "--ee-frame",
            "wrist",
        ],
    )

    assert result.exit_code == 0, result.output
    assert run.call_args.kwargs[field] == expected


@pytest.mark.parametrize(
    ("options", "error"),
    [
        (["--orbit-axis", "xy"], "axis"),
        (["--orbit-axis", "x", "--orbit-calibration", "prior/hand_eye.json"], "orbit"),
    ],
)
def test_run_cli_rejects_invalid_orbit_options_before_connecting(tmp_path, mocker, options, error):
    read_info = mocker.patch("dimos.calibration.hand_eye_capture.read_camera_info")

    result = CliRunner().invoke(
        handeyecalibrate_app,
        [
            "run",
            "--session",
            str(tmp_path),
            "--home",
            "0",
            "--ee-frame",
            "wrist",
            *options,
        ],
    )

    assert result.exit_code == 1
    assert error in result.output.lower()
    read_info.assert_not_called()


def test_run_cli_aborts_if_arm_feedback_stops_during_capture(
    tmp_path, live_color_stream, connected_arm, mocker
):
    connected_arm.capture_at_view.side_effect = RuntimeError("No fresh arm feedback.")
    preview = mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["run", "--session", str(tmp_path), "--ee-frame", "wrist", "--home", *(["0"] * 7)],
    )

    assert result.exit_code == 1
    assert "No fresh arm feedback" in result.output
    connected_arm.move_joints.assert_called_once_with((0.0,) * 7)
    preview.return_value.__enter__.return_value.assert_not_called()
    assert load_session(tmp_path)["samples"] == []


def test_run_cli_rejects_wrong_home_length_before_moving(
    tmp_path, live_color_stream, connected_arm, mocker
):
    mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")

    result = CliRunner().invoke(
        handeyecalibrate_app,
        ["run", "--session", str(tmp_path), "--home=0", "1", "-1", "--ee-frame", "wrist"],
    )

    assert result.exit_code == 1, result.output
    assert "7" in result.output
    connected_arm.move_joints.assert_not_called()
    connected_arm.move_pose.assert_not_called()
    assert not (tmp_path / "session.json").exists()
