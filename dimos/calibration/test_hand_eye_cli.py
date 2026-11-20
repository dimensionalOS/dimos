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

import pytest
from reactivex.scheduler import ThreadPoolScheduler
from typer.testing import CliRunner
import yaml

from dimos.calibration import hand_eye_run
from dimos.calibration.hand_eye_session import load_session
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


def test_capture_cli_retries_then_saves_a_held_out_pair(tmp_path, marker_image, mocker):
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
    topic_capture = mocker.patch("dimos.calibration.hand_eye_capture.TopicCapture")
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
def live_color_stream(marker_image, mocker):
    image, info, _ = marker_image
    info.frame_id = image.frame_id
    info.D[0] = -0.05
    bus = mocker.patch("dimos.core.transport.Zenoh", autospec=True).return_value

    def send_info(topic, callback):
        callback(info, topic)
        return bus.subscribe.return_value

    bus.subscribe.side_effect = send_info
    capture = mocker.patch("dimos.calibration.hand_eye_capture.TopicCapture")

    def open_topics(image_topic, tf_topic, base_frame, ee_frame):
        pose = Transform(frame_id=base_frame, child_frame_id=ee_frame, ts=image.ts)
        capture.return_value.__enter__.return_value.capture.return_value = (image, pose)
        return capture.return_value

    capture.side_effect = open_topics
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
    tmp_path, live_color_stream, connected_arm, mocker
):
    info, _ = live_color_stream
    run = mocker.spy(hand_eye_run, "run_hand_eye")
    preview = mocker.patch("dimos.cli.hand_eye_preview.hand_eye_preview")
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
    assert run.call_args.kwargs["return_joints"] == connected_arm.return_joints
    connected_arm.move_joints.assert_called_once()
    assert list(connected_arm.move_joints.call_args.args[0]) == home
    assert connected_arm.move_pose.call_count == 1
    assert [entry["image"] for entry in session["samples"]] == ["0001.png"]


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
