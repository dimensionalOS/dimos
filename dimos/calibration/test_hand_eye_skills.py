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
from queue import Queue
from threading import Event
from unittest.mock import MagicMock

import pytest

from dimos.agents.mcp import tool_stream
from dimos.calibration import hand_eye_skills
from dimos.calibration.hand_eye_skills import HandEyeCalibrationSkills
from dimos.protocol.rpc.spec import RPCSpec


class OfflineRPC(MagicMock, RPCSpec):
    """Construct the module without connecting to a running DimOS stack."""


@pytest.fixture
def calibration(tmp_path, mocker):
    frames = Queue()
    transport = mocker.patch.object(tool_stream, "make_transport").return_value
    transport.publish.side_effect = frames.put
    run = mocker.patch.object(hand_eye_skills, "run_live_hand_eye", autospec=True)
    skill = HandEyeCalibrationSkills(session_root=tmp_path, rpc_transport=OfflineRPC)
    try:
        yield skill, run, frames
    finally:
        skill.stop()


def reported_result(skill, frames):
    update = frames.get(timeout=5)
    assert update["method"] == tool_stream.NOTIFICATIONS_MESSAGE_METHOD
    result = update["params"]["data"]
    assert frames.get(timeout=5)["method"] == tool_stream.TOOL_STREAM_STOPPED_METHOD
    assert skill.hand_eye_calibration_status() == result
    return result


def test_mcp_start_schema_requires_home_and_mount(calibration):
    skill, _, _ = calibration
    start = next(
        tool for tool in skill.get_skills() if tool.func_name == "start_hand_eye_calibration"
    )
    assert set(json.loads(start.args_schema)["required"]) == {"home", "ee_frame"}
    assert not start.allow_reentry


@pytest.mark.parametrize(
    ("arguments", "error", "missing"),
    [
        ({"home": [0.2]}, TypeError, "ee_frame"),
        ({"ee_frame": "bracket"}, TypeError, "home"),
        ({"home": [], "ee_frame": "bracket"}, ValueError, "home"),
        ({"home": [0.2], "ee_frame": ""}, ValueError, "ee_frame"),
    ],
)
def test_missing_or_empty_home_or_mount_is_rejected_before_creating_a_session(
    calibration, tmp_path, arguments, error, missing
):
    skill, run, _ = calibration

    with pytest.raises(error, match=missing):
        skill.start_hand_eye_calibration(**arguments)

    run.assert_not_called()
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize(
    ("count", "validation_count", "invalid_role"), [(7, 4, "training"), (8, 4.5, "validation")]
)
def test_invalid_counts_are_rejected_before_creating_a_session(
    calibration, tmp_path, count, validation_count, invalid_role
):
    skill, run, frames = calibration

    with pytest.raises(ValueError, match=f"{invalid_role} view count"):
        skill.start_hand_eye_calibration(
            home=[0.1], ee_frame="wrist", count=count, validation_count=validation_count
        )

    run.assert_not_called()
    assert not list(tmp_path.iterdir())
    assert frames.get(timeout=5)["method"] == tool_stream.TOOL_STREAM_STOPPED_METHOD


def test_duplicate_start_preserves_active_run_and_explicit_retry_uses_fresh_session(calibration):
    skill, run, frames = calibration
    arrived, release = Event(), Event()
    reason = "Train 1/8 (home): Expected exactly one tag36h11 ID 0; detected 0"

    def reject_view(session, *, report, **kwargs):
        report("At home")
        arrived.set()
        assert release.wait(timeout=5)
        raise ValueError(reason)

    run.side_effect = reject_view
    try:
        started = skill.start_hand_eye_calibration(home=[0.4, -0.2], ee_frame="camera_bracket")
        assert arrived.wait(timeout=5)
        first_session = run.call_args.args[0]
        assert run.call_args.kwargs["home"] == [0.4, -0.2]
        assert run.call_args.kwargs["ee_frame"] == "camera_bracket"
        assert str(first_session) in started
        assert "At home" in skill.hand_eye_calibration_status()
        with pytest.raises(RuntimeError, match="already in progress"):
            skill.start_hand_eye_calibration(home=[0.4, -0.2], ee_frame="camera_bracket")
        run.assert_called_once()
    finally:
        release.set()
    assert reason in reported_result(skill, frames)

    skill.start_hand_eye_calibration(home=[0.1, 0.2], ee_frame="wrist")
    assert reason in reported_result(skill, frames)
    assert run.call_count == 2
    assert run.call_args.args[0] != first_session
    assert run.call_args.kwargs["home"] == [0.1, 0.2]
    assert run.call_args.kwargs["ee_frame"] == "wrist"


@pytest.mark.parametrize(
    ("validation_passed", "run_error", "grade"),
    [
        (True, "", "PASS"),
        (True, "Return move failed", "PASS"),
        (False, "", "FAIL"),
        (None, "Validation 1/4: tag missing", "Candidate awaiting validation"),
    ],
)
def test_reports_saved_grade_and_any_run_error(calibration, validation_passed, run_error, grade):
    skill, run, frames = calibration
    metrics = {
        "passed": True,
        "sample_count": 4,
        "max_translation_error_m": 0.0012,
        "max_rotation_error_deg": 0.4,
        "max_reprojection_error_px": 0.5,
    }

    def grade_session(session, **kwargs):
        saved = {
            "validated": bool(validation_passed),
            "training_metrics": {**metrics, "sample_count": 12},
        }
        if validation_passed is not None:
            saved["validation_metrics"] = {
                **metrics,
                "passed": validation_passed,
                "max_translation_error_m": 0.0012 if validation_passed else 0.015,
            }
        (session / "hand_eye.json").write_text(json.dumps(saved))
        if run_error:
            raise RuntimeError(run_error)

    run.side_effect = grade_session
    skill.start_hand_eye_calibration(home=[0.1, 0.2], ee_frame="wrist", count=12)
    result = reported_result(skill, frames)

    path = run.call_args.args[0] / "hand_eye.json"
    failure = f"Run failed in {path.parent}. {run_error} " if run_error else ""
    assert result.startswith(f"{failure}{grade}  {path}")
    assert "training (12 views): 1.200 mm, 0.400 deg, 0.500 px" in result
    assert ("validation (4 views)" in result) == (validation_passed is not None)
    assert run.call_args.kwargs["count"] == 12
    assert run.call_args.kwargs["home"] == [0.1, 0.2]
    assert run.call_args.kwargs["ee_frame"] == "wrist"


@pytest.mark.parametrize("stop_method", ["stop_hand_eye_calibration", "stop"])
def test_stop_and_shutdown_cancel_before_next_move(calibration, stop_method):
    skill, run, frames = calibration
    moving, next_move = Event(), Event()

    def move(session, *, check_cancelled, **kwargs):
        moving.set()
        # Finish the simulated move only after the stop has been delivered.
        assert skill._cancel.wait(timeout=5)
        check_cancelled()
        next_move.set()

    run.side_effect = move
    skill.start_hand_eye_calibration(home=[0.1, 0.2], ee_frame="wrist")
    assert moving.wait(timeout=5)
    getattr(skill, stop_method)()

    assert "STOPPED" in reported_result(skill, frames)
    assert not next_move.is_set()


def test_shutdown_during_blocked_move_keeps_tool_active_until_move_finishes(
    calibration, monkeypatch
):
    skill, run, frames = calibration
    moving, finish_move = Event(), Event()
    monkeypatch.setattr(hand_eye_skills, "DEFAULT_THREAD_JOIN_TIMEOUT", 0)

    def blocked_move(session, *, check_cancelled, **kwargs):
        moving.set()
        assert finish_move.wait(timeout=5)
        check_cancelled()

    run.side_effect = blocked_move
    skill.start_hand_eye_calibration(home=[0.2], ee_frame="wrist")
    try:
        assert moving.wait(timeout=5)
        with pytest.raises(RuntimeError, match="still stopping"):
            skill.stop()
        assert frames.empty()
        assert "Stop requested" in skill.hand_eye_calibration_status()
        with pytest.raises(RuntimeError, match="shutting down"):
            skill.start_hand_eye_calibration(home=[0.2], ee_frame="wrist")
        run.assert_called_once()
    finally:
        finish_move.set()

    assert "STOPPED" in reported_result(skill, frames)
