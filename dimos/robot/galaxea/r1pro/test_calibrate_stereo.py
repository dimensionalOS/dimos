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

"""The CLI on synthetic recordings: every way it refuses, and the one way it fits.

Each recording is a few seconds of a made-up scene written straight into the
tables ``RawRecording`` reads, so nothing here needs the recorder, the robot or
the Rust matcher. The matcher is replaced by a function that returns a floor
sitting off the lidar's by the rotation error, which is enough for the whole
pipeline -- census, floor, sweep, file -- to run end to end.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.galaxea.r1pro import calibrate_stereo, stereo_fit
from dimos.robot.galaxea.r1pro.calibrate_stereo import (
    BUILD_COMMAND,
    EXIT_BARELY_MOVED,
    EXIT_MATCHER_FAILED,
    EXIT_MISSING_STREAMS,
    EXIT_NO_FLOOR,
    EXIT_NO_INTRINSICS,
    EXIT_NO_MATCHER,
    EXIT_NO_RECORDING,
    EXIT_TOO_FEW_PAIRS,
    EXIT_TOO_SHORT,
    RECORDER_BLUEPRINT,
    main,
)
from dimos.robot.galaxea.r1pro.stereo_calibration import parse_stereo_calibration

# The scene, in base_link: x forward, z up, floor at z = 0. The lidar sits a
# metre up; the head camera 1.5 m up and 0.2 m forward, looking along +x with
# the optical convention (+z forward, +y down).
LIDAR_HEIGHT_M = 1.0
CAMERA_HEIGHT_M = 1.5
CAMERA_FORWARD_M = 0.2
LIDAR_FRAME = "lidar_pointlio_link"
CAMERA_FRAME = "camera_head_left_link"

TRUE = {"right_roll_rad": -0.0025, "right_pitch_rad": 0.00375, "right_yaw_rad": -0.0125}


@dataclass(frozen=True)
class Scene:
    """What to put in a synthetic recording; the defaults make a fittable one."""

    duration_s: float = 6.0
    travel_m: float = 3.0
    headings: int = 4
    floor: bool = True
    omit: tuple[str, ...] = ()
    empty: tuple[str, ...] = ()
    camera_edge: bool = True
    right_offset_s: float = 0.0
    lidar_offset_s: float = 0.0
    intrinsics: bool = True


def _lidar_sweep(scene: Scene) -> np.ndarray:
    """Floor 0.5-7.5 m ahead and a wall at 8 m, in the lidar's frame."""
    parts = []
    if scene.floor:
        x, y = np.meshgrid(np.arange(0.5, 7.5, 0.1), np.arange(-3.0, 3.0, 0.1))
        parts.append(np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -LIDAR_HEIGHT_M)]))
    y, z = np.meshgrid(np.arange(-3.0, 3.0, 0.1), np.arange(0.0, 2.5, 0.1))
    parts.append(np.column_stack([np.full(y.size, 8.0), y.ravel(), z.ravel() - LIDAR_HEIGHT_M]))
    return np.vstack(parts).astype(np.float32)


# Recordings carry wall-clock stamps; the synthetic ones start here.
BASE_TS = 1_700_000_000.0


def _pose_at(scene: Scene, t: float) -> tuple[Vector3, Quaternion]:
    fraction = (t - BASE_TS) / scene.duration_s
    yaw = math.floor(min(fraction, 0.999) * scene.headings) * (2 * math.pi / scene.headings)
    return Vector3(scene.travel_m * fraction, 0.0, 0.0), Quaternion.from_euler(Vector3(0, 0, yaw))


def _tf_at(scene: Scene, t: float) -> TFMessage:
    position, orientation = _pose_at(scene, t)
    transforms = [
        Transform(
            translation=position,
            rotation=orientation,
            frame_id="odom",
            child_frame_id=LIDAR_FRAME,
            ts=t,
        ),
        Transform(
            translation=Vector3(0.0, 0.0, -LIDAR_HEIGHT_M),
            frame_id=LIDAR_FRAME,
            child_frame_id="base_link",
            ts=t,
        ),
    ]
    if scene.camera_edge:
        transforms.append(
            Transform(
                translation=Vector3(CAMERA_FORWARD_M, 0.0, CAMERA_HEIGHT_M),
                rotation=Quaternion.from_euler(Vector3(-math.pi / 2, 0.0, -math.pi / 2)),
                frame_id="base_link",
                child_frame_id=CAMERA_FRAME,
                ts=t,
            )
        )
    return TFMessage(*transforms)


def _camera_info(scene: Scene, t: float) -> CameraInfo:
    k = [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0] if scene.intrinsics else [0.0] * 9
    return CameraInfo(
        width=1280,
        height=720,
        K=k,
        D=[-0.1, 0.02, 0.0, 0.0, 0.0],
        distortion_model="plumb_bob",
        frame_id=CAMERA_FRAME,
        ts=t,
    )


def build_recording(path: Path, scene: Scene = Scene()) -> Path:
    """Write a recording in the shape the Rust recorder leaves behind."""
    sweep = _lidar_sweep(scene)
    streams: dict[str, tuple[type, float, object]] = {
        "head_left_color": (
            CompressedImage,
            30.0,
            lambda t: CompressedImage(data=b"\xff\xd8" + f"{t:.3f}".encode(), ts=t),
        ),
        "head_right_color": (
            CompressedImage,
            30.0,
            lambda t: CompressedImage(
                data=b"\xff\xd8" + f"{t:.3f}".encode(), ts=t + scene.right_offset_s
            ),
        ),
        "head_left_info": (CameraInfo, 0.0, lambda t: _camera_info(scene, t)),
        "head_right_info": (CameraInfo, 0.0, lambda t: _camera_info(scene, t)),
        "pointlio_lidar": (
            PointCloud2,
            10.0,
            lambda t: PointCloud2.from_numpy(
                sweep, frame_id=LIDAR_FRAME, timestamp=t + scene.lidar_offset_s
            ),
        ),
        "pointlio_odometry": (
            Odometry,
            20.0,
            lambda t: Odometry(
                ts=t, frame_id="odom", child_frame_id=LIDAR_FRAME, pose=Pose(*_pose_at(scene, t))
            ),
        ),
        "tf": (TFMessage, 10.0, lambda t: _tf_at(scene, t)),
    }
    connection = sqlite3.connect(path)
    connection.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
    for name, (payload_type, rate_hz, make) in streams.items():
        if name in scene.omit:
            continue
        config = {
            "payload_module": f"{payload_type.__module__}.{payload_type.__qualname__}",
            "codec_id": "lcm",
        }
        connection.execute("INSERT INTO _streams VALUES (?, ?)", (name, json.dumps(config)))
        connection.execute(f'CREATE TABLE "{name}" (id INTEGER PRIMARY KEY, ts REAL NOT NULL)')
        connection.execute(f'CREATE TABLE "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB)')
        if name in scene.empty:
            continue
        offsets = [0.0] if rate_hz == 0.0 else np.arange(0.0, scene.duration_s, 1.0 / rate_hz)
        for row, offset in enumerate(offsets, start=1):
            t = BASE_TS + float(offset)
            message = make(t)
            stamp = getattr(message, "ts", t)
            connection.execute(f'INSERT INTO "{name}" VALUES (?, ?)', (row, stamp))
            connection.execute(
                f'INSERT INTO "{name}_blob" VALUES (?, ?)', (row, message.lcm_encode())
            )
    connection.commit()
    connection.close()
    return path


def _fake_matcher(binary, left, right, calibration, prefix, *, downscale, rotation, extra):
    """A stereo floor sitting off the true floor by the rotation error.

    Yaw dominates, as on the real head, and the error is a quadrature sum so
    each angle can be found with the others still wrong.
    """
    assert left.exists() and right.exists() and calibration.exists()
    error = math.sqrt(
        (3.0 * (rotation["right_yaw_rad"] - TRUE["right_yaw_rad"])) ** 2
        + (rotation["right_pitch_rad"] - TRUE["right_pitch_rad"]) ** 2
        + (0.5 * (rotation["right_roll_rad"] - TRUE["right_roll_rad"])) ** 2
    )
    x, z = np.meshgrid(np.arange(-3.0, 3.0, 0.05), np.arange(0.5, 7.0, 0.05))
    height = CAMERA_HEIGHT_M + 10.0 * error
    return np.column_stack([x.ravel(), np.full(x.size, height), z.ravel()]), 1000


@pytest.fixture
def matcher(monkeypatch, tmp_path: Path) -> Path:
    """A binary that exists, and a run_matcher that never runs it."""
    binary = tmp_path / "stereo_offline"
    binary.write_text("#!/bin/sh\nexit 1\n")
    monkeypatch.setattr(stereo_fit, "run_matcher", _fake_matcher)
    return binary


def _run(recording: Path, binary: Path, *extra: str, out: Path | None = None) -> int:
    out = out or recording.with_name("calibration.json")
    return main(
        [
            str(recording),
            "--out",
            str(out),
            "--stereo-offline",
            str(binary),
            # Small enough that a six-second synthetic recording passes.
            "--min-duration-s",
            "5",
            "--min-pairs",
            "4",
            "--instants",
            "6",
            "--yaw-sweep",
            "-0.03:0.03:0.01",
            "--pitch-sweep",
            "-0.01:0.01:0.005",
            "--roll-sweep",
            "-0.01:0.01:0.005",
            *extra,
        ]
    )


# --- the fit ------------------------------------------------------------------


def test_fits_the_angles_and_writes_the_calibration(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db")
    out = tmp_path / "calibration.json"
    assert _run(recording, matcher, out=out) == 0

    written = parse_stereo_calibration(out.read_text(), source=str(out))
    assert written.right_yaw_rad == pytest.approx(TRUE["right_yaw_rad"], abs=1e-9)
    assert written.right_pitch_rad == pytest.approx(TRUE["right_pitch_rad"], abs=1e-9)
    assert written.right_roll_rad == pytest.approx(TRUE["right_roll_rad"], abs=1e-9)
    assert written.baseline_m == pytest.approx(0.120195)
    assert written.pairs_used == 6
    assert written.source_recording == str(recording)
    assert written.fitted_at and "T" in written.fitted_at
    assert written.left is not None and written.left.fx == 900.0 and written.left.width == 1280
    assert written.right is not None and written.right.distortion == [-0.1, 0.02, 0.0, 0.0, 0.0]
    score = written.score or {}
    for key in ("floor_rms_m_before", "floor_rms_m_after", "recall_before", "recall_after"):
        assert key in score
    assert score["floor_rms_m_after"] < score["floor_rms_m_before"]
    assert score["floor_rms_m_after"] == pytest.approx(0.0, abs=1e-6)
    assert score["recall_after"] > 0.9

    verdict = capsys.readouterr().out
    assert "yaw -0.0125 rad" in verdict
    assert "6 pairs used" in verdict
    assert str(out) in verdict


def test_a_directory_holding_memory_db_is_accepted(tmp_path: Path, matcher: Path):
    build_recording(tmp_path / "memory.db")
    assert _run(tmp_path, matcher, out=tmp_path / "calibration.json") == 0


def test_verbose_reports_each_trial(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db")
    assert _run(recording, matcher, "--verbose") == 0
    err = capsys.readouterr().err
    assert "--- right_yaw_rad ---" in err
    assert "lidar floor points" in err
    assert "usable pairs" in err


# --- the refusals -------------------------------------------------------------


def _refusal(capsys) -> str:
    err = capsys.readouterr().err
    assert err.startswith("calibrate_stereo: ")
    return err


def test_missing_recording(tmp_path: Path, matcher: Path, capsys):
    assert _run(tmp_path / "nope.db", matcher) == EXIT_NO_RECORDING
    message = _refusal(capsys)
    assert "recording not found" in message
    assert RECORDER_BLUEPRINT in message


def test_directory_without_a_recording(tmp_path: Path, matcher: Path, capsys):
    assert _run(tmp_path, matcher) == EXIT_NO_RECORDING
    assert "without a memory.db" in _refusal(capsys)


def test_missing_matcher_names_the_build_command(tmp_path: Path, capsys):
    recording = build_recording(tmp_path / "memory.db")
    assert _run(recording, tmp_path / "missing_stereo_offline") == EXIT_NO_MATCHER
    message = _refusal(capsys)
    assert BUILD_COMMAND in message
    assert "--stereo-offline" in message


def test_missing_streams_are_named_with_the_recorder(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(
        tmp_path / "memory.db", Scene(omit=("pointlio_lidar", "head_right_info"))
    )
    assert _run(recording, matcher) == EXIT_MISSING_STREAMS
    message = _refusal(capsys)
    assert "pointlio_lidar" in message and "head_right_info" in message
    assert RECORDER_BLUEPRINT in message
    assert "record the tf stream" not in message


def test_missing_tf_says_to_record_it(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(omit=("tf",)))
    assert _run(recording, matcher) == EXIT_MISSING_STREAMS
    assert "record the tf stream" in _refusal(capsys)


def test_an_empty_stream_is_reported_as_such(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(empty=("pointlio_odometry",)))
    assert _run(recording, matcher) == EXIT_MISSING_STREAMS
    message = _refusal(capsys)
    assert "pointlio_odometry" in message and "no messages" in message


def test_too_short_says_how_long_to_record(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(duration_s=3.0))
    assert _run(recording, matcher) == EXIT_TOO_SHORT
    message = _refusal(capsys)
    assert "needs at least 5 s" in message
    assert "Record for at least 60 s" in message


def test_default_minimum_duration_is_forty_five_seconds(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db")
    code = main(
        [str(recording), "--out", str(tmp_path / "c.json"), "--stereo-offline", str(matcher)]
    )
    assert code == EXIT_TOO_SHORT
    assert "at least 45 s" in _refusal(capsys)


def test_barely_moved_on_a_short_straight_line(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(travel_m=0.3, headings=1))
    assert _run(recording, matcher) == EXIT_BARELY_MOVED
    message = _refusal(capsys)
    assert "barely moved" in message
    assert "0.30 m" in message and "1 distinct heading" in message
    assert "degenerate" in message
    assert "Drive" in message


def test_barely_moved_on_too_few_headings_even_when_far(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(travel_m=5.0, headings=2))
    assert _run(recording, matcher) == EXIT_BARELY_MOVED
    assert "2 distinct heading" in _refusal(capsys)


def test_no_tf_path_to_the_camera(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(camera_edge=False))
    assert _run(recording, matcher) == EXIT_TOO_FEW_PAIRS
    message = _refusal(capsys)
    assert f"no path {CAMERA_FRAME} <- {LIDAR_FRAME}" in message
    assert "0 of those a tf path" in message


def test_eyes_too_far_apart_in_time(tmp_path: Path, matcher: Path, capsys):
    # The right eye runs half a frame behind the left, and the skew allowed is
    # tighter than that.
    recording = build_recording(tmp_path / "memory.db", Scene(right_offset_s=1 / 60))
    assert _run(recording, matcher, "--pair-skew-s", "0.005") == EXIT_TOO_FEW_PAIRS
    message = _refusal(capsys)
    assert "never within 0.005 s" in message
    assert "--pair-skew-s" in message
    # Loosening the skew, as the message suggests, makes it fit.
    assert _run(recording, matcher, "--pair-skew-s", "0.03") == 0


def test_no_lidar_sweep_near_the_frames(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(lidar_offset_s=1 / 60))
    assert _run(recording, matcher, "--cloud-skew-s", "0.005") == EXIT_TOO_FEW_PAIRS
    message = _refusal(capsys)
    assert "No lidar sweep lands within 0.005 s" in message
    assert "--cloud-skew-s" in message


def test_not_enough_pairs_for_the_minimum(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db")
    assert _run(recording, matcher, "--min-pairs", "100000") == EXIT_TOO_FEW_PAIRS
    message = _refusal(capsys)
    assert "needs at least 100000" in message
    assert "Record for longer" in message


def test_no_floor_says_where_to_point_the_camera(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(floor=False))
    assert _run(recording, matcher) == EXIT_NO_FLOOR
    message = _refusal(capsys)
    assert "found a floor in only 0 of 6" in message
    assert "floor 1-6 m ahead is in view" in message


def test_no_intrinsics(tmp_path: Path, matcher: Path, capsys):
    recording = build_recording(tmp_path / "memory.db", Scene(intrinsics=False))
    assert _run(recording, matcher) == EXIT_NO_INTRINSICS
    message = _refusal(capsys)
    assert "no usable intrinsics" in message
    assert "head_left_info" in message


def test_matcher_failure_is_reported_with_its_stderr(
    tmp_path: Path, matcher: Path, monkeypatch, capsys
):
    def broken(*args, **kwargs):
        raise stereo_fit.MatcherError("cannot decode 00_l.jpg: not a JPEG")

    monkeypatch.setattr(stereo_fit, "run_matcher", broken)
    recording = build_recording(tmp_path / "memory.db")
    assert _run(recording, matcher) == EXIT_MATCHER_FAILED
    message = _refusal(capsys)
    assert "stereo_offline failed" in message
    assert "not a JPEG" in message


def test_every_exit_code_is_distinct_and_documented():
    codes = {
        name: value
        for name, value in vars(calibrate_stereo).items()
        if name.startswith("EXIT_") and isinstance(value, int)
    }
    assert len(set(codes.values())) == len(codes)
    assert 0 not in codes.values() and 1 not in codes.values() and 2 not in codes.values()
    for value in codes.values():
        assert f"\n{value:<8}" in calibrate_stereo.__doc__


# --- pieces -------------------------------------------------------------------


def test_negative_sweep_bounds_survive_argparse():
    # argparse takes "-0.03:0.03:0.01" for an option name; the natural spelling
    # has to work, since almost every sweep starts below zero.
    argv = ["x.db", "--yaw-sweep", "-0.03:0.03:0.01", "--pitch", "-0.01", "--out", "-"]
    joined = calibrate_stereo.join_negative_values(argv)
    assert joined == ["x.db", "--yaw-sweep=-0.03:0.03:0.01", "--pitch=-0.01", "--out", "-"]
    args = calibrate_stereo.build_parser().parse_args(joined)
    assert args.yaw_sweep == "-0.03:0.03:0.01" and args.pitch == -0.01
    # A positive value, an `=` form and a trailing option are left alone.
    assert calibrate_stereo.join_negative_values(["--yaw-sweep", "0:1:0.5", "--yaw-sweep"]) == [
        "--yaw-sweep",
        "0:1:0.5",
        "--yaw-sweep",
    ]


def test_choose_instants_spreads_through_the_usable_pairs():
    usable = [float(i) for i in range(100)]
    chosen = calibrate_stereo.choose_instants(usable, 5)
    assert chosen == [0.0, 24.0, 49.0, 74.0, 99.0]
    assert calibrate_stereo.choose_instants(usable[:3], 10) == [0.0, 1.0, 2.0]
    assert calibrate_stereo.choose_instants([], 10) == []
