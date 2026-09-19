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

"""The fit engine on synthetic geometry: no recording, no robot, no matcher."""

from __future__ import annotations

import math
import os
from pathlib import Path
import stat

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.galaxea.r1pro.stereo_fit import (
    OPTICAL_UP,
    Axis,
    Floor,
    Instant,
    MatcherError,
    Trial,
    evaluate,
    find_floor,
    fit_plane,
    measure,
    parse_sweep,
    pick_best,
    plane_distance,
    read_cloud_bin,
    rotate_direction,
    run_matcher,
    spread,
    sweep,
    transform_points,
    write_calibration,
    yaw_of,
)

# The camera sits this high above the floor in every synthetic scene. In the
# optical frame the floor is then the plane y = CAMERA_HEIGHT_M.
CAMERA_HEIGHT_M = 1.5


def floor_grid(*, height_m: float = CAMERA_HEIGHT_M, spacing: float = 0.1) -> np.ndarray:
    """Floor points in the optical frame, 0.5-7 m ahead and 3 m either side."""
    x, z = np.meshgrid(np.arange(-3.0, 3.0, spacing), np.arange(0.5, 7.0, spacing))
    return np.column_stack([x.ravel(), np.full(x.size, height_m), z.ravel()])


def wall_grid(*, distance_m: float = 8.0) -> np.ndarray:
    """A wall facing the camera, 3 m wide and 2.5 m tall."""
    x, y = np.meshgrid(np.arange(-3.0, 3.0, 0.1), np.arange(-1.0, CAMERA_HEIGHT_M, 0.1))
    return np.column_stack([x.ravel(), y.ravel(), np.full(x.size, distance_m)])


def synthetic_floor() -> Floor:
    found = find_floor(np.vstack([floor_grid(), wall_grid()]), up=OPTICAL_UP)
    assert found is not None
    return found


# --- sweeps -------------------------------------------------------------------


def test_parse_sweep_range_is_inclusive_and_rounded():
    assert parse_sweep("-0.01:0.01:0.005") == [-0.01, -0.005, 0.0, 0.005, 0.01]
    # lo + step * i must not come out as 0.012500000000000002: it ends up in
    # the calibration file.
    assert 0.0125 in parse_sweep("-0.03:0.03:0.0025")


def test_parse_sweep_accepts_a_list():
    assert parse_sweep("0.1, -0.2,0.3") == [0.1, -0.2, 0.3]


@pytest.mark.parametrize("text", ["0:1:0", "0:1:-1", "1:0:0.1", "a:b", "", "1:2:3:4"])
def test_parse_sweep_rejects_nonsense(text):
    with pytest.raises(ValueError):
        parse_sweep(text)


def test_spread_skips_the_ends_and_keeps_the_count():
    stamps = spread((0.0, 100.0), 5)
    assert len(stamps) == 5
    assert stamps[0] == pytest.approx(5.0)
    assert stamps[-1] == pytest.approx(95.0)
    assert spread((0.0, 100.0), 1) == [50.0]


# --- frames -------------------------------------------------------------------


def _transform(*, translation=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> Transform:
    return Transform(
        translation=Vector3(*translation),
        rotation=Quaternion.from_euler(Vector3(*rpy)),
        frame_id="parent",
        child_frame_id="child",
        ts=1.0,
    )


def test_transform_points_rotates_then_translates():
    # A quarter turn about z takes +x to +y, then the offset is added.
    transform = _transform(translation=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, math.pi / 2))
    out = transform_points(np.array([[1.0, 0.0, 0.0]]), transform)
    assert out.shape == (1, 3)
    assert out[0] == pytest.approx([1.0, 3.0, 3.0], abs=1e-9)


def test_transform_points_agrees_with_the_transform_class():
    transform = _transform(translation=(0.3, -0.2, 1.1), rpy=(0.1, -0.2, 0.7))
    points = np.random.default_rng(1).normal(size=(20, 3))
    matrix = transform.to_matrix()
    homogeneous = np.column_stack([points, np.ones(len(points))])
    expected = (homogeneous @ matrix.T)[:, :3]
    assert transform_points(points, transform) == pytest.approx(expected, abs=1e-9)


def test_rotate_direction_ignores_translation():
    transform = _transform(translation=(5.0, 5.0, 5.0), rpy=(0.0, 0.0, math.pi / 2))
    assert rotate_direction((1.0, 0.0, 0.0), transform) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_yaw_of_reads_the_heading_back():
    for yaw in (-2.0, -0.3, 0.0, 0.7, 2.9):
        assert yaw_of(Quaternion.from_euler(Vector3(0.0, 0.0, yaw))) == pytest.approx(yaw)


# --- the floor ----------------------------------------------------------------


def test_fit_plane_recovers_a_tilted_plane_through_noise():
    rng = np.random.default_rng(2)
    normal = np.array([0.1, -0.98, 0.05])
    normal /= np.linalg.norm(normal)
    offset = -1.7
    basis_a = np.cross(normal, [1.0, 0.0, 0.0])
    basis_b = np.cross(normal, basis_a)
    coefficients = rng.uniform(-3.0, 3.0, size=(500, 2))
    points = coefficients[:, :1] * basis_a + coefficients[:, 1:] * basis_b - offset * normal
    points += rng.normal(scale=0.005, size=points.shape)
    plane = fit_plane(points)
    assert plane is not None
    fitted_normal = np.array(plane[:3])
    # The sign of the normal is arbitrary here; find_floor is what orients it.
    assert abs(float(fitted_normal @ normal)) == pytest.approx(1.0, abs=1e-4)
    assert np.abs(plane_distance(plane, points)).max() < 0.03


def test_fit_plane_refuses_a_line_and_too_few_points():
    line = np.column_stack([np.linspace(0, 5, 50), np.zeros(50), np.zeros(50)])
    assert fit_plane(line) is None
    assert fit_plane(np.zeros((2, 3))) is None


def test_find_floor_picks_the_floor_not_the_wall():
    floor = synthetic_floor()
    a, b, c, d = floor.plane
    # Normal points up (-y in optical), offset puts the plane at y = height.
    assert (a, b, c) == pytest.approx((0.0, -1.0, 0.0), abs=1e-6)
    assert d == pytest.approx(CAMERA_HEIGHT_M, abs=1e-6)
    assert len(floor.points) == len(floor_grid())
    assert len(floor.other) == len(wall_grid())


def test_find_floor_refuses_a_wall_when_there_is_no_floor():
    # A wall's lowest band fits a perfectly good plane. Without the tilt guard
    # that plane would be "the floor" and every stereo point metres from it.
    assert find_floor(wall_grid(), up=OPTICAL_UP) is None


def test_find_floor_needs_enough_points():
    assert find_floor(floor_grid()[:100], up=OPTICAL_UP) is None
    assert find_floor(floor_grid(), up=OPTICAL_UP, min_points=50) is not None


def test_find_floor_rejects_a_zero_up():
    with pytest.raises(ValueError):
        find_floor(floor_grid(), up=(0.0, 0.0, 0.0))


# --- the score ----------------------------------------------------------------


def test_measure_scores_a_perfect_cloud_as_zero_error_and_high_recall():
    floor = synthetic_floor()
    score = measure(floor_grid(spacing=0.05), floor)
    assert score.rms_m == pytest.approx(0.0, abs=1e-9)
    assert score.recall > 0.9


def test_measure_reports_a_shifted_floor_as_its_shift():
    floor = synthetic_floor()
    # 10 cm below the lidar's floor: further from the camera along +y.
    score = measure(floor_grid(height_m=CAMERA_HEIGHT_M + 0.10, spacing=0.05), floor)
    assert score.rms_m == pytest.approx(0.10, abs=1e-6)
    assert score.mean_signed_m == pytest.approx(-0.10, abs=1e-6)


def test_measure_windows_the_lidar_by_range_not_the_depth():
    floor = synthetic_floor()
    near = measure(floor_grid(spacing=0.05), floor, min_range_m=1.0, max_range_m=2.0)
    everything = measure(floor_grid(spacing=0.05), floor, min_range_m=0.0, max_range_m=100.0)
    assert 0 < near.wanted < everything.wanted


def test_measure_drops_directions_where_the_lidar_saw_something_standing():
    floor = synthetic_floor()
    # Put a box on the floor 3 m ahead; those directions must not be scored,
    # because the stereo cloud is entitled to see the box there.
    box = floor_grid()[(np.abs(floor_grid()[:, 0]) < 0.5) & (np.abs(floor_grid()[:, 2] - 3) < 0.3)]
    box = box - np.array([0.0, 0.3, 0.0])
    occluded = Floor(plane=floor.plane, points=floor.points, other=np.vstack([floor.other, box]))
    assert (
        measure(floor_grid(spacing=0.05), occluded).wanted
        < measure(floor_grid(spacing=0.05), floor).wanted
    )


def test_measure_with_nothing_recovered():
    floor = synthetic_floor()
    empty = measure(np.zeros((0, 3)), floor)
    assert empty.recovered == 0 and empty.recall == 0.0 and empty.wanted > 0
    elsewhere = measure(np.array([[0.0, -5.0, 1.0]]), floor)
    assert elsewhere.recovered == 0


# --- the matcher --------------------------------------------------------------


def _camera_info(**overrides) -> CameraInfo:
    fields = dict(
        width=1920,
        height=1536,
        K=[1013.8, 0.0, 958.7, 0.0, 1013.4, 768.2, 0.0, 0.0, 1.0],
        D=[-0.25, -0.43, 0.0001, 0.0, -0.02],
        distortion_model="plumb_bob",
    )
    fields.update(overrides)
    return CameraInfo(**fields)


def test_write_calibration_writes_two_eyes_of_fourteen_numbers(tmp_path: Path):
    path = write_calibration(tmp_path / "calib.txt", _camera_info(), _camera_info(D=[0.1] * 8))
    lines = [line for line in path.read_text().splitlines() if not line.startswith("#")]
    assert len(lines) == 2
    left = [float(v) for v in lines[0].split()]
    right = [float(v) for v in lines[1].split()]
    assert len(left) == len(right) == 14
    assert left[:6] == [1920, 1536, 1013.8, 1013.4, 958.7, 768.2]
    # Five plumb_bob coefficients are padded out to the eight the parser wants.
    assert left[6:] == [-0.25, -0.43, 0.0001, 0.0, -0.02, 0.0, 0.0, 0.0]
    assert right[6:] == [0.1] * 8


def test_write_calibration_refuses_empty_intrinsics(tmp_path: Path):
    with pytest.raises(ValueError, match="right CameraInfo"):
        write_calibration(tmp_path / "calib.txt", _camera_info(), _camera_info(K=[0.0] * 9))


def test_read_cloud_bin_round_trips_f32_triples(tmp_path: Path):
    points = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 9.0]], dtype=np.float32)
    path = tmp_path / "x_cloud.bin"
    path.write_bytes(points.astype("<f4").tobytes())
    assert read_cloud_bin(path) == pytest.approx(points)


def _fake_binary(path: Path, script: str) -> Path:
    path.write_text("#!/bin/sh\n" + script)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return path


def test_run_matcher_passes_the_rotation_and_reads_the_cloud_back(tmp_path: Path):
    # A stand-in matcher that echoes its arguments and writes one point.
    binary = _fake_binary(
        tmp_path / "stereo_offline",
        'echo "$@" > "$4_args"\n'
        'printf "\\001\\000\\000\\000\\002\\000\\000\\000\\003\\000\\000\\000" > "$4_cloud.bin"\n'
        'echo "64x48 valid=1234/3072 near=1 far=2"\n',
    )
    cloud, matched = run_matcher(
        binary,
        tmp_path / "l.jpg",
        tmp_path / "r.jpg",
        tmp_path / "calib.txt",
        tmp_path / "out",
        downscale=8,
        rotation={"right_yaw_rad": -0.0125},
        extra=("baseline_m=0.12",),
    )
    assert matched == 1234
    assert cloud.shape == (1, 3)
    args = (tmp_path / "out_args").read_text().split()
    assert args[4] == "8"
    assert "right_yaw_rad=-0.0125" in args
    assert args[-1] == "baseline_m=0.12"


def test_run_matcher_raises_with_the_binary_stderr(tmp_path: Path):
    binary = _fake_binary(tmp_path / "stereo_offline", 'echo "cannot decode l.jpg" >&2\nexit 3\n')
    with pytest.raises(MatcherError, match="cannot decode"):
        run_matcher(
            binary,
            tmp_path / "l.jpg",
            tmp_path / "r.jpg",
            tmp_path / "calib.txt",
            tmp_path / "out",
            downscale=8,
            rotation={},
        )


# --- trials and the sweep -----------------------------------------------------

TRUE = {"right_roll_rad": -0.0025, "right_pitch_rad": 0.00375, "right_yaw_rad": -0.0125}


def _synthetic_matcher(instant: Instant, rotation: dict[str, float]) -> tuple[np.ndarray, int]:
    """A stereo cloud whose floor sits off the true floor by the rotation error.

    Yaw dominates, as it does on the real head. The error is a quadrature sum
    so that each angle can be found with the others still wrong, which is what
    the axis-by-axis sweep relies on.
    """
    error = math.sqrt(
        (3.0 * (rotation["right_yaw_rad"] - TRUE["right_yaw_rad"])) ** 2
        + (rotation["right_pitch_rad"] - TRUE["right_pitch_rad"]) ** 2
        + (0.5 * (rotation["right_roll_rad"] - TRUE["right_roll_rad"])) ** 2
    )
    return floor_grid(height_m=CAMERA_HEIGHT_M + 10.0 * error, spacing=0.05), 1000


def _instants(count: int = 3) -> list[Instant]:
    floor = synthetic_floor()
    return [
        Instant(ts=float(i), left=Path(f"{i}_l.jpg"), right=Path(f"{i}_r.jpg"), floor=floor)
        for i in range(count)
    ]


def test_evaluate_takes_medians_over_the_instants():
    trial = evaluate(_instants(3), dict(TRUE), _synthetic_matcher)
    assert trial.rms_m == pytest.approx(0.0, abs=1e-9)
    assert trial.matched == 3000
    assert trial.recall > 0.9
    wrong = dict(TRUE, right_yaw_rad=TRUE["right_yaw_rad"] + 0.01)
    assert evaluate(_instants(3), wrong, _synthetic_matcher).rms_m == pytest.approx(0.3, abs=1e-6)


def test_the_objective_prefers_the_true_rotation():
    instants = _instants(2)
    trials = []
    for yaw in (-0.03, -0.02, -0.0125, -0.005, 0.0, 0.01):
        trials.append(evaluate(instants, dict(TRUE, right_yaw_rad=yaw), _synthetic_matcher))
    assert pick_best(trials).rotation["right_yaw_rad"] == -0.0125


def _trial(rms: float, recall: float = 1.0, **rotation) -> Trial:
    return Trial(rotation=rotation, matched=0, recall=recall, rms_m=rms, bias_m=0.0)


def test_pick_best_gates_on_recall():
    # The lowest RMS belongs to a trial that only sees a sliver of the floor:
    # it must not win.
    trials = [_trial(0.05, 0.8, yaw=0.0), _trial(0.01, 0.1, yaw=0.03), _trial(0.04, 0.9, yaw=0.01)]
    assert pick_best(trials).rotation == {"yaw": 0.01}


def test_pick_best_ignores_trials_with_no_rms_unless_there_is_nothing_else():
    nothing = _trial(float("nan"), 0.0, yaw=0.0)
    assert pick_best([nothing, _trial(0.2, 0.5, yaw=1.0)]).rotation == {"yaw": 1.0}
    assert pick_best([nothing]) is nothing
    with pytest.raises(ValueError):
        pick_best([])


def test_sweep_finds_each_angle_and_refines_off_the_coarse_grid():
    instants = _instants(2)
    calls: list[dict[str, float]] = []

    def evaluator(rotation):
        calls.append(dict(rotation))
        return evaluate(instants, rotation, _synthetic_matcher)

    start = {"right_roll_rad": 0.0, "right_pitch_rad": 0.0, "right_yaw_rad": 0.0}
    axes = [
        Axis("right_yaw_rad", parse_sweep("-0.03:0.03:0.01")),
        Axis("right_pitch_rad", parse_sweep("-0.01:0.01:0.005")),
        Axis("right_roll_rad", parse_sweep("-0.01:0.01:0.005")),
    ]
    lines: list[str] = []
    best, history = sweep(evaluator, start, axes, report=lines.append)
    # None of the true values is on its coarse grid; the fine pass at a
    # quarter step lands on each exactly.
    assert best == pytest.approx(TRUE, abs=1e-9)
    # Yaw was swept first, holding pitch and roll at their start.
    assert calls[0]["right_yaw_rad"] == -0.03 and calls[0]["right_pitch_rad"] == 0.0
    # Pitch was swept with the fitted yaw held.
    first_pitch = next(c for c in calls if c["right_pitch_rad"] == -0.01)
    assert first_pitch["right_yaw_rad"] == pytest.approx(TRUE["right_yaw_rad"])
    assert len(history) == len(calls)
    assert any("best right_yaw_rad" in line for line in lines)


def test_sweep_without_refinement_stays_on_the_grid():
    evaluator = lambda rotation: evaluate(_instants(1), rotation, _synthetic_matcher)  # noqa: E731
    start = {"right_roll_rad": 0.0, "right_pitch_rad": 0.0, "right_yaw_rad": 0.0}
    best, history = sweep(
        evaluator, start, [Axis("right_yaw_rad", parse_sweep("-0.03:0.03:0.01"))], refine=False
    )
    assert best["right_yaw_rad"] == -0.01
    assert len(history) == 7


def test_sweep_skips_empty_axes_and_keeps_the_start():
    evaluator = lambda rotation: _trial(0.0, **rotation)  # noqa: E731
    start = {"right_roll_rad": 0.1, "right_pitch_rad": 0.2, "right_yaw_rad": 0.3}
    best, history = sweep(evaluator, start, [Axis("right_yaw_rad", [])])
    assert best == start and history == []


def test_fake_binary_helper_is_executable(tmp_path: Path):
    # The run_matcher tests lean on this; make sure the mode bit really lands.
    binary = _fake_binary(tmp_path / "bin", "exit 0\n")
    assert os.access(binary, os.X_OK)
