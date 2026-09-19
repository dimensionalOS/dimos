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

"""The geometry and the search behind ``calibrate_stereo``.

Everything here is pure: given lidar points already in the camera's frame, find
the floor; given a stereo cloud and that floor, score it; given a scorer, sweep
an angle and refine around its best. The recording, the subprocess and the exit
codes live in :mod:`calibrate_stereo`, so this half can be tested on synthetic
planes without a robot, a recording or the Rust matcher.

Conventions, because they are the whole difficulty:

* The camera frame is **optical**: +x right, +y down, +z forward. ``stereo_offline``
  writes its cloud in that frame, and the lidar is transformed into it before
  anything is compared, so "up" in this frame is ``(0, -1, 0)`` unless tf says
  the head is tilted.
* A plane is ``(a, b, c, d)`` with ``a*x + b*y + c*z + d = 0`` and ``(a, b, c)``
  a unit normal, so :func:`plane_distance` is a signed distance in metres.
* The right eye's rotation is the dict ``stereo_offline`` takes on its command
  line: ``right_roll_rad``, ``right_pitch_rad``, ``right_yaw_rad``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from itertools import pairwise
import math
from pathlib import Path
import struct
import subprocess
from typing import Any

import numpy as np

Point = tuple[float, float, float]
Plane = tuple[float, float, float, float]
Rotation = dict[str, float]

ROTATION_KEYS = ("right_roll_rad", "right_pitch_rad", "right_yaw_rad")

# Up and forward in a camera-optical frame.
OPTICAL_UP: Point = (0.0, -1.0, 0.0)

# Angular bin for pairing the two clouds by direction, in radians (~0.6 deg).
# Bearing is what two clouds of very different density still share: the lidar
# has a few hundred floor returns where the stereo cloud has tens of thousands.
DEFAULT_BIN_RAD = 0.01

# A lidar point is "on the floor plane" if it is within this of it. Loose
# enough to hold a real floor's roughness and the lidar's own noise, tight
# enough to exclude a foot or a table leg.
DEFAULT_PLANE_TOLERANCE_M = 0.06

# Below this many lidar floor points the plane fit is not worth trusting.
MIN_PLANE_POINTS = 150

# The floor is scored over this range window, applied to the *lidar*. Closer
# than a metre the head cannot see the floor at all; further than six the
# lidar's own floor returns thin out.
DEFAULT_NEAR_M = 1.0
DEFAULT_FAR_M = 6.0

# A candidate rotation whose recall is below this fraction of the sweep's best
# recall is not allowed to win on RMS. See `pick_best`.
RECALL_GATE = 0.5

# A "floor" whose normal leans further than this from up is not a floor. The
# research fitter had no such check, and on a cloud with no floor in it the
# lowest band of a wall fits a perfectly good plane -- vertical, and then every
# stereo point is metres from it. The head tilts a few degrees at most.
MAX_FLOOR_TILT_RAD = math.radians(20.0)

# The middle eigenvalue must be at least this fraction of the largest for the
# points to span a plane rather than a line: a single lidar scan line across a
# floor has infinitely many best-fit planes, and any one of them is fabricated.
_PLANE_RANK_TOLERANCE = 1e-6


def parse_sweep(text: str) -> list[float]:
    """``lo:hi:step`` or a comma-separated list, in radians.

    The values are rounded to nine decimals so that ``lo + step * i`` comes out
    as ``0.0125`` rather than ``0.012500000000000002``, which is what ends up
    in the calibration file.
    """
    if ":" in text:
        parts = text.split(":")
        if len(parts) != 3:
            raise ValueError(f"a sweep is lo:hi:step, got {text!r}")
        lo, hi, step = (float(part) for part in parts)
        if step <= 0.0:
            raise ValueError(f"sweep step must be positive, got {step}")
        if hi < lo:
            raise ValueError(f"sweep hi {hi} is below lo {lo}")
        count = round((hi - lo) / step)
        return [round(lo + step * i, 9) for i in range(count + 1)]
    values = [round(float(part), 9) for part in text.split(",") if part.strip()]
    if not values:
        raise ValueError(f"empty sweep {text!r}")
    return values


def spread(span: tuple[float, float], count: int) -> list[float]:
    """*count* instants drawn evenly across *span*, skipping the very ends.

    The first and last moments of a recording are where streams are still
    starting and stopping, so a sample there is likelier to be missing one of
    the things it needs.
    """
    if count < 1:
        raise ValueError("count must be positive")
    first, last = span
    margin = (last - first) * 0.05
    lo, hi = first + margin, last - margin
    if hi <= lo:
        return [(first + last) / 2.0]
    if count == 1:
        return [(lo + hi) / 2.0]
    step = (hi - lo) / (count - 1)
    return [lo + step * i for i in range(count)]


def rotation_matrix(rotation: Any) -> np.ndarray:
    """A 3x3 matrix from anything with unit-quaternion ``x, y, z, w`` fields."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_points(points: Any, transform: Any) -> np.ndarray:
    """Apply a tf ``Transform`` (parent <- child) to child-frame points.

    ``tf.get(camera, lidar, ts)`` hands back exactly this: the transform whose
    parent is the camera, so applying it to lidar-frame points places them in
    the camera's frame.
    """
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    matrix = rotation_matrix(transform.rotation)
    translation = transform.translation
    offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
    return np.asarray(array @ matrix.T + offset)


def rotate_direction(direction: Point, transform: Any) -> Point:
    """Rotate a direction by a tf ``Transform``, ignoring its translation."""
    rotated = rotation_matrix(transform.rotation) @ np.asarray(direction, dtype=np.float64)
    return (float(rotated[0]), float(rotated[1]), float(rotated[2]))


def yaw_of(rotation: Any) -> float:
    """Heading about +z, in radians, from a unit quaternion."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def fit_plane(points: Any) -> Plane | None:
    """Least-squares plane through *points*, as a unit normal and offset.

    The normal is the covariance's smallest-eigenvalue eigenvector: the
    direction the points vary least in, which for a floor is "up".

    Returns ``None`` when the points are too few, or when they do not actually
    span a plane -- all on one line, or all at one spot.
    """
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(array) < 3:
        return None
    centroid = array.mean(axis=0)
    centred = array - centroid
    covariance = centred.T @ centred / len(array)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    smallest, middle, largest = eigenvalues
    if largest <= 0.0 or middle <= _PLANE_RANK_TOLERANCE * largest:
        return None
    normal = eigenvectors[:, 0]
    normal = normal / np.linalg.norm(normal)
    a, b, c = (float(v) for v in normal)
    return (a, b, c, float(-(normal @ centroid)))


def plane_distance(plane: Plane, points: Any) -> np.ndarray:
    """Signed distance from each point to *plane*; the normal is already unit."""
    a, b, c, d = plane
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return np.asarray(array @ np.array([a, b, c]) + d)


@dataclass(frozen=True)
class Floor:
    """The floor found in one lidar sweep, in the camera's frame."""

    plane: Plane
    points: np.ndarray
    """The lidar returns on the plane."""
    other: np.ndarray
    """Every other lidar return: whatever is standing on the floor."""


def find_floor(
    lidar: Any,
    *,
    up: Point = OPTICAL_UP,
    tolerance_m: float = DEFAULT_PLANE_TOLERANCE_M,
    min_points: int = MIN_PLANE_POINTS,
    max_tilt_rad: float = MAX_FLOOR_TILT_RAD,
) -> Floor | None:
    """The floor in a lidar cloud: the lowest large plane along *up*.

    *up* is the world's up expressed in the cloud's own frame. It is what
    separates the floor from a table top, and in a camera-optical frame down is
    +y, not -z.

    Seeds the fit with the points in the lowest 15% of the along-*up* spread
    and refits once on everything within *tolerance_m*, so a slightly tilted
    seed does not permanently exclude the far half of the floor. The normal is
    flipped to point up, so a signed distance above the floor is positive, and
    a plane leaning more than *max_tilt_rad* from up is refused: that is a wall
    or a ramp, not a floor.
    """
    array = np.asarray(lidar, dtype=np.float64).reshape(-1, 3)
    if len(array) < min_points:
        return None
    unit_up = np.asarray(up, dtype=np.float64)
    norm = np.linalg.norm(unit_up)
    if norm < 1e-9:
        raise ValueError("up must be a non-zero direction")
    unit_up = unit_up / norm

    heights = array @ unit_up
    cutoff = np.sort(heights)[max(0, int(0.15 * (len(heights) - 1)))]
    plane = fit_plane(array[heights <= cutoff])
    if plane is None:
        return None
    on_plane = np.abs(plane_distance(plane, array)) <= tolerance_m
    if on_plane.sum() < min_points:
        return None
    refit = fit_plane(array[on_plane])
    if refit is None:
        return None
    if np.dot(refit[:3], unit_up) < 0.0:
        refit = (-refit[0], -refit[1], -refit[2], -refit[3])
    if np.dot(refit[:3], unit_up) < math.cos(max_tilt_rad):
        return None
    final = np.abs(plane_distance(refit, array)) <= tolerance_m
    if final.sum() < min_points:
        return None
    return Floor(plane=refit, points=array[final], other=array[~final])


@dataclass(frozen=True)
class FloorScore:
    """How a stereo cloud agrees with the lidar's floor, for one instant."""

    wanted: int
    """Directions (bearing bins) the lidar found floor in, inside the window."""
    recovered: int
    """Of those, the directions the stereo cloud answered in."""
    rms_m: float
    """RMS of the stereo floor's distance from the lidar's plane."""
    mean_signed_m: float
    """Mean signed distance: positive is above the lidar's floor."""

    @property
    def recall(self) -> float:
        return self.recovered / self.wanted if self.wanted else 0.0


def bearing_bins(points: np.ndarray, bin_rad: float) -> np.ndarray:
    """``(azimuth, elevation)`` bin indices of each point, as an (N, 2) int array."""
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    azimuth = np.arctan2(array[:, 1], array[:, 0])
    elevation = np.arctan2(array[:, 2], np.hypot(array[:, 0], array[:, 1]))
    return np.asarray(np.floor(np.column_stack([azimuth, elevation]) / bin_rad).astype(np.int64))


def _bin_keys(bins: np.ndarray) -> np.ndarray:
    """Fold (azimuth, elevation) bin pairs into one integer per point."""
    return np.asarray(bins[:, 0] * 1_000_003 + bins[:, 1])


def measure(
    depth_cloud: Any,
    floor: Floor,
    *,
    bin_rad: float = DEFAULT_BIN_RAD,
    min_range_m: float = DEFAULT_NEAR_M,
    max_range_m: float = DEFAULT_FAR_M,
) -> FloorScore:
    """Score *depth_cloud* against the lidar's floor.

    Recall is per *direction*, not per point: the two clouds sample at wildly
    different densities, so counting points would reward whichever happens to
    be denser rather than whichever actually sees the floor.

    The range window is applied to the *lidar*, which decides the denominator.
    It is deliberately not applied to the depth: a reading in the window that
    is wildly wrong has to count as wrong, not vanish.

    Directions where the lidar saw something standing on the floor are dropped
    rather than scored, because there the depth is entitled to report the chair
    instead of the carpet and no rule looking only at the depth can tell that
    from a mistake.

    Within a direction the stereo reading is the *median* of the bin, not the
    point closest to the plane: picking the closest would score a denser cloud
    better for being denser, since with four times the pixels there are four
    times the chances one lands near the truth by luck.
    """
    lidar_floor = floor.points
    ranges = np.linalg.norm(lidar_floor, axis=1)
    in_window = lidar_floor[(ranges >= min_range_m) & (ranges <= max_range_m)]
    wanted = set(_bin_keys(bearing_bins(in_window, bin_rad)).tolist()) if len(in_window) else set()
    if len(floor.other):
        other_ranges = np.linalg.norm(floor.other, axis=1)
        other = floor.other[(other_ranges >= min_range_m) & (other_ranges <= max_range_m)]
        if len(other):
            wanted -= set(_bin_keys(bearing_bins(other, bin_rad)).tolist())
    if not wanted:
        return FloorScore(0, 0, 0.0, 0.0)

    cloud = np.asarray(depth_cloud, dtype=np.float64).reshape(-1, 3)
    if not len(cloud):
        return FloorScore(len(wanted), 0, 0.0, 0.0)
    keys = _bin_keys(bearing_bins(cloud, bin_rad))
    wanted_keys = np.fromiter(wanted, dtype=np.int64)
    hit = np.isin(keys, wanted_keys)
    if not hit.any():
        return FloorScore(len(wanted), 0, 0.0, 0.0)
    distances = plane_distance(floor.plane, cloud[hit])
    hit_keys = keys[hit]
    order = np.argsort(hit_keys, kind="stable")
    sorted_keys = hit_keys[order]
    sorted_distances = distances[order]
    starts = np.flatnonzero(np.r_[True, sorted_keys[1:] != sorted_keys[:-1]])
    ends = np.r_[starts[1:], len(sorted_keys)]
    per_direction = np.array(
        [np.median(sorted_distances[start:end]) for start, end in zip(starts, ends, strict=True)]
    )
    return FloorScore(
        wanted=len(wanted),
        recovered=len(per_direction),
        rms_m=float(np.sqrt(np.mean(per_direction**2))),
        mean_signed_m=float(per_direction.mean()),
    )


def write_calibration(path: Path, left_info: Any, right_info: Any) -> Path:
    """Write both eyes' intrinsics in the form ``stereo_offline`` reads.

    One line per eye: ``width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6``.
    Both eyes, each with its own distortion, because the matcher rectifies the
    pair onto one shared pinhole geometry and cannot do that from one eye's
    numbers: the R1's head lenses bend the bottom of the frame -- where the
    floor is -- by over a hundred pixels, and the two eyes by visibly different
    amounts.
    """
    lines = []
    for name, info in (("left", left_info), ("right", right_info)):
        k = list(info.K)
        if len(k) < 9 or not k[0] or not k[4]:
            raise ValueError(
                f"the {name} CameraInfo carries no usable intrinsics (K is empty or has a "
                "zero focal length); without them the matcher cannot turn disparity into metres"
            )
        distortion = [float(value) for value in list(info.D)[:8]]
        distortion += [0.0] * (8 - len(distortion))
        numbers = [
            float(info.width),
            float(info.height),
            float(k[0]),
            float(k[4]),
            float(k[2]),
            float(k[5]),
            *distortion,
        ]
        lines.append(" ".join(repr(value) for value in numbers))
    path.write_text(
        "# width height fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6\n" + "\n".join(lines) + "\n"
    )
    return path


def read_cloud_bin(path: Path) -> np.ndarray:
    """``stereo_offline``'s ``<prefix>_cloud.bin``: little-endian f32 xyz triples."""
    raw = path.read_bytes()
    count = len(raw) // 12
    values = struct.unpack(f"<{count * 3}f", raw[: count * 12])
    return np.array(values, dtype=np.float64).reshape(-1, 3)


class MatcherError(RuntimeError):
    """``stereo_offline`` ran and failed; the message carries its stderr."""


def run_matcher(
    binary: Path,
    left: Path,
    right: Path,
    calibration: Path,
    prefix: Path,
    *,
    downscale: int,
    rotation: Rotation,
    extra: Sequence[str] = (),
) -> tuple[np.ndarray, int]:
    """Match one pair at one rotation. Returns the cloud and the matched pixel count."""
    command = [
        str(binary),
        str(left),
        str(right),
        str(calibration),
        str(prefix),
        str(downscale),
        *(f"{name}={value!r}" for name, value in rotation.items()),
        *extra,
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
        raise MatcherError(detail)
    matched = 0
    for token in result.stdout.split():
        if token.startswith("valid="):
            matched = int(token.removeprefix("valid=").split("/")[0])
    return read_cloud_bin(Path(f"{prefix}_cloud.bin")), matched


@dataclass(frozen=True)
class Instant:
    """One moment of the recording, decoded once and kept.

    The lidar is already in the camera's frame and the floor already fitted:
    neither depends on the rotation being fitted, so doing it per candidate
    would be the same arithmetic thousands of times over.
    """

    ts: float
    left: Path
    right: Path
    floor: Floor


@dataclass(frozen=True)
class Trial:
    """What one candidate rotation scored, as medians over the instants."""

    rotation: Rotation
    matched: int
    recall: float
    rms_m: float
    bias_m: float

    def line(self) -> str:
        angles = " ".join(f"{name}={value:+.4f}" for name, value in self.rotation.items())
        return (
            f"{angles}  matched {self.matched:>8}  recall {self.recall * 100:5.1f}%  "
            f"RMS {self.rms_m * 100:6.1f} cm  bias {self.bias_m * 100:+7.1f} cm"
        )


Matcher = Callable[[Instant, Rotation], tuple[np.ndarray, int]]
"""Runs the matcher on one instant at one rotation: ``(cloud, matched_count)``."""

Evaluator = Callable[[Rotation], Trial]


def _median(values: Sequence[float]) -> float:
    return float(np.median(values)) if len(values) else float("nan")


def evaluate(
    instants: Sequence[Instant],
    rotation: Rotation,
    matcher: Matcher,
    *,
    bin_rad: float = DEFAULT_BIN_RAD,
    near_m: float = DEFAULT_NEAR_M,
    far_m: float = DEFAULT_FAR_M,
) -> Trial:
    """Score one candidate rotation over every instant.

    Medians rather than means, so one instant where the matcher fell apart (a
    person walked through the frame, a reflection on the floor) does not decide
    the angle for the other twenty.
    """
    matched = 0
    recalls: list[float] = []
    rmss: list[float] = []
    biases: list[float] = []
    for instant in instants:
        cloud, count = matcher(instant, rotation)
        matched += count
        score = measure(
            cloud, instant.floor, bin_rad=bin_rad, min_range_m=near_m, max_range_m=far_m
        )
        recalls.append(score.recall)
        if score.recovered:
            rmss.append(score.rms_m)
            biases.append(score.mean_signed_m)
    return Trial(
        rotation=dict(rotation),
        matched=matched,
        recall=_median(recalls),
        rms_m=_median(rmss),
        bias_m=_median(biases),
    )


def pick_best(trials: Sequence[Trial]) -> Trial:
    """The trial to keep: lowest floor RMS among those that still see the floor.

    The research fitter chose on RMS alone and printed recall beside it. That
    is kept as the objective, with one guard the unattended version needs: a
    rotation so wrong that the matcher only answers in a handful of directions
    can post a *lower* RMS than the truth, because a few lucky bins are easier
    to fit than the whole floor. So a candidate whose recall has fallen below
    half of the sweep's best recall is not allowed to win. Recall is a gate,
    not a term in the score, because trading centimetres of floor error for
    percentage points of coverage has no exchange rate that is right for both
    a yaw (which moves every bin equally) and a pitch (which drops bins first).

    A trial with no RMS at all (nothing recovered anywhere) never wins unless
    every trial is like that.
    """
    if not trials:
        raise ValueError("no trials to choose from")
    scored = [trial for trial in trials if not math.isnan(trial.rms_m)]
    if not scored:
        return trials[0]
    best_recall = max(trial.recall for trial in scored)
    eligible = [trial for trial in scored if trial.recall >= RECALL_GATE * best_recall]
    return min(eligible, key=lambda trial: trial.rms_m)


@dataclass(frozen=True)
class Axis:
    """One angle to sweep: its rotation key and the coarse values to try."""

    key: str
    values: Sequence[float]


def sweep(
    evaluator: Evaluator,
    start: Rotation,
    axes: Sequence[Axis],
    *,
    refine: bool = True,
    report: Callable[[str], None] | None = None,
) -> tuple[Rotation, list[Trial]]:
    """Sweep each axis in turn, holding the others at their current best.

    Axes are taken in the order given: yaw first, because it has by far the
    largest effect on the floor and a wrong yaw would swamp the pitch sweep.
    After the coarse pass over an axis the best value is refined at a quarter
    of the step, one step either side, so the answer is not quantised to the
    coarse grid. Every trial is returned, coarse and fine, for the log.
    """
    current = dict(start)
    history: list[Trial] = []
    for axis in axes:
        if not axis.values:
            continue
        if report:
            report(f"--- {axis.key} ---")
        best, trials = _sweep_axis(evaluator, current, axis, refine=refine, report=report)
        history.extend(trials)
        current[axis.key] = best
        if report:
            report(f"  best {axis.key}: {best:+.4f} rad ({math.degrees(best):+.3f} deg)")
    return current, history


def _sweep_axis(
    evaluator: Evaluator,
    current: Rotation,
    axis: Axis,
    *,
    refine: bool,
    report: Callable[[str], None] | None,
) -> tuple[float, list[Trial]]:
    """One axis: the coarse values, then a fine pass about the best of them."""
    seen: dict[float, Trial] = {}

    def try_value(value: float) -> None:
        candidate = dict(current)
        candidate[axis.key] = value
        trial = evaluator(candidate)
        seen[value] = trial
        if report:
            report("  " + trial.line())

    for value in axis.values:
        try_value(value)
    best = pick_best(list(seen.values()))
    coarse_step = _step_of(axis.values)
    if refine and coarse_step > 0.0:
        fine_step = coarse_step / 4.0
        centre = best.rotation[axis.key]
        for offset in range(-3, 4):
            value = round(centre + offset * fine_step, 9)
            if all(abs(value - tried) >= 1e-9 for tried in seen):
                try_value(value)
        best = pick_best(list(seen.values()))
    return best.rotation[axis.key], list(seen.values())


def _step_of(values: Sequence[float]) -> float:
    """The spacing of a sweep, or zero when it has fewer than two values."""
    if len(values) < 2:
        return 0.0
    return min(b - a for a, b in pairwise(sorted(values)))
