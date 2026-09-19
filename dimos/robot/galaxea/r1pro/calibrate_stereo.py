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

"""Fit the R1 Pro head's stereo calibration from a recording.

The head is two monocular cameras that were never calibrated as a pair. This
takes a recording made by the ``r1pro-calibration-recorder`` blueprint, fits
how the right eye is aimed relative to the left (roll, pitch, yaw) against the
chassis lidar's floor, and writes ``calibration.json`` for
:mod:`dimos.robot.galaxea.r1pro.stereo_calibration` to load::

    python -m dimos.robot.galaxea.r1pro.calibrate_stereo recording.db --out calibration.json

The recording needs ``head_left_color`` and ``head_right_color`` (JPEG),
``head_left_info`` and ``head_right_info`` (CameraInfo), ``pointlio_lidar``
(PointCloud2 in ``lidar_pointlio_link``), ``pointlio_odometry`` and ``tf``.

Yaw is swept first, because it has by far the largest effect: a relative yaw
shifts every disparity by the same amount and "the whole room is nearer than it
is" and "the eyes converge" are the same two pictures, so only a metric
reference -- the lidar's floor -- can tell them apart. Pitch and roll follow,
each holding the earlier answers fixed, each refined around its best.

**Exit codes.** Every way the data can be insufficient has its own code and a
message that says what was found, what is needed and what to do about it:

======  ===========================================================
0       fitted; ``calibration.json`` written
2       bad command line (argparse)
3       the recording path does not exist or is not a recording
4       a required stream is missing or empty
5       the recording is shorter than ``--min-duration-s``
6       the robot barely moved (odometry span or headings too small)
7       fewer than ``--min-pairs`` usable left/right/lidar/tf instants
8       the lidar found no floor in most instants
9       the ``stereo_offline`` binary is missing
10      ``stereo_offline`` ran and failed
11      a CameraInfo carries no usable intrinsics
======  ===========================================================
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import tempfile
from typing import Any

import numpy as np

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.robot.galaxea.r1pro import stereo_fit
from dimos.robot.galaxea.r1pro.stereo_fit import (
    OPTICAL_UP,
    Axis,
    Instant,
    MatcherError,
    Rotation,
    Trial,
    parse_sweep,
)

EXIT_NO_RECORDING = 3
EXIT_MISSING_STREAMS = 4
EXIT_TOO_SHORT = 5
EXIT_BARELY_MOVED = 6
EXIT_TOO_FEW_PAIRS = 7
EXIT_NO_FLOOR = 8
EXIT_NO_MATCHER = 9
EXIT_MATCHER_FAILED = 10
EXIT_NO_INTRINSICS = 11

RECORDER_BLUEPRINT = "r1pro-calibration-recorder"
BUILD_COMMAND = "cargo build --release -p dimos-depth-cloud"
DEFAULT_MATCHER = DIMOS_PROJECT_ROOT / "target" / "release" / "stereo_offline"

# The URDF's head camera joints: y +0.059919 and y -0.060276.
DEFAULT_BASELINE_M = 0.120195

DEFAULT_YAW_SWEEP = "-0.03:0.03:0.0025"
DEFAULT_PITCH_SWEEP = "-0.01:0.01:0.001"
DEFAULT_ROLL_SWEEP = "-0.01:0.01:0.002"

# Two odometry headings this far apart count as different. Three of them is
# the least that rules out a fit from one pose or a straight line.
HEADING_BIN_RAD = math.radians(15.0)
MIN_DISTINCT_HEADINGS = 3

# At least this fraction of the chosen instants must have a floor, or the
# recording is of the wrong scene rather than merely unlucky.
MIN_FLOOR_FRACTION = 0.5

# How many odometry messages to decode for the movement check. The span and
# the headings do not need every message at 50 Hz.
ODOMETRY_SAMPLES = 400


class InsufficientError(Exception):
    """The recording cannot be fitted; carries the exit code and the advice."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Streams:
    """The stream names this fitter reads, all overridable."""

    left: str = "head_left_color"
    right: str = "head_right_color"
    left_info: str = "head_left_info"
    right_info: str = "head_right_info"
    lidar: str = "pointlio_lidar"
    odometry: str = "pointlio_odometry"
    tf: str = "tf"

    def required(self) -> tuple[str, ...]:
        return (
            self.left,
            self.right,
            self.left_info,
            self.right_info,
            self.lidar,
            self.odometry,
            self.tf,
        )


@dataclass(frozen=True)
class PairCensus:
    """How many left frames survived each requirement, in order."""

    lefts: int
    with_right: int
    with_cloud: int
    with_tf: int
    usable: list[float]
    """Timestamps of the left frames that met every requirement."""


@dataclass(frozen=True)
class Verdict:
    """What the fit found, for the terminal and the file."""

    rotation: Rotation
    before: Trial
    after: Trial
    pairs_used: int
    out: Path


# --- sufficiency --------------------------------------------------------------


def resolve_recording(argument: str) -> Path:
    """The ``.db`` file: the argument itself, or ``memory.db`` inside a directory."""
    path = Path(argument).expanduser()
    if path.is_dir():
        inside = path / "memory.db"
        if inside.exists():
            return inside
        raise InsufficientError(
            EXIT_NO_RECORDING,
            f"{path} is a directory without a memory.db. Pass the memory.db the "
            f"{RECORDER_BLUEPRINT} blueprint wrote, or the directory holding it.",
        )
    if not path.exists():
        raise InsufficientError(
            EXIT_NO_RECORDING,
            f"recording not found: {path}. Pass the memory.db the {RECORDER_BLUEPRINT} "
            "blueprint wrote, or the directory holding it.",
        )
    return path


def resolve_matcher(explicit: str | None) -> Path:
    """Where ``stereo_offline`` is, or a message saying how to build it."""
    path = Path(explicit).expanduser() if explicit else DEFAULT_MATCHER
    if path.exists():
        return path
    raise InsufficientError(
        EXIT_NO_MATCHER,
        f"stereo_offline not found at {path}. Build it from the repo root "
        f"({DIMOS_PROJECT_ROOT}) with:\n  {BUILD_COMMAND}\nor pass --stereo-offline <path>.",
    )


def check_streams(recording: Any, streams: Streams) -> None:
    present = set(recording.list_streams())
    missing = [name for name in streams.required() if name not in present]
    if missing:
        hint = ""
        if streams.tf in missing:
            hint = (
                f" The {streams.tf} stream carries the lidar-to-camera mount; without it "
                "the lidar cannot be placed in the camera's frame, so record the tf stream."
            )
        raise InsufficientError(
            EXIT_MISSING_STREAMS,
            f"{recording.path} is missing stream(s) {', '.join(missing)} "
            f"(found: {', '.join(sorted(present)) or 'none'}). Record with the "
            f"{RECORDER_BLUEPRINT} blueprint, which writes all of "
            f"{', '.join(streams.required())}.{hint}",
        )
    empty = [name for name in streams.required() if len(recording.stream(name)) == 0]
    if empty:
        raise InsufficientError(
            EXIT_MISSING_STREAMS,
            f"stream(s) {', '.join(empty)} in {recording.path} are present but hold no "
            f"messages. Check the source was publishing while the {RECORDER_BLUEPRINT} "
            "blueprint ran, then record again.",
        )


def check_duration(recording: Any, streams: Streams, min_duration_s: float) -> float:
    stream = recording.stream(streams.left)
    duration = float(stream.last_ts() - stream.first_ts())
    if duration < min_duration_s:
        raise InsufficientError(
            EXIT_TOO_SHORT,
            f"the recording holds {duration:.1f} s of {streams.left}; the fit needs at least "
            f"{min_duration_s:.0f} s. Record for at least {max(60.0, min_duration_s):.0f} s "
            "while driving the robot around.",
        )
    return duration


def check_movement(recording: Any, streams: Streams, min_travel_m: float) -> tuple[float, int]:
    """Translation span and distinct headings from odometry, or a complaint.

    A fit from one pose is degenerate: any rotation that flattens *that* floor
    is a solution, and the lidar's floor at one pose is one plane. Moving and
    turning is what makes the floor's geometry over-determine the angles.
    """
    stream = recording.stream(streams.odometry)
    count = len(stream)
    picks = sorted(set(np.linspace(0, count - 1, min(count, ODOMETRY_SAMPLES)).astype(int)))
    positions: list[tuple[float, float, float]] = []
    headings: set[int] = set()
    for index in picks:
        message = stream.find_closest(stream.stamps[index], tolerance=0.0)
        if message is None:
            continue
        position = message.pose.position
        positions.append((float(position.x), float(position.y), float(position.z)))
        yaw = stereo_fit.yaw_of(message.pose.orientation)
        headings.add(math.floor((yaw + math.pi) / HEADING_BIN_RAD))
    if not positions:
        raise InsufficientError(
            EXIT_BARELY_MOVED,
            f"{streams.odometry} could not be decoded, so the robot's movement is unknown. "
            "Record again with Point-LIO running so odometry is written.",
        )
    array = np.array(positions)
    span = float(np.linalg.norm(array.max(axis=0) - array.min(axis=0)))
    if span < min_travel_m or len(headings) < MIN_DISTINCT_HEADINGS:
        raise InsufficientError(
            EXIT_BARELY_MOVED,
            f"the robot barely moved: odometry spans {span:.2f} m over {len(headings)} "
            f"distinct heading(s); the fit needs at least {min_travel_m:.1f} m and "
            f"{MIN_DISTINCT_HEADINGS} headings, because a fit from one pose is degenerate. "
            "Drive a few metres with at least two turns while recording.",
        )
    return span, len(headings)


def _nearest(stamps: Sequence[float], ts: float, tolerance: float) -> int | None:
    """Index of the stamp nearest *ts* within *tolerance*, without decoding anything."""
    if not stamps:
        return None
    position = bisect_left(stamps, ts)
    candidates = [i for i in (position - 1, position) if 0 <= i < len(stamps)]
    best = min(candidates, key=lambda i: abs(stamps[i] - ts))
    return best if abs(stamps[best] - ts) <= tolerance else None


def census_pairs(
    recording: Any,
    tf: Any,
    streams: Streams,
    *,
    camera_frame: str,
    lidar_frame: str,
    pair_skew_s: float,
    cloud_skew_s: float,
) -> PairCensus:
    """Count, over every left frame, which have a right, a lidar sweep and a tf path.

    Works on the stamp indexes alone, so a thirty-hertz hour costs nothing to
    census; only the instants that are actually used are decoded later.
    """
    lefts = recording.stream(streams.left).stamps
    rights = recording.stream(streams.right).stamps
    clouds = recording.stream(streams.lidar).stamps
    with_right = with_cloud = with_tf = 0
    usable: list[float] = []
    for ts in lefts:
        if _nearest(rights, ts, pair_skew_s) is None:
            continue
        with_right += 1
        if _nearest(clouds, ts, cloud_skew_s) is None:
            continue
        with_cloud += 1
        if tf.get(camera_frame, lidar_frame, ts, warn=False) is None:
            continue
        with_tf += 1
        usable.append(ts)
    return PairCensus(len(lefts), with_right, with_cloud, with_tf, usable)


def check_pairs(
    census: PairCensus,
    streams: Streams,
    *,
    min_pairs: int,
    camera_frame: str,
    lidar_frame: str,
    pair_skew_s: float,
    cloud_skew_s: float,
) -> None:
    if len(census.usable) >= min_pairs:
        return
    found = (
        f"of {census.lefts} {streams.left} frames, {census.with_right} had a "
        f"{streams.right} frame within {pair_skew_s} s, {census.with_cloud} of those a "
        f"{streams.lidar} sweep within {cloud_skew_s} s, and {census.with_tf} of those a tf "
        f"path {camera_frame} <- {lidar_frame}"
    )
    if census.with_cloud and not census.with_tf:
        advice = (
            f"The tf stream has no path {camera_frame} <- {lidar_frame} at those times: "
            "the mount transforms were not being published. Run the coordinator and "
            f"Point-LIO blueprints alongside {RECORDER_BLUEPRINT} so tf carries the mount, "
            "then record again."
        )
    elif census.with_right and not census.with_cloud:
        advice = (
            f"No lidar sweep lands within {cloud_skew_s} s of the frames: check "
            f"{streams.lidar} was being published, or loosen --cloud-skew-s."
        )
    elif census.lefts and not census.with_right:
        advice = (
            f"The two eyes are never within {pair_skew_s} s of each other: check both "
            "cameras were streaming, or loosen --pair-skew-s."
        )
    else:
        advice = "Record for longer, or loosen --pair-skew-s / --cloud-skew-s."
    raise InsufficientError(
        EXIT_TOO_FEW_PAIRS,
        f"only {len(census.usable)} usable left/right pair(s); the fit needs at least "
        f"{min_pairs}: {found}. {advice}",
    )


def choose_instants(usable: Sequence[float], count: int) -> list[float]:
    """*count* timestamps spread evenly through the usable ones."""
    if not usable:
        return []
    picks = sorted(set(np.linspace(0, len(usable) - 1, min(count, len(usable))).astype(int)))
    return [usable[i] for i in picks]


def collect_instants(
    recording: Any,
    tf: Any,
    streams: Streams,
    stamps: Sequence[float],
    work: Path,
    *,
    camera_frame: str,
    lidar_frame: str,
    world_up_frame: str,
    pair_skew_s: float,
    cloud_skew_s: float,
    plane_tolerance_m: float,
    verbose: bool,
) -> list[Instant]:
    """Decode the chosen instants, place the lidar in the camera and find its floor."""
    lefts = recording.stream(streams.left)
    rights = recording.stream(streams.right)
    clouds = recording.stream(streams.lidar)
    kept: list[Instant] = []
    for index, ts in enumerate(stamps):
        left = lefts.find_closest(ts, tolerance=1e-6)
        right = rights.find_closest(ts, tolerance=pair_skew_s)
        cloud = clouds.find_closest(ts, tolerance=cloud_skew_s)
        camera_from_lidar = tf.get(camera_frame, lidar_frame, ts, warn=False)
        if left is None or right is None or cloud is None or camera_from_lidar is None:
            continue
        # Up in the camera's frame comes from tf when the recording can say
        # where the world is; otherwise the head is assumed level, which on
        # the R1 it nearly is.
        up = OPTICAL_UP
        camera_from_world = tf.get(camera_frame, world_up_frame, ts, warn=False)
        if camera_from_world is not None:
            up = stereo_fit.rotate_direction((0.0, 0.0, 1.0), camera_from_world)
        points, _ = cloud.as_numpy()
        lidar = stereo_fit.transform_points(points, camera_from_lidar)
        floor = stereo_fit.find_floor(lidar, up=up, tolerance_m=plane_tolerance_m)
        if floor is None:
            if verbose:
                print(f"  t={ts:.3f}: no floor in the lidar", file=sys.stderr)
            continue
        left_path = work / f"{index:02d}_l.jpg"
        right_path = work / f"{index:02d}_r.jpg"
        left_path.write_bytes(bytes(left.data))
        right_path.write_bytes(bytes(right.data))
        kept.append(Instant(ts=ts, left=left_path, right=right_path, floor=floor))
        if verbose:
            print(f"  t={ts:.3f}: {len(floor.points)} lidar floor points", file=sys.stderr)
    return kept


def check_floor(kept: Sequence[Instant], chosen: int) -> None:
    needed = max(1, math.ceil(MIN_FLOOR_FRACTION * chosen))
    if len(kept) >= needed:
        return
    raise InsufficientError(
        EXIT_NO_FLOOR,
        f"the lidar found a floor in only {len(kept)} of {chosen} instants; the fit needs "
        f"at least {needed}. Point the camera so the floor 1-6 m ahead is in view: record "
        "in an open area facing away from walls, with the head level, and drive slowly.",
    )


# --- the fit ------------------------------------------------------------------


def write_calibration_file(
    recording: Any,
    streams: Streams,
    work: Path,
) -> tuple[Path, Any, Any]:
    left_info = recording.stream(streams.left_info).first()
    right_info = recording.stream(streams.right_info).first()
    try:
        path = stereo_fit.write_calibration(work / "calib.txt", left_info, right_info)
    except ValueError as error:
        raise InsufficientError(
            EXIT_NO_INTRINSICS,
            f"{error}. The robot publishes them on /calib/head_*/camera_info; check that "
            f"{streams.left_info} and {streams.right_info} were recorded from there.",
        ) from error
    return path, left_info, right_info


def eye_intrinsics(info: Any) -> Any:
    from dimos.robot.galaxea.r1pro.stereo_calibration import EyeIntrinsics

    k = list(info.K)
    return EyeIntrinsics(
        width=int(info.width),
        height=int(info.height),
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        distortion_model=str(info.distortion_model or "plumb_bob"),
        distortion=[float(value) for value in list(info.D)],
    )


def write_result(
    out: Path,
    *,
    rotation: Rotation,
    before: Trial,
    after: Trial,
    pairs_used: int,
    baseline_m: float,
    left_info: Any,
    right_info: Any,
    source: Path,
) -> Path:
    from dimos.robot.galaxea.r1pro.stereo_calibration import (
        R1StereoCalibration,
        write_stereo_calibration,
    )

    calibration = R1StereoCalibration(
        baseline_m=baseline_m,
        right_roll_rad=round(rotation["right_roll_rad"], 6),
        right_pitch_rad=round(rotation["right_pitch_rad"], 6),
        right_yaw_rad=round(rotation["right_yaw_rad"], 6),
        left=eye_intrinsics(left_info),
        right=eye_intrinsics(right_info),
        source_recording=str(source),
        fitted_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        pairs_used=pairs_used,
        score={
            "floor_rms_m_before": before.rms_m,
            "floor_rms_m_after": after.rms_m,
            "recall_before": before.recall,
            "recall_after": after.recall,
            "floor_bias_m_before": before.bias_m,
            "floor_bias_m_after": after.bias_m,
            "matched_before": float(before.matched),
            "matched_after": float(after.matched),
        },
        notes=(
            "Fitted by calibrate_stereo against the chassis lidar's floor. These angles "
            "belong to this rig's head mount: refit after the head is unbolted, and do not "
            "copy to another R1 Pro."
        ),
    )
    return write_stereo_calibration(out, calibration)


def fit(args: argparse.Namespace) -> Verdict:
    from dimos.memory.raw_replay import RawRecording, tf_buffer

    streams = Streams(
        left=args.left_stream,
        right=args.right_stream,
        left_info=args.left_info_stream,
        right_info=args.right_info_stream,
        lidar=args.lidar_stream,
        odometry=args.odometry_stream,
        tf=args.tf_stream,
    )
    source = resolve_recording(args.recording)
    binary = resolve_matcher(args.stereo_offline)
    axes = [
        Axis("right_yaw_rad", parse_sweep(args.yaw_sweep) if args.yaw_sweep else []),
        Axis("right_pitch_rad", parse_sweep(args.pitch_sweep) if args.pitch_sweep else []),
        Axis("right_roll_rad", parse_sweep(args.roll_sweep) if args.roll_sweep else []),
    ]
    start: Rotation = {
        "right_roll_rad": args.roll,
        "right_pitch_rad": args.pitch,
        "right_yaw_rad": args.yaw,
    }
    report = (lambda line: print(line, file=sys.stderr, flush=True)) if args.verbose else None

    with (
        RawRecording(source) as recording,
        tempfile.TemporaryDirectory(prefix="calibrate-stereo-") as tmp,
    ):
        work = Path(tmp)
        check_streams(recording, streams)
        duration = check_duration(recording, streams, args.min_duration_s)
        span, headings = check_movement(recording, streams, args.min_travel_m)
        tf = tf_buffer(recording, streams.tf)
        census = census_pairs(
            recording,
            tf,
            streams,
            camera_frame=args.camera_frame,
            lidar_frame=args.lidar_frame,
            pair_skew_s=args.pair_skew_s,
            cloud_skew_s=args.cloud_skew_s,
        )
        check_pairs(
            census,
            streams,
            min_pairs=args.min_pairs,
            camera_frame=args.camera_frame,
            lidar_frame=args.lidar_frame,
            pair_skew_s=args.pair_skew_s,
            cloud_skew_s=args.cloud_skew_s,
        )
        if args.verbose:
            print(
                f"{duration:.0f} s, {span:.1f} m over {headings} headings, "
                f"{len(census.usable)} usable pairs",
                file=sys.stderr,
            )
        chosen = choose_instants(census.usable, args.instants)
        instants = collect_instants(
            recording,
            tf,
            streams,
            chosen,
            work,
            camera_frame=args.camera_frame,
            lidar_frame=args.lidar_frame,
            world_up_frame=args.world_up_frame,
            pair_skew_s=args.pair_skew_s,
            cloud_skew_s=args.cloud_skew_s,
            plane_tolerance_m=args.plane_tolerance_m,
            verbose=args.verbose,
        )
        check_floor(instants, len(chosen))
        calibration, left_info, right_info = write_calibration_file(recording, streams, work)

        extra = (f"baseline_m={args.baseline_m!r}", *args.matcher_param)

        def matcher(instant: Instant, rotation: Rotation) -> tuple[np.ndarray, int]:
            try:
                return stereo_fit.run_matcher(
                    binary,
                    instant.left,
                    instant.right,
                    calibration,
                    work / "fit",
                    downscale=args.downscale,
                    rotation=rotation,
                    extra=extra,
                )
            except MatcherError as error:
                raise InsufficientError(
                    EXIT_MATCHER_FAILED,
                    f"stereo_offline failed on the pair at t={instant.ts:.3f}: {error}. "
                    "Check the JPEGs decode and the CameraInfos match the frames' resolution.",
                ) from error

        def evaluator(rotation: Rotation) -> Trial:
            return stereo_fit.evaluate(
                instants,
                rotation,
                matcher,
                bin_rad=args.bin_rad,
                near_m=args.near_m,
                far_m=args.far_m,
            )

        before = evaluator(start)
        if report:
            report("--- before ---")
            report("  " + before.line())
        best, history = stereo_fit.sweep(
            evaluator, start, axes, refine=not args.no_refine, report=report
        )
        after = next(
            (trial for trial in reversed(history) if trial.rotation == best),
            before,
        )
        out = write_result(
            Path(args.out).expanduser(),
            rotation=best,
            before=before,
            after=after,
            pairs_used=len(instants),
            baseline_m=args.baseline_m,
            left_info=left_info,
            right_info=right_info,
            source=source,
        )
    return Verdict(rotation=best, before=before, after=after, pairs_used=len(instants), out=out)


def verdict_lines(verdict: Verdict) -> list[str]:
    rotation = verdict.rotation
    before, after = verdict.before, verdict.after
    return [
        "right eye: "
        f"roll {rotation['right_roll_rad']:+.4f} rad, "
        f"pitch {rotation['right_pitch_rad']:+.4f} rad, "
        f"yaw {rotation['right_yaw_rad']:+.4f} rad "
        f"({math.degrees(rotation['right_roll_rad']):+.2f}, "
        f"{math.degrees(rotation['right_pitch_rad']):+.2f}, "
        f"{math.degrees(rotation['right_yaw_rad']):+.2f} deg)",
        f"floor RMS {before.rms_m * 100:.1f} cm -> {after.rms_m * 100:.1f} cm, "
        f"recall {before.recall * 100:.0f}% -> {after.recall * 100:.0f}%, "
        f"{verdict.pairs_used} pairs used",
        f"wrote {verdict.out}",
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fit the R1 Pro head's stereo calibration from a recording.",
        epilog=f"Exit codes: see the module docstring. Record with the {RECORDER_BLUEPRINT} "
        "blueprint.",
    )
    parser.add_argument("recording", help="memory.db, or the directory holding it")
    parser.add_argument("--out", required=True, help="calibration.json to write")
    parser.add_argument(
        "--stereo-offline",
        default=None,
        help=f"the stereo_offline binary (default {DEFAULT_MATCHER}; build with `{BUILD_COMMAND}`)",
    )
    parser.add_argument("--instants", type=int, default=24, help="pairs to fit on")
    parser.add_argument("--downscale", type=int, default=8, help="matcher downscale factor")
    parser.add_argument("--baseline-m", type=float, default=DEFAULT_BASELINE_M)
    parser.add_argument("--yaw-sweep", default=DEFAULT_YAW_SWEEP, help="lo:hi:step rad, or ''")
    parser.add_argument("--pitch-sweep", default=DEFAULT_PITCH_SWEEP, help="lo:hi:step rad")
    parser.add_argument("--roll-sweep", default=DEFAULT_ROLL_SWEEP, help="lo:hi:step rad")
    parser.add_argument("--no-refine", action="store_true", help="skip the fine pass")
    parser.add_argument("--roll", type=float, default=0.0, help="starting roll, rad")
    parser.add_argument("--pitch", type=float, default=0.0, help="starting pitch, rad")
    parser.add_argument("--yaw", type=float, default=0.0, help="starting yaw, rad")
    parser.add_argument("--min-pairs", type=int, default=12)
    parser.add_argument("--min-duration-s", type=float, default=45.0)
    parser.add_argument("--min-travel-m", type=float, default=1.0)
    parser.add_argument("--pair-skew-s", type=float, default=0.06)
    parser.add_argument("--cloud-skew-s", type=float, default=0.15)
    parser.add_argument(
        "--plane-tolerance-m", type=float, default=stereo_fit.DEFAULT_PLANE_TOLERANCE_M
    )
    parser.add_argument("--bin-rad", type=float, default=stereo_fit.DEFAULT_BIN_RAD)
    parser.add_argument("--near-m", type=float, default=stereo_fit.DEFAULT_NEAR_M)
    parser.add_argument("--far-m", type=float, default=stereo_fit.DEFAULT_FAR_M)
    parser.add_argument("--camera-frame", default="camera_head_left_link")
    parser.add_argument("--lidar-frame", default="lidar_pointlio_link")
    parser.add_argument("--world-up-frame", default="odom")
    parser.add_argument("--left-stream", default=Streams.left)
    parser.add_argument("--right-stream", default=Streams.right)
    parser.add_argument("--left-info-stream", default=Streams.left_info)
    parser.add_argument("--right-info-stream", default=Streams.right_info)
    parser.add_argument("--lidar-stream", default=Streams.lidar)
    parser.add_argument("--odometry-stream", default=Streams.odometry)
    parser.add_argument("--tf-stream", default=Streams.tf)
    parser.add_argument(
        "--matcher-param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="extra stereo_offline parameter, repeatable",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser


NEGATIVE_VALUE_OPTIONS = (
    "--yaw-sweep",
    "--pitch-sweep",
    "--roll-sweep",
    "--roll",
    "--pitch",
    "--yaw",
)


def join_negative_values(argv: Sequence[str]) -> list[str]:
    """``--yaw-sweep -0.03:0.03:0.0025`` as one token, so argparse reads it as a value.

    A sweep's lower bound is negative more often than not, and argparse takes
    any token starting with ``-`` that is not a plain number for an option
    name, so the natural spelling fails with "expected one argument". The
    ``--option=value`` form always works; this rewrites the space-separated
    form into it for the options whose values are angles.
    """
    joined: list[str] = []
    tokens = list(argv)
    index = 0
    while index < len(tokens):
        token = tokens[index]
        follower = tokens[index + 1] if index + 1 < len(tokens) else None
        if token in NEGATIVE_VALUE_OPTIONS and follower is not None and follower.startswith("-"):
            joined.append(f"{token}={follower}")
            index += 2
            continue
        joined.append(token)
        index += 1
    return joined


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(join_negative_values(sys.argv[1:] if argv is None else argv))
    try:
        verdict = fit(args)
    except InsufficientError as error:
        print(f"calibrate_stereo: {error.message}", file=sys.stderr)
        return error.code
    for line in verdict_lines(verdict):
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
