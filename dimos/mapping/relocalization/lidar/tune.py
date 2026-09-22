#!/usr/bin/env python3
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

"""Does relocalization find its place in a prior map, how well, and how fast?

Give it a recording and a premap built from the same world, and it probes:
accumulate a few lidar scans from one moment of the walk, relocalize that
little cloud against the premap, and check where it landed. See tune.md
for the tuning workflow.

The recording and the premap must share a coordinate frame - the usual way
being one recording, its premap assembled from a different stretch of it.
Ground truth is then the identity, and the error is how far the recovered
transform sits from it. Nothing else has to be labelled.

Probes are placed only where the premap actually covers the walk. A loop
walk revisits little of its route, so a probe from an uncovered stretch has
no right answer at all, and scoring it caps the eval below what any
configuration can reach. ``--from/--to`` narrows the search further when
you already know which stretch built the premap.
"""

from __future__ import annotations

from functools import lru_cache
from itertools import islice
import time
from typing import TYPE_CHECKING, Any, NamedTuple

import numpy as np
import typer

from dimos.mapping.relocalization.lidar.relocalize import (
    DEFAULT_PRESET,
    PRESETS,
    LidarRelocalizer,
    RelocalizeConfig,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from collections.abc import Callable

    from rich.table import Table

    from dimos.memory.store.sqlite import SqliteStore

# Bump when the search space or the objective changes; it keys the study.
SPACE = 13

# Every study lands here; browse with `uvx optuna-dashboard sqlite:///optuna.db`.
STORAGE = "sqlite:///optuna.db"

# Ground truth, and the only thing that decides whether a fix is right: a
# probe within this of identity found its place in the map. Real failures
# are nothing like marginal - they land tens of meters out - so this splits
# two far-apart populations rather than cutting a continuum.
TOLERANCE_M = 0.5
TOLERANCE_DEG = 2.0

# Scored when a config never lands a correct fix, so the accuracy objective
# still has a number optuna can rank. Any real miss is tens of meters.
NO_FIX_M = 1e3

# Seconds reserved after the last probe start, so every probe can gather its
# scans without running past the window. Fixed rather than derived from the
# frame count, so probes sit at the same places whatever `frames` a trial
# picks - otherwise trials would be scored on subtly different questions.
PROBE_RESERVE_S = 5.0

# A robot does not get one guess. It keeps listening while it thinks, so a
# failed match is not wasted time - the scans that arrived during it are the
# next attempt's evidence. Retries therefore need no schedule: the frame
# count for attempt N is simply everything the lidar has delivered by then,
# which makes each failure cost the wall clock it actually costs and no
# more. Where the window starts and ends is `config.min_frames` /
# `config.max_frames`, because how many sweeps it takes to recognise a place
# is a fact about the sensor, not about this eval.
#
# The price is that every attempt is another chance to accept something
# wrong, so this needs a stricter cutoff than a single shot would. That is
# what the false-fix rate over the unmatchable probes is here to measure.

# A local-map point this close to a premap point counts as covered.
COVERAGE_TOL_M = 0.5
# Frame count at which coverage is always measured, whatever a trial is
# using. Whether the premap holds a place has to be a fact about the place,
# not about how long that trial happened to accumulate: a longer window
# spans more ground and so drops below MIN_COVERAGE sooner near the edge of
# the map, which would tie the hit rate's denominator to the very knob the
# search is trying to judge.
COVERAGE_FRAMES = 10
# Below this share of the local map present in the premap, the premap does
# not contain this place. Such a probe is not excluded - it is the negative
# half of the test. The only right answer for it is a refusal, and accepting
# it anyway is the failure that matters most, because the module publishes
# what it accepts and everything downstream believes the TF.
MIN_COVERAGE = 0.8


# Outcome to the colour it prints in. A false fix is the one that matters, so
# it is the one that shouts.
OUTCOME_STYLE = {
    "hit": "green",
    "missed": "yellow",
    "refused": "dim",
    "FALSE FIX": "bold white on red",
}


def _table(title: str, *columns: str) -> Table:
    """A right-aligned table; a column name prefixed with ``<`` aligns left.

    Headers are kept short on purpose: these tables run to ten columns and
    get read in a half-width terminal, where rich would otherwise truncate
    cells to `st…` and `NOT in …`.
    """
    from rich.table import Table

    table = Table(title=title, title_justify="left", title_style="bold", header_style="bold cyan")
    for col in columns:
        table.add_column(
            col.lstrip("<"), justify="left" if col.startswith("<") else "right", no_wrap=True
        )
    return table


def probe_row(probe: Probe) -> list[str]:
    """One probe as table cells, in PROBE_COLUMNS order."""
    return [
        f"{probe.start:.0f}s",
        f"{probe.coverage:.0%}",
        "in map" if probe.answerable else "outside",
        f"[{OUTCOME_STYLE[probe.outcome]}]{probe.outcome}[/]"
        + (" [yellow]TRUTH?[/]" if probe.truth_suspect else ""),
        f"{probe.displacement_m:.3f}",
        f"{probe.frames_used}/{probe.attempts}",
        f"{probe.fitness:.3f}",
        f"{probe.latency_s:.2f}",
        f"{probe.wall_s:.2f}",
        f"{probe.cpu_s:.0f}",
    ]


PROBE_COLUMNS = (
    "start",
    "cov",
    "<where",
    "<outcome",
    "off m",
    "f/t",
    "fit",
    "lat",
    "wall",
    "cpu",
)


class Dataset(NamedTuple):
    """A recording, a premap of the same world, and where they overlap.

    ``window`` bounds the stretch of the recording probes may be drawn
    from. Leave it ``None`` to search the whole recording and let coverage
    decide; set it when you know which stretch built the premap and want
    the rest excluded outright.
    """

    recording: str
    premap: str
    lidar_stream: str = "fastlio_lidar"
    window: tuple[float, float] | None = None


DATASETS = {
    # ~780 s loop walk; its premap was assembled from the scans after 400 s,
    # so probes come from before that.
    "go2-sf-area1": Dataset(
        recording="recording_go2_mid360_2026-05-29_4-45pm-PST_corrected.db",
        premap="recording_go2_mid360_2026-05-29_4-45pm-PST_corrected.pc2.lcm",
        # Runs the full stretch before the premap's own scans: 0-250 s is
        # ground the loop revisits after 400 s and the premap therefore
        # holds, while 250-400 s is a one-time excursion it never saw.
        # Probes land in both, so every run carries places that must be
        # found and - about a third of them - places that must be refused.
        window=(0.0, 400.0),
    ),
}
DEFAULT_DATASET = "go2-sf-area1"


class Probe(NamedTuple):
    """One relocalization attempt and everything needed to judge it."""

    start: float
    n_points: int
    coverage: float
    translation_m: float
    rotation_deg: float
    tilt_deg: float
    # How far the recovered transform actually moved the cloud. Rotation and
    # translation do not add up to a distance on their own - a small angle
    # about a far-off center displaces a cloud further than a big one about
    # its middle - so this is the honest "how wrong was it" in meters.
    displacement_m: float
    # Median point-to-premap distance with the cloud at the truth pose, and
    # where the aligner put it. Aligned fitting *better* than truth means
    # the recording's own poses drifted, not that relocalization missed.
    fit_truth_m: float
    fit_aligned_m: float
    fitness: float
    rmse: float
    # Frames the ladder had reached when it stopped, and how many attempts
    # that took. `accepted` is False when it climbed the whole ladder without
    # a fix clearing the cutoff - a refusal, not a failure to run.
    frames_used: int
    attempts: int
    accepted: bool
    # Seconds of scans gathered plus seconds spent matching them: the whole
    # wait for a fix, which is what the frame count actually trades against.
    latency_s: float
    # Wall-clock seconds actually spent matching, with the waiting for scans
    # taken out. The gap between this and `latency_s` is how much of the wait
    # was gathering evidence rather than computing on it.
    wall_s: float
    # CPU seconds across every thread, as `time` reports user+sys. Open3D
    # threads its FPFH and RANSAC, so this runs several times the wall clock
    # and by a ratio that is not constant - wall time alone cannot stand in
    # for it. It is the honest price on a robot sharing its cores with
    # everything else.
    cpu_s: float
    # Best minus runner-up fitness across RANSAC restarts; 0 with one restart.
    cloud: PointCloud2
    transform: np.ndarray

    @property
    def answerable(self) -> bool:
        """Does the premap contain this place at all? Measured at COVERAGE_FRAMES."""
        return self.coverage >= MIN_COVERAGE

    @property
    def placed(self) -> bool:
        """Did it land where ground truth says? Meaningless unless answerable."""
        return self.translation_m <= TOLERANCE_M and self.rotation_deg <= TOLERANCE_DEG

    @property
    def outcome(self) -> str:
        """What this probe proves, once the ladder has stopped.

        ``hit`` and ``refused`` are the two right answers - one for a place
        the premap holds, one for a place it does not. ``FALSE FIX`` is the
        dangerous outcome: a transform the module would publish that puts
        the robot somewhere it is not. ``missed`` merely wastes the attempt.
        """
        if not self.accepted:
            return "refused" if not self.answerable else "missed"
        if self.answerable and self.placed:
            return "hit"
        return "FALSE FIX"

    @property
    def truth_suspect(self) -> bool:
        """Misplaced, yet fitting the premap better than ground truth does."""
        return self.answerable and not self.placed and self.fit_aligned_m < self.fit_truth_m


@lru_cache(maxsize=4)
def fixtures(name: str) -> tuple[PointCloud2, Any]:
    """The premap and an open store for a dataset, loaded once and shared by every trial."""
    from dimos.memory.store.sqlite import SqliteStore

    ds = DATASETS[name]
    premap = PointCloud2.lcm_decode(get_data(ds.premap).read_bytes())
    store = SqliteStore(path=str(get_data(ds.recording)), must_exist=True)
    store.start()
    return premap, store


@lru_cache(maxsize=4)
def premap_index(name: str, voxel: float = 0.2) -> Any:
    """The premap thinned for nearest-neighbour queries, built once."""
    premap, _ = fixtures(name)
    return premap.pointcloud.voxel_down_sample(voxel)


def premap_distance(name: str, points: np.ndarray) -> np.ndarray:
    """Per-point distance from ``points`` to the nearest premap point."""
    cloud = PointCloud2.from_numpy(points, frame_id="world", timestamp=0.0)
    return np.asarray(cloud.pointcloud.compute_point_cloud_distance(premap_index(name)))


def accumulate(
    store: SqliteStore,
    n_frames: int,
    *,
    start: float = 0.0,
    stop: float | None = None,
    lidar_stream: str = "fastlio_lidar",
    voxel: float = 0.1,
    max_range: float = 30.0,
) -> PointCloud2:
    """``n_frames`` lidar scans from ``start`` s in, ray-traced into one world cloud.

    ``fastlio_lidar`` clouds arrive already registered into the world frame,
    so each observation's own pose is only the sensor origin the raycast
    clears from - there is no tf tree to walk and no odom to join against.
    ``stop`` bounds the window in seconds from the recording's first scan.
    """
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

    lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
    t0, _ = lidar.get_time_range()
    frames = iter(lidar.after(t0 + start))
    limit = None if stop is None else t0 + stop

    mapper = VoxelRayMapper(voxel_size=voxel, max_range=max_range, emit_every=1)
    used = 0
    last_ts = t0 + start
    # A cloud with no pose is skipped, not counted: n_frames is a number of
    # registered scans, which is what the window question is about.
    for obs in islice(frames, n_frames * 4):
        if limit is not None and obs.ts > limit:
            break
        if obs.pose_tuple is None:
            continue
        x, y, z = obs.pose_tuple[:3]
        mapper.add_frame_world(obs.data.points_f32(), (x, y, z))
        used += 1
        last_ts = obs.ts
        if used == n_frames:
            break
    if used < n_frames:
        raise ValueError(f"only {used} of {n_frames} scans had a pose from {start:.0f}s")

    return PointCloud2.from_numpy(mapper.global_map(), frame_id="world", timestamp=last_ts)


@lru_cache(maxsize=512)
def local_map(name: str, frames: int, start: float, voxel: float) -> PointCloud2:
    """One probe's accumulated cloud, cached: it does not depend on RelocalizeConfig.

    Every trial of a study probes the same starts with the same frame count,
    so without this the tuning loop spends most of its time re-accumulating
    clouds it already built.
    """
    ds = DATASETS[name]
    _, store = fixtures(name)
    return accumulate(
        store,
        frames,
        start=start,
        stop=ds.window[1] if ds.window else None,
        lidar_stream=ds.lidar_stream,
        voxel=voxel,
    )


@lru_cache(maxsize=256)
def place_coverage(name: str, start: float, voxel: float) -> float:
    """How much of the premap holds the place a probe stands in.

    Always measured on a ``COVERAGE_FRAMES`` cloud, so two trials asking
    about the same start get the same answer no matter how many frames each
    of them accumulates.
    """
    return coverage(name, local_map(name, COVERAGE_FRAMES, start, voxel))


def coverage(name: str, cloud: PointCloud2) -> float:
    """Fraction of ``cloud`` the premap contains, with the cloud at its true pose.

    Answers whether the question is answerable at all, separately from
    whether the aligner answered it: a probe standing where the premap has
    no points cannot be placed by any parameter setting.
    """
    distances = premap_distance(name, np.asarray(cloud.pointcloud.points))
    return float((distances < COVERAGE_TOL_M).mean())


@lru_cache(maxsize=32)
def probe_starts(name: str, samples: int, half_step: bool = False) -> tuple[float, ...]:
    """``samples`` starts spread evenly over the dataset's window.

    Deliberately unfiltered. A stretch the premap never saw is not a broken
    question - it is the negative half of the test, and the only place a
    false fix can be caught. Coverage labels each probe afterwards; it does
    not decide which ones are asked.

    ``half_step`` returns the same number of starts, offset from the tuned
    ones and over the same window - the holdout. A config tuned on one set
    and still good on the other did not merely memorise which handful of
    places happen to be easy, and the two rates are comparable only because
    the counts match.
    """
    if samples < 1:
        raise ValueError(f"samples must be at least 1, got {samples}")
    ds = DATASETS[name]
    _, store = fixtures(name)
    lidar = store.stream(ds.lidar_stream, PointCloud2)
    span = lidar.get_time_range()[1] - lidar.get_time_range()[0]
    lo, hi = ds.window or (0.0, span)
    # A window may name an end past the recording - `--from` with no `--to`
    # does exactly that - and probes scheduled out there have no scans.
    hi = min(hi, span)
    last = hi - PROBE_RESERVE_S
    if last < lo:
        raise ValueError(f"the {lo}-{hi}s window leaves no room for a probe")
    if samples == 1:
        return (lo + (last - lo) / 2 if half_step else lo,)
    tuned = np.linspace(lo, last, samples)
    if not half_step:
        return tuple(float(t) for t in tuned)
    # Midpoints of a grid one finer: `samples` starts inside the same window,
    # offset from the tuned ones. Taking midpoints of the tuned grid instead
    # would return one probe fewer and compare unequal populations.
    edges = np.linspace(lo, last, samples + 1)
    starts = edges[:-1] + np.diff(edges) / 2
    # Two evenly spaced grids over one window share their centre whenever
    # `samples` is odd. Nudge those off, or the holdout would reuse a place
    # the config was tuned on.
    step = (last - lo) / samples
    for i, start in enumerate(starts):
        if np.isclose(start, tuned).any():
            starts[i] = start + step / 4
    return tuple(float(t) for t in starts)


@lru_cache(maxsize=8)
def scan_rate(name: str) -> float:
    """Lidar scans per second, for turning a frame count into seconds of waiting."""
    ds = DATASETS[name]
    _, store = fixtures(name)
    lidar = store.stream(ds.lidar_stream, PointCloud2)
    lo, hi = lidar.get_time_range()
    return float(lidar.count() / (hi - lo))


def identity_error(T: np.ndarray) -> tuple[float, float, float]:
    """``(meters, degrees, tilt degrees)`` that ``T`` sits from identity.

    Tilt is the part of the rotation that takes gravity off vertical. Both
    clouds come from a lidar-inertial odometry, so a correct fix has none of
    it and a tilted answer is physically impossible - which makes tilt a
    free sanity check on a result, separate from how far it landed. A purely
    yawed miss is the more dangerous kind: the right pose at the wrong place
    in the map, and it looks perfectly legal.
    """
    translation = float(np.linalg.norm(T[:3, 3]))
    cos = (float(np.trace(T[:3, :3])) - 1.0) / 2.0
    rotation = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    tilt = float(np.degrees(np.arccos(np.clip(T[2, 2], -1.0, 1.0))))
    return translation, rotation, tilt


def applied(T: np.ndarray, cloud: PointCloud2) -> np.ndarray:
    """``cloud``'s points where ``T`` puts them."""
    points: np.ndarray = np.asarray(cloud.pointcloud.points) @ T[:3, :3].T + T[:3, 3]
    return points


def run_probes(
    name: str,
    *,
    samples: int,
    voxel: float,
    config: RelocalizeConfig,
    half_step: bool = False,
    on_probe: Callable[[Probe], None] | None = None,
) -> list[Probe]:
    """Retry at each start until a fix clears the config's threshold, or evidence runs out.

    Uses :func:`align` rather than :func:`relocalize` so a refused fix is
    still visible: where a rejected hypothesis actually landed is the whole
    diagnosis, and the accept decision is a comparison this makes itself.

    Scans keep arriving while a match runs, so each attempt sees everything
    the lidar has delivered by the time it starts - the clock, not a
    schedule, decides how much evidence the next try gets. Latency is the
    whole wait: the scans gathered before the attempt that answered, plus
    every attempt made along the way, including the failed ones.
    """
    premap, _ = fixtures(name)
    # The premap's preprocessing depends on the config but not the probe, so
    # it is paid once per trial rather than once per attempt.
    relocalizer = LidarRelocalizer(premap.pointcloud, config)
    rate = scan_rate(name)
    probes: list[Probe] = []

    min_frames, max_frames = config.min_frames, config.max_frames

    for start in probe_starts(name, samples, half_step):
        # `elapsed` is the robot's clock: it starts owing the time it takes to
        # gather min_frames, and each attempt adds the time that attempt took.
        # `wall` and `cpu` are what the machine actually spent, which is not
        # the same number - wall excludes the waiting, cpu counts every thread.
        elapsed, wall, cpu = min_frames / rate, 0.0, 0.0
        attempts, accepted = 0, False
        while True:
            frames = min(max(int(elapsed * rate), min_frames), max_frames)
            cloud = local_map(name, frames, start, voxel)
            t0, c0 = time.monotonic(), time.process_time()
            result = relocalizer.align(cloud.pointcloud)
            dt = time.monotonic() - t0
            elapsed += dt
            wall += dt
            cpu += time.process_time() - c0
            attempts += 1
            if result.fitness >= config.fitness_threshold:
                accepted = True
                break
            if frames >= max_frames:
                break

        T = np.asarray(result.transformation)
        truth_points = np.asarray(cloud.pointcloud.points)
        moved = applied(T, cloud)
        translation, rotation, tilt = identity_error(T)
        probes.append(
            Probe(
                start=start,
                n_points=len(cloud),
                coverage=place_coverage(name, start, voxel),
                translation_m=translation,
                rotation_deg=rotation,
                tilt_deg=tilt,
                displacement_m=float(np.median(np.linalg.norm(moved - truth_points, axis=1))),
                fit_truth_m=float(np.median(premap_distance(name, truth_points))),
                fit_aligned_m=float(np.median(premap_distance(name, moved))),
                fitness=float(result.fitness),
                rmse=float(result.inlier_rmse),
                frames_used=frames,
                attempts=attempts,
                accepted=accepted,
                latency_s=elapsed,
                wall_s=wall,
                cpu_s=cpu,
                cloud=cloud,
                transform=T,
            )
        )
        if on_probe is not None:
            on_probe(probes[-1])
    return probes


def summarize(probes: list[Probe]) -> tuple[float, float, float, float, float]:
    """``(hit rate, false fix rate, error, median latency)`` - the objective.

    Ground truth decides where a fix belongs; ``fitness_threshold`` decides
    whether the module would have published it. Scoring both together is the
    point, because relocalization has two ways to be right and they pull
    against each other:

    * **hit rate** - of the probes the premap actually contains, the share
      placed correctly *and* accepted. Raising the cutoff costs hits.
    * **false fix rate** - of *all* probes, the share accepted while wrong,
      including every probe of a place the premap never saw. This is the
      outcome that hurts: the module publishes what it accepts, and a
      confident wrong TF is worse than no TF. Lowering the cutoff costs here.

    A probe of an uncovered place is not a broken question. Refusing it is
    the right answer, and it is the only evidence that the cutoff rejects
    anything at all - drop those probes and a config that accepts
    everything looks flawless.

    ``error`` is the median displacement of the hits, since the rates alone
    cannot tell a 2 cm fix from a 45 cm one under a 50 cm tolerance.

    ``latency`` is the whole wait, over every probe: from the first scan to
    the moment there is either a fix or a refusal. It is measured across
    retries, so it prices the real tradeoff - a slower matcher gets fewer
    attempts in the same time but each sees more scans, and a stricter
    cutoff buys safety by spending attempts. Nothing has to schedule the
    frame count against the compute; the clock does it.

    ``cpu`` is last, and is not what ``latency`` already says: Open3D threads
    its FPFH and RANSAC, so a config can be quick on the clock while eating
    every core, and the ratio between the two moves with the parameters. On
    a robot the cores are shared, so the compute is a real price even when
    the wait is short. Five objectives make for a wide Pareto front - read
    it in the order the values come in.
    """
    if not probes:
        raise ValueError("summarize() needs at least one probe")
    latency = float(np.median([p.latency_s for p in probes]))
    answerable = sum(1 for p in probes if p.answerable)
    hits = [p for p in probes if p.outcome == "hit"]
    false_fixes = sum(1 for p in probes if p.outcome == "FALSE FIX")
    error = float(np.median([p.displacement_m for p in hits])) if hits else NO_FIX_M
    return (
        len(hits) / answerable if answerable else 0.0,
        false_fixes / len(probes),
        error,
        latency,
        float(np.median([p.cpu_s for p in probes])),
    )


def view(name: str, probes: list[Probe], out: str | None) -> None:
    """Premap in height colours; each probe red where relocalize put it, green where it belongs."""
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init

    premap, _ = fixtures(name)
    rerun_init("dimos relocalize eval")
    if out is not None:
        rr.save(out)
    else:
        rr.spawn()
    rr.log("world/premap/pointcloud", premap.to_rerun(voxel_size=0.05), static=True)
    for probe in probes:
        aligned = PointCloud2.from_numpy(
            applied(probe.transform, probe.cloud),
            frame_id=probe.cloud.frame_id,
            timestamp=probe.cloud.ts,
        )
        entity = f"world/probe_{probe.start:04.0f}s"
        rr.log(
            f"{entity}/truth",
            probe.cloud.to_rerun(voxel_size=0.05, colors=[76, 220, 41]),
            static=True,
        )
        rr.log(
            f"{entity}/aligned",
            aligned.to_rerun(voxel_size=0.05, colors=[231, 76, 60]),
            static=True,
        )
    if out is not None:
        print(f"wrote {out}")


app = typer.Typer(help="Relocalization eval and tuning over a recording plus its premap")


def _register(
    dataset: str,
    recording: str | None,
    premap: str | None,
    lidar: str | None,
    window_from: float | None,
    window_to: float | None,
) -> str:
    """Resolve a dataset name, overriding the registry entry with any explicit parts."""
    base = DATASETS.get(dataset)
    if base is None and not (recording and premap):
        raise typer.BadParameter(
            f"unknown dataset {dataset!r}; known: {', '.join(DATASETS)}. "
            "Pass --recording and --premap to use one that is not registered."
        )
    base = base or Dataset(recording=recording or "", premap=premap or "")
    window = base.window
    if window_from is not None or window_to is not None:
        lo = window_from if window_from is not None else (window[0] if window else 0.0)
        # No upper bound given and none registered: `probe_starts` clamps to
        # the recording, so ask for everything rather than inventing an end.
        hi = window_to if window_to is not None else (window[1] if window else float("inf"))
        window = (lo, hi)
    DATASETS[dataset] = base._replace(
        recording=recording or base.recording,
        premap=premap or base.premap,
        lidar_stream=lidar or base.lidar_stream,
        window=window,
    )
    return dataset


DatasetOpt = typer.Option(DEFAULT_DATASET, "--dataset", "-d", help="Registered dataset name")
RecordingOpt = typer.Option(None, "--recording", help="Recording, resolved through get_data")
PremapOpt = typer.Option(None, "--premap", help="Premap .pc2.lcm, resolved through get_data")
LidarOpt = typer.Option(None, "--lidar", help="Lidar stream in the recording")
FromOpt = typer.Option(None, "--from", help="Earliest second probes may come from")
ToOpt = typer.Option(None, "--to", help="Latest second probes may come from")


@app.command()
def run(
    samples: int = typer.Option(10, "--samples", "-s", min=1, help="Probes over the window"),
    voxel: float = typer.Option(0.1, "--voxel", help="Local-map voxel size (m)"),
    preset: str = typer.Option(DEFAULT_PRESET, "--preset", help=f"One of {sorted(PRESETS)}"),
    cutoff: float | None = typer.Option(
        None, "--cutoff", help="Override the preset's fitness_threshold"
    ),
    min_frames: int | None = typer.Option(
        None, "--min-frames", help="Override the preset's scans before the first try"
    ),
    max_frames: int | None = typer.Option(
        None, "--max-frames", help="Override the preset's scans after which it gives up"
    ),
    dataset: str = DatasetOpt,
    recording: str | None = RecordingOpt,
    premap: str | None = PremapOpt,
    lidar: str | None = LidarOpt,
    start_s: float | None = FromOpt,
    stop_s: float | None = ToOpt,
    view_result: bool = typer.Option(False, "--view", help="Show premap + local maps in rerun"),
    out: str | None = typer.Option(None, "--out", help="Write a .rrd instead of opening rerun"),
) -> None:
    """Relocalize N accumulated scans against the premap, from starts across the window."""
    name = _register(dataset, recording, premap, lidar, start_s, stop_s)
    pre, _ = fixtures(name)
    if preset not in PRESETS:
        raise typer.BadParameter(f"unknown preset {preset!r}; known: {', '.join(sorted(PRESETS))}")
    config = PRESETS[preset]
    overrides = {
        "fitness_threshold": cutoff,
        "min_frames": min_frames,
        "max_frames": max_frames,
    }
    config = config.model_copy(update={k: v for k, v in overrides.items() if v is not None})
    from rich.console import Console
    from rich.live import Live

    console = Console()
    table = _table(
        f"{name}  premap {len(pre):,} pts  preset={preset}  "
        f"cutoff={config.fitness_threshold}  "
        f"frames {config.min_frames}-{config.max_frames}",
        *PROBE_COLUMNS,
    )
    # Live so the rows appear as they are measured - a probe takes seconds,
    # and a run of thirty should not look hung.
    with Live(table, console=console, refresh_per_second=4):
        probes = run_probes(
            name,
            samples=samples,
            voxel=voxel,
            config=config,
            on_probe=lambda probe: table.add_row(*probe_row(probe)),
        )

    hit_rate, false_rate, error, latency, cpu = summarize(probes)
    tally: dict[str, int] = {}
    for probe in probes:
        tally[probe.outcome] = tally.get(probe.outcome, 0) + 1
    in_map = sum(1 for p in probes if p.answerable)
    suspect = sum(1 for p in probes if p.truth_suspect)
    accuracy = f"{error:.3f} m off when hit" if error < NO_FIX_M else "never found its place"
    console.print()
    console.print(
        f"[bold]{hit_rate:.0%}[/] of {in_map} in-map probes hit, "
        f"[{'red' if false_rate else 'green'}]{false_rate:.0%} false fixes[/] "
        f"of {len(probes)} total "
        f"({', '.join(f'{n} {k}' for k, n in sorted(tally.items()))}), "
        f"{latency:.2f}s latency / {cpu:.1f}s cpu; {accuracy}"
        + (f"; {suspect} miss(es) fit the premap better than ground truth" if suspect else "")
    )
    if view_result or out is not None:
        view(name, probes, out)


def objective(
    trial: Any, *, name: str, samples: int, voxel: float
) -> tuple[float, float, float, float, float]:
    """One trial: suggest a config, probe with it, return ``(good, bad, error, seconds)``.

    The knobs are declared here by calling ``suggest_*`` - that is the search
    space, and why a conditional knob costs nothing to add.
    """
    base = PRESETS[DEFAULT_PRESET]
    config = RelocalizeConfig(
        voxel_coarse=trial.suggest_float("voxel_coarse", 0.2, 2.0, log=True),
        voxel_fine=trial.suggest_float("voxel_fine", 0.1, 1.0, log=True),
        normal_radius_factor=trial.suggest_float("normal_radius_factor", 1.5, 4.0),
        fpfh_radius_factor=trial.suggest_float("fpfh_radius_factor", 3.0, 8.0),
        coarse_dist_factor=trial.suggest_float("coarse_dist_factor", 1.0, 3.0),
        ransac_iters=trial.suggest_int("ransac_iters", 50_000, 2_000_000, log=True),
        mutual_filter=trial.suggest_categorical("mutual_filter", [True, False]),
        edge_length=trial.suggest_float("edge_length", 0.7, 0.99),
        icp_dist_factor=trial.suggest_float("icp_dist_factor", 0.2, 1.5),
        icp_stages=trial.suggest_int("icp_stages", 1, 4),
        orient_normals=trial.suggest_categorical("orient_normals", [True, False]),
        ransac_restarts=trial.suggest_int("ransac_restarts", 1, 3),
        # The publish gate is tuned alongside the aligner - it is what turns
        # a fitness number into a decision, and it is a field of the same config.
        fitness_threshold=trial.suggest_float("fitness_threshold", 0.0, 0.9),
        # These belong in the search space - the window trades hit rate
        # against latency exactly like the knobs above do. Taking them from
        # the preset instead is a shortcut, and the preset's values are
        # hand-picked. suggest_int them and bump SPACE when it is worth the
        # trials.
        min_frames=base.min_frames,
        max_frames=base.max_frames,
    )
    probes = run_probes(name, samples=samples, voxel=voxel, config=config)
    hit_rate, false_rate, error, latency, cpu = summarize(probes)
    trial.set_user_attr("median_attempts", float(np.median([p.attempts for p in probes])))
    print(
        f"  trial {trial.number}: {hit_rate:.0%} hit, {false_rate:.0%} false, "
        f"{error:.3f} m, {latency:.2f}s latency / {cpu:.1f}s cpu"
    )
    return hit_rate, false_rate, error, latency, cpu


@app.command()
def tune(
    trials: int = typer.Option(50, "--trials", "-t", help="Optuna trials to run"),
    samples: int = typer.Option(10, "--samples", "-s", min=1, help="Probes per trial"),
    voxel: float = typer.Option(0.1, "--voxel", help="Local-map voxel size (m)"),
    dataset: str = DatasetOpt,
    recording: str | None = RecordingOpt,
    premap: str | None = PremapOpt,
    lidar: str | None = LidarOpt,
    start_s: float | None = FromOpt,
    stop_s: float | None = ToOpt,
) -> None:
    """Search RelocalizeConfig for the hit / false-fix / accuracy / speed tradeoff."""
    from functools import partial

    import optuna

    name = _register(dataset, recording, premap, lidar, start_s, stop_s)
    # Studies resume by name (load_if_exists), so the name carries what a
    # trial's numbers mean: the dataset and probes they were measured over,
    # and the version of the space and objective below. Widening the space or
    # changing the score bumps SPACE rather than silently mixing incomparable
    # trials into one front. Seeded sampler so a rerun repeats.
    s = optuna.create_study(
        study_name=f"{name}-v{SPACE}-s{samples}",
        directions=["maximize", "minimize", "minimize", "minimize", "minimize"],
        storage=STORAGE,
        sampler=optuna.samplers.TPESampler(seed=0),
        load_if_exists=True,
    )
    # Named, so the dashboard and best_trials read as hit_rate/latency_s
    # rather than an unlabelled list of floats.
    s.set_metric_names(["hit_rate", "false_fix", "error_m", "latency_s", "cpu_s"])
    s.optimize(partial(objective, name=name, samples=samples, voxel=voxel), n_trials=trials)
    # The front is unordered; read it correctness-first, speed as the tiebreak.
    from rich.console import Console

    table = _table(
        f"{len(s.best_trials)} trials on the Pareto front",
        "trial",
        "hit",
        "false",
        "err m",
        "lat",
        "cpu",
    )
    front = sorted(s.best_trials, key=lambda t: (-t.values[0], *t.values[1:]))
    for t in front:
        table.add_row(
            f"t{t.number}",
            f"[{'green' if t.values[0] == 1.0 else 'yellow'}]{t.values[0]:.0%}[/]",
            f"[{'red' if t.values[1] else 'green'}]{t.values[1]:.0%}[/]",
            f"{t.values[2]:.3f}",
            f"{t.values[3]:.2f}",
            f"{t.values[4]:.1f}",
        )
    console = Console()
    console.print(table)
    # Params below rather than in a column: they are twelve fields wide and
    # would squeeze every number in the table down to nothing.
    for t in front:
        params = ", ".join(
            f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in t.params.items()
        )
        console.print(f"  [bold]t{t.number}[/]  {params}", highlight=False)


def config_from_params(params: dict[str, Any]) -> RelocalizeConfig:
    """Rebuild a trial's config from the params optuna recorded.

    Anything the study did not search (the retry window) comes from the
    default preset, which is where it was set by hand.
    """
    fields = set(RelocalizeConfig.model_fields)
    searched = {k: v for k, v in params.items() if k in fields}
    return PRESETS[DEFAULT_PRESET].model_copy(update=searched)


@app.command()
def verify(
    study_name: str = typer.Option(..., "--study", help="Study whose front to re-measure"),
    top: int = typer.Option(8, "--top", help="Candidates to verify, cheapest latency first"),
    repeats: int = typer.Option(10, "--repeats", "-r", help="Re-runs per candidate per probe set"),
    samples: int = typer.Option(10, "--samples", "-s", min=1, help="Probes per run"),
    voxel: float = typer.Option(0.1, "--voxel", help="Local-map voxel size (m)"),
    min_hit: float = typer.Option(1.0, "--min-hit", help="Hit rate a trial needed to qualify"),
    dataset: str = DatasetOpt,
    recording: str | None = RecordingOpt,
    premap: str | None = PremapOpt,
    lidar: str | None = LidarOpt,
    start_s: float | None = FromOpt,
    stop_s: float | None = ToOpt,
) -> None:
    """Re-measure a study's best configs, on its own probes and on fresh ones.

    A trial's score is one draw of ``samples`` probes against an unseeded
    RANSAC, so the trials that top a front are partly the lucky ones. This
    runs each candidate ``repeats`` times to get a rate worth trusting, and
    again on the half-step holdout, where a config that merely learned which
    handful of places are easy will fall over.
    """
    import optuna

    name = _register(dataset, recording, premap, lidar, start_s, stop_s)
    study = optuna.load_study(study_name=study_name, storage=STORAGE)
    clean = [t for t in study.trials if t.values and t.values[0] >= min_hit and t.values[1] == 0.0]
    if not clean:
        raise typer.BadParameter(
            f"{study_name}: no trial reached {min_hit:.0%} hit with no false fixes"
        )
    clean.sort(key=lambda t: t.values[3])
    from rich.console import Console
    from rich.live import Live

    console = Console()
    table = _table(
        f"{study_name}: {min(top, len(clean))} of {len(clean)} clean trials, {repeats}x each",
        "trial",
        "<probes",
        "hit",
        "false",
        "err m",
        "lat",
        "frames",
        "ornt",
        "stg",
        "rst",
    )
    # A candidate is two rows and many seconds; show them as they land.
    with Live(table, console=console, refresh_per_second=4):
        for trial in clean[:top]:
            config = config_from_params(trial.params)
            for label, half in (("tuned", False), ("holdout", True)):
                runs = [
                    summarize(
                        run_probes(
                            name,
                            samples=samples,
                            voxel=voxel,
                            config=config,
                            half_step=half,
                        )
                    )
                    for _ in range(repeats)
                ]
                hit = [r[0] for r in runs]
                false = [r[1] for r in runs]
                err = [r[2] for r in runs if r[2] < NO_FIX_M]
                params = trial.params
                table.add_row(
                    f"t{trial.number}",
                    label,
                    f"[{'green' if np.mean(hit) == 1.0 else 'yellow'}]"
                    f"{np.mean(hit):.0%}±{np.std(hit):.0%}[/]",
                    f"[{'red' if np.mean(false) else 'green'}]"
                    f"{np.mean(false):.0%}±{np.std(false):.0%}[/]",
                    f"{(np.mean(err) if err else float('nan')):.3f}",
                    f"{np.mean([r[3] for r in runs]):.2f}",
                    str(config.max_frames),
                    str(params["orient_normals"])[0],
                    str(params["icp_stages"]),
                    str(params["ransac_restarts"]),
                    end_section=half,
                )


if __name__ == "__main__":
    app()
