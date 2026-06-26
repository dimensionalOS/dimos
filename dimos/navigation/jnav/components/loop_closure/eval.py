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

"""Evaluate a loop-closure module against a recording.

Two ground-truth-free scores, before vs after correction:
  * April-tag agreement — a fixed tag re-seen along the run should map to one
    world position; the spread of its per-visit robot positions measures drift.
    Tags are taken as relative to the chosen odom stream (sighting time ->
    nearest odom pose), so no static transforms or stored db poses are needed.
  * Lidar-voxel agreement — re-anchoring the registered scans onto the
    corrected trajectory should collapse double walls, so the corrected map
    should occupy FEWER voxels than the raw one.

Pipeline:
  1. April tags: read the db's `april_tags` stream (ts + marker_id only), or
     detect them with sane defaults (medoid, blur/reproj/size/distance gates).
  2. Raw agreement over the raw odometry.
  3. Replay lidar + odom through the module (loaded dynamically from
     --module-path/--module-name), capture its optimized pose graph.
  4. Corrected agreement + voxel agreement, written to
     eval_results/<recording>__<module>/summary.json (and an eval.rrd with the
     raw + corrected trajectories when --with-rrd true).

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/eval.py \\
        --db-path ~/datasets/go2_recordings/2026-06-04_12-56pm-PST/mem2.db \\
        --odom-stream fastlio_odometry \\
        --camera-stream color_image \\
        --camera-intrinsics-json-path \\
            ~/datasets/go2_recordings/2026-06-04_12-56pm-PST/camera_intrinsics.json \\
        --module-path dimos/navigation/jnav/components/loop_closure/gsc_pgo/module.py \\
        --module-name PGO \\
        --pgo-config-json '{"use_scan_context": true}' \\
        --with-rrd true
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import AsyncGenerator, Iterable
import importlib
import json
from pathlib import Path
import tempfile
import time
from typing import Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.jnav.utils.apriltag_agreement import (
    VISIT_GAP_S,
    AgreementReport,
    agreement_improvement,
    agreement_report,
    paired_tag_visit_positions,
)
from dimos.navigation.jnav.utils.apriltags import (
    detect_apriltags,
    load_intrinsics_json,
    load_or_detect_sightings,
)
from dimos.navigation.jnav.utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.jnav.utils.recording_db import (
    MAX_REPLAY_ODOM,
    MAX_REPLAY_SCANS,
    ODOM_MATCH_TOLERANCE_S,
    REPLAY_DRAIN_MARGIN_S,
    REPLAY_PUBLISH_HZ,
    iterate_stream,
    list_streams,
    odometry_lookup,
    store,
    stream_count,
)
from dimos.navigation.jnav.utils.trajectory_metrics import (
    GraphPose,
    PoseLookup7,
    drifted_lookup,
    graph_lookup,
    has_drift,
    lidar_voxel_agreement,
    pose7_lookup,
    trajectory_recovery_error,
)

RESULTS_DIR = Path(__file__).resolve().parent / "eval_results"
APRIL_TAGS_STREAM = "april_tags"
_RRD_MAX_PATH_POINTS = 5000

# Cap replayed scans fed to voxel agreement so the map fits in memory.
VOXEL_MAX_SCANS = 300

# Bump to invalidate every cached cell (scoring/replay semantics changed).
EVAL_VERSION = 1


def cell_fingerprint(
    db_path: Path,
    pgo_config: dict[str, Any],
    lidar_stream: str,
    odom_stream: str,
    drift_per_sec: list[float] | None = None,
) -> dict[str, Any]:
    """Identity of a completed cell — the driver re-runs only when this changes
    (db edited, config changed, streams changed, drift changed, or version)."""
    stat = db_path.stat()
    return {
        "db_bytes": stat.st_size,
        "db_mtime": int(stat.st_mtime),
        "pgo_config": pgo_config,
        "lidar_stream": lidar_stream,
        "odom_stream": odom_stream,
        "drift_per_sec": list(drift_per_sec or [0.0, 0.0, 0.0]),
        "version": EVAL_VERSION,
    }


# A known-good PGO config for replayed recordings: revisit gates loose enough
# for walks where the same spot is re-seen tens of seconds later. These now
# match gsc_pgo's own defaults (so it's a no-op for gsc_pgo); kept here to apply
# the same gates to the other PGO modules, which have different defaults.
DEFAULT_PGO_CONFIG: dict[str, Any] = {
    "loop_search_radius": 3.0,
    "loop_time_thresh": 5.0,
    "min_loop_detect_duration": 2.0,
    "key_pose_delta_trans": 0.5,
    "use_scan_context": True,
}


class GraphCaptureConfig(ModuleConfig):
    output_path: str = ""


class GraphCapture(Module):
    """Captures the module's optimized pose graph WITH orientations + closures.

    Results are handed back via a JSON file written on teardown (modules run in
    separate worker processes)."""

    config: GraphCaptureConfig

    pose_graph: In[Graph3D]
    loop_closure_event: In[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._graph: list[GraphPose] = []
        self._closures = 0

    async def handle_pose_graph(self, msg: Graph3D) -> None:
        self._graph = [
            (
                node.pose.ts,
                node.pose.position.x,
                node.pose.position.y,
                node.pose.position.z,
                node.pose.orientation.x,
                node.pose.orientation.y,
                node.pose.orientation.z,
                node.pose.orientation.w,
            )
            for node in msg.nodes
        ]

    async def handle_loop_closure_event(self, msg: GraphDelta3D) -> None:
        self._closures += 1

    async def main(self) -> AsyncGenerator[None, None]:
        yield
        Path(self.config.output_path).write_text(
            json.dumps({"graph": self._graph, "closures": self._closures})
        )


class LockstepReplayConfig(ModuleConfig):
    db: str = ""
    lidar_stream: str = "lidar"
    odometry_stream: str = "odom"
    lidar_stride: int = 1
    odometry_stride: int = 2
    odom_publish_hz: float = 500.0
    ack_timeout_s: float = 30.0
    done_path: str = ""
    # Artificial odometry drift: a constant-velocity world offset added to both
    # odom poses and lidar clouds at time t (offset = drift_per_sec * (t - t0)).
    # Consistent per-instant, so the trajectory warps over time — exactly the
    # accumulating error loop closure is supposed to fix. [0,0,0] = no drift.
    drift_per_sec: list[float] = [0.0, 0.0, 0.0]
    drift_t0: float = 0.0


class LockstepReplay(Module):
    """Closed-loop replay: after each scan, wait for the module's
    corrected_odometry ack before sending the next.

    Every module under test sees 100% of the (strided) scans regardless of
    machine speed — wall clock varies, the data the module processes doesn't.
    Odometry messages are cheap latest-state updates and stay fire-and-forget
    (lightly paced). Writes a done-marker JSON (ack timeout count) at the end
    so the host knows when to tear down.

    odom and lidar are merged into one time-sorted stream, so playback runs in
    bursts: all odoms whose timestamps fall before the next scan are emitted
    fire-and-forget (paced by odom_publish_hz), then one scan is sent and the
    loop blocks on its ack. The only guarantee is one ack-wait per scan; the
    odom burst size per gap is data-dependent (~ odom_rate / lidar_rate)."""

    config: LockstepReplayConfig

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    corrected_odometry: In[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._ack_count = 0
        self._ack_event: asyncio.Event | None = None

    async def handle_corrected_odometry(self, msg: Odometry) -> None:
        self._ack_count += 1
        if self._ack_event is not None:
            self._ack_event.set()

    def _load(self) -> list[tuple[float, str, Any]]:
        db_path = Path(self.config.db)
        merged: list[tuple[float, str, Any]] = []
        for timestamp, pose in iterate_stream(
            db_path, self.config.odometry_stream, stride=self.config.odometry_stride
        ):
            merged.append((timestamp, "odom", pose))
        for timestamp, cloud in iterate_stream(
            db_path, self.config.lidar_stream, stride=self.config.lidar_stride
        ):
            merged.append((timestamp, "lidar", cloud))
        merged.sort(key=lambda item: item[0])
        return merged

    async def main(self) -> AsyncGenerator[None, None]:
        messages = await asyncio.to_thread(self._load)
        self._task = asyncio.create_task(self._replay(messages))
        yield
        self._task.cancel()

    async def _replay(self, messages: list[tuple[float, str, Any]]) -> None:
        odom_period = 1.0 / self.config.odom_publish_hz
        timeouts = 0
        scans_sent = 0
        drift = np.asarray(self.config.drift_per_sec, dtype=np.float64)
        t0 = self.config.drift_t0
        apply_drift = has_drift(drift)
        # Timestamps of scans the module never acked — the frames it (likely)
        # skipped. Recorded for reproducibility of partial runs.
        skipped_scan_ts: list[float] = []
        for timestamp, kind, payload in messages:
            if kind == "odom":
                pose = RateReplay._payload_pose(payload)
                if apply_drift:
                    offset = drift * (timestamp - t0)
                    pose = Pose(
                        position=[
                            pose.position.x + offset[0],
                            pose.position.y + offset[1],
                            pose.position.z + offset[2],
                        ],
                        orientation=[
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ],
                    )
                self.odometry.publish(
                    Odometry(
                        ts=timestamp,
                        frame_id="map",
                        child_frame_id="base_link",
                        pose=pose,
                    )
                )
                await asyncio.sleep(odom_period)
                continue

            acks_before = self._ack_count
            self._ack_event = asyncio.Event()
            points = payload.points_f32()
            if apply_drift:
                points = points + (drift * (timestamp - t0)).astype(np.float32)
            self.lidar.publish(PointCloud2.from_numpy(points, frame_id="map", timestamp=timestamp))
            scans_sent += 1
            deadline = time.monotonic() + self.config.ack_timeout_s
            while self._ack_count == acks_before:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    timeouts += 1
                    skipped_scan_ts.append(timestamp)
                    break
                try:
                    await asyncio.wait_for(self._ack_event.wait(), timeout=remaining)
                except TimeoutError:
                    continue
                self._ack_event.clear()
            if scans_sent % _PROGRESS_EVERY_N_SCANS == 0:
                # Periodic progress so a capped run still reports coverage.
                Path(self.config.done_path + ".progress").write_text(
                    json.dumps(self._stats(timeouts, scans_sent, skipped_scan_ts))
                )

        Path(self.config.done_path).write_text(
            json.dumps(self._stats(timeouts, scans_sent, skipped_scan_ts))
        )

    @staticmethod
    def _stats(timeouts: int, scans_sent: int, skipped_scan_ts: list[float]) -> dict[str, Any]:
        return {
            "timeouts": timeouts,
            "scans_sent": scans_sent,
            "skipped_scan_ts": skipped_scan_ts,
        }


class RateReplayConfig(ModuleConfig):
    db: str = ""
    lidar_stream: str = "lidar"
    odometry_stream: str = "odom"
    lidar_stride: int = 1
    odometry_stride: int = 2
    publish_hz: float = 40.0


class RateReplay(Module):
    """Legacy fixed-rate replay: publishes world-frame lidar + odometry at a set
    Hz with timestamps preserved (no ack pacing — wall-clock dependent).

    Works for both odometry payload shapes found in recordings: ``Odometry``
    (go2 ``fastlio_odometry``) and ``PoseStamped`` (hk_village ``odom``).
    """

    config: RateReplayConfig

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.done = False

    def _load(self) -> list[tuple[float, str, Any]]:
        db_path = Path(self.config.db)
        merged: list[tuple[float, str, Any]] = []
        for timestamp, pose in iterate_stream(
            db_path, self.config.odometry_stream, stride=self.config.odometry_stride
        ):
            merged.append((timestamp, "odom", pose))
        for timestamp, cloud in iterate_stream(
            db_path, self.config.lidar_stream, stride=self.config.lidar_stride
        ):
            merged.append((timestamp, "lidar", cloud))
        merged.sort(key=lambda item: item[0])
        return merged

    @staticmethod
    def _payload_pose(payload: Any) -> Pose:
        if hasattr(payload, "pose"):  # Odometry
            return payload.pose  # type: ignore[no-any-return]
        return Pose(  # PoseStamped
            payload.x,
            payload.y,
            payload.z,
            payload.orientation.x,
            payload.orientation.y,
            payload.orientation.z,
            payload.orientation.w,
        )

    async def main(self) -> AsyncGenerator[None, None]:
        messages = await asyncio.to_thread(self._load)
        self._task = asyncio.create_task(self._replay(messages))
        yield
        self._task.cancel()

    async def _replay(self, messages: list[tuple[float, str, Any]]) -> None:
        period = 1.0 / self.config.publish_hz
        for timestamp, kind, payload in messages:
            if kind == "odom":
                self.odometry.publish(
                    Odometry(
                        ts=timestamp,
                        frame_id="map",
                        child_frame_id="base_link",
                        pose=self._payload_pose(payload),
                    )
                )
            else:
                self.lidar.publish(
                    PointCloud2.from_numpy(
                        payload.points_f32(), frame_id="map", timestamp=timestamp
                    )
                )
            await asyncio.sleep(period)
        self.done = True


# Run cap scales with the workload: a per-scan budget (well above any sane
# processing time, below the 30s ack timeout) plus fixed startup overhead.
LOCKSTEP_PER_SCAN_BUDGET_S = 2.0
LOCKSTEP_BASE_OVERHEAD_S = 120.0
LOCKSTEP_POLL_S = 5.0
LOCKSTEP_DRAIN_S = 10.0
_PROGRESS_EVERY_N_SCANS = 200


def run_module_graph(
    db_path: Path,
    module_class: type,
    config_overrides: dict[str, Any],
    *,
    lidar_stream: str,
    odom_stream: str,
    lockstep: bool = True,
    drift_per_sec: list[float] | None = None,
    drift_t0: float = 0.0,
) -> tuple[list[GraphPose], int, dict[str, Any]]:
    """Replay the recording through the module; return its optimized pose graph
    (with orientations), loop-closure count, and replay stats.

    lockstep=True (default) paces scans on the module's corrected_odometry
    acks — machine-speed independent. lockstep=False is the legacy fixed-rate
    wall-clock replay. drift_per_sec injects a constant-velocity world offset
    into the replayed odom+lidar (see LockstepReplayConfig)."""
    drift_per_sec = drift_per_sec or [0.0, 0.0, 0.0]
    output_path = Path(tempfile.gettempdir()) / f"jnav_lc_eval_{db_path.parent.name}.json"
    output_path.unlink(missing_ok=True)
    done_path = Path(tempfile.gettempdir()) / f"jnav_lc_eval_done_{db_path.parent.name}.json"
    done_path.unlink(missing_ok=True)
    Path(str(done_path) + ".progress").unlink(missing_ok=True)
    lidar_stride = max(1, -(-stream_count(db_path, lidar_stream) // MAX_REPLAY_SCANS))
    odometry_stride = max(1, -(-stream_count(db_path, odom_stream) // MAX_REPLAY_ODOM))
    n_messages = stream_count(db_path, odom_stream) // odometry_stride
    n_messages += stream_count(db_path, lidar_stream) // lidar_stride

    if lockstep:
        replay_blueprint = LockstepReplay.blueprint(
            db=str(db_path),
            lidar_stream=lidar_stream,
            odometry_stream=odom_stream,
            lidar_stride=lidar_stride,
            odometry_stride=odometry_stride,
            done_path=str(done_path),
            drift_per_sec=drift_per_sec,
            drift_t0=drift_t0,
        )
    else:
        replay_blueprint = RateReplay.blueprint(
            db=str(db_path),
            lidar_stream=lidar_stream,
            odometry_stream=odom_stream,
            lidar_stride=lidar_stride,
            odometry_stride=odometry_stride,
            publish_hz=REPLAY_PUBLISH_HZ,
        )

    blueprint = autoconnect(
        replay_blueprint,
        module_class.blueprint(**config_overrides),  # type: ignore[attr-defined]
        GraphCapture.blueprint(output_path=str(output_path)),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    mode = "lockstep" if lockstep else f"fixed-rate {REPLAY_PUBLISH_HZ}Hz"
    print(
        f"replaying {n_messages} messages through {module_class.__name__}"
        f" ({mode}, lidar stride {lidar_stride}, odom stride {odometry_stride})"
    )
    replay_stats: dict[str, Any] = {"mode": mode}
    try:
        if lockstep:
            # Per-frame budget: the cap scales with how many scans are fed.
            n_scans = stream_count(db_path, lidar_stream) // lidar_stride
            max_run_s = n_scans * LOCKSTEP_PER_SCAN_BUDGET_S + LOCKSTEP_BASE_OVERHEAD_S
            started = time.monotonic()
            while not done_path.exists():
                elapsed = time.monotonic() - started
                if elapsed > max_run_s:
                    replay_stats["hit_max_run_s"] = max_run_s
                    print(
                        f"lockstep replay hit the per-frame cap"
                        f" ({n_scans} scans x {LOCKSTEP_PER_SCAN_BUDGET_S}s"
                        f" + {LOCKSTEP_BASE_OVERHEAD_S}s = {round(max_run_s)}s) — stopping early"
                    )
                    break
                if int(elapsed) % 60 < LOCKSTEP_POLL_S and elapsed > LOCKSTEP_POLL_S:
                    print(f"  ... lockstep replay running ({round(elapsed)}s)")
                time.sleep(LOCKSTEP_POLL_S)
            progress_path = Path(str(done_path) + ".progress")
            if done_path.exists():
                replay_stats.update(json.loads(done_path.read_text()))
            elif progress_path.exists():
                # Capped run: last periodic progress still tells us coverage
                # and which frames the module never acked.
                replay_stats.update(json.loads(progress_path.read_text()))
                replay_stats["partial"] = True
            progress_path.unlink(missing_ok=True)
            time.sleep(LOCKSTEP_DRAIN_S)
        else:
            time.sleep(n_messages / REPLAY_PUBLISH_HZ + REPLAY_DRAIN_MARGIN_S)
    finally:
        coordinator.stop()

    if not output_path.exists():
        raise SystemExit(f"{module_class.__name__} produced no pose graph output")
    data = json.loads(output_path.read_text())
    graph = [tuple(row) for row in data["graph"]]
    return graph, int(data["closures"]), replay_stats  # type: ignore[return-value]


def odometry_pose7_lookup(db_path: Path, odom_stream: str) -> PoseLookup7:
    times: list[float] = []
    poses: list[list[float]] = []
    for timestamp, payload in iterate_stream(db_path, odom_stream):
        pose = RateReplay._payload_pose(payload)
        times.append(timestamp)
        poses.append(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    return pose7_lookup(
        np.asarray(times, dtype=np.float64),
        np.asarray(poses, dtype=np.float64),
        ODOM_MATCH_TOLERANCE_S,
    )


def _subsampled_path(positions: np.ndarray) -> np.ndarray:
    stride = max(1, len(positions) // _RRD_MAX_PATH_POINTS)
    return positions[::stride]


def write_trajectory_rrd(rrd_path: Path, raw_positions: np.ndarray, graph: list[GraphPose]) -> None:
    """Raw + corrected trajectories (no tag rendering — tags are scored as
    odom-relative, not placed in 3D)."""
    import rerun as rr

    rr.init("jnav_loop_closure_eval", spawn=False)
    rr.save(str(rrd_path))
    rr.log(
        "trajectory/raw_odom",
        rr.LineStrips3D([_subsampled_path(raw_positions)], colors=[[200, 90, 90]]),
        static=True,
    )
    corrected = np.asarray([[node[1], node[2], node[3]] for node in graph], dtype=np.float64)
    rr.log(
        "trajectory/corrected",
        rr.LineStrips3D([_subsampled_path(corrected)], colors=[[90, 200, 120]]),
        static=True,
    )


def _report_dict(report: AgreementReport) -> dict[str, Any]:
    return {
        "mean_spread_m": report.mean_spread,
        "total_observations": report.total_observations,
        "per_tag": [
            {"tag_id": tag.tag_id, "observations": tag.observations, "spread_m": tag.spread}
            for tag in report.per_tag
        ],
    }


def evaluate(
    db_path: Path,
    *,
    odom_stream: str,
    camera_stream: str | None,
    intrinsics_json: Path | None,
    module_path: Path,
    module_name: str,
    pgo_config: dict[str, Any],
    with_rrd: bool,
    lidar_stream: str,
    lockstep: bool = True,
    results_suffix: str = "",
    recording_name: str | None = None,
    drift_per_sec: list[float] | None = None,
    ignore_tags: set[int] | None = None,
) -> dict[str, Any]:
    streams = list_streams(db_path)
    for required in (odom_stream, lidar_stream):
        if required not in streams:
            raise SystemExit(f"no stream {required!r} in {db_path} (have: {streams})")

    module_class = load_module_class(module_path, module_name)
    pgo_config = filter_config_for_module(module_class, pgo_config)

    # Artificial drift: the module is fed odom+lidar with a constant-velocity
    # world offset added at each time; the raw-baseline scoring must apply the
    # SAME offset so it compares against what the module actually saw.
    drift_per_sec = drift_per_sec or [0.0, 0.0, 0.0]
    drift_t0 = next(iterate_stream(db_path, odom_stream))[0] if has_drift(drift_per_sec) else 0.0

    # April-tag agreement needs a camera + intrinsics; voxel agreement does not.
    # Datasets without either (kitti-360, bare lidar recordings) still score on
    # voxel agreement alone, so the same harness fills every table cell.
    sightings: dict[int, list[float]] = {}
    tag_source = "none"
    have_camera = camera_stream is not None and camera_stream in streams
    if have_camera and intrinsics_json is not None and intrinsics_json.exists():
        assert camera_stream is not None  # narrowed by have_camera
        camera = camera_stream
        intrinsics_config = load_intrinsics_json(intrinsics_json)
        db_store = store(db_path)
        stored_stream: Any = (
            db_store.stream(APRIL_TAGS_STREAM)
            if APRIL_TAGS_STREAM in db_store.list_streams()
            else []
        )
        stored = ((int(obs.tags["marker_id"]), float(obs.ts)) for obs in stored_stream)

        def detect() -> Iterable[tuple[int, float]]:
            detections = detect_apriltags(
                db_store,
                intrinsics_config["intrinsics"],
                intrinsics_config["distortion"],
                image_stream=camera,
                stream_name=APRIL_TAGS_STREAM,
                marker_length=intrinsics_config.get("marker_length", 0.10),
                dictionary=intrinsics_config.get("dictionary", "DICT_APRILTAG_36h11"),
            )
            return ((int(d["marker_id"]), float(d["ts"])) for d in detections)

        sightings, tag_source = load_or_detect_sightings(stored, detect)
    # Drop dynamic/unreliable tags (e.g. a tag on a moving object) so their
    # motion isn't mistaken for trajectory drift. huge_loop_realsense tag #17 is
    # dynamic; all others are static.
    if ignore_tags:
        dropped = sorted(tag_id for tag_id in sightings if tag_id in ignore_tags)
        for tag_id in dropped:
            del sightings[tag_id]
        if dropped:
            print(f"ignoring tags {dropped} (declared dynamic/unreliable)")
    n_sightings = sum(len(times) for times in sightings.values())
    if sightings:
        print(f"april tags ({tag_source}): {n_sightings} sightings across ids {sorted(sightings)}")
    else:
        print("no April tags (camera/intrinsics absent or none detected) — voxel agreement only")

    started = time.monotonic()
    graph, closures, replay_stats = run_module_graph(
        db_path,
        module_class,
        pgo_config,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        lockstep=lockstep,
        drift_per_sec=drift_per_sec,
        drift_t0=drift_t0,
    )
    runtime_s = time.monotonic() - started
    if not graph:
        raise SystemExit(f"{module_name} produced an empty pose graph")

    # The module solved on drifted input, so its graph lives in the drifted
    # world; the raw baselines must be drifted to match (see drift_per_sec).
    raw_xyz_lookup = drifted_lookup(odometry_lookup(db_path, odom_stream), drift_per_sec, drift_t0)
    raw_pose7_lookup = drifted_lookup(
        odometry_pose7_lookup(db_path, odom_stream), drift_per_sec, drift_t0
    )

    xyz_graph = [(node[0], node[1], node[2], node[3]) for node in graph]
    if sightings:
        raw_tag_positions, corrected_tag_positions = paired_tag_visit_positions(
            sightings,
            raw_xyz_lookup,
            graph_lookup(xyz_graph),
            gap_s=VISIT_GAP_S,
        )
        raw_report = agreement_report(raw_tag_positions)
        corrected_report = agreement_report(corrected_tag_positions)
        improvement: float | None = agreement_improvement(raw_report, corrected_report)
    else:
        raw_report = agreement_report({})
        corrected_report = agreement_report({})
        improvement = None  # no tags — tag agreement is N/A for this cell

    voxel_stride = max(1, -(-stream_count(db_path, lidar_stream) // VOXEL_MAX_SCANS))
    voxel = lidar_voxel_agreement(
        (
            (timestamp, cloud.points_f32())
            for timestamp, cloud in iterate_stream(db_path, lidar_stream, stride=voxel_stride)
        ),
        raw_pose7_lookup,
        graph,
        drift_per_sec=drift_per_sec,
        drift_t0=drift_t0,
    )

    # Drift-recovery ATE: corrected trajectory vs the UN-drifted ground truth
    # (the odom before drift was injected). Only meaningful with --drift-per-sec;
    # the right metric where tag/voxel agreement is weak (e.g. KITTI's long loop).
    trajectory = trajectory_recovery_error(
        graph, odometry_lookup(db_path, odom_stream), drift_per_sec, drift_t0
    )
    if trajectory is not None:
        print(
            f"  drift recovery:    {trajectory['drifted_ate_m']:.2f}"
            f" -> {trajectory['corrected_ate_m']:.2f} m ATE"
            f" ({trajectory['trajectory_improvement']:+.3f})"
        )

    # Key by package + class — several loop-closure modules are all named PGO.
    # results_suffix (dot-joined, NOT "__" which delimits the recording name)
    # separates runs that differ in inputs, e.g. fastlio vs pointlio odometry.
    module_package = module_class.__module__.rsplit(".", 2)[-2]
    module_key = f"{module_package}.{module_name}" + (
        f".{results_suffix}" if results_suffix else ""
    )
    # db.parent.name is the recording dir for go2; LFS dbs (hk_village) sit
    # directly in data/, so an explicit recording_name avoids cell collisions.
    out_dir = RESULTS_DIR / f"{recording_name or db_path.parent.name}__{module_key}"
    out_dir.mkdir(parents=True, exist_ok=True)
    rrd_path = out_dir / "eval.rrd"
    if with_rrd:
        raw_positions = np.asarray(
            [
                [pose.position.x, pose.position.y, pose.position.z]
                for _, payload in iterate_stream(db_path, odom_stream)
                for pose in [RateReplay._payload_pose(payload)]
            ],
            dtype=np.float64,
        )
        write_trajectory_rrd(rrd_path, raw_positions, graph)

    summary = {
        "db": str(db_path),
        "odom_stream": odom_stream,
        "camera_stream": camera_stream,
        "lidar_stream": lidar_stream,
        "module": {"path": str(module_path), "name": module_name},
        "pgo_config": pgo_config,
        "drift_per_sec": list(drift_per_sec),
        "fingerprint": cell_fingerprint(
            db_path, pgo_config, lidar_stream, odom_stream, drift_per_sec
        ),
        "replay": replay_stats,
        "april_tags": {
            "source": tag_source,
            "sightings": n_sightings,
            "ids": sorted(sightings),
        },
        "scores": {
            "raw_spread_m": raw_report.mean_spread if sightings else None,
            "corrected_spread_m": corrected_report.mean_spread if sightings else None,
            "tag_improvement": improvement,
            "voxel_improvement": voxel.get("improvement"),
            "trajectory_improvement": trajectory["trajectory_improvement"] if trajectory else None,
            "drifted_ate_m": trajectory["drifted_ate_m"] if trajectory else None,
            "corrected_ate_m": trajectory["corrected_ate_m"] if trajectory else None,
            "closures": closures,
            "keyframes": len(graph),
            "runtime_s": round(runtime_s, 1),
        },
        "raw_agreement": _report_dict(raw_report),
        "corrected_agreement": _report_dict(corrected_report),
        "voxel_agreement": voxel,
        "rrd": str(rrd_path) if with_rrd else None,
        "evaluated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"\nresults -> {out_dir / 'summary.json'}")
    if sightings:
        print(
            f"  tag spread:        {raw_report.mean_spread:.3f}"
            f" -> {corrected_report.mean_spread:.3f} m"
        )
        print(f"  tag improvement:   {improvement:+.3f} (1.0 = perfect)")
    else:
        print("  tag improvement:   n/a (no tags)")
    if voxel.get("status") == "ok":
        print(
            f"  voxel agreement:   {voxel['raw_voxels']} -> {voxel['corrected_voxels']} voxels"
            f" ({voxel['improvement']:+.3f}, {voxel['scans_used']} scans @ {voxel['voxel_size_m']}m)"
        )
    else:
        print(f"  voxel agreement:   {voxel.get('status')}")
    print(f"  closures:          {closures}, keyframes: {len(graph)}")
    if with_rrd:
        print(f"  rrd:               {rrd_path}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db-path", type=Path, required=True)
    parser.add_argument("--odom-stream", required=True)
    parser.add_argument(
        "--camera-stream", default=None, help="omit for tagless datasets (voxel agreement only)"
    )
    parser.add_argument(
        "--camera-intrinsics-json-path",
        type=Path,
        default=None,
        help="omit for tagless datasets (voxel agreement only)",
    )
    parser.add_argument("--module-path", type=Path, required=True)
    parser.add_argument("--module-name", required=True)
    parser.add_argument(
        "--pgo-config-json",
        help="inline JSON of module config overrides (default: scan_context variant)",
    )
    parser.add_argument("--with-rrd", default="false", choices=["true", "false"])
    parser.add_argument(
        "--lidar-stream",
        default="fastlio_lidar",
        help="lidar stream replayed into the module alongside the odometry",
    )
    parser.add_argument(
        "--lockstep",
        default="true",
        choices=["true", "false"],
        help="pace scans on corrected_odometry acks (machine-independent); false = fixed-rate",
    )
    parser.add_argument(
        "--results-suffix",
        default="",
        help="extra results-dir key for runs with different inputs (e.g. pointlio)",
    )
    parser.add_argument(
        "--recording-name",
        default=None,
        help="results-dir recording key (default: db parent dir name)",
    )
    parser.add_argument(
        "--drift-per-sec",
        default=None,
        help="inject odom drift as a constant world velocity 'x,y,z' in m/s "
        "(offset = this * (t - t0), added to odom+lidar). e.g. '0.01,0,0'",
    )
    parser.add_argument(
        "--ignore-tags",
        default=None,
        help="comma-separated April-tag ids to drop from scoring (dynamic/unreliable "
        "tags whose motion would look like drift). e.g. '17'",
    )
    args = parser.parse_args()

    drift_per_sec = (
        [float(v) for v in args.drift_per_sec.split(",")] if args.drift_per_sec else None
    )
    if drift_per_sec is not None and len(drift_per_sec) != 3:
        raise SystemExit(f"--drift-per-sec must be 'x,y,z', got {args.drift_per_sec!r}")

    ignore_tags = (
        {int(tag_id) for tag_id in args.ignore_tags.split(",")} if args.ignore_tags else None
    )

    db_path = args.db_path.expanduser()
    if not db_path.exists():
        raise SystemExit(f"no such db: {db_path}")
    intrinsics_json = (
        args.camera_intrinsics_json_path.expanduser()
        if args.camera_intrinsics_json_path is not None
        else None
    )
    if intrinsics_json is not None and not intrinsics_json.exists():
        raise SystemExit(f"no such intrinsics json: {intrinsics_json}")

    pgo_config = dict(DEFAULT_PGO_CONFIG)
    if args.pgo_config_json:
        pgo_config.update(json.loads(args.pgo_config_json))

    evaluate(
        db_path,
        odom_stream=args.odom_stream,
        camera_stream=args.camera_stream,
        intrinsics_json=intrinsics_json,
        module_path=args.module_path,
        module_name=args.module_name,
        pgo_config=pgo_config,
        with_rrd=args.with_rrd == "true",
        lidar_stream=args.lidar_stream,
        lockstep=args.lockstep == "true",
        results_suffix=args.results_suffix,
        recording_name=args.recording_name,
        drift_per_sec=drift_per_sec,
        ignore_tags=ignore_tags,
    )


if __name__ == "__main__":
    # Re-import under the canonical dotted name so module classes defined here
    # (GraphCapture) deploy into workers with a picklable __module__.
    importlib.import_module("dimos.navigation.jnav.components.loop_closure.eval").main()
