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

"""Replay harness for scoring a loop-closure module against a recording.

``run_module_graph`` streams a recording through the module under test and hands
back its optimized pose graph. The replay itself comes in two flavors:
``LockstepReplay`` (ack-paced, machine-speed independent) and ``RateReplay``
(legacy fixed-rate). ``GraphCapture`` collects the module's output graph.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
from contextlib import suppress
from itertools import islice
import json
from pathlib import Path
import tempfile
import time
from typing import Any, TypedDict

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.jnav.utils.trajectory_metrics import GraphPose, has_drift

# Watchdog for a hung run: a generous per-scan budget (well above any sane
# processing time) plus fixed startup overhead. Hitting it raises — never a
# silent partial result.
LOCKSTEP_PER_SCAN_BUDGET_S = 2.0
LOCKSTEP_BASE_OVERHEAD_S = 120.0
LOCKSTEP_POLL_S = 5.0
# After the last scan is acked the module may still fold a background GNC
# classification into the pose graph. While that solve is in flight the module
# republishes the graph every ~2s (touching the heartbeat), so waiting for
# quiet is safe; the solve itself can take minutes on 4k+ keyframe graphs,
# hence the generous cap.
GRAPH_SETTLE_QUIET_S = 5.0
GRAPH_SETTLE_MAX_S = 1800.0
GRAPH_SETTLE_POLL_S = 0.5
_PROGRESS_EVERY_N_SCANS = 200
# mirrors EDGE_LOOP_CLOSURE in rust/src/utils.rs pose-graph metadata ids
EDGE_LOOP_CLOSURE = 1


class ReplayStats(TypedDict, total=False):
    scans_sent: int
    error: str


class GraphCaptureConfig(ModuleConfig):
    output_path: Path = Path("graph-capture.json")


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
        self._loop_edges: list[tuple[float, float]] = []
        self._nodes: list[list[float]] = []
        self._edges: list[list[int]] = []

    async def handle_pose_graph(self, msg: Graph3D) -> None:
        # Heartbeat lets the host detect when the graph has settled post-replay.
        Path(str(self.config.output_path) + ".heartbeat").touch()
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
        ts_by_id = {node.id: node.pose.ts for node in msg.nodes}
        self._loop_edges = [
            (ts_by_id[edge.start_id], ts_by_id[edge.end_id])
            for edge in msg.edges
            if edge.metadata_id == EDGE_LOOP_CLOSURE
            and edge.start_id in ts_by_id
            and edge.end_id in ts_by_id
        ]
        self._nodes = [
            [
                node.id,
                node.pose.ts,
                node.pose.position.x,
                node.pose.position.y,
                node.pose.position.z,
                node.pose.orientation.x,
                node.pose.orientation.y,
                node.pose.orientation.z,
                node.pose.orientation.w,
            ]
            for node in msg.nodes
        ]
        self._edges = [[edge.start_id, edge.end_id, edge.metadata_id] for edge in msg.edges]

    async def handle_loop_closure_event(self, msg: GraphDelta3D) -> None:
        self._closures += 1

    async def main(self) -> AsyncGenerator[None, None]:
        yield
        self.config.output_path.write_text(
            json.dumps(
                {
                    "graph": self._graph,
                    "closures": self._closures,
                    "loop_edges": self._loop_edges,
                    "nodes": self._nodes,
                    "edges": self._edges,
                }
            )
        )


class LockstepReplayConfig(ModuleConfig):
    db: Path = Path("mem2.db")
    lidar_stream: str = "lidar"
    odometry_stream: str = "odom"
    lidar_stride: int = 1
    odometry_stride: int = 1
    odom_publish_hz: float = 500.0
    ack_timeout_s: float = 30.0
    done_path: Path = Path("replay-done.json")
    # Artificial odometry drift: a constant-velocity world offset added to both
    # odom poses and lidar clouds at time t (offset = drift_per_sec * (t - t0)).
    # Consistent per-instant, so the trajectory warps over time — exactly the
    # accumulating error loop closure is supposed to fix. [0,0,0] = no drift.
    drift_per_sec: list[float] = [0.0, 0.0, 0.0]
    drift_t0: float = 0.0


class LockstepReplay(Module):
    """Closed-loop replay: after each scan, wait for the module's
    corrected_odometry ack before sending the next.

    Every module under test sees 100% of the scans regardless of machine
    speed — wall clock varies, the data the module processes doesn't.
    Odometry messages are cheap latest-state updates and stay fire-and-forget
    (lightly paced). Writes a done-marker JSON at the end so the host knows
    when to tear down; a scan that is never acked is an error (skipping it
    would silently thin the data), reported through the same marker.

    odom and lidar are merged into one time-sorted stream, so playback runs in
    bursts: all odoms whose timestamps fall before the next scan are emitted
    fire-and-forget (paced by odom_publish_hz), then one scan is sent and the
    loop blocks on its ack. The only guarantee is one ack-wait per scan; the
    odom burst size per gap is data-dependent (~ odom_rate / lidar_rate)."""

    config: LockstepReplayConfig

    cloud: Out[PointCloud2]
    odometry: Out[Odometry]
    corrected_odometry: In[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._ack_event = asyncio.Event()

    async def handle_corrected_odometry(self, msg: Odometry) -> None:
        self._ack_event.set()

    def _load(self) -> list[tuple[float, str, Any]]:
        merged: list[tuple[float, str, Any]] = []
        with SqliteStore(path=self.config.db, must_exist=True) as store:
            for observation in islice(
                store.stream(self.config.odometry_stream, Odometry),
                0,
                None,
                self.config.odometry_stride,
            ):
                merged.append((float(observation.ts), "odom", observation.data))
            for lidar_observation in islice(
                store.stream(self.config.lidar_stream, PointCloud2),
                0,
                None,
                self.config.lidar_stride,
            ):
                merged.append((float(lidar_observation.ts), "lidar", lidar_observation.data))
        merged.sort(key=lambda item: item[0])
        return merged

    async def main(self) -> AsyncGenerator[None, None]:
        messages = await asyncio.to_thread(self._load)
        self._task = asyncio.create_task(self._replay(messages))
        yield
        self._task.cancel()
        with suppress(asyncio.CancelledError):
            await self._task

    async def _replay(self, messages: list[tuple[float, str, Any]]) -> None:
        odom_period = 1.0 / self.config.odom_publish_hz
        scans_sent = 0
        scans_skipped_pre_odom = 0
        first_odom_ts = next((timestamp for timestamp, kind, _ in messages if kind == "odom"), None)
        drift = np.asarray(self.config.drift_per_sec, dtype=np.float64)
        t0 = self.config.drift_t0
        apply_drift = has_drift(drift)
        for timestamp, kind, payload in messages:
            if kind == "odom":
                pose = payload.pose.pose
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

            if first_odom_ts is None or timestamp < first_odom_ts:
                # A scan before any odom can't be posed; the module drops it
                # without acking, so waiting for an ack would deadlock.
                scans_skipped_pre_odom += 1
                continue
            points = payload.points_f32()
            frame_id = payload.frame_id or "map"
            if apply_drift:
                points = points + (drift * (timestamp - t0)).astype(np.float32)
            self._ack_event.clear()
            self.cloud.publish(
                PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=timestamp)
            )
            scans_sent += 1
            try:
                await asyncio.wait_for(self._ack_event.wait(), timeout=self.config.ack_timeout_s)
            except TimeoutError:
                self.config.done_path.write_text(
                    json.dumps(
                        {
                            "scans_sent": scans_sent,
                            "scans_skipped_pre_odom": scans_skipped_pre_odom,
                            "error": (
                                f"scan at ts {timestamp} not acked within"
                                f" {self.config.ack_timeout_s}s — module dropped or stalled"
                            ),
                        }
                    )
                )
                return
            if scans_sent % _PROGRESS_EVERY_N_SCANS == 0:
                # Periodic progress so a watchdogged run still reports coverage.
                Path(str(self.config.done_path) + ".progress").write_text(
                    json.dumps({"scans_sent": scans_sent})
                )

        self.config.done_path.write_text(
            json.dumps({"scans_sent": scans_sent, "scans_skipped_pre_odom": scans_skipped_pre_odom})
        )


def run_module_graph(
    db_path: Path,
    module_class: type,
    config_overrides: dict[str, Any],
    *,
    lidar_stream: str,
    odom_stream: str,
    drift_per_sec: list[float] | None = None,
    drift_t0: float = 0.0,
) -> tuple[list[GraphPose], int, list[tuple[float, float]], dict[str, Any], dict[str, Any]]:
    """Replay the recording through the module; return its optimized pose graph
    (with orientations), loop-closure count, committed loop-edge keyframe
    timestamp pairs, replay stats, and the full graph detail (nodes with ids,
    edges with metadata ids).

    Scans are paced on the module's corrected_odometry acks — machine-speed
    independent, and every recorded message is fed (no strides, no caps).
    drift_per_sec injects a constant-velocity world offset into the replayed
    odom+lidar (see LockstepReplayConfig)."""
    drift_per_sec = drift_per_sec or [0.0, 0.0, 0.0]
    output_path = Path(tempfile.gettempdir()) / f"jnav_lc_eval_{db_path.parent.name}.json"
    output_path.unlink(missing_ok=True)
    heartbeat_path = Path(str(output_path) + ".heartbeat")
    heartbeat_path.unlink(missing_ok=True)
    done_path = Path(tempfile.gettempdir()) / f"jnav_lc_eval_done_{db_path.parent.name}.json"
    done_path.unlink(missing_ok=True)
    progress_path = Path(str(done_path) + ".progress")
    progress_path.unlink(missing_ok=True)
    counts_store = SqliteStore(path=db_path, must_exist=True)
    counts_store.start()
    lidar_count = int(counts_store.stream(lidar_stream).count())
    odom_count = int(counts_store.stream(odom_stream).count())
    counts_store.stop()

    blueprint = autoconnect(
        LockstepReplay.blueprint(
            db=db_path,
            lidar_stream=lidar_stream,
            odometry_stream=odom_stream,
            done_path=done_path,
            drift_per_sec=drift_per_sec,
            drift_t0=drift_t0,
        ),
        module_class.blueprint(**config_overrides),  # type: ignore[attr-defined]
        GraphCapture.blueprint(output_path=output_path),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    print(
        f"replaying {odom_count + lidar_count} messages"
        f" ({lidar_count} scans) through {module_class.__name__} (lockstep)"
    )
    replay_stats: dict[str, Any] = {}
    try:
        max_run_s = lidar_count * LOCKSTEP_PER_SCAN_BUDGET_S + LOCKSTEP_BASE_OVERHEAD_S
        started = time.monotonic()
        while not done_path.exists():
            elapsed = time.monotonic() - started
            if elapsed > max_run_s:
                coverage = ""
                if progress_path.exists():
                    coverage = f" (last progress: {progress_path.read_text()})"
                raise RuntimeError(
                    f"lockstep replay exceeded its watchdog budget"
                    f" ({lidar_count} scans x {LOCKSTEP_PER_SCAN_BUDGET_S}s"
                    f" + {LOCKSTEP_BASE_OVERHEAD_S}s = {round(max_run_s)}s);"
                    f" the module is likely hung{coverage}"
                )
            if int(elapsed) % 60 < LOCKSTEP_POLL_S and elapsed > LOCKSTEP_POLL_S:
                print(f"  ... lockstep replay running ({round(elapsed)}s)")
            time.sleep(LOCKSTEP_POLL_S)
        replay_stats.update(json.loads(done_path.read_text()))
        progress_path.unlink(missing_ok=True)
        if "error" in replay_stats:
            raise RuntimeError(f"lockstep replay failed: {replay_stats['error']}")

        # Let the pose graph settle (background GNC classification may still be
        # in flight) before tearing the modules down.
        settle_started = time.monotonic()
        while time.monotonic() - settle_started < GRAPH_SETTLE_MAX_S:
            if heartbeat_path.exists():
                quiet_s = time.time() - heartbeat_path.stat().st_mtime
            else:
                quiet_s = time.monotonic() - settle_started
            if quiet_s >= GRAPH_SETTLE_QUIET_S:
                break
            time.sleep(GRAPH_SETTLE_POLL_S)
    finally:
        coordinator.stop()
        heartbeat_path.unlink(missing_ok=True)

    if not output_path.exists():
        raise SystemExit(f"{module_class.__name__} produced no pose graph output")
    data = json.loads(output_path.read_text())
    graph = [tuple(row) for row in data["graph"]]
    loop_edges = [
        (float(start_ts), float(end_ts)) for start_ts, end_ts in data.get("loop_edges", [])
    ]
    graph_detail = {"nodes": data.get("nodes", []), "edges": data.get("edges", [])}
    return graph, int(data["closures"]), loop_edges, replay_stats, graph_detail  # type: ignore[return-value]
