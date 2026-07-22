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
from dimos.navigation.jnav.utils.recording_db import (
    MAX_REPLAY_ODOM,
    MAX_REPLAY_SCANS,
    REPLAY_DRAIN_MARGIN_S,
    REPLAY_PUBLISH_HZ,
    iterate_stream,
    stream_count,
)
from dimos.navigation.jnav.utils.recording_tf import payload_pose
from dimos.navigation.jnav.utils.trajectory_metrics import GraphPose, has_drift

# Run cap scales with the workload: a per-scan budget (well above any sane
# processing time, below the 30s ack timeout) plus fixed startup overhead.
LOCKSTEP_PER_SCAN_BUDGET_S = 2.0
LOCKSTEP_BASE_OVERHEAD_S = 120.0
LOCKSTEP_POLL_S = 5.0
LOCKSTEP_DRAIN_S = 10.0
_PROGRESS_EVERY_N_SCANS = 200


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
                pose = payload_pose(payload)
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
                        pose=payload_pose(payload),
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
