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

"""Demonstrate that Scan Context catches loop closures that the
position-based search would miss.

Setup: build a synthetic point-cloud "room", drive a virtual robot
out-and-back along a corridor, and inject a linear drift into the
reported odometry. On the return leg the robot is *physically* back at
the start (so the body-frame scan is byte-identical to the first
scan), but the reported odom pose is offset by several metres. With
``loop_search_radius=1.0m`` the position-based search cannot match
the two visits; Scan Context, which works on the appearance of the
scan rather than the pose, can.

This test runs PGO twice with the same input via the DimOS Module +
Blueprint pipeline (no direct LCM topic strings here):

1. ``use_scan_context=true``  → expect ≥1 loop_closure event.
2. ``use_scan_context=false`` → expect 0 loop_closure events.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import math
import time
from typing import Any

import numpy as np
import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.slow]

# Cross-trajectory drift injected at the revisit. Must be >> loop_search_radius
# so position-based search cannot accidentally find the loop.
DRIFT_AT_REVISIT_M = 5.0

# Loop closure thresholds passed to the binary.
LOOP_SEARCH_RADIUS_M = 1.0
LOOP_TIME_THRESH_S = 5.0
MIN_LOOP_DETECT_DURATION_S = 1.0

# Per-frame publish interval driving the synthetic playback module.
INTER_FRAME_SLEEP_SEC = 0.15
# Drain after the playback module reports finished, so PGO can flush
# any pending loop closure events before the coordinator stops.
POST_FEED_DRAIN_SEC = 3.0
# Poll period when waiting for the playback module to drain.
POLL_INTERVAL_SEC = 0.25


def _make_room_points(half_size: float = 20.0, density: float = 0.15) -> np.ndarray:
    """Sample points on the inside of a 4-wall square room."""
    points: list[np.ndarray] = []
    z_levels = np.arange(0.0, 3.0, density)
    wall_axis = np.arange(-half_size, half_size, density)

    for wall_y in (half_size, -half_size):
        grid_x, grid_z = np.meshgrid(wall_axis, z_levels)
        block = np.column_stack([grid_x.ravel(), np.full(grid_x.size, wall_y), grid_z.ravel()])
        points.append(block)
    for wall_x in (half_size, -half_size):
        grid_y, grid_z = np.meshgrid(wall_axis, z_levels)
        block = np.column_stack([np.full(grid_y.size, wall_x), grid_y.ravel(), grid_z.ravel()])
        points.append(block)

    # Distinctive interior columns so the scene isn't rotationally symmetric.
    column_radius = 0.5
    for column_center_x, column_center_y in [(5.0, 0.0), (-5.0, 8.0)]:
        angles = np.arange(0.0, 2.0 * math.pi, 0.2)
        column_z_levels = np.arange(0.0, 3.0, density)
        grid_angle, grid_z = np.meshgrid(angles, column_z_levels)
        column_x = column_center_x + column_radius * np.cos(grid_angle.ravel())
        column_y = column_center_y + column_radius * np.sin(grid_angle.ravel())
        points.append(np.column_stack([column_x, column_y, grid_z.ravel()]))

    return np.concatenate(points).astype(np.float32)


def _make_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position = Vector3(x, y, z)
    half_yaw = yaw * 0.5
    pose.orientation = Quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    return pose


def _yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _world_to_body(points_world: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rotation = _yaw_rotation(yaw).T
    return (points_world - position) @ rotation.T


def _body_to_world(points_body: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rotation = _yaw_rotation(yaw)
    return points_body @ rotation.T + position


def _trajectory_with_drift(
    num_outbound: int = 20, num_inbound: int = 20, leg_length: float = 8.0
) -> list[tuple[float, np.ndarray, float, np.ndarray, float]]:
    """``(t, true_position, true_yaw, drifted_position, drifted_yaw)`` waypoints
    for an out-and-back trajectory that physically returns to the start.

    The drift is purely additive in (x, y) and ramps linearly with the total
    travelled distance, so by the time the robot returns to (0, 0) the reported
    odom pose is offset by ``DRIFT_AT_REVISIT_M``.
    """
    samples: list[tuple[float, np.ndarray, float, np.ndarray, float]] = []
    # Start at timestamp=1.0 because Odometry(ts=0.0) is treated as "now" by
    # the constructor — using 0.0 would inject wall-clock time and break the
    # monotonic-ts assumption in PGO's on_registered_scan.
    timestamp = 1.0
    time_step = 0.5
    total_steps = num_outbound + num_inbound
    for step in range(num_outbound + 1):
        progress = step / max(num_outbound, 1)
        x = progress * leg_length
        true_position = np.array([x, 0.0, 0.5])
        yaw = 0.0
        drift_amount = (step / total_steps) * DRIFT_AT_REVISIT_M
        drifted_position = true_position + np.array([0.0, drift_amount, 0.0])
        samples.append((timestamp, true_position, yaw, drifted_position, yaw))
        timestamp += time_step
    for step in range(1, num_inbound + 1):
        progress = step / max(num_inbound, 1)
        x = leg_length * (1.0 - progress)
        true_position = np.array([x, 0.0, 0.5])
        yaw = 0.0  # keep heading the same so descriptors are directly comparable
        drift_amount = ((num_outbound + step) / total_steps) * DRIFT_AT_REVISIT_M
        drifted_position = true_position + np.array([0.0, drift_amount, 0.0])
        samples.append((timestamp, true_position, yaw, drifted_position, yaw))
        timestamp += time_step
    return samples


def _trajectory_reverse_loop(
    num_outbound: int = 20, num_inbound: int = 20, leg_length: float = 8.0
) -> list[tuple[float, np.ndarray, float, np.ndarray, float]]:
    """Out-and-back where the robot turns 180° at the far end.

    Exercises ICP's yaw-around-source-keyframe init_guess fix in
    ``simple_pgo.cpp::searchForLoopPairs``.
    """
    samples: list[tuple[float, np.ndarray, float, np.ndarray, float]] = []
    timestamp = 1.0
    time_step = 0.5
    for step in range(num_outbound + 1):
        progress = step / max(num_outbound, 1)
        x = progress * leg_length
        position = np.array([x, 0.0, 0.5])
        yaw = 0.0
        samples.append((timestamp, position, yaw, position.copy(), yaw))
        timestamp += time_step
    for step in range(1, num_inbound + 1):
        progress = step / max(num_inbound, 1)
        x = leg_length * (1.0 - progress)
        position = np.array([x, 0.0, 0.5])
        yaw = math.pi
        samples.append((timestamp, position, yaw, position.copy(), yaw))
        timestamp += time_step
    return samples


def _trajectory_payload(
    trajectory: list[tuple[float, np.ndarray, float, np.ndarray, float]],
) -> list[list[float]]:
    """Flatten the trajectory into a JSON-serializable matrix for ModuleConfig.

    Each row is ``[timestamp, true_x, true_y, true_z, true_yaw,
    drifted_x, drifted_y, drifted_z, drifted_yaw]``.
    """
    rows: list[list[float]] = []
    for timestamp, true_position, true_yaw, drifted_position, drifted_yaw in trajectory:
        rows.append(
            [
                float(timestamp),
                float(true_position[0]),
                float(true_position[1]),
                float(true_position[2]),
                float(true_yaw),
                float(drifted_position[0]),
                float(drifted_position[1]),
                float(drifted_position[2]),
                float(drifted_yaw),
            ]
        )
    return rows


class SyntheticDriftPlaybackConfig(ModuleConfig):
    trajectory: list[list[float]]
    inter_frame_sleep_sec: float = INTER_FRAME_SLEEP_SEC
    room_half_size: float = 20.0
    room_density: float = 0.15


class SyntheticDriftPlaybackModule(Module):
    """Publishes synthetic scans + drifted odometry from a precomputed trajectory."""

    config: SyntheticDriftPlaybackConfig

    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frames_published: int = 0
        self._playback_finished: bool = False

    async def main(self) -> AsyncIterator[None]:
        self._room_points = _make_room_points(self.config.room_half_size, self.config.room_density)
        self._playback_task = asyncio.create_task(self._run_playback())
        yield
        self._playback_task.cancel()

    async def _run_playback(self) -> None:
        for row in self.config.trajectory:
            (
                timestamp,
                true_x,
                true_y,
                true_z,
                true_yaw,
                drifted_x,
                drifted_y,
                drifted_z,
                drifted_yaw,
            ) = row
            true_position = np.array([true_x, true_y, true_z])
            drifted_position = np.array([drifted_x, drifted_y, drifted_z])
            body_points = _world_to_body(self._room_points, true_position, true_yaw)
            world_points = _body_to_world(body_points, drifted_position, drifted_yaw)
            scan_message = PointCloud2.from_numpy(
                world_points.astype(np.float32), frame_id="map", timestamp=timestamp
            )
            odometry_message = Odometry(
                ts=timestamp,
                frame_id="odom",
                child_frame_id="base_link",
                pose=_make_pose(
                    float(drifted_position[0]),
                    float(drifted_position[1]),
                    float(drifted_position[2]),
                    float(drifted_yaw),
                ),
            )
            self.odometry.publish(odometry_message)
            self.registered_scan.publish(scan_message)
            self._frames_published += 1
            if self.config.inter_frame_sleep_sec > 0:
                await asyncio.sleep(self.config.inter_frame_sleep_sec)
        self._playback_finished = True

    @rpc
    def is_finished(self) -> bool:
        return self._playback_finished

    @rpc
    def frames_published(self) -> int:
        return self._frames_published


class LoopClosureCounterConfig(ModuleConfig):
    pass


class LoopClosureCounterModule(Module):
    """Counts loop_closure events from any pose-graph SLAM module."""

    config: LoopClosureCounterConfig
    loop_closure: In[NavPath]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._count: int = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.loop_closure.subscribe(self._on_loop_closure)))

    def _on_loop_closure(self, message: NavPath) -> None:
        self._count += 1
        logger.info(
            f"[loop_closure_counter] event #{self._count - 1}: "
            f"keyframe_count={len(message.poses)}, ts={message.ts:.3f}"
        )

    @rpc
    def count(self) -> int:
        return self._count


def _run_pgo(
    use_scan_context: bool,
    trajectory: list[tuple[float, np.ndarray, float, np.ndarray, float]] | None = None,
) -> int:
    """Build the blueprint, run the synthetic trajectory through PGO, return loop count."""
    if trajectory is None:
        trajectory = _trajectory_with_drift()

    playback_blueprint = SyntheticDriftPlaybackModule.blueprint(
        trajectory=_trajectory_payload(trajectory),
    )
    pgo_blueprint = PGO.blueprint(
        use_scan_context=use_scan_context,
        key_pose_delta_trans=0.4,
        loop_search_radius=LOOP_SEARCH_RADIUS_M,
        loop_time_thresh=LOOP_TIME_THRESH_S,
        loop_score_thresh=1.0,
        loop_submap_half_range=5,
        submap_resolution=0.1,
        min_loop_detect_duration=MIN_LOOP_DETECT_DURATION_S,
        global_map_voxel_size=0.1,
        global_map_publish_rate=1.0,
        unregister_input=True,
        scan_context_max_range_m=30.0,
        scan_context_match_threshold=0.6,
    )
    counter_blueprint = LoopClosureCounterModule.blueprint()

    blueprint = autoconnect(playback_blueprint, pgo_blueprint, counter_blueprint)
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        playback = coordinator.get_instance(SyntheticDriftPlaybackModule)
        counter = coordinator.get_instance(LoopClosureCounterModule)
        while not playback.is_finished():
            time.sleep(POLL_INTERVAL_SEC)
        time.sleep(POST_FEED_DRAIN_SEC)
        return counter.count()
    finally:
        coordinator.stop()


class TestPGOSyntheticDrift:
    """Scan Context catches the loop; position search misses it."""

    def test_scan_context_catches_drifted_loop(self) -> None:
        scan_context_events = _run_pgo(use_scan_context=True)
        logger.info(f"[synthetic_drift] scan_context=true  → {scan_context_events} loop events")
        assert scan_context_events >= 1, (
            f"Scan Context should catch the loop at the revisit point "
            f"(drift={DRIFT_AT_REVISIT_M}m). Got {scan_context_events} events."
        )

    def test_position_search_misses_drifted_loop(self) -> None:
        position_search_events = _run_pgo(use_scan_context=False)
        logger.info(f"[synthetic_drift] scan_context=false → {position_search_events} loop events")
        assert position_search_events == 0, (
            f"Position-based search shouldn't fire when drift "
            f"({DRIFT_AT_REVISIT_M}m) >> loop_search_radius "
            f"({LOOP_SEARCH_RADIUS_M}m). Got {position_search_events} events."
        )

    def test_scan_context_catches_reverse_loop(self) -> None:
        """Robot drives 8m east facing east, turns 180°, drives back facing west.

        Regression test for the init_guess fix in
        ``simple_pgo.cpp::searchForLoopPairs``: ICP must seed the yaw rotation
        about the source keyframe (not the world origin) for the rotated source
        cloud to stay co-located with the target.
        """
        events = _run_pgo(use_scan_context=True, trajectory=_trajectory_reverse_loop())
        logger.info(f"[reverse_loop] → {events} loop events")
        assert events >= 1, (
            "Scan Context + ICP should catch the 180° reverse-heading loop. "
            f"Got {events} events. This regresses the init_guess fix in "
            "simple_pgo.cpp (rotation must be about the source keyframe, "
            "not the world origin)."
        )
