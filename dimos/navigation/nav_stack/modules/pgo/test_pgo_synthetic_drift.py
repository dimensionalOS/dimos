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

This test runs the PGO native binary twice with the same input:

1. ``use_scan_context=true``  → expect ≥1 pgo_loop_closure event.
2. ``use_scan_context=false`` → expect 0 pgo_loop_closure events.

Exposes the actual on-the-wire payload (event count, per-event shape)
on stdout for the user to inspect.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    NativeProcessRunner,
    lcm_handle_loop,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.slow]

PGO_BIN = Path(__file__).parent / "cpp" / "result" / "bin" / "pgo"

SCAN_LCM = "/sdpgo_scan#sensor_msgs.PointCloud2"
ODOM_LCM = "/sdpgo_odom#nav_msgs.Odometry"
CORRECTED_ODOM_LCM = "/sdpgo_corrected#nav_msgs.Odometry"
GLOBAL_MAP_LCM = "/sdpgo_global_map#sensor_msgs.PointCloud2"
TF_LCM = "/sdpgo_tf#tf2_msgs.TFMessage"
GRAPH_NODES_LCM = "/sdpgo_graph_nodes#nav_msgs.GraphNodes3D"
GRAPH_EDGES_LCM = "/sdpgo_graph_edges#nav_msgs.LineSegments3D"
LOOP_CLOSURE_LCM = "/sdpgo_loop_closure#nav_msgs.Path"

# Cross-trajectory drift injected at the revisit. Must be >> loop_search_radius
# so position-based search cannot accidentally find the loop.
DRIFT_AT_REVISIT_M = 5.0

# Loop closure thresholds passed to the binary.
LOOP_SEARCH_RADIUS_M = 1.0
LOOP_TIME_THRESH_S = 5.0
MIN_LOOP_DETECT_DURATION_S = 1.0


def _make_room_points(half_size: float = 20.0, density: float = 0.15) -> np.ndarray:
    """Sample points on the inside of a 4-wall square room.

    Walls are at x=±half_size and y=±half_size, z ∈ [0, 3]. ``density``
    is the in-plane point spacing in metres.
    """
    points: list[np.ndarray] = []
    z_levels = np.arange(0.0, 3.0, density)

    line_xy = np.arange(-half_size, half_size, density)
    # north / south walls (y = ±half_size, x varies)
    for y in (half_size, -half_size):
        xx = line_xy
        zz = z_levels
        xv, zv = np.meshgrid(xx, zz)
        block = np.column_stack([xv.ravel(), np.full(xv.size, y), zv.ravel()])
        points.append(block)
    # east / west walls (x = ±half_size, y varies)
    for x in (half_size, -half_size):
        yy = line_xy
        zz = z_levels
        yv, zv = np.meshgrid(yy, zz)
        block = np.column_stack([np.full(yv.size, x), yv.ravel(), zv.ravel()])
        points.append(block)

    # add a couple of distinctive interior columns so the scene isn't
    # rotationally symmetric — helps Scan Context disambiguate.
    for cx, cy in [(5.0, 0.0), (-5.0, 8.0)]:
        ang = np.arange(0.0, 2.0 * math.pi, 0.2)
        zz = np.arange(0.0, 3.0, density)
        ag, zg = np.meshgrid(ang, zz)
        rx = cx + 0.5 * np.cos(ag.ravel())
        ry = cy + 0.5 * np.sin(ag.ravel())
        points.append(np.column_stack([rx, ry, zg.ravel()]))

    return np.concatenate(points).astype(np.float32)


def _make_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position = Vector3(x, y, z)
    # yaw-only quaternion (rotation about z)
    half = yaw * 0.5
    pose.orientation = Quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    return pose


def _yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _world_to_body(points_world: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rot = _yaw_rotation(yaw).T
    return (points_world - position) @ rot.T


def _body_to_world(points_body: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    rot = _yaw_rotation(yaw)
    return points_body @ rot.T + position


def _trajectory_reverse_loop(
    n_outbound: int = 20, n_inbound: int = 20, leg_length: float = 8.0
) -> list[tuple[float, np.ndarray, float, np.ndarray, float]]:
    """Out-and-back trajectory where the robot turns 180° at the far
    end, so the return leg is driven *facing west* while the outbound
    leg was *facing east*.

    No translation drift — the only thing distinguishing inbound from
    outbound observations at the same position is the heading. This
    forces Scan Context to find a column shift of ~n_sectors/2 to
    match, and forces ICP to be seeded with a yaw init_guess rotated
    about the source keyframe (not the world origin). Without the
    init_guess fix in ``searchForLoopPairs``, a 180° rotation around
    the world origin sends the source cloud kilometres away from the
    target and ICP fails to converge.
    """
    samples: list[tuple[float, np.ndarray, float, np.ndarray, float]] = []
    t = 1.0
    dt = 0.5
    # outbound: drive east, facing east (yaw=0)
    for i in range(n_outbound + 1):
        frac = i / max(n_outbound, 1)
        x = frac * leg_length
        pos = np.array([x, 0.0, 0.5])
        yaw = 0.0
        samples.append((t, pos, yaw, pos.copy(), yaw))
        t += dt
    # inbound: drive west, facing west (yaw=π) — body-frame scans see
    # the room rotated 180° relative to the outbound leg.
    for i in range(1, n_inbound + 1):
        frac = i / max(n_inbound, 1)
        x = leg_length * (1.0 - frac)
        pos = np.array([x, 0.0, 0.5])
        yaw = math.pi
        samples.append((t, pos, yaw, pos.copy(), yaw))
        t += dt
    return samples


def _trajectory_with_drift(
    n_outbound: int = 20, n_inbound: int = 20, leg_length: float = 8.0
) -> list[tuple[float, np.ndarray, float, np.ndarray, float]]:
    """Generate a list of ``(t, true_position, true_yaw,
    drifted_position, drifted_yaw)`` waypoints for an out-and-back
    trajectory that physically returns to the start.

    The drift is purely additive in (x, y) and ramps linearly with the
    total travelled distance, so by the time the robot returns to
    (0, 0) the reported odom pose is offset by ``DRIFT_AT_REVISIT_M``.
    """
    samples: list[tuple[float, np.ndarray, float, np.ndarray, float]] = []
    # Start at t=1.0 because Odometry(ts=0.0) is treated as "now" by the
    # message constructor — using 0.0 would inject wall-clock time and
    # break the monotonic-ts assumption in PGO's on_registered_scan.
    t = 1.0
    dt = 0.5
    # outbound: drive east
    for i in range(n_outbound + 1):
        frac = i / max(n_outbound, 1)
        x = frac * leg_length
        true_pos = np.array([x, 0.0, 0.5])
        yaw = 0.0
        drift_amount = (i / (n_outbound + n_inbound)) * DRIFT_AT_REVISIT_M
        drifted = true_pos + np.array([0.0, drift_amount, 0.0])
        samples.append((t, true_pos, yaw, drifted, yaw))
        t += dt
    # inbound: drive west back to origin
    for i in range(1, n_inbound + 1):
        frac = i / max(n_inbound, 1)
        x = leg_length * (1.0 - frac)
        true_pos = np.array([x, 0.0, 0.5])
        yaw = 0.0  # keep heading the same so descriptors are directly comparable
        drift_amount = ((n_outbound + i) / (n_outbound + n_inbound)) * DRIFT_AT_REVISIT_M
        drifted = true_pos + np.array([0.0, drift_amount, 0.0])
        samples.append((t, true_pos, yaw, drifted, yaw))
        t += dt
    return samples


def _publish_scan(
    lcm_instance: lcmlib.LCM,
    body_points: np.ndarray,
    drifted_pose: tuple[np.ndarray, float],
    ts: float,
) -> None:
    # registered_scan is the body-frame scan transformed via the (drifted)
    # odometry — that's what a SLAM frontend publishes.
    world_points = _body_to_world(body_points, drifted_pose[0], drifted_pose[1])
    msg = PointCloud2.from_numpy(world_points.astype(np.float32), frame_id="map", timestamp=ts)
    lcm_instance.publish(SCAN_LCM, msg.lcm_encode())


def _publish_odom(
    lcm_instance: lcmlib.LCM, drifted_pose: tuple[np.ndarray, float], ts: float
) -> None:
    pos, yaw = drifted_pose
    msg = Odometry(
        ts=ts,
        frame_id="odom",
        child_frame_id="base_link",
        pose=_make_pose(float(pos[0]), float(pos[1]), float(pos[2]), float(yaw)),
    )
    lcm_instance.publish(ODOM_LCM, msg.lcm_encode())


def _run_pgo(
    use_scan_context: bool,
    trajectory: list[tuple[float, np.ndarray, float, np.ndarray, float]] | None = None,
) -> int:
    """Run a single PGO instance over the synthetic trajectory and
    return the number of pgo_loop_closure events received."""
    if not PGO_BIN.exists():
        pytest.skip(f"PGO binary not found: {PGO_BIN}")

    room_points = _make_room_points()
    if trajectory is None:
        trajectory = _trajectory_with_drift()

    lcm_instance = lcmlib.LCM()
    received_events: list[NavPath] = []
    events_lock = threading.Lock()

    def _on_loop_closure(_channel: str, data: bytes) -> None:
        msg = NavPath.lcm_decode(data)
        with events_lock:
            idx = len(received_events)
            received_events.append(msg)
        logger.info(
            f"[synthetic_drift sc={use_scan_context}] event #{idx}: "
            f"N={len(msg.poses)}, ts={msg.ts:.3f}"
        )

    sub = lcm_instance.subscribe(LOOP_CLOSURE_LCM, _on_loop_closure)

    stop_event = threading.Event()
    handle_thread = threading.Thread(
        target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True
    )
    handle_thread.start()

    runner = NativeProcessRunner(
        binary_path=str(PGO_BIN),
        args=[
            "--registered_scan",
            SCAN_LCM,
            "--odometry",
            ODOM_LCM,
            "--corrected_odometry",
            CORRECTED_ODOM_LCM,
            "--global_map",
            GLOBAL_MAP_LCM,
            "--tf_channel",
            TF_LCM,
            "--pgo_graph_nodes",
            GRAPH_NODES_LCM,
            "--pgo_graph_edges",
            GRAPH_EDGES_LCM,
            "--pgo_loop_closure",
            LOOP_CLOSURE_LCM,
            "--key_pose_delta_deg",
            "10.0",
            "--key_pose_delta_trans",
            "0.4",
            "--loop_search_radius",
            str(LOOP_SEARCH_RADIUS_M),
            "--loop_time_thresh",
            str(LOOP_TIME_THRESH_S),
            "--loop_score_thresh",
            "1.0",
            "--loop_submap_half_range",
            "5",
            "--submap_resolution",
            "0.1",
            "--min_loop_detect_duration",
            str(MIN_LOOP_DETECT_DURATION_S),
            "--global_map_voxel_size",
            "0.1",
            "--global_map_publish_rate",
            "1.0",
            "--unregister_input",
            "true",
            "--use_scan_context",
            "true" if use_scan_context else "false",
            "--sc_max_range_m",
            "30.0",
            "--sc_match_threshold",
            "0.6",
            "--world_frame",
            "map",
            "--local_frame",
            "odom",
        ],
    )

    stderr_data = b""
    try:
        runner.start(capture_stderr=True)
        time.sleep(1.5)
        assert runner.is_running, "PGO failed to start"

        for t, true_pos, true_yaw, drifted_pos, drifted_yaw in trajectory:
            body_points = _world_to_body(room_points, true_pos, true_yaw)
            _publish_odom(lcm_instance, (drifted_pos, drifted_yaw), t)
            _publish_scan(lcm_instance, body_points, (drifted_pos, drifted_yaw), t)
            time.sleep(0.15)

        time.sleep(3.0)

        # Read stderr while process is still alive
        if runner.process is not None and runner.process.stderr is not None:
            runner.process.terminate()
            try:
                stderr_data = runner.process.stderr.read()
            except Exception:
                stderr_data = b""
    finally:
        runner.stop()
        stop_event.set()
        handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        lcm_instance.unsubscribe(sub)

        if stderr_data:
            logger.info(f"\n--- PGO stderr (sc={use_scan_context}) ---")
            logger.info(stderr_data.decode("utf-8", errors="replace"))
            logger.info("--- end PGO stderr ---\n")

    with events_lock:
        return len(received_events)


class TestPGOSyntheticDrift:
    """Scan Context catches the loop; position search misses it."""

    def test_scan_context_catches_drifted_loop(self) -> None:
        sc_events = _run_pgo(use_scan_context=True)
        logger.info(f"[synthetic_drift] scan_context=true  → {sc_events} loop events")
        assert sc_events >= 1, (
            f"Scan Context should catch the loop at the revisit point "
            f"(drift={DRIFT_AT_REVISIT_M}m). Got {sc_events} events."
        )

    def test_position_search_misses_drifted_loop(self) -> None:
        pos_events = _run_pgo(use_scan_context=False)
        logger.info(f"[synthetic_drift] scan_context=false → {pos_events} loop events")
        assert pos_events == 0, (
            f"Position-based search shouldn't fire when drift "
            f"({DRIFT_AT_REVISIT_M}m) >> loop_search_radius "
            f"({LOOP_SEARCH_RADIUS_M}m). Got {pos_events} events."
        )

    def test_scan_context_catches_reverse_loop(self) -> None:
        """Robot drives 8m east facing east, turns 180°, drives back to
        the start facing west. Body-frame scans on the return leg are
        rotated 180° relative to outbound, so Scan Context must use a
        non-zero sector shift and ICP must be seeded with a yaw init
        rotated about the *source keyframe* (not the world origin) for
        the clouds to align. Without that fix in
        ``simple_pgo.cpp::searchForLoopPairs``, the rotated source ends
        up displaced from the target and ICP can't converge.
        """
        events = _run_pgo(
            use_scan_context=True,
            trajectory=_trajectory_reverse_loop(),
        )
        logger.info(f"[reverse_loop] → {events} loop events")
        assert events >= 1, (
            "Scan Context + iCP should catch the 180° reverse-heading loop. "
            f"Got {events} events. This regresses the init_guess fix in "
            "simple_pgo.cpp (rotation must be about the source keyframe, "
            "not the world origin)."
        )
