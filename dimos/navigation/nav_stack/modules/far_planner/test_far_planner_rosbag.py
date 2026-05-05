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

"""Rosbag accuracy test: replays inputs at original timing and compares waypoints to ROS reference."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    LcmCollector,
    NativeProcessRunner,
    feed_at_original_timing,
    lcm_handle_loop,
    load_rosbag_window,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.slow]

# Time for the native process to initialize before feeding data.
_PROCESS_STARTUP_SEC = 1.5
# Time after feeding data for the process to finish emitting outputs.
_POST_FEED_DRAIN_SEC = 3.0

FAR_PLANNER_BIN = (
    Path(__file__).parent.parent
    / "modules"
    / "far_planner"
    / "result"
    / "bin"
    / "far_planner_native"
)

# LCM topics
SCAN_LCM = "/rbfp_scan#sensor_msgs.PointCloud2"
ODOM_LCM = "/rbfp_odom#nav_msgs.Odometry"
TERRAIN_LCM = "/rbfp_terrain#sensor_msgs.PointCloud2"
TERRAIN_EXT_LCM = "/rbfp_terrain_ext#sensor_msgs.PointCloud2"
GOAL_LCM = "/rbfp_goal#geometry_msgs.PointStamped"
STOP_LCM = "/rbfp_stop#std_msgs.Bool"
WAYPOINT_OUT_LCM = "/rbfp_wp#geometry_msgs.PointStamped"
GOAL_PATH_LCM = "/rbfp_gp#nav_msgs.Path"
GRAPH_NODES_LCM = "/rbfp_gn#nav_msgs.GraphNodes3D"
GRAPH_EDGES_LCM = "/rbfp_ge#nav_msgs.LineSegments3D"
CONTOUR_LCM = "/rbfp_cp#nav_msgs.ContourPolygons3D"
NAV_BOUNDARY_LCM = "/rbfp_nb#nav_msgs.LineSegments3D"


def _far_planner_args() -> list[str]:
    return [
        "--terrain_map_ext",
        TERRAIN_EXT_LCM,
        "--terrain_map",
        TERRAIN_LCM,
        "--registered_scan",
        SCAN_LCM,
        "--odometry",
        ODOM_LCM,
        "--goal",
        GOAL_LCM,
        "--stop_movement",
        STOP_LCM,
        "--way_point",
        WAYPOINT_OUT_LCM,
        "--goal_path",
        GOAL_PATH_LCM,
        "--graph_nodes",
        GRAPH_NODES_LCM,
        "--graph_edges",
        GRAPH_EDGES_LCM,
        "--contour_polygons",
        CONTOUR_LCM,
        "--nav_boundary",
        NAV_BOUNDARY_LCM,
        # Exact OG nav stack runtime params (from params.txt dump)
        "--update_rate",
        "5.0",
        "--robot_dim",
        "0.5",
        "--voxel_dim",
        "0.1",
        "--sensor_range",
        "15.0",
        "--terrain_range",
        "7.5",
        "--local_planner_range",
        "2.5",
        "--vehicle_height",
        "0.6",
        "--is_static_env",
        "false",
        "--is_viewpoint_extend",
        "true",
        "--is_attempt_autoswitch",
        "true",
        "--is_debug_output",
        "false",
        "--is_multi_layer",
        "false",
        "--world_frame",
        "map",
        "--converge_dist",
        "0.4",
        "--goal_adjust_radius",
        "1.0",
        "--free_counter_thred",
        "7",
        "--reach_goal_vote_size",
        "3",
        "--path_momentum_thred",
        "3",
        "--floor_height",
        "1.5",
        "--cell_length",
        "1.5",
        "--map_grid_max_length",
        "300.0",
        "--map_grad_max_height",
        "15.0",
        "--connect_votes_size",
        "10",
        "--clear_dumper_thred",
        "4",
        "--node_finalize_thred",
        "6",
        "--filter_pool_size",
        "12",
        "--resize_ratio",
        "3.0",
        "--filter_count_value",
        "6",
        "--angle_noise",
        "15.0",
        "--accept_max_align_angle",
        "4.0",
        "--new_intensity_thred",
        "2.0",
        "--dynamic_obs_decay_time",
        "2.0",
        "--new_points_decay_time",
        "1.0",
        "--dyobs_update_thred",
        "4",
        "--new_point_counter",
        "5",
        "--obs_inflate_size",
        "1",
        "--visualize_ratio",
        "0.4",
        "--wp_churn_dist",
        "0",  # Disable churn reduction for rosbag comparison
    ]


def _compute_waypoint_deviation(
    our_wps: list[tuple[float, float]], ref_wp: np.ndarray
) -> dict[str, float]:
    """Compute deviation score between our waypoints and reference.

    Returns dict with: mean_error_m, max_error_m, count_ratio, mean_x_diff, mean_y_diff.
    """
    if len(our_wps) == 0 or len(ref_wp) == 0:
        return {
            "mean_error_m": float("inf"),
            "max_error_m": float("inf"),
            "count_ratio": 0.0,
            "mean_x_diff": float("inf"),
            "mean_y_diff": float("inf"),
        }

    our_arr = np.array(our_wps)
    ref_xy = ref_wp[:, 1:3]  # x, y columns

    # For each reference waypoint, find nearest our waypoint by position
    errors = []
    for ref_pt in ref_xy:
        dists = np.sqrt((our_arr[:, 0] - ref_pt[0]) ** 2 + (our_arr[:, 1] - ref_pt[1]) ** 2)
        errors.append(float(dists.min()))

    our_mean = our_arr.mean(axis=0)
    ref_mean = ref_xy.mean(axis=0)

    return {
        "mean_error_m": float(np.mean(errors)),
        "max_error_m": float(np.max(errors)),
        "count_ratio": len(our_wps) / len(ref_wp),
        "mean_x_diff": float(abs(our_mean[0] - ref_mean[0])),
        "mean_y_diff": float(abs(our_mean[1] - ref_mean[1])),
    }


class TestFarPlannerRosbag:
    """Validate FAR planner accuracy against OG nav stack recording."""

    def test_waypoint_accuracy(self) -> None:
        """Feed identical inputs at original timing and compare waypoint output."""
        if not FAR_PLANNER_BIN.exists():
            pytest.skip(f"FAR planner binary not found: {FAR_PLANNER_BIN}")

        window = load_rosbag_window()
        ref_wp = window.way_point
        assert len(ref_wp) > 0, "No reference waypoints in fixture"

        lcm = lcmlib.LCM()
        wp_collector = LcmCollector(topic=WAYPOINT_OUT_LCM, msg_type=PointStamped)
        wp_collector.start(lcm)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm, stop_event), daemon=True
        )
        handle_thread.start()

        runner = NativeProcessRunner(binary_path=str(FAR_PLANNER_BIN), args=_far_planner_args())

        try:
            runner.start()
            assert runner.is_running, "FAR planner binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            # Feed at original timing (1:1 with rosbag)
            feed_at_original_timing(
                lcm,
                window,
                topic_map={
                    "odom": ODOM_LCM,
                    "scan": SCAN_LCM,
                    "terrain": TERRAIN_LCM,
                    "terrain_ext": TERRAIN_EXT_LCM,
                    "goal": GOAL_LCM,
                },
            )

            time.sleep(_POST_FEED_DRAIN_SEC)

        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            wp_collector.stop(lcm)

        our_wps = [(msg.x, msg.y) for msg in wp_collector.messages]

        # Compute deviation score
        score = _compute_waypoint_deviation(our_wps, ref_wp)

        # Log score for visibility
        logger.info(f"\n{'=' * 60}")
        logger.info("FAR PLANNER DEVIATION SCORE")
        logger.info(f"  Our waypoints:     {len(our_wps)}")
        logger.info(f"  Reference:         {len(ref_wp)}")
        logger.info(f"  Count ratio:       {score['count_ratio']:.3f}")
        logger.info(f"  Mean error:        {score['mean_error_m']:.3f} m")
        logger.info(f"  Max error:         {score['max_error_m']:.3f} m")
        logger.info(f"  Mean X diff:       {score['mean_x_diff']:.3f} m")
        logger.info(f"  Mean Y diff:       {score['mean_y_diff']:.3f} m")
        logger.info(f"{'=' * 60}\n")

        # Assertions — generous thresholds, the point is to measure
        assert len(our_wps) > 0, "FAR planner produced no waypoints"
        assert score["mean_error_m"] < 5.0, (
            f"Mean waypoint error {score['mean_error_m']:.2f}m exceeds 5m threshold"
        )
