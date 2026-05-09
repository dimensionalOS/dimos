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

"""Rosbag accuracy test: replays inputs at original timing and compares paths to ROS reference."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.nav_msgs.Path import Path as NavPath
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

_PROCESS_STARTUP_SEC = 2.0
_POST_FEED_DRAIN_SEC = 2.0

LOCAL_PLANNER_BIN = Path(__file__).parent / "result" / "bin" / "local_planner"

# LCM topics
SCAN_LCM = "/rblp_scan#sensor_msgs.PointCloud2"
ODOM_LCM = "/rblp_odom#nav_msgs.Odometry"
TERRAIN_LCM = "/rblp_terrain#sensor_msgs.PointCloud2"
WAYPOINT_LCM = "/rblp_wp#geometry_msgs.PointStamped"
PATH_LCM = "/rblp_path#nav_msgs.Path"
CMD_VEL_LCM = "/rblp_cmd#geometry_msgs.Twist"
SLOW_DOWN_LCM = "/rblp_slow#std_msgs.Int8"
GOAL_REACHED_LCM = "/rblp_reached#std_msgs.Bool"


def _local_planner_args() -> list[str]:
    return [
        "--registered_scan",
        SCAN_LCM,
        "--odometry",
        ODOM_LCM,
        "--terrain_map",
        TERRAIN_LCM,
        "--way_point",
        WAYPOINT_LCM,
        "--path",
        PATH_LCM,
        "--effective_cmd_vel",
        CMD_VEL_LCM,
        "--slow_down",
        SLOW_DOWN_LCM,
        "--goal_reached",
        GOAL_REACHED_LCM,
        # Exact OG nav stack runtime params (from params.txt dump)
        "--maxSpeed",
        "0.75",
        "--autonomySpeed",
        "0.75",
        "--autonomyMode",
        "true",
        "--useTerrainAnalysis",
        "true",
        "--checkObstacle",
        "true",
        "--checkRotObstacle",
        "false",
        "--obstacleHeightThre",
        "0.1",
        "--groundHeightThre",
        "0.1",
        "--costHeightThre1",
        "0.1",
        "--costHeightThre2",
        "0.05",
        "--maxRelZ",
        "0.3",
        "--minRelZ",
        "-0.4",
        "--goalClearance",
        "0.6",
        "--goalReachedThreshold",
        "0.3",
        "--goalBehindRange",
        "0.8",
        "--goalYawThreshold",
        "0.15",
        "--freezeAng",
        "90.0",
        "--freezeTime",
        "0.0",
        "--twoWayDrive",
        "true",
        "--vehicleLength",
        "0.5",
        "--vehicleWidth",
        "0.5",
        "--pathScale",
        "0.875",
        "--minPathScale",
        "0.675",
        "--pathScaleStep",
        "0.1",
        "--pathScaleBySpeed",
        "true",
        "--minPathRange",
        "0.8",
        "--pathRangeStep",
        "0.6",
        "--pathRangeBySpeed",
        "true",
        "--pathCropByGoal",
        "true",
        "--adjacentRange",
        "3.5",
        "--laserVoxelSize",
        "0.05",
        "--terrainVoxelSize",
        "0.2",
        "--dirWeight",
        "0.02",
        "--dirThre",
        "90.0",
        "--dirToVehicle",
        "false",
        "--omniDirGoalThre",
        "0.5",
        "--pointPerPathThre",
        "2",
        "--slowPathNumThre",
        "5",
        "--slowGroupNumThre",
        "1",
        "--useCost",
        "false",
    ]


def _compute_path_deviation(
    our_paths: list[NavPath], ref_endpoints: np.ndarray
) -> dict[str, float]:
    """Loss function: nearest-endpoint error + arc-length ratio vs reference."""
    if len(our_paths) == 0 or len(ref_endpoints) == 0:
        return {
            "mean_endpoint_error_m": float("inf"),
            "max_endpoint_error_m": float("inf"),
            "mean_length_ratio": 0.0,
            "count_ratio": 0.0,
            "multi_pose_ratio": 0.0,
        }

    # Our path endpoints and arc lengths
    our_endpoints = []
    our_arc_lengths = []
    for path_msg in our_paths:
        poses = path_msg.poses
        if len(poses) > 1:
            last = poses[-1]
            arc = sum(
                np.sqrt(
                    (poses[j].position.x - poses[j - 1].position.x) ** 2
                    + (poses[j].position.y - poses[j - 1].position.y) ** 2
                )
                for j in range(1, len(poses))
            )
            our_endpoints.append([last.position.x, last.position.y])
            our_arc_lengths.append(arc)

    multi_pose_ratio = len(our_endpoints) / len(our_paths) if our_paths else 0.0

    if len(our_endpoints) == 0:
        return {
            "mean_endpoint_error_m": float("inf"),
            "max_endpoint_error_m": float("inf"),
            "mean_length_ratio": 0.0,
            "count_ratio": len(our_paths) / len(ref_endpoints),
            "multi_pose_ratio": 0.0,
        }

    our_ep = np.array(our_endpoints)
    ref_ep = ref_endpoints[ref_endpoints[:, 1] > 1]  # Only multi-pose reference paths
    ref_xy = ref_ep[:, 2:4]  # last_x, last_y
    ref_arcs = ref_ep[:, 4]

    # For each reference endpoint, find nearest our endpoint
    endpoint_errors = []
    for ref_pt in ref_xy:
        dists = np.sqrt((our_ep[:, 0] - ref_pt[0]) ** 2 + (our_ep[:, 1] - ref_pt[1]) ** 2)
        endpoint_errors.append(float(dists.min()))

    # Arc length comparison
    our_mean_arc = float(np.mean(our_arc_lengths)) if our_arc_lengths else 0.0
    ref_mean_arc = float(ref_arcs.mean()) if len(ref_arcs) > 0 else 1.0
    length_ratio = our_mean_arc / ref_mean_arc if ref_mean_arc > 0 else 0.0

    return {
        "mean_endpoint_error_m": float(np.mean(endpoint_errors)),
        "max_endpoint_error_m": float(np.max(endpoint_errors)),
        "mean_length_ratio": length_ratio,
        "count_ratio": len(our_paths) / len(ref_endpoints),
        "multi_pose_ratio": multi_pose_ratio,
    }


class TestLocalPlannerRosbag:
    """Validate LocalPlanner accuracy against OG nav stack recording."""

    def test_path_accuracy(self) -> None:
        """Feed identical inputs at original timing and compare path output."""
        if not LOCAL_PLANNER_BIN.exists():
            pytest.skip(f"LocalPlanner binary not found: {LOCAL_PLANNER_BIN}")

        window = load_rosbag_window()
        ref_paths = window.path_endpoints
        assert len(ref_paths) > 0, "No reference path data in fixture"

        lcm = lcmlib.LCM()
        path_collector = LcmCollector(topic=PATH_LCM, msg_type=NavPath)
        path_collector.start(lcm)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm, stop_event), daemon=True
        )
        handle_thread.start()

        runner = NativeProcessRunner(binary_path=str(LOCAL_PLANNER_BIN), args=_local_planner_args())

        try:
            runner.start()
            assert runner.is_running, "LocalPlanner binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            # Feed at original timing
            feed_at_original_timing(
                lcm,
                window,
                topic_map={
                    "odom": ODOM_LCM,
                    "scan": SCAN_LCM,
                    "terrain": TERRAIN_LCM,
                    "waypoint": WAYPOINT_LCM,
                },
            )

            time.sleep(_POST_FEED_DRAIN_SEC)

        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            path_collector.stop(lcm)

        # Compute deviation score
        score = _compute_path_deviation(path_collector.messages, ref_paths)

        logger.info(f"\n{'=' * 60}")
        logger.info("LOCAL PLANNER DEVIATION SCORE")
        logger.info(f"  Our paths:          {len(path_collector.messages)}")
        logger.info(f"  Reference paths:    {len(ref_paths)}")
        logger.info(f"  Count ratio:        {score['count_ratio']:.3f}")
        logger.info(f"  Multi-pose ratio:   {score['multi_pose_ratio']:.3f}")
        logger.info(f"  Mean endpoint err:  {score['mean_endpoint_error_m']:.3f} m")
        logger.info(f"  Max endpoint err:   {score['max_endpoint_error_m']:.3f} m")
        logger.info(f"  Mean length ratio:  {score['mean_length_ratio']:.3f}")
        logger.info(f"{'=' * 60}\n")

        # Assertions
        assert len(path_collector.messages) > 0, "LocalPlanner produced no paths"
        assert score["multi_pose_ratio"] > 0, "No multi-pose paths produced"
        assert score["mean_endpoint_error_m"] < 2.0, (
            f"Mean endpoint error {score['mean_endpoint_error_m']:.2f}m exceeds 2m threshold"
        )
