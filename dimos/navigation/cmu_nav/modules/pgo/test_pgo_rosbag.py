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

"""Rosbag accuracy test: replays scan+odom at original timing, validates PGO outputs."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.modules.pgo.pgo import PGOConfig
from dimos.navigation.cmu_nav.tests.deprecation import SKIP_CMU_NAV
from dimos.navigation.cmu_nav.tests.rosbag_fixtures import (
    LcmCollector,
    NativeProcessRunner,
    feed_at_original_timing,
    lcm_handle_loop,
    load_rosbag_window,
    make_odometry_msg,
    make_pointcloud_msg,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.self_hosted, SKIP_CMU_NAV]

_PROCESS_STARTUP_SEC = 2.0
_POST_FEED_DRAIN_SEC = 3.0
_BURST_DRAIN_SEC = 2.0

PGO_BIN = Path(__file__).parent / "cpp" / "result" / "bin" / "pgo"

# LCM topic names for this test (prefixed to avoid collision)
SCAN_LCM = "/rbpgo_scan#sensor_msgs.PointCloud2"
ODOM_LCM = "/rbpgo_odom#nav_msgs.Odometry"
CORRECTED_ODOM_LCM = "/rbpgo_corr_odom#nav_msgs.Odometry"
GLOBAL_MAP_LCM = "/rbpgo_global_map#sensor_msgs.PointCloud2"
TF_LCM = "/rbpgo_tf#nav_msgs.Odometry"

BURST_SCAN_LCM = "/burstpgo_scan#sensor_msgs.PointCloud2"
BURST_ODOM_LCM = "/burstpgo_odom#nav_msgs.Odometry"
BURST_CORRECTED_LCM = "/burstpgo_corr_odom#nav_msgs.Odometry"
BURST_GLOBAL_MAP_LCM = "/burstpgo_global_map#sensor_msgs.PointCloud2"
BURST_TF_LCM = "/burstpgo_tf#nav_msgs.Odometry"


def _stdin_blob(topics: dict[str, str]) -> bytes:
    """The line NativeModule.start() would write, built from PGOConfig itself.

    Deriving the config means a field added or renamed on either side of the
    boundary fails here rather than drifting.
    """
    blob = {"topics": topics, "config": PGOConfig().to_config_dict()}
    return json.dumps(blob).encode() + b"\n"


class TestPGORosbag:
    """Validate PGO native module accuracy against OG nav stack recording."""

    def test_pgo_corrected_odometry(self) -> None:
        """Feed scan + odom at original timing and validate PGO outputs.

        Checks:
        - PGO produces corrected odometry messages
        - Corrected odometry tracks the input trajectory (no wild divergence)
        - Global map is published with non-zero points
        - TF corrections are published
        """
        if not PGO_BIN.exists():
            pytest.skip(f"PGO binary not found: {PGO_BIN}")

        window = load_rosbag_window()
        assert len(window.scans) > 0, "No scans in rosbag fixture"
        assert len(window.odom) > 0, "No odometry in rosbag fixture"

        lcm_instance = lcmlib.LCM()

        corrected_odom_collector = LcmCollector(topic=CORRECTED_ODOM_LCM, msg_type=Odometry)
        global_map_collector = LcmCollector(topic=GLOBAL_MAP_LCM, msg_type=PointCloud2)
        tf_collector = LcmCollector(topic=TF_LCM, msg_type=Odometry)

        corrected_odom_collector.start(lcm_instance)
        global_map_collector.start(lcm_instance)
        tf_collector.start(lcm_instance)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True
        )
        handle_thread.start()

        # The port reads topics and config from one JSON stdin line and picks
        # its transport from DIMOS_TRANSPORT, matching NativeModule.start().
        stdin_blob = _stdin_blob(
            {
                "registered_scan": SCAN_LCM,
                "odometry": ODOM_LCM,
                "corrected_odometry": CORRECTED_ODOM_LCM,
                "global_map": GLOBAL_MAP_LCM,
                "pgo_tf": TF_LCM,
            }
        )

        runner = NativeProcessRunner(
            binary_path=str(PGO_BIN),
            stdin_blob=stdin_blob,
            env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
        )

        try:
            runner.start(capture_stderr=True)
            assert runner.is_running, "PGO binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            feed_at_original_timing(
                lcm_instance,
                window,
                topic_map={
                    "odom": ODOM_LCM,
                    "scan": SCAN_LCM,
                },
            )

            time.sleep(_POST_FEED_DRAIN_SEC)

        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            corrected_odom_collector.stop(lcm_instance)
            global_map_collector.stop(lcm_instance)
            tf_collector.stop(lcm_instance)

        # -- Analysis --
        corrected_count = len(corrected_odom_collector.messages)
        global_map_count = len(global_map_collector.messages)
        tf_count = len(tf_collector.messages)

        logger.info(f"\n{'=' * 60}")
        logger.info("PGO NATIVE ROSBAG DEVIATION SCORE")
        logger.info(f"  Input scans:            {len(window.scans)}")
        logger.info(f"  Input odom messages:     {len(window.odom)}")
        logger.info(f"  Corrected odom outputs:  {corrected_count}")
        logger.info(f"  Global map outputs:      {global_map_count}")
        logger.info(f"  TF outputs:              {tf_count}")

        # Basic output checks
        assert corrected_count > 0, f"PGO produced no corrected odometry{runner.exit_report()}"
        assert global_map_count > 0, f"PGO produced no global map messages{runner.exit_report()}"
        assert tf_count > 0, f"PGO produced no TF messages{runner.exit_report()}"

        # Extract corrected trajectory
        corrected_positions = np.array(
            [
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                for msg in corrected_odom_collector.messages
            ]
        )

        # Extract input trajectory (subsample to match)
        input_positions = window.odom[:, 1:4]

        # Corrected trajectory should be spatially close to input (no loop closures
        # expected in 60s recording, so correction should be near-identity)
        corrected_centroid = corrected_positions.mean(axis=0)
        input_centroid = input_positions.mean(axis=0)
        centroid_error = float(np.linalg.norm(corrected_centroid - input_centroid))

        # Check trajectory extent (PGO shouldn't collapse trajectory to a point)
        corrected_extent = corrected_positions.max(axis=0) - corrected_positions.min(axis=0)
        input_extent = input_positions.max(axis=0) - input_positions.min(axis=0)
        extent_ratio_xy = float(
            np.linalg.norm(corrected_extent[:2]) / max(np.linalg.norm(input_extent[:2]), 1e-6)
        )

        # Check global map point count
        global_map_point_counts = []
        for msg in global_map_collector.messages:
            points, _ = msg.as_numpy()
            if points is not None:
                global_map_point_counts.append(len(points))

        mean_map_points = (
            float(np.mean(global_map_point_counts)) if global_map_point_counts else 0.0
        )
        last_map_points = global_map_point_counts[-1] if global_map_point_counts else 0

        # TF should be near-identity for a short recording without loop closures
        last_tf = tf_collector.messages[-1]
        tf_translation_norm = float(
            np.linalg.norm(
                [
                    last_tf.pose.position.x,
                    last_tf.pose.position.y,
                    last_tf.pose.position.z,
                ]
            )
        )

        logger.info(f"  Centroid error:           {centroid_error:.3f} m")
        logger.info(f"  Extent ratio (XY):        {extent_ratio_xy:.3f}")
        logger.info(f"  Mean global map points:   {mean_map_points:.0f}")
        logger.info(f"  Last global map points:   {last_map_points}")
        logger.info(f"  Final TF translation:     {tf_translation_norm:.4f} m")
        logger.info(f"{'=' * 60}\n")

        # Assertions
        assert centroid_error < 5.0, (
            f"Corrected trajectory centroid too far from input: {centroid_error:.3f} m"
        )
        assert extent_ratio_xy > 0.5, (
            f"Corrected trajectory collapsed (extent ratio {extent_ratio_xy:.3f})"
        )
        assert extent_ratio_xy < 2.0, (
            f"Corrected trajectory exploded (extent ratio {extent_ratio_xy:.3f})"
        )
        assert last_map_points > 0, "Final global map has zero points"
        assert mean_map_points > 100, f"Global map too sparse: mean {mean_map_points:.0f} points"

    def test_pgo_stays_current_under_a_scan_burst(self) -> None:
        """A burst faster than the PGO round collapses to the newest scan.

        Clouds are tiny so this measures the collapse, not what a UDP socket
        can absorb.
        """
        if not PGO_BIN.exists():
            pytest.skip(f"PGO binary not found: {PGO_BIN}")

        burst = 60
        cloud = np.tile(np.array([[1.0, 0.0, 0.0, 10.0]], dtype=np.float32), (32, 1))

        lcm_instance = lcmlib.LCM()
        corrected = LcmCollector(topic=BURST_CORRECTED_LCM, msg_type=Odometry)
        corrected.start(lcm_instance)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm_instance, stop_event), daemon=True
        )
        handle_thread.start()

        runner = NativeProcessRunner(
            binary_path=str(PGO_BIN),
            stdin_blob=_stdin_blob(
                {
                    "registered_scan": BURST_SCAN_LCM,
                    "odometry": BURST_ODOM_LCM,
                    "corrected_odometry": BURST_CORRECTED_LCM,
                    "global_map": BURST_GLOBAL_MAP_LCM,
                    "pgo_tf": BURST_TF_LCM,
                }
            ),
            env={**os.environ, "DIMOS_TRANSPORT": "lcm"},
        )

        try:
            runner.start(capture_stderr=True)
            assert runner.is_running, "PGO binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            # Walk forward a metre per step so each scan carries a distinct,
            # increasing stamp and PGO's out-of-order gate accepts every one.
            for i in range(burst):
                ts = 1000.0 + i * 0.1
                lcm_instance.publish(
                    BURST_ODOM_LCM,
                    make_odometry_msg(
                        np.array([float(i), 0.0, 0.0]),
                        np.array([0.0, 0.0, 0.0, 1.0]),
                        ts=ts,
                    ).lcm_encode(),
                )
                lcm_instance.publish(BURST_SCAN_LCM, make_pointcloud_msg(cloud, ts=ts).lcm_encode())

            time.sleep(_BURST_DRAIN_SEC)
        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            corrected.stop(lcm_instance)

        published = len(corrected.messages)
        assert published > 0, (
            f"PGO produced no corrected odometry for the burst{runner.exit_report()}"
        )

        # Each scan was stamped with a pose one metre further along, so the x of
        # the last correction says which scan PGO finished on.
        reached = corrected.messages[-1].x

        # One round processes one scan, so a collapsing module publishes far
        # fewer than it received. A FIFO queue would work through every one.
        assert published < burst // 2, (
            f"PGO published {published} of {burst} burst scans, so the backlog was not shed"
        )
        # It ended on the newest scan fed, not on one buried in a backlog.
        assert reached >= burst - 2, (
            f"PGO's last correction is at x={reached:.1f}, behind the final scan at x={burst - 1}"
        )
