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

"""Rosbag accuracy test: PathFollower is a pure (path, odom, slow_down) -> cmd_vel function,
so given identical inputs and matching parameters its output should be near-exact."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
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

_PROCESS_STARTUP_SEC = 1.0
_POST_FEED_DRAIN_SEC = 2.0

PATH_FOLLOWER_BIN = (
    Path(__file__).parent.parent / "modules" / "path_follower" / "result" / "bin" / "path_follower"
)

# LCM topics
PATH_LCM = "/rbpf_path#nav_msgs.Path"
ODOM_LCM = "/rbpf_odom#nav_msgs.Odometry"
CMD_VEL_LCM = "/rbpf_cmd#geometry_msgs.Twist"
SLOW_DOWN_LCM = "/rbpf_slow#std_msgs.Int8"
SAFETY_STOP_LCM = "/rbpf_safety#std_msgs.Int8"

# OG nav stack G1 config values (from unitree_g1.yaml)
# Exact OG nav stack runtime params (from params.txt dump)
OG_PATHFOLLOWER_ARGS = [
    "--lookAheadDis",
    "0.5",
    "--maxSpeed",
    "0.75",
    "--autonomySpeed",
    "0.75",
    "--maxAccel",
    "1.5",
    "--maxYawRate",
    "40.0",
    "--yawRateGain",
    "1.5",
    "--stopYawRateGain",
    "1.5",
    "--goalYawGain",
    "2.0",
    "--slowDwnDisThre",
    "0.875",
    "--dirDiffThre",
    "0.4",
    "--stopDisThre",
    "0.4",
    "--omniDirGoalThre",
    "0.5",
    "--omniDirDiffThre",
    "1.5",
    "--twoWayDrive",
    "false",  # OG runtime value (not omniDir default)
    "--switchTimeThre",
    "1.0",
    "--autonomyMode",
    "true",  # Set true at runtime by cross_wall_test.py
    "--pubSkipNum",
    "1",
    "--noRotAtGoal",
    "false",  # OG default
    "--noRotAtStop",
    "false",  # OG default
    "--slowRate1",
    "0.25",
    "--slowRate2",
    "0.5",
    "--slowRate3",
    "0.75",
    "--slowTime1",
    "2.0",
    "--slowTime2",
    "2.0",
]


class TestPathFollowerRosbag:
    """Validate PathFollower accuracy against OG nav stack recording."""

    def test_cmd_vel_accuracy(self) -> None:
        """Feed path + odom at original timing and compare cmd_vel."""
        if not PATH_FOLLOWER_BIN.exists():
            pytest.skip(f"PathFollower binary not found: {PATH_FOLLOWER_BIN}")

        window = load_rosbag_window()
        ref_cmd = window.cmd_vel
        assert len(ref_cmd) > 0, "No reference cmd_vel in fixture"

        lcm = lcmlib.LCM()
        cmd_collector = LcmCollector(topic=CMD_VEL_LCM, msg_type=Twist)
        cmd_collector.start(lcm)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm, stop_event), daemon=True
        )
        handle_thread.start()

        runner = NativeProcessRunner(
            binary_path=str(PATH_FOLLOWER_BIN),
            args=[
                "--path",
                PATH_LCM,
                "--odometry",
                ODOM_LCM,
                "--slow_down",
                SLOW_DOWN_LCM,
                "--safety_stop",
                SAFETY_STOP_LCM,
                "--cmd_vel",
                CMD_VEL_LCM,
                *OG_PATHFOLLOWER_ARGS,
            ],
        )

        try:
            runner.start()
            assert runner.is_running, "PathFollower binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            # Feed path + odom from the rosbag at original timing.
            # PathFollower subscribes to /path (LocalPlanner output) and /odometry.
            feed_at_original_timing(
                lcm,
                window,
                topic_map={
                    "odom": ODOM_LCM,
                    "path": PATH_LCM,
                },
                odom_subsample=1,
            )

            time.sleep(_POST_FEED_DRAIN_SEC)

        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            cmd_collector.stop(lcm)

        our_cmds = [(msg.linear.x, msg.linear.y, msg.angular.z) for msg in cmd_collector.messages]

        ref_nonzero = ref_cmd[np.abs(ref_cmd[:, 1]) > 0.01]
        our_nonzero = [c for c in our_cmds if abs(c[0]) > 0.01 or abs(c[1]) > 0.01]

        ref_mean_speed = (
            float(np.sqrt(ref_nonzero[:, 1] ** 2 + ref_nonzero[:, 2] ** 2).mean())
            if len(ref_nonzero) > 0
            else 0.0
        )
        our_mean_speed = (
            float(np.mean([np.sqrt(lx**2 + ly**2) for lx, ly, _ in our_nonzero]))
            if our_nonzero
            else 0.0
        )
        speed_ratio = our_mean_speed / ref_mean_speed if ref_mean_speed > 0 else 0.0

        # Speed comparison at multiple levels
        ref_speeds = (
            np.sqrt(ref_nonzero[:, 1] ** 2 + ref_nonzero[:, 2] ** 2)
            if len(ref_nonzero) > 0
            else np.array([])
        )
        our_speeds = (
            np.array([np.sqrt(lx**2 + ly**2) for lx, ly, _ in our_nonzero])
            if our_nonzero
            else np.array([])
        )

        # Steady-state comparison: filter to speeds > 0.5 m/s (fully in autonomy,
        # past acceleration ramp, not in joy-gated zero phase)
        ref_steady = ref_speeds[ref_speeds > 0.5] if len(ref_speeds) > 0 else np.array([])
        our_steady = our_speeds[our_speeds > 0.5] if len(our_speeds) > 0 else np.array([])
        steady_ratio = (
            float(our_steady.mean() / ref_steady.mean())
            if len(ref_steady) > 0 and len(our_steady) > 0
            else 0.0
        )

        logger.info(f"\n{'=' * 60}")
        logger.info("PATH FOLLOWER DEVIATION SCORE")
        logger.info(f"  Our cmd_vel:        {len(our_cmds)}")
        logger.info(f"  Reference:          {len(ref_cmd)}")
        logger.info(f"  Count ratio:        {len(our_cmds) / len(ref_cmd):.3f}")
        logger.info(f"  Our non-zero:       {len(our_nonzero)}")
        logger.info(f"  Ref non-zero:       {len(ref_nonzero)}")
        logger.info(f"  Our mean speed:     {our_mean_speed:.3f} m/s")
        logger.info(f"  Ref mean speed:     {ref_mean_speed:.3f} m/s")
        logger.info(f"  Speed ratio (all):  {speed_ratio:.3f}")
        logger.info(f"  Steady-state ratio: {steady_ratio:.3f}  (>0.5 m/s only)")
        if len(our_speeds) > 0:
            logger.info(f"  Our max speed:      {our_speeds.max():.3f} m/s")
        if len(ref_speeds) > 0:
            logger.info(f"  Ref max speed:      {ref_speeds.max():.3f} m/s")
        logger.info(f"{'=' * 60}\n")

        assert len(our_cmds) > 0, "PathFollower produced no cmd_vel"
        assert len(our_nonzero) > 0, "All cmd_vel are zero"
        # Steady-state speeds should be within 15% of reference
        assert 0.8 < steady_ratio < 1.2, (
            f"Steady-state speed ratio {steady_ratio:.3f} outside [0.8, 1.2] tolerance"
        )
