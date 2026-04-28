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

"""Tests for the TF-tree-first transform system.

Validates:
  - Frame constants match REP-105
  - FastLio2 publishes odom→body TF from odometry
  - PGO publishes map→odom correction TF
  - SimplePlanner queries map→body via TF instead of Odometry stream
  - MovementManager queries map→body via TF instead of Odometry stream
  - BFS chain composition: map→odom + odom→body = map→body
  - Odometry remappings only apply to NativeModules
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, cast
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2Config, _odom_to_body_tf
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.navigation.nav_stack.modules.movement_manager.movement_manager import (
    MovementManager,
    MovementManagerConfig,
)
from dimos.navigation.nav_stack.modules.simple_planner.simple_planner import (
    Costmap,
    SimplePlanner,
    SimplePlannerConfig,
    resolve_tf_chain,
)
from dimos.protocol.tf.tf import MultiTBuffer

# PGO + create_nav_stack pull in gtsam; gate behind a try so the rest of the
# file is still importable without it. Tests that touch these are class- or
# module-level skipped via ``_has_gtsam`` below.
_has_gtsam: bool
try:
    import gtsam  # noqa: F401

    from dimos.navigation.nav_stack.main import create_nav_stack
    from dimos.navigation.nav_stack.modules.far_planner.far_planner import FarPlanner
    from dimos.navigation.nav_stack.modules.pgo.pgo import (
        PGO,
        PGOConfig,
        _SimplePGO,
        build_corrected_odometry,
        build_map_odom_tf,
    )
    from dimos.navigation.nav_stack.modules.terrain_analysis.terrain_analysis import TerrainAnalysis

    _has_gtsam = True
except ImportError:
    _has_gtsam = False


class TestFrameConstants:
    def test_frame_map(self):
        assert FRAME_MAP == "map"

    def test_frame_odom(self):
        assert FRAME_ODOM == "odom"

    def test_frame_body(self):
        assert FRAME_BODY == "body"


class TestTFChainComposition:
    """Verify that publishing odom→body and map→odom composes to map→body."""

    def _make_buffer(self) -> MultiTBuffer:
        return MultiTBuffer()

    def test_direct_lookup(self):
        buf = self._make_buffer()
        tf = Transform(
            frame_id=FRAME_ODOM,
            child_frame_id=FRAME_BODY,
            translation=Vector3(1.0, 2.0, 0.5),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        buf.receive_transform(tf)
        result = buf.get(FRAME_ODOM, FRAME_BODY)
        assert result is not None
        assert result.translation.x == pytest.approx(1.0)
        assert result.translation.y == pytest.approx(2.0)
        assert result.translation.z == pytest.approx(0.5)

    def test_chain_map_odom_body(self):
        """map→odom + odom→body should compose to map→body via BFS."""
        buf = self._make_buffer()
        now = time.time()

        # odom→body: robot at (1, 2, 0) in odom frame
        buf.receive_transform(
            Transform(
                frame_id=FRAME_ODOM,
                child_frame_id=FRAME_BODY,
                translation=Vector3(1.0, 2.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now,
            )
        )

        # map→odom: correction offset of (10, 20, 0) with identity rotation
        buf.receive_transform(
            Transform(
                frame_id=FRAME_MAP,
                child_frame_id=FRAME_ODOM,
                translation=Vector3(10.0, 20.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now,
            )
        )

        # BFS should find map→body
        result = buf.get(FRAME_MAP, FRAME_BODY)
        assert result is not None
        # With identity rotations, translations add up:
        # map→body = map→odom(10,20) + odom→body(1,2) = (11,22)
        assert result.translation.x == pytest.approx(11.0, abs=0.01)
        assert result.translation.y == pytest.approx(22.0, abs=0.01)

    def test_chain_with_rotation(self):
        """map→odom with 90° yaw + odom→body should rotate correctly."""
        buf = self._make_buffer()
        now = time.time()

        # odom→body: robot at (1, 0, 0) in odom frame, no rotation
        buf.receive_transform(
            Transform(
                frame_id=FRAME_ODOM,
                child_frame_id=FRAME_BODY,
                translation=Vector3(1.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now,
            )
        )

        # map→odom: 90° yaw rotation, no translation
        yaw_90 = Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2))
        buf.receive_transform(
            Transform(
                frame_id=FRAME_MAP,
                child_frame_id=FRAME_ODOM,
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=yaw_90,
                ts=now,
            )
        )

        result = buf.get(FRAME_MAP, FRAME_BODY)
        assert result is not None
        # odom→body (1,0) rotated 90° around Z → (0,1) in map frame
        assert result.translation.x == pytest.approx(0.0, abs=0.05)
        assert result.translation.y == pytest.approx(1.0, abs=0.05)

    def test_no_chain_returns_none(self):
        """Querying a frame that hasn't been published should return None."""
        buf = self._make_buffer()
        result = buf.get(FRAME_MAP, FRAME_BODY)
        assert result is None

    def test_partial_chain_returns_none(self):
        """Only odom→body published, map→body should return None."""
        buf = self._make_buffer()
        buf.receive_transform(
            Transform(
                frame_id=FRAME_ODOM,
                child_frame_id=FRAME_BODY,
                translation=Vector3(1.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=time.time(),
            )
        )
        result = buf.get(FRAME_MAP, FRAME_BODY)
        assert result is None

    def test_updates_reflect_latest(self):
        """Publishing a new transform should update the chain result."""
        buf = self._make_buffer()
        now = time.time()

        buf.receive_transform(
            Transform(
                frame_id=FRAME_MAP,
                child_frame_id=FRAME_ODOM,
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now,
            )
        )
        buf.receive_transform(
            Transform(
                frame_id=FRAME_ODOM,
                child_frame_id=FRAME_BODY,
                translation=Vector3(1.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now,
            )
        )

        r1 = buf.get(FRAME_MAP, FRAME_BODY)
        assert r1 is not None
        assert r1.translation.x == pytest.approx(1.0, abs=0.01)

        # Update odom→body
        buf.receive_transform(
            Transform(
                frame_id=FRAME_ODOM,
                child_frame_id=FRAME_BODY,
                translation=Vector3(5.0, 3.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ts=now + 0.1,
            )
        )

        r2 = buf.get(FRAME_MAP, FRAME_BODY)
        assert r2 is not None
        assert r2.translation.x == pytest.approx(5.0, abs=0.01)
        assert r2.translation.y == pytest.approx(3.0, abs=0.01)


class TestFastLio2TF:
    """Verify FastLio2 config defaults and TF callback logic."""

    def test_default_frame_id_is_odom(self):
        cfg = FastLio2Config()
        assert cfg.frame_id == FRAME_ODOM

    def test_default_child_frame_id_is_body(self):
        cfg = FastLio2Config()
        assert cfg.child_frame_id == FRAME_BODY

    def test_odom_to_body_tf_builds_transform(self):
        """_odom_to_body_tf should produce an odom→body Transform from an odometry msg."""
        odom = Odometry(
            ts=100.0,
            frame_id=FRAME_ODOM,
            child_frame_id=FRAME_BODY,
            pose=Pose(
                position=[3.0, 4.0, 0.5],
                orientation=[0.0, 0.0, 0.0, 1.0],
            ),
        )
        tf_arg = _odom_to_body_tf(odom)
        assert tf_arg.frame_id == FRAME_ODOM
        assert tf_arg.child_frame_id == FRAME_BODY
        assert tf_arg.translation.x == pytest.approx(3.0)
        assert tf_arg.translation.y == pytest.approx(4.0)
        assert tf_arg.translation.z == pytest.approx(0.5)
        assert tf_arg.ts == pytest.approx(100.0)


@pytest.mark.skipif(not _has_gtsam, reason="gtsam not installed")
class TestPGOTF:
    """Verify PGO publishes map→odom TF and corrected odometry uses correct frames."""

    def test_build_map_odom_tf(self):
        """build_map_odom_tf should produce a map→odom Transform from r/t."""

        r_offset = np.eye(3)
        t_offset = np.array([1.0, 2.0, 0.0])
        tf_arg = build_map_odom_tf(r_offset, t_offset, 42.0)
        assert tf_arg.frame_id == FRAME_MAP
        assert tf_arg.child_frame_id == FRAME_ODOM
        assert tf_arg.translation.x == pytest.approx(1.0)
        assert tf_arg.translation.y == pytest.approx(2.0)
        assert tf_arg.ts == pytest.approx(42.0)

    def test_build_corrected_odometry_uses_frame_constants(self):
        """build_corrected_odometry should use FRAME_MAP and FRAME_BODY."""

        r = np.eye(3)
        t = np.array([5.0, 6.0, 0.0])
        odom_msg = build_corrected_odometry(r, t, 99.0)
        assert odom_msg.frame_id == FRAME_MAP
        assert odom_msg.child_frame_id == FRAME_BODY

    def test_seed_initial_tf_publishes_identity(self):
        """PGO._seed_initial_tf should publish identity map→odom (called during start)."""
        # Use __new__ to avoid the full Module construction; the helper
        # only reads ``self._tf`` so we don't need any other state here.
        pgo_mod = cast("Any", PGO.__new__(PGO))
        pgo_mod._tf = MagicMock()

        pgo_mod._seed_initial_tf(123.0)

        pgo_mod.tf.publish.assert_called_once()
        tf_arg = pgo_mod.tf.publish.call_args[0][0]
        assert tf_arg.frame_id == FRAME_MAP
        assert tf_arg.child_frame_id == FRAME_ODOM
        assert tf_arg.translation.x == pytest.approx(0.0, abs=1e-6)
        assert tf_arg.translation.y == pytest.approx(0.0, abs=1e-6)
        assert tf_arg.rotation.w == pytest.approx(1.0, abs=1e-6)
        assert tf_arg.ts == pytest.approx(123.0)

    def test_process_scan_returns_odom_and_map_tf(self):
        """process_scan should return both a corrected odometry and a map→odom TF."""
        from dimos.navigation.nav_stack.modules.pgo.pgo import process_scan

        cfg = PGOConfig()
        pgo = _SimplePGO(cfg)

        pts = np.random.default_rng(42).standard_normal((100, 3)).astype(np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="map", timestamp=1.0)
        result = process_scan(
            pgo,
            cloud,
            r_local=np.eye(3),
            t_local=np.array([1.0, 2.0, 0.0]),
            ts=1.0,
            unregister_input=cfg.unregister_input,
        )

        assert result is not None
        odom_msg, tf_msg = result
        assert odom_msg.frame_id == FRAME_MAP
        assert odom_msg.child_frame_id == FRAME_BODY
        assert tf_msg.frame_id == FRAME_MAP
        assert tf_msg.child_frame_id == FRAME_ODOM

    def test_process_scan_empty_cloud_returns_none(self):
        """process_scan should return None for an empty point cloud."""
        from dimos.navigation.nav_stack.modules.pgo.pgo import process_scan

        cfg = PGOConfig()
        pgo = _SimplePGO(cfg)
        empty = PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32), "map", 0.0)
        result = process_scan(
            pgo,
            empty,
            r_local=np.eye(3),
            t_local=np.zeros(3),
            ts=0.0,
            unregister_input=cfg.unregister_input,
        )
        assert result is None


class TestSimplePlannerTF:
    """Verify SimplePlanner queries TF instead of subscribing to Odometry."""

    def _make_planner(self) -> Any:
        p = SimplePlanner.__new__(SimplePlanner)
        p.config = SimplePlannerConfig()
        p._lock = threading.Lock()
        p._costmap = Costmap(
            cell_size=p.config.cell_size,
            obstacle_height=p.config.obstacle_height_threshold,
            inflation_radius=p.config.inflation_radius,
        )
        p._robot_x = 0.0
        p._robot_y = 0.0
        p._robot_z = 0.0
        p._has_odom = False
        p._goal_x = None
        p._goal_y = None
        p._goal_z = 0.0
        p._ref_goal_dist = float("inf")
        p._last_progress_time = 0.0
        p._effective_inflation = p.config.inflation_radius
        p._cached_path = None
        p._last_plan_time = 0.0
        p._last_diag_print = 0.0
        p._last_costmap_pub = 0.0
        p._current_wp = None
        p._current_wp_is_goal = False
        p._running = False
        p._thread = None
        p._tf = MagicMock()
        p.way_point = MagicMock()
        p.goal_path = MagicMock()
        p.costmap_cloud = MagicMock()
        return p

    def test_no_odometry_port(self):
        """SimplePlanner should not have an odometry In stream."""

        # Check class annotations for In[Odometry]
        annotations = {}
        for cls in reversed(SimplePlanner.__mro__):
            annotations.update(getattr(cls, "__annotations__", {}))
        assert "odometry" not in annotations, "SimplePlanner should not have an 'odometry' port"

    def test_query_pose_updates_position(self):
        """_query_pose should update robot position from TF."""
        p = self._make_planner()

        tf_result = Transform(
            frame_id=FRAME_MAP,
            child_frame_id=FRAME_BODY,
            translation=Vector3(3.0, 4.0, 0.5),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        p.tf.get.return_value = tf_result

        result = p._query_pose()
        assert result is True
        assert p._has_odom is True
        assert p._robot_x == pytest.approx(3.0)
        assert p._robot_y == pytest.approx(4.0)
        assert p._robot_z == pytest.approx(0.5)

    def test_query_pose_returns_false_when_no_tf(self):
        """_query_pose should return False when both chains unavailable."""
        p = self._make_planner()
        p.tf.get.return_value = None

        result = p._query_pose()
        assert result is False
        assert p._has_odom is False

    def test_replan_once_queries_tf(self):
        """_replan_once should call _query_pose (which queries TF)."""
        p = self._make_planner()

        tf_result = Transform(
            frame_id=FRAME_MAP,
            child_frame_id=FRAME_BODY,
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        p.tf.get.return_value = tf_result

        # No goal set, so _replan_once should return early after querying TF
        p._replan_once()
        p.tf.get.assert_called_with(FRAME_MAP, FRAME_BODY)

    def test_waypoint_uses_frame_map(self):
        """Published waypoints should use FRAME_MAP as frame_id."""
        p = self._make_planner()

        p._has_odom = True
        p._goal_x = 5.0
        p._goal_y = 0.0
        p._goal_z = 0.0
        p._cached_path = [(x, 0.0) for x in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
        p._current_wp = (2.0, 0.0)
        p._current_wp_is_goal = False

        p._robot_x = 1.9
        p._robot_y = 0.0
        p._maybe_advance_waypoint(1.9, 0.0, 0.0)

        if p.way_point.publish.called:
            msg: PointStamped = p.way_point.publish.call_args[0][0]
            assert msg.frame_id == FRAME_MAP


class TestResolveTfChain:
    """resolve_tf_chain handles the (parent, child) priority list."""

    def test_returns_first_available(self):
        """First chain that returns non-None wins."""
        odom_tf = Transform(
            frame_id=FRAME_ODOM,
            child_frame_id=FRAME_BODY,
            translation=Vector3(1.0, 2.0, 0.3),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        tf_buffer = MagicMock()
        tf_buffer.get.side_effect = lambda p, c: None if p == FRAME_MAP else odom_tf
        result = resolve_tf_chain(tf_buffer, [(FRAME_MAP, FRAME_BODY), (FRAME_ODOM, FRAME_BODY)])
        assert result is odom_tf

    def test_returns_none_when_all_chains_empty(self):
        tf_buffer = MagicMock()
        tf_buffer.get.return_value = None
        result = resolve_tf_chain(tf_buffer, [(FRAME_MAP, FRAME_BODY), (FRAME_ODOM, FRAME_BODY)])
        assert result is None

    def test_first_match_wins(self):
        """Earlier query wins over later one when both have transforms."""
        first = Transform(
            frame_id=FRAME_MAP,
            child_frame_id=FRAME_BODY,
            translation=Vector3(7.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        second = Transform(
            frame_id=FRAME_ODOM,
            child_frame_id=FRAME_BODY,
            translation=Vector3(99.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        tf_buffer = MagicMock()
        tf_buffer.get.side_effect = lambda p, c: first if p == FRAME_MAP else second
        result = resolve_tf_chain(tf_buffer, [(FRAME_MAP, FRAME_BODY), (FRAME_ODOM, FRAME_BODY)])
        assert result is first


class TestWaypointAdvance:
    """Verify the waypoint advance logic prevents stopping on intermediate waypoints."""

    def _make_planner(self) -> Any:
        p = SimplePlanner.__new__(SimplePlanner)
        p.config = SimplePlannerConfig(
            lookahead_distance=2.0,
            waypoint_advance_radius=1.0,
        )
        p._lock = threading.Lock()
        p._costmap = Costmap(cell_size=0.3, obstacle_height=0.15, inflation_radius=0.2)
        p._cached_path = [(x, 0.0) for x in range(20)]
        p._current_wp = (4.0, 0.0)
        p._current_wp_is_goal = False
        p.way_point = MagicMock()
        p._tf = MagicMock()
        return p

    def test_advance_when_close(self):
        """Waypoint should advance when robot is within advance radius."""
        p = self._make_planner()
        # Robot is at (3.5, 0), waypoint is at (4.0, 0) — distance = 0.5 < 1.0
        p._maybe_advance_waypoint(3.5, 0.0, 0.0)
        p.way_point.publish.assert_called_once()
        # New waypoint should be further ahead
        msg: PointStamped = p.way_point.publish.call_args[0][0]
        assert msg.x > 4.0

    def test_no_advance_when_far(self):
        """Waypoint should NOT advance when robot is outside advance radius."""
        p = self._make_planner()
        # Robot is at (1.0, 0), waypoint is at (4.0, 0) — distance = 3.0 > 1.0
        p._maybe_advance_waypoint(1.0, 0.0, 0.0)
        p.way_point.publish.assert_not_called()

    def test_no_advance_at_goal(self):
        """Waypoint should NOT advance when it IS the final goal."""
        p = self._make_planner()
        p._current_wp = (19.0, 0.0)  # last point in path
        p._current_wp_is_goal = True
        p._maybe_advance_waypoint(18.5, 0.0, 0.0)
        p.way_point.publish.assert_not_called()

    def test_no_advance_without_cached_path(self):
        """Waypoint should NOT advance when there's no cached path."""
        p = self._make_planner()
        p._cached_path = None
        p._maybe_advance_waypoint(3.5, 0.0, 0.0)
        p.way_point.publish.assert_not_called()

    def test_advance_sets_goal_flag_at_end(self):
        """When advancing reaches the end of the path, is_goal should be True."""
        p = self._make_planner()
        # Short path where advance reaches the end
        p._cached_path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        p._current_wp = (1.0, 0.0)
        p._current_wp_is_goal = False
        # Robot close to waypoint
        p._maybe_advance_waypoint(0.5, 0.0, 0.0)
        # Extended lookahead = 2.0 * 1.5 = 3.0, path ends at (2, 0)
        # so waypoint should be (2, 0) = last = goal
        assert p._current_wp == (2.0, 0.0)
        assert p._current_wp_is_goal is True

    def test_advance_uses_extended_lookahead(self):
        """Advanced waypoint should use 1.5x the normal lookahead."""
        p = self._make_planner()
        p.config.lookahead_distance = 2.0
        # Robot at (3.5, 0), close to waypoint at (4.0, 0)
        # Extended lookahead = 3.0, from robot at 3.5 → should pick point ≥ 3.0m away
        # That's (7.0, 0.0) or further (6.5 is 3.0 away from 3.5)
        p._maybe_advance_waypoint(3.5, 0.0, 0.0)
        if p.way_point.publish.called:
            msg = p.way_point.publish.call_args[0][0]
            dist = math.hypot(msg.x - 3.5, msg.y - 0.0)
            assert dist >= 3.0 - 0.5  # allow for cell discretization


class TestMovementManagerTF:
    """Verify MovementManager queries TF instead of subscribing to Odometry."""

    def _make_mgr(self) -> Any:
        # MovementManager.__init__ pulls the full Module lifecycle which we
        # don't want to spin up for unit tests. Construct via __new__ and
        # set up the fields the methods under test actually read.
        mgr = cast("Any", MovementManager.__new__(MovementManager))
        mgr.config = MovementManagerConfig()
        mgr._lock = threading.Lock()
        mgr._teleop_active = False
        mgr._timer = None
        mgr._timer_gen = 0
        mgr._robot_x = 0.0
        mgr._robot_y = 0.0
        mgr._robot_z = 0.0
        mgr.cmd_vel = MagicMock()
        mgr.stop_movement = MagicMock()
        mgr.goal = MagicMock()
        mgr.way_point = MagicMock()
        mgr._tf = MagicMock()
        return mgr

    def test_no_odometry_port(self):
        """MovementManager should not have an odometry In stream."""
        annotations = {}
        for cls in reversed(MovementManager.__mro__):
            annotations.update(getattr(cls, "__annotations__", {}))
        assert "odometry" not in annotations, "MovementManager should not have an 'odometry' port"

    def test_cancel_goal_uses_frame_constant(self):
        """_cancel_goal should use FRAME_MAP for the NaN sentinel."""
        mgr = self._make_mgr()
        mgr._cancel_goal()

        assert mgr.goal.publish.call_count == 1
        cancel_msg: PointStamped = mgr.goal.publish.call_args[0][0]
        assert cancel_msg.frame_id == FRAME_MAP
        assert math.isnan(cancel_msg.x)


@pytest.mark.skipif(
    not _has_gtsam, reason="gtsam not installed (PGO is wired into create_nav_stack)"
)
class TestSmartNavRemappings:
    """Verify that odometry remappings only apply to NativeModules."""

    def test_simple_planner_no_odometry_remapping(self):
        bp = create_nav_stack(use_simple_planner=True)
        rmap = bp.remapping_map
        assert (SimplePlanner, "odometry") not in rmap, (
            "SimplePlanner should not have an odometry remapping"
        )

    def test_movement_manager_no_odometry_remapping(self):
        bp = create_nav_stack(use_simple_planner=True)
        rmap = bp.remapping_map
        assert (MovementManager, "odometry") not in rmap, (
            "MovementManager should not have an odometry remapping"
        )

    def test_terrain_analysis_still_remapped(self):
        bp = create_nav_stack(use_simple_planner=True)
        rmap = bp.remapping_map
        assert (TerrainAnalysis, "odometry") in rmap
        assert rmap[(TerrainAnalysis, "odometry")] == "corrected_odometry"

    def test_far_planner_remapped_when_active(self):
        bp = create_nav_stack(use_simple_planner=False)
        rmap = bp.remapping_map
        assert (FarPlanner, "odometry") in rmap
        assert rmap[(FarPlanner, "odometry")] == "corrected_odometry"


@pytest.mark.skipif(not _has_gtsam, reason="gtsam not installed")
class TestPGOCorrectionToTF:
    """Verify PGO's R/t offset correctly maps to a TF transform."""

    def test_identity_correction(self):
        tf_arg = build_map_odom_tf(np.eye(3), np.zeros(3), 1.0)
        assert tf_arg.translation.x == pytest.approx(0.0, abs=1e-6)
        assert tf_arg.translation.y == pytest.approx(0.0, abs=1e-6)
        assert tf_arg.translation.z == pytest.approx(0.0, abs=1e-6)
        assert tf_arg.rotation.w == pytest.approx(1.0, abs=1e-6)

    def test_translation_correction(self):
        tf_arg = build_map_odom_tf(np.eye(3), np.array([0.5, -0.3, 0.0]), 1.0)
        assert tf_arg.translation.x == pytest.approx(0.5, abs=1e-6)
        assert tf_arg.translation.y == pytest.approx(-0.3, abs=1e-6)

    def test_rotation_correction(self):
        yaw = math.pi / 6  # 30°
        r_offset = Rotation.from_euler("z", yaw).as_matrix()
        tf_arg = build_map_odom_tf(r_offset, np.zeros(3), 1.0)
        q = [tf_arg.rotation.x, tf_arg.rotation.y, tf_arg.rotation.z, tf_arg.rotation.w]
        recovered_yaw = Rotation.from_quat(q).as_euler("xyz")[2]
        assert recovered_yaw == pytest.approx(yaw, abs=1e-4)
