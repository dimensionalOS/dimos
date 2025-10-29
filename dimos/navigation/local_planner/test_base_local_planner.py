#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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

"""
Unit tests for the GLAP (Gradient-Augmented Look-Ahead Pursuit) holonomic local planner.
"""

import numpy as np
import pytest

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner


class TestHolonomicLocalPlanner:
    """Test suite for HolonomicLocalPlanner."""

    @pytest.fixture
    def planner(self):
        """Create a planner instance for testing."""
        planner = HolonomicLocalPlanner(
            lookahead_dist=1.5,
            k_rep=1.0,
            alpha=1.0,  # No filtering for deterministic tests
            v_max=1.0,
            goal_tolerance=0.5,
            control_frequency=10.0,
        )
        yield planner
        # TODO: This should call `planner.stop()` but that causes errors.
        # Calling just this for now to fix thread leaks.
        planner._close_module()

    @pytest.fixture
    def empty_costmap(self):
        """Create an empty costmap (all free space)."""
        costmap = OccupancyGrid(
            grid=np.zeros((100, 100), dtype=np.int8), resolution=0.1, origin=Pose()
        )
        costmap.origin.position.x = -5.0
        costmap.origin.position.y = -5.0
        return costmap

    def test_straight_path_no_obstacles(self, planner, empty_costmap) -> None:
        """Test that planner follows straight path with no obstacles."""
        # Set current position at origin
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        # Create straight path along +X
        path = Path()
        for i in range(10):
            ps = PoseStamped()
            ps.position.x = float(i)
            ps.position.y = 0.0
            ps.orientation.w = 1.0  # Identity quaternion
            path.poses.append(ps)
        planner.latest_path = path

        # Set empty costmap
        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Should move along +X
        assert vel is not None
        assert vel.linear.x > 0.9  # Close to v_max
        assert abs(vel.linear.y) < 0.1  # Near zero
        assert abs(vel.angular.z) < 0.1  # Small angular velocity when aligned with path

    def test_obstacle_gradient_repulsion(self, planner) -> None:
        """Test that obstacle gradients create repulsive forces."""
        # Set position at origin
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        # Simple path forward
        path = Path()
        ps = PoseStamped()
        ps.position.x = 5.0
        ps.position.y = 0.0
        ps.orientation.w = 1.0
        path.poses.append(ps)
        planner.latest_path = path

        # Create costmap with gradient pointing south (higher cost north)
        costmap_grid = np.zeros((100, 100), dtype=np.int8)
        for i in range(100):
            costmap_grid[i, :] = max(0, 50 - i)  # Gradient from north to south

        planner.latest_costmap = OccupancyGrid(grid=costmap_grid, resolution=0.1, origin=Pose())
        planner.latest_costmap.origin.position.x = -5.0
        planner.latest_costmap.origin.position.y = -5.0

        # Compute velocity
        vel = planner.compute_velocity()

        # Should have positive Y component (pushed north by gradient)
        assert vel is not None
        assert vel.linear.y > 0.1  # Repulsion pushes north

    def test_lowpass_filter(self) -> None:
        """Test that low-pass filter smooths velocity commands."""
        # Create planner with alpha=0.5 for filtering
        planner = HolonomicLocalPlanner(
            lookahead_dist=1.0,
            k_rep=0.0,  # No repulsion
            alpha=0.5,  # 50% filtering
            v_max=1.0,
        )

        # Setup similar to straight path test
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        path = Path()
        ps = PoseStamped()
        ps.position.x = 5.0
        ps.position.y = 0.0
        ps.orientation.w = 1.0
        path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = OccupancyGrid(
            grid=np.zeros((100, 100), dtype=np.int8), resolution=0.1, origin=Pose()
        )
        planner.latest_costmap.origin.position.x = -5.0
        planner.latest_costmap.origin.position.y = -5.0

        # First call - previous velocity is zero
        vel1 = planner.compute_velocity()
        assert vel1 is not None

        # Store first velocity
        first_vx = vel1.linear.x

        # Second call - should be filtered
        vel2 = planner.compute_velocity()
        assert vel2 is not None

        # With alpha=0.5 and same conditions:
        # v2 = 0.5 * v_raw + 0.5 * v1
        # The filtering effect should be visible
        # v2 should be between v1 and the raw velocity
        assert vel2.linear.x != first_vx  # Should be different due to filtering
        assert 0 < vel2.linear.x <= planner.v_max  # Should still be positive and within limits
        planner._close_module()

    def test_no_path(self, planner, empty_costmap) -> None:
        """Test that planner returns None when no path is available."""
        planner.latest_odom = PoseStamped()
        planner.latest_costmap = empty_costmap
        planner.latest_path = Path()  # Empty path

        vel = planner.compute_velocity()
        assert vel is None

    def test_no_odometry(self, planner, empty_costmap) -> None:
        """Test that planner returns None when no odometry is available."""
        planner.latest_odom = None
        planner.latest_costmap = empty_costmap

        path = Path()
        ps = PoseStamped()
        ps.position.x = 1.0
        ps.position.y = 0.0
        path.poses.append(ps)
        planner.latest_path = path

        vel = planner.compute_velocity()
        assert vel is None

    def test_no_costmap(self, planner) -> None:
        """Test that planner returns None when no costmap is available."""
        planner.latest_odom = PoseStamped()
        planner.latest_costmap = None

        path = Path()
        ps = PoseStamped()
        ps.position.x = 1.0
        ps.position.y = 0.0
        path.poses.append(ps)
        planner.latest_path = path

        vel = planner.compute_velocity()
        assert vel is None

    def test_goal_reached(self, planner, empty_costmap) -> None:
        """Test velocity when robot is at goal."""
        # Set robot at goal position
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 5.0
        planner.latest_odom.position.y = 0.0

        # Path with single point at robot position
        path = Path()
        ps = PoseStamped()
        ps.position.x = 5.0
        ps.position.y = 0.0
        ps.orientation.w = 1.0
        path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Should have near-zero velocity
        assert vel is not None
        assert abs(vel.linear.x) < 0.1
        assert abs(vel.linear.y) < 0.1

    def test_velocity_saturation(self, planner, empty_costmap) -> None:
        """Test that velocities are capped at v_max."""
        # Set robot far from goal to maximize commanded velocity
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        # Create path far away
        path = Path()
        ps = PoseStamped()
        ps.position.x = 100.0  # Very far
        ps.position.y = 0.0
        ps.orientation.w = 1.0
        path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Velocity should be saturated at v_max
        assert vel is not None
        assert abs(vel.linear.x) <= planner.v_max + 0.01  # Small tolerance
        assert abs(vel.linear.y) <= planner.v_max + 0.01
        assert abs(vel.angular.z) <= planner.v_max + 0.01

    def test_lookahead_interpolation(self, planner, empty_costmap) -> None:
        """Test that lookahead point is correctly interpolated on path."""
        # Set robot at origin
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        # Create path with waypoints closer than lookahead distance
        path = Path()
        for i in range(5):
            ps = PoseStamped()
            ps.position.x = i * 0.5  # 0.5m spacing
            ps.position.y = 0.0
            ps.orientation.w = 1.0
            path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Should move forward along path
        assert vel is not None
        assert vel.linear.x > 0.5  # Moving forward
        assert abs(vel.linear.y) < 0.1  # Staying on path

    def test_curved_path_following(self, planner, empty_costmap) -> None:
        """Test following a curved path."""
        # Set robot at origin
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0

        # Create curved path (quarter circle)
        path = Path()
        for i in range(10):
            angle = (np.pi / 2) * (i / 9.0)  # 0 to 90 degrees
            ps = PoseStamped()
            ps.position.x = 2.0 * np.cos(angle)
            ps.position.y = 2.0 * np.sin(angle)
            ps.orientation.w = 1.0
            path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Should have both X and Y components for curved motion
        assert vel is not None
        # Test general behavior: should be moving (not exact values)
        assert vel.linear.x > 0  # Moving forward (any positive value)
        assert vel.linear.y > 0  # Turning left (any positive value)
        # Ensure we have meaningful movement, not just noise
        total_linear = np.sqrt(vel.linear.x**2 + vel.linear.y**2)
        assert total_linear > 0.1  # Some reasonable movement

    def test_robot_frame_transformation(self, empty_costmap) -> None:
        """Test that velocities are correctly transformed to robot frame."""
        # Create planner with no filtering for deterministic test
        planner = HolonomicLocalPlanner(
            lookahead_dist=1.0,
            k_rep=0.0,  # No repulsion
            alpha=1.0,  # No filtering
            v_max=1.0,
        )

        # Set robot at origin but rotated 90 degrees (facing +Y in odom frame)
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0
        # Quaternion for 90 degree rotation around Z
        planner.latest_odom.orientation = Quaternion(0.0, 0.0, 0.7071068, 0.7071068)

        # Create path along +X axis in odom frame
        path = Path()
        for i in range(5):
            ps = PoseStamped()
            ps.position.x = float(i)
            ps.position.y = 0.0
            ps.orientation.w = 1.0
            path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Robot is facing +Y, path is along +X
        # So in robot frame: forward is +Y direction, path is to the right
        assert vel is not None
        # Test relative magnitudes and signs rather than exact values
        # Path is to the right, so Y velocity should be negative
        assert vel.linear.y < 0  # Should move right (negative Y in robot frame)
        # Should turn to align with path
        assert vel.angular.z < 0  # Should turn right (negative angular velocity)
        # X velocity should be relatively small compared to Y
        assert abs(vel.linear.x) < abs(vel.linear.y)  # Lateral movement dominates
        planner._close_module()

    def test_angular_velocity_computation(self, empty_costmap) -> None:
        """Test that angular velocity is computed to align with path."""
        planner = HolonomicLocalPlanner(
            lookahead_dist=2.0,
            k_rep=0.0,  # No repulsion
            alpha=1.0,  # No filtering
            v_max=1.0,
        )

        # Robot at origin facing +X
        planner.latest_odom = PoseStamped()
        planner.latest_odom.position.x = 0.0
        planner.latest_odom.position.y = 0.0
        planner.latest_odom.orientation.w = 1.0  # Identity quaternion

        # Create path at 45 degrees
        path = Path()
        for i in range(5):
            ps = PoseStamped()
            ps.position.x = float(i)
            ps.position.y = float(i)  # Diagonal path
            ps.orientation.w = 1.0
            path.poses.append(ps)
        planner.latest_path = path

        planner.latest_costmap = empty_costmap

        # Compute velocity
        vel = planner.compute_velocity()

        # Path is at 45 degrees, robot facing 0 degrees
        # Should have positive angular velocity to turn left
        assert vel is not None
        # Test general behavior without exact thresholds
        assert vel.linear.x > 0  # Moving forward (any positive value)
        assert vel.linear.y > 0  # Moving left (holonomic, any positive value)
        assert vel.angular.z > 0  # Turning left (positive angular velocity)
        # Verify the robot is actually moving with reasonable speed
        total_linear = np.sqrt(vel.linear.x**2 + vel.linear.y**2)
        assert total_linear > 0.1  # Some meaningful movement
        # Since path is diagonal, X and Y should be similar magnitude
        assert (
            abs(vel.linear.x - vel.linear.y) < max(vel.linear.x, vel.linear.y) * 0.5
        )  # Within 50% of each other
        planner._close_module()
