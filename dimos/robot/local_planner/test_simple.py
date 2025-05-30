import math
import time
from unittest.mock import Mock

import pytest
import reactivex as rx
from reactivex import operators as ops

from dimos.robot.local_planner.simple import SimplePlanner
from dimos.types.position import Position
from dimos.types.vector import Vector


class TestSimplePlanner:
    """Test suite for SimplePlanner class."""

    @pytest.fixture
    def mock_get_costmap(self):
        """Mock costmap getter function."""
        return Mock(return_value=None)

    @pytest.fixture
    def mock_get_robot_pos(self):
        """Mock robot position getter function."""
        return Mock(return_value=Position(Vector(0, 0, 0)))

    @pytest.fixture
    def planner(self, mock_get_costmap, mock_get_robot_pos):
        """Create a SimplePlanner instance with mocked dependencies."""
        return SimplePlanner(get_costmap=mock_get_costmap, get_robot_pos=mock_get_robot_pos)

    def test_initialization(self, planner, mock_get_costmap, mock_get_robot_pos):
        """Test that SimplePlanner initializes correctly."""
        assert planner.get_costmap == mock_get_costmap
        assert planner.get_robot_pos == mock_get_robot_pos
        assert planner.goal is None

    def test_set_goal_with_vector(self, planner):
        """Test setting goal with a Vector object."""
        goal = Vector(1, 2, 0)
        result = planner.set_goal(goal)

        assert result is True
        assert planner.goal == goal

    def test_get_move_stream_returns_observable(self, planner):
        """Test that get_move_stream returns an Observable."""
        move_stream = planner.get_move_stream(frequency=10.0)
        assert isinstance(move_stream, rx.Observable)

    def test_get_move_stream_with_no_goal(self, planner):
        """Test that move stream doesn't emit when no goal is set."""
        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1), ops.timeout(0.5)).subscribe(
            on_next=lambda x: emissions.append(x),
            on_error=lambda e: None,  # Ignore timeout errors
        )

        time.sleep(0.6)
        assert len(emissions) == 0

    def test_get_move_stream_no_rotation(self, planner, mock_get_robot_pos):
        """Test movement with robot facing forward (no rotation)."""
        # Robot at origin facing forward (no rotation), goal ahead
        mock_get_robot_pos.return_value = Position(Vector(0, 0, 0), Vector(0, 0, 0))
        planner.set_goal(Vector(1, 0, 0))  # Goal directly ahead

        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1)).subscribe(on_next=lambda x: emissions.append(x))

        time.sleep(0.5)

        assert len(emissions) == 1
        move_command = emissions[0]
        assert isinstance(move_command, Vector)

        # Should move forward (positive X in robot frame)
        assert abs(move_command.x - 0.2) < 0.001  # Forward
        assert abs(move_command.y) < 0.001  # No left/right movement

    def test_get_move_stream_with_rotation_90_degrees(self, planner, mock_get_robot_pos):
        """Test movement with robot rotated 90 degrees (facing left in global frame)."""
        # Robot facing left (90 degrees rotation), goal to the right in global frame
        mock_get_robot_pos.return_value = Position(Vector(0, 0, 0), Vector(0, 0, math.pi / 2))
        planner.set_goal(Vector(1, 0, 0))  # Goal to the right in global frame

        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1)).subscribe(on_next=lambda x: emissions.append(x))

        time.sleep(0.5)

        assert len(emissions) == 1
        move_command = emissions[0]

        # Goal is to the right in global frame, but robot is facing left
        # So in robot frame, this should be a right turn (negative Y)
        assert abs(move_command.x) < 0.001  # No forward/backward
        assert abs(move_command.y + 0.2) < 0.001  # Right turn (negative Y)

    def test_get_move_stream_with_rotation_180_degrees(self, planner, mock_get_robot_pos):
        """Test movement with robot rotated 180 degrees (facing backward)."""
        # Robot facing backward, goal behind in global frame (which is forward for robot)
        mock_get_robot_pos.return_value = Position(Vector(0, 0, 0), Vector(0, 0, math.pi))
        planner.set_goal(Vector(-1, 0, 0))  # Goal behind in global frame

        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1)).subscribe(on_next=lambda x: emissions.append(x))

        time.sleep(0.5)

        assert len(emissions) == 1
        move_command = emissions[0]

        # Goal is behind in global frame, but robot is facing backward
        # So in robot frame, this should be forward movement
        assert abs(move_command.x - 0.2) < 0.001  # Forward
        assert abs(move_command.y) < 0.001  # No left/right

    def test_get_move_stream_diagonal_movement(self, planner, mock_get_robot_pos):
        """Test diagonal movement with no robot rotation."""
        # Robot at origin, goal at (1,1) - should move diagonally
        mock_get_robot_pos.return_value = Position(Vector(0, 0, 0), Vector(0, 0, 0))
        planner.set_goal(Vector(1, 1, 0))

        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1)).subscribe(on_next=lambda x: emissions.append(x))

        time.sleep(0.5)

        assert len(emissions) == 1
        move_command = emissions[0]

        # Should be normalized direction vector * 0.2 speed
        # Direction from (0,0) to (1,1) normalized is approximately (0.707, 0.707)
        # Multiplied by 0.2 gives approximately (0.141, 0.141)
        assert abs(move_command.x - 0.14142136) < 0.001
        assert abs(move_command.y - 0.14142136) < 0.001

    def test_transform_to_robot_frame_no_rotation(self, planner):
        """Test coordinate transformation with no robot rotation."""
        robot_pos = Position(Vector(0, 0, 0), Vector(0, 0, 0))  # No rotation
        global_vec = Vector(1, 0, 0)  # Forward in global frame

        result = planner._transform_to_robot_frame(global_vec, robot_pos)

        # With no rotation, global and robot frames should be the same
        assert abs(result.x - 1.0) < 0.001
        assert abs(result.y) < 0.001

    def test_transform_to_robot_frame_90_degree_rotation(self, planner):
        """Test coordinate transformation with 90-degree robot rotation."""
        robot_pos = Position(Vector(0, 0, 0), Vector(0, 0, math.pi / 2))  # 90 degrees
        global_vec = Vector(1, 0, 0)  # Forward in global frame

        result = planner._transform_to_robot_frame(global_vec, robot_pos)

        # Robot is facing left, so global forward becomes robot right (negative Y)
        assert abs(result.x) < 0.001  # No forward/backward
        assert abs(result.y + 1.0) < 0.001  # Right turn (negative Y)

    def test_goal_at_robot_position(self, planner, mock_get_robot_pos):
        """Test behavior when goal is at current robot position."""
        position = Position(Vector(1, 1, 0), Vector(0, 0, 0))
        mock_get_robot_pos.return_value = position
        planner.set_goal(Vector(1, 1, 0))

        move_stream = planner.get_move_stream(frequency=10.0)

        emissions = []
        move_stream.pipe(ops.take(1)).subscribe(on_next=lambda x: emissions.append(x))

        time.sleep(0.3)

        assert len(emissions) == 1
        move_command = emissions[0]

        # When goal equals current position, difference is zero
        # Normalizing zero vector should result in zero vector
        # So movement should be zero
        assert abs(move_command.x) < 0.001
        assert abs(move_command.y) < 0.001
