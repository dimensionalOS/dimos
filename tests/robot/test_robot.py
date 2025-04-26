#
#
#

"""Tests for the robot module in the dimos package."""

import pytest
from unittest import mock

import tests.test_header

from dimos.robot.robot import Robot, MockRobot
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl


def test_robot_base_class():
    """Test that the Robot base class can be instantiated."""
    robot = Robot()
    assert robot is not None
    assert hasattr(robot, 'get_video_stream')
    assert hasattr(robot, 'cleanup')


def test_mock_robot():
    """Test that the MockRobot can be instantiated and used."""
    robot = MockRobot()
    assert robot is not None
    
    video_stream = robot.get_video_stream()
    assert video_stream is not None
    
    robot.cleanup()


@pytest.mark.parametrize("mock_connection", [True, False])
def test_unitree_ros_control_creation(mock_connection):
    """Test that UnitreeROSControl can be created with different configurations."""
    with mock.patch('dimos.robot.unitree.unitree_ros_control.rclpy'):
        ros_control = UnitreeROSControl(
            node_name="test_node",
            mock_connection=mock_connection
        )
        
        assert ros_control is not None
        assert ros_control.mock_connection == mock_connection


@pytest.mark.parametrize("mock_connection", [True])
def test_unitree_go2_creation(mock_connection):
    """Test that UnitreeGo2 can be created with a mocked connection."""
    with mock.patch('dimos.robot.unitree.unitree_ros_control.rclpy'):
        ros_control = UnitreeROSControl(
            node_name="test_node",
            mock_connection=mock_connection
        )
        
        robot = UnitreeGo2(ros_control=ros_control)
        assert robot is not None
        
        video_stream = robot.get_ros_video_stream()
        assert video_stream is not None


@pytest.mark.hardware
def test_unitree_go2_real_hardware():
    """Test UnitreeGo2 with real hardware (skipped by default)."""
    pytest.skip("This test requires physical hardware")
    
    
    import rclpy
    rclpy.init()
    
    ros_control = UnitreeROSControl(
        node_name="test_node",
        mock_connection=False
    )
    
    robot = UnitreeGo2(ros_control=ros_control)
    
    video_stream = robot.get_ros_video_stream()
    assert video_stream is not None
    
    robot.cleanup()
    rclpy.shutdown()


def test_robot_movement_commands():
    """Test robot movement commands with mocked hardware."""
    with mock.patch('dimos.robot.unitree.unitree_ros_control.rclpy'):
        ros_control = UnitreeROSControl(
            node_name="test_node",
            mock_connection=True
        )
        
        ros_control.publish_cmd_vel = mock.MagicMock()
        
        robot = UnitreeGo2(ros_control=ros_control)
        
        robot.move_forward(speed=0.5)
        ros_control.publish_cmd_vel.assert_called_with(linear_x=0.5, angular_z=0.0)
        
        robot.turn(angular_speed=0.3)
        ros_control.publish_cmd_vel.assert_called_with(linear_x=0.0, angular_z=0.3)
        
        robot.stop()
        ros_control.publish_cmd_vel.assert_called_with(linear_x=0.0, angular_z=0.0)
