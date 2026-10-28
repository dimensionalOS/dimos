# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from types import SimpleNamespace

from dimos_cdrgen.registry import encode
import numpy as np

from dimos.message_utils import pointcloud
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Vector3
from dimos.messages.sensor_msgs.msg import Imu
from dimos.messages.std_msgs.msg import Header
from dimos.robot.unitree.go2.dds import ros
from dimos.robot.unitree.go2.dds.cli.mcap2db import _dynamic_joint_state
from dimos.robot.unitree.go2.dds.cli.render import integrate_position, integrate_velocity
from dimos.robot.unitree.go2.dds.pointcloud import sensor_cloud, sensor_field


def test_unitree_imu_boundary_keeps_exact_ns_and_reorders_wxyz():
    source = Imu(
        Header(Time(1_700_000_000, 1), "lidar"),
        Quaternion(1.0, 0.0, 0.0, 0.0),
        np.arange(9, dtype=np.float64),
        Vector3(1.0, 2.0, 3.0),
        np.zeros(9),
        Vector3(4.0, 5.0, 6.0),
        np.ones(9),
    )
    result = ros.decode_imu(encode(source))
    assert (result.header.stamp.sec, result.header.stamp.nanosec, result.header.frame_id) == (
        1_700_000_000,
        1,
        "lidar",
    )
    assert (
        result.orientation.x,
        result.orientation.y,
        result.orientation.z,
        result.orientation.w,
    ) == (0.0, 0.0, 0.0, 1.0)
    np.testing.assert_array_equal(result.orientation_covariance, np.arange(9))
    assert (result.angular_velocity.x, result.linear_acceleration.z) == (1.0, 6.0)


def test_sensor_cloud_keeps_canonical_point_field_types_and_values():
    cloud = sensor_cloud(
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        header=Header(Time(17, 1), "lidar"),
        intensities=np.array([7.0, 8.0]),
        offset_times=np.array([0, 50_000_000]),
        tags=np.array([2, 3]),
        lines=np.array([4, 5]),
    )
    np.testing.assert_array_equal(pointcloud.xyz(cloud), [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    assert [(f.name, f.datatype) for f in cloud.fields] == [
        ("x", 7),
        ("y", 7),
        ("z", 7),
        ("intensity", 7),
        ("offset_time", 6),
        ("tag", 2),
        ("line", 2),
    ]
    np.testing.assert_array_equal(sensor_field(cloud, "offset_time"), [0, 50_000_000])
    np.testing.assert_array_equal(sensor_field(cloud, "intensity"), [7.0, 8.0])
    assert cloud.header.stamp.nanosec == 1


def test_cli_integrators_and_motor_interface_values_keep_numerical_contract():
    imu = SimpleNamespace(
        data=SimpleNamespace(
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0), linear_acceleration=Vector3(2.0, 0.0, 9.0)
        ),
        ts=3.0,
    )
    _, velocity = integrate_velocity(np.array([0.0, 0.0, 9.0]))((np.zeros(3), 2.0), imu)
    assert (velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z) == (
        2.0,
        0.0,
        0.0,
    )
    _, pose = integrate_position(
        (np.array([1.0, 2.0, 3.0]), 2.0), SimpleNamespace(data=velocity, ts=3.0)
    )
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (3.0, 2.0, 3.0)
    motor = _dynamic_joint_state(17.125, [("joint_a", {"position": 1.0, "effort": 2.0})])
    assert motor.joint_names == ["joint_a"]
    assert motor.interface_values[0].interface_names == ["position", "effort"]
    np.testing.assert_array_equal(motor.interface_values[0].values, [1.0, 2.0])
    assert (motor.header.stamp.sec, motor.header.stamp.nanosec) == (17, 125_000_000)
