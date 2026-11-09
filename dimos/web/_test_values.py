# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
from typing import Any

"""Small generated-message values shared by web protocol fixtures."""
import numpy as np

from dimos.message_utils import (
    geometry,
    image as image_utils,
    occupancy,
    pointcloud,
    time as message_time,
)
from dimos.messages.dimos_msgs.msg import MotorCommandArray
from dimos.messages.geometry_msgs import msg as geometry_msgs
from dimos.messages.nav_msgs import msg as nav_msgs
from dimos.messages.sensor_msgs import msg as sensor_msgs
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.messages.trajectory_msgs.msg import JointTrajectory


def header(ts: float = 0.0, frame_id: str = "") -> Header:
    return Header(message_time.time_from_seconds(ts), frame_id)


def vector(x: Any = 0.0, y: Any = 0.0, z: Any = 0.0) -> geometry_msgs.Vector3:
    return geometry_msgs.Vector3(x, y, z)


def quaternion(x: Any = 0.0, y: Any = 0.0, z: Any = 0.0, w: Any = 1.0) -> geometry_msgs.Quaternion:
    return geometry_msgs.Quaternion(x, y, z, w)


def quaternion_from_euler(value: Any) -> geometry_msgs.Quaternion:
    return geometry.quaternion_from_euler(value.x, value.y, value.z)


def pose(*args: Any, position: Any = None, orientation: Any = None) -> geometry_msgs.Pose:
    if len(args) in (3, 7):
        position = args[:3]
        if len(args) == 7:
            orientation = args[3:]
    elif args:
        position = args[0]
        if len(args) > 1:
            orientation = args[1]
    if position is None:
        position = (0.0, 0.0, 0.0)
    if orientation is None:
        orientation = (0.0, 0.0, 0.0, 1.0)
    p = (
        geometry_msgs.Point(position.x, position.y, position.z)
        if hasattr(position, "x")
        else geometry_msgs.Point(*position)
    )
    q = (
        geometry_msgs.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        if hasattr(orientation, "w")
        else geometry_msgs.Quaternion(*orientation)
    )
    return geometry_msgs.Pose(p, q)


def pose_stamped(
    *args: Any, ts: Any = 0.0, frame_id: Any = "", position: Any = None, orientation: Any = None
) -> geometry_msgs.PoseStamped:
    return geometry_msgs.PoseStamped(
        header(ts, frame_id), pose(*args, position=position, orientation=orientation)
    )


def point_stamped(
    x: Any = 0.0, y: Any = 0.0, z: Any = 0.0, *, ts: Any = 0.0, frame_id: Any = ""
) -> geometry_msgs.PointStamped:
    return geometry_msgs.PointStamped(header(ts, frame_id), geometry_msgs.Point(x, y, z))


def twist(linear: Any = None, angular: Any = None) -> geometry_msgs.Twist:
    def value(v: Any) -> Any:
        if v is None:
            return vector()
        return vector(v.x, v.y, v.z) if hasattr(v, "x") else vector(*v)

    return geometry_msgs.Twist(value(linear), value(angular))


def twist_stamped(
    *, linear: Any = None, angular: Any = None, ts: Any = 0.0, frame_id: Any = ""
) -> geometry_msgs.TwistStamped:
    return geometry_msgs.TwistStamped(header(ts, frame_id), twist(linear, angular))


def image(
    data: Any, *, encoding: Any = "bgr8", ts: Any = 0.0, frame_id: Any = ""
) -> sensor_msgs.Image:
    return image_utils.from_array(np.asarray(data), encoding=encoding, header=header(ts, frame_id))


def file_image(path: Any, *, encoding: Any = None) -> sensor_msgs.Image:
    value = image_utils.from_file(path)
    return image(image_utils.to_rgb(value), encoding="rgb8") if encoding == "rgb8" else value


def cloud(points: Any = None, *, frame_id: Any = "world", ts: Any = 0.0) -> sensor_msgs.PointCloud2:
    return pointcloud.from_xyz(
        np.empty((0, 3), np.float32) if points is None else np.asarray(points),
        header=header(ts, frame_id),
    )


def grid(
    grid: Any = None,
    *,
    resolution: Any = 0.05,
    origin: Any = None,
    ts: Any = 0.0,
    frame_id: Any = "map",
) -> nav_msgs.OccupancyGrid:
    return occupancy.from_array(
        np.empty((0, 0), np.int8) if grid is None else np.asarray(grid),
        header=header(ts, frame_id),
        resolution=resolution,
        origin=origin,
    )


def path(poses: Any = None, *, ts: Any = 0.0, frame_id: Any = "") -> nav_msgs.Path:
    return nav_msgs.Path(header(ts, frame_id), list(poses or []))


def joint_state(
    *,
    name: Any = None,
    position: Any = None,
    velocity: Any = None,
    effort: Any = None,
    ts: Any = 0.0,
    frame_id: Any = "",
) -> sensor_msgs.JointState:
    return sensor_msgs.JointState(
        header(ts, frame_id),
        list(name or []),
        np.asarray([] if position is None else position, dtype=np.float64),
        np.asarray([] if velocity is None else velocity, dtype=np.float64),
        np.asarray([] if effort is None else effort, dtype=np.float64),
    )


def imu(
    *,
    orientation: Any = None,
    angular_velocity: Any = None,
    linear_acceleration: Any = None,
    orientation_covariance: Any = None,
    angular_velocity_covariance: Any = None,
    linear_acceleration_covariance: Any = None,
    ts: Any = 0.0,
    frame_id: Any = "",
) -> sensor_msgs.Imu:
    return sensor_msgs.Imu(
        header(ts, frame_id),
        orientation or quaternion(),
        np.asarray(orientation_covariance or [0.0] * 9, dtype=np.float64),
        angular_velocity or vector(),
        np.asarray(angular_velocity_covariance or [0.0] * 9, dtype=np.float64),
        linear_acceleration or vector(),
        np.asarray(linear_acceleration_covariance or [0.0] * 9, dtype=np.float64),
    )


def transform_stamped(
    *,
    translation: Any = None,
    rotation: Any = None,
    ts: Any = 0.0,
    frame_id: Any = "",
    child_frame_id: Any = "",
) -> geometry_msgs.TransformStamped:
    return geometry_msgs.TransformStamped(
        header(ts, frame_id),
        child_frame_id,
        geometry_msgs.Transform(translation or vector(), rotation or quaternion()),
    )


def tf_message(*transforms: Any) -> TFMessage:
    return TFMessage(list(transforms))


def motor_command(
    *,
    q: Any = None,
    dq: Any = None,
    kp: Any = None,
    kd: Any = None,
    tau: Any = None,
    timestamp: Any = 0.0,
) -> MotorCommandArray:
    return MotorCommandArray(
        header(timestamp),
        *[np.asarray(v if v is not None else [], dtype=np.float64) for v in (q, dq, kp, kd, tau)],
    )


def trajectory(
    points: Any = None, joint_names: Any = None, timestamp: Any = 0.0
) -> JointTrajectory:
    return JointTrajectory(header(timestamp), list(joint_names or []), list(points or []))


def odometry(*, ts: Any = 0.0, frame_id: Any = "", child_frame_id: Any = "") -> nav_msgs.Odometry:
    return nav_msgs.Odometry(
        header(ts, frame_id),
        child_frame_id,
        geometry_msgs.PoseWithCovariance(pose(), np.zeros(36, dtype=np.float64)),
        geometry_msgs.TwistWithCovariance(twist(), np.zeros(36, dtype=np.float64)),
    )


def twist_is_zero(value: geometry_msgs.Twist) -> bool:
    return all(
        coordinate == 0.0
        for vector in (value.linear, value.angular)
        for coordinate in (vector.x, vector.y, vector.z)
    )
