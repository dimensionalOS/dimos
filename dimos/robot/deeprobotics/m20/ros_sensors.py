# Copyright 2025-2026 Dimensional Inc.
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

"""ROS 2 sensor interface for M20 quadruped.

Wraps a RawROS instance to subscribe to the M20's native ROS 2 topics
(/ODOM, /tf, /ALIGNED_POINTS, /IMU, /MOTION_INFO) and publish velocity
commands to /NAV_CMD for Navigation Mode operation.

The M20 uses FastRTPS (ros-foxy-fastrtps) with custom vendor messages
(drdds-ros2-msgs v1.0.4). Standard types use rclpy's RMW interop layer;
vendor types (NavCmd, MotionInfo) require drdds to be installed.

Note: /LIDAR/POINTS is restricted to the robot's AOS host (dev guide 2.1)
and requires multicast-relay.service. /ALIGNED_POINTS is the localization-
processed point cloud in map frame, accessible from GOS at ~11Hz.

Reference: M20 Software Development Guide sections 2.1-2.3
"""

import logging
import time
from typing import Any, NamedTuple

import numpy as np
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import PointCloud2 as DimosPointCloud2
from dimos.protocol.pubsub.impl.rospubsub import ROS_AVAILABLE, RawROS, RawROSTopic

# QoS import — only available when rclpy is installed
_QoSProfile = None
if ROS_AVAILABLE:
    try:
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )

        _QoSProfile = QoSProfile
    except ImportError:
        pass

logger = logging.getLogger(__name__)

# Standard ROS 2 message types (available wherever rclpy is installed)
_STD_TYPES_AVAILABLE = False
if ROS_AVAILABLE:
    try:
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu as RosImu
        from sensor_msgs.msg import PointCloud2 as RosPointCloud2
        from tf2_msgs.msg import TFMessage

        _STD_TYPES_AVAILABLE = True
    except ImportError:
        pass

# Vendor message types (only on GOS with drdds-ros2-msgs installed)
DRDDS_AVAILABLE = False
if ROS_AVAILABLE:
    try:
        from drdds.msg import MotionInfo, NavCmd

        DRDDS_AVAILABLE = True
    except ImportError:
        pass


class MotionInfoData(NamedTuple):
    """Extracted motion info from /MOTION_INFO topic."""

    vel_x: float
    vel_y: float
    vel_yaw: float
    height: float
    state: int  # 0=Idle, 1=Stand, 2=SoftEStop, 3=Damping, 4=Sit, 17=RLControl
    gait: int  # 0x1001=Basic, 0x3002=FlatAgile, 0x3003=StairAgile
    remain_mile: float


# --- Conversion functions ---


def _odom_to_pose_stamped(odom_msg: Any) -> PoseStamped:
    """Convert nav_msgs/Odometry to dimos PoseStamped."""
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    stamp = odom_msg.header.stamp
    return PoseStamped(
        position=Vector3(p.x, p.y, p.z),
        orientation=Quaternion(q.x, q.y, q.z, q.w),
        frame_id=odom_msg.header.frame_id or "odom",
        ts=stamp.sec + stamp.nanosec * 1e-9,
    )


def _tf_to_transforms(tf_msg: Any) -> list[Transform]:
    """Convert tf2_msgs/TFMessage to list of dimos Transforms."""
    transforms = []
    for t in tf_msg.transforms:
        stamp = t.header.stamp
        tr = t.transform.translation
        ro = t.transform.rotation
        transforms.append(
            Transform(
                translation=Vector3(tr.x, tr.y, tr.z),
                rotation=Quaternion(ro.x, ro.y, ro.z, ro.w),
                frame_id=t.header.frame_id,
                child_frame_id=t.child_frame_id,
                ts=stamp.sec + stamp.nanosec * 1e-9,
            )
        )
    return transforms


def _ros_pc2_to_dimos(pc_msg: Any) -> DimosPointCloud2:
    """Convert sensor_msgs/PointCloud2 to dimos PointCloud2.

    Extracts XYZ float32 points from the raw byte data, filtering NaN/Inf.
    Logic adapted from lidar.py:_dds_to_dimos_pointcloud.
    """
    stamp = pc_msg.header.stamp
    ts = stamp.sec + stamp.nanosec * 1e-9
    frame_id = pc_msg.header.frame_id or "world"

    n_points = pc_msg.width * pc_msg.height
    point_step = pc_msg.point_step

    if n_points == 0 or len(pc_msg.data) < n_points * point_step:
        return DimosPointCloud2(frame_id=frame_id, ts=ts)

    # Find x, y, z field offsets
    offsets = {}
    for f in pc_msg.fields:
        if f.name in ("x", "y", "z"):
            offsets[f.name] = f.offset

    if not all(k in offsets for k in ("x", "y", "z")):
        logger.warning("PointCloud2 missing x/y/z fields, cannot extract points")
        return DimosPointCloud2(frame_id=frame_id, ts=ts)

    raw = np.frombuffer(bytes(pc_msg.data), dtype=np.uint8)
    raw_2d = raw[: n_points * point_step].reshape(n_points, point_step)
    points = np.column_stack(
        [
            raw_2d[:, off : off + 4].view(np.float32).reshape(-1)
            for off in (offsets["x"], offsets["y"], offsets["z"])
        ]
    )

    # Filter NaN/Inf points
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    return DimosPointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)


def _motion_info_to_data(msg: Any) -> MotionInfoData:
    """Convert drdds/MotionInfo to MotionInfoData tuple.

    MotionInfo structure: msg.header (MetaType) + msg.data (MotionInfoValue).
    MotionInfoValue has: vel_x, vel_y, vel_yaw, height, motion_state, gait_state,
    payload, remain_mile.
    """
    d = msg.data
    return MotionInfoData(
        vel_x=d.vel_x,
        vel_y=d.vel_y,
        vel_yaw=d.vel_yaw,
        height=d.height,
        state=d.motion_state.state,
        gait=d.gait_state.gait,
        remain_mile=d.remain_mile,
    )


class M20ROSSensors:
    """ROS 2 sensor interface for the M20 quadruped.

    Composes a RawROS instance for rclpy node management and subscribes
    to the M20's native ROS 2 topics, converting to dimos message types.

    Standard topics (/ODOM, /tf, /ALIGNED_POINTS, /IMU) require rclpy
    with standard message packages. Vendor topics (/MOTION_INFO, /NAV_CMD)
    additionally require drdds-ros2-msgs.

    Usage:
        sensors = M20ROSSensors()
        sensors.start()
        sensors.odom_stream().subscribe(lambda pose: ...)
        sensors.publish_nav_cmd(0.5, 0.0, 0.0)
        sensors.stop()
    """

    def __init__(self, node_name: str = "dimos_m20") -> None:
        if not ROS_AVAILABLE:
            raise RuntimeError(
                "rclpy not available. Install ROS 2 or run on GOS."
            )
        if not _STD_TYPES_AVAILABLE:
            raise RuntimeError(
                "Standard ROS 2 message types not available "
                "(nav_msgs, sensor_msgs, tf2_msgs). "
                "Source /opt/ros/foxy/setup.bash or install ros-foxy-common-interfaces."
            )

        # QoS matching M20's RELIABLE/VOLATILE with moderate depth
        qos = None
        if _QoSProfile is not None:
            qos = _QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            )

        self._ros = RawROS(node_name=node_name, qos=qos)

        # Observable streams
        self._odom_subject: Subject[PoseStamped] = Subject()
        self._tf_subject: Subject[list[Transform]] = Subject()
        self._lidar_subject: Subject[DimosPointCloud2] = Subject()
        self._imu_subject: Subject[Any] = Subject()
        self._motion_info_subject: Subject[MotionInfoData] = Subject()

        # Unsubscribe handles
        self._unsubs: list[Any] = []

        # Topic descriptors (created after import guards pass)
        self._odom_topic = RawROSTopic(topic="/ODOM", ros_type=Odometry)
        self._tf_topic = RawROSTopic(topic="/tf", ros_type=TFMessage)
        self._lidar_topic = RawROSTopic(
            topic="/ALIGNED_POINTS", ros_type=RosPointCloud2
        )
        self._imu_topic = RawROSTopic(topic="/IMU", ros_type=RosImu)

        self._nav_cmd_topic: RawROSTopic | None = None
        self._motion_info_topic: RawROSTopic | None = None

        if DRDDS_AVAILABLE:
            self._nav_cmd_topic = RawROSTopic(
                topic="/NAV_CMD", ros_type=NavCmd
            )
            self._motion_info_topic = RawROSTopic(
                topic="/MOTION_INFO", ros_type=MotionInfo
            )
        else:
            logger.warning(
                "drdds-ros2-msgs not available — /NAV_CMD publisher "
                "and /MOTION_INFO subscription disabled"
            )

    def start(self) -> None:
        """Start the ROS node and subscribe to all M20 topics."""
        self._ros.start()

        # Standard subscriptions
        self._unsubs.append(
            self._ros.subscribe(self._odom_topic, self._on_odom)
        )
        self._unsubs.append(
            self._ros.subscribe(self._tf_topic, self._on_tf)
        )
        self._unsubs.append(
            self._ros.subscribe(self._lidar_topic, self._on_lidar)
        )
        self._unsubs.append(
            self._ros.subscribe(self._imu_topic, self._on_imu)
        )

        # Vendor subscriptions
        if self._motion_info_topic is not None:
            self._unsubs.append(
                self._ros.subscribe(
                    self._motion_info_topic, self._on_motion_info
                )
            )

        logger.info(
            "M20ROSSensors started — /ODOM, /tf, /ALIGNED_POINTS, /IMU"
            + (", /MOTION_INFO, /NAV_CMD" if DRDDS_AVAILABLE else "")
        )

    def stop(self) -> None:
        """Stop all subscriptions and shut down the ROS node."""
        for unsub in self._unsubs:
            try:
                unsub()
            except Exception:
                pass
        self._unsubs.clear()
        self._ros.stop()
        logger.info("M20ROSSensors stopped")

    # --- Observable streams ---

    def odom_stream(self) -> Observable[PoseStamped]:
        return self._odom_subject

    def tf_stream(self) -> Observable[list[Transform]]:
        return self._tf_subject

    def lidar_stream(self) -> Observable[DimosPointCloud2]:
        return self._lidar_subject

    def imu_stream(self) -> Observable[Any]:
        return self._imu_subject

    def motion_info_stream(self) -> Observable[MotionInfoData]:
        return self._motion_info_subject

    # --- /NAV_CMD publisher ---

    @property
    def nav_cmd_available(self) -> bool:
        """Whether /NAV_CMD publishing is available (drdds types installed)."""
        return self._nav_cmd_topic is not None

    def publish_nav_cmd(
        self, x_vel: float, y_vel: float, yaw_vel: float
    ) -> None:
        """Publish velocity command to /NAV_CMD (Navigation Mode).

        Args:
            x_vel: Forward/backward speed in m/s
            y_vel: Left/right speed in m/s
            yaw_vel: Yaw rotation speed in rad/s
        """
        if self._nav_cmd_topic is None:
            logger.error(
                "Cannot publish /NAV_CMD — drdds-ros2-msgs not available"
            )
            return

        msg = NavCmd()
        msg.data.x_vel = float(x_vel)
        msg.data.y_vel = float(y_vel)
        msg.data.yaw_vel = float(yaw_vel)
        now = time.time()
        msg.header.timestamp.sec = int(now)
        msg.header.timestamp.nsec = int((now - int(now)) * 1e9)
        self._ros.publish(self._nav_cmd_topic, msg)

    # --- ROS callbacks ---

    def _on_odom(self, msg: Any, _topic: RawROSTopic) -> None:
        try:
            pose = _odom_to_pose_stamped(msg)
            self._odom_subject.on_next(pose)
        except Exception:
            logger.exception("Error converting /ODOM message")

    def _on_tf(self, msg: Any, _topic: RawROSTopic) -> None:
        try:
            transforms = _tf_to_transforms(msg)
            self._tf_subject.on_next(transforms)
        except Exception:
            logger.exception("Error converting /tf message")

    def _on_lidar(self, msg: Any, _topic: RawROSTopic) -> None:
        try:
            pc = _ros_pc2_to_dimos(msg)
            self._lidar_subject.on_next(pc)
        except Exception:
            logger.exception("Error converting /ALIGNED_POINTS message")

    def _on_imu(self, msg: Any, _topic: RawROSTopic) -> None:
        try:
            self._imu_subject.on_next(msg)
        except Exception:
            logger.exception("Error converting /IMU message")

    def _on_motion_info(self, msg: Any, _topic: RawROSTopic) -> None:
        try:
            data = _motion_info_to_data(msg)
            self._motion_info_subject.on_next(data)
        except Exception:
            # /MOTION_INFO can be intermittent due to DDS discovery with
            # the bare-DDS publisher on AOS. Log at debug to avoid spam.
            logger.debug("Error converting /MOTION_INFO message", exc_info=True)
