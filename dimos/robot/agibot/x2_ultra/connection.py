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

from abc import ABC, abstractmethod
from pathlib import Path
import struct
import subprocess
import sys
from threading import Lock, Thread
import time
from typing import Any

import numpy as np
import open3d as o3d
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_ROBOT_FRAME, DEFAULT_THREAD_JOIN_TIMEOUT, DEFAULT_WORLD_FRAME
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.agibot.x2_ultra._arm_ik import X2ArmIK
from dimos.robot.agibot.x2_ultra.config import X2Config
from dimos.spec.perception import IMU, Camera, Lidar, Pointcloud
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ROS2 topic constants
# Using the *compressed* (JPEG, ~150 KB/frame, RELIABLE QoS) RGB topic instead
# of the raw 1280x720 RGB topic (~2.7 MB/frame, BEST_EFFORT): the raw stream's
# UDP fragments were being dropped by DDS on the laptop, so callbacks never
# fired. The robot publishes both; the compressed one delivers reliably.
_TOPIC_RGB_IMAGE = "/aima/hal/sensor/rgbd_head_front/rgb_image/compressed"
_TOPIC_DEPTH_IMAGE = "/aima/hal/sensor/rgbd_head_front/depth_image"
_TOPIC_DEPTH_CLOUD = "/aima/hal/sensor/rgbd_head_front/depth_pointcloud"
_TOPIC_RGB_CAM_INFO = "/aima/hal/sensor/rgbd_head_front/rgb_camera_info"
_TOPIC_LIDAR = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
_TOPIC_IMU = "/aima/hal/imu/chest/state"
_TOPIC_VELOCITY = "/aima/mc/locomotion/velocity"
_TOPIC_JOINT_ARM = "/aima/hal/joint/arm/state"
_TOPIC_JOINT_LEG = "/aima/hal/joint/leg/state"
_TOPIC_JOINT_WAIST = "/aima/hal/joint/waist/state"
_TOPIC_JOINT_HEAD = "/aima/hal/joint/head/state"
_TOPIC_ARM_COMMAND = "/aima/hal/joint/arm/command"
_SVC_INPUT_SOURCE = "/aimdk_5Fmsgs/srv/SetMcInputSource"

# Path to the URDF bundled in this package — used for IK.
_X2_URDF_PATH = Path(__file__).resolve().parent / "x2_ultra.urdf"

# Joint name ordering per the AgiBot X2 SDK docs (Interface > Control > Joint Control).
# Names match the X2 URDF/MJCF (sans the "_joint" suffix, which the viewer adds).
# JointStateArray.joints[] is positionally indexed in this exact order.
_ARM_JOINT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_yaw",
    "left_wrist_pitch",
    "left_wrist_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_yaw",
    "right_wrist_pitch",
    "right_wrist_roll",
)
_LEG_JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
)
_WAIST_JOINT_NAMES: tuple[str, ...] = ("waist_yaw", "waist_pitch", "waist_roll")
_HEAD_JOINT_NAMES: tuple[str, ...] = ("head_yaw", "head_pitch")

# Per-joint position-control gains for the arm, matching the SDK's
# motocontrol.py example. Index aligned with _ARM_JOINT_NAMES.
# (kp, kd, lower_limit_rad, upper_limit_rad)
_ARM_JOINT_GAINS: tuple[tuple[float, float, float, float], ...] = (
    # left_shoulder_pitch, _roll, _yaw, elbow, wrist_yaw, _pitch, _roll
    (20.0, 2.0, -3.08, 2.04),
    (20.0, 2.0, -0.061, 2.993),
    (20.0, 2.0, -2.556, 2.556),
    (20.0, 2.0, -2.3556, 0.0),
    (20.0, 2.0, -2.556, 2.556),
    (20.0, 2.0, -0.558, 0.558),
    (20.0, 2.0, -1.571, 0.724),
    # right_shoulder_pitch, _roll, _yaw, elbow, wrist_yaw, _pitch, _roll
    (20.0, 2.0, -3.08, 2.04),
    (20.0, 2.0, -2.993, 0.061),
    (20.0, 2.0, -2.556, 2.556),
    (20.0, 2.0, -2.3556, 0.0),
    (20.0, 2.0, -2.556, 2.556),
    (20.0, 2.0, -0.558, 0.558),
    (20.0, 2.0, -0.724, 1.571),
)

# Ruckig trajectory limits per arm joint (per the SDK example: max v=1.0 rad/s,
# max a=1.0 rad/s², max j=25.0 rad/s³). 2 ms control period = 500 Hz.
_ARM_CONTROL_PERIOD_S = 0.002
_ARM_MAX_VEL = 1.0
_ARM_MAX_ACC = 1.0
_ARM_MAX_JERK = 25.0

# Velocity limits from the SDK
_MAX_FORWARD = 1.0
_MIN_FORWARD = 0.2
_MAX_LATERAL = 1.0
_MIN_LATERAL = 0.2
_MAX_ANGULAR = 1.0
_MIN_ANGULAR = 0.1

# Input source config for velocity control
_INPUT_SOURCE_NAME = "dimos"
_INPUT_SOURCE_PRIORITY = 40
_INPUT_SOURCE_TIMEOUT = 1000  # ms


class X2ConnectionBase(Module, ABC):
    """Abstract base for X2 Ultra connections (real hardware and future simulation).

    Other modules that depend on X2 RPC methods should reference this base class
    so blueprint wiring works regardless of which concrete connection is deployed.
    """

    config: ModuleConfig

    @rpc
    @abstractmethod
    def start(self) -> None:
        super().start()

    @rpc
    @abstractmethod
    def stop(self) -> None:
        super().stop()

    @rpc
    @abstractmethod
    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...

    @rpc
    @abstractmethod
    def stop_motion(self) -> bool: ...

    @rpc
    @abstractmethod
    def observe(self) -> Image | None: ...


class ConnectionConfig(ModuleConfig):
    ros_domain_id: int = Field(default=0, description="ROS_DOMAIN_ID matching the robot")
    force_cyclonedds: bool = Field(
        default=True,
        description="Force CycloneDDS with an explicit peer/interface before rclpy init.",
    )
    clear_rmw_env: bool = Field(
        default=False,
        description="Clear inherited RMW_IMPLEMENTATION/CYCLONEDDS_URI before rclpy init.",
    )
    cyclone_interface: str = Field(
        default="wlp0s20f3",
        description="Network interface used when force_cyclonedds is enabled.",
    )
    cyclone_peer: str = Field(
        default="10.0.0.209",
        description="Static DDS peer address used when force_cyclonedds is enabled.",
    )
    enable_lidar: bool = Field(
        default=False,
        description="Subscribe to the chest lidar pointcloud and publish it to DimOS.",
    )
    enable_depth_cloud: bool = Field(
        default=False,
        description="Subscribe to the head depth pointcloud and publish it to DimOS.",
    )
    frame_id: str = Field(
        default=DEFAULT_ROBOT_FRAME,
        description="Robot body frame published on TF.",
    )
    parent_frame_id: str = Field(
        default=DEFAULT_WORLD_FRAME,
        description="Parent frame for the odom→base_link transform.",
    )
    static_publish_rate: float = Field(
        default=1.0,
        description="Rate (Hz) at which static TF frames are republished.",
    )
    static_transforms: dict[str, Transform] = Field(
        default_factory=lambda: dict(X2Config.static_transforms)
    )
    frame_mapping: dict[str, str] = Field(
        default_factory=lambda: {each: each for each in X2Config.all_frame_ids}
    )


def _cyclonedds_uri(interface: str, peer: str) -> str:
    return (
        "<CycloneDDS><Domain>"
        "<General>"
        f'<Interfaces><NetworkInterface name="{interface}"/></Interfaces>'
        "</General>"
        "<Discovery>"
        f'<Peers><Peer address="{peer}"/></Peers>'
        "<ParticipantIndex>auto</ParticipantIndex>"
        "</Discovery>"
        "</Domain></CycloneDDS>"
    )


def _clamp_velocity(value: float, min_mag: float, max_mag: float) -> float:
    """Apply SDK dead zone: zero if near zero, clamp to [min, max] magnitude otherwise."""
    if abs(value) < 0.005:
        return 0.0
    return (
        float(np.clip(value, -max_mag, max_mag))
        if abs(value) >= min_mag
        else (min_mag * np.sign(value))
    )


def _ros_image_to_dimos(msg: Any) -> Image:
    """Convert a ROS2 sensor_msgs/Image to a dimos Image."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    enc = msg.encoding.lower()

    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if enc in ("rgb8",):
        arr = data.reshape((msg.height, msg.width, 3))
        fmt = ImageFormat.RGB
    elif enc in ("bgr8",):
        arr = data.reshape((msg.height, msg.width, 3))
        fmt = ImageFormat.BGR
    elif enc in ("rgba8",):
        arr = data.reshape((msg.height, msg.width, 4))
        fmt = ImageFormat.RGBA
    elif enc in ("bgra8",):
        arr = data.reshape((msg.height, msg.width, 4))
        fmt = ImageFormat.BGRA
    elif enc in ("mono8",):
        arr = data.reshape((msg.height, msg.width))
        fmt = ImageFormat.GRAY
    elif enc in ("16uc1", "mono16"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape((msg.height, msg.width))
        fmt = ImageFormat.DEPTH16
    elif enc in ("32fc1",):
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape((msg.height, msg.width))
        fmt = ImageFormat.DEPTH
    else:
        # Fall back to raw reshape; assume 3-channel
        logger.warning("X2Connection: unknown image encoding %s, treating as BGR", enc)
        arr = data.reshape((msg.height, msg.width, -1))
        fmt = ImageFormat.BGR

    return Image(data=arr, format=fmt, frame_id=msg.header.frame_id, ts=ts)


def _ros_pointcloud2_to_dimos(msg: Any) -> PointCloud2:
    """Convert a ROS2 sensor_msgs/PointCloud2 to a dimos PointCloud2.

    Handles any point_step by stride-indexing float32 values at the x/y/z
    field offsets (offsets must be 4-byte aligned, which is standard).
    """
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    n_points = msg.width * msg.height
    if n_points == 0:
        return PointCloud2(frame_id=msg.header.frame_id, ts=ts)

    offsets = {f.name: f.offset for f in msg.fields}
    x_off = offsets.get("x", 0)
    y_off = offsets.get("y", 4)
    z_off = offsets.get("z", 8)
    step = msg.point_step
    step_f32 = step // 4

    # Interpret entire buffer as float32 then stride-index each field
    raw_f32 = np.frombuffer(bytes(msg.data), dtype=np.float32)
    xs = raw_f32[x_off // 4 :: step_f32][:n_points]
    ys = raw_f32[y_off // 4 :: step_f32][:n_points]
    zs = raw_f32[z_off // 4 :: step_f32][:n_points]

    pts = np.column_stack([xs, ys, zs]).astype(np.float64)
    mask = np.isfinite(pts).all(axis=1)
    pts = pts[mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    return PointCloud2(pointcloud=pcd, frame_id=msg.header.frame_id, ts=ts)


def _ros_camera_info_to_dimos(msg: Any) -> CameraInfo:
    """Convert a ROS2 sensor_msgs/CameraInfo to a dimos CameraInfo."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return CameraInfo(
        height=msg.height,
        width=msg.width,
        distortion_model=msg.distortion_model,
        D=list(msg.d),
        K=list(msg.k),
        R=list(msg.r),
        P=list(msg.p),
        binning_x=msg.binning_x,
        binning_y=msg.binning_y,
        frame_id=msg.header.frame_id,
        ts=ts,
    )


def _ros_imu_to_dimos(msg: Any) -> Imu:
    """Convert a ROS2 sensor_msgs/Imu to a dimos Imu."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return Imu(
        angular_velocity=Vector3(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ),
        linear_acceleration=Vector3(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ),
        orientation=Quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ),
        orientation_covariance=list(msg.orientation_covariance),
        angular_velocity_covariance=list(msg.angular_velocity_covariance),
        linear_acceleration_covariance=list(msg.linear_acceleration_covariance),
        frame_id=msg.header.frame_id,
        ts=ts,
    )


class X2Connection(X2ConnectionBase, Camera, Pointcloud, IMU, Lidar):
    """DIMOS module for the AgiBot X2 Ultra humanoid robot.

    Connects via ROS2 (rclpy) — set ROS_DOMAIN_ID to match the robot before launching.

    Streams:
      color_image  — RGB image from head RGBD camera
      camera_info  — camera intrinsics (auto-read from robot)
      pointcloud   — depth point cloud from head RGBD camera
      lidar        — chest LiDAR point cloud
      imu          — chest IMU
      joint_state  — combined arm/leg/waist/head joint positions (names match URDF/MJCF
                     without the "_joint" suffix)

    Inputs:
      cmd_vel — geometry_msgs/Twist: linear.x=forward, linear.y=lateral, angular.z=yaw
    """

    config: ConnectionConfig
    cmd_vel: In[Twist]
    # Direct joint command in the URDF's ordered arm-joint space. The 14
    # values follow _ARM_JOINT_NAMES (left-then-right per the SDK). Names in
    # the message are optional; if present we map by name.
    arm_joint_command: In[JointState]
    # End-effector cartesian targets in the pelvis frame. We solve IK on the
    # corresponding arm and forward to /aima/hal/joint/arm/command.
    cartesian_left: In[PoseStamped]
    cartesian_right: In[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_image: Out[Image]
    depth_camera_info: Out[CameraInfo]
    pointcloud: Out[PointCloud2]
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    joint_state: Out[JointState]

    _ros_node: Any = None
    _ros_thread: Thread | None = None
    _vel_publisher: Any = None
    _arm_publisher: Any = None
    _latest_video_frame: Image | None = None
    _input_source_registered: bool = False
    _cam_proc: subprocess.Popen | None = None  # type: ignore[type-arg]
    _cam_thread: Thread | None = None
    _arm_ik: X2ArmIK | None = None
    _arm_ctrl_thread: Thread | None = None
    _static_publish_thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_video_frame = None
        # Joint positions aggregated across the 4 per-bodypart topics.
        # Locked because each subtopic callback writes its slice independently.
        self._joint_lock = Lock()
        self._joint_positions: dict[str, float] = {}
        self._input_source_registered = False
        self._cam_proc = None
        self._cam_thread = None
        self._cam_stop = False
        # Arm controller (built once we have the first joint_state snapshot so
        # we can seed Ruckig with the real current pose, not zeros).
        self._arm_ik = None
        self._arm_lock = Lock()
        self._arm_target: list[float] | None = None  # 14-vector target, ARM_JOINT_NAMES order
        self._arm_ctrl_thread = None
        self._arm_ctrl_stop = False
        self._arm_state_seen = False  # set once joint_state slice arrives

    @rpc
    def start(self) -> None:
        super().start()
        self._start_ros()
        self._static_publish_thread = Thread(
            target=self._static_publish,
            daemon=True,
        )
        self._static_publish_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_ros()
        if self._static_publish_thread and self._static_publish_thread.is_alive():
            self._static_publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _start_ros(self) -> None:
        try:
            import os

            import rclpy

            os.environ.setdefault("ROS_DOMAIN_ID", str(self.config.ros_domain_id))
            if self.config.clear_rmw_env:
                os.environ.pop("RMW_IMPLEMENTATION", None)
                os.environ.pop("CYCLONEDDS_URI", None)
            if self.config.force_cyclonedds:
                # Laptop-to-robot runs may need CycloneDDS with a pinned peer.
                # Bot-local runs should leave this disabled and use the robot's
                # installed default RMW.
                os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
                os.environ.setdefault(
                    "CYCLONEDDS_URI",
                    _cyclonedds_uri(self.config.cyclone_interface, self.config.cyclone_peer),
                )
            # dimos workers are forked from a forkserver, which can carry a
            # half-initialized DDS context into the child. Reusing it works
            # for the first message or two on high-bandwidth topics, then
            # silently drops. Tear it down and re-init in this worker so
            # the DDS layer owns a clean state.
            try:
                rclpy.init()
            except RuntimeError as init_exc:
                if "must only be called once" not in str(init_exc):
                    raise
                logger.info("X2Connection: shutting down inherited rclpy context and re-initing")
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
                rclpy.init()
        except Exception as e:
            logger.error("X2Connection: failed to init rclpy: %s", e)
            raise

        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        node = rclpy.create_node("dimos_x2_connection")
        self._ros_node = node
        # ReentrantCallbackGroup pairs with MultiThreadedExecutor (see _ros_spin):
        # without it, callbacks on this node still serialise even with a thread
        # pool. With it, the slow camera-decode callback runs in parallel with
        # high-rate joint-state callbacks.
        sensor_group = ReentrantCallbackGroup()

        # NOTE: the /compressed RGB topic is not subscribed here. Inside the
        # dimos forkserver-spawned worker, FastDDS drops camera frames after
        # the first one or two — a standalone rclpy process with identical
        # QoS receives the same topic at 10 Hz with no issue. We sidestep
        # this by running the subscription in a clean subprocess (see
        # `_camera_bridge.py`) and piping JPEG frames back here.
        self._start_camera_bridge()

        # NOTE: depth_image is also large (1280x720 float32 ~3.7 MB/frame); not
        # subscribed here because nothing downstream consumes it yet, and the
        # extra UDP bandwidth used to crowd out the camera bridge subprocess.
        # Re-enable when you actually need depth in a downstream module.
        # depth_pointcloud + lidar_pointcloud are 30 Hz × MB-per-frame
        # streams. Subscribing here pulls in ~50 MB/s on the laptop NIC and
        # crowds out the camera bridge subprocess. They're not consumed by
        # any downstream module in this blueprint, so we leave them off.
        # If you need them later, re-enable here and confirm the camera
        # bridge still flows.
        if self.config.enable_depth_cloud:
            node.create_subscription(
                self._import_msg("sensor_msgs.msg", "PointCloud2"),
                _TOPIC_DEPTH_CLOUD,
                self._on_depth_cloud,
                sensor_qos,
                callback_group=sensor_group,
            )
        if self.config.enable_lidar:
            node.create_subscription(
                self._import_msg("sensor_msgs.msg", "PointCloud2"),
                _TOPIC_LIDAR,
                self._on_lidar,
                sensor_qos,
                callback_group=sensor_group,
            )
        node.create_subscription(
            self._import_msg("sensor_msgs.msg", "Imu"),
            _TOPIC_IMU,
            self._on_imu,
            sensor_qos,
            callback_group=sensor_group,
        )

        # Joint state subscriptions (one per body part, all aimdk_msgs/JointStateArray).
        joint_state_msg = self._import_msg("aimdk_msgs.msg", "JointStateArray")
        node.create_subscription(
            joint_state_msg,
            _TOPIC_JOINT_ARM,
            lambda m: self._on_joint_state_array(m, _ARM_JOINT_NAMES),
            sensor_qos,
            callback_group=sensor_group,
        )
        node.create_subscription(
            joint_state_msg,
            _TOPIC_JOINT_LEG,
            lambda m: self._on_joint_state_array(m, _LEG_JOINT_NAMES),
            sensor_qos,
            callback_group=sensor_group,
        )
        node.create_subscription(
            joint_state_msg,
            _TOPIC_JOINT_WAIST,
            lambda m: self._on_joint_state_array(m, _WAIST_JOINT_NAMES),
            sensor_qos,
            callback_group=sensor_group,
        )
        node.create_subscription(
            joint_state_msg,
            _TOPIC_JOINT_HEAD,
            lambda m: self._on_joint_state_array(m, _HEAD_JOINT_NAMES),
            sensor_qos,
            callback_group=sensor_group,
        )

        from rclpy.qos import QoSDurabilityPolicy

        cam_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        node.create_subscription(
            self._import_msg("sensor_msgs.msg", "CameraInfo"),
            _TOPIC_RGB_CAM_INFO,
            self._on_camera_info,
            cam_info_qos,
            callback_group=sensor_group,
        )

        VelMsg = self._import_msg("aimdk_msgs.msg", "McLocomotionVelocity")
        self._vel_publisher = node.create_publisher(VelMsg, _TOPIC_VELOCITY, 10)

        # Arm joint command publisher. Robot must be in JOINT_DEFAULT (or have
        # MC stopped) for these commands to actually drive the joints — see
        # set_motion_mode().
        ArmCmdMsg = self._import_msg("aimdk_msgs.msg", "JointCommandArray")
        self._arm_publisher = node.create_publisher(ArmCmdMsg, _TOPIC_ARM_COMMAND, 10)

        self.register_disposable(self.cmd_vel.subscribe(self.move))
        self.register_disposable(self.arm_joint_command.subscribe(self._on_arm_joint_command))
        self.register_disposable(self.cartesian_left.subscribe(self._on_cartesian_left))
        self.register_disposable(self.cartesian_right.subscribe(self._on_cartesian_right))

        self._ros_thread = Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

        # Arm trajectory controller: 500 Hz Ruckig planner that always
        # publishes all 14 arm joints. Starts paused until a target is set.
        self._arm_ctrl_stop = False
        self._arm_ctrl_thread = Thread(target=self._arm_controller_loop, daemon=True)
        self._arm_ctrl_thread.start()

        # Register input source in a background thread so start() returns quickly
        Thread(target=self._register_input_source, daemon=True).start()

    def _ros_spin(self) -> None:
        from rclpy.executors import MultiThreadedExecutor

        # MultiThreadedExecutor: joint-state topics on this node tick at
        # 50-100 Hz × 4 body parts. A SingleThreadedExecutor starves the
        # camera callback (verified empirically: ~4 RGB frames then silence).
        try:
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(self._ros_node)
            executor.spin()
        except Exception as e:
            logger.warning("X2Connection: ROS spin exited: %s", e)

    def _stop_ros(self) -> None:
        # Tear down the camera bridge first so its rclpy shutdown isn't racing
        # ours.
        self._stop_camera_bridge()

        # Stop the arm controller loop before rclpy shuts down.
        self._arm_ctrl_stop = True
        if self._arm_ctrl_thread and self._arm_ctrl_thread.is_alive():
            self._arm_ctrl_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._arm_ctrl_thread = None

        if self._ros_node is not None:
            try:
                import rclpy

                self._ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                logger.warning("X2Connection: error during ROS shutdown: %s", e)
            self._ros_node = None

        if self._ros_thread and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._ros_thread = None

    def _register_input_source(self) -> None:
        """Register the DIMOS input source with the motion controller.

        Uses polling instead of spin_until_future_complete because _ros_spin
        is already driving the node's executor in its own thread; calling
        spin_until_future_complete concurrently causes both to race and the
        response never gets delivered.
        """
        from aimdk_msgs.srv import SetMcInputSource

        client = self._ros_node.create_client(SetMcInputSource, _SVC_INPUT_SOURCE)

        svc_timeout = 15.0
        start = time.time()
        while not client.service_is_ready():
            if time.time() - start > svc_timeout:
                logger.error(
                    "X2Connection: input source service not available after %.0fs", svc_timeout
                )
                return
            logger.info("X2Connection: waiting for input source service...")
            time.sleep(1.0)

        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = _INPUT_SOURCE_NAME
        req.input_source.priority = _INPUT_SOURCE_PRIORITY
        req.input_source.timeout = _INPUT_SOURCE_TIMEOUT

        future = None
        for i in range(8):
            req.request.header.stamp = self._ros_node.get_clock().now().to_msg()
            future = client.call_async(req)
            # Let the _ros_spin thread process the response — just poll here.
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                break
            logger.info("X2Connection: retrying input source registration [%d]", i)

        if future is not None and future.done():
            try:
                resp = future.result()
                logger.info(
                    "X2Connection: input source registered, state=%s task_id=%s",
                    resp.response.state.value,
                    resp.response.task_id,
                )
                self._input_source_registered = True
            except Exception as e:
                logger.error("X2Connection: input source registration failed: %s", e)
        else:
            logger.error("X2Connection: input source registration timed out")

    # --- Arm control (matches SDK example motocontrol.py) ---
    #
    # Architecture:
    #   * One Ruckig planner sized to all 14 arm joints, period 2 ms (500 Hz).
    #   * Targets are set by skills; the controller thread always runs and
    #     publishes every cycle whether moving or holding pose.
    #   * Per-joint kp/kd from _ARM_JOINT_GAINS (the SDK's calibrated table).
    #   * Robot does NOT need to be in JOINT_DEFAULT — works in STAND_DEFAULT
    #     while MC handles balance. The SDK example does exactly this.

    def _ensure_arm_ik(self) -> X2ArmIK:
        if self._arm_ik is None:
            self._arm_ik = X2ArmIK(_X2_URDF_PATH)
            logger.info("X2Connection: arm IK loaded from %s", _X2_URDF_PATH)
        return self._arm_ik

    def _arm_state_vector(self) -> list[float]:
        """Return the current 14-DoF arm state from joint_state in the SDK's
        canonical order. Missing slots default to 0 — only used until the
        first joint_state arrives."""
        with self._joint_lock:
            return [float(self._joint_positions.get(n, 0.0)) for n in _ARM_JOINT_NAMES]

    def _clamp_arm(self, vec: list[float]) -> list[float]:
        out = []
        for v, (_, _, lo, hi) in zip(vec, _ARM_JOINT_GAINS, strict=True):
            out.append(float(min(max(v, lo), hi)))
        return out

    def _set_arm_target(self, target: list[float]) -> None:
        if len(target) != 14:
            raise ValueError(f"arm target must be 14 floats, got {len(target)}")
        clamped = self._clamp_arm(target)
        with self._arm_lock:
            self._arm_target = clamped
        logger.info("X2Connection: arm target set")

    def _arm_controller_loop(self) -> None:
        """500 Hz Ruckig position controller. Publishes all 14 arm joints
        every cycle. No-op until the first joint_state arrives AND a target
        has been set at least once."""
        import ruckig

        rk = ruckig.Ruckig(14, _ARM_CONTROL_PERIOD_S)
        in_ = ruckig.InputParameter(14)
        out_ = ruckig.OutputParameter(14)
        in_.max_velocity = [_ARM_MAX_VEL] * 14
        in_.max_acceleration = [_ARM_MAX_ACC] * 14
        in_.max_jerk = [_ARM_MAX_JERK] * 14
        in_.target_velocity = [0.0] * 14
        in_.target_acceleration = [0.0] * 14
        initialised = False

        while not self._arm_ctrl_stop:
            # Wait until we have a real joint_state snapshot AND a target.
            if not self._arm_state_seen:
                time.sleep(0.02)
                continue
            with self._arm_lock:
                target = list(self._arm_target) if self._arm_target is not None else None
            if target is None:
                time.sleep(0.02)
                continue

            if not initialised:
                # Seed Ruckig from the real current arm pose.
                state = self._arm_state_vector()
                in_.current_position = state
                in_.current_velocity = [0.0] * 14
                in_.current_acceleration = [0.0] * 14
                initialised = True

            in_.target_position = target
            result = rk.update(in_, out_)

            # Build + publish JointCommandArray (all 14 joints every cycle).
            try:
                from aimdk_msgs.msg import (
                    JointCommand,
                    JointCommandArray,
                    MessageHeader,
                )

                cmd = JointCommandArray()
                cmd.header = MessageHeader()
                cmd.header.stamp = self._ros_node.get_clock().now().to_msg()
                for i, short_name in enumerate(_ARM_JOINT_NAMES):
                    kp, kd, _, _ = _ARM_JOINT_GAINS[i]
                    jc = JointCommand()
                    jc.name = short_name + "_joint"
                    jc.position = float(out_.new_position[i])
                    jc.velocity = float(out_.new_velocity[i])
                    jc.effort = 0.0
                    jc.stiffness = kp
                    jc.damping = kd
                    cmd.joints.append(jc)
                self._arm_publisher.publish(cmd)
            except Exception as exc:
                logger.exception("X2Connection: arm cmd publish failed: %s", exc)
                time.sleep(0.1)
                continue

            # Roll forward
            in_.current_position = out_.new_position
            in_.current_velocity = out_.new_velocity
            in_.current_acceleration = out_.new_acceleration

            time.sleep(_ARM_CONTROL_PERIOD_S)

            # If we reached target with zero velocity, stay here but keep
            # republishing (the SDK example does the same; holds pose).
            if result == ruckig.Result.Finished:
                pass

    def _on_arm_joint_command(self, msg: JointState) -> None:
        """Set a 14-joint position target directly. Names use the short form,
        e.g. 'left_shoulder_pitch'. Missing entries default to current state.
        """
        positions = dict(zip(msg.name, msg.position, strict=False))
        target = [
            float(positions.get(n, self._joint_positions.get(n, 0.0))) for n in _ARM_JOINT_NAMES
        ]
        self._set_arm_target(target)

    def _solve_and_target_cartesian(self, target_pose: PoseStamped, side: str) -> bool:
        import numpy as np
        import pinocchio as pin

        ik = self._ensure_arm_ik()
        chain = ik.left if side == "left" else ik.right

        target_se3 = pin.SE3(
            pin.Quaternion(
                float(target_pose.orientation.w),
                float(target_pose.orientation.x),
                float(target_pose.orientation.y),
                float(target_pose.orientation.z),
            ).matrix(),
            np.array([target_pose.x, target_pose.y, target_pose.z], dtype=np.float64),
        )

        # Seed IK from the current robot state via pinocchio q.
        ik_q = ik.home_q()
        with self._joint_lock:
            snap = dict(self._joint_positions)
        for jname, pos in snap.items():
            full = jname + "_joint"
            if ik.model.existJointName(full):
                jid = ik.model.getJointId(full)
                ik_q[ik.model.joints[jid].idx_q] = float(pos)

        q_sol, ok, err = ik.solve(target_se3, ik_q, chain)
        if not ok:
            logger.warning(
                "X2Connection: %s IK no-converge (err=%.4f), using best effort", side, err
            )

        # Start from current arm target (preserve unaffected joints if a
        # previous target was set; else start from current state).
        with self._arm_lock:
            base = (
                list(self._arm_target) if self._arm_target is not None else self._arm_state_vector()
            )
        for short_name, slot in zip(chain.joint_names, chain.qpos_indices, strict=True):
            key = short_name[: -len("_joint")] if short_name.endswith("_joint") else short_name
            idx = _ARM_JOINT_NAMES.index(key)
            base[idx] = float(q_sol[slot])

        self._set_arm_target(base)
        return True

    def _on_cartesian_left(self, msg: PoseStamped) -> None:
        self._solve_and_target_cartesian(msg, "left")

    def _on_cartesian_right(self, msg: PoseStamped) -> None:
        self._solve_and_target_cartesian(msg, "right")

    @skill
    def home_arms(self) -> bool:
        """Drive both arms to the all-zeros (hanging) pose, smoothly.

        Works in ``STAND_DEFAULT`` while the robot is balanced — the trajectory
        controller publishes at 500 Hz with the SDK's per-joint gains. No mode
        switch needed.
        """
        self._set_arm_target([0.0] * 14)
        return True

    @skill
    def tuck_arms(self) -> bool:
        """Move both arms to a tucked pose (elbows bent ~90°, arms close to body)."""
        target = [0.0] * 14
        # elbow indices in _ARM_JOINT_NAMES: 3 (left), 10 (right)
        target[3] = -1.2  # left elbow (limits: -2.3556..0.0)
        target[10] = -1.2  # right elbow
        self._set_arm_target(target)
        return True

    @skill
    def move_left_hand_to(self, x: float, y: float, z: float) -> bool:
        """Move the left wrist to (x, y, z) metres in the pelvis frame.

        Orientation kept at its current value.
        """
        import pinocchio as pin

        ik = self._ensure_arm_ik()
        ik_q = ik.home_q()
        with self._joint_lock:
            snap = dict(self._joint_positions)
        for jname, pos in snap.items():
            full = jname + "_joint"
            if ik.model.existJointName(full):
                jid = ik.model.getJointId(full)
                ik_q[ik.model.joints[jid].idx_q] = float(pos)
        current = ik.fk_pose(ik_q, ik.left)
        q_wxyz = pin.Quaternion(current.rotation)
        target = PoseStamped(
            x=float(x),
            y=float(y),
            z=float(z),
            orientation=Quaternion(
                float(q_wxyz.x), float(q_wxyz.y), float(q_wxyz.z), float(q_wxyz.w)
            ),
        )
        return self._solve_and_target_cartesian(target, "left")

    @skill
    def move_right_hand_to(self, x: float, y: float, z: float) -> bool:
        """Move the right wrist to (x, y, z) metres in the pelvis frame.

        Orientation kept at its current value.
        """
        import pinocchio as pin

        ik = self._ensure_arm_ik()
        ik_q = ik.home_q()
        with self._joint_lock:
            snap = dict(self._joint_positions)
        for jname, pos in snap.items():
            full = jname + "_joint"
            if ik.model.existJointName(full):
                jid = ik.model.getJointId(full)
                ik_q[ik.model.joints[jid].idx_q] = float(pos)
        current = ik.fk_pose(ik_q, ik.right)
        q_wxyz = pin.Quaternion(current.rotation)
        target = PoseStamped(
            x=float(x),
            y=float(y),
            z=float(z),
            orientation=Quaternion(
                float(q_wxyz.x), float(q_wxyz.y), float(q_wxyz.z), float(q_wxyz.w)
            ),
        )
        return self._solve_and_target_cartesian(target, "right")

    # --- ROS2 sensor callbacks ---

    # --- Camera bridge (sidecar subprocess) ---

    def _start_camera_bridge(self) -> None:
        """Spawn the camera-bridge subprocess and start the reader thread.

        The bridge owns a fresh Python interpreter + fresh DDS state, avoiding
        the forkserver-worker bug where in-worker camera subscriptions stall
        after the first frame.

        We pin the bridge to CycloneDDS with a static unicast-only config:
        - CycloneDDS so it doesn't share FastDDS shared memory with the
          main worker (two FastDDS participants on the same host raced).
        - Unicast + explicit Peer so we don't depend on the multicast
          route that dimos adds to loopback for LCM (`224.0.0.0/4 dev lo`),
          which breaks normal DDS multicast discovery on enp2s0.
        - Interface pinned to enp2s0 (the robot's LAN).
        """
        import os as _os

        bridge = Path(__file__).parent / "_camera_bridge.py"
        env = _os.environ.copy()
        if self.config.clear_rmw_env:
            env.pop("RMW_IMPLEMENTATION", None)
            env.pop("CYCLONEDDS_URI", None)
        if self.config.force_cyclonedds:
            env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            env["CYCLONEDDS_URI"] = _cyclonedds_uri(
                self.config.cyclone_interface,
                self.config.cyclone_peer,
            )
        self._cam_stop = False
        self._cam_proc = subprocess.Popen(
            [sys.executable, str(bridge), _TOPIC_RGB_IMAGE],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,  # drop fastcdr "sequence size" noise
            bufsize=0,
            env=env,
        )
        self._cam_thread = Thread(target=self._read_camera_bridge, daemon=True)
        self._cam_thread.start()

    def _read_camera_bridge(self) -> None:
        """Consume length-prefixed JPEG frames from the bridge's stdout.

        Frame format on the wire: [u32 little-endian length] [length bytes JPEG].
        We decode, apply the 180° rotation (the X2 head RGBD sensor is mounted
        upside-down), and publish a dimos Image on color_image.
        """
        import cv2

        proc = self._cam_proc
        if proc is None or proc.stdout is None:
            return
        out = proc.stdout
        logger.info("X2Connection: camera-bridge reader thread started (pid=%s)", proc.pid)

        def _read_exact(n: int) -> bytes | None:
            buf = bytearray()
            while len(buf) < n:
                if self._cam_stop:
                    return None
                chunk = out.read(n - len(buf))
                if not chunk:
                    return None
                buf.extend(chunk)
            return bytes(buf)

        recv = 0
        try:
            while not self._cam_stop:
                header = _read_exact(4)
                if header is None:
                    logger.info("X2Connection: camera-bridge stdout EOF (after %d frames)", recv)
                    break
                (length,) = struct.unpack("<I", header)
                data = _read_exact(length)
                if data is None:
                    break

                jpeg = np.frombuffer(data, dtype=np.uint8)
                arr = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)  # BGR
                if arr is None:
                    continue

                # X2 head RGBD sensor is mounted with optical axis rotated 180°.
                arr = np.ascontiguousarray(arr[::-1, ::-1])

                image = Image(
                    data=arr,
                    format=ImageFormat.BGR,
                    frame_id="rgbd_head_front",
                    ts=time.time(),
                )
                self.color_image.publish(image)
                self._latest_video_frame = image
                recv += 1
                if recv == 1 or recv % 60 == 0:
                    logger.info(
                        "X2Connection: camera-bridge frame %d (%d KB)", recv, length // 1024
                    )
        except Exception as exc:
            logger.exception("X2Connection: camera-bridge reader crashed: %s", exc)

    def _stop_camera_bridge(self) -> None:
        self._cam_stop = True
        proc = self._cam_proc
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            self._cam_proc = None
        if self._cam_thread and self._cam_thread.is_alive():
            self._cam_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._cam_thread = None

    def _on_depth_image(self, msg: Any) -> None:
        image = _ros_image_to_dimos(msg)
        image.data = np.ascontiguousarray(image.data[::-1, ::-1])
        self.depth_image.publish(image)

    def _on_depth_cloud(self, msg: Any) -> None:
        self.pointcloud.publish(_ros_pointcloud2_to_dimos(msg))

    def _on_lidar(self, msg: Any) -> None:
        self.lidar.publish(_ros_pointcloud2_to_dimos(msg))

    def _on_imu(self, msg: Any) -> None:
        imu = _ros_imu_to_dimos(msg)
        self.imu.publish(imu)
        pose = PoseStamped(
            position=Vector3(0.0, 0.0, 0.0),
            orientation=imu.orientation,
            frame_id=self.config.frame_id,
            ts=imu.ts,
        )
        self.tf.publish(
            Transform(
                translation=pose.position,
                rotation=pose.orientation,
                frame_id=self.config.parent_frame_id,
                child_frame_id=self.config.frame_id,
                ts=imu.ts,
            )
        )
        self.odom.publish(pose)

    def _static_publish(self) -> None:
        if self.config.static_publish_rate <= 0:
            return
        remap = {X2Config.body_frame: self.config.frame_id, **self.config.frame_mapping}
        remapped_statics = [
            Transform(
                translation=transform.translation,
                rotation=transform.rotation,
                frame_id=remap.get(transform.frame_id, transform.frame_id),
                child_frame_id=remap.get(transform.child_frame_id, transform.child_frame_id),
            )
            for transform in self.config.static_transforms.values()
        ]
        period = 1.0 / self.config.static_publish_rate
        while True:
            now = time.time()
            self.tf.publish(
                *(
                    Transform(
                        translation=transform.translation,
                        rotation=transform.rotation,
                        frame_id=transform.frame_id,
                        child_frame_id=transform.child_frame_id,
                        ts=now,
                    )
                    for transform in remapped_statics
                )
            )
            time.sleep(period)

    def _on_camera_info(self, msg: Any) -> None:
        self.camera_info.publish(_ros_camera_info_to_dimos(msg))

    # Joint state visualization throttle: the robot publishes at ~750 Hz per
    # body part (motor-control telemetry rate). Anything above ~50 Hz is wasted
    # work for visualization and starves slower callbacks (camera) of executor
    # time. Update the internal map every message (cheap dict write); only
    # publish to dimos at JOINT_VIZ_HZ.
    _JOINT_VIZ_HZ = 50.0

    def _on_joint_state_array(self, msg: Any, joint_names: tuple[str, ...]) -> None:
        """Merge one body-part slice into the unified joint_positions and publish.

        AgiBot publishes per-bodypart JointStateArray messages with positionally
        indexed joints; we map them to names via the SDK's documented ordering
        and emit the merged set on every update.
        """
        joints = list(msg.joints)
        if len(joints) != len(joint_names):
            logger.warning(
                "X2Connection: joint topic %s len=%d != expected %d (skipping)",
                joint_names[0] if joint_names else "?",
                len(joints),
                len(joint_names),
            )
            return

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._joint_lock:
            for name, joint in zip(joint_names, joints, strict=True):
                self._joint_positions[name] = float(joint.position)
            # Once we've seen at least one arm sample, the controller can seed.
            if joint_names is _ARM_JOINT_NAMES:
                self._arm_state_seen = True
            # Rate-limit dimos publishes: cheap dict updates always happen,
            # only emit a merged snapshot when the throttle interval is up.
            now = time.monotonic()
            last = getattr(self, "_last_joint_pub", 0.0)
            if now - last < 1.0 / self._JOINT_VIZ_HZ:
                return
            self._last_joint_pub = now
            names = list(self._joint_positions.keys())
            positions = list(self._joint_positions.values())

        self.joint_state.publish(
            JointState(ts=ts, frame_id="pelvis", name=names, position=positions)
        )

    # --- Motion control ---

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a velocity command to the robot.

        Maps Twist fields:
          linear.x  → forward velocity  (±0.2-1.0 m/s or 0)
          linear.y  → lateral velocity  (±0.2-1.0 m/s or 0)
          angular.z → angular velocity  (±0.1-1.0 rad/s or 0)
        """
        if self._vel_publisher is None:
            return False

        from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader

        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self._ros_node.get_clock().now().to_msg()
        msg.source = _INPUT_SOURCE_NAME
        msg.forward_velocity = _clamp_velocity(twist.linear.x, _MIN_FORWARD, _MAX_FORWARD)
        msg.lateral_velocity = _clamp_velocity(twist.linear.y, _MIN_LATERAL, _MAX_LATERAL)
        msg.angular_velocity = _clamp_velocity(twist.angular.z, _MIN_ANGULAR, _MAX_ANGULAR)

        self._vel_publisher.publish(msg)

        if duration > 0.0:
            time.sleep(duration)
            self.stop_motion()

        return True

    @rpc
    def stop_motion(self) -> bool:
        """Send a zero-velocity command to stop the robot."""
        from dimos.msgs.geometry_msgs.Twist import Twist as DimTwist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3 as DimVec3

        return self.move(DimTwist(linear=DimVec3(0.0, 0.0, 0.0), angular=DimVec3(0.0, 0.0, 0.0)))

    @skill
    def observe(self) -> Image | None:
        """Returns the latest RGB frame from the robot's head camera.

        Use this skill for visual world queries.
        Returns None if no frame has been received yet.
        """
        return self._latest_video_frame

    # --- Helpers ---

    @staticmethod
    def _import_msg(module: str, cls: str) -> Any:
        import importlib

        return getattr(importlib.import_module(module), cls)
