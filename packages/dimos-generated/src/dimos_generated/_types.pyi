# Copyright 2020 - 2025 Ternaris
# SPDX-License-Identifier: Apache-2.0
#
# THIS FILE IS GENERATED, DO NOT EDIT
"""Message type definitions."""

# ruff: noqa: N801,N816,RUF100,TC004

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from rosbags.interfaces import Nodetype as T

if TYPE_CHECKING:
    from typing import ClassVar

    import numpy as np
    from rosbags.interfaces.typing import Typesdict

@dataclass
class builtin_interfaces__msg__Duration:
    """Class for builtin_interfaces/msg/Duration."""

    sec: int
    nanosec: int
    __msgtype__: ClassVar[str] = "builtin_interfaces/msg/Duration"

@dataclass
class builtin_interfaces__msg__Time:
    """Class for builtin_interfaces/msg/Time."""

    sec: int
    nanosec: int
    __msgtype__: ClassVar[str] = "builtin_interfaces/msg/Time"

@dataclass
class std_msgs__msg__Header:
    """Class for std_msgs/msg/Header."""

    stamp: builtin_interfaces__msg__Time
    frame_id: str
    __msgtype__: ClassVar[str] = "std_msgs/msg/Header"

@dataclass
class vision_msgs__msg__Point2D:
    """Class for vision_msgs/msg/Point2D."""

    x: float
    y: float
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Point2D"

@dataclass
class vision_msgs__msg__Pose2D:
    """Class for vision_msgs/msg/Pose2D."""

    position: vision_msgs__msg__Point2D
    theta: float
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Pose2D"

@dataclass
class vision_msgs__msg__BoundingBox2D:
    """Class for vision_msgs/msg/BoundingBox2D."""

    center: vision_msgs__msg__Pose2D
    size_x: float
    size_y: float
    __msgtype__: ClassVar[str] = "vision_msgs/msg/BoundingBox2D"

@dataclass
class dimos_msgs__msg__BoundingBox2DArray:
    """Class for dimos_msgs/msg/BoundingBox2DArray."""

    header: std_msgs__msg__Header
    boxes: list[vision_msgs__msg__BoundingBox2D]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/BoundingBox2DArray"

@dataclass
class geometry_msgs__msg__Point:
    """Class for geometry_msgs/msg/Point."""

    x: float
    y: float
    z: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Point"

@dataclass
class geometry_msgs__msg__Quaternion:
    """Class for geometry_msgs/msg/Quaternion."""

    x: float
    y: float
    z: float
    w: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Quaternion"

@dataclass
class geometry_msgs__msg__Pose:
    """Class for geometry_msgs/msg/Pose."""

    position: geometry_msgs__msg__Point
    orientation: geometry_msgs__msg__Quaternion
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Pose"

@dataclass
class geometry_msgs__msg__Vector3:
    """Class for geometry_msgs/msg/Vector3."""

    x: float
    y: float
    z: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Vector3"

@dataclass
class vision_msgs__msg__BoundingBox3D:
    """Class for vision_msgs/msg/BoundingBox3D."""

    center: geometry_msgs__msg__Pose
    size: geometry_msgs__msg__Vector3
    __msgtype__: ClassVar[str] = "vision_msgs/msg/BoundingBox3D"

@dataclass
class dimos_msgs__msg__BoundingBox3DArray:
    """Class for dimos_msgs/msg/BoundingBox3DArray."""

    header: std_msgs__msg__Header
    boxes: list[vision_msgs__msg__BoundingBox3D]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/BoundingBox3DArray"

@dataclass
class dimos_msgs__msg__EntityMarker:
    """Class for dimos_msgs/msg/EntityMarker."""

    entity_id: str
    label: str
    entity_type: str
    position: geometry_msgs__msg__Point
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/EntityMarker"

@dataclass
class dimos_msgs__msg__EntityMarkers:
    """Class for dimos_msgs/msg/EntityMarkers."""

    header: std_msgs__msg__Header
    markers: list[dimos_msgs__msg__EntityMarker]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/EntityMarkers"

@dataclass
class dimos_msgs__msg__GraspCandidate:
    """Class for dimos_msgs/msg/GraspCandidate."""

    pose: geometry_msgs__msg__Pose
    score: float
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/GraspCandidate"

@dataclass
class dimos_msgs__msg__GraspCandidateArray:
    """Class for dimos_msgs/msg/GraspCandidateArray."""

    header: std_msgs__msg__Header
    candidates: list[dimos_msgs__msg__GraspCandidate]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/GraspCandidateArray"

@dataclass
class dimos_msgs__msg__ImuInfo:
    """Class for dimos_msgs/msg/ImuInfo."""

    header: std_msgs__msg__Header
    gyro_noise_density: float
    gyro_random_walk: float
    accel_noise_density: float
    accel_random_walk: float
    frequency: float
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/ImuInfo"

@dataclass
class dimos_msgs__msg__JointCommand:
    """Class for dimos_msgs/msg/JointCommand."""

    header: std_msgs__msg__Header
    positions: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/JointCommand"

@dataclass
class dimos_msgs__msg__LineSegment3D:
    """Class for dimos_msgs/msg/LineSegment3D."""

    start: geometry_msgs__msg__Point
    end: geometry_msgs__msg__Point
    weight: float
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/LineSegment3D"

@dataclass
class dimos_msgs__msg__LineSegments3D:
    """Class for dimos_msgs/msg/LineSegments3D."""

    header: std_msgs__msg__Header
    segments: list[dimos_msgs__msg__LineSegment3D]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/LineSegments3D"

@dataclass
class dimos_msgs__msg__MotorCommandArray:
    """Class for dimos_msgs/msg/MotorCommandArray."""

    header: std_msgs__msg__Header
    q: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    dq: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    kp: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    kd: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    tau: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/MotorCommandArray"

@dataclass
class dimos_msgs__msg__RobotState:
    """Class for dimos_msgs/msg/RobotState."""

    header: std_msgs__msg__Header
    state: int
    mode: int
    error_code: int
    warn_code: int
    cmdnum: int
    mt_brake: int
    mt_able: int
    tcp_pose: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    tcp_offset: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    joints: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/RobotState"

@dataclass
class dimos_msgs__msg__TrajectoryStatus:
    """Class for dimos_msgs/msg/TrajectoryStatus."""

    header: std_msgs__msg__Header
    state: int
    progress: float
    time_elapsed: builtin_interfaces__msg__Duration
    time_remaining: builtin_interfaces__msg__Duration
    error: str
    IDLE: ClassVar[int] = 0
    EXECUTING: ClassVar[int] = 1
    COMPLETED: ClassVar[int] = 2
    ABORTED: ClassVar[int] = 3
    FAULT: ClassVar[int] = 4
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/TrajectoryStatus"

@dataclass
class dimos_msgs__msg__VideoStats:
    """Class for dimos_msgs/msg/VideoStats."""

    header: std_msgs__msg__Header
    fps: float
    kbps: float
    width: int
    height: int
    loss_pct: float
    jitter_buffer_ms: float
    decode_ms: float
    frames_dropped: int
    freezes: int
    e2e_latency_ms: float
    __msgtype__: ClassVar[str] = "dimos_msgs/msg/VideoStats"

@dataclass
class foxglove_msgs__msg__CompressedVideo:
    """Class for foxglove_msgs/msg/CompressedVideo."""

    timestamp: builtin_interfaces__msg__Time
    frame_id: str
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    format: str
    __msgtype__: ClassVar[str] = "foxglove_msgs/msg/CompressedVideo"

@dataclass
class geometry_msgs__msg__Accel:
    """Class for geometry_msgs/msg/Accel."""

    linear: geometry_msgs__msg__Vector3
    angular: geometry_msgs__msg__Vector3
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Accel"

@dataclass
class geometry_msgs__msg__AccelStamped:
    """Class for geometry_msgs/msg/AccelStamped."""

    header: std_msgs__msg__Header
    accel: geometry_msgs__msg__Accel
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelStamped"

@dataclass
class geometry_msgs__msg__AccelWithCovariance:
    """Class for geometry_msgs/msg/AccelWithCovariance."""

    accel: geometry_msgs__msg__Accel
    covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelWithCovariance"

@dataclass
class geometry_msgs__msg__AccelWithCovarianceStamped:
    """Class for geometry_msgs/msg/AccelWithCovarianceStamped."""

    header: std_msgs__msg__Header
    accel: geometry_msgs__msg__AccelWithCovariance
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/AccelWithCovarianceStamped"

@dataclass
class geometry_msgs__msg__Inertia:
    """Class for geometry_msgs/msg/Inertia."""

    m: float
    com: geometry_msgs__msg__Vector3
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Inertia"

@dataclass
class geometry_msgs__msg__InertiaStamped:
    """Class for geometry_msgs/msg/InertiaStamped."""

    header: std_msgs__msg__Header
    inertia: geometry_msgs__msg__Inertia
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/InertiaStamped"

@dataclass
class geometry_msgs__msg__Point32:
    """Class for geometry_msgs/msg/Point32."""

    x: float
    y: float
    z: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Point32"

@dataclass
class geometry_msgs__msg__PointStamped:
    """Class for geometry_msgs/msg/PointStamped."""

    header: std_msgs__msg__Header
    point: geometry_msgs__msg__Point
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PointStamped"

@dataclass
class geometry_msgs__msg__Polygon:
    """Class for geometry_msgs/msg/Polygon."""

    points: list[geometry_msgs__msg__Point32]
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Polygon"

@dataclass
class geometry_msgs__msg__PolygonInstance:
    """Class for geometry_msgs/msg/PolygonInstance."""

    polygon: geometry_msgs__msg__Polygon
    id: int
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonInstance"

@dataclass
class geometry_msgs__msg__PolygonInstanceStamped:
    """Class for geometry_msgs/msg/PolygonInstanceStamped."""

    header: std_msgs__msg__Header
    polygon: geometry_msgs__msg__PolygonInstance
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonInstanceStamped"

@dataclass
class geometry_msgs__msg__PolygonStamped:
    """Class for geometry_msgs/msg/PolygonStamped."""

    header: std_msgs__msg__Header
    polygon: geometry_msgs__msg__Polygon
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PolygonStamped"

@dataclass
class geometry_msgs__msg__Pose2D:
    """Class for geometry_msgs/msg/Pose2D."""

    x: float
    y: float
    theta: float
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Pose2D"

@dataclass
class geometry_msgs__msg__PoseArray:
    """Class for geometry_msgs/msg/PoseArray."""

    header: std_msgs__msg__Header
    poses: list[geometry_msgs__msg__Pose]
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseArray"

@dataclass
class geometry_msgs__msg__PoseStamped:
    """Class for geometry_msgs/msg/PoseStamped."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__Pose
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseStamped"

@dataclass
class geometry_msgs__msg__PoseWithCovariance:
    """Class for geometry_msgs/msg/PoseWithCovariance."""

    pose: geometry_msgs__msg__Pose
    covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseWithCovariance"

@dataclass
class geometry_msgs__msg__PoseWithCovarianceStamped:
    """Class for geometry_msgs/msg/PoseWithCovarianceStamped."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__PoseWithCovariance
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/PoseWithCovarianceStamped"

@dataclass
class geometry_msgs__msg__QuaternionStamped:
    """Class for geometry_msgs/msg/QuaternionStamped."""

    header: std_msgs__msg__Header
    quaternion: geometry_msgs__msg__Quaternion
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/QuaternionStamped"

@dataclass
class geometry_msgs__msg__Transform:
    """Class for geometry_msgs/msg/Transform."""

    translation: geometry_msgs__msg__Vector3
    rotation: geometry_msgs__msg__Quaternion
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Transform"

@dataclass
class geometry_msgs__msg__TransformStamped:
    """Class for geometry_msgs/msg/TransformStamped."""

    header: std_msgs__msg__Header
    child_frame_id: str
    transform: geometry_msgs__msg__Transform
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TransformStamped"

@dataclass
class geometry_msgs__msg__Twist:
    """Class for geometry_msgs/msg/Twist."""

    linear: geometry_msgs__msg__Vector3
    angular: geometry_msgs__msg__Vector3
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Twist"

@dataclass
class geometry_msgs__msg__TwistStamped:
    """Class for geometry_msgs/msg/TwistStamped."""

    header: std_msgs__msg__Header
    twist: geometry_msgs__msg__Twist
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistStamped"

@dataclass
class geometry_msgs__msg__TwistWithCovariance:
    """Class for geometry_msgs/msg/TwistWithCovariance."""

    twist: geometry_msgs__msg__Twist
    covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistWithCovariance"

@dataclass
class geometry_msgs__msg__TwistWithCovarianceStamped:
    """Class for geometry_msgs/msg/TwistWithCovarianceStamped."""

    header: std_msgs__msg__Header
    twist: geometry_msgs__msg__TwistWithCovariance
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/TwistWithCovarianceStamped"

@dataclass
class geometry_msgs__msg__Vector3Stamped:
    """Class for geometry_msgs/msg/Vector3Stamped."""

    header: std_msgs__msg__Header
    vector: geometry_msgs__msg__Vector3
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Vector3Stamped"

@dataclass
class geometry_msgs__msg__VelocityStamped:
    """Class for geometry_msgs/msg/VelocityStamped."""

    header: std_msgs__msg__Header
    body_frame_id: str
    reference_frame_id: str
    velocity: geometry_msgs__msg__Twist
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/VelocityStamped"

@dataclass
class geometry_msgs__msg__VelocityWithCovarianceStamped:
    """Class for geometry_msgs/msg/VelocityWithCovarianceStamped."""

    header: std_msgs__msg__Header
    body_frame_id: str
    reference_frame_id: str
    velocity: geometry_msgs__msg__TwistWithCovariance
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/VelocityWithCovarianceStamped"

@dataclass
class geometry_msgs__msg__Wrench:
    """Class for geometry_msgs/msg/Wrench."""

    force: geometry_msgs__msg__Vector3
    torque: geometry_msgs__msg__Vector3
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/Wrench"

@dataclass
class geometry_msgs__msg__WrenchStamped:
    """Class for geometry_msgs/msg/WrenchStamped."""

    header: std_msgs__msg__Header
    wrench: geometry_msgs__msg__Wrench
    __msgtype__: ClassVar[str] = "geometry_msgs/msg/WrenchStamped"

@dataclass
class nav_msgs__msg__Goals:
    """Class for nav_msgs/msg/Goals."""

    header: std_msgs__msg__Header
    goals: list[geometry_msgs__msg__PoseStamped]
    __msgtype__: ClassVar[str] = "nav_msgs/msg/Goals"

@dataclass
class nav_msgs__msg__GridCells:
    """Class for nav_msgs/msg/GridCells."""

    header: std_msgs__msg__Header
    cell_width: float
    cell_height: float
    cells: list[geometry_msgs__msg__Point]
    __msgtype__: ClassVar[str] = "nav_msgs/msg/GridCells"

@dataclass
class nav_msgs__msg__MapMetaData:
    """Class for nav_msgs/msg/MapMetaData."""

    map_load_time: builtin_interfaces__msg__Time
    resolution: float
    width: int
    height: int
    origin: geometry_msgs__msg__Pose
    __msgtype__: ClassVar[str] = "nav_msgs/msg/MapMetaData"

@dataclass
class nav_msgs__msg__OccupancyGrid:
    """Class for nav_msgs/msg/OccupancyGrid."""

    header: std_msgs__msg__Header
    info: nav_msgs__msg__MapMetaData
    data: np.ndarray[tuple[int, ...], np.dtype[np.int8]]
    __msgtype__: ClassVar[str] = "nav_msgs/msg/OccupancyGrid"

@dataclass
class nav_msgs__msg__Odometry:
    """Class for nav_msgs/msg/Odometry."""

    header: std_msgs__msg__Header
    child_frame_id: str
    pose: geometry_msgs__msg__PoseWithCovariance
    twist: geometry_msgs__msg__TwistWithCovariance
    __msgtype__: ClassVar[str] = "nav_msgs/msg/Odometry"

@dataclass
class nav_msgs__msg__Path:
    """Class for nav_msgs/msg/Path."""

    header: std_msgs__msg__Header
    poses: list[geometry_msgs__msg__PoseStamped]
    __msgtype__: ClassVar[str] = "nav_msgs/msg/Path"

@dataclass
class nav_msgs__msg__TrajectoryPoint:
    """Class for nav_msgs/msg/TrajectoryPoint."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__Pose
    velocity: geometry_msgs__msg__Twist
    acceleration: geometry_msgs__msg__Accel
    effort: geometry_msgs__msg__Wrench
    __msgtype__: ClassVar[str] = "nav_msgs/msg/TrajectoryPoint"

@dataclass
class nav_msgs__msg__Trajectory:
    """Class for nav_msgs/msg/Trajectory."""

    header: std_msgs__msg__Header
    points: list[nav_msgs__msg__TrajectoryPoint]
    __msgtype__: ClassVar[str] = "nav_msgs/msg/Trajectory"

@dataclass
class sensor_msgs__msg__BatteryState:
    """Class for sensor_msgs/msg/BatteryState."""

    header: std_msgs__msg__Header
    voltage: float
    temperature: float
    current: float
    charge: float
    capacity: float
    design_capacity: float
    percentage: float
    power_supply_status: int
    power_supply_health: int
    power_supply_technology: int
    present: bool
    cell_voltage: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    cell_temperature: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    location: str
    serial_number: str
    POWER_SUPPLY_STATUS_UNKNOWN: ClassVar[int] = 0
    POWER_SUPPLY_STATUS_CHARGING: ClassVar[int] = 1
    POWER_SUPPLY_STATUS_DISCHARGING: ClassVar[int] = 2
    POWER_SUPPLY_STATUS_NOT_CHARGING: ClassVar[int] = 3
    POWER_SUPPLY_STATUS_FULL: ClassVar[int] = 4
    POWER_SUPPLY_HEALTH_UNKNOWN: ClassVar[int] = 0
    POWER_SUPPLY_HEALTH_GOOD: ClassVar[int] = 1
    POWER_SUPPLY_HEALTH_OVERHEAT: ClassVar[int] = 2
    POWER_SUPPLY_HEALTH_DEAD: ClassVar[int] = 3
    POWER_SUPPLY_HEALTH_OVERVOLTAGE: ClassVar[int] = 4
    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: ClassVar[int] = 5
    POWER_SUPPLY_HEALTH_COLD: ClassVar[int] = 6
    POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: ClassVar[int] = 7
    POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: ClassVar[int] = 8
    POWER_SUPPLY_TECHNOLOGY_UNKNOWN: ClassVar[int] = 0
    POWER_SUPPLY_TECHNOLOGY_NIMH: ClassVar[int] = 1
    POWER_SUPPLY_TECHNOLOGY_LION: ClassVar[int] = 2
    POWER_SUPPLY_TECHNOLOGY_LIPO: ClassVar[int] = 3
    POWER_SUPPLY_TECHNOLOGY_LIFE: ClassVar[int] = 4
    POWER_SUPPLY_TECHNOLOGY_NICD: ClassVar[int] = 5
    POWER_SUPPLY_TECHNOLOGY_LIMN: ClassVar[int] = 6
    POWER_SUPPLY_TECHNOLOGY_TERNARY: ClassVar[int] = 7
    POWER_SUPPLY_TECHNOLOGY_VRLA: ClassVar[int] = 8
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/BatteryState"

@dataclass
class sensor_msgs__msg__RegionOfInterest:
    """Class for sensor_msgs/msg/RegionOfInterest."""

    x_offset: int
    y_offset: int
    height: int
    width: int
    do_rectify: bool
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/RegionOfInterest"

@dataclass
class sensor_msgs__msg__CameraInfo:
    """Class for sensor_msgs/msg/CameraInfo."""

    header: std_msgs__msg__Header
    height: int
    width: int
    distortion_model: str
    d: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    k: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    r: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    p: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    binning_x: int
    binning_y: int
    roi: sensor_msgs__msg__RegionOfInterest
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/CameraInfo"

@dataclass
class sensor_msgs__msg__ChannelFloat32:
    """Class for sensor_msgs/msg/ChannelFloat32."""

    name: str
    values: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/ChannelFloat32"

@dataclass
class sensor_msgs__msg__CompressedImage:
    """Class for sensor_msgs/msg/CompressedImage."""

    header: std_msgs__msg__Header
    format: str
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/CompressedImage"

@dataclass
class sensor_msgs__msg__FluidPressure:
    """Class for sensor_msgs/msg/FluidPressure."""

    header: std_msgs__msg__Header
    fluid_pressure: float
    variance: float
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/FluidPressure"

@dataclass
class sensor_msgs__msg__Illuminance:
    """Class for sensor_msgs/msg/Illuminance."""

    header: std_msgs__msg__Header
    illuminance: float
    variance: float
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Illuminance"

@dataclass
class sensor_msgs__msg__Image:
    """Class for sensor_msgs/msg/Image."""

    header: std_msgs__msg__Header
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Image"

@dataclass
class sensor_msgs__msg__Imu:
    """Class for sensor_msgs/msg/Imu."""

    header: std_msgs__msg__Header
    orientation: geometry_msgs__msg__Quaternion
    orientation_covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    angular_velocity: geometry_msgs__msg__Vector3
    angular_velocity_covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    linear_acceleration: geometry_msgs__msg__Vector3
    linear_acceleration_covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Imu"

@dataclass
class sensor_msgs__msg__JointState:
    """Class for sensor_msgs/msg/JointState."""

    header: std_msgs__msg__Header
    name: list[str]
    position: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    velocity: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    effort: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JointState"

@dataclass
class sensor_msgs__msg__Joy:
    """Class for sensor_msgs/msg/Joy."""

    header: std_msgs__msg__Header
    axes: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    buttons: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Joy"

@dataclass
class sensor_msgs__msg__JoyFeedback:
    """Class for sensor_msgs/msg/JoyFeedback."""

    type: int
    id: int
    intensity: float
    TYPE_LED: ClassVar[int] = 0
    TYPE_RUMBLE: ClassVar[int] = 1
    TYPE_BUZZER: ClassVar[int] = 2
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JoyFeedback"

@dataclass
class sensor_msgs__msg__JoyFeedbackArray:
    """Class for sensor_msgs/msg/JoyFeedbackArray."""

    array: list[sensor_msgs__msg__JoyFeedback]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/JoyFeedbackArray"

@dataclass
class sensor_msgs__msg__LaserEcho:
    """Class for sensor_msgs/msg/LaserEcho."""

    echoes: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/LaserEcho"

@dataclass
class sensor_msgs__msg__LaserScan:
    """Class for sensor_msgs/msg/LaserScan."""

    header: std_msgs__msg__Header
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    intensities: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/LaserScan"

@dataclass
class sensor_msgs__msg__MagneticField:
    """Class for sensor_msgs/msg/MagneticField."""

    header: std_msgs__msg__Header
    magnetic_field: geometry_msgs__msg__Vector3
    magnetic_field_covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MagneticField"

@dataclass
class sensor_msgs__msg__MultiDOFJointState:
    """Class for sensor_msgs/msg/MultiDOFJointState."""

    header: std_msgs__msg__Header
    joint_names: list[str]
    transforms: list[geometry_msgs__msg__Transform]
    twist: list[geometry_msgs__msg__Twist]
    wrench: list[geometry_msgs__msg__Wrench]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MultiDOFJointState"

@dataclass
class sensor_msgs__msg__MultiEchoLaserScan:
    """Class for sensor_msgs/msg/MultiEchoLaserScan."""

    header: std_msgs__msg__Header
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: list[sensor_msgs__msg__LaserEcho]
    intensities: list[sensor_msgs__msg__LaserEcho]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/MultiEchoLaserScan"

@dataclass
class sensor_msgs__msg__NavSatStatus:
    """Class for sensor_msgs/msg/NavSatStatus."""

    status: int
    service: int
    STATUS_UNKNOWN: ClassVar[int] = -2
    STATUS_NO_FIX: ClassVar[int] = -1
    STATUS_FIX: ClassVar[int] = 0
    STATUS_SBAS_FIX: ClassVar[int] = 1
    STATUS_GBAS_FIX: ClassVar[int] = 2
    SERVICE_UNKNOWN: ClassVar[int] = 0
    SERVICE_GPS: ClassVar[int] = 1
    SERVICE_GLONASS: ClassVar[int] = 2
    SERVICE_COMPASS: ClassVar[int] = 4
    SERVICE_GALILEO: ClassVar[int] = 8
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/NavSatStatus"

@dataclass
class sensor_msgs__msg__NavSatFix:
    """Class for sensor_msgs/msg/NavSatFix."""

    header: std_msgs__msg__Header
    status: sensor_msgs__msg__NavSatStatus
    latitude: float
    longitude: float
    altitude: float
    position_covariance: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    position_covariance_type: int
    COVARIANCE_TYPE_UNKNOWN: ClassVar[int] = 0
    COVARIANCE_TYPE_APPROXIMATED: ClassVar[int] = 1
    COVARIANCE_TYPE_DIAGONAL_KNOWN: ClassVar[int] = 2
    COVARIANCE_TYPE_KNOWN: ClassVar[int] = 3
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/NavSatFix"

@dataclass
class sensor_msgs__msg__PointCloud:
    """Class for sensor_msgs/msg/PointCloud."""

    header: std_msgs__msg__Header
    points: list[geometry_msgs__msg__Point32]
    channels: list[sensor_msgs__msg__ChannelFloat32]
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointCloud"

@dataclass
class sensor_msgs__msg__PointField:
    """Class for sensor_msgs/msg/PointField."""

    name: str
    offset: int
    datatype: int
    count: int
    INT8: ClassVar[int] = 1
    UINT8: ClassVar[int] = 2
    INT16: ClassVar[int] = 3
    UINT16: ClassVar[int] = 4
    INT32: ClassVar[int] = 5
    UINT32: ClassVar[int] = 6
    FLOAT32: ClassVar[int] = 7
    FLOAT64: ClassVar[int] = 8
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointField"

@dataclass
class sensor_msgs__msg__PointCloud2:
    """Class for sensor_msgs/msg/PointCloud2."""

    header: std_msgs__msg__Header
    height: int
    width: int
    fields: list[sensor_msgs__msg__PointField]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    is_dense: bool
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointCloud2"

@dataclass
class sensor_msgs__msg__Range:
    """Class for sensor_msgs/msg/Range."""

    header: std_msgs__msg__Header
    radiation_type: int
    field_of_view: float
    min_range: float
    max_range: float
    range: float
    variance: float
    ULTRASOUND: ClassVar[int] = 0
    INFRARED: ClassVar[int] = 1
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Range"

@dataclass
class sensor_msgs__msg__RelativeHumidity:
    """Class for sensor_msgs/msg/RelativeHumidity."""

    header: std_msgs__msg__Header
    relative_humidity: float
    variance: float
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/RelativeHumidity"

@dataclass
class sensor_msgs__msg__Temperature:
    """Class for sensor_msgs/msg/Temperature."""

    header: std_msgs__msg__Header
    temperature: float
    variance: float
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/Temperature"

@dataclass
class sensor_msgs__msg__TimeReference:
    """Class for sensor_msgs/msg/TimeReference."""

    header: std_msgs__msg__Header
    time_ref: builtin_interfaces__msg__Time
    source: str
    __msgtype__: ClassVar[str] = "sensor_msgs/msg/TimeReference"

@dataclass
class shape_msgs__msg__MeshTriangle:
    """Class for shape_msgs/msg/MeshTriangle."""

    vertex_indices: np.ndarray[tuple[int, ...], np.dtype[np.uint32]]
    __msgtype__: ClassVar[str] = "shape_msgs/msg/MeshTriangle"

@dataclass
class shape_msgs__msg__Mesh:
    """Class for shape_msgs/msg/Mesh."""

    triangles: list[shape_msgs__msg__MeshTriangle]
    vertices: list[geometry_msgs__msg__Point]
    __msgtype__: ClassVar[str] = "shape_msgs/msg/Mesh"

@dataclass
class shape_msgs__msg__Plane:
    """Class for shape_msgs/msg/Plane."""

    coef: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "shape_msgs/msg/Plane"

@dataclass
class shape_msgs__msg__SolidPrimitive:
    """Class for shape_msgs/msg/SolidPrimitive."""

    type: int
    dimensions: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    polygon: geometry_msgs__msg__Polygon
    BOX: ClassVar[int] = 1
    SPHERE: ClassVar[int] = 2
    CYLINDER: ClassVar[int] = 3
    CONE: ClassVar[int] = 4
    PRISM: ClassVar[int] = 5
    BOX_X: ClassVar[int] = 0
    BOX_Y: ClassVar[int] = 1
    BOX_Z: ClassVar[int] = 2
    SPHERE_RADIUS: ClassVar[int] = 0
    CYLINDER_HEIGHT: ClassVar[int] = 0
    CYLINDER_RADIUS: ClassVar[int] = 1
    CONE_HEIGHT: ClassVar[int] = 0
    CONE_RADIUS: ClassVar[int] = 1
    PRISM_HEIGHT: ClassVar[int] = 0
    __msgtype__: ClassVar[str] = "shape_msgs/msg/SolidPrimitive"

@dataclass
class std_msgs__msg__Bool:
    """Class for std_msgs/msg/Bool."""

    data: bool
    __msgtype__: ClassVar[str] = "std_msgs/msg/Bool"

@dataclass
class std_msgs__msg__Byte:
    """Class for std_msgs/msg/Byte."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Byte"

@dataclass
class std_msgs__msg__MultiArrayDimension:
    """Class for std_msgs/msg/MultiArrayDimension."""

    label: str
    size: int
    stride: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/MultiArrayDimension"

@dataclass
class std_msgs__msg__MultiArrayLayout:
    """Class for std_msgs/msg/MultiArrayLayout."""

    dim: list[std_msgs__msg__MultiArrayDimension]
    data_offset: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/MultiArrayLayout"

@dataclass
class std_msgs__msg__ByteMultiArray:
    """Class for std_msgs/msg/ByteMultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/ByteMultiArray"

@dataclass
class std_msgs__msg__Char:
    """Class for std_msgs/msg/Char."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Char"

@dataclass
class std_msgs__msg__ColorRGBA:
    """Class for std_msgs/msg/ColorRGBA."""

    r: float
    g: float
    b: float
    a: float
    __msgtype__: ClassVar[str] = "std_msgs/msg/ColorRGBA"

@dataclass
class std_msgs__msg__Empty:
    """Class for std_msgs/msg/Empty."""

    structure_needs_at_least_one_member: int = 0
    __msgtype__: ClassVar[str] = "std_msgs/msg/Empty"

@dataclass
class std_msgs__msg__Float32:
    """Class for std_msgs/msg/Float32."""

    data: float
    __msgtype__: ClassVar[str] = "std_msgs/msg/Float32"

@dataclass
class std_msgs__msg__Float32MultiArray:
    """Class for std_msgs/msg/Float32MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.float32]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Float32MultiArray"

@dataclass
class std_msgs__msg__Float64:
    """Class for std_msgs/msg/Float64."""

    data: float
    __msgtype__: ClassVar[str] = "std_msgs/msg/Float64"

@dataclass
class std_msgs__msg__Float64MultiArray:
    """Class for std_msgs/msg/Float64MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Float64MultiArray"

@dataclass
class std_msgs__msg__Int16:
    """Class for std_msgs/msg/Int16."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int16"

@dataclass
class std_msgs__msg__Int16MultiArray:
    """Class for std_msgs/msg/Int16MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int16]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int16MultiArray"

@dataclass
class std_msgs__msg__Int32:
    """Class for std_msgs/msg/Int32."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int32"

@dataclass
class std_msgs__msg__Int32MultiArray:
    """Class for std_msgs/msg/Int32MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int32]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int32MultiArray"

@dataclass
class std_msgs__msg__Int64:
    """Class for std_msgs/msg/Int64."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int64"

@dataclass
class std_msgs__msg__Int64MultiArray:
    """Class for std_msgs/msg/Int64MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int64]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int64MultiArray"

@dataclass
class std_msgs__msg__Int8:
    """Class for std_msgs/msg/Int8."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int8"

@dataclass
class std_msgs__msg__Int8MultiArray:
    """Class for std_msgs/msg/Int8MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.int8]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/Int8MultiArray"

@dataclass
class std_msgs__msg__String:
    """Class for std_msgs/msg/String."""

    data: str
    __msgtype__: ClassVar[str] = "std_msgs/msg/String"

@dataclass
class std_msgs__msg__UInt16:
    """Class for std_msgs/msg/UInt16."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt16"

@dataclass
class std_msgs__msg__UInt16MultiArray:
    """Class for std_msgs/msg/UInt16MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint16]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt16MultiArray"

@dataclass
class std_msgs__msg__UInt32:
    """Class for std_msgs/msg/UInt32."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt32"

@dataclass
class std_msgs__msg__UInt32MultiArray:
    """Class for std_msgs/msg/UInt32MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint32]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt32MultiArray"

@dataclass
class std_msgs__msg__UInt64:
    """Class for std_msgs/msg/UInt64."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt64"

@dataclass
class std_msgs__msg__UInt64MultiArray:
    """Class for std_msgs/msg/UInt64MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint64]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt64MultiArray"

@dataclass
class std_msgs__msg__UInt8:
    """Class for std_msgs/msg/UInt8."""

    data: int
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt8"

@dataclass
class std_msgs__msg__UInt8MultiArray:
    """Class for std_msgs/msg/UInt8MultiArray."""

    layout: std_msgs__msg__MultiArrayLayout
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = "std_msgs/msg/UInt8MultiArray"

@dataclass
class tf2_msgs__msg__TF2Error:
    """Class for tf2_msgs/msg/TF2Error."""

    error: int
    error_string: str
    NO_ERROR: ClassVar[int] = 0
    LOOKUP_ERROR: ClassVar[int] = 1
    CONNECTIVITY_ERROR: ClassVar[int] = 2
    EXTRAPOLATION_ERROR: ClassVar[int] = 3
    INVALID_ARGUMENT_ERROR: ClassVar[int] = 4
    TIMEOUT_ERROR: ClassVar[int] = 5
    TRANSFORM_ERROR: ClassVar[int] = 6
    __msgtype__: ClassVar[str] = "tf2_msgs/msg/TF2Error"

@dataclass
class tf2_msgs__msg__TFMessage:
    """Class for tf2_msgs/msg/TFMessage."""

    transforms: list[geometry_msgs__msg__TransformStamped]
    __msgtype__: ClassVar[str] = "tf2_msgs/msg/TFMessage"

@dataclass
class trajectory_msgs__msg__JointTrajectoryPoint:
    """Class for trajectory_msgs/msg/JointTrajectoryPoint."""

    positions: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    velocities: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    accelerations: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    effort: np.ndarray[tuple[int, ...], np.dtype[np.float64]]
    time_from_start: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = "trajectory_msgs/msg/JointTrajectoryPoint"

@dataclass
class trajectory_msgs__msg__JointTrajectory:
    """Class for trajectory_msgs/msg/JointTrajectory."""

    header: std_msgs__msg__Header
    joint_names: list[str]
    points: list[trajectory_msgs__msg__JointTrajectoryPoint]
    __msgtype__: ClassVar[str] = "trajectory_msgs/msg/JointTrajectory"

@dataclass
class trajectory_msgs__msg__MultiDOFJointTrajectoryPoint:
    """Class for trajectory_msgs/msg/MultiDOFJointTrajectoryPoint."""

    transforms: list[geometry_msgs__msg__Transform]
    velocities: list[geometry_msgs__msg__Twist]
    accelerations: list[geometry_msgs__msg__Twist]
    time_from_start: builtin_interfaces__msg__Duration
    __msgtype__: ClassVar[str] = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint"

@dataclass
class trajectory_msgs__msg__MultiDOFJointTrajectory:
    """Class for trajectory_msgs/msg/MultiDOFJointTrajectory."""

    header: std_msgs__msg__Header
    joint_names: list[str]
    points: list[trajectory_msgs__msg__MultiDOFJointTrajectoryPoint]
    __msgtype__: ClassVar[str] = "trajectory_msgs/msg/MultiDOFJointTrajectory"

@dataclass
class vision_msgs__msg__BoundingBox2DArray:
    """Class for vision_msgs/msg/BoundingBox2DArray."""

    header: std_msgs__msg__Header
    boxes: list[vision_msgs__msg__BoundingBox2D]
    __msgtype__: ClassVar[str] = "vision_msgs/msg/BoundingBox2DArray"

@dataclass
class vision_msgs__msg__BoundingBox3DArray:
    """Class for vision_msgs/msg/BoundingBox3DArray."""

    header: std_msgs__msg__Header
    boxes: list[vision_msgs__msg__BoundingBox3D]
    __msgtype__: ClassVar[str] = "vision_msgs/msg/BoundingBox3DArray"

@dataclass
class vision_msgs__msg__ObjectHypothesis:
    """Class for vision_msgs/msg/ObjectHypothesis."""

    class_id: str
    score: float
    __msgtype__: ClassVar[str] = "vision_msgs/msg/ObjectHypothesis"

@dataclass
class vision_msgs__msg__Classification:
    """Class for vision_msgs/msg/Classification."""

    header: std_msgs__msg__Header
    results: list[vision_msgs__msg__ObjectHypothesis]
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Classification"

@dataclass
class vision_msgs__msg__ObjectHypothesisWithPose:
    """Class for vision_msgs/msg/ObjectHypothesisWithPose."""

    hypothesis: vision_msgs__msg__ObjectHypothesis
    pose: geometry_msgs__msg__PoseWithCovariance
    __msgtype__: ClassVar[str] = "vision_msgs/msg/ObjectHypothesisWithPose"

@dataclass
class vision_msgs__msg__Detection2D:
    """Class for vision_msgs/msg/Detection2D."""

    header: std_msgs__msg__Header
    results: list[vision_msgs__msg__ObjectHypothesisWithPose]
    bbox: vision_msgs__msg__BoundingBox2D
    id: str
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Detection2D"

@dataclass
class vision_msgs__msg__Detection2DArray:
    """Class for vision_msgs/msg/Detection2DArray."""

    header: std_msgs__msg__Header
    detections: list[vision_msgs__msg__Detection2D]
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Detection2DArray"

@dataclass
class vision_msgs__msg__Detection3D:
    """Class for vision_msgs/msg/Detection3D."""

    header: std_msgs__msg__Header
    results: list[vision_msgs__msg__ObjectHypothesisWithPose]
    bbox: vision_msgs__msg__BoundingBox3D
    id: str
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Detection3D"

@dataclass
class vision_msgs__msg__Detection3DArray:
    """Class for vision_msgs/msg/Detection3DArray."""

    header: std_msgs__msg__Header
    detections: list[vision_msgs__msg__Detection3D]
    __msgtype__: ClassVar[str] = "vision_msgs/msg/Detection3DArray"

@dataclass
class vision_msgs__msg__VisionClass:
    """Class for vision_msgs/msg/VisionClass."""

    class_id: int
    class_name: str
    __msgtype__: ClassVar[str] = "vision_msgs/msg/VisionClass"

@dataclass
class vision_msgs__msg__LabelInfo:
    """Class for vision_msgs/msg/LabelInfo."""

    header: std_msgs__msg__Header
    class_map: list[vision_msgs__msg__VisionClass]
    threshold: float
    __msgtype__: ClassVar[str] = "vision_msgs/msg/LabelInfo"

@dataclass
class vision_msgs__msg__VisionInfo:
    """Class for vision_msgs/msg/VisionInfo."""

    header: std_msgs__msg__Header
    method: str
    database_location: str
    database_version: int
    __msgtype__: ClassVar[str] = "vision_msgs/msg/VisionInfo"

@dataclass
class visualization_msgs__msg__ImageMarker:
    """Class for visualization_msgs/msg/ImageMarker."""

    header: std_msgs__msg__Header
    ns: str
    id: int
    type: int
    action: int
    position: geometry_msgs__msg__Point
    scale: float
    outline_color: std_msgs__msg__ColorRGBA
    filled: int
    fill_color: std_msgs__msg__ColorRGBA
    lifetime: builtin_interfaces__msg__Duration
    points: list[geometry_msgs__msg__Point]
    outline_colors: list[std_msgs__msg__ColorRGBA]
    CIRCLE: ClassVar[int] = 0
    LINE_STRIP: ClassVar[int] = 1
    LINE_LIST: ClassVar[int] = 2
    POLYGON: ClassVar[int] = 3
    POINTS: ClassVar[int] = 4
    ADD: ClassVar[int] = 0
    REMOVE: ClassVar[int] = 1
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/ImageMarker"

@dataclass
class visualization_msgs__msg__MeshFile:
    """Class for visualization_msgs/msg/MeshFile."""

    filename: str
    data: np.ndarray[tuple[int, ...], np.dtype[np.uint8]]
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/MeshFile"

@dataclass
class visualization_msgs__msg__UVCoordinate:
    """Class for visualization_msgs/msg/UVCoordinate."""

    u: float
    v: float
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/UVCoordinate"

@dataclass
class visualization_msgs__msg__Marker:
    """Class for visualization_msgs/msg/Marker."""

    header: std_msgs__msg__Header
    ns: str
    id: int
    type: int
    action: int
    pose: geometry_msgs__msg__Pose
    scale: geometry_msgs__msg__Vector3
    color: std_msgs__msg__ColorRGBA
    lifetime: builtin_interfaces__msg__Duration
    frame_locked: bool
    points: list[geometry_msgs__msg__Point]
    colors: list[std_msgs__msg__ColorRGBA]
    texture_resource: str
    texture: sensor_msgs__msg__CompressedImage
    uv_coordinates: list[visualization_msgs__msg__UVCoordinate]
    text: str
    mesh_resource: str
    mesh_file: visualization_msgs__msg__MeshFile
    mesh_use_embedded_materials: bool
    ARROW: ClassVar[int] = 0
    CUBE: ClassVar[int] = 1
    SPHERE: ClassVar[int] = 2
    CYLINDER: ClassVar[int] = 3
    LINE_STRIP: ClassVar[int] = 4
    LINE_LIST: ClassVar[int] = 5
    CUBE_LIST: ClassVar[int] = 6
    SPHERE_LIST: ClassVar[int] = 7
    POINTS: ClassVar[int] = 8
    TEXT_VIEW_FACING: ClassVar[int] = 9
    MESH_RESOURCE: ClassVar[int] = 10
    TRIANGLE_LIST: ClassVar[int] = 11
    ARROW_STRIP: ClassVar[int] = 12
    ADD: ClassVar[int] = 0
    MODIFY: ClassVar[int] = 0
    DELETE: ClassVar[int] = 2
    DELETEALL: ClassVar[int] = 3
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/Marker"

@dataclass
class visualization_msgs__msg__InteractiveMarkerControl:
    """Class for visualization_msgs/msg/InteractiveMarkerControl."""

    name: str
    orientation: geometry_msgs__msg__Quaternion
    orientation_mode: int
    interaction_mode: int
    always_visible: bool
    markers: list[visualization_msgs__msg__Marker]
    independent_marker_orientation: bool
    description: str
    INHERIT: ClassVar[int] = 0
    FIXED: ClassVar[int] = 1
    VIEW_FACING: ClassVar[int] = 2
    NONE: ClassVar[int] = 0
    MENU: ClassVar[int] = 1
    BUTTON: ClassVar[int] = 2
    MOVE_AXIS: ClassVar[int] = 3
    MOVE_PLANE: ClassVar[int] = 4
    ROTATE_AXIS: ClassVar[int] = 5
    MOVE_ROTATE: ClassVar[int] = 6
    MOVE_3D: ClassVar[int] = 7
    ROTATE_3D: ClassVar[int] = 8
    MOVE_ROTATE_3D: ClassVar[int] = 9
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarkerControl"

@dataclass
class visualization_msgs__msg__MenuEntry:
    """Class for visualization_msgs/msg/MenuEntry."""

    id: int
    parent_id: int
    title: str
    command: str
    command_type: int
    FEEDBACK: ClassVar[int] = 0
    ROSRUN: ClassVar[int] = 1
    ROSLAUNCH: ClassVar[int] = 2
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/MenuEntry"

@dataclass
class visualization_msgs__msg__InteractiveMarker:
    """Class for visualization_msgs/msg/InteractiveMarker."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__Pose
    name: str
    description: str
    scale: float
    menu_entries: list[visualization_msgs__msg__MenuEntry]
    controls: list[visualization_msgs__msg__InteractiveMarkerControl]
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarker"

@dataclass
class visualization_msgs__msg__InteractiveMarkerFeedback:
    """Class for visualization_msgs/msg/InteractiveMarkerFeedback."""

    header: std_msgs__msg__Header
    client_id: str
    marker_name: str
    control_name: str
    event_type: int
    pose: geometry_msgs__msg__Pose
    menu_entry_id: int
    mouse_point: geometry_msgs__msg__Point
    mouse_point_valid: bool
    KEEP_ALIVE: ClassVar[int] = 0
    POSE_UPDATE: ClassVar[int] = 1
    MENU_SELECT: ClassVar[int] = 2
    BUTTON_CLICK: ClassVar[int] = 3
    MOUSE_DOWN: ClassVar[int] = 4
    MOUSE_UP: ClassVar[int] = 5
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarkerFeedback"

@dataclass
class visualization_msgs__msg__InteractiveMarkerInit:
    """Class for visualization_msgs/msg/InteractiveMarkerInit."""

    server_id: str
    seq_num: int
    markers: list[visualization_msgs__msg__InteractiveMarker]
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarkerInit"

@dataclass
class visualization_msgs__msg__InteractiveMarkerPose:
    """Class for visualization_msgs/msg/InteractiveMarkerPose."""

    header: std_msgs__msg__Header
    pose: geometry_msgs__msg__Pose
    name: str
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarkerPose"

@dataclass
class visualization_msgs__msg__InteractiveMarkerUpdate:
    """Class for visualization_msgs/msg/InteractiveMarkerUpdate."""

    server_id: str
    seq_num: int
    type: int
    markers: list[visualization_msgs__msg__InteractiveMarker]
    poses: list[visualization_msgs__msg__InteractiveMarkerPose]
    erases: list[str]
    KEEP_ALIVE: ClassVar[int] = 0
    UPDATE: ClassVar[int] = 1
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/InteractiveMarkerUpdate"

@dataclass
class visualization_msgs__msg__MarkerArray:
    """Class for visualization_msgs/msg/MarkerArray."""

    markers: list[visualization_msgs__msg__Marker]
    __msgtype__: ClassVar[str] = "visualization_msgs/msg/MarkerArray"

FIELDDEFS: Typesdict = {
    "builtin_interfaces/msg/Duration": (
        [],
        [
            ("sec", (T.BASE, ("int32", 0))),
            ("nanosec", (T.BASE, ("uint32", 0))),
        ],
    ),
    "builtin_interfaces/msg/Time": (
        [],
        [
            ("sec", (T.BASE, ("int32", 0))),
            ("nanosec", (T.BASE, ("uint32", 0))),
        ],
    ),
    "std_msgs/msg/Header": (
        [],
        [
            ("stamp", (T.NAME, "builtin_interfaces/msg/Time")),
            ("frame_id", (T.BASE, ("string", 0))),
        ],
    ),
    "vision_msgs/msg/Point2D": (
        [],
        [
            ("x", (T.BASE, ("float64", 0))),
            ("y", (T.BASE, ("float64", 0))),
        ],
    ),
    "vision_msgs/msg/Pose2D": (
        [],
        [
            ("position", (T.NAME, "vision_msgs/msg/Point2D")),
            ("theta", (T.BASE, ("float64", 0))),
        ],
    ),
    "vision_msgs/msg/BoundingBox2D": (
        [],
        [
            ("center", (T.NAME, "vision_msgs/msg/Pose2D")),
            ("size_x", (T.BASE, ("float64", 0))),
            ("size_y", (T.BASE, ("float64", 0))),
        ],
    ),
    "dimos_msgs/msg/BoundingBox2DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("boxes", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/BoundingBox2D"), 0))),
        ],
    ),
    "geometry_msgs/msg/Point": (
        [],
        [
            ("x", (T.BASE, ("float64", 0))),
            ("y", (T.BASE, ("float64", 0))),
            ("z", (T.BASE, ("float64", 0))),
        ],
    ),
    "geometry_msgs/msg/Quaternion": (
        [],
        [
            ("x", (T.BASE, ("float64", 0))),
            ("y", (T.BASE, ("float64", 0))),
            ("z", (T.BASE, ("float64", 0))),
            ("w", (T.BASE, ("float64", 0))),
        ],
    ),
    "geometry_msgs/msg/Pose": (
        [],
        [
            ("position", (T.NAME, "geometry_msgs/msg/Point")),
            ("orientation", (T.NAME, "geometry_msgs/msg/Quaternion")),
        ],
    ),
    "geometry_msgs/msg/Vector3": (
        [],
        [
            ("x", (T.BASE, ("float64", 0))),
            ("y", (T.BASE, ("float64", 0))),
            ("z", (T.BASE, ("float64", 0))),
        ],
    ),
    "vision_msgs/msg/BoundingBox3D": (
        [],
        [
            ("center", (T.NAME, "geometry_msgs/msg/Pose")),
            ("size", (T.NAME, "geometry_msgs/msg/Vector3")),
        ],
    ),
    "dimos_msgs/msg/BoundingBox3DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("boxes", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/BoundingBox3D"), 0))),
        ],
    ),
    "dimos_msgs/msg/EntityMarker": (
        [],
        [
            ("entity_id", (T.BASE, ("string", 0))),
            ("label", (T.BASE, ("string", 0))),
            ("entity_type", (T.BASE, ("string", 0))),
            ("position", (T.NAME, "geometry_msgs/msg/Point")),
        ],
    ),
    "dimos_msgs/msg/EntityMarkers": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("markers", (T.SEQUENCE, ((T.NAME, "dimos_msgs/msg/EntityMarker"), 0))),
        ],
    ),
    "dimos_msgs/msg/GraspCandidate": (
        [],
        [
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("score", (T.BASE, ("float64", 0))),
        ],
    ),
    "dimos_msgs/msg/GraspCandidateArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("candidates", (T.SEQUENCE, ((T.NAME, "dimos_msgs/msg/GraspCandidate"), 0))),
        ],
    ),
    "dimos_msgs/msg/ImuInfo": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("gyro_noise_density", (T.BASE, ("float64", 0))),
            ("gyro_random_walk", (T.BASE, ("float64", 0))),
            ("accel_noise_density", (T.BASE, ("float64", 0))),
            ("accel_random_walk", (T.BASE, ("float64", 0))),
            ("frequency", (T.BASE, ("float64", 0))),
        ],
    ),
    "dimos_msgs/msg/JointCommand": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("positions", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
        ],
    ),
    "dimos_msgs/msg/LineSegment3D": (
        [],
        [
            ("start", (T.NAME, "geometry_msgs/msg/Point")),
            ("end", (T.NAME, "geometry_msgs/msg/Point")),
            ("weight", (T.BASE, ("float64", 0))),
        ],
    ),
    "dimos_msgs/msg/LineSegments3D": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("segments", (T.SEQUENCE, ((T.NAME, "dimos_msgs/msg/LineSegment3D"), 0))),
        ],
    ),
    "dimos_msgs/msg/MotorCommandArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("q", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("dq", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("kp", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("kd", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("tau", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
        ],
    ),
    "dimos_msgs/msg/RobotState": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("state", (T.BASE, ("int32", 0))),
            ("mode", (T.BASE, ("int32", 0))),
            ("error_code", (T.BASE, ("int32", 0))),
            ("warn_code", (T.BASE, ("int32", 0))),
            ("cmdnum", (T.BASE, ("int32", 0))),
            ("mt_brake", (T.BASE, ("int32", 0))),
            ("mt_able", (T.BASE, ("int32", 0))),
            ("tcp_pose", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("tcp_offset", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("joints", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
        ],
    ),
    "dimos_msgs/msg/TrajectoryStatus": (
        [
            ("IDLE", "uint8", 0),
            ("EXECUTING", "uint8", 1),
            ("COMPLETED", "uint8", 2),
            ("ABORTED", "uint8", 3),
            ("FAULT", "uint8", 4),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("state", (T.BASE, ("uint8", 0))),
            ("progress", (T.BASE, ("float64", 0))),
            ("time_elapsed", (T.NAME, "builtin_interfaces/msg/Duration")),
            ("time_remaining", (T.NAME, "builtin_interfaces/msg/Duration")),
            ("error", (T.BASE, ("string", 0))),
        ],
    ),
    "dimos_msgs/msg/VideoStats": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("fps", (T.BASE, ("float64", 0))),
            ("kbps", (T.BASE, ("float64", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("height", (T.BASE, ("uint32", 0))),
            ("loss_pct", (T.BASE, ("float64", 0))),
            ("jitter_buffer_ms", (T.BASE, ("float64", 0))),
            ("decode_ms", (T.BASE, ("float64", 0))),
            ("frames_dropped", (T.BASE, ("uint64", 0))),
            ("freezes", (T.BASE, ("uint64", 0))),
            ("e2e_latency_ms", (T.BASE, ("float64", 0))),
        ],
    ),
    "foxglove_msgs/msg/CompressedVideo": (
        [],
        [
            ("timestamp", (T.NAME, "builtin_interfaces/msg/Time")),
            ("frame_id", (T.BASE, ("string", 0))),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
            ("format", (T.BASE, ("string", 0))),
        ],
    ),
    "geometry_msgs/msg/Accel": (
        [],
        [
            ("linear", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("angular", (T.NAME, "geometry_msgs/msg/Vector3")),
        ],
    ),
    "geometry_msgs/msg/AccelStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("accel", (T.NAME, "geometry_msgs/msg/Accel")),
        ],
    ),
    "geometry_msgs/msg/AccelWithCovariance": (
        [],
        [
            ("accel", (T.NAME, "geometry_msgs/msg/Accel")),
            ("covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 36))),
        ],
    ),
    "geometry_msgs/msg/AccelWithCovarianceStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("accel", (T.NAME, "geometry_msgs/msg/AccelWithCovariance")),
        ],
    ),
    "geometry_msgs/msg/Inertia": (
        [],
        [
            ("m", (T.BASE, ("float64", 0))),
            ("com", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("ixx", (T.BASE, ("float64", 0))),
            ("ixy", (T.BASE, ("float64", 0))),
            ("ixz", (T.BASE, ("float64", 0))),
            ("iyy", (T.BASE, ("float64", 0))),
            ("iyz", (T.BASE, ("float64", 0))),
            ("izz", (T.BASE, ("float64", 0))),
        ],
    ),
    "geometry_msgs/msg/InertiaStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("inertia", (T.NAME, "geometry_msgs/msg/Inertia")),
        ],
    ),
    "geometry_msgs/msg/Point32": (
        [],
        [
            ("x", (T.BASE, ("float32", 0))),
            ("y", (T.BASE, ("float32", 0))),
            ("z", (T.BASE, ("float32", 0))),
        ],
    ),
    "geometry_msgs/msg/PointStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("point", (T.NAME, "geometry_msgs/msg/Point")),
        ],
    ),
    "geometry_msgs/msg/Polygon": (
        [],
        [
            ("points", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point32"), 0))),
        ],
    ),
    "geometry_msgs/msg/PolygonInstance": (
        [],
        [
            ("polygon", (T.NAME, "geometry_msgs/msg/Polygon")),
            ("id", (T.BASE, ("int64", 0))),
        ],
    ),
    "geometry_msgs/msg/PolygonInstanceStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("polygon", (T.NAME, "geometry_msgs/msg/PolygonInstance")),
        ],
    ),
    "geometry_msgs/msg/PolygonStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("polygon", (T.NAME, "geometry_msgs/msg/Polygon")),
        ],
    ),
    "geometry_msgs/msg/Pose2D": (
        [],
        [
            ("x", (T.BASE, ("float64", 0))),
            ("y", (T.BASE, ("float64", 0))),
            ("theta", (T.BASE, ("float64", 0))),
        ],
    ),
    "geometry_msgs/msg/PoseArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("poses", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Pose"), 0))),
        ],
    ),
    "geometry_msgs/msg/PoseStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
        ],
    ),
    "geometry_msgs/msg/PoseWithCovariance": (
        [],
        [
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 36))),
        ],
    ),
    "geometry_msgs/msg/PoseWithCovarianceStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("pose", (T.NAME, "geometry_msgs/msg/PoseWithCovariance")),
        ],
    ),
    "geometry_msgs/msg/QuaternionStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("quaternion", (T.NAME, "geometry_msgs/msg/Quaternion")),
        ],
    ),
    "geometry_msgs/msg/Transform": (
        [],
        [
            ("translation", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("rotation", (T.NAME, "geometry_msgs/msg/Quaternion")),
        ],
    ),
    "geometry_msgs/msg/TransformStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("child_frame_id", (T.BASE, ("string", 0))),
            ("transform", (T.NAME, "geometry_msgs/msg/Transform")),
        ],
    ),
    "geometry_msgs/msg/Twist": (
        [],
        [
            ("linear", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("angular", (T.NAME, "geometry_msgs/msg/Vector3")),
        ],
    ),
    "geometry_msgs/msg/TwistStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("twist", (T.NAME, "geometry_msgs/msg/Twist")),
        ],
    ),
    "geometry_msgs/msg/TwistWithCovariance": (
        [],
        [
            ("twist", (T.NAME, "geometry_msgs/msg/Twist")),
            ("covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 36))),
        ],
    ),
    "geometry_msgs/msg/TwistWithCovarianceStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("twist", (T.NAME, "geometry_msgs/msg/TwistWithCovariance")),
        ],
    ),
    "geometry_msgs/msg/Vector3Stamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("vector", (T.NAME, "geometry_msgs/msg/Vector3")),
        ],
    ),
    "geometry_msgs/msg/VelocityStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("body_frame_id", (T.BASE, ("string", 0))),
            ("reference_frame_id", (T.BASE, ("string", 0))),
            ("velocity", (T.NAME, "geometry_msgs/msg/Twist")),
        ],
    ),
    "geometry_msgs/msg/VelocityWithCovarianceStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("body_frame_id", (T.BASE, ("string", 0))),
            ("reference_frame_id", (T.BASE, ("string", 0))),
            ("velocity", (T.NAME, "geometry_msgs/msg/TwistWithCovariance")),
        ],
    ),
    "geometry_msgs/msg/Wrench": (
        [],
        [
            ("force", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("torque", (T.NAME, "geometry_msgs/msg/Vector3")),
        ],
    ),
    "geometry_msgs/msg/WrenchStamped": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("wrench", (T.NAME, "geometry_msgs/msg/Wrench")),
        ],
    ),
    "nav_msgs/msg/Goals": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("goals", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/PoseStamped"), 0))),
        ],
    ),
    "nav_msgs/msg/GridCells": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("cell_width", (T.BASE, ("float32", 0))),
            ("cell_height", (T.BASE, ("float32", 0))),
            ("cells", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point"), 0))),
        ],
    ),
    "nav_msgs/msg/MapMetaData": (
        [],
        [
            ("map_load_time", (T.NAME, "builtin_interfaces/msg/Time")),
            ("resolution", (T.BASE, ("float32", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("height", (T.BASE, ("uint32", 0))),
            ("origin", (T.NAME, "geometry_msgs/msg/Pose")),
        ],
    ),
    "nav_msgs/msg/OccupancyGrid": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("info", (T.NAME, "nav_msgs/msg/MapMetaData")),
            ("data", (T.SEQUENCE, ((T.BASE, ("int8", 0)), 0))),
        ],
    ),
    "nav_msgs/msg/Odometry": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("child_frame_id", (T.BASE, ("string", 0))),
            ("pose", (T.NAME, "geometry_msgs/msg/PoseWithCovariance")),
            ("twist", (T.NAME, "geometry_msgs/msg/TwistWithCovariance")),
        ],
    ),
    "nav_msgs/msg/Path": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("poses", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/PoseStamped"), 0))),
        ],
    ),
    "nav_msgs/msg/TrajectoryPoint": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("velocity", (T.NAME, "geometry_msgs/msg/Twist")),
            ("acceleration", (T.NAME, "geometry_msgs/msg/Accel")),
            ("effort", (T.NAME, "geometry_msgs/msg/Wrench")),
        ],
    ),
    "nav_msgs/msg/Trajectory": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("points", (T.SEQUENCE, ((T.NAME, "nav_msgs/msg/TrajectoryPoint"), 0))),
        ],
    ),
    "sensor_msgs/msg/BatteryState": (
        [
            ("POWER_SUPPLY_STATUS_UNKNOWN", "uint8", 0),
            ("POWER_SUPPLY_STATUS_CHARGING", "uint8", 1),
            ("POWER_SUPPLY_STATUS_DISCHARGING", "uint8", 2),
            ("POWER_SUPPLY_STATUS_NOT_CHARGING", "uint8", 3),
            ("POWER_SUPPLY_STATUS_FULL", "uint8", 4),
            ("POWER_SUPPLY_HEALTH_UNKNOWN", "uint8", 0),
            ("POWER_SUPPLY_HEALTH_GOOD", "uint8", 1),
            ("POWER_SUPPLY_HEALTH_OVERHEAT", "uint8", 2),
            ("POWER_SUPPLY_HEALTH_DEAD", "uint8", 3),
            ("POWER_SUPPLY_HEALTH_OVERVOLTAGE", "uint8", 4),
            ("POWER_SUPPLY_HEALTH_UNSPEC_FAILURE", "uint8", 5),
            ("POWER_SUPPLY_HEALTH_COLD", "uint8", 6),
            ("POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE", "uint8", 7),
            ("POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE", "uint8", 8),
            ("POWER_SUPPLY_TECHNOLOGY_UNKNOWN", "uint8", 0),
            ("POWER_SUPPLY_TECHNOLOGY_NIMH", "uint8", 1),
            ("POWER_SUPPLY_TECHNOLOGY_LION", "uint8", 2),
            ("POWER_SUPPLY_TECHNOLOGY_LIPO", "uint8", 3),
            ("POWER_SUPPLY_TECHNOLOGY_LIFE", "uint8", 4),
            ("POWER_SUPPLY_TECHNOLOGY_NICD", "uint8", 5),
            ("POWER_SUPPLY_TECHNOLOGY_LIMN", "uint8", 6),
            ("POWER_SUPPLY_TECHNOLOGY_TERNARY", "uint8", 7),
            ("POWER_SUPPLY_TECHNOLOGY_VRLA", "uint8", 8),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("voltage", (T.BASE, ("float32", 0))),
            ("temperature", (T.BASE, ("float32", 0))),
            ("current", (T.BASE, ("float32", 0))),
            ("charge", (T.BASE, ("float32", 0))),
            ("capacity", (T.BASE, ("float32", 0))),
            ("design_capacity", (T.BASE, ("float32", 0))),
            ("percentage", (T.BASE, ("float32", 0))),
            ("power_supply_status", (T.BASE, ("uint8", 0))),
            ("power_supply_health", (T.BASE, ("uint8", 0))),
            ("power_supply_technology", (T.BASE, ("uint8", 0))),
            ("present", (T.BASE, ("bool", 0))),
            ("cell_voltage", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
            ("cell_temperature", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
            ("location", (T.BASE, ("string", 0))),
            ("serial_number", (T.BASE, ("string", 0))),
        ],
    ),
    "sensor_msgs/msg/RegionOfInterest": (
        [],
        [
            ("x_offset", (T.BASE, ("uint32", 0))),
            ("y_offset", (T.BASE, ("uint32", 0))),
            ("height", (T.BASE, ("uint32", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("do_rectify", (T.BASE, ("bool", 0))),
        ],
    ),
    "sensor_msgs/msg/CameraInfo": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("height", (T.BASE, ("uint32", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("distortion_model", (T.BASE, ("string", 0))),
            ("d", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("k", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
            ("r", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
            ("p", (T.ARRAY, ((T.BASE, ("float64", 0)), 12))),
            ("binning_x", (T.BASE, ("uint32", 0))),
            ("binning_y", (T.BASE, ("uint32", 0))),
            ("roi", (T.NAME, "sensor_msgs/msg/RegionOfInterest")),
        ],
    ),
    "sensor_msgs/msg/ChannelFloat32": (
        [],
        [
            ("name", (T.BASE, ("string", 0))),
            ("values", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/CompressedImage": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("format", (T.BASE, ("string", 0))),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/FluidPressure": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("fluid_pressure", (T.BASE, ("float64", 0))),
            ("variance", (T.BASE, ("float64", 0))),
        ],
    ),
    "sensor_msgs/msg/Illuminance": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("illuminance", (T.BASE, ("float64", 0))),
            ("variance", (T.BASE, ("float64", 0))),
        ],
    ),
    "sensor_msgs/msg/Image": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("height", (T.BASE, ("uint32", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("encoding", (T.BASE, ("string", 0))),
            ("is_bigendian", (T.BASE, ("uint8", 0))),
            ("step", (T.BASE, ("uint32", 0))),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/Imu": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("orientation", (T.NAME, "geometry_msgs/msg/Quaternion")),
            ("orientation_covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
            ("angular_velocity", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("angular_velocity_covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
            ("linear_acceleration", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("linear_acceleration_covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
        ],
    ),
    "sensor_msgs/msg/JointState": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("name", (T.SEQUENCE, ((T.BASE, ("string", 0)), 0))),
            ("position", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("velocity", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("effort", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/Joy": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("axes", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
            ("buttons", (T.SEQUENCE, ((T.BASE, ("int32", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/JoyFeedback": (
        [
            ("TYPE_LED", "uint8", 0),
            ("TYPE_RUMBLE", "uint8", 1),
            ("TYPE_BUZZER", "uint8", 2),
        ],
        [
            ("type", (T.BASE, ("uint8", 0))),
            ("id", (T.BASE, ("uint8", 0))),
            ("intensity", (T.BASE, ("float32", 0))),
        ],
    ),
    "sensor_msgs/msg/JoyFeedbackArray": (
        [],
        [
            ("array", (T.SEQUENCE, ((T.NAME, "sensor_msgs/msg/JoyFeedback"), 0))),
        ],
    ),
    "sensor_msgs/msg/LaserEcho": (
        [],
        [
            ("echoes", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/LaserScan": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("angle_min", (T.BASE, ("float32", 0))),
            ("angle_max", (T.BASE, ("float32", 0))),
            ("angle_increment", (T.BASE, ("float32", 0))),
            ("time_increment", (T.BASE, ("float32", 0))),
            ("scan_time", (T.BASE, ("float32", 0))),
            ("range_min", (T.BASE, ("float32", 0))),
            ("range_max", (T.BASE, ("float32", 0))),
            ("ranges", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
            ("intensities", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
        ],
    ),
    "sensor_msgs/msg/MagneticField": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("magnetic_field", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("magnetic_field_covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
        ],
    ),
    "sensor_msgs/msg/MultiDOFJointState": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("joint_names", (T.SEQUENCE, ((T.BASE, ("string", 0)), 0))),
            ("transforms", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Transform"), 0))),
            ("twist", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Twist"), 0))),
            ("wrench", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Wrench"), 0))),
        ],
    ),
    "sensor_msgs/msg/MultiEchoLaserScan": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("angle_min", (T.BASE, ("float32", 0))),
            ("angle_max", (T.BASE, ("float32", 0))),
            ("angle_increment", (T.BASE, ("float32", 0))),
            ("time_increment", (T.BASE, ("float32", 0))),
            ("scan_time", (T.BASE, ("float32", 0))),
            ("range_min", (T.BASE, ("float32", 0))),
            ("range_max", (T.BASE, ("float32", 0))),
            ("ranges", (T.SEQUENCE, ((T.NAME, "sensor_msgs/msg/LaserEcho"), 0))),
            ("intensities", (T.SEQUENCE, ((T.NAME, "sensor_msgs/msg/LaserEcho"), 0))),
        ],
    ),
    "sensor_msgs/msg/NavSatStatus": (
        [
            ("STATUS_UNKNOWN", "int8", -2),
            ("STATUS_NO_FIX", "int8", -1),
            ("STATUS_FIX", "int8", 0),
            ("STATUS_SBAS_FIX", "int8", 1),
            ("STATUS_GBAS_FIX", "int8", 2),
            ("SERVICE_UNKNOWN", "uint16", 0),
            ("SERVICE_GPS", "uint16", 1),
            ("SERVICE_GLONASS", "uint16", 2),
            ("SERVICE_COMPASS", "uint16", 4),
            ("SERVICE_GALILEO", "uint16", 8),
        ],
        [
            ("status", (T.BASE, ("int8", 0))),
            ("service", (T.BASE, ("uint16", 0))),
        ],
    ),
    "sensor_msgs/msg/NavSatFix": (
        [
            ("COVARIANCE_TYPE_UNKNOWN", "uint8", 0),
            ("COVARIANCE_TYPE_APPROXIMATED", "uint8", 1),
            ("COVARIANCE_TYPE_DIAGONAL_KNOWN", "uint8", 2),
            ("COVARIANCE_TYPE_KNOWN", "uint8", 3),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("status", (T.NAME, "sensor_msgs/msg/NavSatStatus")),
            ("latitude", (T.BASE, ("float64", 0))),
            ("longitude", (T.BASE, ("float64", 0))),
            ("altitude", (T.BASE, ("float64", 0))),
            ("position_covariance", (T.ARRAY, ((T.BASE, ("float64", 0)), 9))),
            ("position_covariance_type", (T.BASE, ("uint8", 0))),
        ],
    ),
    "sensor_msgs/msg/PointCloud": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("points", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point32"), 0))),
            ("channels", (T.SEQUENCE, ((T.NAME, "sensor_msgs/msg/ChannelFloat32"), 0))),
        ],
    ),
    "sensor_msgs/msg/PointField": (
        [
            ("INT8", "uint8", 1),
            ("UINT8", "uint8", 2),
            ("INT16", "uint8", 3),
            ("UINT16", "uint8", 4),
            ("INT32", "uint8", 5),
            ("UINT32", "uint8", 6),
            ("FLOAT32", "uint8", 7),
            ("FLOAT64", "uint8", 8),
        ],
        [
            ("name", (T.BASE, ("string", 0))),
            ("offset", (T.BASE, ("uint32", 0))),
            ("datatype", (T.BASE, ("uint8", 0))),
            ("count", (T.BASE, ("uint32", 0))),
        ],
    ),
    "sensor_msgs/msg/PointCloud2": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("height", (T.BASE, ("uint32", 0))),
            ("width", (T.BASE, ("uint32", 0))),
            ("fields", (T.SEQUENCE, ((T.NAME, "sensor_msgs/msg/PointField"), 0))),
            ("is_bigendian", (T.BASE, ("bool", 0))),
            ("point_step", (T.BASE, ("uint32", 0))),
            ("row_step", (T.BASE, ("uint32", 0))),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
            ("is_dense", (T.BASE, ("bool", 0))),
        ],
    ),
    "sensor_msgs/msg/Range": (
        [
            ("ULTRASOUND", "uint8", 0),
            ("INFRARED", "uint8", 1),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("radiation_type", (T.BASE, ("uint8", 0))),
            ("field_of_view", (T.BASE, ("float32", 0))),
            ("min_range", (T.BASE, ("float32", 0))),
            ("max_range", (T.BASE, ("float32", 0))),
            ("range", (T.BASE, ("float32", 0))),
            ("variance", (T.BASE, ("float32", 0))),
        ],
    ),
    "sensor_msgs/msg/RelativeHumidity": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("relative_humidity", (T.BASE, ("float64", 0))),
            ("variance", (T.BASE, ("float64", 0))),
        ],
    ),
    "sensor_msgs/msg/Temperature": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("temperature", (T.BASE, ("float64", 0))),
            ("variance", (T.BASE, ("float64", 0))),
        ],
    ),
    "sensor_msgs/msg/TimeReference": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("time_ref", (T.NAME, "builtin_interfaces/msg/Time")),
            ("source", (T.BASE, ("string", 0))),
        ],
    ),
    "shape_msgs/msg/MeshTriangle": (
        [],
        [
            ("vertex_indices", (T.ARRAY, ((T.BASE, ("uint32", 0)), 3))),
        ],
    ),
    "shape_msgs/msg/Mesh": (
        [],
        [
            ("triangles", (T.SEQUENCE, ((T.NAME, "shape_msgs/msg/MeshTriangle"), 0))),
            ("vertices", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point"), 0))),
        ],
    ),
    "shape_msgs/msg/Plane": (
        [],
        [
            ("coef", (T.ARRAY, ((T.BASE, ("float64", 0)), 4))),
        ],
    ),
    "shape_msgs/msg/SolidPrimitive": (
        [
            ("BOX", "uint8", 1),
            ("SPHERE", "uint8", 2),
            ("CYLINDER", "uint8", 3),
            ("CONE", "uint8", 4),
            ("PRISM", "uint8", 5),
            ("BOX_X", "uint8", 0),
            ("BOX_Y", "uint8", 1),
            ("BOX_Z", "uint8", 2),
            ("SPHERE_RADIUS", "uint8", 0),
            ("CYLINDER_HEIGHT", "uint8", 0),
            ("CYLINDER_RADIUS", "uint8", 1),
            ("CONE_HEIGHT", "uint8", 0),
            ("CONE_RADIUS", "uint8", 1),
            ("PRISM_HEIGHT", "uint8", 0),
        ],
        [
            ("type", (T.BASE, ("uint8", 0))),
            ("dimensions", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 3))),
            ("polygon", (T.NAME, "geometry_msgs/msg/Polygon")),
        ],
    ),
    "std_msgs/msg/Bool": (
        [],
        [
            ("data", (T.BASE, ("bool", 0))),
        ],
    ),
    "std_msgs/msg/Byte": (
        [],
        [
            ("data", (T.BASE, ("byte", 0))),
        ],
    ),
    "std_msgs/msg/MultiArrayDimension": (
        [],
        [
            ("label", (T.BASE, ("string", 0))),
            ("size", (T.BASE, ("uint32", 0))),
            ("stride", (T.BASE, ("uint32", 0))),
        ],
    ),
    "std_msgs/msg/MultiArrayLayout": (
        [],
        [
            ("dim", (T.SEQUENCE, ((T.NAME, "std_msgs/msg/MultiArrayDimension"), 0))),
            ("data_offset", (T.BASE, ("uint32", 0))),
        ],
    ),
    "std_msgs/msg/ByteMultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("byte", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Char": (
        [],
        [
            ("data", (T.BASE, ("char", 0))),
        ],
    ),
    "std_msgs/msg/ColorRGBA": (
        [],
        [
            ("r", (T.BASE, ("float32", 0))),
            ("g", (T.BASE, ("float32", 0))),
            ("b", (T.BASE, ("float32", 0))),
            ("a", (T.BASE, ("float32", 0))),
        ],
    ),
    "std_msgs/msg/Empty": (
        [],
        [
            ("structure_needs_at_least_one_member", (T.BASE, ("uint8", 0))),
        ],
    ),
    "std_msgs/msg/Float32": (
        [],
        [
            ("data", (T.BASE, ("float32", 0))),
        ],
    ),
    "std_msgs/msg/Float32MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("float32", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Float64": (
        [],
        [
            ("data", (T.BASE, ("float64", 0))),
        ],
    ),
    "std_msgs/msg/Float64MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Int16": (
        [],
        [
            ("data", (T.BASE, ("int16", 0))),
        ],
    ),
    "std_msgs/msg/Int16MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("int16", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Int32": (
        [],
        [
            ("data", (T.BASE, ("int32", 0))),
        ],
    ),
    "std_msgs/msg/Int32MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("int32", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Int64": (
        [],
        [
            ("data", (T.BASE, ("int64", 0))),
        ],
    ),
    "std_msgs/msg/Int64MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("int64", 0)), 0))),
        ],
    ),
    "std_msgs/msg/Int8": (
        [],
        [
            ("data", (T.BASE, ("int8", 0))),
        ],
    ),
    "std_msgs/msg/Int8MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("int8", 0)), 0))),
        ],
    ),
    "std_msgs/msg/String": (
        [],
        [
            ("data", (T.BASE, ("string", 0))),
        ],
    ),
    "std_msgs/msg/UInt16": (
        [],
        [
            ("data", (T.BASE, ("uint16", 0))),
        ],
    ),
    "std_msgs/msg/UInt16MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint16", 0)), 0))),
        ],
    ),
    "std_msgs/msg/UInt32": (
        [],
        [
            ("data", (T.BASE, ("uint32", 0))),
        ],
    ),
    "std_msgs/msg/UInt32MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint32", 0)), 0))),
        ],
    ),
    "std_msgs/msg/UInt64": (
        [],
        [
            ("data", (T.BASE, ("uint64", 0))),
        ],
    ),
    "std_msgs/msg/UInt64MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint64", 0)), 0))),
        ],
    ),
    "std_msgs/msg/UInt8": (
        [],
        [
            ("data", (T.BASE, ("uint8", 0))),
        ],
    ),
    "std_msgs/msg/UInt8MultiArray": (
        [],
        [
            ("layout", (T.NAME, "std_msgs/msg/MultiArrayLayout")),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
        ],
    ),
    "tf2_msgs/msg/TF2Error": (
        [
            ("NO_ERROR", "uint8", 0),
            ("LOOKUP_ERROR", "uint8", 1),
            ("CONNECTIVITY_ERROR", "uint8", 2),
            ("EXTRAPOLATION_ERROR", "uint8", 3),
            ("INVALID_ARGUMENT_ERROR", "uint8", 4),
            ("TIMEOUT_ERROR", "uint8", 5),
            ("TRANSFORM_ERROR", "uint8", 6),
        ],
        [
            ("error", (T.BASE, ("uint8", 0))),
            ("error_string", (T.BASE, ("string", 0))),
        ],
    ),
    "tf2_msgs/msg/TFMessage": (
        [],
        [
            ("transforms", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/TransformStamped"), 0))),
        ],
    ),
    "trajectory_msgs/msg/JointTrajectoryPoint": (
        [],
        [
            ("positions", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("velocities", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("accelerations", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("effort", (T.SEQUENCE, ((T.BASE, ("float64", 0)), 0))),
            ("time_from_start", (T.NAME, "builtin_interfaces/msg/Duration")),
        ],
    ),
    "trajectory_msgs/msg/JointTrajectory": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("joint_names", (T.SEQUENCE, ((T.BASE, ("string", 0)), 0))),
            ("points", (T.SEQUENCE, ((T.NAME, "trajectory_msgs/msg/JointTrajectoryPoint"), 0))),
        ],
    ),
    "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint": (
        [],
        [
            ("transforms", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Transform"), 0))),
            ("velocities", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Twist"), 0))),
            ("accelerations", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Twist"), 0))),
            ("time_from_start", (T.NAME, "builtin_interfaces/msg/Duration")),
        ],
    ),
    "trajectory_msgs/msg/MultiDOFJointTrajectory": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("joint_names", (T.SEQUENCE, ((T.BASE, ("string", 0)), 0))),
            (
                "points",
                (T.SEQUENCE, ((T.NAME, "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint"), 0)),
            ),
        ],
    ),
    "vision_msgs/msg/BoundingBox2DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("boxes", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/BoundingBox2D"), 0))),
        ],
    ),
    "vision_msgs/msg/BoundingBox3DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("boxes", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/BoundingBox3D"), 0))),
        ],
    ),
    "vision_msgs/msg/ObjectHypothesis": (
        [],
        [
            ("class_id", (T.BASE, ("string", 0))),
            ("score", (T.BASE, ("float64", 0))),
        ],
    ),
    "vision_msgs/msg/Classification": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("results", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/ObjectHypothesis"), 0))),
        ],
    ),
    "vision_msgs/msg/ObjectHypothesisWithPose": (
        [],
        [
            ("hypothesis", (T.NAME, "vision_msgs/msg/ObjectHypothesis")),
            ("pose", (T.NAME, "geometry_msgs/msg/PoseWithCovariance")),
        ],
    ),
    "vision_msgs/msg/Detection2D": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("results", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/ObjectHypothesisWithPose"), 0))),
            ("bbox", (T.NAME, "vision_msgs/msg/BoundingBox2D")),
            ("id", (T.BASE, ("string", 0))),
        ],
    ),
    "vision_msgs/msg/Detection2DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("detections", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/Detection2D"), 0))),
        ],
    ),
    "vision_msgs/msg/Detection3D": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("results", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/ObjectHypothesisWithPose"), 0))),
            ("bbox", (T.NAME, "vision_msgs/msg/BoundingBox3D")),
            ("id", (T.BASE, ("string", 0))),
        ],
    ),
    "vision_msgs/msg/Detection3DArray": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("detections", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/Detection3D"), 0))),
        ],
    ),
    "vision_msgs/msg/VisionClass": (
        [],
        [
            ("class_id", (T.BASE, ("uint16", 0))),
            ("class_name", (T.BASE, ("string", 0))),
        ],
    ),
    "vision_msgs/msg/LabelInfo": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("class_map", (T.SEQUENCE, ((T.NAME, "vision_msgs/msg/VisionClass"), 0))),
            ("threshold", (T.BASE, ("float32", 0))),
        ],
    ),
    "vision_msgs/msg/VisionInfo": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("method", (T.BASE, ("string", 0))),
            ("database_location", (T.BASE, ("string", 0))),
            ("database_version", (T.BASE, ("int32", 0))),
        ],
    ),
    "visualization_msgs/msg/ImageMarker": (
        [
            ("CIRCLE", "int32", 0),
            ("LINE_STRIP", "int32", 1),
            ("LINE_LIST", "int32", 2),
            ("POLYGON", "int32", 3),
            ("POINTS", "int32", 4),
            ("ADD", "int32", 0),
            ("REMOVE", "int32", 1),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("ns", (T.BASE, ("string", 0))),
            ("id", (T.BASE, ("int32", 0))),
            ("type", (T.BASE, ("int32", 0))),
            ("action", (T.BASE, ("int32", 0))),
            ("position", (T.NAME, "geometry_msgs/msg/Point")),
            ("scale", (T.BASE, ("float32", 0))),
            ("outline_color", (T.NAME, "std_msgs/msg/ColorRGBA")),
            ("filled", (T.BASE, ("uint8", 0))),
            ("fill_color", (T.NAME, "std_msgs/msg/ColorRGBA")),
            ("lifetime", (T.NAME, "builtin_interfaces/msg/Duration")),
            ("points", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point"), 0))),
            ("outline_colors", (T.SEQUENCE, ((T.NAME, "std_msgs/msg/ColorRGBA"), 0))),
        ],
    ),
    "visualization_msgs/msg/MeshFile": (
        [],
        [
            ("filename", (T.BASE, ("string", 0))),
            ("data", (T.SEQUENCE, ((T.BASE, ("uint8", 0)), 0))),
        ],
    ),
    "visualization_msgs/msg/UVCoordinate": (
        [],
        [
            ("u", (T.BASE, ("float32", 0))),
            ("v", (T.BASE, ("float32", 0))),
        ],
    ),
    "visualization_msgs/msg/Marker": (
        [
            ("ARROW", "int32", 0),
            ("CUBE", "int32", 1),
            ("SPHERE", "int32", 2),
            ("CYLINDER", "int32", 3),
            ("LINE_STRIP", "int32", 4),
            ("LINE_LIST", "int32", 5),
            ("CUBE_LIST", "int32", 6),
            ("SPHERE_LIST", "int32", 7),
            ("POINTS", "int32", 8),
            ("TEXT_VIEW_FACING", "int32", 9),
            ("MESH_RESOURCE", "int32", 10),
            ("TRIANGLE_LIST", "int32", 11),
            ("ARROW_STRIP", "int32", 12),
            ("ADD", "int32", 0),
            ("MODIFY", "int32", 0),
            ("DELETE", "int32", 2),
            ("DELETEALL", "int32", 3),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("ns", (T.BASE, ("string", 0))),
            ("id", (T.BASE, ("int32", 0))),
            ("type", (T.BASE, ("int32", 0))),
            ("action", (T.BASE, ("int32", 0))),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("scale", (T.NAME, "geometry_msgs/msg/Vector3")),
            ("color", (T.NAME, "std_msgs/msg/ColorRGBA")),
            ("lifetime", (T.NAME, "builtin_interfaces/msg/Duration")),
            ("frame_locked", (T.BASE, ("bool", 0))),
            ("points", (T.SEQUENCE, ((T.NAME, "geometry_msgs/msg/Point"), 0))),
            ("colors", (T.SEQUENCE, ((T.NAME, "std_msgs/msg/ColorRGBA"), 0))),
            ("texture_resource", (T.BASE, ("string", 0))),
            ("texture", (T.NAME, "sensor_msgs/msg/CompressedImage")),
            ("uv_coordinates", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/UVCoordinate"), 0))),
            ("text", (T.BASE, ("string", 0))),
            ("mesh_resource", (T.BASE, ("string", 0))),
            ("mesh_file", (T.NAME, "visualization_msgs/msg/MeshFile")),
            ("mesh_use_embedded_materials", (T.BASE, ("bool", 0))),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarkerControl": (
        [
            ("INHERIT", "uint8", 0),
            ("FIXED", "uint8", 1),
            ("VIEW_FACING", "uint8", 2),
            ("NONE", "uint8", 0),
            ("MENU", "uint8", 1),
            ("BUTTON", "uint8", 2),
            ("MOVE_AXIS", "uint8", 3),
            ("MOVE_PLANE", "uint8", 4),
            ("ROTATE_AXIS", "uint8", 5),
            ("MOVE_ROTATE", "uint8", 6),
            ("MOVE_3D", "uint8", 7),
            ("ROTATE_3D", "uint8", 8),
            ("MOVE_ROTATE_3D", "uint8", 9),
        ],
        [
            ("name", (T.BASE, ("string", 0))),
            ("orientation", (T.NAME, "geometry_msgs/msg/Quaternion")),
            ("orientation_mode", (T.BASE, ("uint8", 0))),
            ("interaction_mode", (T.BASE, ("uint8", 0))),
            ("always_visible", (T.BASE, ("bool", 0))),
            ("markers", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/Marker"), 0))),
            ("independent_marker_orientation", (T.BASE, ("bool", 0))),
            ("description", (T.BASE, ("string", 0))),
        ],
    ),
    "visualization_msgs/msg/MenuEntry": (
        [
            ("FEEDBACK", "uint8", 0),
            ("ROSRUN", "uint8", 1),
            ("ROSLAUNCH", "uint8", 2),
        ],
        [
            ("id", (T.BASE, ("uint32", 0))),
            ("parent_id", (T.BASE, ("uint32", 0))),
            ("title", (T.BASE, ("string", 0))),
            ("command", (T.BASE, ("string", 0))),
            ("command_type", (T.BASE, ("uint8", 0))),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarker": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("name", (T.BASE, ("string", 0))),
            ("description", (T.BASE, ("string", 0))),
            ("scale", (T.BASE, ("float32", 0))),
            ("menu_entries", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/MenuEntry"), 0))),
            (
                "controls",
                (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/InteractiveMarkerControl"), 0)),
            ),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarkerFeedback": (
        [
            ("KEEP_ALIVE", "uint8", 0),
            ("POSE_UPDATE", "uint8", 1),
            ("MENU_SELECT", "uint8", 2),
            ("BUTTON_CLICK", "uint8", 3),
            ("MOUSE_DOWN", "uint8", 4),
            ("MOUSE_UP", "uint8", 5),
        ],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("client_id", (T.BASE, ("string", 0))),
            ("marker_name", (T.BASE, ("string", 0))),
            ("control_name", (T.BASE, ("string", 0))),
            ("event_type", (T.BASE, ("uint8", 0))),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("menu_entry_id", (T.BASE, ("uint32", 0))),
            ("mouse_point", (T.NAME, "geometry_msgs/msg/Point")),
            ("mouse_point_valid", (T.BASE, ("bool", 0))),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarkerInit": (
        [],
        [
            ("server_id", (T.BASE, ("string", 0))),
            ("seq_num", (T.BASE, ("uint64", 0))),
            ("markers", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/InteractiveMarker"), 0))),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarkerPose": (
        [],
        [
            ("header", (T.NAME, "std_msgs/msg/Header")),
            ("pose", (T.NAME, "geometry_msgs/msg/Pose")),
            ("name", (T.BASE, ("string", 0))),
        ],
    ),
    "visualization_msgs/msg/InteractiveMarkerUpdate": (
        [
            ("KEEP_ALIVE", "uint8", 0),
            ("UPDATE", "uint8", 1),
        ],
        [
            ("server_id", (T.BASE, ("string", 0))),
            ("seq_num", (T.BASE, ("uint64", 0))),
            ("type", (T.BASE, ("uint8", 0))),
            ("markers", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/InteractiveMarker"), 0))),
            ("poses", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/InteractiveMarkerPose"), 0))),
            ("erases", (T.SEQUENCE, ((T.BASE, ("string", 0)), 0))),
        ],
    ),
    "visualization_msgs/msg/MarkerArray": (
        [],
        [
            ("markers", (T.SEQUENCE, ((T.NAME, "visualization_msgs/msg/Marker"), 0))),
        ],
    ),
}
