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

"""Galaxea R1 Pro connection module: ROS 2 control + sensor streams.

Owns all ROS 2 traffic for the R1 Pro and exposes it as dimos streams under
the control contract: the upper body's 18 joints (torso 4 + left arm 7 +
right arm 7) take positions on ``position_command``. The ordinary planar
model joints take world-frame velocities on ``velocity_command``. Feedback
goes out on ``joint_state``, the two
IMU streams and wheel odometry. The chassis lidar is ours, not the
vendor's: see ``r1pro_lidar_odometry``.

Commands arrive as standard messages and go through ``command``: checked
against the model limits (out-of-range speeds are rejected), then forwarded
by ``write``. Measured odometry supplies planar positions and rotates the
vendor body velocity into world derivatives; fresh measured yaw rotates
outgoing velocities back to the vendor body frame. Once a command was accepted, going
``deadman_timeout_s`` without another halts the robot: zero chassis speed
and the arms held where they were last measured.

Sensors run on a second RawROS node. Conversion happens on per-stream workers
behind latest-wins queues.

The on-robot joint tracker manages PD gains; targets are followed at
``config.tracking_speed``. Chassis gate handling lives in the on-robot
``galaxea-dimos`` gatekeeper. ROS env (``ROS_DOMAIN_ID`` etc.) comes from
the environment.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import queue
import subprocess
import threading
from threading import Thread
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex.disposable import Disposable

if TYPE_CHECKING:
    from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE, R1PRO_PLANAR_MODEL
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Joint layout — flat 18-element command indexing, torso then left then right.
_TORSO_SLICE = slice(0, 4)
_LEFT_SLICE = slice(4, 11)
_RIGHT_SLICE = slice(11, 18)
_NUM_MOTORS = 18

_FEEDBACK_DISCOVERY_TIMEOUT_S = 5.0

R1PRO_UPPER_BODY_JOINTS: list[str] = [coordinator_name(j) for j in UPPER_BODY_JOINTS]
assert len(R1PRO_UPPER_BODY_JOINTS) == _NUM_MOTORS

# Ordinary model joints, in world x/y/yaw order.
R1PRO_BASE_JOINTS: list[str] = list(R1PRO_PLANAR_BASE.joint_names)

# JPEG color streams gated by config.enable_wrist_color: stream name → ROS topic.
_COLOR_CAMERAS: dict[str, str] = {
    "wrist_left_color": "/hdas/camera_wrist_left/color/image_raw/compressed",
    "wrist_right_color": "/hdas/camera_wrist_right/color/image_raw/compressed",
}

# Raw-image depth streams, gated by config.enable_wrist_depth.
_WRIST_DEPTH_CAMERAS: dict[str, str] = {
    "wrist_left_depth": "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
    "wrist_right_depth": "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
}
_HEAD_DEPTH_TOPIC = "/hdas/camera_head/depth/depth_registered"


@dataclass
class _StreamStat:
    """Per-stream counters; see the ``sensor_stats`` rpc."""

    received: int = 0
    dropped: int = 0
    decoded: int = 0
    errors: int = 0
    bytes_in: int = 0
    decode_ms_sum: float = 0.0
    last_mono: float = 0.0


def _make_qos() -> Any:
    """BEST_EFFORT + VOLATILE QoS — the profile the R1 Pro topics expect."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(  # type: ignore[no-untyped-call]
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _stamp_secs(msg: Any) -> float:
    """Header stamp in seconds, falling back to wall clock if the driver left it 0."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return ts if ts > 0 else time.time()


def _ros_stamp_now() -> Any:
    from builtin_interfaces.msg import Time as RosTime

    t = time.time()
    sec = int(t)
    return RosTime(sec=sec, nanosec=int((t - sec) * 1e9))


class R1ProConnectionConfig(ConnectionConfig):
    """How the R1 Pro is told to move, plus its sensor and vendor-stack knobs.

    The contract fields (``source``, ``model``, ``joint_commands``,
    ``deadman_timeout_s``) default to the real robot: the
    18 upper-body joints take positions only — the vendor tracker's speed
    is a tracking-speed cap, not a continuous velocity command — and the
    planar joints take velocities bounded by ``R1PRO_PLANAR_BASE.velocity_limits``.

    Attributes:
        odometry_topic: ROS nav_msgs/Odometry input. Configure this to the
            deployed vendor/localization topic, with odom -> base_link frames.
            No speed-integrated or assumed-zero-yaw fallback is used.
        odometry_timeout_s: Independent pose timestamp and receipt age limit.
        tracking_speed: rad/s the on-board tracker follows targets at.
        publish_odom_tf: Gates wheel odometry's odom -> base_link tf and the
            `odom` pose; `odometry` always goes out.
        frame_id: The robot's root frame, stamped on feedback.
        odom_frame_id: The frame wheel odometry integrates in.
        publish_rate_hz: How often the aggregated joint_state goes out.
        sensor_stats_interval_s: Seconds between per-stream stats log lines
            (0 disables).
        enable_wrist_depth: Wrist depth is raw 16-bit at up to 30 Hz per
            wrist — too heavy for the on-robot CPU budget by default.
        enable_wrist_color: Each wrist copies and publishes JPEG frames even
            when unread; turn off if nothing reads them.
        color_publish_hz: Max Hz per color camera (0 = no cap); the cameras
            arrive at ~28 Hz, so 30 passes every frame.
        boot_vendor_stack: Boot the vendor stack (HDAS, mobiman), which
            serves every ROS topic used here, if it is not running.
        vendor_profile: "" boots vendor_setup.sh's own set; a vendor profile,
            e.g. "ATCStandard/R1PROVRTeleop", boots that.
    """

    source: str = "r1pro"
    model: RobotModel | None = R1PRO_PLANAR_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = {
        **dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, (Interface.POSITION,)),
        **dict.fromkeys(R1PRO_BASE_JOINTS, (Interface.VELOCITY,)),
    }
    odometry_topic: str = "/odom"
    odometry_timeout_s: float = Field(default=0.2, gt=0)
    publish_rate_hz: float = Field(default=100.0)
    tracking_speed: float = Field(default=0.5)
    publish_odom_tf: bool = Field(default=True)
    frame_id: str = Field(default="base_link")
    odom_frame_id: str = Field(default="odom")
    sensor_stats_interval_s: float = Field(default=10.0)
    enable_wrist_depth: bool = Field(default=False)
    enable_wrist_color: bool = Field(default=True)
    color_publish_hz: float = Field(default=30.0)
    boot_vendor_stack: bool = Field(default=True)
    vendor_profile: str = Field(default="")


class R1ProConnection(ConnectionModule):
    """R1 Pro connection module — ROS 2 control and sensor nodes.

    ``position_command`` carries all 18 upper-body joints' targets in rad,
    named ``r1pro/<joint>``; ``velocity_command`` carries planar joint velocities. Both
    are checked against the limits table before anything is sent. Feedback
    for the upper body is published once every segment has reported, at
    ``config.publish_rate_hz``. Planar feedback is published on actual odometry
    receipts, independently of that timer and the IMUs.
    """

    config: R1ProConnectionConfig

    # Control inputs.
    position_command: In[JointState]
    velocity_command: In[JointState]

    # Whole-body feedback.
    joint_state: Out[JointState]
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]

    # Base feedback. `odometry` carries the wheel odometry as pose + twist
    # for navigation consumers (voxel map, planner); `odom` is the same pose
    # as a PoseStamped, gated with the tf by config.publish_odom_tf.
    odom: Out[PoseStamped]
    odometry: Out[Odometry]
    tf: Out[TFMessage]

    # Perception.
    head_depth: Out[Image]
    wrist_left_color: Out[CompressedImage]
    wrist_left_depth: Out[Image]
    wrist_right_color: Out[CompressedImage]
    wrist_right_depth: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # ROS pubsub instances.
        self._ros: RawROS | None = None
        self._sensors: RawROS | None = None
        self._cmd_torso_topic: RawROSTopic | None = None
        self._cmd_left_topic: RawROSTopic | None = None
        self._cmd_right_topic: RawROSTopic | None = None
        self._speed_topic: RawROSTopic | None = None
        self._control_unsubs: list[Any] = []
        self._sensor_unsubs: list[Any] = []

        # Guards the _latest_* / _*_seen snapshot shared between feedback
        # callbacks, IMU workers, and the publish loop.
        self._lock = threading.Lock()
        self._latest_torso_q: list[float] = [0.0] * 4
        self._latest_torso_dq: list[float] = [0.0] * 4
        self._latest_torso_eff: list[float] = [0.0] * 4
        self._latest_left_q: list[float] = [0.0] * 7
        self._latest_left_dq: list[float] = [0.0] * 7
        self._latest_left_eff: list[float] = [0.0] * 7
        self._latest_right_q: list[float] = [0.0] * 7
        self._latest_right_dq: list[float] = [0.0] * 7
        self._latest_right_eff: list[float] = [0.0] * 7
        self._torso_seen = False
        self._ts_torso = 0.0
        self._ts_left = 0.0
        self._ts_right = 0.0
        self._left_seen = False
        self._right_seen = False
        self._latest_imu_chassis: Imu | None = None
        self._latest_imu_torso: Imu | None = None

        # Only actual validated odometry authorizes world -> body commands.
        self._base_state: JointState | None = None
        self._odom_received_mono: float | None = None

        # Sensor decode pipeline.
        self._sensor_stop = threading.Event()
        self._sensor_workers: list[Thread] = []
        self._queues: dict[str, queue.Queue[Any]] = {}
        # Per-stream diagnostics, under their own lock.
        self._stats: dict[str, _StreamStat] = {}
        self._stats_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    # Lifecycle

    @rpc
    def build(self) -> None:
        super().build()
        # In build, so the vendor stack is up before any module starts.
        if self.config.boot_vendor_stack:
            setup = Path(__file__).with_name("vendor_setup.sh")
            subprocess.run(["bash", str(setup), self.config.vendor_profile], check=True)

    def connect(self) -> None:
        """Bring up both RawROS nodes and every topic, moving nothing."""
        # Lazy import — RawROS pulls rclpy which must not load on import in
        # environments without ROS 2.
        from dimos.protocol.pubsub.impl.rospubsub import RawROS

        self._ros = RawROS(node_name="r1pro_control")
        self._ros.start()
        self._sensors = RawROS(node_name="r1pro_sensors")
        self._sensors.start()

        self._setup_control_topics()
        self._setup_sensor_streams()

    @rpc
    def start(self) -> None:
        super().start()

        # Wait for at least one feedback frame from each segment so the
        # publish loop can ship a fully-populated joint_state.
        deadline = time.monotonic() + _FEEDBACK_DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline:
            with self._lock:
                if self._torso_seen and self._left_seen and self._right_seen:
                    break
            time.sleep(0.05)

        with self._lock:
            seen = (self._torso_seen, self._left_seen, self._right_seen)
        if not all(seen):
            logger.warning(
                "Feedback discovery timeout: torso=%s left=%s right=%s — "
                "joint_state will gate first publish until all three arrive.",
                *seen,
            )

        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.velocity_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.VELOCITY,)))
                )
            )
        )

        self._publish_thread = Thread(target=self._publish_loop, name="r1pro-publish", daemon=True)
        self._publish_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._sensor_stop.set()

        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        # Unblock sensor workers.
        for q in self._queues.values():
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for t in self._sensor_workers:
            t.join(timeout=1.0)
        self._sensor_workers.clear()
        self._queues.clear()

        # Halts the robot (zero chassis speed, arms held) before the ROS
        # nodes come down, then tears the module down.
        super().stop()

        for unsubs in (self._sensor_unsubs, self._control_unsubs):
            for unsub in unsubs:
                try:
                    unsub()
                except (OSError, RuntimeError) as e:
                    logger.warning(f"unsubscribe raised: {e}")
            unsubs.clear()
        for ros in (self._sensors, self._ros):
            if ros is not None:
                try:
                    ros.stop()
                except (OSError, RuntimeError) as e:
                    logger.warning(f"RawROS stop raised: {e}")
        self._sensors = None
        self._ros = None

        with self._lock:
            self._torso_seen = self._left_seen = self._right_seen = False
            self._latest_imu_chassis = None
            self._latest_imu_torso = None
            self._base_state = None
            self._odom_received_mono = None

    # The control contract

    def write(self, values: Values) -> None:
        """Forward one checked command: joint targets to the three vendor
        joint-state topics, chassis speeds to the speed topic."""
        ros = self._ros
        if ros is None:
            raise RuntimeError("R1ProConnection is not connected")
        stamp = _ros_stamp_now()

        positions = [
            values.get(f"{joint}{SEPARATOR}{Interface.POSITION}")
            for joint in R1PRO_UPPER_BODY_JOINTS
        ]
        speeds = [values.get(f"{joint}/velocity") for joint in R1PRO_BASE_JOINTS]
        base_packet: tuple[float, float, float] | None = None
        if any(speed is not None for speed in speeds):
            vx, vy, wz = (0.0 if speed is None else speed for speed in speeds)
            # A literal zero is frame independent, including when odometry is absent.
            if vx == vy == wz == 0.0:
                base_packet = (0.0, 0.0, 0.0)
            else:
                with self._lock:
                    state, received = self._base_state, self._odom_received_mono
                if (
                    state is None
                    or received is None
                    or time.monotonic() - received > self.config.odometry_timeout_s
                    or not 0 <= time.time() - state.ts <= self.config.odometry_timeout_s
                ):
                    raise ValueError("fresh measured odometry yaw is required")
                yaw = float(state.position[2])
                cy, sy = math.cos(yaw), math.sin(yaw)
                base_packet = (cy * vx + sy * vy, -sy * vx + cy * vy, wz)
        # Validate/compute the whole batch before any arm SDK send.
        if any(target is not None for target in positions):
            if not all(target is not None for target in positions):
                raise ValueError(
                    "a position command must cover all "
                    f"{_NUM_MOTORS} joints, got "
                    f"{sum(target is not None for target in positions)}"
                )
            self._send_joint_targets([float(q) for q in positions if q is not None], stamp)

        if base_packet is not None:
            self._send_chassis_speed(*base_packet, stamp)

    def halt(self) -> None:
        """Zero the chassis, and hold the arms where they were last measured.

        Safe to call any number of times, including before any command or
        feedback: without a full reading there is nothing to hold the arms
        at, and the vendor tracker keeps its last target.
        """
        ros = self._ros
        if ros is None:
            return
        stamp = _ros_stamp_now()
        if self._speed_topic is not None:
            self._send_chassis_speed(0.0, 0.0, 0.0, stamp)
        with self._lock:
            if not (self._torso_seen and self._left_seen and self._right_seen):
                return
            q = list(self._latest_torso_q) + list(self._latest_left_q) + list(self._latest_right_q)
        self._send_joint_targets(q, stamp)

    # ROS command senders

    def _send_joint_targets(self, q: list[float], stamp: Any) -> None:
        """One ROS JointState per segment: position targets + tracking speed."""
        from sensor_msgs.msg import JointState as RosJointState

        ros = self._ros
        if ros is None:
            return  # pre-connect / post-stop
        speed = self.config.tracking_speed
        for topic, sl in (
            (self._cmd_torso_topic, _TORSO_SLICE),
            (self._cmd_left_topic, _LEFT_SLICE),
            (self._cmd_right_topic, _RIGHT_SLICE),
        ):
            if topic is None:
                continue
            segment = q[sl]
            cmd = RosJointState()
            cmd.header.stamp = stamp
            cmd.name = [""]
            cmd.position = segment
            cmd.velocity = [speed] * len(segment)
            cmd.effort = [0.0]
            ros.publish(topic, cmd)

    def _send_chassis_speed(self, vx: float, vy: float, wz: float, stamp: Any) -> None:
        from geometry_msgs.msg import TwistStamped

        ros = self._ros
        if ros is None or self._speed_topic is None:
            return  # pre-connect / post-stop

        cmd = TwistStamped()
        cmd.header.stamp = stamp
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.angular.z = wz
        ros.publish(self._speed_topic, cmd)

    # Control topics

    def _setup_control_topics(self) -> None:
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry as RosOdometry
        from sensor_msgs.msg import JointState as RosJointState

        from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic

        assert self._ros is not None
        qos = _make_qos()

        self._cmd_torso_topic = RawROSTopic(
            "/motion_target/target_joint_state_torso", RosJointState, qos=qos
        )
        self._cmd_left_topic = RawROSTopic(
            "/motion_target/target_joint_state_arm_left", RosJointState, qos=qos
        )
        self._cmd_right_topic = RawROSTopic(
            "/motion_target/target_joint_state_arm_right", RosJointState, qos=qos
        )
        self._speed_topic = RawROSTopic(
            "/motion_target/target_speed_chassis", TwistStamped, qos=qos
        )

        for topic, cb in (
            (RawROSTopic("/hdas/feedback_torso", RosJointState, qos=qos), self._on_feedback_torso),
            (
                RawROSTopic("/hdas/feedback_arm_left", RosJointState, qos=qos),
                self._on_feedback_left,
            ),
            (
                RawROSTopic("/hdas/feedback_arm_right", RosJointState, qos=qos),
                self._on_feedback_right,
            ),
        ):
            self._control_unsubs.append(self._ros.subscribe(topic, cb))

        # A real measured pose is required; do not integrate speed from yaw=0.
        self._control_unsubs.append(
            self._ros.subscribe(
                RawROSTopic(self.config.odometry_topic, RosOdometry, qos=qos),
                self._on_odometry,
            )
        )

    # Sensor streams (isolated RawROS + per-stream decode workers)

    def _setup_sensor_streams(self) -> None:
        try:
            from sensor_msgs.msg import (
                CompressedImage as RosCompressedImage,
                Image as RosImage,
                Imu as RosImu,
            )
        except ImportError:
            logger.warning("sensor_msgs not available — sensor streams disabled")
            return

        from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic

        assert self._sensors is not None
        qos = _make_qos()

        def add_stream(
            stream: str,
            ros_topic: str,
            ros_type: type,
            worker: Any,
            *args: Any,
            maxsize: int = 1,
        ) -> None:
            q: queue.Queue[Any] = queue.Queue(maxsize=maxsize)
            self._queues[stream] = q
            assert self._sensors is not None
            self._sensor_unsubs.append(
                self._sensors.subscribe(
                    RawROSTopic(ros_topic, ros_type, qos=qos),
                    self._make_rx_cb(stream, q),
                )
            )
            self._sensor_workers.append(
                Thread(target=worker, args=(stream, q, *args), daemon=True, name=f"r1pro-{stream}")
            )

        if self.config.enable_wrist_color:
            for stream, topic in _COLOR_CAMERAS.items():
                add_stream(stream, topic, RosCompressedImage, self._compressed_image_loop)

        add_stream("head_depth", _HEAD_DEPTH_TOPIC, RosImage, self._convert_loop, Image)

        if self.config.enable_wrist_depth:
            for stream, topic in _WRIST_DEPTH_CAMERAS.items():
                add_stream(stream, topic, RosImage, self._convert_loop, Image)

        add_stream("imu_chassis", "/hdas/imu_chassis", RosImu, self._imu_loop, maxsize=4)
        add_stream("imu_torso", "/hdas/imu_torso", RosImu, self._imu_loop, maxsize=4)

        # Periodic per-stream stats reporter (joined on stop via _sensor_stop).
        if self.config.sensor_stats_interval_s > 0:
            self._sensor_workers.append(
                Thread(target=self._stats_report_loop, daemon=True, name="r1pro-sensor-stats")
            )

        for t in self._sensor_workers:
            t.start()

    # Per-stream diagnostics

    def _make_rx_cb(self, stream: str, q: queue.Queue[Any]) -> Any:
        """Record receive stats + latest-wins enqueue (runs on the spin thread)."""

        def cb(msg: Any, _topic: Any) -> None:
            data = getattr(msg, "data", None)
            nbytes = len(data) if data is not None else 0
            dropped = _enqueue_drop_oldest(q, msg)
            with self._stats_lock:
                st = self._stats.setdefault(stream, _StreamStat())
                st.received += 1
                st.bytes_in += nbytes
                st.last_mono = time.monotonic()
                if dropped:
                    st.dropped += 1

        return cb

    def _record_decode(self, stream: str, ms: float, ok: bool) -> None:
        with self._stats_lock:
            st = self._stats.setdefault(stream, _StreamStat())
            if ok:
                st.decoded += 1
                st.decode_ms_sum += ms
            else:
                st.errors += 1

    def _stats_snapshot(self) -> dict[str, _StreamStat]:
        with self._stats_lock:
            return {k: replace(v) for k, v in self._stats.items()}

    def _stats_report_loop(self) -> None:
        """Log per-stream rates every ``sensor_stats_interval_s`` until stop."""
        interval = self.config.sensor_stats_interval_s
        prev = self._stats_snapshot()
        prev_t = time.monotonic()
        while not self._sensor_stop.wait(interval):
            cur = self._stats_snapshot()
            now = time.monotonic()
            dt = now - prev_t
            if dt <= 0:
                continue
            lines = []
            for name in sorted(cur):
                s = cur[name]
                p = prev.get(name, _StreamStat())
                d_dec = s.decoded - p.decoded
                if not (s.received or s.decoded):
                    continue
                rx = (s.received - p.received) / dt
                drop = (s.dropped - p.dropped) / dt
                dec = d_dec / dt
                dms = (s.decode_ms_sum - p.decode_ms_sum) / d_dec if d_dec > 0 else 0.0
                mibps = (s.bytes_in - p.bytes_in) / dt / 1048576
                age = now - s.last_mono if s.last_mono else -1.0
                lines.append(
                    f"{name}: rx={rx:.1f}/s drop={drop:.1f}/s dec={dec:.1f}/s "
                    f"decode={dms:.1f}ms in={mibps:.1f}MiB/s age={age:.1f}s err={s.errors}"
                )
            if lines:
                logger.info("R1Pro sensor stats (%.0fs):\n  %s", dt, "\n  ".join(lines))
            prev = cur
            prev_t = now

    @rpc
    def sensor_stats(self) -> dict[str, Any]:
        """Per-stream cumulative counters: received/dropped/decoded/errors/bytes,
        avg decode ms, and last-message age."""
        now = time.monotonic()
        return {
            name: {
                "received": s.received,
                "dropped": s.dropped,
                "decoded": s.decoded,
                "errors": s.errors,
                "bytes_in": s.bytes_in,
                "avg_decode_ms": (s.decode_ms_sum / s.decoded if s.decoded else 0.0),
                "age_s": (now - s.last_mono if s.last_mono else -1.0),
            }
            for name, s in self._stats_snapshot().items()
        }

    # Control feedback callbacks (3 segments)

    def _on_feedback_torso(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(
                msg, self._latest_torso_q, self._latest_torso_dq, self._latest_torso_eff
            )
            self._ts_torso = _stamp_secs(msg)
            self._torso_seen = True

    def _on_feedback_left(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(
                msg, self._latest_left_q, self._latest_left_dq, self._latest_left_eff
            )
            self._ts_left = _stamp_secs(msg)
            self._left_seen = True

    def _on_feedback_right(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(
                msg, self._latest_right_q, self._latest_right_dq, self._latest_right_eff
            )
            self._ts_right = _stamp_secs(msg)
            self._right_seen = True

    @staticmethod
    def _copy_segment(
        msg: Any, q_dst: list[float], dq_dst: list[float], eff_dst: list[float]
    ) -> None:
        n = min(len(msg.position), len(q_dst))
        q_dst[:n] = msg.position[:n]
        if msg.velocity:
            nv = min(len(msg.velocity), len(dq_dst))
            dq_dst[:nv] = msg.velocity[:nv]
        if msg.effort:
            ne = min(len(msg.effort), len(eff_dst))
            eff_dst[:ne] = msg.effort[:ne]

    # Measured vendor odometry: world pose, body-frame twist.

    def _on_odometry(self, msg: Any, _topic: Any) -> None:
        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion

        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            v, w = msg.twist.twist.linear, msg.twist.twist.angular
            numbers = (ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z)
            if not all(math.isfinite(x) for x in numbers):
                raise ValueError("non-finite odometry")
            if not math.isclose(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, abs_tol=1e-3):
                raise ValueError("odometry quaternion must be unit length")
            if not 0 <= time.time() - ts <= self.config.odometry_timeout_s:
                raise ValueError("stale odometry timestamp")
            if (
                msg.header.frame_id != self.config.odom_frame_id
                or msg.child_frame_id != self.config.frame_id
            ):
                raise ValueError("odometry frame mismatch")
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            cy, sy = math.cos(yaw), math.sin(yaw)
            state = JointState(
                ts=ts,
                frame_id=self.config.odom_frame_id,
                name=R1PRO_BASE_JOINTS,
                position=[p.x, p.y, yaw],
                velocity=[cy * v.x - sy * v.y, sy * v.x + cy * v.y, w.z],
            )
        except (AttributeError, TypeError, ValueError):
            # Invalid traffic must neither refresh nor retain output authority.
            with self._lock:
                self._base_state = None
                self._odom_received_mono = None
            return
        with self._lock:
            self._base_state = state
            self._odom_received_mono = time.monotonic()
        # Publish only on real odometry receipts, never from the arm timer.
        self.joint_state.publish(state)
        position = Vector3(p.x, p.y, p.z)
        orientation = Quaternion(q.x, q.y, q.z, q.w)
        pose = PoseStamped(
            ts=ts, frame_id=self.config.odom_frame_id, position=position, orientation=orientation
        )
        self.odometry.publish(
            Odometry(
                ts=ts,
                frame_id=self.config.odom_frame_id,
                child_frame_id=self.config.frame_id,
                pose=Pose(position, orientation),
                twist=Twist(Vector3(v.x, v.y, v.z), Vector3(w.x, w.y, w.z)),
            )
        )
        if self.config.publish_odom_tf:
            self.odom.publish(pose)
            self.tf.publish(TFMessage(Transform.from_pose(self.config.frame_id, pose)))

    # Aggregated joint_state publish loop

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id
        bootstrapped = False

        while not self._stop_event.is_set():
            with self._lock:
                if not bootstrapped:
                    if not (self._torso_seen and self._left_seen and self._right_seen):
                        # No publish until every segment reports once — a
                        # zero-position snapshot would walk the arms to home.
                        positions = None
                    else:
                        bootstrapped = True
                if bootstrapped:
                    positions = (
                        list(self._latest_torso_q)
                        + list(self._latest_left_q)
                        + list(self._latest_right_q)
                    )
                    velocities = (
                        list(self._latest_torso_dq)
                        + list(self._latest_left_dq)
                        + list(self._latest_right_dq)
                    )
                    efforts = (
                        list(self._latest_torso_eff)
                        + list(self._latest_left_eff)
                        + list(self._latest_right_eff)
                    )
                    imu_chassis = self._latest_imu_chassis
                    imu_torso = self._latest_imu_torso
                    # Oldest of the three segments: the fused snapshot is only
                    # as fresh as its stalest part.
                    ts = min(self._ts_torso, self._ts_left, self._ts_right)

            if bootstrapped:
                self.joint_state.publish(
                    JointState(
                        ts=ts,
                        frame_id=frame_id,
                        name=R1PRO_UPPER_BODY_JOINTS,
                        position=positions,  # type: ignore[arg-type]
                        velocity=velocities,
                        effort=efforts,
                    )
                )
                if imu_chassis is not None:
                    self.imu_chassis.publish(imu_chassis)
                if imu_torso is not None:
                    self.imu_torso.publish(imu_torso)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    # Sensor workers

    def _compressed_image_loop(self, stream: str, q: queue.Queue[Any]) -> None:
        out: Out[CompressedImage] = getattr(self, stream)
        hz = self.config.color_publish_hz
        min_period = 1.0 / hz if hz > 0 else 0.0
        last_pub = 0.0
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            t0 = time.perf_counter()
            if min_period > 0.0 and t0 - last_pub < min_period:
                continue
            last_pub = t0
            try:
                stamp = msg.header.stamp
                ts = stamp.sec + stamp.nanosec * 1e-9
                out.publish(
                    CompressedImage(
                        data=bytes(msg.data),
                        format="jpeg",
                        frame_id=msg.header.frame_id or stream,
                        ts=ts if ts > 0 else time.time(),
                    )
                )
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=True)
            except Exception:
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=False)
                logger.exception(f"R1Pro {stream} conversion error")

    def _convert_loop(self, stream: str, q: queue.Queue[Any], dimos_type: type) -> None:
        """ros_to_dimos passthrough worker (depth images)."""
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        out: Out[Any] = getattr(self, stream)
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            t0 = time.perf_counter()
            try:
                out.publish(ros_to_dimos(msg, dimos_type))
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=True)
            except Exception:
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=False)
                logger.exception(f"R1Pro {stream} decode error")

    def _imu_loop(self, stream: str, q: queue.Queue[Any]) -> None:
        """Store the latest converted IMU; re-emitted by the publish loop."""
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        target_attr = "_latest_imu_chassis" if stream == "imu_chassis" else "_latest_imu_torso"
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                imu = ros_to_dimos(msg, Imu)
                with self._lock:
                    setattr(self, target_attr, imu)
            except Exception:
                logger.exception(f"R1Pro {stream} decode error")


def _enqueue_drop_oldest(q: queue.Queue[Any], item: Any) -> bool:
    """Latest-frame-wins enqueue for size-1 sensor queues.

    Returns True if a queued item had to be evicted (consumer not keeping up).
    """
    try:
        q.put_nowait(item)
        return False
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass
        return True
