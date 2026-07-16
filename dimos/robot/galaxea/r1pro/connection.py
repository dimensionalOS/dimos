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

"""Galaxea R1 Pro connection Module: ROS 2 control + sensor streams.

Owns all ROS 2 traffic for the R1 Pro and exposes it as dimos streams:
whole-body joint control (18 DOF: torso 4 + left arm 7 + right arm 7) for
the ``transport_lcm`` whole-body adapter, chassis ``cmd_vel``/``odom`` for
the twist-base adapter, plus cameras, lidar, and IMUs.

Sensors run on a second, isolated RawROS instance (own DDS participant) so
bulk camera traffic never contends with 100 Hz control traffic. Decoding
happens on per-stream workers behind latest-wins queues.

The on-robot joint tracker manages PD gains; only ``q`` and ``dq`` are
forwarded. Chassis gate handling lives in the on-robot ``galaxea-dimos``
gatekeeper. ROS env (``ROS_DOMAIN_ID`` etc.) comes from the environment.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import queue
import threading
from threading import Thread
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex.disposable import Disposable

if TYPE_CHECKING:
    from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import VEL_STOP
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Joint layout — flat 18-element MotorCommandArray indexing.
_TORSO_SLICE = slice(0, 4)
_LEFT_SLICE = slice(4, 11)
_RIGHT_SLICE = slice(11, 18)
_NUM_MOTORS = 18

_FEEDBACK_DISCOVERY_TIMEOUT_S = 5.0

# URDF-faithful joint names. Indices match the flat MotorCommandArray layout.
_R1PRO_UPPER_BODY_BARE: list[str] = (
    [f"torso_joint{i}" for i in range(1, 5)]
    + [f"left_arm_joint{i}" for i in range(1, 8)]
    + [f"right_arm_joint{i}" for i in range(1, 8)]
)
R1PRO_UPPER_BODY_JOINTS: list[str] = [f"r1pro/{j}" for j in _R1PRO_UPPER_BODY_BARE]
assert len(R1PRO_UPPER_BODY_JOINTS) == _NUM_MOTORS

# JPEG color streams: stream name → ROS topic.
_COLOR_CAMERAS: dict[str, str] = {
    "head_left_color": "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "head_right_color": "/hdas/camera_head/right_raw/image_raw_color/compressed",
    "wrist_left_color": "/hdas/camera_wrist_left/color/image_raw/compressed",
    "wrist_right_color": "/hdas/camera_wrist_right/color/image_raw/compressed",
}

# Raw-image depth streams, gated by config.enable_wrist_depth.
_WRIST_DEPTH_CAMERAS: dict[str, str] = {
    "wrist_left_depth": "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
    "wrist_right_depth": "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
}
_HEAD_DEPTH_TOPIC = "/hdas/camera_head/depth/depth_registered"
_LIDAR_TOPIC = "/hdas/lidar_chassis_left"


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

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _ros_stamp_now() -> Any:
    from builtin_interfaces.msg import Time as RosTime

    t = time.time()
    sec = int(t)
    return RosTime(sec=sec, nanosec=int((t - sec) * 1e9))


class R1ProConnectionConfig(ModuleConfig):
    publish_rate_hz: float = Field(default=100.0)
    # rad/s used when MotorCommand.dq is the VEL_STOP sentinel or 0.
    tracking_speed: float = Field(default=0.5)
    publish_odom: bool = Field(default=True)
    frame_id: str = Field(default="r1pro_base_link")
    odom_frame_id: str = Field(default="odom")
    # Seconds between per-stream sensor-stats log lines (0 disables).
    sensor_stats_interval_s: float = Field(default=10.0)
    # Wrist depth is raw 16-bit at up to 30 Hz per wrist — too heavy for the
    # on-robot CPU budget by default; enable when manipulation needs it.
    enable_wrist_depth: bool = Field(default=False)
    # Max Hz per color camera (0 = no cap); the gate runs before the JPEG
    # decode so skipped frames cost ~nothing.
    color_publish_hz: float = Field(default=5.0)


class R1ProConnection(Module):
    """R1 Pro Module — ROS 2 control node + isolated sensor node."""

    config: R1ProConnectionConfig

    # Control inputs.
    motor_command: In[MotorCommandArray]
    cmd_vel: In[Twist]

    # Whole-body feedback.
    motor_states: Out[JointState]
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]

    # Base feedback. `odom` feeds the twist-base transport adapter;
    # `odometry` carries the same wheel odometry as pose + twist for
    # navigation consumers (voxel map, planner).
    odom: Out[PoseStamped]
    odometry: Out[Odometry]

    # Perception.
    head_left_color: Out[Image]
    head_right_color: Out[Image]
    head_depth: Out[Image]
    lidar: Out[PointCloud2]
    wrist_left_color: Out[Image]
    wrist_left_depth: Out[Image]
    wrist_right_color: Out[Image]
    wrist_right_depth: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # ROS pubsub instances (control + isolated sensors).
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
        self._left_seen = False
        self._right_seen = False
        self._latest_imu_chassis: Imu | None = None
        self._latest_imu_torso: Imu | None = None

        # Odom dead-reckoning, integrated from /motion_control/chassis_speed.
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_ts: float | None = None

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
    def start(self) -> None:
        super().start()

        # Lazy import — RawROS pulls rclpy which must not load on import in
        # environments without ROS 2.
        from dimos.protocol.pubsub.impl.rospubsub import RawROS

        logger.info("Starting R1ProConnection (control + isolated sensor node)...")
        self._ros = RawROS(node_name="r1pro_control")
        self._ros.start()
        self._sensors = RawROS(node_name="r1pro_sensors", isolated_context=True)
        self._sensors.start()

        self._setup_control_topics()
        self._setup_sensor_streams()

        # Wait for at least one feedback frame from each segment so the
        # publish loop can ship a fully-populated motor_states.
        logger.info(
            "Waiting up to %.0fs for first feedback from torso/left_arm/right_arm...",
            _FEEDBACK_DISCOVERY_TIMEOUT_S,
        )
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
                "motor_states will gate first publish until all three arrive.",
                *seen,
            )

        self.register_disposable(
            Disposable(self.motor_command.subscribe(self._on_motor_command))
        )
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

        self._publish_thread = Thread(
            target=self._publish_loop, name="r1pro-publish", daemon=True
        )
        self._publish_thread.start()

        logger.info("R1ProConnection started")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._sensor_stop.set()

        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        # Unblock decode workers, then stop the sensor pubsub (owns the
        # isolated context) and the control pubsub.
        for q in self._queues.values():
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for t in self._sensor_workers:
            t.join(timeout=1.0)
        self._sensor_workers.clear()
        self._queues.clear()

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

        logger.info("R1ProConnection stopped")
        super().stop()

    # Control topics

    def _setup_control_topics(self) -> None:
        from geometry_msgs.msg import TwistStamped
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
            (RawROSTopic("/hdas/feedback_arm_left", RosJointState, qos=qos), self._on_feedback_left),
            (
                RawROSTopic("/hdas/feedback_arm_right", RosJointState, qos=qos),
                self._on_feedback_right,
            ),
        ):
            self._control_unsubs.append(self._ros.subscribe(topic, cb))

        if self.config.publish_odom:
            # Executed chassis speed — integrated into wheel odometry.
            self._control_unsubs.append(
                self._ros.subscribe(
                    RawROSTopic("/motion_control/chassis_speed", TwistStamped, qos=qos),
                    self._on_chassis_speed,
                )
            )

    # Sensor streams (isolated RawROS + per-stream decode workers)

    def _setup_sensor_streams(self) -> None:
        try:
            from sensor_msgs.msg import CompressedImage
            from sensor_msgs.msg import Image as RosImage
            from sensor_msgs.msg import Imu as RosImu
            from sensor_msgs.msg import PointCloud2 as RosPointCloud2
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

        for stream, topic in _COLOR_CAMERAS.items():
            add_stream(stream, topic, CompressedImage, self._jpeg_decode_loop)

        add_stream("head_depth", _HEAD_DEPTH_TOPIC, RosImage, self._convert_loop, Image)
        add_stream("lidar", _LIDAR_TOPIC, RosPointCloud2, self._convert_loop, PointCloud2)

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

        logger.info(
            f"R1Pro sensor streams up: {len(_COLOR_CAMERAS)} color cams + head_depth "
            f"+ lidar + 2 imus, wrist depth "
            f"{'on' if self.config.enable_wrist_depth else 'off'} (isolated DDS participant)"
        )

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

    # Control input handlers

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_MOTORS:
            logger.warning(
                f"Expected {_NUM_MOTORS} motor commands, got {msg.num_joints}; ignoring"
            )
            return

        from sensor_msgs.msg import JointState as RosJointState

        ros = self._ros
        if ros is None:
            return  # pre-start / post-stop
        stamp = _ros_stamp_now()

        for topic, sl in (
            (self._cmd_torso_topic, _TORSO_SLICE),
            (self._cmd_left_topic, _LEFT_SLICE),
            (self._cmd_right_topic, _RIGHT_SLICE),
        ):
            if topic is None:
                continue
            cmd = RosJointState()
            cmd.header.stamp = stamp
            cmd.name = [""]
            cmd.position = list(msg.q[sl])
            cmd.velocity = self._tracking_velocities(msg.dq[sl])
            cmd.effort = [0.0]
            ros.publish(topic, cmd)

    def _tracking_velocities(self, dqs: list[float]) -> list[float]:
        """Map MotorCommand.dq to ROS tracking velocity (0/sentinel → configured)."""
        speed = self.config.tracking_speed
        return [speed if (v == 0.0 or v == VEL_STOP) else float(v) for v in dqs]

    def _on_cmd_vel(self, msg: Twist) -> None:
        from geometry_msgs.msg import TwistStamped

        ros = self._ros
        if ros is None or self._speed_topic is None:
            return

        cmd = TwistStamped()
        cmd.header.stamp = _ros_stamp_now()
        cmd.twist.linear.x = msg.linear.x
        cmd.twist.linear.y = msg.linear.y
        cmd.twist.angular.z = msg.angular.z
        ros.publish(self._speed_topic, cmd)

    # Control feedback callbacks (3 segments)

    def _on_feedback_torso(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(msg, self._latest_torso_q, self._latest_torso_dq, self._latest_torso_eff)
            self._torso_seen = True

    def _on_feedback_left(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(msg, self._latest_left_q, self._latest_left_dq, self._latest_left_eff)
            self._left_seen = True

    def _on_feedback_right(self, msg: Any, _topic: Any) -> None:
        with self._lock:
            self._copy_segment(msg, self._latest_right_q, self._latest_right_dq, self._latest_right_eff)
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

    # Wheel odometry (integrated from executed chassis speed)

    def _on_chassis_speed(self, msg: Any, _topic: Any) -> None:
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._odom_last_ts is None:
            self._odom_last_ts = now
            return
        dt = now - self._odom_last_ts
        self._odom_last_ts = now
        if dt <= 0.0 or dt > 1.0:
            # Clock jump or first-tick anomaly — keep position, refresh ts.
            return
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z
        cy, sy = math.cos(self._odom_yaw), math.sin(self._odom_yaw)
        self._odom_x += (cy * vx - sy * vy) * dt
        self._odom_y += (sy * vx + cy * vy) * dt
        self._odom_yaw += wz * dt

        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        half = self._odom_yaw * 0.5
        position = Vector3(self._odom_x, self._odom_y, 0.0)
        orientation = Quaternion(0.0, 0.0, math.sin(half), math.cos(half))
        frame_id = self.config.odom_frame_id
        self.odom.publish(
            PoseStamped(ts=now, frame_id=frame_id, position=position, orientation=orientation)
        )
        self.odometry.publish(
            Odometry(
                ts=now,
                frame_id=frame_id,
                child_frame_id=self.config.frame_id,
                pose=Pose(position, orientation),
                twist=Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz)),
            )
        )

    # Aggregated motor_states publish loop

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

            if bootstrapped:
                now = time.time()
                self.motor_states.publish(
                    JointState(
                        ts=now,
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

    # Decode workers — convert ROS messages off the spin thread and publish

    def _jpeg_decode_loop(self, stream: str, q: queue.Queue[Any]) -> None:
        import cv2
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import ImageFormat

        out: Out[Image] = getattr(self, stream)
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
            # Rate gate BEFORE the decode — dropping here is ~free.
            if min_period > 0.0 and t0 - last_pub < min_period:
                continue
            last_pub = t0
            try:
                arr = np.frombuffer(bytes(msg.data), np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=False)
                    continue
                out.publish(Image(bgr, format=ImageFormat.BGR, frame_id=stream))
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=True)
            except Exception:
                self._record_decode(stream, (time.perf_counter() - t0) * 1e3, ok=False)
                logger.exception(f"R1Pro {stream} decode error")

    def _convert_loop(self, stream: str, q: queue.Queue[Any], dimos_type: type) -> None:
        """ros_to_dimos passthrough worker (depth images, lidar)."""
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


__all__ = [
    "R1PRO_UPPER_BODY_JOINTS",
    "R1ProConnection",
    "R1ProConnectionConfig",
]
