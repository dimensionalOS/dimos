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

"""Galaxea R1 Pro DDS Module: ROS 2 control + sensor streams.

Owns *all* ROS 2 traffic for the R1 Pro: the control RawROS node (joint
commands + chassis Twist + feedback) and an isolated rclpy Context for
sensor subscriptions (cameras / LiDAR / IMUs). Exposes streams the
coordinator-side ``transport_lcm`` ``WholeBody`` and ``Twist`` adapters
bridge to.

Joint layout (18 motors): torso 0-3, left arm 4-10, right arm 11-17.
The on-robot joint tracker manages PD gains internally; ``MotorCommand.
{kp,kd,tau}`` are intentionally ignored. Only ``q`` (position) and
``dq`` (tracking velocity, with sentinel/0 → ``config.tracking_speed``)
are forwarded.

ROS environment (``ROS_DOMAIN_ID``, ``RMW_IMPLEMENTATION``, peer config)
is expected to come from the docker container that runs this Module —
no Python-side env munging here.
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

# Head stereo pair: stream name → ROS topic.
_HEAD_CAMERAS: dict[str, str] = {
    "head_left_color":     "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "head_right_color":    "/hdas/camera_head/right_raw/image_raw_color/compressed",
}

# Off by default (config.enable_chassis_cameras): streaming several surround
# cams concurrently melts the robot's CSI/VI capture subsystem (see
# scripts/r1pro_test/STREAM_RELIABILITY_DIAGNOSTIC.md), and the next chassis
# revision has no surround cameras.
_CHASSIS_CAMERAS: dict[str, str] = {
    "chassis_front_left":  "/hdas/camera_chassis_front_left/rgb/compressed",
    "chassis_front_right": "/hdas/camera_chassis_front_right/rgb/compressed",
    "chassis_left":        "/hdas/camera_chassis_left/rgb/compressed",
    "chassis_right":       "/hdas/camera_chassis_right/rgb/compressed",
    "chassis_rear":        "/hdas/camera_chassis_rear/rgb/compressed",
}


@dataclass
class _StreamStat:
    """Cheap per-stream counters for the sensor pipeline.

    received/dropped are recorded in the DDS callback (wire delivery + size-1
    queue backpressure); decoded/errors/decode_ms in the decode worker. A
    growing ``dropped`` means decode/publish can't keep up; a stale ``last_mono``
    means the wire/robot stopped delivering. See ``sensor_stats`` rpc.
    """

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


class R1ProConnectionConfig(ModuleConfig):
    publish_rate_hz: float = Field(default=100.0)
    # rad/s used when MotorCommand.dq is the VEL_STOP sentinel or 0 (which is
    # what ConnectedWholeBody sends every tick).
    tracking_speed: float = Field(default=0.5)
    publish_odom: bool = Field(default=True)
    acc_limit_x: float = Field(default=2.5)
    acc_limit_y: float = Field(default=1.0)
    acc_limit_yaw: float = Field(default=1.0)
    frame_id: str = Field(default="r1pro_base_link")
    # Seconds between per-stream sensor-stats log lines (received/dropped/
    # decoded/decode-ms per camera/lidar/imu). 0 disables the periodic log;
    # the ``sensor_stats`` rpc is always available regardless.
    sensor_stats_interval_s: float = Field(default=10.0)
    # Subscribe to the 5 chassis surround cameras. Off by default — see the
    # _CHASSIS_CAMERAS comment (CSI capture meltdown; absent on new chassis).
    enable_chassis_cameras: bool = Field(default=False)
    # Subscribe to the aligned wrist depth streams. Off by default: raw 16-bit
    # depth is ~1.5 MB/frame at up to 30 Hz PER WRIST — when the wrist cams
    # came back healthy (2026-07-17) these streams alone re-saturated the
    # on-robot CPU and the rerun link that the visual_override triage had
    # bounded. Flip on for manipulation work that needs wrist depth; until
    # Tier-2 passthrough lands the budget doesn't fit otherwise.
    enable_wrist_depth: bool = Field(default=False)
    # Max Hz at which decoded color frames are published per camera (0 = no
    # cap). The rerun bridge shows at most 5 Hz (its max_hz cap) but must
    # JPEG-decode every LCM frame BEFORE that cap applies; decoding 8-14 Hz
    # x4 cams on both the connection and the bridge just to discard most of
    # it starved the bridge and viewer lag grew unbounded (2026-07-17). The
    # gate here runs BEFORE cv2.imdecode, so skipped frames cost ~nothing on
    # either end. Raise only after Tier-2 passthrough.
    color_publish_hz: float = Field(default=5.0)


class R1ProConnection(Module):
    """R1 Pro Module — owns the ROS 2 control node + isolated sensor context."""

    config: R1ProConnectionConfig

    # Control inputs.
    motor_command: In[MotorCommandArray]
    cmd_vel: In[Twist]

    # Whole-body feedback.
    motor_states: Out[JointState]
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]

    # Base feedback.
    odom: Out[PoseStamped]

    # Chassis perception.
    head_left_color: Out[Image]
    head_right_color: Out[Image]
    head_depth: Out[Image]
    chassis_front_left: Out[Image]
    chassis_front_right: Out[Image]
    chassis_left: Out[Image]
    chassis_right: Out[Image]
    chassis_rear: Out[Image]
    lidar: Out[PointCloud2]

    # Wrist perception.
    wrist_left_color: Out[Image]
    wrist_left_depth: Out[Image]
    wrist_right_color: Out[Image]
    wrist_right_depth: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Control RawROS handles.
        self._ros: RawROS | None = None
        self._cmd_torso_topic: RawROSTopic | None = None
        self._cmd_left_topic: RawROSTopic | None = None
        self._cmd_right_topic: RawROSTopic | None = None
        self._fb_torso_topic: RawROSTopic | None = None
        self._fb_left_topic: RawROSTopic | None = None
        self._fb_right_topic: RawROSTopic | None = None
        self._speed_topic: RawROSTopic | None = None
        self._acc_topic: RawROSTopic | None = None
        self._brake_topic: RawROSTopic | None = None
        self._chassis_speed_topic: RawROSTopic | None = None
        self._control_unsubs: list[Any] = []

        # Lock guards every _latest_* / _*_seen field across DDS, control
        # subscription threads, and the publish loop.
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

        # Odom dead-reckoning (driven by chassis_speed Gate 1 callback).
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_ts: float | None = None

        # Sensor isolated context.
        self._sensor_context: Any = None
        self._sensor_node: Any = None
        self._sensor_executor: Any = None
        self._sensor_spin_thread: Thread | None = None
        self._sensor_stop = threading.Event()
        self._sensor_workers: list[Thread] = []
        # Per-stream diagnostics (received/dropped/decoded/decode-ms), guarded by
        # its own lock so the stats path never contends with the control _lock.
        self._stats: dict[str, _StreamStat] = {}
        self._stats_lock = threading.Lock()
        # Per-stream queues for off-spin-thread decode.
        self._cam_queues: dict[str, queue.Queue[Any]] = {}
        self._head_depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._lidar_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._imu_chassis_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._imu_torso_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._wrist_left_color_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._wrist_left_depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._wrist_right_color_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._wrist_right_depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)

        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()

        # Lazy import — RawROS pulls rclpy which must not load on import in
        # environments without ROS 2.
        from dimos.protocol.pubsub.impl.rospubsub import RawROS

        logger.info("Starting R1ProConnection control RawROS...")
        self._ros = RawROS(node_name="r1pro_control")
        self._ros.start()

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

        # Sensor teardown first — its callbacks reference the isolated context
        # and will hot-loop on a shut-down node otherwise.
        if self._sensor_executor is not None:
            try:
                self._sensor_executor.shutdown(timeout_sec=1.0)
            except (OSError, RuntimeError) as e:
                logger.warning(f"sensor executor shutdown raised: {e}")
            self._sensor_executor = None
        if self._sensor_spin_thread is not None and self._sensor_spin_thread.is_alive():
            self._sensor_spin_thread.join(timeout=2.0)
            self._sensor_spin_thread = None
        # Unblock decode workers.
        all_qs: list[queue.Queue[Any]] = [
            *self._cam_queues.values(),
            self._head_depth_q,
            self._lidar_q,
            self._imu_chassis_q,
            self._imu_torso_q,
            self._wrist_left_color_q,
            self._wrist_left_depth_q,
            self._wrist_right_color_q,
            self._wrist_right_depth_q,
        ]
        for q in all_qs:
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for t in self._sensor_workers:
            t.join(timeout=1.0)
        self._sensor_workers.clear()
        self._cam_queues.clear()
        if self._sensor_node is not None:
            try:
                self._sensor_node.destroy_node()
            except (OSError, RuntimeError) as e:
                logger.warning(f"sensor node destroy raised: {e}")
            self._sensor_node = None
        if self._sensor_context is not None:
            try:
                import rclpy

                rclpy.shutdown(context=self._sensor_context)
            except (OSError, RuntimeError) as e:
                logger.warning(f"rclpy.shutdown(sensor_context) raised: {e}")
            self._sensor_context = None

        # Control unsubs + RawROS.
        for unsub in self._control_unsubs:
            try:
                unsub()
            except (OSError, RuntimeError) as e:
                logger.warning(f"control unsubscribe raised: {e}")
        self._control_unsubs.clear()
        if self._ros is not None:
            try:
                self._ros.stop()
            except (OSError, RuntimeError) as e:
                logger.warning(f"RawROS stop raised: {e}")
        self._ros = None

        with self._lock:
            self._torso_seen = self._left_seen = self._right_seen = False
            self._latest_imu_chassis = None
            self._latest_imu_torso = None

        logger.info("R1ProConnection stopped")
        super().stop()

    # Control RawROS setup

    def _setup_control_topics(self) -> None:
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import JointState as RosJointState
        from std_msgs.msg import Bool

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
        self._fb_torso_topic = RawROSTopic(
            "/hdas/feedback_torso", RosJointState, qos=qos
        )
        self._fb_left_topic = RawROSTopic(
            "/hdas/feedback_arm_left", RosJointState, qos=qos
        )
        self._fb_right_topic = RawROSTopic(
            "/hdas/feedback_arm_right", RosJointState, qos=qos
        )
        self._speed_topic = RawROSTopic(
            "/motion_target/target_speed_chassis", TwistStamped, qos=qos
        )
        self._acc_topic = RawROSTopic(
            "/motion_target/chassis_acc_limit", TwistStamped, qos=qos
        )
        self._brake_topic = RawROSTopic(
            "/motion_target/brake_mode", Bool, qos=qos
        )
        self._chassis_speed_topic = RawROSTopic(
            "/motion_control/chassis_speed", TwistStamped, qos=qos
        )

        self._control_unsubs.append(
            self._ros.subscribe(self._fb_torso_topic, self._on_feedback_torso)
        )
        self._control_unsubs.append(
            self._ros.subscribe(self._fb_left_topic, self._on_feedback_left)
        )
        self._control_unsubs.append(
            self._ros.subscribe(self._fb_right_topic, self._on_feedback_right)
        )
        # Gate 1 — chassis_control_node only runs IK if someone subscribes to
        # this topic. Also drives odom integration.
        self._control_unsubs.append(
            self._ros.subscribe(self._chassis_speed_topic, self._on_chassis_speed)
        )

    # Sensor isolated-context setup

    def _setup_sensor_streams(self) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node as RclpyNode

        try:
            from sensor_msgs.msg import CompressedImage
            from sensor_msgs.msg import Image as RosImage
            from sensor_msgs.msg import Imu as RosImu
            from sensor_msgs.msg import PointCloud2 as RosPointCloud2
        except ImportError:
            logger.warning("sensor_msgs not available — sensor streams disabled")
            return

        qos = _make_qos()

        # Isolated DDS participant: control traffic at 100 Hz must not saturate
        # the DDS receive threads handling fragmented camera UDP.
        self._sensor_context = Context()
        rclpy.init(context=self._sensor_context)
        self._sensor_node = RclpyNode("r1pro_sensors", context=self._sensor_context)
        # 4 threads is empirically sufficient on Jetson Orin; bumping doesn't
        # add throughput, just contention.
        self._sensor_executor = MultiThreadedExecutor(
            num_threads=4, context=self._sensor_context
        )
        self._sensor_executor.add_node(self._sensor_node)

        # JPEG color cameras (head + optional chassis surround) → one decode
        # worker each.
        jpeg_cameras = dict(_HEAD_CAMERAS)
        if self.config.enable_chassis_cameras:
            jpeg_cameras.update(_CHASSIS_CAMERAS)
        for stream_name, ros_topic in jpeg_cameras.items():
            cam_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._cam_queues[stream_name] = cam_q
            self._sensor_node.create_subscription(
                CompressedImage,
                ros_topic,
                self._make_rx_cb(stream_name, cam_q),
                qos,
            )
            self._sensor_workers.append(
                Thread(
                    target=self._jpeg_camera_decode_loop,
                    args=(stream_name, cam_q),
                    daemon=True,
                    name=f"r1pro-{stream_name}",
                )
            )

        # Head depth (32FC1).
        self._sensor_node.create_subscription(
            RosImage,
            "/hdas/camera_head/depth/depth_registered",
            self._make_rx_cb("head_depth", self._head_depth_q),
            qos,
        )
        self._sensor_workers.append(
            Thread(target=self._head_depth_decode_loop, daemon=True, name="r1pro-head_depth")
        )

        # LiDAR.
        self._sensor_node.create_subscription(
            RosPointCloud2,
            "/hdas/lidar_chassis_left",
            self._make_rx_cb("lidar", self._lidar_q),
            qos,
        )
        self._sensor_workers.append(
            Thread(target=self._lidar_decode_loop, daemon=True, name="r1pro-lidar")
        )

        # IMUs.
        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_chassis",
            self._make_rx_cb("imu_chassis", self._imu_chassis_q),
            qos,
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_chassis_q, "imu_chassis"),
                daemon=True,
                name="r1pro-imu_chassis",
            )
        )
        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_torso",
            self._make_rx_cb("imu_torso", self._imu_torso_q),
            qos,
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_torso_q, "imu_torso"),
                daemon=True,
                name="r1pro-imu_torso",
            )
        )

        # Wrist cameras (left + right, color + depth).
        for side, color_q, depth_q in (
            ("left", self._wrist_left_color_q, self._wrist_left_depth_q),
            ("right", self._wrist_right_color_q, self._wrist_right_depth_q),
        ):
            self._sensor_node.create_subscription(
                CompressedImage,
                f"/hdas/camera_wrist_{side}/color/image_raw/compressed",
                self._make_rx_cb(f"wrist_{side}_color", color_q),
                qos,
            )
            if self.config.enable_wrist_depth:
                self._sensor_node.create_subscription(
                    RosImage,
                    f"/hdas/camera_wrist_{side}/aligned_depth_to_color/image_raw",
                    self._make_rx_cb(f"wrist_{side}_depth", depth_q),
                    qos,
                )
            self._sensor_workers.append(
                Thread(
                    target=self._wrist_color_decode_loop,
                    args=(side, color_q),
                    daemon=True,
                    name=f"r1pro-wrist_{side}_color",
                )
            )
            if self.config.enable_wrist_depth:
                self._sensor_workers.append(
                    Thread(
                        target=self._wrist_depth_decode_loop,
                        args=(side, depth_q),
                        daemon=True,
                        name=f"r1pro-wrist_{side}_depth",
                    )
                )

        # Periodic per-stream stats reporter (joined on stop via _sensor_stop).
        if self.config.sensor_stats_interval_s > 0:
            self._sensor_workers.append(
                Thread(target=self._stats_report_loop, daemon=True, name="r1pro-sensor-stats")
            )

        for t in self._sensor_workers:
            t.start()

        self._sensor_spin_thread = Thread(
            target=self._sensor_spin, daemon=True, name="r1pro-sensor-spin"
        )
        self._sensor_spin_thread.start()

        logger.info(
            f"R1Pro sensor streams up: {len(jpeg_cameras)} color cams "
            f"(chassis surround {'on' if self.config.enable_chassis_cameras else 'off'}) "
            "+ head_depth + lidar + 2 imus + "
            f"{'4' if self.config.enable_wrist_depth else '2'} wrist streams "
            f"(depth {'on' if self.config.enable_wrist_depth else 'off'}, isolated DDS)"
        )

    def _sensor_spin(self) -> None:
        # spin_once-with-recovery pattern: a callback exception must not kill
        # the whole sensor thread, and shutdown must drop us out promptly.
        executor = self._sensor_executor
        ctx = self._sensor_context
        if executor is None or ctx is None:
            return
        while not self._sensor_stop.is_set() and ctx.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001 — we genuinely want to log-and-continue
                if not ctx.ok() or "context is not valid" in str(exc):
                    logger.warning(f"Sensor context invalid, exiting spin: {exc}")
                    break
                logger.warning(f"sensor spin_once raised (continuing): {exc}", exc_info=True)

    # Per-stream diagnostics

    def _make_rx_cb(self, stream: str, q: queue.Queue[Any]) -> Any:
        """Subscription callback: record receive stats + latest-wins enqueue.

        Records on the DDS spin thread (kept trivial); the actual decode happens
        on the worker. ``dropped`` counts size-1 queue evictions = the consumer
        falling behind the wire.
        """
        def cb(msg: Any) -> None:
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
        if interval <= 0:
            return
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
        avg decode ms, and last-message age. Cheap snapshot for monitoring and
        regression detection (no re-profiling needed)."""
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
        from std_msgs.msg import Bool

        with self._lock:
            if self._ros is None:
                return  # pre-start / post-stop
            stamp = self._ros._node.get_clock().now().to_msg()  # type: ignore[union-attr]

            torso_q = list(msg.q[_TORSO_SLICE])
            left_q = list(msg.q[_LEFT_SLICE])
            right_q = list(msg.q[_RIGHT_SLICE])
            torso_dq = self._tracking_velocities(msg.dq[_TORSO_SLICE])
            left_dq = self._tracking_velocities(msg.dq[_LEFT_SLICE])
            right_dq = self._tracking_velocities(msg.dq[_RIGHT_SLICE])

            for topic, qs, dqs in (
                (self._cmd_torso_topic, torso_q, torso_dq),
                (self._cmd_left_topic, left_q, left_dq),
                (self._cmd_right_topic, right_q, right_dq),
            ):
                if topic is None:
                    continue
                cmd = RosJointState()
                cmd.header.stamp = stamp
                cmd.name = [""]
                cmd.position = qs
                cmd.velocity = dqs
                cmd.effort = [0.0]
                self._ros.publish(topic, cmd)

            if self._brake_topic is not None:
                self._ros.publish(self._brake_topic, Bool(data=False))

    def _tracking_velocities(self, dqs: list[float]) -> list[float]:
        """Map MotorCommand.dq to ROS tracking velocity.

        ConnectedWholeBody.write_command always sends dq=0.0 (not VEL_STOP),
        so 0.0 must also collapse to the configured tracking speed.
        """
        speed = self.config.tracking_speed
        return [speed if (v == 0.0 or v == VEL_STOP) else float(v) for v in dqs]

    def _on_cmd_vel(self, msg: Twist) -> None:
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Bool

        with self._lock:
            if self._ros is None or self._acc_topic is None or self._speed_topic is None:
                return
            stamp = self._ros._node.get_clock().now().to_msg()  # type: ignore[union-attr]

            acc = TwistStamped()
            acc.header.stamp = stamp
            acc.twist.linear.x = self.config.acc_limit_x
            acc.twist.linear.y = self.config.acc_limit_y
            acc.twist.angular.z = self.config.acc_limit_yaw
            self._ros.publish(self._acc_topic, acc)

            if self._brake_topic is not None:
                self._ros.publish(self._brake_topic, Bool(data=False))

            cmd = TwistStamped()
            cmd.header.stamp = stamp
            cmd.twist.linear.x = msg.linear.x
            cmd.twist.linear.y = msg.linear.y
            cmd.twist.angular.z = msg.angular.z
            self._ros.publish(self._speed_topic, cmd)

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

    # Chassis Gate 1 + odom integration

    def _on_chassis_speed(self, msg: Any, _topic: Any) -> None:
        if not self.config.publish_odom:
            return
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

        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        half = self._odom_yaw * 0.5
        pose = PoseStamped(
            ts=now,
            frame_id="odom",
            position=Vector3(self._odom_x, self._odom_y, 0.0),
            orientation=Quaternion(0.0, 0.0, math.sin(half), math.cos(half)),
        )
        self.odom.publish(pose)

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
                        # Skip publishes until every segment has reported in
                        # once; otherwise TransportWholeBodyAdapter latches a
                        # zero-position snapshot and a position-mode tick walks
                        # the arms to home.
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

    def _jpeg_camera_decode_loop(self, stream_name: str, q: queue.Queue[Any]) -> None:
        import cv2
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import ImageFormat

        out = getattr(self, stream_name)
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
                    self._record_decode(stream_name, (time.perf_counter() - t0) * 1e3, ok=False)
                    continue
                out.publish(Image(bgr, format=ImageFormat.BGR, frame_id=stream_name))
                self._record_decode(stream_name, (time.perf_counter() - t0) * 1e3, ok=True)
            except Exception:
                self._record_decode(stream_name, (time.perf_counter() - t0) * 1e3, ok=False)
                logger.exception(f"R1Pro camera {stream_name} decode error")

    def _head_depth_decode_loop(self) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._head_depth_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                self.head_depth.publish(ros_to_dimos(msg, Image))
            except Exception:
                logger.exception("R1Pro head_depth decode error")

    def _lidar_decode_loop(self) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._lidar_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                self.lidar.publish(ros_to_dimos(msg, PointCloud2))
            except Exception:
                logger.exception("R1Pro lidar decode error")

    def _imu_decode_loop(self, q: queue.Queue[Any], which: str) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        target_attr = "_latest_imu_chassis" if which == "imu_chassis" else "_latest_imu_torso"
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
                logger.exception(f"R1Pro {which} decode error")

    def _wrist_color_decode_loop(self, side: str, q: queue.Queue[Any]) -> None:
        import cv2
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import ImageFormat

        out: Out[Image] = self.wrist_left_color if side == "left" else self.wrist_right_color
        frame_id = f"wrist_{side}_color"
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
                    self._record_decode(frame_id, (time.perf_counter() - t0) * 1e3, ok=False)
                    continue
                out.publish(Image(bgr, format=ImageFormat.BGR, frame_id=frame_id))
                self._record_decode(frame_id, (time.perf_counter() - t0) * 1e3, ok=True)
            except Exception:
                self._record_decode(frame_id, (time.perf_counter() - t0) * 1e3, ok=False)
                logger.exception(f"R1Pro wrist_{side} color decode error")

    def _wrist_depth_decode_loop(self, side: str, q: queue.Queue[Any]) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        out: Out[Image] = self.wrist_left_depth if side == "left" else self.wrist_right_depth
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                out.publish(ros_to_dimos(msg, Image))
            except Exception:
                logger.exception(f"R1Pro wrist_{side} depth decode error")


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
