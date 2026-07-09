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

"""Galaxea R1 Lite DDS Module: ROS 2 control + sensor streams.

Sibling of ``r1pro/connection.py`` (same architecture: one Module owns all
ROS 2 traffic — a control RawROS node plus an isolated rclpy Context for
camera subscriptions), adapted to the R1 Lite as hardware-validated on
2026-07-02..09 (see ``scripts/r1lite_test/BRINGUP_LOG.md``).

Differences vs the R1 Pro module, each backed by hardware findings:

* Joint layout (16 motors): torso 0-3, left arm 4-9, right arm 10-15.
  Arms are 6-DOF (A1X). Torso feedback carries 4 motor values but the
  URDF models only 3 joints (parallelogram lift).
* **Torso commands are dropped** (feedback still published). The torso is
  a parallelogram linkage — joint-space targets are only valid as
  linkage-consistent tuples, and a single-joint delta physically shook
  the robot. Torso motion belongs to Galaxea's task-space MPC path
  (``/motion_target/target_speed_torso``), not wired up in v1.
* **Grippers are first-class** (the R1 Pro module ignored them):
  one command/feedback stream per side, positions in Galaxea's native
  0-100 scale (0=closed, 100=open ≈ stroke %).
* **Chassis commands are published at a fixed rate with a dead-man**:
  the R1 Lite chassis node LATCHES the last target forever (no staleness
  timeout robot-side), so this module streams speed + acc_limit +
  brake=false every tick and collapses to an explicit zero-velocity
  stream when ``cmd_vel`` goes stale. Acceleration limits default to the
  hardware-validated gentle 0.5 values.
* Chassis software control additionally requires, outside this module:
  robot cold-booted with e-stop released (a latched e-stop poisons the
  VCU for the whole power session) and the RC ON with all switches in
  position 1 (``/controller`` mode 5 = software may drive).
* Sensor set: stereo head pair (compressed RGB, no depth topic on this
  robot), D405 wrist cameras (compressed color + aligned depth), chassis
  and torso IMUs. No lidar, no chassis cameras.

The on-robot jointTracker manages gains internally; ``MotorCommand.
{kp,kd,tau}`` are ignored. Only ``q`` and ``dq`` (with sentinel/0 →
``config.tracking_speed``) are forwarded, exactly like the R1 Pro.

ROS environment (``ROS_DOMAIN_ID=2``, multicast) is expected from the
container/launch environment — no Python-side env munging here.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from threading import Thread
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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Joint layout — flat 16-element MotorCommandArray indexing.
_TORSO_SLICE = slice(0, 4)
_LEFT_SLICE = slice(4, 10)
_RIGHT_SLICE = slice(10, 16)
_NUM_MOTORS = 16
_ARM_DOF = 6

_FEEDBACK_DISCOVERY_TIMEOUT_S = 5.0

# Motor-space names. torso_joint1-3 match the URDF; torso_joint4 is the
# fourth HDAS motor value the URDF's 3-joint linkage model doesn't expose.
_R1LITE_UPPER_BODY_BARE: list[str] = (
    [f"torso_joint{i}" for i in range(1, 5)]
    + [f"left_arm_joint{i}" for i in range(1, 7)]
    + [f"right_arm_joint{i}" for i in range(1, 7)]
)
R1LITE_UPPER_BODY_JOINTS: list[str] = [f"r1lite/{j}" for j in _R1LITE_UPPER_BODY_BARE]
assert len(R1LITE_UPPER_BODY_JOINTS) == _NUM_MOTORS

R1LITE_GRIPPER_OPEN = 100.0
R1LITE_GRIPPER_CLOSED = 0.0

# Cameras: stream name → (ROS topic, compressed?). The head is a pure
# stereo RGB pair; wrist color is compressed, wrist depth is raw 16UC1.
_COMPRESSED_CAMERAS: dict[str, str] = {
    "head_left_color":  "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "head_right_color": "/hdas/camera_head/right_raw/image_raw_color/compressed",
    "wrist_left_color":  "/hdas/camera_wrist_left/color/image_raw/compressed",
    "wrist_right_color": "/hdas/camera_wrist_right/color/image_raw/compressed",
}
_DEPTH_CAMERAS: dict[str, str] = {
    "wrist_left_depth":  "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
    "wrist_right_depth": "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
}


def _make_qos() -> Any:
    """BEST_EFFORT + VOLATILE — for SUBSCRIPTIONS (robot publishes best-effort)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _make_cmd_qos() -> Any:
    """RELIABLE + VOLATILE — for command PUBLISHERS.

    A reliable publisher delivers to both reliable and best-effort
    subscribers; a best-effort publisher is DDS-incompatible with reliable
    subscribers ("No messages will be sent") — and the R1 Lite has at
    least one RELIABLE subscriber on target_speed_chassis (observed
    2026-07-09 during keyboard-teleop run 3).
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class R1LiteConnectionConfig(ModuleConfig):
    publish_rate_hz: float = Field(default=100.0)
    # rad/s used when MotorCommand.dq is the VEL_STOP sentinel or 0 (which
    # is what ConnectedWholeBody sends every tick). 0.5 validated on hw.
    tracking_speed: float = Field(default=0.5)
    publish_odom: bool = Field(default=True)
    # Hardware-validated gentle limits (R1 Pro used 2.5/1.0/1.0).
    acc_limit_x: float = Field(default=0.5)
    acc_limit_y: float = Field(default=0.5)
    acc_limit_yaw: float = Field(default=0.5)
    # cmd_vel older than this streams zeros instead (dead-man). The robot's
    # chassis node latches its last target forever, so the timeout must
    # live here.
    cmd_vel_timeout_s: float = Field(default=0.3)
    frame_id: str = Field(default="r1lite_base_link")


class R1LiteConnection(Module):
    """R1 Lite Module — owns the ROS 2 control node + isolated sensor context."""

    config: R1LiteConnectionConfig

    # Control inputs.
    motor_command: In[MotorCommandArray]  # 16 joints; torso slice ignored (v1)
    cmd_vel: In[Twist]
    gripper_left_command: In[JointState]   # position[0] in 0-100
    gripper_right_command: In[JointState]  # position[0] in 0-100

    # Whole-body feedback.
    motor_states: Out[JointState]  # 16 joints @ publish_rate_hz
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]
    gripper_left_state: Out[JointState]
    gripper_right_state: Out[JointState]

    # Base feedback.
    odom: Out[PoseStamped]

    # Perception.
    head_left_color: Out[Image]
    head_right_color: Out[Image]
    wrist_left_color: Out[Image]
    wrist_left_depth: Out[Image]
    wrist_right_color: Out[Image]
    wrist_right_depth: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Control RawROS handles.
        self._ros: RawROS | None = None
        self._cmd_left_topic: RawROSTopic | None = None
        self._cmd_right_topic: RawROSTopic | None = None
        self._cmd_gripper_left_topic: RawROSTopic | None = None
        self._cmd_gripper_right_topic: RawROSTopic | None = None
        self._fb_torso_topic: RawROSTopic | None = None
        self._fb_left_topic: RawROSTopic | None = None
        self._fb_right_topic: RawROSTopic | None = None
        self._fb_gripper_left_topic: RawROSTopic | None = None
        self._fb_gripper_right_topic: RawROSTopic | None = None
        self._speed_topic: RawROSTopic | None = None
        self._acc_topic: RawROSTopic | None = None
        self._brake_topic: RawROSTopic | None = None
        self._chassis_speed_topic: RawROSTopic | None = None
        self._control_unsubs: list[Any] = []

        # Lock guards every _latest_* / _*_seen field across DDS callbacks
        # and the publish loop.
        self._lock = threading.Lock()
        self._latest_torso_q: list[float] = [0.0] * 4
        self._latest_torso_dq: list[float] = [0.0] * 4
        self._latest_torso_eff: list[float] = [0.0] * 4
        self._latest_left_q: list[float] = [0.0] * _ARM_DOF
        self._latest_left_dq: list[float] = [0.0] * _ARM_DOF
        self._latest_left_eff: list[float] = [0.0] * _ARM_DOF
        self._latest_right_q: list[float] = [0.0] * _ARM_DOF
        self._latest_right_dq: list[float] = [0.0] * _ARM_DOF
        self._latest_right_eff: list[float] = [0.0] * _ARM_DOF
        self._torso_seen = False
        self._left_seen = False
        self._right_seen = False
        self._latest_imu_chassis: Imu | None = None
        self._latest_imu_torso: Imu | None = None
        self._torso_cmd_warned = False

        # Chassis dead-man state (see module docstring).
        self._latest_cmd_vel: Twist | None = None
        self._latest_cmd_vel_ts = 0.0
        self._cmd_vel_active = False  # True while we still owe zero-streams

        # Odom dead-reckoning (driven by the chassis_speed Gate-1 callback).
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
        self._cam_queues: dict[str, queue.Queue[Any]] = {}
        self._depth_queues: dict[str, queue.Queue[Any]] = {}
        self._imu_chassis_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._imu_torso_q: queue.Queue[Any] = queue.Queue(maxsize=4)

        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()

        # Lazy import — RawROS pulls rclpy which must not load on import in
        # environments without ROS 2.
        from dimos.protocol.pubsub.impl.rospubsub import RawROS

        logger.info("Starting R1LiteConnection control RawROS...")
        self._ros = RawROS(node_name="r1lite_control")
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
        self.register_disposable(
            Disposable(
                self.gripper_left_command.subscribe(
                    lambda msg: self._on_gripper_command("left", msg)
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.gripper_right_command.subscribe(
                    lambda msg: self._on_gripper_command("right", msg)
                )
            )
        )

        self._publish_thread = Thread(
            target=self._publish_loop, name="r1lite-publish", daemon=True
        )
        self._publish_thread.start()

        logger.info("R1LiteConnection started")

    @rpc
    def stop(self) -> None:
        # Stop the publish loop FIRST so it can't race the ROS teardown,
        # then leave the chassis with an explicit zero target — the
        # robot-side node latches whatever it heard last.
        self._stop_event.set()
        self._sensor_stop.set()

        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        try:
            self._publish_chassis_command(Twist())
        except Exception:  # noqa: BLE001 — best-effort courtesy stop
            pass

        # Sensor teardown first — its callbacks reference the isolated
        # context and will hot-loop on a shut-down node otherwise.
        if self._sensor_executor is not None:
            try:
                self._sensor_executor.shutdown(timeout_sec=1.0)
            except (OSError, RuntimeError) as e:
                logger.warning(f"sensor executor shutdown raised: {e}")
            self._sensor_executor = None
        if self._sensor_spin_thread is not None and self._sensor_spin_thread.is_alive():
            self._sensor_spin_thread.join(timeout=2.0)
            self._sensor_spin_thread = None
        all_qs: list[queue.Queue[Any]] = [
            *self._cam_queues.values(),
            *self._depth_queues.values(),
            self._imu_chassis_q,
            self._imu_torso_q,
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
        self._depth_queues.clear()
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

        logger.info("R1LiteConnection stopped")
        super().stop()

    # Control RawROS setup

    def _setup_control_topics(self) -> None:
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import JointState as RosJointState
        from std_msgs.msg import Bool

        from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic

        assert self._ros is not None
        qos = _make_qos()
        cmd_qos = _make_cmd_qos()

        self._cmd_left_topic = RawROSTopic(
            "/motion_target/target_joint_state_arm_left", RosJointState, qos=cmd_qos
        )
        self._cmd_right_topic = RawROSTopic(
            "/motion_target/target_joint_state_arm_right", RosJointState, qos=cmd_qos
        )
        self._cmd_gripper_left_topic = RawROSTopic(
            "/motion_target/target_position_gripper_left", RosJointState, qos=cmd_qos
        )
        self._cmd_gripper_right_topic = RawROSTopic(
            "/motion_target/target_position_gripper_right", RosJointState, qos=cmd_qos
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
        self._fb_gripper_left_topic = RawROSTopic(
            "/hdas/feedback_gripper_left", RosJointState, qos=qos
        )
        self._fb_gripper_right_topic = RawROSTopic(
            "/hdas/feedback_gripper_right", RosJointState, qos=qos
        )
        self._speed_topic = RawROSTopic(
            "/motion_target/target_speed_chassis", TwistStamped, qos=cmd_qos
        )
        self._acc_topic = RawROSTopic(
            "/motion_target/chassis_acc_limit", TwistStamped, qos=cmd_qos
        )
        self._brake_topic = RawROSTopic(
            "/motion_target/brake_mode", Bool, qos=cmd_qos
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
        self._control_unsubs.append(
            self._ros.subscribe(
                self._fb_gripper_left_topic,
                lambda msg, _t: self._on_gripper_feedback("left", msg),
            )
        )
        self._control_unsubs.append(
            self._ros.subscribe(
                self._fb_gripper_right_topic,
                lambda msg, _t: self._on_gripper_feedback("right", msg),
            )
        )
        # Gate 1 — the chassis node only runs its control path if someone
        # subscribes to its measured-speed output. Also drives odom.
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
        except ImportError:
            logger.warning("sensor_msgs not available — sensor streams disabled")
            return

        qos = _make_qos()

        # Isolated DDS participant: control traffic at 100 Hz must not
        # contend with fragmented camera UDP (Mustafa's sensor-drop saga).
        self._sensor_context = Context()
        rclpy.init(context=self._sensor_context)
        self._sensor_node = RclpyNode("r1lite_sensors", context=self._sensor_context)
        self._sensor_executor = MultiThreadedExecutor(
            num_threads=4, context=self._sensor_context
        )
        self._sensor_executor.add_node(self._sensor_node)

        for stream_name, ros_topic in _COMPRESSED_CAMERAS.items():
            cam_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._cam_queues[stream_name] = cam_q
            self._sensor_node.create_subscription(
                CompressedImage,
                ros_topic,
                lambda msg, q=cam_q: _enqueue_drop_oldest(q, msg),
                qos,
            )
            self._sensor_workers.append(
                Thread(
                    target=self._compressed_decode_loop,
                    args=(stream_name, cam_q),
                    daemon=True,
                    name=f"r1lite-{stream_name}",
                )
            )

        for stream_name, ros_topic in _DEPTH_CAMERAS.items():
            depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._depth_queues[stream_name] = depth_q
            self._sensor_node.create_subscription(
                RosImage,
                ros_topic,
                lambda msg, q=depth_q: _enqueue_drop_oldest(q, msg),
                qos,
            )
            self._sensor_workers.append(
                Thread(
                    target=self._depth_decode_loop,
                    args=(stream_name, depth_q),
                    daemon=True,
                    name=f"r1lite-{stream_name}",
                )
            )

        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_chassis",
            lambda msg: _enqueue_drop_oldest(self._imu_chassis_q, msg),
            qos,
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_chassis_q, "imu_chassis"),
                daemon=True,
                name="r1lite-imu_chassis",
            )
        )
        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_torso",
            lambda msg: _enqueue_drop_oldest(self._imu_torso_q, msg),
            qos,
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_torso_q, "imu_torso"),
                daemon=True,
                name="r1lite-imu_torso",
            )
        )

        for t in self._sensor_workers:
            t.start()

        self._sensor_spin_thread = Thread(
            target=self._sensor_spin, daemon=True, name="r1lite-sensor-spin"
        )
        self._sensor_spin_thread.start()

        logger.info(
            "R1Lite sensor streams up: stereo head + 2 wrist RGBD + 2 imus (isolated DDS)"
        )

    def _sensor_spin(self) -> None:
        executor = self._sensor_executor
        ctx = self._sensor_context
        if executor is None or ctx is None:
            return
        while not self._sensor_stop.is_set() and ctx.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:  # noqa: BLE001 — log-and-continue by design
                if not ctx.ok() or "context is not valid" in str(exc):
                    logger.warning(f"Sensor context invalid, exiting spin: {exc}")
                    break
                logger.warning(f"sensor spin_once raised (continuing): {exc}", exc_info=True)

    # Control input handlers

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_MOTORS:
            logger.warning(
                f"Expected {_NUM_MOTORS} motor commands, got {msg.num_joints}; ignoring"
            )
            return

        from sensor_msgs.msg import JointState as RosJointState

        # Torso slice intentionally dropped: parallelogram linkage —
        # joint-space targets shook the robot (BRINGUP_LOG 2026-07-03).
        torso_q = list(msg.q[_TORSO_SLICE])
        if not self._torso_cmd_warned and any(abs(v) > 1e-6 for v in torso_q):
            self._torso_cmd_warned = True
            logger.warning(
                "R1LiteConnection: torso joint commands are not supported "
                "(linkage-coupled); torso slice is ignored. Use the task-space "
                "torso path when it lands."
            )

        with self._lock:
            if self._ros is None:
                return  # pre-start / post-stop
            stamp = self._ros._node.get_clock().now().to_msg()  # type: ignore[union-attr]

            left_q = list(msg.q[_LEFT_SLICE])
            right_q = list(msg.q[_RIGHT_SLICE])
            left_dq = self._tracking_velocities(msg.dq[_LEFT_SLICE])
            right_dq = self._tracking_velocities(msg.dq[_RIGHT_SLICE])

            for topic, qs, dqs in (
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

    def _tracking_velocities(self, dqs: list[float]) -> list[float]:
        """Map MotorCommand.dq to jointTracker tracking velocity.

        ConnectedWholeBody.write_command always sends dq=0.0 (not VEL_STOP),
        so 0.0 must also collapse to the configured tracking speed.
        """
        speed = self.config.tracking_speed
        return [speed if (v == 0.0 or v == VEL_STOP) else float(v) for v in dqs]

    def _on_gripper_command(self, side: str, msg: JointState) -> None:
        """Forward a 0-100 gripper position target (streamed by the caller).

        The gripper controller is a follow-the-stream tracker: it acts only
        while targets keep arriving, so callers must publish continuously
        (one-shots are ignored by the robot — hardware-verified).
        """
        if not msg.position:
            return
        target = float(msg.position[0])
        if not (R1LITE_GRIPPER_CLOSED <= target <= R1LITE_GRIPPER_OPEN + 5.0):
            logger.warning(
                f"gripper_{side} target {target} outside 0-100 range; ignoring"
            )
            return

        from sensor_msgs.msg import JointState as RosJointState

        topic = (
            self._cmd_gripper_left_topic if side == "left" else self._cmd_gripper_right_topic
        )
        with self._lock:
            if self._ros is None or topic is None:
                return
            cmd = RosJointState()
            cmd.header.stamp = self._ros._node.get_clock().now().to_msg()  # type: ignore[union-attr]
            cmd.name = [""]
            cmd.position = [target]
            cmd.velocity = [0.0]
            cmd.effort = [0.0]
            self._ros.publish(topic, cmd)

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Latch locally; the publish loop streams it (dead-man semantics —
        # see _publish_chassis_command).
        with self._lock:
            self._latest_cmd_vel = msg
            self._latest_cmd_vel_ts = time.monotonic()
            self._cmd_vel_active = True

    def _publish_chassis_command(self, twist: Twist) -> None:
        """One chassis tick: acc_limit + brake=false + speed target.

        All three every tick — the validated recipe. Gate 1 is held open by
        our _chassis_speed_topic subscription.
        """
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
            cmd.twist.linear.x = twist.linear.x
            cmd.twist.linear.y = twist.linear.y
            cmd.twist.angular.z = twist.angular.z
            self._ros.publish(self._speed_topic, cmd)

    # Control feedback callbacks

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

    def _on_gripper_feedback(self, side: str, msg: Any) -> None:
        if not msg.position:
            return
        out = self.gripper_left_state if side == "left" else self.gripper_right_state
        out.publish(
            JointState(
                ts=time.time(),
                frame_id=f"r1lite_gripper_{side}",
                name=[f"r1lite/{side}_gripper"],
                position=[float(msg.position[0])],
                velocity=[float(msg.velocity[0])] if msg.velocity else [],
                effort=[float(msg.effort[0])] if msg.effort else [],
            )
        )

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

    # Aggregated publish loop (motor_states + chassis dead-man)

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id
        bootstrapped = False
        zero = Twist()

        while not self._stop_event.is_set():
            with self._lock:
                if not bootstrapped:
                    if not (self._torso_seen and self._left_seen and self._right_seen):
                        # Gate first publish until every segment reports in;
                        # otherwise TransportWholeBodyAdapter latches a
                        # zero snapshot and a position tick jumps the arms.
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
                cmd_vel = self._latest_cmd_vel
                cmd_fresh = (
                    cmd_vel is not None
                    and (time.monotonic() - self._latest_cmd_vel_ts)
                    < self.config.cmd_vel_timeout_s
                )
                chassis_active = self._cmd_vel_active
                if not cmd_fresh and chassis_active:
                    # One stale transition: stream zeros from now on until
                    # a fresh cmd_vel arrives. Keeps the robot-side latch
                    # pointed at zero without spamming before first use.
                    self._latest_cmd_vel = None

            if bootstrapped:
                now = time.time()
                self.motor_states.publish(
                    JointState(
                        ts=now,
                        frame_id=frame_id,
                        name=R1LITE_UPPER_BODY_JOINTS,
                        position=positions,  # type: ignore[arg-type]
                        velocity=velocities,
                        effort=efforts,
                    )
                )
                if imu_chassis is not None:
                    self.imu_chassis.publish(imu_chassis)
                if imu_torso is not None:
                    self.imu_torso.publish(imu_torso)

            # Chassis: stream the fresh command, or zeros once cmd_vel has
            # ever been used (dead-man — robot side latches forever).
            if chassis_active and not self._stop_event.is_set():
                try:
                    self._publish_chassis_command(cmd_vel if cmd_fresh else zero)  # type: ignore[arg-type]
                except Exception as exc:  # noqa: BLE001 — teardown race: context can die first
                    logger.warning(f"chassis publish failed (shutting down?): {exc}")
                    break

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    # Sensor decode workers

    def _compressed_decode_loop(self, stream_name: str, q: queue.Queue[Any]) -> None:
        import cv2
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import ImageFormat

        out = getattr(self, stream_name)
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                arr = np.frombuffer(bytes(msg.data), np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                out.publish(Image(bgr, format=ImageFormat.BGR, frame_id=stream_name))
            except Exception:
                logger.exception(f"R1Lite camera {stream_name} decode error")

    def _depth_decode_loop(self, stream_name: str, q: queue.Queue[Any]) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        out = getattr(self, stream_name)
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
                logger.exception(f"R1Lite {stream_name} decode error")

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
                logger.exception(f"R1Lite {which} decode error")


def _enqueue_drop_oldest(q: queue.Queue[Any], item: Any) -> None:
    """Latest-frame-wins enqueue for size-1 sensor queues."""
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)


__all__ = [
    "R1LITE_GRIPPER_CLOSED",
    "R1LITE_GRIPPER_OPEN",
    "R1LITE_UPPER_BODY_JOINTS",
    "R1LiteConnection",
    "R1LiteConnectionConfig",
]
