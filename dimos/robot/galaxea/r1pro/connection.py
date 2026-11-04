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
right arm 7) take positions on ``position_command``, the chassis takes
speeds on ``cmd_vel``, and feedback goes out on ``joint_state``, the two
IMU streams and wheel odometry. The chassis lidar is ours, not the
vendor's: see ``r1pro_lidar_odometry``.

Commands arrive as standard messages and go through ``command``: checked
against the URDF limits and ``base_range`` (out-of-range base speeds are
clamped), then forwarded by ``write``. Once a command was accepted, going
``deadman_timeout_s`` without another halts the robot: zero chassis speed
and the arms held where they were last measured.

The two grippers ride the same contract as ``r1pro/left_gripper`` and
``r1pro/right_gripper``: a jaw takes its opening in metres (the vendor
topics speak millimetres, 0-100 mm) on the same ``position_command``
messages, and reports its measured opening on ``joint_state`` in a second,
position-only message. A jaw is told only what a task explicitly asks for —
never a hold, never a lifecycle nudge — and only while its feedback is
fresh, parseable and fault-free. Feedback that goes stale, malformed or
faulted interrupts the intent it was carrying: that value is not sent
again, even if a task still holds it, until the intent is withdrawn (a
command without the jaw) or replaced by a different value. There is no
verified vendor cancel: a target the vendor accepted keeps driving the jaw,
so stop and halt leave the grippers alone rather than pretending to recall
it.

Sensors run on a second RawROS node. Conversion happens on per-stream workers
behind latest-wins queues.

The on-robot joint tracker manages PD gains; targets are followed at
``config.tracking_speed``. Chassis gate handling lives in the on-robot
``galaxea-dimos`` gatekeeper. ROS env (``ROS_DOMAIN_ID`` etc.) comes from
the environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
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
from dimos.control.contract.convert import (
    TWIST_INTERFACES,
    joint_state_to_values,
    twist_to_values,
)
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
from dimos.robot.galaxea.r1pro.config import R1PRO_CONNECTION_MODEL, R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.joints import (
    GRIPPER_JOINTS,
    UPPER_BODY_JOINTS,
    coordinator_name,
)
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

#: The chassis as the control contract names it: "r1pro/base", with vx/vy/wz.
R1PRO_BASE = f"r1pro{SEPARATOR}base"
#: The chassis' axes as control tasks claim them, e.g. "r1pro/base/vx".
R1PRO_BASE_JOINTS: list[str] = [f"{R1PRO_BASE}{SEPARATOR}{axis}" for axis in TWIST_INTERFACES]
#: The jaws as the control contract names them: "r1pro/left_gripper" and
#: "r1pro/right_gripper"; each value is the jaw's opening in metres.
R1PRO_GRIPPER_JOINTS: list[str] = [coordinator_name(joint) for joint in GRIPPER_JOINTS]
#: Contract joint name -> gripper side, e.g. "r1pro/left_gripper" -> "left".
R1PRO_GRIPPER_SIDES: dict[str, str] = {
    coordinator_name(joint): joint.split("_", 1)[0] for joint in GRIPPER_JOINTS
}

# The vendor gripper interface (mm, 0-100 fully closed to fully open): one
# target topic and one feedback topic per side, plus per-segment fault
# reports the jaw depends on. The jaw controller owns interpolation and
# speed, so a repeated target is not re-sent.
_GRIPPER_MM_PER_M = 1000.0
_GRIPPER_COMMAND_TOPIC = "/motion_target/target_position_gripper_{}"
_GRIPPER_FEEDBACK_TOPIC = "/hdas/feedback_gripper_{}"
# Segments whose fault report interrupts a jaw's intent: its own, its arm's
# (a faulted arm may drop what the jaw holds), and the torso both arms hang
# from.
_GRIPPER_FAULT_SEGMENTS: dict[str, tuple[str, ...]] = {
    "left": ("torso", "arm_left", "gripper_left"),
    "right": ("torso", "arm_right", "gripper_right"),
}
_GRIPPER_STATUS_TOPIC = "/hdas/feedback_status_{}"

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


@dataclass
class _GripperChannel:
    """One jaw's runtime state; every field is guarded by the module lock.

    ``blocked_intent_m`` is the value a fault, a malformed or a stale
    feedback interrupted. It is not sent again — even while a task still
    holds it — until a command arrives without the jaw (the intent was
    withdrawn) or with a different value (a new explicit command).
    """

    measured_m: float | None = None
    feedback_at: float = 0.0  # time.monotonic() of the last parseable feedback
    feedback_wall: float = 0.0  # time.time() of the same, for message stamps
    feedback_ok: bool = False
    # Segment -> (its reported errors, time.monotonic() of the report). A
    # clean report clears its segment; a stale one is ignored, not trusted.
    faults: dict[str, tuple[tuple[str, ...], float]] = field(default_factory=dict)
    last_sent_m: float | None = None
    last_intent_m: float | None = None
    blocked_intent_m: float | None = None


def _gripper_fault(ch: _GripperChannel, timeout_s: float, now: float) -> str | None:
    """The faults a jaw's fresh status reports carry, or ``None``."""
    active = [
        f"{segment}: {error}"
        for segment, (errors, at) in ch.faults.items()
        if errors and now - at <= timeout_s
        for error in errors
    ]
    return "; ".join(active) if active else None


def _gripper_valid(ch: _GripperChannel, timeout_s: float, now: float) -> bool:
    """Whether the jaw's feedback may carry commands and readbacks."""
    return (
        ch.measured_m is not None
        and ch.feedback_ok
        and now - ch.feedback_at <= timeout_s
        and _gripper_fault(ch, timeout_s, now) is None
    )


def _gripper_reportable(ch: _GripperChannel, timeout_s: float, now: float) -> bool:
    """Whether the measured opening is publishable: valid, and not gated by
    an interrupted intent a task has yet to withdraw."""
    return _gripper_valid(ch, timeout_s, now) and ch.blocked_intent_m is None


def _ros_stamp_now() -> Any:
    from builtin_interfaces.msg import Time as RosTime

    t = time.time()
    sec = int(t)
    return RosTime(sec=sec, nanosec=int((t - sec) * 1e9))


def _r1pro_base_range() -> Twist:
    """The chassis' top speeds, shared with the planning model's planar base."""
    vx, vy, wz = R1PRO_PLANAR_BASE.velocity_limits
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))


class R1ProConnectionConfig(ConnectionConfig):
    """How the R1 Pro is told to move, plus its sensor and vendor-stack knobs.

    The contract fields (``source``, ``model``, ``joint_commands``,
    ``base_range``, ``deadman_timeout_s``) default to the real robot: the
    18 upper-body joints take positions only — the vendor tracker's speed
    is a tracking-speed cap, not a continuous velocity command — and the
    chassis takes vx/vy/wz clamped to ``R1PRO_PLANAR_BASE.velocity_limits``.

    Attributes:
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
        gripper_feedback_timeout_s: How old a jaw's feedback may be before
            its commands are dropped and its readback withheld.
        boot_vendor_stack: Boot the vendor stack (HDAS, mobiman), which
            serves every ROS topic used here, if it is not running.
        vendor_profile: "" boots vendor_setup.sh's own set; a vendor profile,
            e.g. "ATCStandard/R1PROVRTeleop", boots that.
    """

    source: str = "r1pro"
    model: RobotModel | None = R1PRO_CONNECTION_MODEL
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    base_range: Twist | None = Field(default_factory=_r1pro_base_range)
    publish_rate_hz: float = Field(default=100.0)
    tracking_speed: float = Field(default=0.5)
    publish_odom_tf: bool = Field(default=True)
    frame_id: str = Field(default="base_link")
    odom_frame_id: str = Field(default="odom")
    sensor_stats_interval_s: float = Field(default=10.0)
    enable_wrist_depth: bool = Field(default=False)
    enable_wrist_color: bool = Field(default=True)
    color_publish_hz: float = Field(default=30.0)
    gripper_feedback_timeout_s: float = Field(default=1.0)
    boot_vendor_stack: bool = Field(default=True)
    vendor_profile: str = Field(default="")


class R1ProConnection(ConnectionModule):
    """R1 Pro connection module — ROS 2 control and sensor nodes.

    ``position_command`` carries all 18 upper-body joints' targets in rad,
    named ``r1pro/<joint>``; ``cmd_vel`` carries the chassis' speeds. Both
    are checked against the limits table before anything is sent. Feedback
    is published once every segment (torso, left arm, right arm) has
    reported, at ``config.publish_rate_hz``.
    """

    config: R1ProConnectionConfig

    # Control inputs.
    position_command: In[JointState]
    cmd_vel: In[Twist]

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
        self._gripper_cmd_topics: dict[str, RawROSTopic] = {}
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

        # The jaws, by side ("left"/"right"); see _GripperChannel.
        self._grippers: dict[str, _GripperChannel] = {
            side: _GripperChannel() for side in _GRIPPER_FAULT_SEGMENTS
        }

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
                self.cmd_vel.subscribe(lambda msg: self.command(twist_to_values(msg, R1PRO_BASE)))
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
        if any(target is not None for target in positions):
            if not all(target is not None for target in positions):
                raise ValueError(
                    "a position command must cover all "
                    f"{_NUM_MOTORS} joints, got "
                    f"{sum(target is not None for target in positions)}"
                )
            self._send_joint_targets([float(q) for q in positions if q is not None], stamp)

        speeds = [values.get(f"{R1PRO_BASE}{SEPARATOR}{axis}") for axis in TWIST_INTERFACES]
        if any(speed is not None for speed in speeds):
            vx, vy, wz = (0.0 if speed is None else speed for speed in speeds)
            self._send_chassis_speed(vx, vy, wz, stamp)

        self._write_grippers(values, stamp)

    def _write_grippers(self, values: Values, stamp: Any) -> None:
        """Forward each commanded jaw opening, or drop it with the reason.

        A jaw gets a vendor target only for an explicit value while its
        feedback is valid, and never twice in a row: the vendor controller
        keeps following the target it has, so re-sending it would only
        restart interpolation. An intent interrupted by bad feedback stays
        blocked until withdrawn or replaced.
        """
        now = time.monotonic()
        sends: list[tuple[str, float]] = []
        drops: list[str] = []
        with self._lock:
            for joint, side in R1PRO_GRIPPER_SIDES.items():
                value = values.get(f"{joint}{SEPARATOR}{Interface.POSITION}")
                ch = self._grippers[side]
                if value is None:
                    ch.last_intent_m = None
                    if ch.blocked_intent_m is not None:
                        # The interrupted intent was withdrawn; the next
                        # explicit command starts clean.
                        ch.blocked_intent_m = None
                        ch.last_sent_m = None
                    continue
                ch.last_intent_m = value
                if not _gripper_valid(ch, self.config.gripper_feedback_timeout_s, now):
                    ch.blocked_intent_m = value
                    fault = _gripper_fault(ch, self.config.gripper_feedback_timeout_s, now)
                    drops.append(
                        f"gripper_{side}: dropped {value:.4f} m — feedback missing, "
                        f"stale or faulted ({fault or 'no live feedback'})"
                    )
                    continue
                if ch.blocked_intent_m is not None and value == ch.blocked_intent_m:
                    continue  # the interrupted intent replaying; not re-sent
                ch.blocked_intent_m = None
                if value == ch.last_sent_m:
                    continue  # the vendor controller still holds this target
                ch.last_sent_m = value
                sends.append((side, value))
        for message in drops:
            self._report(message, rejection=True)
        for side, value in sends:
            self._send_gripper_target(side, value, stamp)

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

    def _send_gripper_target(self, side: str, opening_m: float, stamp: Any) -> None:
        """One jaw's opening target on its vendor topic, in millimetres."""
        from sensor_msgs.msg import JointState as RosJointState

        ros = self._ros
        topic = self._gripper_cmd_topics.get(side)
        if ros is None or topic is None:
            return  # pre-connect / post-stop
        cmd = RosJointState()
        cmd.header.stamp = stamp
        cmd.name = [f"{side}_gripper"]
        cmd.position = [opening_m * _GRIPPER_MM_PER_M]
        ros.publish(topic, cmd)

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
        self._gripper_cmd_topics = {
            side: RawROSTopic(_GRIPPER_COMMAND_TOPIC.format(side), RosJointState, qos=qos)
            for side in self._grippers
        }

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

        # The jaws: one position feedback per side, plus the fault reports
        # that interrupt a jaw's intent. hdas_msg is a vendor package with no
        # stub in this environment, so it is imported lazily here — without
        # it the jaws gate on their position feedback alone.
        for side in self._grippers:
            self._control_unsubs.append(
                self._ros.subscribe(
                    RawROSTopic(_GRIPPER_FEEDBACK_TOPIC.format(side), RosJointState, qos=qos),
                    self._make_gripper_feedback_cb(side),
                )
            )
        try:
            from hdas_msg.msg import (
                FeedbackStatus,  # type: ignore[import-not-found, import-untyped]
            )
        except ImportError:
            logger.warning(
                "hdas_msg not available — gripper fault reports disabled; "
                "the jaws gate on their position feedback alone."
            )
        else:
            for segment in sorted({s for segs in _GRIPPER_FAULT_SEGMENTS.values() for s in segs}):
                self._control_unsubs.append(
                    self._ros.subscribe(
                        RawROSTopic(_GRIPPER_STATUS_TOPIC.format(segment), FeedbackStatus, qos=qos),
                        self._on_gripper_status,
                    )
                )

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

    # Gripper feedback callbacks (one jaw per side)

    def _make_gripper_feedback_cb(self, side: str) -> Any:
        """One jaw's opening feedback: a single finite position, in mm.

        A malformed frame marks the feedback not-ok: the last measured value
        is kept (so a readback never shows a number the vendor did not send
        this side of the failure) but nothing is commanded or reported from
        it until a parseable frame arrives.
        """

        def cb(msg: Any, _topic: Any) -> None:
            now_mono = time.monotonic()
            now_wall = time.time()
            with self._lock:
                ch = self._grippers[side]
                positions = getattr(msg, "position", None)
                if positions is None or len(positions) != 1 or not math.isfinite(positions[0]):
                    ch.feedback_ok = False
                    return
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                age = max(0.0, now_wall - stamp) if stamp else 0.0
                ch.measured_m = float(positions[0]) / _GRIPPER_MM_PER_M
                ch.feedback_at = now_mono - age
                ch.feedback_wall = now_wall - age
                ch.feedback_ok = True

        return cb

    def _on_gripper_status(self, msg: Any, topic: Any) -> None:
        """One segment's fault report, applied to every jaw that depends on it.

        Mirrors the vendor semantics: an entry with a code or a description
        is an error, and a report with no entries at all is itself an error
        ("empty status report"). A clean report clears the segment.
        """
        segment = topic.topic.removeprefix("/hdas/feedback_status_")
        errors = tuple(
            f"{e.name}: {e.error_code} {e.error_description}"
            for e in msg.errors
            if e.error_code or e.error_description
        )
        if not msg.errors:
            errors = ("empty status report",)
        with self._lock:
            for side, segments in _GRIPPER_FAULT_SEGMENTS.items():
                if segment in segments:
                    self._grippers[side].faults[segment] = (errors, time.monotonic())

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

        half = self._odom_yaw * 0.5
        position = Vector3(self._odom_x, self._odom_y, 0.0)
        orientation = Quaternion(0.0, 0.0, math.sin(half), math.cos(half))
        frame_id = self.config.odom_frame_id
        base = self.config.frame_id
        pose = PoseStamped(ts=now, frame_id=frame_id, position=position, orientation=orientation)
        self.odometry.publish(
            Odometry(
                ts=now,
                frame_id=frame_id,
                child_frame_id=base,
                pose=Pose(position, orientation),
                twist=Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz)),
            )
        )
        if self.config.publish_odom_tf:
            self.odom.publish(pose)
            self.tf.publish(TFMessage(Transform.from_pose(base, pose)))

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
                # The jaws ride their own message: only a jaw whose feedback
                # is valid, fresh and not interrupted by a blocked intent is
                # reported; a jaw that drops out stays out until it earns it.
                now_mono = time.monotonic()
                gripper_readings = []
                for joint, side in R1PRO_GRIPPER_SIDES.items():
                    ch = self._grippers[side]
                    if ch.measured_m is not None and _gripper_reportable(
                        ch, self.config.gripper_feedback_timeout_s, now_mono
                    ):
                        gripper_readings.append((joint, ch.measured_m, ch.feedback_wall))

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

            if gripper_readings:
                self.joint_state.publish(
                    JointState(
                        ts=min(stamp for _, _, stamp in gripper_readings),
                        frame_id=frame_id,
                        name=[joint for joint, _, _ in gripper_readings],
                        position=[measured for _, measured, _ in gripper_readings],
                    )
                )

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
