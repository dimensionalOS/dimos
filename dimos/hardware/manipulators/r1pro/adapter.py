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

"""Galaxea R1 Pro arm adapter — implements ManipulatorAdapter via ROS 2.

The R1 Pro is a bimanual humanoid with 7-DOF arms.  Each arm is
controlled by publishing ``sensor_msgs/JointState`` commands and
subscribing to joint feedback over ROS 2.  One adapter instance is
created per arm (``side="left"`` or ``side="right"``).

SDK Units: radians (no conversion needed — matches DimOS SI convention).

ROS topics (parameterized by *side*):
  Actuation:
    - Feedback : ``/hdas/feedback_arm_{side}``   (JointState)
    - Command  : ``/motion_target/target_joint_state_arm_{side}`` (JointState)
    - Gripper  : ``/motion_target/target_position_gripper_{side}`` (JointState)
    - Brake    : ``/motion_target/brake_mode``  (Bool)

  Sensors (published to LCM transports on connect):
    - Wrist RGB  : ``/hdas/camera_wrist_{side}/color/image_raw/compressed``
                   → LCM topic ``/r1pro/{hardware_id}/wrist_color``
    - Wrist depth: ``/hdas/camera_wrist_{side}/aligned_depth_to_color/image_raw``
                   → LCM topic ``/r1pro/{hardware_id}/wrist_depth``

All topics use BEST_EFFORT + VOLATILE QoS to match the robot's
``chassis_control_node`` and HDAS drivers.
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from functools import partial
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorAdapter,
    ManipulatorInfo,
)

log = logging.getLogger(__name__)

# Default tracking speed (rad/s) used when the coordinator sends
# velocity=1.0 (the "go as fast as reasonable" default).
_DEFAULT_TRACKING_SPEED = 0.5  # rad/s — conservative, tested on hardware

# DDS discovery takes 3-10 s across the Humble↔Jazzy ethernet link.
_DISCOVERY_TIMEOUT_S = 10.0


def _make_qos() -> Any:
    """Create BEST_EFFORT + VOLATILE QoS profile required by R1 Pro topics."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class R1ProArmAdapter:
    """Galaxea R1 Pro arm adapter.

    Implements the ``ManipulatorAdapter`` protocol via duck typing.
    Uses ``RawROS`` internally for all ROS 2 communication.

    On ``connect()``, the adapter also subscribes to the wrist D405 camera
    topics and publishes decoded frames to LCM transports named
    ``/r1pro/{hardware_id}/wrist_color`` and ``/r1pro/{hardware_id}/wrist_depth``.
    These transports are independent of ``ControlCoordinator`` — external
    consumers subscribe directly.

    Args:
        address: Unused (kept for registry compatibility).
        dof: Degrees of freedom (always 7 for R1 Pro arms).
        side: ``"left"`` or ``"right"``.
        hardware_id: Coordinator hardware ID — used for node naming and
            sensor transport topic names (e.g., ``"left_arm"``).
        tracking_speed: Default tracking speed in rad/s when
            ``velocity=1.0`` is passed to ``write_joint_positions``.
    """

    def __init__(
        self,
        address: str | None = None,
        dof: int = 7,
        side: str = "left",
        hardware_id: str = "arm",
        tracking_speed: float = _DEFAULT_TRACKING_SPEED,
        **_: object,
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if dof != 7:
            log.warning("R1 Pro arms have 7 DOF; got dof=%d — overriding to 7", dof)
            dof = 7

        self._side = side
        self._dof = dof
        self._hardware_id = hardware_id
        self._tracking_speed = tracking_speed

        # ROS handles (populated on connect)
        self._ros: Any | None = None

        # Topic descriptors (populated on connect)
        self._feedback_topic: Any | None = None
        self._command_topic: Any | None = None
        self._gripper_topic: Any | None = None
        self._brake_topic: Any | None = None

        # Cached feedback (protected by _lock)
        self._lock = threading.Lock()
        self._positions: list[float] = [0.0] * self._dof
        self._velocities: list[float] = [0.0] * self._dof
        self._efforts: list[float] = [0.0] * self._dof
        self._feedback_received = False

        # Sensor transports (created on connect, None until then)
        self._wrist_color_transport: Any | None = None
        self._wrist_depth_transport: Any | None = None

        # Off-spin-thread decode queues (latest-frame, size 1)
        self._color_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._color_worker: threading.Thread | None = None
        self._depth_worker: threading.Thread | None = None
        self._sensor_stop = threading.Event()
        # Callback counters — incremented on the spin thread, read by worker for logging
        self._color_cb_count: int = 0
        self._depth_cb_count: int = 0

        # Separate rclpy context for sensor subscriptions — gives sensors their
        # own DDS participant so control traffic cannot starve large camera frames.
        self._sensor_context: Any | None = None
        self._sensor_node: Any | None = None
        self._sensor_executor: Any | None = None
        self._sensor_spin_thread: threading.Thread | None = None

        # State
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.SERVO_POSITION
        self._unsubscribe_feedback: Any | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the R1 Pro arm via ROS 2."""
        from dimos.hardware.r1pro_ros_env import ensure_r1pro_ros_env
        from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

        ensure_r1pro_ros_env()

        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        qos = _make_qos()
        side = self._side

        # Build actuation topic descriptors
        self._feedback_topic = RawROSTopic(
            f"/hdas/feedback_arm_{side}", JointState, qos=qos
        )
        self._command_topic = RawROSTopic(
            f"/motion_target/target_joint_state_arm_{side}", JointState, qos=qos
        )
        self._gripper_topic = RawROSTopic(
            f"/motion_target/target_position_gripper_{side}", JointState, qos=qos
        )
        self._brake_topic = RawROSTopic(
            "/motion_target/brake_mode", Bool, qos=qos
        )

        # Create and start ROS node
        node_name = f"r1pro_arm_{side}_{self._hardware_id}"
        self._ros = RawROS(node_name=node_name)

        try:
            self._ros.start()
        except Exception:
            log.exception("Failed to start RawROS node for R1 Pro %s arm", side)
            self._ros = None
            return False

        # Subscribe to joint feedback
        self._unsubscribe_feedback = self._ros.subscribe(
            self._feedback_topic, self._on_feedback
        )

        # Set up wrist camera sensor streams
        self._setup_sensor_streams(qos)

        # Wait for first feedback message (DDS discovery delay)
        log.info(
            "Waiting up to %.0fs for R1 Pro %s arm feedback...",
            _DISCOVERY_TIMEOUT_S,
            side,
        )
        deadline = time.monotonic() + _DISCOVERY_TIMEOUT_S
        while not self._feedback_received and time.monotonic() < deadline:
            time.sleep(0.05)

        if not self._feedback_received:
            log.warning(
                "No feedback from /hdas/feedback_arm_%s within %.0fs — "
                "adapter connected but positions may be stale.",
                side,
                _DISCOVERY_TIMEOUT_S,
            )

        self._connected = True
        log.info("R1 Pro %s arm adapter connected (feedback=%s)", side, self._feedback_received)
        return True

    def _setup_sensor_streams(self, qos: Any) -> None:
        """Subscribe to wrist camera topics and create LCM transports.

        Subscriptions run in a **separate rclpy context** so the sensor DDS
        participant is isolated from the arm control node (which publishes
        commands at 100 Hz and handles joint feedback).  Without isolation,
        control traffic saturates the shared DDS receive threads and large
        camera frames (requiring UDP fragmentation) are silently dropped.
        """
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node as RclpyNode

        from dimos.core.transport import LCMTransport
        from dimos.msgs.sensor_msgs.Image import Image

        try:
            from sensor_msgs.msg import CompressedImage
            from sensor_msgs.msg import Image as RosImage
        except ImportError:
            log.warning("sensor_msgs not available — wrist camera streams disabled")
            return

        hw_id = self._hardware_id
        side = self._side

        # Create LCM transports
        self._wrist_color_transport = LCMTransport(f"/r1pro/{hw_id}/wrist_color", Image)
        self._wrist_depth_transport = LCMTransport(f"/r1pro/{hw_id}/wrist_depth", Image)

        # --- Isolated DDS participant for sensor subscriptions ---
        self._sensor_context = Context()
        rclpy.init(context=self._sensor_context)
        self._sensor_node = RclpyNode(
            f"r1pro_{side}_sensors",
            context=self._sensor_context,
        )
        self._sensor_executor = MultiThreadedExecutor(
            num_threads=2,
            context=self._sensor_context,
        )
        self._sensor_executor.add_node(self._sensor_node)

        # Subscribe directly on the isolated sensor node.
        # rclpy calls callbacks with (msg) only — wrap to match our (msg, _topic) signature.
        self._sensor_node.create_subscription(
            CompressedImage,
            f"/hdas/camera_wrist_{side}/color/image_raw/compressed",
            lambda msg: self._on_wrist_color(msg, None),
            qos,
        )
        self._sensor_node.create_subscription(
            RosImage,
            f"/hdas/camera_wrist_{side}/aligned_depth_to_color/image_raw",
            lambda msg: self._on_wrist_depth(msg, None),
            qos,
        )

        # Start decode workers (off the spin thread)
        self._sensor_stop.clear()
        self._color_worker = threading.Thread(
            target=self._color_decode_loop, daemon=True,
            name=f"r1pro_{side}_color",
        )
        self._depth_worker = threading.Thread(
            target=self._depth_decode_loop, daemon=True,
            name=f"r1pro_{side}_depth",
        )
        self._color_worker.start()
        self._depth_worker.start()

        # Spin sensor executor in a background thread.
        # Use spin_once in a loop instead of spin() so that:
        #   (a) any callback exception is caught and logged rather than
        #       killing the entire spin thread, and
        #   (b) the loop exits promptly when _sensor_stop is set.
        sensor_stop = self._sensor_stop
        sensor_executor = self._sensor_executor
        sensor_context = self._sensor_context

        def _run_sensor_spin() -> None:
            log.info("R1 Pro %s sensor spin thread started", side)
            while not sensor_stop.is_set() and sensor_context.ok():
                try:
                    sensor_executor.spin_once(timeout_sec=0.1)
                except Exception as exc:
                    # A shut-down rclpy context raises on every spin_once —
                    # don't hot-loop, surface it once and exit.
                    if not sensor_context.ok() or "context is not valid" in str(exc):
                        log.warning(
                            "R1 Pro %s sensor context invalidated, exiting spin loop: %s",
                            side, exc,
                        )
                        break
                    log.warning(
                        "R1 Pro %s sensor executor exception (continuing): %s",
                        side, exc, exc_info=True,
                    )
            log.info("R1 Pro %s sensor spin thread stopped", side)

        self._sensor_spin_thread = threading.Thread(
            target=_run_sensor_spin,
            daemon=True,
            name=f"r1pro_{side}_sensor_spin",
        )
        self._sensor_spin_thread.start()

        log.info(
            "R1 Pro %s arm: wrist camera streams → /r1pro/%s/wrist_color, "
            "/r1pro/%s/wrist_depth (isolated DDS participant)",
            side, hw_id, hw_id,
        )

    def disconnect(self) -> None:
        """Disconnect from the R1 Pro arm."""
        # Signal the sensor spin loop and all decode workers to stop first —
        # must happen before executor.shutdown() so the spin_once loop exits
        # cleanly rather than spinning on a shutting-down executor.
        self._sensor_stop.set()

        # Shutdown sensor executor (separate DDS participant)
        if self._sensor_executor is not None:
            try:
                self._sensor_executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            self._sensor_executor = None
        if self._sensor_spin_thread is not None:
            self._sensor_spin_thread.join(timeout=2.0)
            self._sensor_spin_thread = None
        if self._sensor_node is not None:
            try:
                self._sensor_node.destroy_node()
            except Exception:
                pass
            self._sensor_node = None
        if self._sensor_context is not None:
            try:
                import rclpy
                rclpy.shutdown(context=self._sensor_context)
            except Exception:
                pass
            self._sensor_context = None

        # Unblock decode worker queues with None sentinels
        for q in (self._color_q, self._depth_q):
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        if self._color_worker:
            self._color_worker.join(timeout=1.0)
            self._color_worker = None
        if self._depth_worker:
            self._depth_worker.join(timeout=1.0)
            self._depth_worker = None

        if self._unsubscribe_feedback:
            self._unsubscribe_feedback()
            self._unsubscribe_feedback = None

        if self._ros:
            self._ros.stop()
            self._ros = None

        self._connected = False
        self._enabled = False
        self._feedback_received = False
        log.info("R1 Pro %s arm adapter disconnected", self._side)

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Galaxea",
            model=f"R1 Pro ({self._side} arm)",
            dof=self._dof,
        )

    def get_dof(self) -> int:
        return self._dof

    # Measured effective joint limits (radians) from test_09_joint_limits.py.
    _LIMITS = {
        "left": {
            "lower": [-4.4485, -0.1717, -2.3547, -2.0889, -2.3553, -1.0462, -0.4713],
            "upper": [1.3087, 3.1409, 2.3557, 0.3474, 2.3555, 1.0470, 0.5862],
        },
        "right": {
            # Not yet measured — using URDF limits
            "lower": [-4.4506, -3.1416, -2.3562, -2.0944, -2.3562, -1.0472, -1.5708],
            "upper": [1.3090, 0.1745, 2.3562, 0.3491, 2.3562, 1.0472, 1.5708],
        },
    }

    def get_limits(self) -> JointLimits:
        limits = self._LIMITS[self._side]
        return JointLimits(
            position_lower=list(limits["lower"]),
            position_upper=list(limits["upper"]),
            velocity_max=[math.pi] * self._dof,
        )

    # ------------------------------------------------------------------
    # Control mode
    # ------------------------------------------------------------------

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            self._control_mode = mode
            return True
        log.warning("R1 Pro arms only support POSITION/SERVO_POSITION, got %s", mode)
        return False

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._positions)

    def read_joint_velocities(self) -> list[float]:
        with self._lock:
            return list(self._velocities)

    def read_joint_efforts(self) -> list[float]:
        with self._lock:
            return list(self._efforts)

    def read_state(self) -> dict[str, int]:
        return {
            "state": 0 if self._enabled else 1,
            "mode": 1,  # always servo position
        }

    def read_error(self) -> tuple[int, str]:
        if not self._connected:
            return 1, "not connected"
        if not self._feedback_received:
            return 2, "no feedback received"
        return 0, ""

    def read_enabled(self) -> bool:
        return self._enabled

    def read_force_torque(self) -> list[float] | None:
        """Return joint efforts as a proxy for force/torque.

        Maps the 7 arm joint efforts to a 6-element [fx, fy, fz, tx, ty, tz]
        list using the first 6 joints. Returns None if no feedback yet.
        """
        with self._lock:
            if not self._feedback_received:
                return None
            efforts = list(self._efforts)
        # Map joint efforts to F/T: treat effort[0:3] as torques, zeros for forces
        return [0.0, 0.0, 0.0, efforts[0], efforts[1], efforts[2]]

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        tracking_vel = velocity * self._tracking_speed

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        cmd.position = list(positions)
        cmd.velocity = [tracking_vel] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)
        self._ros.publish(self._brake_topic, Bool(data=False))

        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        log.warning("R1 Pro arms do not support velocity control mode")
        return False

    def write_stop(self) -> bool:
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        with self._lock:
            cmd.position = list(self._positions)
        cmd.velocity = [0.0] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)
        return True

    def write_enable(self, enable: bool) -> bool:
        if not self._ros:
            return False

        from std_msgs.msg import Bool

        self._ros.publish(self._brake_topic, Bool(data=not enable))
        self._enabled = enable
        return True

    def write_clear_errors(self) -> bool:
        return True

    # ------------------------------------------------------------------
    # Optional features
    # ------------------------------------------------------------------

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        # TODO: subscribe to gripper feedback topic once format is confirmed
        return None

    def write_gripper_position(self, position: float) -> bool:
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.position = [position]
        self._ros.publish(self._gripper_topic, cmd)
        return True

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _on_wrist_color(self, msg: Any, _topic: Any) -> None:
        """Enqueue raw ROS message — spin thread does NO data copying."""
        if self._wrist_color_transport is None:
            return
        self._color_cb_count += 1
        # Put the message object directly (O(1), no GIL-heavy bytes copy here).
        # bytes(msg.data) is deferred to the worker thread.
        try:
            self._color_q.put_nowait(msg)
        except queue.Full:
            try:
                self._color_q.get_nowait()  # drop stale frame
            except queue.Empty:
                pass
            self._color_q.put_nowait(msg)

    def _on_wrist_depth(self, msg: Any, _topic: Any) -> None:
        """Enqueue raw depth message — conversion happens off the spin thread."""
        if self._wrist_depth_transport is None:
            return
        self._depth_cb_count += 1
        try:
            self._depth_q.put_nowait(msg)
        except queue.Full:
            try:
                self._depth_q.get_nowait()
            except queue.Empty:
                pass
            self._depth_q.put_nowait(msg)

    def _color_decode_loop(self) -> None:
        """Worker thread: decode JPEG and broadcast to wrist_color transport."""
        import cv2
        import numpy as np
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        frame_count = 0
        cb_count_last = 0
        last_log = time.monotonic()
        while not self._sensor_stop.is_set():
            try:
                msg = self._color_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                # bytes() copy happens here, off the spin thread
                data = bytes(msg.data)
                arr = np.frombuffer(data, np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                img = Image(bgr, format=ImageFormat.BGR,
                            frame_id=f"{self._hardware_id}_wrist_color")
                if self._wrist_color_transport:
                    self._wrist_color_transport.broadcast(None, img)
                    frame_count += 1
            except Exception:
                log.exception("R1 Pro %s wrist color decode error", self._side)
            now = time.monotonic()
            if now - last_log >= 5.0:
                cb_now = self._color_cb_count
                log.info(
                    "R1 Pro %s wrist_color: %d callbacks, %d frames broadcast in last %.0fs",
                    self._side, cb_now - cb_count_last, frame_count, now - last_log,
                )
                frame_count = 0
                cb_count_last = cb_now
                last_log = now

    def _depth_decode_loop(self) -> None:
        """Worker thread: convert depth message and broadcast to wrist_depth transport."""
        from dimos.msgs.sensor_msgs.Image import Image
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        frame_count = 0
        cb_count_last = 0
        last_log = time.monotonic()
        while not self._sensor_stop.is_set():
            try:
                msg = self._depth_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                img = ros_to_dimos(msg, Image)
                if self._wrist_depth_transport:
                    self._wrist_depth_transport.broadcast(None, img)
                    frame_count += 1
            except Exception:
                log.exception("R1 Pro %s wrist depth decode error", self._side)
            now = time.monotonic()
            if now - last_log >= 5.0:
                cb_now = self._depth_cb_count
                log.info(
                    "R1 Pro %s wrist_depth: %d callbacks, %d frames broadcast in last %.0fs",
                    self._side, cb_now - cb_count_last, frame_count, now - last_log,
                )
                frame_count = 0
                cb_count_last = cb_now
                last_log = now

    # ------------------------------------------------------------------
    # Joint feedback callback
    # ------------------------------------------------------------------

    def _on_feedback(self, msg: Any, _topic: Any) -> None:
        """Callback for ``/hdas/feedback_arm_{side}``."""
        with self._lock:
            n = min(len(msg.position), self._dof)
            self._positions[:n] = msg.position[:n]
            if msg.velocity:
                nv = min(len(msg.velocity), self._dof)
                self._velocities[:nv] = msg.velocity[:nv]
            if msg.effort:
                ne = min(len(msg.effort), self._dof)
                self._efforts[:ne] = msg.effort[:ne]
            self._feedback_received = True


# ===========================================================================
# R1ProTorsoAdapter — 4-DOF torso (pitch/pitch/pitch/yaw)
# ===========================================================================


class R1ProTorsoAdapter:
    """Galaxea R1 Pro torso adapter.

    Implements the ``ManipulatorAdapter`` protocol via duck typing.
    Controls the 4-DOF torso stack that sits between the chassis and
    the dual arms.  No sensor streams — the torso IMU is handled by
    the chassis adapter.

    ROS topics:
      - Feedback: ``/hdas/feedback_torso``   (JointState, ~50 Hz)
      - Command : ``/motion_target/target_joint_state_torso`` (JointState)

    Joint order (matches URDF):
      [0] torso_joint1 — lower pitch (main tilt),  [-1.13,  1.83] rad
      [1] torso_joint2 — upper pitch,               [-2.79,  2.53] rad
      [2] torso_joint3 — neck/shoulder pitch,       [-1.83,  1.57] rad
      [3] torso_joint4 — yaw (~full rotation),      [-3.05,  3.05] rad
    """

    # Joint limits from r1_pro.urdf (confirmed in hardware deep-dive)
    _LIMITS = JointLimits(
        position_lower=[-1.13, -2.79, -1.83, -3.05],
        position_upper=[ 1.83,  2.53,  1.57,  3.05],
        velocity_max=[math.pi] * 4,  # URDF cap is 2.5 rad/s; π is safe
    )

    def __init__(
        self,
        address: str | None = None,
        dof: int = 4,
        hardware_id: str = "torso",
        tracking_speed: float = _DEFAULT_TRACKING_SPEED,
        **_: object,
    ) -> None:
        if dof != 4:
            log.warning("R1 Pro torso has 4 DOF; got dof=%d — overriding to 4", dof)
        self._dof = 4
        self._hardware_id = hardware_id
        self._tracking_speed = tracking_speed

        # ROS handles (populated on connect)
        self._ros: Any | None = None
        self._feedback_topic: Any | None = None
        self._command_topic: Any | None = None
        self._brake_topic: Any | None = None

        # Cached feedback (protected by _lock)
        self._lock = threading.Lock()
        self._positions: list[float] = [0.0] * self._dof
        self._velocities: list[float] = [0.0] * self._dof
        self._efforts: list[float] = [0.0] * self._dof
        self._feedback_received = False

        # State
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.SERVO_POSITION
        self._unsubscribe_feedback: Any | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the R1 Pro torso via ROS 2."""
        from dimos.hardware.r1pro_ros_env import ensure_r1pro_ros_env
        from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

        ensure_r1pro_ros_env()

        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        qos = _make_qos()

        self._feedback_topic = RawROSTopic(
            "/hdas/feedback_torso", JointState, qos=qos
        )
        self._command_topic = RawROSTopic(
            "/motion_target/target_joint_state_torso", JointState, qos=qos
        )
        self._brake_topic = RawROSTopic(
            "/motion_target/brake_mode", Bool, qos=qos
        )

        node_name = f"r1pro_torso_{self._hardware_id}"
        self._ros = RawROS(node_name=node_name)

        try:
            self._ros.start()
        except Exception:
            log.exception("Failed to start RawROS node for R1 Pro torso")
            self._ros = None
            return False

        self._unsubscribe_feedback = self._ros.subscribe(
            self._feedback_topic, self._on_feedback
        )

        log.info("Waiting up to %.0fs for R1 Pro torso feedback...", _DISCOVERY_TIMEOUT_S)
        deadline = time.monotonic() + _DISCOVERY_TIMEOUT_S
        while not self._feedback_received and time.monotonic() < deadline:
            time.sleep(0.05)

        if not self._feedback_received:
            log.warning(
                "No feedback from /hdas/feedback_torso within %.0fs — "
                "adapter connected but positions may be stale.",
                _DISCOVERY_TIMEOUT_S,
            )

        self._connected = True
        log.info("R1 Pro torso adapter connected (feedback=%s)", self._feedback_received)
        return True

    def disconnect(self) -> None:
        """Disconnect from the R1 Pro torso."""
        if self._unsubscribe_feedback:
            self._unsubscribe_feedback()
            self._unsubscribe_feedback = None

        if self._ros:
            self._ros.stop()
            self._ros = None

        self._connected = False
        self._enabled = False
        self._feedback_received = False
        log.info("R1 Pro torso adapter disconnected")

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Galaxea", model="R1 Pro (torso)", dof=self._dof)

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=list(self._LIMITS.position_lower),
            position_upper=list(self._LIMITS.position_upper),
            velocity_max=list(self._LIMITS.velocity_max),
        )

    # ------------------------------------------------------------------
    # Control mode
    # ------------------------------------------------------------------

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            self._control_mode = mode
            return True
        log.warning("R1 Pro torso only supports POSITION/SERVO_POSITION, got %s", mode)
        return False

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._positions)

    def read_joint_velocities(self) -> list[float]:
        with self._lock:
            return list(self._velocities)

    def read_joint_efforts(self) -> list[float]:
        with self._lock:
            return list(self._efforts)

    def read_state(self) -> dict[str, int]:
        return {"state": 0 if self._enabled else 1, "mode": 1}

    def read_error(self) -> tuple[int, str]:
        if not self._connected:
            return 1, "not connected"
        if not self._feedback_received:
            return 2, "no feedback received"
        return 0, ""

    def read_enabled(self) -> bool:
        return self._enabled

    def read_force_torque(self) -> list[float] | None:
        return None

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def read_gripper_position(self) -> float | None:
        return None

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        tracking_vel = velocity * self._tracking_speed

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        cmd.position = list(positions)
        cmd.velocity = [tracking_vel] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)
        self._ros.publish(self._brake_topic, Bool(data=False))
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        log.warning("R1 Pro torso does not support velocity control mode")
        return False

    def write_stop(self) -> bool:
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        with self._lock:
            cmd.position = list(self._positions)
        cmd.velocity = [0.0] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)
        return True

    def write_enable(self, enable: bool) -> bool:
        if not self._ros:
            return False

        from std_msgs.msg import Bool

        self._ros.publish(self._brake_topic, Bool(data=not enable))
        self._enabled = enable
        return True

    def write_clear_errors(self) -> bool:
        return True

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def write_gripper_position(self, position: float) -> bool:
        return False

    # ------------------------------------------------------------------
    # Feedback callback
    # ------------------------------------------------------------------

    def _on_feedback(self, msg: Any, _topic: Any) -> None:
        """Callback for ``/hdas/feedback_torso``."""
        with self._lock:
            n = min(len(msg.position), self._dof)
            self._positions[:n] = msg.position[:n]
            if msg.velocity:
                nv = min(len(msg.velocity), self._dof)
                self._velocities[:nv] = msg.velocity[:nv]
            if msg.effort:
                ne = min(len(msg.effort), self._dof)
                self._efforts[:ne] = msg.effort[:ne]
            self._feedback_received = True


# ===========================================================================
# R1ProUpperBodyAdapter — composite 18-DOF (torso + left arm + right arm)
# ===========================================================================


class R1ProUpperBodyAdapter:
    """Galaxea R1 Pro upper-body adapter — composite 18-DOF interface.

    Wraps one :class:`R1ProTorsoAdapter` and two :class:`R1ProArmAdapter`
    instances and exposes them as a single flat ``ManipulatorAdapter``.
    This allows external controllers and policies to command the full
    upper body without knowing about sub-adapter boundaries.

    **Flat joint order** (18 DOF total):

    +---------+---------+----------------------------------------------------+
    | Indices | Segment | Description                                        |
    +=========+=========+====================================================+
    | 0 – 3   | torso   | torso_joint1–4 (pitch, pitch, pitch, yaw)          |
    +---------+---------+----------------------------------------------------+
    | 4 – 10  | left    | left_arm_joint1–7                                  |
    +---------+---------+----------------------------------------------------+
    | 11 – 17 | right   | right_arm_joint1–7                                 |
    +---------+---------+----------------------------------------------------+

    Gripper control is not handled here; use separate gripper
    ``HardwareComponent`` entries if needed.
    """

    _TORSO_SLICE = slice(0, 4)
    _LEFT_SLICE = slice(4, 11)
    _RIGHT_SLICE = slice(11, 18)
    _DOF = 18

    def __init__(
        self,
        address: str | None = None,
        hardware_id: str = "upper_body",
        tracking_speed: float = _DEFAULT_TRACKING_SPEED,
        **_: object,
    ) -> None:
        self._hardware_id = hardware_id
        self._torso = R1ProTorsoAdapter(
            hardware_id=f"{hardware_id}_torso",
            tracking_speed=tracking_speed,
        )
        self._left = R1ProArmAdapter(
            side="left",
            hardware_id=f"{hardware_id}_left",
            tracking_speed=tracking_speed,
        )
        self._right = R1ProArmAdapter(
            side="right",
            hardware_id=f"{hardware_id}_right",
            tracking_speed=tracking_speed,
        )
        self._control_mode = ControlMode.SERVO_POSITION

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect all sub-adapters. On partial failure, disconnects already-connected ones."""
        connected: list[Any] = []
        for sub in (self._torso, self._left, self._right):
            if sub.connect():
                connected.append(sub)
            else:
                log.error(
                    "R1 Pro upper-body: sub-adapter %s failed to connect — aborting",
                    sub.__class__.__name__,
                )
                for already in connected:
                    already.disconnect()
                return False
        log.info("R1 Pro upper-body adapter connected (18 DOF)")
        return True

    def disconnect(self) -> None:
        for sub in (self._torso, self._left, self._right):
            sub.disconnect()
        log.info("R1 Pro upper-body adapter disconnected")

    def is_connected(self) -> bool:
        return (
            self._torso.is_connected()
            and self._left.is_connected()
            and self._right.is_connected()
        )

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Galaxea",
            model="R1 Pro (upper body: torso + left + right)",
            dof=self._DOF,
        )

    def get_dof(self) -> int:
        return self._DOF

    def get_limits(self) -> JointLimits:
        t = self._torso.get_limits()
        l = self._left.get_limits()
        r = self._right.get_limits()
        return JointLimits(
            position_lower=t.position_lower + l.position_lower + r.position_lower,
            position_upper=t.position_upper + l.position_upper + r.position_upper,
            velocity_max=t.velocity_max + l.velocity_max + r.velocity_max,
        )

    # ------------------------------------------------------------------
    # Control mode
    # ------------------------------------------------------------------

    def set_control_mode(self, mode: ControlMode) -> bool:
        ok = all(sub.set_control_mode(mode) for sub in (self._torso, self._left, self._right))
        if ok:
            self._control_mode = mode
        return ok

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_joint_positions(self) -> list[float]:
        return (
            self._torso.read_joint_positions()
            + self._left.read_joint_positions()
            + self._right.read_joint_positions()
        )

    def read_joint_velocities(self) -> list[float]:
        return (
            self._torso.read_joint_velocities()
            + self._left.read_joint_velocities()
            + self._right.read_joint_velocities()
        )

    def read_joint_efforts(self) -> list[float]:
        return (
            self._torso.read_joint_efforts()
            + self._left.read_joint_efforts()
            + self._right.read_joint_efforts()
        )

    def read_state(self) -> dict[str, int]:
        state: dict[str, int] = {}
        for sub in (self._torso, self._left, self._right):
            state.update(sub.read_state())
        return state

    def read_error(self) -> tuple[int, str]:
        for sub in (self._torso, self._left, self._right):
            code, msg = sub.read_error()
            if code != 0:
                return code, msg
        return 0, ""

    def read_enabled(self) -> bool:
        return self._torso.read_enabled() and self._left.read_enabled() and self._right.read_enabled()

    def read_force_torque(self) -> list[float] | None:
        return None

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def read_gripper_position(self) -> float | None:
        return None

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        ok1 = self._torso.write_joint_positions(list(positions[self._TORSO_SLICE]), velocity)
        ok2 = self._left.write_joint_positions(list(positions[self._LEFT_SLICE]), velocity)
        ok3 = self._right.write_joint_positions(list(positions[self._RIGHT_SLICE]), velocity)
        return ok1 and ok2 and ok3

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        log.warning("R1 Pro upper-body does not support velocity control mode")
        return False

    def write_stop(self) -> bool:
        return all(sub.write_stop() for sub in (self._torso, self._left, self._right))

    def write_enable(self, enable: bool) -> bool:
        return all(sub.write_enable(enable) for sub in (self._torso, self._left, self._right))

    def write_clear_errors(self) -> bool:
        return all(sub.write_clear_errors() for sub in (self._torso, self._left, self._right))

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def write_gripper_position(self, position: float) -> bool:
        return False


# ------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------


def register(registry: AdapterRegistry) -> None:
    """Register R1 Pro arm, torso, and upper-body adapters with the manipulator registry."""
    registry.register("r1pro_arm_left", partial(R1ProArmAdapter, side="left"))
    registry.register("r1pro_arm_right", partial(R1ProArmAdapter, side="right"))
    registry.register("r1pro_arm", R1ProArmAdapter)
    registry.register("r1pro_torso", R1ProTorsoAdapter)
    registry.register("r1pro_upper_body", R1ProUpperBodyAdapter)


__all__ = ["R1ProArmAdapter", "R1ProTorsoAdapter", "R1ProUpperBodyAdapter"]
