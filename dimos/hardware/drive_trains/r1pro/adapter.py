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

"""Galaxea R1 Pro chassis adapter — implements TwistBaseAdapter via ROS 2.

The R1 Pro has a 3-wheel swerve drive base.  Velocity commands flow
through ``chassis_control_node`` which performs inverse kinematics and
motion profiling before forwarding to the HDAS CAN bus driver.

Three "gates" must be open for commands to take effect:

  Gate 1 — Subscribe to ``/motion_control/chassis_speed`` (IK output).
           The node skips IK when the subscriber count is zero.
           This adapter opens Gate 1 in ``connect()`` with a no-op
           subscription.

  Gate 2 — Publish ``ControllerSignalStamped`` with ``data.mode=5``
           on ``/controller_unused``.  Must run as a separate process
           on the robot (see ``scripts/r1pro_test/``).

  Gate 3 — Publish acceleration limits on
           ``/motion_target/chassis_acc_limit``.  Published alongside
           every velocity command.

Sensor streams (published as independent LCM transports on connect):
  - ``/r1pro/{hardware_id}/head``                — head stereo RGB
  - ``/r1pro/{hardware_id}/chassis_front_left``  — front-left chassis camera
  - ``/r1pro/{hardware_id}/chassis_front_right`` — front-right chassis camera
  - ``/r1pro/{hardware_id}/chassis_left``        — left chassis camera
  - ``/r1pro/{hardware_id}/chassis_right``       — right chassis camera
  - ``/r1pro/{hardware_id}/chassis_rear``        — rear chassis camera
  - ``/r1pro/{hardware_id}/head_depth``          — head depth image
  - ``/r1pro/{hardware_id}/lidar``               — chassis LiDAR point cloud
  - ``/r1pro/{hardware_id}/imu_chassis``         — chassis IMU
  - ``/r1pro/{hardware_id}/imu_torso``           — torso IMU

All topics use BEST_EFFORT + VOLATILE QoS.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from functools import partial
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

log = logging.getLogger(__name__)

_ACC_LIMIT_X = 2.5    # m/s²
_ACC_LIMIT_Y = 1.0    # m/s²
_ACC_LIMIT_YAW = 1.0  # rad/s²
_DISCOVERY_TIMEOUT_S = 5.0

# Camera name → ROS topic (all CompressedImage)
_CAMERA_TOPICS: dict[str, str] = {
    "head":               "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "chassis_front_left": "/hdas/camera_chassis_front_left/rgb/compressed",
    "chassis_front_right": "/hdas/camera_chassis_front_right/rgb/compressed",
    "chassis_left":       "/hdas/camera_chassis_left/rgb/compressed",
    "chassis_right":      "/hdas/camera_chassis_right/rgb/compressed",
    "chassis_rear":       "/hdas/camera_chassis_rear/rgb/compressed",
}


def _make_qos() -> Any:
    """Create BEST_EFFORT + VOLATILE QoS profile required by R1 Pro topics."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class R1ProChassisAdapter:
    """Galaxea R1 Pro chassis adapter.

    Implements the ``TwistBaseAdapter`` protocol via duck typing.
    Uses ``RawROS`` internally for all ROS 2 communication.

    On ``connect()``, the adapter subscribes to all chassis-mounted sensors
    and publishes decoded data to independent LCM transports (see module
    docstring for topic names). The sensor streams are fully decoupled from
    the control tick loop.

    Args:
        dof: Number of velocity DOFs (always 3: vx, vy, wz).
        hardware_id: Coordinator hardware ID — used for node naming and
            sensor transport names (e.g., ``"chassis"``).
        address: Unused (kept for registry compatibility).
    """

    def __init__(
        self,
        dof: int = 3,
        hardware_id: str = "chassis",
        address: str | None = None,
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"R1 Pro chassis is holonomic (3-DOF), got dof={dof}")

        self._dof = dof
        self._hardware_id = hardware_id

        # ROS handles (populated on connect)
        self._ros: Any | None = None

        # Actuation topic descriptors (populated on connect)
        self._speed_topic: Any | None = None
        self._acc_topic: Any | None = None
        self._brake_topic: Any | None = None
        self._chassis_speed_topic: Any | None = None

        # State (protected by _lock)
        self._lock = threading.Lock()
        self._last_velocities: list[float] = [0.0] * self._dof
        self._connected = False
        self._enabled = False
        self._gate1_received = False

        # Unsubscribe callables — collected so disconnect() can clean up all
        self._unsubs: list[Any] = []

        # Sensor transports (created on connect)
        self._camera_transports: dict[str, Any] = {}
        self._depth_transport: Any | None = None
        self._lidar_transport: Any | None = None
        self._imu_chassis_transport: Any | None = None
        self._imu_torso_transport: Any | None = None

        # Off-spin-thread decode: one queue per camera (latest-frame, size 1)
        # plus shared queues for depth/lidar/imu
        self._camera_qs: dict[str, queue.Queue[Any]] = {}
        self._depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._lidar_q: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._imu_chassis_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._imu_torso_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._sensor_stop = threading.Event()
        self._sensor_workers: list[threading.Thread] = []
        # Callback counters — incremented on the spin thread, read by workers for logging
        self._camera_cb_counts: dict[str, int] = {}
        self._depth_cb_count: int = 0
        self._lidar_cb_count: int = 0

        # Separate rclpy context for sensor subscriptions — gives sensors their
        # own DDS participant so control traffic (arm commands at 100 Hz) cannot
        # saturate the shared DDS receive threads and drop large camera frames.
        self._sensor_context: Any | None = None
        self._sensor_node: Any | None = None
        self._sensor_executor: Any | None = None
        self._sensor_spin_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the R1 Pro chassis via ROS 2.

        Opens Gate 1 (IK subscriber), prepares actuation publishers, and
        subscribes all sensor topics with transport-backed publishers.
        """
        from dimos.hardware.r1pro_ros_env import ensure_r1pro_ros_env
        from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

        ensure_r1pro_ros_env()

        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Bool

        qos = _make_qos()

        # Actuation topics
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

        node_name = f"r1pro_chassis_{self._hardware_id}"
        self._ros = RawROS(node_name=node_name)

        try:
            self._ros.start()
        except Exception:
            log.exception("Failed to start RawROS node for R1 Pro chassis")
            self._ros = None
            return False

        # Gate 1: subscribe to IK output to unlock chassis_control_node
        unsub = self._ros.subscribe(self._chassis_speed_topic, self._on_chassis_speed)
        self._unsubs.append(unsub)

        # Set up sensor streams
        self._setup_sensor_streams(qos)

        log.info("Waiting %.0fs for DDS discovery (R1 Pro chassis)...", _DISCOVERY_TIMEOUT_S)
        deadline = time.monotonic() + _DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline:
            time.sleep(0.1)

        self._connected = True
        log.info(
            "R1 Pro chassis adapter connected (Gate 1 feedback=%s)", self._gate1_received
        )
        return True

    def _setup_sensor_streams(self, qos: Any) -> None:
        """Subscribe to all chassis sensor topics and create LCM transports.

        Sensor subscriptions run in a **separate rclpy context** (separate DDS
        participant) so that control traffic (arm commands / Gate 1 at 100 Hz)
        cannot saturate the shared DDS receive threads and cause large camera
        messages (which require UDP fragmentation) to be silently dropped.
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
            from sensor_msgs.msg import Imu as RosImu
            from sensor_msgs.msg import PointCloud2 as RosPointCloud2
        except ImportError:
            log.warning("sensor_msgs not available — chassis sensor streams disabled")
            return

        from dimos.msgs.sensor_msgs.Imu import Imu
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        hw_id = self._hardware_id

        # --- Isolated DDS participant for sensors ---
        self._sensor_context = Context()
        rclpy.init(context=self._sensor_context)
        self._sensor_node = RclpyNode(
            f"r1pro_{hw_id}_sensors",
            context=self._sensor_context,
        )
        # MultiThreadedExecutor: concurrent callbacks for multiple camera streams
        self._sensor_executor = MultiThreadedExecutor(
            num_threads=4,
            context=self._sensor_context,
        )
        self._sensor_executor.add_node(self._sensor_node)

        # --- Start off-spin-thread decode workers ---
        self._sensor_stop.clear()
        self._sensor_workers.clear()

        def _start(name: str, target: Any) -> None:
            t = threading.Thread(target=target, daemon=True, name=f"r1pro_chassis_{name}")
            t.start()
            self._sensor_workers.append(t)

        # rclpy calls callbacks with (msg) only — wrap to match our (msg, _topic) signature.

        # --- RGB cameras (CompressedImage) ---
        for name, ros_topic in _CAMERA_TOPICS.items():
            transport = LCMTransport(f"/r1pro/{hw_id}/{name}", Image)
            self._camera_transports[name] = transport
            cam_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._camera_qs[name] = cam_q
            self._sensor_node.create_subscription(
                CompressedImage, ros_topic,
                lambda msg, n=name: self._on_camera(n, msg, None),
                qos,
            )
            _start(name, partial(self._camera_decode_loop, name))

        # --- Head depth (uncompressed Image, 32FC1) ---
        self._depth_transport = LCMTransport(f"/r1pro/{hw_id}/head_depth", Image)
        self._sensor_node.create_subscription(
            RosImage, "/hdas/camera_head/depth/depth_registered",
            lambda msg: self._on_head_depth(msg, None), qos,
        )
        _start("depth", self._depth_decode_loop)

        # --- LiDAR (PointCloud2, ~10 Hz) ---
        self._lidar_transport = LCMTransport(f"/r1pro/{hw_id}/lidar", PointCloud2)
        self._sensor_node.create_subscription(
            RosPointCloud2, "/hdas/lidar_chassis_left",
            lambda msg: self._on_lidar(msg, None), qos,
        )
        _start("lidar", self._lidar_decode_loop)

        # --- IMUs (~100 Hz each) ---
        self._imu_chassis_transport = LCMTransport(f"/r1pro/{hw_id}/imu_chassis", Imu)
        self._sensor_node.create_subscription(
            RosImu, "/hdas/imu_chassis",
            lambda msg: self._on_imu_chassis(msg, None), qos,
        )
        _start("imu_chassis", self._imu_chassis_loop)

        self._imu_torso_transport = LCMTransport(f"/r1pro/{hw_id}/imu_torso", Imu)
        self._sensor_node.create_subscription(
            RosImu, "/hdas/imu_torso",
            lambda msg: self._on_imu_torso(msg, None), qos,
        )
        _start("imu_torso", self._imu_torso_loop)

        # Spin sensor executor in a background thread.
        # Use spin_once in a loop so that any callback exception is caught and
        # logged rather than killing the entire spin thread, and the loop exits
        # promptly when _sensor_stop is set.
        sensor_stop = self._sensor_stop
        sensor_executor = self._sensor_executor
        sensor_context = self._sensor_context

        def _run_sensor_spin() -> None:
            log.info("R1 Pro %s chassis sensor spin thread started", hw_id)
            while not sensor_stop.is_set() and sensor_context.ok():
                try:
                    sensor_executor.spin_once(timeout_sec=0.1)
                except Exception as exc:
                    # A shut-down rclpy context raises on every spin_once —
                    # don't hot-loop, surface it once and exit.
                    if not sensor_context.ok() or "context is not valid" in str(exc):
                        log.warning(
                            "R1 Pro %s chassis sensor context invalidated, exiting spin loop: %s",
                            hw_id, exc,
                        )
                        break
                    log.warning(
                        "R1 Pro %s chassis sensor executor exception (continuing): %s",
                        hw_id, exc, exc_info=True,
                    )
            log.info("R1 Pro %s chassis sensor spin thread stopped", hw_id)

        self._sensor_spin_thread = threading.Thread(
            target=_run_sensor_spin,
            daemon=True,
            name=f"r1pro_{hw_id}_sensor_spin",
        )
        self._sensor_spin_thread.start()

        log.info(
            "R1 Pro chassis: %d camera streams, lidar, 2x IMU → /r1pro/%s/* "
            "(isolated DDS participant)",
            len(_CAMERA_TOPICS),
            hw_id,
        )

    def disconnect(self) -> None:
        """Disconnect from the R1 Pro chassis."""
        if self._ros and self._connected:
            try:
                self._publish_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass

        # Signal the sensor spin loop and all decode workers to stop first —
        # must happen before executor.shutdown() so the spin_once loop exits
        # cleanly rather than spinning on a shutting-down executor.
        self._sensor_stop.set()

        # Unsubscribe Gate 1 (actuation subscriptions via self._ros)
        for unsub in self._unsubs:
            try:
                unsub()
            except Exception:
                pass
        self._unsubs.clear()

        # Stop sensor executor (separate DDS participant)
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
        for cam_q in self._camera_qs.values():
            try:
                cam_q.put_nowait(None)
            except queue.Full:
                pass
        for q in (self._depth_q, self._lidar_q, self._imu_chassis_q, self._imu_torso_q):
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for t in self._sensor_workers:
            t.join(timeout=1.0)
        self._sensor_workers.clear()
        self._camera_qs.clear()

        if self._ros:
            self._ros.stop()
            self._ros = None

        self._connected = False
        self._enabled = False
        log.info("R1 Pro chassis adapter disconnected")

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def get_dof(self) -> int:
        return self._dof

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_velocities(self) -> list[float]:
        with self._lock:
            return list(self._last_velocities)

    def read_odometry(self) -> list[float] | None:
        # R1 Pro chassis feedback is wheel-level (hdas_msg/MotorControl).
        # Full swerve kinematics deferred — return None for now.
        return None

    def read_enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write_velocities(self, velocities: list[float]) -> bool:
        """Command chassis velocities [vx, vy, wz]."""
        if not self._ros or not self._connected:
            return False
        if len(velocities) != self._dof:
            log.warning("Expected %d velocities, got %d", self._dof, len(velocities))
            return False

        vx, vy, wz = velocities
        self._publish_velocity(vx, vy, wz)

        with self._lock:
            self._last_velocities = list(velocities)
        return True

    def write_stop(self) -> bool:
        return self.write_velocities([0.0, 0.0, 0.0])

    def write_enable(self, enable: bool) -> bool:
        if not self._ros:
            return False

        from std_msgs.msg import Bool

        self._ros.publish(self._brake_topic, Bool(data=not enable))
        self._enabled = enable
        return True

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    # ROS callbacks — enqueue only, never block the spin thread

    def _enqueue(self, q: "queue.Queue[Any]", item: Any) -> None:
        """Put item in queue, replacing stale data if full."""
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            q.put_nowait(item)

    def _on_camera(self, name: str, msg: Any, _topic: Any) -> None:
        # Enqueue message object directly — no bytes copy on the spin thread.
        # bytes(msg.data) is deferred to the worker.
        cam_q = self._camera_qs.get(name)
        if cam_q is not None:
            self._camera_cb_counts[name] = self._camera_cb_counts.get(name, 0) + 1
            self._enqueue(cam_q, msg)

    def _on_head_depth(self, msg: Any, _topic: Any) -> None:
        if self._depth_transport is not None:
            self._depth_cb_count += 1
            self._enqueue(self._depth_q, msg)

    def _on_lidar(self, msg: Any, _topic: Any) -> None:
        if self._lidar_transport is not None:
            self._lidar_cb_count += 1
            self._enqueue(self._lidar_q, msg)

    def _on_imu_chassis(self, msg: Any, _topic: Any) -> None:
        if self._imu_chassis_transport is not None:
            self._enqueue(self._imu_chassis_q, msg)

    def _on_imu_torso(self, msg: Any, _topic: Any) -> None:
        if self._imu_torso_transport is not None:
            self._enqueue(self._imu_torso_q, msg)

    # Worker loops — decode/convert off the spin thread

    def _camera_decode_loop(self, name: str) -> None:
        import cv2
        import numpy as np
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        cam_q = self._camera_qs.get(name)
        if cam_q is None:
            return
        frame_count = 0
        cb_count_last = 0
        last_log = time.monotonic()
        while not self._sensor_stop.is_set():
            try:
                msg = cam_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                # bytes() copy deferred here, off the spin thread
                data = bytes(msg.data)
                arr = np.frombuffer(data, np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                img = Image(bgr, format=ImageFormat.BGR,
                            frame_id=f"{self._hardware_id}_{name}")
                t = self._camera_transports.get(name)
                if t:
                    t.broadcast(None, img)
                    frame_count += 1
            except Exception:
                log.exception("R1 Pro chassis camera %s decode error", name)
            now = time.monotonic()
            if now - last_log >= 5.0:
                cb_now = self._camera_cb_counts.get(name, 0)
                log.info(
                    "R1 Pro chassis %s: %d callbacks, %d frames broadcast in last %.0fs",
                    name, cb_now - cb_count_last, frame_count, now - last_log,
                )
                frame_count = 0
                cb_count_last = cb_now
                last_log = now

    def _depth_decode_loop(self) -> None:
        from dimos.msgs.sensor_msgs.Image import Image
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._depth_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                img = ros_to_dimos(msg, Image)
                if self._depth_transport:
                    self._depth_transport.broadcast(None, img)
            except Exception:
                log.exception("R1 Pro chassis depth decode error")

    def _lidar_decode_loop(self) -> None:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._lidar_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                pc = ros_to_dimos(msg, PointCloud2)
                if self._lidar_transport:
                    self._lidar_transport.broadcast(None, pc)
            except Exception:
                log.exception("R1 Pro chassis lidar decode error")

    def _imu_chassis_loop(self) -> None:
        from dimos.msgs.sensor_msgs.Imu import Imu
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._imu_chassis_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                imu = ros_to_dimos(msg, Imu)
                if self._imu_chassis_transport:
                    self._imu_chassis_transport.broadcast(None, imu)
            except Exception:
                log.exception("R1 Pro chassis IMU decode error")

    def _imu_torso_loop(self) -> None:
        from dimos.msgs.sensor_msgs.Imu import Imu
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        while not self._sensor_stop.is_set():
            try:
                msg = self._imu_torso_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                imu = ros_to_dimos(msg, Imu)
                if self._imu_torso_transport:
                    self._imu_torso_transport.broadcast(None, imu)
            except Exception:
                log.exception("R1 Pro chassis torso IMU decode error")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _publish_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Publish velocity command + Gate 3 (acc_limit) + brake release."""
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Bool

        now = self._ros._node.get_clock().now().to_msg()

        acc = TwistStamped()
        acc.header.stamp = now
        acc.twist.linear.x = _ACC_LIMIT_X
        acc.twist.linear.y = _ACC_LIMIT_Y
        acc.twist.angular.z = _ACC_LIMIT_YAW
        self._ros.publish(self._acc_topic, acc)

        self._ros.publish(self._brake_topic, Bool(data=False))

        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.angular.z = wz
        self._ros.publish(self._speed_topic, cmd)

    def _on_chassis_speed(self, msg: Any, _topic: Any) -> None:
        """Gate 1 callback — records that the IK pipeline is active."""
        if not self._gate1_received:
            log.info("R1 Pro chassis Gate 1 active (IK output received)")
            self._gate1_received = True


# ------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register R1 Pro chassis adapter with the drive train registry."""
    registry.register("r1pro_chassis", R1ProChassisAdapter)


__all__ = ["R1ProChassisAdapter"]
