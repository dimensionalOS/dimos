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

"""
Cartesian Motion Controller

Hardware-agnostic Cartesian space motion controller for robotic manipulators.
Converts Cartesian pose goals to joint commands using IK/FK from the arm driver.

Architecture:
- Subscribes to joint_state and robot_state from hardware driver
- Subscribes to target_pose (PoseStamped) from high-level planners
- Publishes joint_position_command to hardware driver
- Uses PID control for smooth Cartesian tracking
- Supports velocity-based and position-based control modes
"""

import math
import threading
import time
from typing import Any

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.control.arm_driver_spec import ArmDriverSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry import pose_matrix, quaternion_euler, quaternion_from_euler
from dimos.msgs.sensor_msgs.JointCommand import JointCommand
from dimos.msgs.sensor_msgs.RobotState import RobotState
from dimos.msgs.time import time_from_seconds
from dimos.utils.logging_config import setup_logger
from dimos.utils.simple_controller import PIDController

logger = setup_logger()


class CartesianMotionControllerConfig(ModuleConfig):
    """Configuration for Cartesian motion controller."""

    # Control loop parameters
    control_frequency: float = 20.0  # Hz - Cartesian control loop rate
    command_timeout: float = 30.0  # seconds - timeout for stale targets (RPC mode needs longer)

    # PID gains for position control (m/s per meter of error)
    position_kp: float = 5.0  # Proportional gain
    position_ki: float = 0.1  # Integral gain
    position_kd: float = 0.1  # Derivative gain

    # PID gains for orientation control (rad/s per radian of error)
    orientation_kp: float = 2.0  # Proportional gain
    orientation_ki: float = 0.0  # Integral gain
    orientation_kd: float = 0.2  # Derivative gain

    # Safety limits
    max_linear_velocity: float = 0.2  # m/s - maximum TCP linear velocity
    max_angular_velocity: float = 1.0  # rad/s - maximum TCP angular velocity
    max_position_error: float = 0.7  # m - max allowed position error before emergency stop
    max_orientation_error: float = 6.28  # rad (~360°) - allow any orientation

    # Convergence thresholds
    position_tolerance: float = 0.001  # m - position considered "reached"
    orientation_tolerance: float = 0.01  # rad (~0.57°) - orientation considered "reached"

    # Control mode
    velocity_control_mode: bool = True  # Use velocity control (True) or position steps (False)

    # Frame configuration
    control_frame: str = "world"  # Frame for target poses (world, base_link, etc.)


class CartesianMotionController(Module):
    """
    Hardware-agnostic Cartesian motion controller.

    This controller provides Cartesian space motion control for manipulators by:
    1. Receiving target poses (PoseStamped)
    2. Computing Cartesian error (position + orientation)
    3. Generating Cartesian velocity commands (Twist)
    4. Computing IK to convert to joint space
    5. Publishing joint commands to the driver

    The controller is hardware-agnostic: it works with any arm driver that
    provides IK/FK RPC methods and JointState/RobotState outputs.
    """

    config: CartesianMotionControllerConfig

    _arm_driver: ArmDriverSpec

    # Input topics (initialized by Module base class)
    joint_state: In[JointState] = None  # type: ignore[assignment]
    robot_state: In[RobotState] = None  # type: ignore[assignment]
    target_pose: In[PoseStamped] = None  # type: ignore[assignment]

    # Output topics (initialized by Module base class)
    joint_position_command: Out[JointCommand] = None  # type: ignore[assignment]
    cartesian_velocity: Out[Twist] = None  # type: ignore[assignment]
    current_pose: Out[PoseStamped] = None  # type: ignore[assignment]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # State tracking
        self._latest_joint_state: JointState | None = None
        self._latest_robot_state: RobotState | None = None
        self._target_pose_: PoseStamped | None = None
        self._last_target_time: float = 0.0

        # Current TCP pose (computed via FK)
        self._current_tcp_pose: Pose | None = None

        # Thread management
        self._control_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # State locks
        self._state_lock = threading.Lock()
        self._target_lock = threading.Lock()

        # PID controllers for Cartesian space
        self._pid_x = PIDController(
            kp=self.config.position_kp,
            ki=self.config.position_ki,
            kd=self.config.position_kd,
            output_limits=(-self.config.max_linear_velocity, self.config.max_linear_velocity),
        )
        self._pid_y = PIDController(
            kp=self.config.position_kp,
            ki=self.config.position_ki,
            kd=self.config.position_kd,
            output_limits=(-self.config.max_linear_velocity, self.config.max_linear_velocity),
        )
        self._pid_z = PIDController(
            kp=self.config.position_kp,
            ki=self.config.position_ki,
            kd=self.config.position_kd,
            output_limits=(-self.config.max_linear_velocity, self.config.max_linear_velocity),
        )

        # Orientation PIDs (using axis-angle representation)
        self._pid_roll = PIDController(
            kp=self.config.orientation_kp,
            ki=self.config.orientation_ki,
            kd=self.config.orientation_kd,
            output_limits=(-self.config.max_angular_velocity, self.config.max_angular_velocity),
        )
        self._pid_pitch = PIDController(
            kp=self.config.orientation_kp,
            ki=self.config.orientation_ki,
            kd=self.config.orientation_kd,
            output_limits=(-self.config.max_angular_velocity, self.config.max_angular_velocity),
        )
        self._pid_yaw = PIDController(
            kp=self.config.orientation_kp,
            ki=self.config.orientation_ki,
            kd=self.config.orientation_kd,
            output_limits=(-self.config.max_angular_velocity, self.config.max_angular_velocity),
        )

        # Control status
        self._is_tracking: bool = False
        self._last_convergence_check: float = 0.0

        logger.info(
            f"CartesianMotionController initialized at {self.config.control_frequency}Hz "
            f"(velocity_mode={self.config.velocity_control_mode})"
        )

    def _call_fk(self, joint_positions: list[float]) -> tuple[int, list[float] | None]:
        """Call FK via the arm driver spec (resolved at blueprint build time)."""
        return self._arm_driver.get_forward_kinematics(joint_positions)

    def _call_ik(self, pose: list[float]) -> tuple[int, list[float] | None]:
        """Call IK via the arm driver spec (resolved at blueprint build time)."""
        return self._arm_driver.get_inverse_kinematics(pose)

    @rpc
    def start(self) -> None:
        """Start the Cartesian motion controller."""
        super().start()

        # Subscribe to input topics
        # Note: Accessing .connection property triggers transport resolution from connected streams
        try:
            if self.joint_state.connection is not None or self.joint_state._transport is not None:
                self.joint_state.subscribe(self._on_joint_state)
                logger.info("Subscribed to joint_state")
        except Exception as e:
            logger.warning(f"Failed to subscribe to joint_state: {e}")

        try:
            if self.robot_state.connection is not None or self.robot_state._transport is not None:
                self.robot_state.subscribe(self._on_robot_state)
                logger.info("Subscribed to robot_state")
        except Exception as e:
            logger.warning(f"Failed to subscribe to robot_state: {e}")

        try:
            if self.target_pose.connection is not None or self.target_pose._transport is not None:
                self.target_pose.subscribe(self._on_target_pose)
                logger.info("Subscribed to target_pose")
        except Exception:
            logger.debug("target_pose not connected (expected - uses RPC)")

        # Start control loop thread
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="cartesian_control_thread"
        )
        self._control_thread.start()

        logger.info("CartesianMotionController started")

    @rpc
    def stop(self) -> None:
        """Stop the Cartesian motion controller."""
        logger.info("Stopping CartesianMotionController...")

        # Signal thread to stop
        self._stop_event.set()

        # Wait for control thread
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()
        logger.info("CartesianMotionController stopped")

    @rpc
    def set_target_pose(
        self, position: list[float], orientation: list[float], frame_id: str = "world"
    ) -> None:
        """
        Set a target Cartesian pose for the controller to track.

        Args:
            position: [x, y, z] in meters
            orientation: [qx, qy, qz, qw] quaternion OR [roll, pitch, yaw] euler angles
            frame_id: Reference frame for the pose
        """
        # Detect if orientation is euler (3 elements) or quaternion (4 elements)
        if len(orientation) == 3:
            # Convert euler to quaternion using Pose's built-in conversion
            euler_angles = Vector3(x=orientation[0], y=orientation[1], z=orientation[2])
            quat = quaternion_from_euler(euler_angles.x, euler_angles.y, euler_angles.z)
            orientation = [quat.x, quat.y, quat.z, quat.w]

        target = PoseStamped(
            header=Header(frame_id=frame_id, stamp=time_from_seconds(time.time())),
            pose=Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(
                    x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
                ),
            ),
        )

        with self._target_lock:
            self._target_pose_ = target
            self._last_target_time = time.time()
            self._is_tracking = True

        logger.info(
            f"New target set: pos=[{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}] m, "
            f"frame={frame_id}"
        )

    @rpc
    def clear_target(self) -> None:
        """Clear the current target (stop tracking)."""
        with self._target_lock:
            self._target_pose_ = None
            self._is_tracking = False
        logger.info("Target cleared, tracking stopped")

    @rpc
    def get_current_pose(self) -> Pose | None:
        """
        Get the current TCP pose (computed via FK).

        Returns:
            Current Pose or None if not available
        """
        return self._current_tcp_pose

    @rpc
    def is_converged(self) -> bool:
        """
        Check if the controller has converged to the target.

        Returns:
            True if within tolerance, False otherwise
        """
        with self._target_lock:
            target_pose = self._target_pose_

        current_pose = self._current_tcp_pose

        if not target_pose or not current_pose:
            return False

        pos_error, ori_error = self._compute_pose_error(current_pose, target_pose.pose)
        return (
            pos_error < self.config.position_tolerance
            and ori_error < self.config.orientation_tolerance
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Callback when new joint state is received."""
        logger.debug(f"Received joint_state: {len(msg.position)} joints")
        with self._state_lock:
            self._latest_joint_state = msg

    def _on_robot_state(self, msg: RobotState) -> None:
        """Callback when new robot state is received."""
        with self._state_lock:
            self._latest_robot_state = msg

    def _on_target_pose(self, msg: PoseStamped) -> None:
        """Callback when new target pose is received."""
        with self._target_lock:
            self._target_pose_ = msg
            self._last_target_time = time.time()
            self._is_tracking = True
        logger.debug(f"New target received: {msg}")

    def _control_loop(self) -> None:
        """
        Main control loop running at control_frequency Hz.

        Algorithm:
        1. Read current joint state
        2. Compute FK to get current TCP pose
        3. Compute Cartesian error to target
        4. Generate Cartesian velocity command (PID)
        5. Integrate velocity to get next desired pose
        6. Compute IK to get target joint angles
        7. Publish joint command
        """
        period = 1.0 / self.config.control_frequency
        next_time = time.time()

        logger.info(f"Cartesian control loop started at {self.config.control_frequency}Hz")

        while not self._stop_event.is_set():
            # Sleep at start of loop to maintain frequency even when using continue
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                if self._stop_event.wait(timeout=sleep_time):
                    break
            else:
                # Loop overrun - reset timing
                next_time = time.time()

            next_time += period

            try:
                current_time = time.time()
                dt = period  # Use fixed timestep for consistent control

                # Read shared state
                with self._state_lock:
                    joint_state = self._latest_joint_state

                with self._target_lock:
                    target_pose = self._target_pose_
                    last_target_time = self._last_target_time
                    is_tracking = self._is_tracking

                # Check if we have valid state
                if joint_state is None or len(joint_state.position) == 0:
                    continue

                # Compute current TCP pose via FK
                code, current_pose_list = self._call_fk(list(joint_state.position))

                if code != 0 or current_pose_list is None:
                    logger.warning(f"FK failed with code: {code}")
                    continue

                # Convert FK result to Pose (xArm returns [x, y, z, roll, pitch, yaw] in mm)
                if len(current_pose_list) == 6:
                    # Convert position from mm to m for internal use
                    position_m = [
                        current_pose_list[0] / 1000.0,
                        current_pose_list[1] / 1000.0,
                        current_pose_list[2] / 1000.0,
                    ]
                    euler_angles = Vector3(
                        x=current_pose_list[3], y=current_pose_list[4], z=current_pose_list[5]
                    )
                    quat = quaternion_from_euler(euler_angles.x, euler_angles.y, euler_angles.z)
                    self._current_tcp_pose = Pose(
                        position=Point(x=position_m[0], y=position_m[1], z=position_m[2]),
                        orientation=Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w),
                    )

                    # Publish current pose for target setters to use
                    current_pose_stamped = PoseStamped(
                        header=Header(frame_id="world", stamp=time_from_seconds(current_time)),
                        pose=Pose(
                            position=Point(x=position_m[0], y=position_m[1], z=position_m[2]),
                            orientation=Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w),
                        ),
                    )
                    self.current_pose.publish(current_pose_stamped)
                else:
                    logger.warning(f"Unexpected FK result format: {current_pose_list}")
                    continue

                # Check for target timeout
                if is_tracking and (current_time - last_target_time) > self.config.command_timeout:
                    logger.warning("Target pose timeout - clearing target")
                    with self._target_lock:
                        self._target_pose_ = None
                        self._is_tracking = False
                    continue

                # If not tracking, skip control
                if not is_tracking or target_pose is None:
                    logger.debug(
                        f"Not tracking: is_tracking={is_tracking}, target_pose={target_pose is not None}"
                    )
                    continue

                # Check if we have current pose
                if self._current_tcp_pose is None:
                    logger.warning("No current TCP pose available, skipping control")
                    continue

                # Compute Cartesian error
                pos_error_mag, ori_error_mag = self._compute_pose_error(
                    self._current_tcp_pose, target_pose.pose
                )

                # Log error periodically (every 1 second)
                if not hasattr(self, "_last_error_log_time"):
                    self._last_error_log_time = 0.0
                if current_time - self._last_error_log_time > 1.0:
                    logger.info(
                        f"Curr=[{self._current_tcp_pose.position.x:.3f},{self._current_tcp_pose.position.y:.3f},{self._current_tcp_pose.position.z:.3f}]m Tgt=[{target_pose.pose.position.x:.3f},{target_pose.pose.position.y:.3f},{target_pose.pose.position.z:.3f}]m Err={pos_error_mag * 1000:.1f}mm"
                    )
                    self._last_error_log_time = current_time

                # Safety check: excessive error
                if pos_error_mag > self.config.max_position_error:
                    logger.error(
                        f"Position error too large: {pos_error_mag:.3f}m > "
                        f"{self.config.max_position_error}m - STOPPING"
                    )
                    with self._target_lock:
                        self._target_pose_ = None
                        self._is_tracking = False
                    continue

                if ori_error_mag > self.config.max_orientation_error:
                    logger.error(
                        f"Orientation error too large: {ori_error_mag:.3f}rad > "
                        f"{self.config.max_orientation_error}rad - STOPPING"
                    )
                    with self._target_lock:
                        self._target_pose_ = None
                        self._is_tracking = False
                    continue

                # Check convergence periodically
                if current_time - self._last_convergence_check > 1.0:
                    if (
                        pos_error_mag < self.config.position_tolerance
                        and ori_error_mag < self.config.orientation_tolerance
                    ):
                        logger.info(
                            f"Converged! pos_err={pos_error_mag * 1000:.2f}mm, "
                            f"ori_err={math.degrees(ori_error_mag):.2f}°"
                        )
                    self._last_convergence_check = current_time

                # Generate Cartesian velocity command
                cartesian_twist = self._compute_cartesian_velocity(
                    self._current_tcp_pose, target_pose.pose, dt
                )

                # Publish debug twist
                if self.cartesian_velocity._transport or hasattr(
                    self.cartesian_velocity, "connection"
                ):
                    try:
                        self.cartesian_velocity.publish(cartesian_twist)
                    except Exception:
                        pass

                # Integrate velocity to get next desired pose
                next_pose = self._integrate_velocity(self._current_tcp_pose, cartesian_twist, dt)

                # Compute IK to get target joint angles
                # Convert Pose to xArm format: [x, y, z, roll, pitch, yaw]
                # Note: xArm IK expects position in mm, so convert from m to mm
                next_pose_list = [
                    next_pose.position.x * 1000.0,  # m to mm
                    next_pose.position.y * 1000.0,  # m to mm
                    next_pose.position.z * 1000.0,  # m to mm
                    quaternion_euler(next_pose.orientation)[0],
                    quaternion_euler(next_pose.orientation)[1],
                    quaternion_euler(next_pose.orientation)[2],
                ]

                logger.debug(
                    f"Calling IK for pose (mm): [{next_pose_list[0]:.1f}, {next_pose_list[1]:.1f}, {next_pose_list[2]:.1f}]"
                )
                code, target_joints = self._call_ik(next_pose_list)

                if code != 0 or target_joints is None:
                    logger.warning(f"IK failed with code: {code}, target_joints={target_joints}")
                    continue

                logger.debug(f"IK successful: {len(target_joints)} joints")

                # Dynamically get joint count from actual joint_state (works for xarm5/6/7)
                # IK may return extra values (e.g., gripper), so truncate to match actual DOF
                num_arm_joints = len(joint_state.position)
                if len(target_joints) > num_arm_joints:
                    if not hasattr(self, "_ik_truncation_logged"):
                        logger.info(
                            f"IK returns {len(target_joints)} joints, using first {num_arm_joints} to match arm DOF"
                        )
                        self._ik_truncation_logged = True
                    target_joints = target_joints[:num_arm_joints]
                elif len(target_joints) < num_arm_joints:
                    logger.warning(
                        f"IK returns {len(target_joints)} joints but arm has {num_arm_joints} - joint count mismatch!"
                    )

                # Publish joint command
                joint_cmd = JointCommand(
                    timestamp=current_time,
                    positions=list(target_joints),
                )

                # Always try to publish - the Out stream will handle transport availability
                try:
                    self.joint_position_command.publish(joint_cmd)
                    logger.debug(
                        f"✓ Pub cmd: [{target_joints[0]:.6f}, {target_joints[1]:.6f}, {target_joints[2]:.6f}, ...]"
                    )
                except Exception as e:
                    logger.error(f"✗ Failed to publish joint command: {e}")

            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                import traceback

                traceback.print_exc()

        logger.info("Cartesian control loop stopped")

    def _compute_pose_error(self, current_pose: Pose, target_pose: Pose) -> tuple[float, float]:
        """
        Compute position and orientation error between current and target pose.

        Args:
            current_pose: Current TCP pose
            target_pose: Desired TCP pose

        Returns:
            Tuple of (position_error_magnitude, orientation_error_magnitude)
        """
        return compute_pose_error(pose_matrix(current_pose), pose_matrix(target_pose))

    def _compute_cartesian_velocity(
        self, current_pose: Pose, target_pose: Pose, dt: float
    ) -> Twist:
        """
        Compute Cartesian velocity command using PID control.

        Args:
            current_pose: Current TCP pose
            target_pose: Desired TCP pose
            dt: Time step

        Returns:
            Twist message with linear and angular velocities
        """
        # Position error
        error_x = target_pose.position.x - current_pose.position.x
        error_y = target_pose.position.y - current_pose.position.y
        error_z = target_pose.position.z - current_pose.position.z

        # Compute linear velocities via PID
        vel_x = self._pid_x.update(error_x, dt)  # type: ignore[no-untyped-call]
        vel_y = self._pid_y.update(error_y, dt)  # type: ignore[no-untyped-call]
        vel_z = self._pid_z.update(error_z, dt)  # type: ignore[no-untyped-call]

        # Orientation error (convert to euler for simpler PID)
        # This is an approximation; axis-angle would be more accurate
        error_roll = self._normalize_angle(
            quaternion_euler(target_pose.orientation)[0]
            - quaternion_euler(current_pose.orientation)[0]
        )
        error_pitch = self._normalize_angle(
            quaternion_euler(target_pose.orientation)[1]
            - quaternion_euler(current_pose.orientation)[1]
        )
        error_yaw = self._normalize_angle(
            quaternion_euler(target_pose.orientation)[2]
            - quaternion_euler(current_pose.orientation)[2]
        )

        # Compute angular velocities via PID
        omega_x = self._pid_roll.update(error_roll, dt)  # type: ignore[no-untyped-call]
        omega_y = self._pid_pitch.update(error_pitch, dt)  # type: ignore[no-untyped-call]
        omega_z = self._pid_yaw.update(error_yaw, dt)  # type: ignore[no-untyped-call]

        return Twist(
            linear=Vector3(x=vel_x, y=vel_y, z=vel_z),
            angular=Vector3(x=omega_x, y=omega_y, z=omega_z),
        )

    def _integrate_velocity(self, current_pose: Pose, velocity: Twist, dt: float) -> Pose:
        """
        Integrate Cartesian velocity to compute next desired pose.

        Args:
            current_pose: Current TCP pose
            velocity: Desired Cartesian velocity (Twist)
            dt: Time step

        Returns:
            Next desired pose
        """
        # Integrate position (simple Euler integration)
        next_position = Vector3(
            x=current_pose.position.x + velocity.linear.x * dt,
            y=current_pose.position.y + velocity.linear.y * dt,
            z=current_pose.position.z + velocity.linear.z * dt,
        )

        # Integrate orientation (simple euler integration - good for small dt)
        next_roll = quaternion_euler(current_pose.orientation)[0] + velocity.angular.x * dt
        next_pitch = quaternion_euler(current_pose.orientation)[1] + velocity.angular.y * dt
        next_yaw = quaternion_euler(current_pose.orientation)[2] + velocity.angular.z * dt

        euler_angles = Vector3(x=next_roll, y=next_pitch, z=next_yaw)
        next_orientation = quaternion_from_euler(euler_angles.x, euler_angles.y, euler_angles.z)

        return Pose(
            position=Point(x=next_position.x, y=next_position.y, z=next_position.z),
            orientation=Quaternion(
                x=next_orientation.x,
                y=next_orientation.y,
                z=next_orientation.z,
                w=next_orientation.w,
            ),
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
