#!/usr/bin/env python3
"""
Force-Torque Pull Skill Module for Dimos

This module implements continuous adaptive door opening with force-torque feedback.
It receives force-torque data from the FT driver module via LCM and performs
continuous pull + rotation motions to open doors while maintaining desired force.

Based on continuous_door_opener.py but as a Dimos skill module.
"""

import numpy as np
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple
import os

from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import Vector3
from dimos.utils.logging_config import setup_logger
from dimos.protocol.skill import skill

from pydrake.all import (
    MultibodyPlant,
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    StartMeshcat,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    DoDifferentialInverseKinematics,
    DifferentialInverseKinematicsStatus,
    DifferentialInverseKinematicsParameters,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)

logger = setup_logger(__name__)


@dataclass
class ForceState:
    """Container for force sensor state with history."""
    force: np.ndarray
    torque: np.ndarray
    timestamp: float

    def lateral_force(self) -> float:
        """Get lateral force magnitude (x,y components only)."""
        return np.linalg.norm(self.force[:2])

    def x_force(self) -> float:
        """Get x-component of force."""
        return self.force[0]


class FTPullModule(Module):
    """Force-Torque based pull skill module for door opening."""

    # Input ports for force-torque data
    force: In[Vector3] = None  # Force vector in Newtons
    torque: In[Vector3] = None  # Torque vector in Newton-meters

    def __init__(
        self,
        xarm_ip: str = None,
        enable_real_robot: bool = True,
        verbose: bool = False
    ):
        """
        Initialize the FT pull module.

        Args:
            xarm_ip: IP address of the xARM robot
            enable_real_robot: Whether to control real robot
            verbose: Enable verbose output
        """
        super().__init__()

        self.xarm_ip = xarm_ip
        self.enable_real_robot = enable_real_robot
        self.verbose = verbose

        # Force-torque data storage
        self.latest_force = None
        self.latest_torque = None
        self.force_lock = threading.Lock()

        # xARM connection
        self.arm = None
        self.xarm_initial_positions = None

        # Drake simulation components
        self.meshcat = None
        self.plant = None
        self.plant_context = None
        self.openft_frame = None
        self.openft_body = None
        self.diagram = None
        self.diagram_context = None
        self.diff_ik_params = None
        self.gripper_position = None

        # Control parameters (will be set by skill)
        self.pivot_distance = 0.2
        self.force_threshold = 7.0
        self.rotation_gain = 0.01
        self.pull_speed = 0.015
        self.sensor_delay = 0.2  # 200ms delay
        self.prediction_gain = 0.3  # How much to trust force rate prediction

        # Motion limits
        self.max_rotation_per_step = 0.2  # 11.5 degrees max per step
        self.min_rotation_per_step = 0.005  # 0.3 degrees min (deadband)

        # Force history for prediction
        self.force_history = deque(maxlen=20)  # Store more history for rate estimation
        self.last_motion_time = time.time()
        self.motion_history = deque(maxlen=10)  # Store recent motions for prediction

        # Rotation damping to prevent oscillation
        self.last_rotation_direction = 0  # Track last rotation sign
        self.rotation_history = deque(maxlen=5)  # Track recent rotations
        self.oscillation_damping = 0.5  # Reduce rotation when oscillating

        # Tracking variables
        self.total_rotation = 0.0
        self.total_pull_distance = 0.0
        self.motion_count = 0

        # Control flags
        self.running = False
        self.stop_requested = False

    @rpc
    def start(self):
        """Start the module and subscribe to inputs."""
        logger.info("Starting FT Pull module")

        # Subscribe to force and torque data
        if self.force:
            self.force.subscribe(self.handle_force)
            logger.info("Subscribed to force stream")

        if self.torque:
            self.torque.subscribe(self.handle_torque)
            logger.info("Subscribed to torque stream")

        # Initialize xARM if configured
        if self.xarm_ip:
            self.init_xarm()

        # Always initialize Drake simulation with Meshcat
        self.setup_drake_simulation()

    def handle_force(self, msg: Vector3):
        """Handle incoming force data."""
        with self.force_lock:
            self.latest_force = np.array([msg.x, msg.y, msg.z])
            if self.verbose:
                logger.debug(f"Force: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}] N")

    def handle_torque(self, msg: Vector3):
        """Handle incoming torque data."""
        with self.force_lock:
            self.latest_torque = np.array([msg.x, msg.y, msg.z])
            if self.verbose:
                logger.debug(f"Torque: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}] N⋅m")

    def init_xarm(self):
        """Initialize xARM connection."""
        if not self.xarm_ip:
            return

        try:
            from xarm.wrapper import XArmAPI

            logger.info(f"Connecting to xARM at {self.xarm_ip}")

            # Get initial positions
            self.xarm_initial_positions = self.get_xarm_positions()

            # Initialize API
            self.arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)  # Position control mode
            self.arm.set_state(0)  # Ready state

            logger.info("xARM initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize xARM: {e}")
            self.arm = None

    def get_xarm_positions(self):
        """Get current joint positions from xARM robot."""
        try:
            from xarm.wrapper import XArmAPI

            arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            arm.clean_error()
            arm.clean_warn()

            code, angles = arm.get_servo_angle(is_radian=True)

            if code == 0 and angles:
                logger.info("Got xARM joint positions:")
                for i, angle in enumerate(angles[:6]):
                    logger.info(f"  joint{i+1}: {np.degrees(angle):.2f} deg")

                # Try to get gripper position
                try:
                    code_gripper, gripper_pos = arm.get_gripper_position()
                    if code_gripper == 0:
                        logger.info(f"  gripper: {gripper_pos:.1f} mm")
                        result = list(angles[:6])
                        result.append(gripper_pos / 1000.0)
                        arm.disconnect()
                        return result
                except:
                    pass

                arm.disconnect()
                return angles[:6]

            arm.disconnect()
        except Exception as e:
            logger.error(f"Error getting xARM positions: {e}")

        return None

    def setup_drake_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        try:
            # Always start meshcat for visualization
            self.meshcat = StartMeshcat()
            logger.info(f"Meshcat URL: {self.meshcat.web_url()}")
            # Clear meshcat
            self.meshcat.Delete()
            self.meshcat.DeleteAddedControls()

            # Create diagram builder
            builder = DiagramBuilder()

            # Create plant and scene graph
            self.plant, scene_graph = AddMultibodyPlantSceneGraph(
                builder, time_step=0.001
            )

            # Parse URDF
            parser = Parser(self.plant)
            package_path = os.path.dirname(os.path.abspath(__file__))
            parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))

            # Load the URDF
            urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
            model_instances = parser.AddModels(urdf_path)
            self.model_instance = model_instances[0] if model_instances else None

            # Get link_openft frame for control
            try:
                self.openft_frame = self.plant.GetFrameByName("link_openft")
                self.openft_body = self.plant.GetBodyByName("link_openft")
                logger.info("Using link_openft as control frame")
            except:
                logger.error("ERROR: Could not find link_openft frame!")
                raise

            # Finalize the plant
            self.plant.Finalize()

            # Always add visualizer with meshcat
            if self.meshcat:
                visualizer = MeshcatVisualizer.AddToBuilder(
                    builder, scene_graph, self.meshcat
                )

            # Build the diagram
            self.diagram = builder.Build()

            # Create contexts
            self.diagram_context = self.diagram.CreateDefaultContext()
            self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)

            # Set initial robot configuration
            initial_positions = np.zeros(self.plant.num_positions())
            self.gripper_position = None

            if self.xarm_initial_positions is not None:
                logger.info("Initializing Drake with xARM joint positions")
                arm_joint_names = [f"joint{i+1}" for i in range(6)]
                for i, joint_name in enumerate(arm_joint_names):
                    try:
                        joint = self.plant.GetJointByName(joint_name)
                        joint_index = joint.position_start()
                        if i < len(self.xarm_initial_positions):
                            initial_positions[joint_index] = self.xarm_initial_positions[i]
                    except Exception as e:
                        logger.error(f"Error setting {joint_name}: {e}")

                # Check if we have a 7th value for the gripper
                if len(self.xarm_initial_positions) > 6:
                    self.gripper_position = self.xarm_initial_positions[6]
                    logger.info(f"Got gripper position from xARM: {self.gripper_position:.3f}")

            # Set gripper position
            try:
                gripper_joint = self.plant.GetJointByName("drive_joint")
                gripper_index = gripper_joint.position_start()
                if self.gripper_position is not None:
                    initial_positions[gripper_index] = self.gripper_position
                else:
                    self.gripper_position = 0.02
                    initial_positions[gripper_index] = self.gripper_position
                    logger.info(f"Using default gripper position: {self.gripper_position:.3f}")
            except:
                pass

            self.plant.SetPositions(self.plant_context, initial_positions)

            # Setup differential IK parameters
            self.setup_diff_ik()

            # Set camera view
            if self.meshcat:
                self.meshcat.SetCameraPose(
                    camera_in_world=[1.5, 1.5, 1.2],
                    target_in_world=[0.0, 0.0, 0.3]
                )

            # Initial publish
            if self.diagram:
                self.diagram.ForcedPublish(self.diagram_context)

            logger.info("Drake simulation setup complete")
        except Exception as e:
            logger.error(f"Failed to setup Drake simulation: {e}")
            self.meshcat = None
            self.plant = None

    def setup_diff_ik(self):
        """Setup differential IK parameters."""
        if not self.plant:
            return

        self.diff_ik_params = DifferentialInverseKinematicsParameters(
            self.plant.num_positions(),
            self.plant.num_velocities()
        )

        # Set timestep
        dt = 0.05
        self.diff_ik_params.set_time_step(dt)

        # Set joint limits
        q_lower = self.plant.GetPositionLowerLimits()
        q_upper = self.plant.GetPositionUpperLimits()
        self.diff_ik_params.set_joint_position_limits((q_lower, q_upper))

        # Set velocity limits
        v_lower = self.plant.GetVelocityLowerLimits()
        v_upper = self.plant.GetVelocityUpperLimits()
        self.diff_ik_params.set_joint_velocity_limits((v_lower, v_upper))

        # Enable all 6 DOF for end-effector control
        velocity_flag = np.ones(6, dtype=bool)
        self.diff_ik_params.set_end_effector_velocity_flag(velocity_flag)

    def get_current_force_state(self) -> Optional[ForceState]:
        """Get current force-torque state."""
        with self.force_lock:
            if self.latest_force is None:
                return None

            return ForceState(
                force=self.latest_force.copy(),
                torque=self.latest_torque.copy() if self.latest_torque is not None else np.zeros(3),
                timestamp=time.time()
            )

    def predict_current_force(self) -> Optional[ForceState]:
        """
        Predict current force based on history and sensor delay.
        Uses linear extrapolation to compensate for sensor delay.
        """
        if len(self.force_history) < 2:
            return self.force_history[-1] if self.force_history else None

        # Get recent force measurements
        recent = list(self.force_history)[-5:]  # Last 5 measurements
        if len(recent) < 2:
            return recent[-1]

        # Calculate force rate of change
        dt_total = recent[-1].timestamp - recent[0].timestamp
        if dt_total < 0.01:  # Not enough time difference
            return recent[-1]

        # Calculate average force rate
        force_diff = recent[-1].force - recent[0].force
        force_rate = force_diff / dt_total

        # Predict force at current time (accounting for delay)
        time_since_measurement = time.time() - recent[-1].timestamp
        prediction_time = self.sensor_delay - time_since_measurement

        if prediction_time > 0:
            # Extrapolate forward
            predicted_force = recent[-1].force + force_rate * prediction_time * self.prediction_gain
        else:
            # Use most recent measurement
            predicted_force = recent[-1].force

        return ForceState(
            force=predicted_force,
            torque=recent[-1].torque,  # Don't predict torque for now
            timestamp=time.time()
        )

    def compute_combined_motion(self, force_state: ForceState, door_opens_clockwise: bool, rotation_axis: str) -> Tuple[np.ndarray, float]:
        """
        Compute combined pull + rotation motion based on force feedback.
        Matches the exact logic from continuous_door_opener.py.

        Args:
            force_state: Current (or predicted) force state
            door_opens_clockwise: Whether door opens clockwise
            rotation_axis: Axis to rotate around ('x', 'y', or 'z')

        Returns:
            translation: 3D translation vector in world frame
            rotation_angle: Rotation angle around pivot (radians)
        """
        # Get current openft pose
        openft_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        openft_rot = openft_pose.rotation()

        # Extract force components
        force_x = force_state.x_force()
        lateral_force = force_state.lateral_force()

        # Compute rotation based on force error with adaptive gain
        rotation_angle = 0.0

        # Safety check for extreme forces
        if lateral_force > 80:
            if self.verbose:
                logger.info(f"  EXTREME FORCES ({lateral_force:.1f}N)! Rotation only, no pull")
            pull_safety_factor = 0.0
        else:
            pull_safety_factor = 1.0

        # Adaptive rotation gain based on force magnitude
        if lateral_force > 60:
            adaptive_gain = self.rotation_gain * 0.3  # 30% gain at very high forces
        elif lateral_force > 40:
            adaptive_gain = self.rotation_gain * 0.5  # 50% gain at high forces
        elif lateral_force > 20:
            adaptive_gain = self.rotation_gain * 0.7  # 70% gain at medium forces
        elif lateral_force > 10:
            adaptive_gain = self.rotation_gain * 0.9  # 90% gain at low-medium forces
        else:
            adaptive_gain = self.rotation_gain * 1.2  # 120% gain at low forces

        # Check for oscillation
        oscillating = False
        if len(self.rotation_history) >= 2:
            recent_signs = [np.sign(r) for r in self.rotation_history if abs(r) > 0.01]
            if len(recent_signs) >= 2:
                if recent_signs[-1] * recent_signs[-2] < 0:
                    oscillating = True
                    adaptive_gain *= self.oscillation_damping
                    if self.verbose:
                        logger.debug(f"  Oscillation detected, reducing gain to {adaptive_gain:.4f}")

        # Check if in success zone
        in_success_zone = abs(force_x) <= self.force_threshold

        # Determine rotation based on door type and force
        if door_opens_clockwise:
            if force_x > self.force_threshold:
                force_error = force_x - self.force_threshold
                if force_error > 30:
                    rotation_angle = -adaptive_gain * 15 * (1 - np.exp(-force_error/30))
                elif force_error > 15:
                    rotation_angle = -adaptive_gain * force_error * 0.7
                else:
                    rotation_angle = -force_error * adaptive_gain
                if self.verbose:
                    logger.debug(f"  Wrong side! Force {force_x:.1f}N > {self.force_threshold}N, rotating CCW")
            elif force_x < -self.force_threshold:
                force_error = abs(force_x) - self.force_threshold
                if force_error > 8:
                    rotation_angle = +adaptive_gain * min(force_error * 0.2, 5)
                    if self.verbose:
                        logger.debug(f"  Optimizing: Force {force_x:.1f}N, gentle CW rotation")
        else:
            if force_x < -self.force_threshold:
                force_error = abs(force_x) - self.force_threshold
                if force_error > 20:
                    rotation_angle = adaptive_gain * 20 * (1 - np.exp(-force_error/20))
                else:
                    rotation_angle = force_error * adaptive_gain
                if self.verbose:
                    logger.debug(f"  Wrong side! Force {force_x:.1f}N < -{self.force_threshold}N, rotating CW")
            elif force_x > self.force_threshold:
                force_error = force_x - self.force_threshold
                if force_error > 5:
                    rotation_angle = -adaptive_gain * min(force_error * 0.3, 10)
                    if self.verbose:
                        logger.debug(f"  Optimizing: Force {force_x:.1f}N, gentle CCW rotation")

        # Apply rotation limits with adaptive maximum
        if lateral_force > 50:
            max_rotation = min(self.max_rotation_per_step * 0.4, 0.08)
        elif lateral_force > 30:
            max_rotation = min(self.max_rotation_per_step * 0.6, 0.12)
        else:
            max_rotation = self.max_rotation_per_step

        rotation_angle = np.clip(rotation_angle, -max_rotation, max_rotation)

        # Store rotation in history
        self.rotation_history.append(rotation_angle)

        # Apply deadband
        if abs(rotation_angle) < self.min_rotation_per_step:
            rotation_angle = 0.0

        # Compute pull component with force-adaptive strategy
        if in_success_zone and lateral_force < 10:
            pull_distance = self.pull_speed * 1.5  # 150% speed when aligned
            if self.verbose:
                logger.debug(f"  Aligned! Boosting pull to {pull_distance*1000:.1f}mm")
        elif lateral_force < 15:
            pull_distance = self.pull_speed
        elif lateral_force < 25:
            pull_distance = self.pull_speed * 0.8
        elif lateral_force < 40:
            pull_distance = self.pull_speed * 0.6
            if self.verbose:
                logger.debug(f"  High forces ({lateral_force:.1f}N), reducing pull to 60%")
        elif lateral_force < 60:
            pull_distance = self.pull_speed * 0.4
            if self.verbose:
                logger.debug(f"  Very high forces ({lateral_force:.1f}N), reducing pull to 40%")
        else:
            pull_distance = self.pull_speed * 0.2
            if self.verbose:
                logger.debug(f"  Extreme forces ({lateral_force:.1f}N), minimal pull 20%")

        # Further reduce pull if rotating heavily
        if abs(rotation_angle) > self.max_rotation_per_step * 0.5:
            rotation_penalty = 0.7
            pull_distance *= rotation_penalty

        # Apply safety factor
        pull_distance *= pull_safety_factor

        # Create pull vector in openft local frame (-z direction)
        pull_local = np.array([0.0, 0.0, -pull_distance])

        # Transform to world frame
        translation = openft_rot @ pull_local

        return translation, rotation_angle

    def compute_target_pose(self, translation: np.ndarray, rotation_angle: float, rotation_axis: str) -> RigidTransform:
        """
        Compute target pose combining translation and rotation around pivot.

        Args:
            translation: Translation vector in world frame
            rotation_angle: Rotation angle around pivot point
            rotation_axis: Axis to rotate around ('x', 'y', or 'z')

        Returns:
            Target pose for link_openft
        """
        # Get current openft pose
        current_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        current_pos = current_pose.translation()
        current_rot = current_pose.rotation()

        # Compute pivot point in world frame
        pivot_offset_local = np.array([0.0, 0.0, self.pivot_distance])
        pivot_point_world = current_pos + current_rot @ pivot_offset_local

        # Apply rotation around pivot if needed
        if abs(rotation_angle) > 0.001:
            # Create rotation matrix based on selected axis
            c = np.cos(rotation_angle)
            s = np.sin(rotation_angle)

            if rotation_axis == 'x':
                rotation_matrix = np.array([
                    [1, 0, 0],
                    [0, c, -s],
                    [0, s, c]
                ])
            elif rotation_axis == 'y':
                rotation_matrix = np.array([
                    [c, 0, s],
                    [0, 1, 0],
                    [-s, 0, c]
                ])
            else:  # 'z'
                rotation_matrix = np.array([
                    [c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]
                ])

            # Vector from pivot to current openft position
            pivot_to_openft = current_pos - pivot_point_world

            # Rotate this vector
            new_pivot_to_openft = rotation_matrix @ pivot_to_openft

            # New position after rotation
            rotated_position = pivot_point_world + new_pivot_to_openft

            # Also rotate the orientation
            rotated_orientation = RotationMatrix(rotation_matrix @ current_rot.matrix())
        else:
            rotated_position = current_pos
            rotated_orientation = current_rot

        # Apply translation after rotation
        target_position = rotated_position + translation

        return RigidTransform(rotated_orientation, target_position)

    def execute_motion(self, target_pose: RigidTransform, speed: float = 0.5) -> bool:
        """
        Execute motion using differential IK.
        """
        if not self.plant or not self.diff_ik_params:
            return False

        # Use differential IK to move towards target
        result = DoDifferentialInverseKinematics(
            self.plant,
            self.plant_context,
            target_pose,
            self.openft_frame,
            self.diff_ik_params
        )

        if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
            # Get current positions
            q_current = self.plant.GetPositions(self.plant_context)

            # Integrate velocities
            v_sol = result.joint_velocities
            dt = 0.05  # Timestep
            q_new = q_current + v_sol.flatten() * dt

            # Apply joint limits
            q_lower = self.plant.GetPositionLowerLimits()
            q_upper = self.plant.GetPositionUpperLimits()
            q_new = np.clip(q_new, q_lower, q_upper)

            # Preserve gripper position
            try:
                gripper_joint = self.plant.GetJointByName("drive_joint")
                gripper_index = gripper_joint.position_start()
                if self.gripper_position is not None:
                    q_new[gripper_index] = self.gripper_position
            except:
                pass

            # Set new positions in simulation
            self.plant.SetPositions(self.plant_context, q_new)

            # Command real robot if enabled
            if self.enable_real_robot and self.arm:
                self.command_xarm(speed)

            return True
        else:
            return False

    def command_xarm(self, speed):
        """Send commands to xARM."""
        if not self.enable_real_robot or not self.arm:
            return

        # Get current joint positions from simulation
        q = self.plant.GetPositions(self.plant_context)

        arm_joint_names = [f"joint{i+1}" for i in range(6)]
        positions = []
        for joint_name in arm_joint_names:
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            positions.append(q[joint_idx])

        # Send command with higher speed
        code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)
        if code != 0:
            logger.error(f"Error commanding xARM: code {code}, attempting recovery")
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.set_state(0)
            self.arm.set_mode(0)
            # Retry command once
            code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)
            if code != 0:
                logger.error(f"Retry failed: code {code}")

    @skill()
    def continuous_pull(self,
                       pivot_distance: float = 0.2,
                       force_threshold: float = 7.0,
                       rotation_gain: float = 0.01,
                       pull_speed: float = 0.015,
                       sensor_delay: float = 0.2,
                       prediction_gain: float = 0.3,
                       door_opens_clockwise: bool = True,  # Most doors open clockwise
                       rotation_axis: str = 'z',
                       max_duration: float = 30.0,
                       end_angle: Optional[float] = None) -> str:
        """
        Execute continuous adaptive door pulling with force feedback.
        Matches the exact control logic from continuous_door_opener.py.

        Args:
            pivot_distance: Distance to virtual pivot point (m)
            force_threshold: Target force threshold (N)
            rotation_gain: Rotation speed per Newton of force error (rad/N)
            pull_speed: Pull distance per step (m)
            sensor_delay: Expected sensor delay (seconds)
            prediction_gain: Weight for predictive compensation (0-1)
            door_opens_clockwise: Whether door opens clockwise
            rotation_axis: Axis to rotate around ('x', 'y', or 'z')
            max_duration: Maximum duration in seconds
            end_angle: Optional maximum rotation angle in degrees

        Returns:
            Status message about the operation
        """
        logger.info(f"Starting continuous adaptive door pulling")
        logger.info(f"  Force threshold: {force_threshold}N")
        logger.info(f"  Rotation gain: {rotation_gain:.4f} rad/N")
        logger.info(f"  Pull speed: {pull_speed*1000:.1f}mm per step")
        logger.info(f"  Door type: {'CLOCKWISE' if door_opens_clockwise else 'COUNTER-CLOCKWISE'}")
        logger.info(f"  Rotation axis: {rotation_axis.upper()}-axis")

        # Set control parameters
        self.pivot_distance = pivot_distance
        self.force_threshold = force_threshold
        self.rotation_gain = rotation_gain
        self.pull_speed = pull_speed
        self.sensor_delay = sensor_delay
        self.prediction_gain = prediction_gain

        # Validate rotation axis
        if rotation_axis.lower() not in ['x', 'y', 'z']:
            logger.error(f"Invalid rotation_axis '{rotation_axis}'. Must be 'x', 'y', or 'z'")
            return "ERROR: Invalid rotation axis"
        rotation_axis = rotation_axis.lower()

        # Reset tracking
        self.total_rotation = 0.0
        self.total_pull_distance = 0.0
        self.motion_count = 0
        self.force_history.clear()
        self.rotation_history.clear()
        self.motion_history.clear()

        # Convert end angle to radians if provided
        end_angle_rad = np.radians(end_angle) if end_angle is not None else None
        if end_angle_rad:
            logger.info(f"  End angle: {end_angle}° (will stop at this rotation)")

        # Start time
        start_time = time.time()
        last_print_time = time.time()
        control_rate = 25  # Hz
        dt = 1.0 / control_rate

        # Wait for initial force data
        logger.info("Waiting for force-torque sensor data...")
        wait_start = time.time()
        while self.get_current_force_state() is None:
            if time.time() - wait_start > 5:
                logger.error("No force data received after 5 seconds")
                return "ERROR: No force data available"
            time.sleep(0.01)

        # Let initial readings stabilize
        logger.info("Stabilizing force readings...")
        for _ in range(10):
            force_state = self.get_current_force_state()
            if force_state:
                self.force_history.append(force_state)
            time.sleep(0.02)

        logger.info("Starting continuous motion...")

        # Control loop
        self.running = True
        self.stop_requested = False

        while self.running:
            loop_start = time.time()

            # Check duration limit
            if time.time() - start_time > max_duration:
                logger.info(f"Reached maximum duration of {max_duration}s")
                break

            # Check rotation limit
            if end_angle_rad and abs(self.total_rotation) >= end_angle_rad:
                logger.info(f"Target angle reached! Total rotation: {np.degrees(abs(self.total_rotation)):.1f}°")
                break

            # Get current force state
            force_state = self.get_current_force_state()
            if force_state is None:
                logger.warning("No force data available, waiting...")
                time.sleep(0.1)
                continue

            # Store in history for prediction
            self.force_history.append(force_state)

            # Use prediction to compensate for sensor delay
            predicted_force = self.predict_current_force()
            force_to_use = predicted_force if predicted_force else force_state

            # Compute combined pull + rotation motion
            translation, rotation_angle = self.compute_combined_motion(
                force_to_use, door_opens_clockwise, rotation_axis
            )

            # Update statistics
            self.total_rotation += rotation_angle
            self.total_pull_distance += np.linalg.norm(translation)

            # Print status periodically
            if time.time() - last_print_time > 0.5:  # Every 500ms
                logger.info(
                    f"[Step {self.motion_count}] "
                    f"Force: x={force_to_use.x_force():.1f}N, "
                    f"lateral={force_to_use.lateral_force():.1f}N | "
                    f"Rot: {np.degrees(rotation_angle):.1f}° "
                    f"(total: {np.degrees(self.total_rotation):.1f}°) | "
                    f"Pull: {np.linalg.norm(translation)*1000:.1f}mm "
                    f"(total: {self.total_pull_distance*100:.1f}cm)"
                )
                last_print_time = time.time()

            # Execute motion if we have Drake simulation and significant motion
            if self.plant and (np.linalg.norm(translation) > 0.001 or abs(rotation_angle) > 0.001):
                # Compute target pose
                target_pose = self.compute_target_pose(translation, rotation_angle, rotation_axis)

                # Update visualizations if meshcat available
                if self.meshcat:
                    # Create target pose visualization if needed
                    if not hasattr(self, 'visualizations_created') or not self.visualizations_created:
                        self.create_target_frame_visualization()
                        self.visualizations_created = True
                    # Update target pose visualization
                    self.meshcat.SetTransform("target_pose", target_pose)

                # Execute motion with higher speed
                success = self.execute_motion(target_pose, speed=0.8)
                if success:
                    self.motion_count += 1
                    self.last_motion_time = time.time()

                    # Store motion in history for prediction
                    self.motion_history.append({
                        'translation': translation,
                        'rotation': rotation_angle,
                        'timestamp': time.time()
                    })

                # Update visualization
                if self.diagram:
                    self.diagram.ForcedPublish(self.diagram_context)
            elif self.arm and not self.plant:
                # Direct xARM control without Drake (fallback)
                code, current_angles = self.arm.get_servo_angle(is_radian=True)
                if code == 0:
                    # Apply rotation to appropriate joint
                    if rotation_axis == 'z':
                        current_angles[5] += rotation_angle
                    elif rotation_axis == 'y':
                        current_angles[4] += rotation_angle
                    elif rotation_axis == 'x':
                        current_angles[3] += rotation_angle

                    # Execute movement
                    code = self.arm.set_servo_angle(angle=current_angles[:6], speed=15, wait=True, is_radian=True)
                    if code == 0:
                        self.motion_count += 1
                        self.last_motion_time = time.time()
            else:
                # Simulation only - just track the motion
                self.motion_count += 1
                self.last_motion_time = time.time()

            # Maintain control rate
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

            # Check stop request
            if self.stop_requested:
                logger.info("Stop requested")
                break

        self.running = False

        # Print final statistics
        logger.info("="*60)
        logger.info("Final Statistics:")
        logger.info(f"  Total motion steps: {self.motion_count}")
        logger.info(f"  Total rotation: {np.degrees(self.total_rotation):.1f} degrees")
        logger.info(f"  Total pull distance: {self.total_pull_distance*100:.1f} cm")
        if self.motion_count > 0:
            logger.info(f"  Average rate: {self.motion_count/(time.time()-start_time):.1f} Hz")
        logger.info("="*60)

        return f"Completed: {self.motion_count} steps, {np.degrees(self.total_rotation):.1f}°, {self.total_pull_distance*100:.1f}cm"

    @skill()
    def stop_pull(self) -> str:
        """Stop the continuous pull operation."""
        self.stop_requested = True
        self.running = False
        return "Pull operation stopped"

    @rpc
    def get_stats(self) -> dict:
        """Get current statistics."""
        force_state = self.get_current_force_state()
        return {
            'has_force_data': force_state is not None,
            'lateral_force': force_state.lateral_force() if force_state else 0,
            'total_rotation_deg': np.degrees(self.total_rotation),
            'total_pull_cm': self.total_pull_distance * 100,
            'motion_count': self.motion_count,
            'running': self.running
        }

    def create_target_frame_visualization(self):
        """Create visualization for target pose."""
        if not self.meshcat:
            return

        axis_length = 0.12
        axis_radius = 0.004

        # X-axis (red)
        self.meshcat.SetObject(
            "target_pose/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(0.8, 0.2, 0.2, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )

        # Y-axis (green)
        self.meshcat.SetObject(
            "target_pose/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.2, 0.8, 0.2, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )

        # Z-axis (blue)
        self.meshcat.SetObject(
            "target_pose/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.2, 0.2, 0.8, 0.7)
        )
        self.meshcat.SetTransform(
            "target_pose/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )

    def cleanup(self):
        """Clean up resources."""
        if self.arm:
            self.arm.disconnect()
            logger.info("xARM connection closed")