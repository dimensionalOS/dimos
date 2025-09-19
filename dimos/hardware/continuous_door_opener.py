#!/usr/bin/env python3
"""
Continuous Adaptive Door Opening Controller with Force-Torque Feedback

This controller continuously pulls while adaptively rotating based on force feedback.
Key features:
- Combined pull + rotation in each motion step
- Proportional rotation control based on force error
- Compensation for force sensor delay (~200ms)
- Continuous motion without discrete states
"""

import numpy as np
import argparse
import time
import lcm
import zmq
import json
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple
from lcm_msgs.sensor_msgs import JointState
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
import os
from xarm.wrapper import XArmAPI


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


class ContinuousDoorOpener:
    def __init__(
        self, 
        xarm_ip=None, 
        enable_real_robot=True,
        pivot_distance=0.2,
        force_threshold=7.0,  # More stable threshold
        rotation_gain=0.01,  # Moderate rotation gain
        pull_speed=0.015,  # 15mm base pull (can boost when aligned)
        sensor_delay=0.2,  # 200ms delay
        prediction_gain=0.3,  # How much to trust force rate prediction
        door_opens_clockwise=True,  # Door opening direction
        rotation_axis='z'  # Rotation axis: 'x', 'y', or 'z'
    ):
        """
        Initialize the continuous door opener controller.
        
        Args:
            xarm_ip: IP address of the xARM robot
            enable_real_robot: Whether to publish commands to real robot
            pivot_distance: Distance to virtual pivot point (m)
            force_threshold: Target force threshold (N)
            rotation_gain: Rotation speed per Newton of force error (rad/N)
            pull_speed: Pull distance per step (m)
            sensor_delay: Expected sensor delay (seconds)
            prediction_gain: Weight for predictive compensation (0-1)
            door_opens_clockwise: Whether door opens clockwise (True) or counter-clockwise (False)
            rotation_axis: Axis to rotate around ('x', 'y', or 'z' in world frame)
        """
        
        # Start meshcat
        self.meshcat = StartMeshcat()
        
        # Control parameters
        self.xarm_ip = xarm_ip
        self.enable_real_robot = enable_real_robot
        self.pivot_distance = pivot_distance
        self.force_threshold = force_threshold
        self.rotation_gain = rotation_gain
        self.pull_speed = pull_speed
        self.sensor_delay = sensor_delay
        self.prediction_gain = prediction_gain
        
        # Validate and store rotation axis
        if rotation_axis.lower() not in ['x', 'y', 'z']:
            raise ValueError(f"Invalid rotation_axis '{rotation_axis}'. Must be 'x', 'y', or 'z'")
        self.rotation_axis = rotation_axis.lower()
        
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
        
        # Door parameters
        self.door_opens_clockwise = door_opens_clockwise
        
        # Statistics tracking
        self.total_rotation = 0.0
        self.total_pull_distance = 0.0
        self.motion_count = 0
        
        # Get initial xARM positions
        self.xarm_initial_positions = None
        if self.xarm_ip:
            print(f"\n{'='*60}")
            print(f"Connecting to xARM at {self.xarm_ip}")
            print(f"{'='*60}")
            self.xarm_initial_positions = self.get_xarm_positions()
            if self.xarm_initial_positions is None:
                print("WARNING: Failed to get xARM positions, using default")
                print(f"{'='*60}\n")
        
        # Initialize xARM connection
        if self.xarm_ip:
            self.arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)  # Position control mode
            self.arm.set_state(0)  # Set to ready state
        else:
            self.arm = None
        
        # Initialize LCM for robot control
        if self.enable_real_robot:
            self.lc = lcm.LCM()
            self.joint_state_msg = JointState()
            print("LCM initialized for real robot control")
        
        # Initialize ZMQ for force-torque data
        self.setup_force_torque_receiver()
        
        # Setup Drake simulation
        self.setup_simulation()
        
        # Setup differential IK
        self.setup_diff_ik()
        
        # Create visualizations
        self.create_visualizations()
        
        # Initial publish
        self.diagram.ForcedPublish(self.diagram_context)
        
        print(f"\n{'='*60}")
        print(f"Meshcat URL: {self.meshcat.web_url()}")
        print(f"Open this URL in your browser to view the robot")
        print(f"{'='*60}\n")
    
    def get_xarm_positions(self):
        """Get current joint positions from xARM robot."""
        try:
            from xarm.wrapper import XArmAPI
            
            print(f"Getting xARM joint positions...")
            arm = XArmAPI(self.xarm_ip, do_not_open=False, is_radian=True)
            
            # Clear any errors
            arm.clean_error()
            arm.clean_warn()
            
            # Get current joint angles (6 DOF)
            code, angles = arm.get_servo_angle(is_radian=True)
            
            if code == 0 and angles:
                print(f"Got xARM joint positions:")
                for i, angle in enumerate(angles[:6]):
                    print(f"  joint{i+1}: {angle:.4f} rad ({np.degrees(angle):.2f} deg)")
                
                # Try to get gripper position
                try:
                    code_gripper, gripper_pos = arm.get_gripper_position()
                    if code_gripper == 0:
                        gripper_rad = gripper_pos / 1000.0  # Rough conversion
                        print(f"  gripper: {gripper_pos:.1f} mm (~{gripper_rad:.3f} rad)")
                        result = list(angles[:6])
                        result.append(gripper_rad)
                        arm.disconnect()
                        return result
                except:
                    pass
                
                arm.disconnect()
                return angles[:6]
            else:
                print(f"Failed to get xARM positions, code: {code}")
            
            arm.disconnect()
            
        except ImportError:
            print("Error: xarm library not installed")
        except Exception as e:
            print(f"Error connecting to xARM: {e}")
        
        return None
    
    def setup_force_torque_receiver(self):
        """Setup ZMQ subscriber for force-torque data."""
        try:
            self.zmq_context = zmq.Context()
            self.ft_socket = self.zmq_context.socket(zmq.SUB)
            self.ft_socket.connect("tcp://localhost:5556")  # Port for calibrated data
            self.ft_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self.ft_socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout
            print("Connected to force-torque data stream on port 5556")
        except Exception as e:
            print(f"Warning: Failed to setup force-torque receiver: {e}")
            self.ft_socket = None
    
    def get_force_torque_data(self) -> Optional[ForceState]:
        """Get latest force-torque sensor data with timestamp."""
        if not self.ft_socket:
            return None
        
        latest_data = None
        try:
            # Drain socket to get latest message
            while True:
                try:
                    data_str = self.ft_socket.recv_string(zmq.NOBLOCK)
                    data = json.loads(data_str)
                    if 'forces' in data and 'torques' in data:
                        latest_data = data
                except zmq.Again:
                    break
                except json.JSONDecodeError as e:
                    print(f"Error decoding force-torque JSON: {e}")
        except Exception as e:
            print(f"Error getting force-torque data: {e}")
        
        if latest_data:
            state = ForceState(
                force=np.array(latest_data['forces']),
                torque=np.array(latest_data['torques']),
                timestamp=time.time()
            )
            self.force_history.append(state)
            return state
        
        return None
    
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
    
    def compute_combined_motion(self, force_state: ForceState) -> Tuple[np.ndarray, float]:
        """
        Compute combined pull + rotation motion based on force feedback.
        
        Args:
            force_state: Current (or predicted) force state
            
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
            print(f"  EXTREME FORCES ({lateral_force:.1f}N)! Rotation only, no pull")
            # Only rotate, no pulling when forces are dangerously high
            pull_safety_factor = 0.0
        else:
            pull_safety_factor = 1.0
        
        # Adaptive rotation gain based on force magnitude
        # Much gentler at high forces to avoid overshoot
        if lateral_force > 60:
            adaptive_gain = self.rotation_gain * 0.3  # 30% gain at very high forces
        elif lateral_force > 40:
            adaptive_gain = self.rotation_gain * 0.5  # 50% gain at high forces
        elif lateral_force > 20:
            adaptive_gain = self.rotation_gain * 0.7  # 70% gain at medium forces
        elif lateral_force > 10:
            adaptive_gain = self.rotation_gain * 0.9  # 90% gain at low-medium forces
        else:
            adaptive_gain = self.rotation_gain * 1.2  # 120% gain at low forces (be aggressive)
        
        # Check for oscillation by looking at rotation history
        oscillating = False
        if len(self.rotation_history) >= 2:
            # Check if we're switching rotation direction frequently
            recent_signs = [np.sign(r) for r in self.rotation_history if abs(r) > 0.01]
            if len(recent_signs) >= 2:
                # If last two rotations were in opposite directions, we're oscillating
                if recent_signs[-1] * recent_signs[-2] < 0:
                    oscillating = True
                    adaptive_gain *= self.oscillation_damping
                    print(f"  Oscillation detected, reducing gain to {adaptive_gain:.4f}")
        
        # Check if we're in the "success zone" (good alignment)
        in_success_zone = abs(force_x) <= self.force_threshold
        
        # Determine if we're on the wrong side of the force threshold
        wrong_side = False
        if self.door_opens_clockwise:
            # For clockwise door: positive force means we need to rotate counter-clockwise
            if force_x > self.force_threshold:
                wrong_side = True
                force_error = force_x - self.force_threshold
                # Use gentler exponential scaling for large errors
                if force_error > 30:
                    # Strong exponential damping for very large errors
                    rotation_angle = -adaptive_gain * 15 * (1 - np.exp(-force_error/30))
                elif force_error > 15:
                    # Moderate damping
                    rotation_angle = -adaptive_gain * force_error * 0.7
                else:
                    rotation_angle = -force_error * adaptive_gain
                print(f"  Wrong side! Force {force_x:.1f}N > {self.force_threshold}N, rotating CCW")
            elif force_x < -self.force_threshold:
                # On correct side but force magnitude is high
                force_error = abs(force_x) - self.force_threshold
                # Only correct if force is significantly negative
                if force_error > 8:  # Only rotate if more than 8N below threshold
                    # Very gentle correction when optimizing
                    rotation_angle = +adaptive_gain * min(force_error * 0.2, 5)
                    print(f"  Optimizing: Force {force_x:.1f}N, gentle CW rotation")
            elif in_success_zone and lateral_force < 15:
                # In success zone with low lateral forces - minimal adjustment
                print(f"  Success zone! Force {force_x:.1f}N, minimal adjustment")
        else:
            # For counter-clockwise door: negative force means rotate clockwise
            if force_x < -self.force_threshold:
                wrong_side = True
                force_error = abs(force_x) - self.force_threshold
                if force_error > 20:
                    rotation_angle = adaptive_gain * 20 * (1 - np.exp(-force_error/20))
                else:
                    rotation_angle = force_error * adaptive_gain
                print(f"  Wrong side! Force {force_x:.1f}N < -{self.force_threshold}N, rotating CW")
            elif force_x > self.force_threshold:
                force_error = force_x - self.force_threshold
                if force_error > 5:
                    rotation_angle = -adaptive_gain * min(force_error * 0.3, 10)
                    print(f"  Optimizing: Force {force_x:.1f}N, gentle CCW rotation")
        
        # Apply rotation limits with adaptive maximum based on force
        if lateral_force > 50:
            max_rotation = min(self.max_rotation_per_step * 0.4, 0.08)  # Max 4.6 degrees at very high force
        elif lateral_force > 30:
            max_rotation = min(self.max_rotation_per_step * 0.6, 0.12)  # Max 6.9 degrees at high force
        else:
            max_rotation = self.max_rotation_per_step
        
        rotation_angle = np.clip(rotation_angle, -max_rotation, max_rotation)
        
        # Store rotation in history
        self.rotation_history.append(rotation_angle)
        
        # Apply deadband to avoid jitter
        if abs(rotation_angle) < self.min_rotation_per_step:
            rotation_angle = 0.0
        
        # Compute pull component with force-adaptive strategy
        # Boost when aligned, reduce when fighting the door
        if in_success_zone and lateral_force < 10:
            # Perfect alignment - boost pull speed!
            pull_distance = self.pull_speed * 1.5  # 150% speed
            print(f"  Aligned! Boosting pull to {pull_distance*1000:.1f}mm")
        elif lateral_force < 15:
            # Good alignment - normal speed
            pull_distance = self.pull_speed
        elif lateral_force < 25:
            # Moderate forces - slight reduction
            pull_distance = self.pull_speed * 0.8
        elif lateral_force < 40:
            # High forces - significant reduction
            pull_distance = self.pull_speed * 0.6
            print(f"  High forces ({lateral_force:.1f}N), reducing pull to 60%")
        elif lateral_force < 60:
            # Very high forces - major reduction
            pull_distance = self.pull_speed * 0.4
            print(f"  Very high forces ({lateral_force:.1f}N), reducing pull to 40%")
        else:
            # Extreme forces - minimal pull
            pull_distance = self.pull_speed * 0.2
            print(f"  Extreme forces ({lateral_force:.1f}N), minimal pull 20%")
        
        # Further reduce pull if rotating heavily
        if abs(rotation_angle) > self.max_rotation_per_step * 0.5:
            rotation_penalty = 0.7  # 30% reduction when rotating a lot
            pull_distance *= rotation_penalty
        
        # Apply safety factor (0 when forces > 80N)
        pull_distance *= pull_safety_factor
        
        # Create pull vector in openft local frame (-z direction)
        pull_local = np.array([0.0, 0.0, -pull_distance])
        
        # Transform to world frame
        translation = openft_rot @ pull_local
        
        return translation, rotation_angle
    
    def compute_target_pose(self, translation: np.ndarray, rotation_angle: float) -> RigidTransform:
        """
        Compute target pose combining translation and rotation around pivot.
        
        Args:
            translation: Translation vector in world frame
            rotation_angle: Rotation angle around pivot point
            
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
            
            if self.rotation_axis == 'x':
                # Rotation around X-axis (pitch)
                rotation_matrix = np.array([
                    [1, 0, 0],
                    [0, c, -s],
                    [0, s, c]
                ])
            elif self.rotation_axis == 'y':
                # Rotation around Y-axis (roll)
                rotation_matrix = np.array([
                    [c, 0, s],
                    [0, 1, 0],
                    [-s, 0, c]
                ])
            else:  # self.rotation_axis == 'z'
                # Rotation around Z-axis (yaw)
                rotation_matrix = np.array([
                    [c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]
                ])
            
            # Vector from pivot to current openft position
            pivot_to_openft = current_pos - pivot_point_world
            
            # Rotate this vector around selected axis
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
    
    def setup_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()
        
        # Create diagram builder
        self.builder = DiagramBuilder()
        
        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.001
        )
        
        # Parse URDF
        parser = Parser(self.plant)
        package_path = os.path.dirname(os.path.abspath(__file__))
        parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))
        
        # Load the URDF
        urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
        self.model_instances = parser.AddModels(urdf_path)
        self.model_instance = self.model_instances[0] if self.model_instances else None
        
        # Get link_openft frame for control
        try:
            self.openft_frame = self.plant.GetFrameByName("link_openft")
            self.openft_body = self.plant.GetBodyByName("link_openft")
            print("Using link_openft as control frame")
        except:
            print("ERROR: Could not find link_openft frame!")
            raise
        
        # Finalize the plant
        self.plant.Finalize()
        
        # Add visualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat
        )
        
        # Build the diagram
        self.diagram = self.builder.Build()
        
        # Create contexts
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)
        
        # Get joint names
        self.arm_joint_names = [f"joint{i+1}" for i in range(6)]
        self.gripper_joint_name = "drive_joint"
        
        # Set initial positions
        initial_positions = np.zeros(self.plant.num_positions())
        
        # Store gripper position separately
        self.gripper_position = None
        
        # Use xARM positions if available
        if self.xarm_initial_positions is not None:
            print("\nInitializing Drake with xARM joint positions")
            for i, joint_name in enumerate(self.arm_joint_names):
                try:
                    joint = self.plant.GetJointByName(joint_name)
                    joint_index = joint.position_start()
                    if i < len(self.xarm_initial_positions):
                        initial_positions[joint_index] = self.xarm_initial_positions[i]
                except Exception as e:
                    print(f"  Error setting {joint_name}: {e}")
            
            # Check if we have a 7th value for the gripper
            if len(self.xarm_initial_positions) > 6:
                self.gripper_position = self.xarm_initial_positions[6]
                print(f"  Got gripper position from xARM: {self.gripper_position:.3f}")
        
        # Set gripper position
        try:
            gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
            gripper_index = gripper_joint.position_start()
            if self.gripper_position is not None:
                initial_positions[gripper_index] = self.gripper_position
            else:
                self.gripper_position = 0.02
                initial_positions[gripper_index] = self.gripper_position
                print(f"  Using default gripper position: {self.gripper_position:.3f}")
        except:
            pass
        
        self.plant.SetPositions(self.plant_context, initial_positions)
        
        # Reset camera
        self.meshcat.SetCameraPose(
            camera_in_world=[1.5, 1.5, 1.2],
            target_in_world=[0.0, 0.0, 0.3]
        )
    
    def setup_diff_ik(self):
        """Setup differential IK parameters."""
        self.diff_ik_params = DifferentialInverseKinematicsParameters(
            self.plant.num_positions(),
            self.plant.num_velocities()
        )
        
        # Set timestep
        self.dt = 0.05
        self.diff_ik_params.set_time_step(self.dt)
        
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
    
    def create_visualizations(self):
        """Create visualizations for forces, motion, and pivot."""
        # Force vector (red)
        self.meshcat.SetObject(
            "force_vector/arrow",
            Cylinder(0.005, 0.1),
            Rgba(1.0, 0.0, 0.0, 0.8)
        )
        self.meshcat.SetObject(
            "force_vector/head",
            Sphere(0.015),
            Rgba(1.0, 0.0, 0.0, 0.9)
        )
        
        # Motion direction (green)
        self.meshcat.SetObject(
            "motion_direction/arrow",
            Cylinder(0.006, 0.1),
            Rgba(0.0, 1.0, 0.0, 0.8)
        )
        self.meshcat.SetObject(
            "motion_direction/head",
            Sphere(0.018),
            Rgba(0.0, 1.0, 0.0, 0.9)
        )
        
        # Pivot point (yellow)
        self.meshcat.SetObject(
            "pivot_point",
            Sphere(0.02),
            Rgba(1.0, 1.0, 0.0, 0.8)
        )
        
        # Pivot axis (vertical line)
        self.meshcat.SetObject(
            "pivot_axis",
            Cylinder(0.003, 0.3),
            Rgba(1.0, 1.0, 0.0, 0.5)
        )
        
        # Target frame
        self.create_target_frame_visualization()
        
        print("Created visualizations")
    
    def create_target_frame_visualization(self):
        """Create visualization for target pose."""
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
    
    def update_visualizations(self, force_state: ForceState, translation: np.ndarray, rotation_angle: float):
        """Update all visualizations."""
        # Get current openft pose
        openft_pose = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        openft_pos = openft_pose.translation()
        openft_rot = openft_pose.rotation()
        
        # Update pivot point
        pivot_offset_local = np.array([0.0, 0.0, self.pivot_distance])
        pivot_point_world = openft_pos + openft_rot @ pivot_offset_local
        self.meshcat.SetTransform(
            "pivot_point",
            RigidTransform(RotationMatrix(), pivot_point_world)
        )
        self.meshcat.SetTransform(
            "pivot_axis",
            RigidTransform(RotationMatrix(), pivot_point_world)
        )
        
        # Update force vector
        force_world = openft_rot @ force_state.force
        force_scale = 0.02  # 2cm per Newton
        force_magnitude = np.linalg.norm(force_world)
        
        if force_magnitude > 0.1:
            force_direction = force_world / force_magnitude
            force_length = force_magnitude * force_scale
            
            # Position arrow
            arrow_center = openft_pos + force_direction * force_length / 2
            
            # Create rotation for arrow
            z_axis = np.array([0, 0, 1])
            if not np.allclose(force_direction, z_axis):
                axis = np.cross(z_axis, force_direction)
                if np.linalg.norm(axis) > 0.001:
                    axis = axis / np.linalg.norm(axis)
                    angle = np.arccos(np.clip(np.dot(z_axis, force_direction), -1, 1))
                    K = np.array([[0, -axis[2], axis[1]],
                                 [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                    rotation = RotationMatrix(R)
                else:
                    rotation = RotationMatrix()
            else:
                rotation = RotationMatrix()
            
            self.meshcat.SetTransform(
                "force_vector/arrow",
                RigidTransform(rotation, arrow_center)
            )
            
            head_position = openft_pos + force_direction * force_length
            self.meshcat.SetTransform(
                "force_vector/head",
                RigidTransform(RotationMatrix(), head_position)
            )
        
        # Update motion direction
        if np.linalg.norm(translation) > 0.001:
            motion_magnitude = np.linalg.norm(translation)
            motion_unit = translation / motion_magnitude
            motion_length = motion_magnitude * 5  # Scale for visibility
            
            arrow_center = openft_pos + motion_unit * motion_length / 2
            
            # Create rotation for arrow
            z_axis = np.array([0, 0, 1])
            if not np.allclose(motion_unit, z_axis):
                axis = np.cross(z_axis, motion_unit)
                if np.linalg.norm(axis) > 0.001:
                    axis = axis / np.linalg.norm(axis)
                    angle = np.arccos(np.clip(np.dot(z_axis, motion_unit), -1, 1))
                    K = np.array([[0, -axis[2], axis[1]],
                                 [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                    rotation = RotationMatrix(R)
                else:
                    rotation = RotationMatrix()
            else:
                rotation = RotationMatrix()
            
            self.meshcat.SetTransform(
                "motion_direction/arrow",
                RigidTransform(rotation, arrow_center)
            )
            
            head_position = openft_pos + motion_unit * motion_length
            self.meshcat.SetTransform(
                "motion_direction/head",
                RigidTransform(RotationMatrix(), head_position)
            )
    
    def execute_motion(self, target_pose: RigidTransform, speed: float = 0.5) -> bool:
        """Execute motion using differential IK."""
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
            q_new = q_current + v_sol.flatten() * self.dt
            
            # Apply joint limits
            q_lower = self.plant.GetPositionLowerLimits()
            q_upper = self.plant.GetPositionUpperLimits()
            q_new = np.clip(q_new, q_lower, q_upper)
            
            # Preserve gripper position
            try:
                gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                gripper_index = gripper_joint.position_start()
                q_new[gripper_index] = self.gripper_position
            except:
                pass
            
            # Set new positions
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
        
        positions = []
        for joint_name in self.arm_joint_names:
            joint = self.plant.GetJointByName(joint_name)
            joint_idx = joint.position_start()
            positions.append(q[joint_idx])
        
        # Send command with higher speed
        code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)
        if code != 0:
            # Only clean errors if command actually failed
            print(f"Error commanding xARM: code {code}, attempting recovery")
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.set_state(0)
            self.arm.set_mode(0)
            # Retry command once
            code = self.arm.set_servo_angle(angle=positions, speed=speed, wait=True, is_radian=True)
            if code != 0:
                print(f"Retry failed: code {code}")
    
    def run(self):
        """Main continuous control loop."""
        print("\n" + "="*60)
        print("Continuous Adaptive Door Opening Controller")
        print("="*60)
        print("Control Strategy:")
        print("  - Continuous combined pull + rotation")
        print("  - Proportional rotation based on force error")
        print(f"  - Target force threshold: ±{self.force_threshold}N")
        print(f"  - Rotation gain: {self.rotation_gain:.4f} rad/N")
        print(f"  - Pull speed: {self.pull_speed*1000:.1f}mm per step")
        print(f"  - Sensor delay compensation: {self.sensor_delay*1000:.0f}ms")
        print(f"  - Door type: {'CLOCKWISE' if self.door_opens_clockwise else 'COUNTER-CLOCKWISE'}")
        print(f"  - Rotation axis: {self.rotation_axis.upper()}-axis (world frame)")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        # Wait for initial force data
        print("Waiting for force-torque sensor data...")
        while self.get_force_torque_data() is None:
            time.sleep(0.01)
        
        # Let initial readings stabilize
        print("Stabilizing force readings...")
        for _ in range(10):
            self.get_force_torque_data()
            time.sleep(0.02)
        
        print("\nStarting continuous motion...\n")
        
        last_print_time = time.time()
        control_rate = 25  # Hz - reduced for larger steps with less overhead
        dt = 1.0 / control_rate
        
        try:
            while True:
                loop_start = time.time()
                
                # Get latest force data
                force_state = self.get_force_torque_data()
                
                if force_state is not None:
                    # Use prediction to compensate for sensor delay
                    predicted_force = self.predict_current_force()
                    if predicted_force:
                        force_to_use = predicted_force
                    else:
                        force_to_use = force_state
                    
                    # Compute combined motion
                    translation, rotation_angle = self.compute_combined_motion(force_to_use)
                    
                    # Update statistics
                    self.total_rotation += rotation_angle
                    self.total_pull_distance += np.linalg.norm(translation)
                    
                    # Print status periodically
                    if time.time() - last_print_time > 0.5:  # Every 500ms
                        print(f"[Step {self.motion_count}] "
                              f"Force: x={force_to_use.x_force():.1f}N, "
                              f"lateral={force_to_use.lateral_force():.1f}N | "
                              f"Rot: {np.degrees(rotation_angle):.1f}° "
                              f"(total: {np.degrees(self.total_rotation):.1f}°) | "
                              f"Pull: {np.linalg.norm(translation)*1000:.1f}mm "
                              f"(total: {self.total_pull_distance*100:.1f}cm)")
                        last_print_time = time.time()
                    
                    # Compute target pose
                    target_pose = self.compute_target_pose(translation, rotation_angle)
                    
                    # Update visualizations
                    self.update_visualizations(force_to_use, translation, rotation_angle)
                    self.meshcat.SetTransform("target_pose", target_pose)
                    
                    # Execute motion with higher speed
                    if np.linalg.norm(translation) > 0.001 or abs(rotation_angle) > 0.001:
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
                    self.diagram.ForcedPublish(self.diagram_context)
                
                # Maintain control rate
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                    
        except KeyboardInterrupt:
            print("\n\nStopping continuous controller...")
        
        # Print final statistics
        print(f"\n{'='*60}")
        print("Final Statistics:")
        print(f"  Total motion steps: {self.motion_count}")
        print(f"  Total rotation: {np.degrees(self.total_rotation):.1f} degrees")
        print(f"  Total pull distance: {self.total_pull_distance*100:.1f} cm")
        print(f"  Average rate: {self.motion_count/(time.time()-self.last_motion_time+0.001):.1f} Hz")
        print(f"{'='*60}")
        
        # Cleanup
        if self.ft_socket:
            self.ft_socket.close()
            self.zmq_context.term()
        
        if self.arm:
            self.arm.disconnect()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Continuous Adaptive Door Opening with Force-Torque Feedback"
    )
    parser.add_argument(
        "--xarm",
        type=str,
        help="xARM IP address for robot control"
    )
    parser.add_argument(
        "--sim_only",
        action="store_true",
        help="Run in simulation only (don't command real robot)"
    )
    parser.add_argument(
        "--pivot_distance",
        type=float,
        default=0.21,
        help="Distance to virtual pivot point (meters)"
    )
    parser.add_argument(
        "--force_threshold",
        type=float,
        default=10.0,
        help="Target force threshold (N)"
    )
    parser.add_argument(
        "--rotation_gain",
        type=float,
        default=0.01,
        help="Rotation speed per Newton of error (rad/N)"
    )
    parser.add_argument(
        "--pull_speed",
        type=float,
        default=0.015,
        help="Pull distance per step (meters)"
    )
    parser.add_argument(
        "--sensor_delay",
        type=float,
        default=0.2,
        help="Expected sensor delay (seconds)"
    )
    parser.add_argument(
        "--prediction_gain",
        type=float,
        default=0.3,
        help="Weight for predictive force compensation (0-1)"
    )
    parser.add_argument(
        "--clockwise",
        action="store_true",
        help="Door opens clockwise (hinge on left). Default: True"
    )
    parser.add_argument(
        "--counter_clockwise",
        action="store_true",
        help="Door opens counter-clockwise (hinge on right)"
    )
    parser.add_argument(
        "--rotation_axis",
        type=str,
        default='z',
        choices=['x', 'y', 'z'],
        help="Axis to rotate around in world frame (default: z for vertical axis)"
    )
    
    args = parser.parse_args()
    
    # Determine door opening direction
    if args.counter_clockwise:
        door_opens_clockwise = False
    else:
        # Default to clockwise
        door_opens_clockwise = True
    
    # Create and run controller
    controller = ContinuousDoorOpener(
        xarm_ip=args.xarm,
        enable_real_robot=not args.sim_only,
        pivot_distance=args.pivot_distance,
        force_threshold=args.force_threshold,
        rotation_gain=args.rotation_gain,
        pull_speed=args.pull_speed,
        sensor_delay=args.sensor_delay,
        prediction_gain=args.prediction_gain,
        door_opens_clockwise=door_opens_clockwise,
        rotation_axis=args.rotation_axis,
        end_angle=90
    )
    
    controller.run()


if __name__ == "__main__":
    main()