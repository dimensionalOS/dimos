#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""OMPL-based motion planner using Drake environment from visualization script."""

import os
import sys
import time
import numpy as np
from typing import List, Optional

# Add path to import visualization_script
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from visualization_script import DrakeKinematicsEnv

from pydrake.all import (
    RigidTransform,
    RollPitchYaw, 
    RotationMatrix,
)

# OMPL imports
try:
    from ompl import base as ob
    from ompl import geometric as og
    OMPL_AVAILABLE = True
except ImportError:
    print("OMPL not available. Install with: pip install ompl-python")
    OMPL_AVAILABLE = False

class OMPLMotionPlanner:
    """OMPL-based motion planner that interfaces with Drake environment"""
    
    def __init__(self, drake_env: DrakeKinematicsEnv, 
                 collision_threshold: float = 0.01,
                 planning_time: float = 5.0):
        """
        Initialize OMPL motion planner
        
        Args:
            drake_env: Drake kinematics environment
            collision_threshold: Maximum acceptable collision depth  
            planning_time: Planning time limit in seconds
        """
        if not OMPL_AVAILABLE:
            raise ImportError("OMPL not available. Install with: pip install ompl-python")
            
        self.drake_env = drake_env
        self.collision_threshold = collision_threshold
        self.planning_time = planning_time
        
        # Get joint information
        self.joint_limits = drake_env.get_joint_limits()
        self.n_joints = len(self.joint_limits)
        
        print(f"OMPL planner initialized with {self.n_joints} joints")
        for i, (lower, upper) in enumerate(self.joint_limits):
            print(f"  Joint {i}: [{lower:.3f}, {upper:.3f}]")
        
        # Set up OMPL space and problem
        self._setup_ompl()
    
    def _setup_ompl(self):
        """Set up OMPL configuration space and problem"""
        # Create configuration space
        self.space = ob.RealVectorStateSpace(self.n_joints)
        
        # Set joint bounds
        bounds = ob.RealVectorBounds(self.n_joints)
        for i, (lower, upper) in enumerate(self.joint_limits):
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)
        self.space.setBounds(bounds)
        
        # Create space information with validity checker
        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        self.si.setStateValidityCheckingResolution(0.01)  # 1% resolution
        self.si.setup()
        
        # Create problem definition
        self.pdef = ob.ProblemDefinition(self.si)
        
        print("OMPL configuration space set up successfully")
    
    def _is_state_valid(self, state):
        """Check if a state is valid (collision-free enough)"""
        # Convert OMPL state to numpy array - RealVectorStateInternal supports direct indexing
        config = np.array([state[i] for i in range(self.n_joints)])
        
        # Check collision cost using Drake environment
        collision_cost = self.drake_env.check_collision_cost(config)
        
        # State is valid if collision cost is below threshold
        return collision_cost <= self.collision_threshold
    
    def _numpy_to_ompl_state(self, config: np.ndarray):
        """Convert numpy configuration to OMPL state"""
        state = self.space.allocState()
        # RealVectorStateInternal supports direct assignment via indexing
        for i in range(self.n_joints):
            state[i] = float(config[i])
        return state
    
    def _ompl_state_to_numpy(self, state):
        """Convert OMPL state to numpy configuration"""
        # RealVectorStateInternal supports direct indexing
        return np.array([state[i] for i in range(self.n_joints)])
    
    def plan(self, start_config: np.ndarray, goal_config: np.ndarray, 
             planner_type: str = "RRTstar") -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal configuration
        
        Args:
            start_config: Start joint configuration
            goal_config: Goal joint configuration  
            planner_type: OMPL planner type ("RRTstar", "RRTConnect", "PRM", "EST")
            
        Returns:
            List of waypoint configurations if successful, None otherwise
        """
        print(f"Planning with {planner_type} from:")
        print(f"  Start: {start_config}")
        print(f"  Goal:  {goal_config}")
        
        # Validate start and goal states
        start_state = self._numpy_to_ompl_state(start_config)
        goal_state = self._numpy_to_ompl_state(goal_config)
        
        if not self.si.isValid(start_state):
            start_cost = self.drake_env.check_collision_cost(start_config)
            print(f"WARNING: Start state has collision cost {start_cost:.4f} (threshold: {self.collision_threshold})")
            if start_cost > self.collision_threshold * 2:  # Too much collision
                print("Start state has too much collision, aborting")
                return None
        
        if not self.si.isValid(goal_state):
            goal_cost = self.drake_env.check_collision_cost(goal_config)
            print(f"WARNING: Goal state has collision cost {goal_cost:.4f} (threshold: {self.collision_threshold})")
            if goal_cost > self.collision_threshold * 2:  # Too much collision
                print("Goal state has too much collision, aborting")
                return None
        
        # Set start and goal states with tighter tolerance
        self.pdef.setStartAndGoalStates(start_state, goal_state, 0.05)  # 0.05 rad tolerance (tighter)
        
        # Get step size (use dynamic if available, otherwise default)
        step_size = getattr(self, 'current_step_size', 0.3)
        
        # Choose planner
        if planner_type == "RRTstar":
            planner = og.RRTstar(self.si)
            planner.setRange(step_size)
        elif planner_type == "RRTConnect":
            planner = og.RRTConnect(self.si)
            planner.setRange(step_size)
        elif planner_type == "PRM":
            planner = og.PRM(self.si)
            # PRM doesn't use range parameter
        elif planner_type == "EST":
            planner = og.EST(self.si)
            planner.setRange(step_size)
        else:
            print(f"Unknown planner type: {planner_type}, using RRTstar")
            planner = og.RRTstar(self.si)
            planner.setRange(step_size)
        
        planner.setProblemDefinition(self.pdef)
        planner.setup()
        
        print(f"Planning for {self.planning_time} seconds...")
        
        # Solve the planning problem
        solved = planner.solve(self.planning_time)
        
        if solved:
            # Check if this is an exact or approximate solution
            is_approximate = self.pdef.hasApproximateSolution() and not self.pdef.hasExactSolution()
            
            solution_path = self.pdef.getSolutionPath()
            
            if is_approximate:
                print(f"⚠️  Found APPROXIMATE solution with {solution_path.getStateCount()} waypoints")
                print("   This means OMPL couldn't reach the exact goal due to constraints")
                
                # Check how close the approximate solution gets to the goal
                final_state = solution_path.getState(solution_path.getStateCount() - 1)
                final_config = self._ompl_state_to_numpy(final_state)
                goal_distance = np.linalg.norm(final_config - goal_config)
                
                print(f"   Distance to true goal: {goal_distance:.4f} radians")
                
                # Only accept approximate solutions if they're reasonably close
                if goal_distance > 0.3:  # 0.3 radians ≈ 17 degrees
                    print(f"   ❌ Approximate solution too far from goal, rejecting")
                    return None
                else:
                    print(f"   ✅ Approximate solution close enough, accepting")
            else:
                print(f"✅ Found EXACT solution with {solution_path.getStateCount()} waypoints")
            
            # Simplify path
            print("Simplifying path...")
            solution_path.interpolate()  # Add intermediate states
            
            # Convert solution to list of numpy arrays
            path = []
            for i in range(solution_path.getStateCount()):
                state = solution_path.getState(i)
                config = self._ompl_state_to_numpy(state)
                path.append(config)
            
            # Print path quality metrics
            total_cost = sum(self.drake_env.check_collision_cost(config) for config in path)
            max_cost = max(self.drake_env.check_collision_cost(config) for config in path)
            print(f"Path quality - Total collision cost: {total_cost:.4f}, Max: {max_cost:.4f}")
            
            return path
        else:
            print(f"❌ No solution found within {self.planning_time} seconds")
            return None
    
    def _validate_pose_achievement(self, joint_config: np.ndarray, target_pose: RigidTransform) -> tuple[bool, float, float]:
        """
        Validate that a joint configuration actually achieves the target pose
        
        Returns:
            (success, position_error, orientation_error_deg)
        """
        try:
            # Set the drake environment to this configuration
            self.drake_env.set_joint_config(joint_config)
            
            # Get actual end effector pose
            actual_pose = self.drake_env.plant.CalcRelativeTransform(
                self.drake_env.plant_context,
                self.drake_env.plant.world_frame(),
                self.drake_env.end_effector_frame
            )
            
            # Calculate position error
            pos_error = np.linalg.norm(target_pose.translation() - actual_pose.translation())
            
            # Calculate orientation error  
            R_target = target_pose.rotation().matrix()
            R_actual = actual_pose.rotation().matrix()
            R_error = R_target.T @ R_actual
            
            from scipy.spatial.transform import Rotation
            r_error = Rotation.from_matrix(R_error)
            axis_angle_error = r_error.as_rotvec()
            orient_error_rad = np.linalg.norm(axis_angle_error)
            orient_error_deg = np.degrees(orient_error_rad)
            
            print(f"  Pose validation:")
            print(f"    Target pos: {target_pose.translation()}")
            print(f"    Actual pos: {actual_pose.translation()}")
            print(f"    Position error: {pos_error:.4f} m")
            print(f"    Orientation error: {orient_error_deg:.2f}°")
            
            # Success criteria: < 2cm position error and < 10° orientation error
            success = pos_error < 0.02 and orient_error_deg < 10.0
            
            return success, pos_error, orient_error_deg
            
        except Exception as e:
            print(f"Error validating pose: {e}")
            return False, float('inf'), float('inf')

    def _plan_with_multiple_strategies(self, start_config: np.ndarray, goal_config: np.ndarray, target_pose: RigidTransform) -> Optional[List[np.ndarray]]:
        """Try multiple planning strategies in order of preference"""
        
        strategies = [
            # Strategy 1: Standard RRT* with normal settings
            {
                'name': 'RRT* (standard)',
                'planner': 'RRTstar', 
                'collision_threshold': self.collision_threshold,
                'step_size': 0.3,
                'resolution': 0.01
            },
            # Strategy 2: RRTConnect (better for tight spaces)
            {
                'name': 'RRT-Connect (tight spaces)',
                'planner': 'RRTConnect',
                'collision_threshold': self.collision_threshold,
                'step_size': 0.2,  # Smaller steps
                'resolution': 0.005  # Higher resolution
            },
            # Strategy 3: RRT* with relaxed collision and smaller steps
            {
                'name': 'RRT* (relaxed collisions)',
                'planner': 'RRTstar',
                'collision_threshold': self.collision_threshold * 2,  # 4cm
                'step_size': 0.15,  # Even smaller steps  
                'resolution': 0.005
            },
            # Strategy 4: EST (Expansive Space Trees - good for narrow passages)
            {
                'name': 'EST (narrow passages)',
                'planner': 'EST',
                'collision_threshold': self.collision_threshold * 2,
                'step_size': 0.1,
                'resolution': 0.003
            },
            # Strategy 5: Last resort - very relaxed
            {
                'name': 'RRT* (very relaxed)',
                'planner': 'RRTstar', 
                'collision_threshold': self.collision_threshold * 3,  # 6cm
                'step_size': 0.1,
                'resolution': 0.003
            }
        ]
        
        original_threshold = self.collision_threshold
        
        for i, strategy in enumerate(strategies):
            print(f"\n🎯 Strategy {i+1}/5: {strategy['name']}")
            
            # Apply strategy settings
            self.collision_threshold = strategy['collision_threshold']
            
            # Recreate OMPL setup with new settings
            self._setup_ompl_with_params(strategy['resolution'])
            
            # Try planning with this strategy
            try:
                # Store step size for this strategy
                self.current_step_size = strategy['step_size']
                path = self.plan(start_config, goal_config, strategy['planner'])
                
                if path is not None:
                    # Validate the path achieves the target pose
                    final_success, final_pos_err, final_orient_err = self._validate_pose_achievement(path[-1], target_pose)
                    
                    if final_success:
                        print(f"✅ Strategy {i+1} succeeded with good pose accuracy!")
                        self.collision_threshold = original_threshold
                        return path
                    elif final_pos_err < 0.1:  # Acceptable error < 10cm
                        print(f"✅ Strategy {i+1} succeeded with acceptable pose accuracy!")
                        print(f"   Position error: {final_pos_err:.4f}m, Orientation error: {final_orient_err:.2f}°")
                        self.collision_threshold = original_threshold
                        return path
                    else:
                        print(f"⚠️  Strategy {i+1} found path but poor pose accuracy ({final_pos_err:.3f}m error)")
                        if i == len(strategies) - 1:  # Last strategy
                            print("   Accepting as last resort")
                            self.collision_threshold = original_threshold
                            return path
                        else:
                            print("   Trying next strategy...")
                else:
                    print(f"❌ Strategy {i+1} failed to find path")
                    
            except Exception as e:
                print(f"❌ Strategy {i+1} encountered error: {e}")
        
        # If all strategies failed, try waypoint-based planning
        print(f"\n🔄 All direct strategies failed. Trying waypoint-based planning...")
        self.collision_threshold = original_threshold * 2  # Relaxed for waypoint planning
        
        waypoint_path = self._plan_through_waypoints(start_config, goal_config, target_pose)
        
        # Restore original threshold
        self.collision_threshold = original_threshold
        
        return waypoint_path
    
    def _setup_ompl_with_params(self, resolution: float):
        """Setup OMPL with specific parameters"""
        # Create configuration space
        self.space = ob.RealVectorStateSpace(self.n_joints)
        
        # Set joint bounds
        bounds = ob.RealVectorBounds(self.n_joints)
        for i, (lower, upper) in enumerate(self.joint_limits):
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)
        self.space.setBounds(bounds)
        
        # Create space information with validity checker
        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        self.si.setStateValidityCheckingResolution(resolution)
        self.si.setup()
        
        # Create problem definition
        self.pdef = ob.ProblemDefinition(self.si)
    
    def _plan_through_waypoints(self, start_config: np.ndarray, goal_config: np.ndarray, target_pose: RigidTransform) -> Optional[List[np.ndarray]]:
        """Plan through intermediate waypoints if direct planning fails"""
        try:
            print("   Generating intermediate waypoint...")
            
            # Create intermediate waypoint by interpolating in joint space
            # But validate it achieves a reasonable intermediate pose
            intermediate_config = 0.5 * (start_config + goal_config)
            
            # Check if intermediate waypoint is valid
            intermediate_cost = self.drake_env.check_collision_cost(intermediate_config)
            
            if intermediate_cost > self.collision_threshold * 2:
                print(f"   Intermediate waypoint has high collision cost ({intermediate_cost:.4f})")
                
                # Try different interpolation points
                for alpha in [0.3, 0.7, 0.2, 0.8]:
                    test_config = alpha * goal_config + (1 - alpha) * start_config
                    test_cost = self.drake_env.check_collision_cost(test_config)
                    
                    if test_cost <= self.collision_threshold * 2:
                        intermediate_config = test_config
                        print(f"   Found better intermediate waypoint (alpha={alpha}, cost={test_cost:.4f})")
                        break
                else:
                    print("   No good intermediate waypoint found")
                    return None
            
            # Plan: start -> intermediate -> goal
            print("   Planning: start -> intermediate")
            self._setup_ompl_with_params(0.005)  # High resolution
            path1 = self.plan(start_config, intermediate_config, "RRTConnect")
            
            if path1 is None:
                print("   Failed to reach intermediate waypoint")
                return None
                
            print("   Planning: intermediate -> goal")
            path2 = self.plan(intermediate_config, goal_config, "RRTConnect") 
            
            if path2 is None:
                print("   Failed to reach goal from intermediate waypoint")
                return None
            
            # Combine paths (remove duplicate intermediate point)
            combined_path = path1 + path2[1:]
            print(f"   ✅ Waypoint planning succeeded! Combined path: {len(combined_path)} waypoints")
            
            return combined_path
            
        except Exception as e:
            print(f"   Error in waypoint planning: {e}")
            return None

    def plan_to_pose(self, target_pose: RigidTransform, 
                     planner_type: str = "RRTstar") -> Optional[List[np.ndarray]]:
        """
        Plan to a target end-effector pose
        
        Args:
            target_pose: Target pose for end effector
            planner_type: OMPL planner type
            
        Returns:
            List of waypoint configurations if successful, None otherwise
        """
        # Get current configuration as start
        start_config = self.drake_env.get_current_joint_config()
        
        # Solve IK for target pose
        goal_config = self.drake_env.solve_ik_for_pose(target_pose)
        if goal_config is None:
            print("Could not solve IK for target pose")
            return None
        
        # Validate that the IK solution actually achieves the target pose
        print("Validating IK solution...")
        ik_success, pos_err, orient_err = self._validate_pose_achievement(goal_config, target_pose)
        
        if not ik_success:
            print(f"❌ IK solution doesn't achieve target pose!")
            print(f"   Position error: {pos_err:.4f}m (max: 0.02m)")
            print(f"   Orientation error: {orient_err:.2f}° (max: 10°)")
            print("   Try adjusting the target pose or IK constraints")
            return None
        else:
            print(f"✅ IK solution validated - errors within tolerance")
        
        # Try multiple planning strategies
        path = self._plan_with_multiple_strategies(start_config, goal_config, target_pose)
        
        return path
    
    def visualize_path(self, path: List[np.ndarray], delay: float = 1.0, target_pose: RigidTransform = None):
        """Visualize planned path by stepping through waypoints"""
        if not path:
            print("No path to visualize")
            return
        
        print(f"Visualizing path with {len(path)} waypoints...")
        
        for i, waypoint in enumerate(path):
            print(f"Waypoint {i+1}/{len(path)}")
            
            # Set configuration and update visualization
            self.drake_env.set_joint_config(waypoint)
            
            # Check and report collision cost
            collision_cost = self.drake_env.check_collision_cost(waypoint)
            if collision_cost > self.collision_threshold:
                print(f"  ⚠️  Collision cost: {collision_cost:.4f} (above threshold)")
            else:
                print(f"  ✅  Collision cost: {collision_cost:.4f}")
            
            # For the final waypoint, validate pose achievement if target provided
            if target_pose is not None and i == len(path) - 1:
                print("  🎯 Final waypoint pose validation:")
                success, pos_err, orient_err = self._validate_pose_achievement(waypoint, target_pose)
                if not success:
                    print(f"     ❌ Target not achieved! Pos: {pos_err:.4f}m, Orient: {orient_err:.2f}°")
                else:
                    print(f"     ✅ Target achieved! Pos: {pos_err:.4f}m, Orient: {orient_err:.2f}°")
            
            time.sleep(delay)
        
        print("Path visualization complete!")

def main():
    """Main function demonstrating OMPL motion planning"""
    try:
        # Initialize Drake environment exactly like visualization script
        kinematic_chain_joints = [
            "pillar_platform_joint",
            "pan_tilt_pan_joint",
            "pan_tilt_head_joint", 
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5", 
            "joint6",
            "joint7",
        ]
        
        links_to_ignore = [
            "devkit_base_link",
            "pillar_platform",
            "piper_angled_mount",
            "pan_tilt_base", 
            "pan_tilt_head",
            "pan_tilt_pan",
            "link_base",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5", 
            "link6",
            "link7"
        ]
        
        urdf_path = "./assets/xarm_devkit_base_descr.urdf"
        urdf_path = os.path.abspath(urdf_path)
        
        print(f"Loading URDF from: {urdf_path}")
        
        # Create Drake environment
        drake_env = DrakeKinematicsEnv(
            urdf_path=urdf_path,
            kinematic_chain_joints=kinematic_chain_joints,
            end_effector_link_name="link7",
            links_to_ignore=links_to_ignore,
        )
        
        # Wait for transforms
        print("Waiting for transforms...")
        time.sleep(2)
        
        # Create OMPL motion planner
        motion_planner = OMPLMotionPlanner(
            drake_env=drake_env,
            collision_threshold=0.02,  # 2cm max collision depth
            planning_time=30  # 30 seconds planning time
        )
        
        # Define goal pose
        goal_translation = np.array([-0.88, 0.21, .95])
        goal_rpy = RollPitchYaw(3.14, -0.2, 3.14)
        goal_pose = RigidTransform(RotationMatrix(goal_rpy), goal_translation)
        
        print(f"\n🎯 Planning to goal pose:")
        print(f"  Position: {goal_translation}")
        print(f"  Orientation (RPY): {goal_rpy.vector()}")
        
        # Plan path to goal pose
        path = motion_planner.plan_to_pose(goal_pose, planner_type="RRTstar")
        
        if path:
            print(f"\n🎉 Motion planning successful!")
            print(f"Found path with {len(path)} waypoints")
            
            # Visualize the planned path
            print(f"\n📺 Starting path visualization...")
            print(f"Visit {drake_env.meshcat.web_url()} to view the robot")
            motion_planner.visualize_path(path, delay=.1, target_pose=goal_pose)
            
            # Keep visualization open
            print(f"\n✨ Path execution complete!")
            print("Press Ctrl+C to exit.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\nExiting...")
        else:
            print("\n❌ Motion planning failed!")
            print("Try adjusting the goal pose, collision threshold, or planning time")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if not OMPL_AVAILABLE:
        print("Please install OMPL: pip install ompl-python")
        sys.exit(1)
    main()