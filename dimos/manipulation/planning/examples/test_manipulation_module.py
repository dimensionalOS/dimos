#!/usr/bin/env python3
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
ManipulationModule Test Script

Tests the ManipulationModule by simulating joint state and calling planning methods.

Usage:
    python test_manipulation_module.py [--robot ROBOT] [--viz]

    Supported robots:
        xarm6   - UFactory xArm 6-DOF arm (default)
        xarm7   - UFactory xArm 7-DOF arm

Examples:
    python test_manipulation_module.py --robot xarm6 --viz
    python test_manipulation_module.py --robot xarm7
"""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from dimos.manipulation import ManipulationModule, ManipulationState
from dimos.msgs.sensor_msgs import JointState

# Robot configurations
ROBOT_CONFIGS = {
    "xarm6": {
        "name": "xarm6",
        "urdf_subpath": "xarm/xarm_description/urdf/xarm_device.urdf.xacro",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "end_effector_link": "link6",
        "base_link": "link_base",
        "package_name": "xarm_description",
        "package_subpath": "xarm/xarm_description",
        "xacro_args": {"dof": "6", "limited": "true"},
    },
    "xarm7": {
        "name": "xarm7",
        "urdf_subpath": "xarm/xarm_description/urdf/xarm_device.urdf.xacro",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        "end_effector_link": "link7",
        "base_link": "link_base",
        "package_name": "xarm_description",
        "package_subpath": "xarm/xarm_description",
        "xacro_args": {"dof": "7", "limited": "true"},
    },
}


def get_base_path() -> Path:
    """Get base path for hardware/manipulators."""
    # examples -> planning -> manipulation -> dimos -> hardware
    return Path(__file__).parent.parent.parent.parent / "hardware" / "manipulators"


def get_urdf_path(robot_type: str) -> str:
    """Get URDF path for robot type."""
    base_path = get_base_path()
    urdf_path = base_path / ROBOT_CONFIGS[robot_type]["urdf_subpath"]
    return str(urdf_path)


def get_package_paths(robot_type: str) -> dict[str, str]:
    """Get package_paths for xacro resolution."""
    robot_cfg = ROBOT_CONFIGS[robot_type]
    base_path = get_base_path()
    return {
        robot_cfg["package_name"]: str(base_path / robot_cfg["package_subpath"]),
    }


def wait_for_user(msg: str = "Press Enter to continue...") -> None:
    """Wait for user input."""
    input(f"\n>>> {msg}")


def test_manipulation_module(
    robot_type: str = "xarm6", enable_viz: bool = False, interactive: bool = False
) -> None:
    """Test the ManipulationModule."""
    print("=" * 60)
    print(f"ManipulationModule Test - {robot_type.upper()}")
    if interactive:
        print("(Interactive mode - press Enter to advance)")
    print("=" * 60)

    robot_cfg = ROBOT_CONFIGS[robot_type]
    urdf_path = get_urdf_path(robot_type)
    package_paths = get_package_paths(robot_type)

    print("\n1. Creating ManipulationModule...")
    print(f"   URDF: {urdf_path}")
    print(f"   Joints: {robot_cfg['joint_names']}")

    # Create module (not using cluster - direct instantiation for testing)
    # Config fields are passed as kwargs
    module = ManipulationModule(
        robot_urdf_path=urdf_path,
        robot_name=robot_cfg["name"],
        joint_names=robot_cfg["joint_names"],
        end_effector_link=robot_cfg["end_effector_link"],
        base_link=robot_cfg["base_link"],
        max_velocity=1.0,
        max_acceleration=2.0,
        planning_timeout=10.0,
        enable_viz=enable_viz,
        package_paths=package_paths,
        xacro_args=robot_cfg.get("xacro_args", {}),
    )

    print("\n2. Initializing planning stack...")
    module._initialize_planning()

    if enable_viz and module._world_monitor:
        url = module._world_monitor.get_meshcat_url()
        if url:
            print(f"   Meshcat URL: {url}")
            print("   Open this URL in your browser to see the visualization.")
        if interactive:
            wait_for_user("Open Meshcat in browser, then press Enter...")

    # Simulate joint state
    print("\n3. Simulating joint state (home position)...")
    num_joints = len(robot_cfg["joint_names"])
    home_positions = np.zeros(num_joints)
    joint_state = JointState(
        ts=time.time(),
        name=robot_cfg["joint_names"],
        position=list(home_positions),
        velocity=[0.0] * num_joints,
        effort=[0.0] * num_joints,
    )
    module._on_joint_state(joint_state)
    # Update visualization
    if module._world_monitor:
        module._world_monitor.world.publish_to_meshcat()
    print(f"   Positions: {list(home_positions)}")

    if interactive:
        wait_for_user("Robot at home position. Press Enter to get EE pose...")

    # Get current EE pose
    print("\n4. Getting current end-effector pose...")
    ee_pose = module.get_ee_pose()
    if ee_pose:
        x, y, z, roll, pitch, yaw = ee_pose
        print(f"   Position: ({x:.4f}, {y:.4f}, {z:.4f}) m")
        print(f"   Orientation: ({roll:.4f}, {pitch:.4f}, {yaw:.4f}) rad")

    # Check state
    print(f"\n5. Module state: {module.get_state_name()}")
    assert module.get_state() == ManipulationState.IDLE.value, "Expected IDLE state"

    if interactive:
        wait_for_user("Press Enter to test move_to_joints...")

    # Test move_to_joints
    print("\n6. Testing move_to_joints...")
    # Small movements from home position (collision-free)
    goal_joints = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    if num_joints == 7:
        goal_joints.append(0.0)  # Add 7th joint for xarm7

    print(f"   Planning path to: {goal_joints}")
    # Note: Without trajectory output connected, this tests planning only
    # The trajectory won't actually be published anywhere
    success = module.move_to_joints(goal_joints)
    print(f"   Success: {success}")
    print(f"   State: {module.get_state_name()}")

    if not success:
        print(f"   Error: {module.get_error()}")
    else:
        # Update viz to show goal position
        if module._world_monitor:
            module._world_monitor.world.sync_from_joint_state(
                module._robot_id, np.array(goal_joints)
            )
            module._world_monitor.world.publish_to_meshcat()
            print("   (Visualization updated to show goal position)")

    if interactive:
        wait_for_user("Path planned! Check Meshcat. Press Enter to continue...")

    # Reset for next test
    module.reset()
    print(f"   Reset state: {module.get_state_name()}")

    # Test move_to_pose
    print("\n7. Testing move_to_pose...")
    # Simulate being at home first
    module._on_joint_state(joint_state)
    if module._world_monitor:
        module._world_monitor.world.sync_from_joint_state(module._robot_id, home_positions)
        module._world_monitor.world.publish_to_meshcat()

    if interactive:
        wait_for_user("Robot reset to home. Press Enter to test move_to_pose...")

    # Target pose (reachable position for xarm6 - front of robot)
    target_x, target_y, target_z = 0.4, 0.0, 0.3
    target_roll, target_pitch, target_yaw = np.pi, 0.0, 0.0  # pointing down (EE Z-axis down)

    print(f"   Target position: ({target_x}, {target_y}, {target_z}) m")
    print(f"   Target orientation: ({target_roll:.2f}, {target_pitch:.2f}, {target_yaw:.2f}) rad")
    print("   Solving IK and planning path...")

    success = module.move_to_pose(
        target_x, target_y, target_z, target_roll, target_pitch, target_yaw
    )
    print(f"   Success: {success}")
    print(f"   State: {module.get_state_name()}")

    if not success:
        print(f"   Error: {module.get_error()}")
    else:
        # Update viz to show IK solution position
        if module._world_monitor:
            current = module.get_current_joints()
            if current:
                module._world_monitor.world.sync_from_joint_state(
                    module._robot_id, np.array(current)
                )
                module._world_monitor.world.publish_to_meshcat()
                print("   (Visualization updated to show IK solution)")

    if interactive:
        wait_for_user("IK solved and path planned! Check Meshcat. Press Enter to finish...")

    # Test collision checking
    print("\n8. Testing collision checking...")
    is_free = module.is_collision_free(list(home_positions))
    print(f"   Home position collision-free: {is_free}")

    # Test obstacle management (now with visualization)
    print("\n9. Testing obstacle management...")
    # Reset to home first
    module._on_joint_state(joint_state)
    if module._world_monitor:
        module._world_monitor.world.sync_from_joint_state(module._robot_id, home_positions)

    # Add a box obstacle
    box_id = module.add_box_obstacle(
        name="test_box",
        x=0.3,
        y=0.0,
        z=0.3,
        width=0.1,
        height=0.1,
        depth=0.1,
    )
    print(f"   Added box obstacle: {box_id}")

    # Add a sphere obstacle
    sphere_id = module.add_sphere_obstacle(
        name="test_sphere",
        x=0.0,
        y=0.3,
        z=0.3,
        radius=0.05,
    )
    print(f"   Added sphere obstacle: {sphere_id}")
    print("   (Obstacles should now be visible in Meshcat)")

    if interactive:
        wait_for_user(
            "Obstacles added! Check Meshcat. Press Enter to test preview/execute workflow..."
        )

    # Test preview/execute workflow (plan-only, then preview, then execute)
    print("\n10. Testing preview/execute workflow...")
    module.reset()

    # Plan to joints (without executing)
    test_goal = [0.3, 0.3, 0.0, 0.0, 0.0, 0.0]
    if num_joints == 7:
        test_goal.append(0.0)

    print(f"   Planning to joints: {test_goal}")
    plan_success = module.plan_to_joints(test_goal)
    print(f"   Plan success: {plan_success}")
    print(f"   Has planned path: {module.has_planned_path()}")

    if plan_success and enable_viz:
        if interactive:
            wait_for_user("Path planned! Press Enter to preview in Drake...")

        print("   Previewing path in Drake (5 second animation)...")
        preview_success = module.preview_path_in_drake(duration=5.0)
        print(f"   Preview success: {preview_success}")

        if interactive:
            wait_for_user("Preview complete! Press Enter to execute...")

        # Note: execute_planned would publish to trajectory output, but it's not connected
        # In a real setup, this would send to the trajectory controller
        print("   Calling execute_planned (trajectory published to output)...")
        execute_success = module.execute_planned()
        print(f"   Execute success: {execute_success}")

    # Test plan_to_pose workflow
    print("\n11. Testing plan_to_pose workflow...")
    module.reset()
    module._on_joint_state(joint_state)  # Reset to home

    pose_plan_success = module.plan_to_pose(0.35, 0.1, 0.25, np.pi, 0.0, 0.0)
    print(f"   Plan to pose success: {pose_plan_success}")

    if pose_plan_success and enable_viz:
        if interactive:
            wait_for_user("Pose plan ready! Press Enter to preview...")
        print("   Previewing pose path in Drake (5 second animation)...")
        module.preview_path_in_drake(duration=5.0)
        print("   Preview complete!")

    # Clear obstacles
    print("\n12. Clearing obstacles...")
    module.clear_obstacles()
    print("   Obstacles cleared")

    # Test cancel
    print("\n13. Testing cancel...")
    cancelled = module.cancel()
    print(f"   Cancelled: {cancelled}")

    # Test get_visualization_url
    print("\n14. Getting visualization URL...")
    viz_url = module.get_visualization_url()
    print(f"   URL: {viz_url if viz_url else 'Not available (viz disabled)'}")

    # Final reset
    module.reset()

    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)

    # Keep visualization open if enabled (unless interactive mode already handled it)
    if enable_viz and not interactive:
        print("\nVisualization is active. Press Ctrl+C to exit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting...")
    elif enable_viz:
        wait_for_user("Press Enter to exit...")


def main():
    """Run the test."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Test ManipulationModule",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="xarm7",
        choices=list(ROBOT_CONFIGS.keys()),
        help="Robot type to use (default: xarm7). Note: MuJoCo only has xarm7 model.",
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Enable Meshcat visualization",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="Interactive mode - pause between steps",
    )

    args = parser.parse_args()
    test_manipulation_module(
        robot_type=args.robot, enable_viz=args.viz, interactive=args.interactive
    )


if __name__ == "__main__":
    main()
