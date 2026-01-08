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
Example: WorldSpec-Based Motion Planning Integration

This demonstrates the new Protocol-based manipulation stack:
1. Factory functions create all components (no concrete type imports)
2. WorldSpec provides context management for thread safety
3. KinematicsSpec solves IK using WorldSpec
4. PlannerSpec finds collision-free paths using WorldSpec

## Key Patterns Demonstrated

### Factory Pattern
All components are created via factory functions that return Protocol types:
```python
world = create_world(backend="drake", enable_viz=True)  # Returns WorldSpec
kinematics = create_kinematics(backend="drake")          # Returns KinematicsSpec
planner = create_planner(name="rrt_connect")             # Returns PlannerSpec
```

### Context Management
The WorldSpec maintains a live context (synced with real robot) and provides
scratch contexts for planning operations:
```python
# Live context mirrors real robot state
world.sync_from_joint_state(robot_id, positions)

# Scratch context for planning (thread-safe clone)
with world.scratch_context() as ctx:
    world.set_positions(ctx, robot_id, q_test)
    if world.is_collision_free(ctx, robot_id):
        # Valid configuration
        pass
```

### Stateless Solvers
Kinematics and planner are stateless - they use WorldSpec for all operations:
```python
ik_result = kinematics.solve(world, robot_id, target_pose, seed=q_current)
plan_result = planner.plan_joint_path(world, robot_id, q_start, q_goal)
```

Usage:
    python example_worldspec_integration.py --sim     # Simulation only
    python example_worldspec_integration.py           # With real hardware
"""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING

import numpy as np

# Factory imports (Protocol types)
from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_world,
)
from dimos.manipulation.planning.spec import (
    Obstacle,
    ObstacleType,
    RobotModelConfig,
)
from dimos.manipulation.planning.utils.path_utils import interpolate_path

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec import PlannerSpec, WorldSpec


def get_piper_config() -> RobotModelConfig:
    """Get Piper robot configuration."""
    base_path = Path(__file__).parent.parent.parent.parent / "hardware" / "manipulators"

    return RobotModelConfig(
        name="piper",
        urdf_path=str(
            base_path / "piper" / "piper_description" / "urdf" / "piper_description.urdf"
        ),
        base_pose=np.eye(4),
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        end_effector_link="link6",
        base_link="base_link",
        package_paths={
            "piper_description": str(base_path / "piper" / "piper_description"),
        },
        auto_convert_meshes=True,
    )


def create_table_obstacle() -> Obstacle:
    """Create a table obstacle."""
    return Obstacle(
        name="table",
        obstacle_type=ObstacleType.BOX,
        pose=np.array(
            [
                [1, 0, 0, 0.3],
                [0, 1, 0, 0.0],
                [0, 0, 1, -0.05],
                [0, 0, 0, 1],
            ]
        ),
        dimensions=(0.6, 0.4, 0.02),
        color=(0.6, 0.4, 0.2, 0.8),
    )


# =============================================================================
# EXAMPLE 1: Basic Planning with WorldSpec
# =============================================================================


def example_basic_planning():
    """Basic example: create world, solve IK, plan path."""

    print("=" * 60)
    print("Example 1: Basic Planning with WorldSpec")
    print("=" * 60)

    # =========================================================================
    # Step 1: Create components via factory
    # =========================================================================
    print("\n1. Creating components via factory functions...")

    # These return Protocol types (WorldSpec, PlannerSpec)
    # No concrete type imports needed!
    world: WorldSpec = create_world(backend="drake", enable_viz=True)
    planner: PlannerSpec = create_planner(name="rrt_connect", backend="drake")
    # kinematics: KinematicsSpec = create_kinematics(backend="drake")  # Available for IK

    print("   Created: world (WorldSpec)")
    print("   Created: planner (PlannerSpec)")

    # =========================================================================
    # Step 2: Add robot and obstacles
    # =========================================================================
    print("\n2. Adding robot and obstacles...")

    robot_config = get_piper_config()
    robot_id = world.add_robot(robot_config)
    print(f"   Added robot: {robot_id}")

    table = create_table_obstacle()
    world.add_obstacle(table)
    print(f"   Added obstacle: {table.name}")

    # =========================================================================
    # Step 3: Finalize world
    # =========================================================================
    print("\n3. Finalizing world...")
    world.finalize()
    print("   World finalized for collision checking")

    # Get visualization URL
    if hasattr(world, "get_meshcat_url"):
        url = world.get_meshcat_url()
        print(f"   Meshcat URL: {url}")
        if "--sim" not in sys.argv:
            input("\n   >>> Open the Meshcat URL in your browser, then press Enter to continue...")

    # =========================================================================
    # Step 4: Set start configuration
    # =========================================================================
    print("\n4. Setting start configuration...")

    q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Sync to live context (simulates receiving joint state from driver)
    world.sync_from_joint_state(robot_id, q_start)

    # Verify collision-free using scratch context
    with world.scratch_context() as ctx:
        world.set_positions(ctx, robot_id, q_start)
        is_valid = world.is_collision_free(ctx, robot_id)
        start_pose = world.get_ee_pose(ctx, robot_id)

    print(f"   Start joints: {q_start}")
    print(f"   Start pose (EE): {start_pose[:3, 3]}")
    print(f"   Collision-free: {is_valid}")

    # Publish to visualization
    if hasattr(world, "publish_to_meshcat"):
        world.publish_to_meshcat()

    # =========================================================================
    # Step 5: Define goal configuration
    # =========================================================================
    print("\n5. Setting goal configuration...")

    # Try several goal configurations until we find a collision-free one
    goal_candidates = [
        np.array([0.5, 0.3, -0.5, 0.0, 0.5, 0.0]),  # Arms up
        np.array([0.3, 0.2, -0.3, 0.0, 0.3, 0.0]),  # Slight move
        np.array([-0.5, 0.3, -0.5, 0.0, 0.5, 0.0]),  # Other side
        np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0]),  # Just joint2
    ]

    q_goal = None
    for candidate in goal_candidates:
        with world.scratch_context() as ctx:
            world.set_positions(ctx, robot_id, candidate)
            if world.is_collision_free(ctx, robot_id):
                q_goal = candidate
                goal_pose = world.get_ee_pose(ctx, robot_id)
                break

    if q_goal is None:
        print("   Could not find collision-free goal!")
        return

    print(f"   Goal joints: {q_goal}")
    print(f"   Goal EE position: {goal_pose[:3, 3]}")
    print("   Collision-free: True")

    # =========================================================================
    # Step 6: Plan path using PlannerSpec
    # =========================================================================
    print("\n6. Planning collision-free path...")

    plan_result = planner.plan_joint_path(
        world=world,
        robot_id=robot_id,
        q_start=q_start,
        q_goal=q_goal,
        timeout=10.0,
    )

    if not plan_result.is_success():
        print(f"   Planning failed: {plan_result.message}")
        return

    path = plan_result.path
    print(f"   Found path with {len(path)} waypoints")
    print(f"   Planning time: {plan_result.planning_time:.3f}s")
    print(f"   Path length: {plan_result.path_length:.3f}rad")

    # =========================================================================
    # Step 7: Visualize path
    # =========================================================================
    print("\n7. Visualizing path...")

    # Interpolate for smooth visualization
    interpolated = interpolate_path(path, resolution=0.05)

    for i, waypoint in enumerate(interpolated):
        world.sync_from_joint_state(robot_id, waypoint)
        if hasattr(world, "publish_to_meshcat"):
            world.publish_to_meshcat()

        if i % 20 == 0:
            print(f"   Progress: {i}/{len(interpolated)}")

        time.sleep(0.02)

    print("   Visualization complete!")

    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"   Start: {q_start}")
    print(f"   Goal:  {q_goal}")
    print(f"   Path waypoints: {len(path)}")
    print(f"   Interpolated points: {len(interpolated)}")

    if "--sim" not in sys.argv:
        input("\nPress Enter to exit...")


# =============================================================================
# EXAMPLE 2: Using WorldMonitor for State Syncing
# =============================================================================


def example_with_monitor():
    """Example using WorldMonitor for reactive state updates."""

    print("=" * 60)
    print("Example 2: WorldMonitor for State Syncing")
    print("=" * 60)

    # Import WorldMonitor (uses factory internally)
    from dimos.manipulation.planning.monitor import WorldMonitor

    # =========================================================================
    # Step 1: Create WorldMonitor
    # =========================================================================
    print("\n1. Creating WorldMonitor...")

    monitor = WorldMonitor(backend="drake", enable_viz=True)
    print("   Monitor created")

    # =========================================================================
    # Step 2: Add robot
    # =========================================================================
    print("\n2. Adding robot...")

    robot_config = get_piper_config()
    robot_id = monitor.add_robot(robot_config)
    print(f"   Added robot: {robot_id}")

    # Add obstacle
    monitor.add_box_obstacle(
        name="table",
        pose=np.array(
            [
                [1, 0, 0, 0.3],
                [0, 1, 0, 0.0],
                [0, 0, 1, -0.05],
                [0, 0, 0, 1],
            ]
        ),
        dimensions=(0.6, 0.4, 0.02),
    )
    print("   Added table obstacle")

    # =========================================================================
    # Step 3: Finalize and start monitoring
    # =========================================================================
    print("\n3. Finalizing and starting monitors...")

    monitor.finalize()

    # Start state monitor for this robot
    monitor.start_state_monitor(robot_id)
    monitor.start_obstacle_monitor()

    print("   Monitors started")

    if monitor.get_meshcat_url():
        print(f"   Meshcat URL: {monitor.get_meshcat_url()}")

    # =========================================================================
    # Step 4: Simulate joint state updates
    # =========================================================================
    print("\n4. Simulating joint state updates...")

    from dimos.msgs.sensor_msgs import JointState

    # Simulate receiving joint state messages
    for i in range(50):
        # Create mock joint state message
        positions = [0.1 * np.sin(i * 0.1 + j) for j in range(6)]
        msg = JointState(
            ts=time.time(),
            frame_id="robot",
            name=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            position=positions,
            velocity=[0.0] * 6,
            effort=[0.0] * 6,
        )

        # Feed to monitor (as if from subscriber)
        monitor.on_joint_state(msg, robot_id)
        monitor.publish_visualization()

        if i % 10 == 0:
            current = monitor.get_current_positions(robot_id)
            if current is not None:
                print(f"   Step {i}: positions = {current[:3]}...")

        time.sleep(0.05)

    print("   State update simulation complete!")

    # =========================================================================
    # Step 5: Use scratch context for planning
    # =========================================================================
    print("\n5. Using scratch context for collision checking...")

    test_configs = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.5, -0.3, 0.5, 0.0, 0.8, 0.0]),
        np.array([1.0, 0.0, -1.0, 0.0, 0.0, 0.0]),
    ]

    for i, q in enumerate(test_configs):
        is_valid = monitor.is_state_valid(robot_id, q)
        print(f"   Config {i + 1}: collision-free = {is_valid}")

    # =========================================================================
    # Step 6: Cleanup
    # =========================================================================
    print("\n6. Stopping monitors...")
    monitor.stop_all_monitors()
    print("   Done!")

    if "--sim" not in sys.argv:
        input("\nPress Enter to exit...")


# =============================================================================
# EXAMPLE 3: Full Integration with Kinematics and Planning
# =============================================================================


def example_full_integration():
    """Full integration: monitor + kinematics + planner."""

    print("=" * 60)
    print("Example 3: Full Integration")
    print("=" * 60)

    from dimos.manipulation.planning.monitor import WorldMonitor

    # =========================================================================
    # Step 1: Create all components
    # =========================================================================
    print("\n1. Creating components...")

    monitor = WorldMonitor(backend="drake", enable_viz=True)
    kinematics = create_kinematics(backend="drake")
    planner = create_planner(name="rrt_connect", backend="drake")

    # =========================================================================
    # Step 2: Setup world
    # =========================================================================
    print("\n2. Setting up world...")

    robot_id = monitor.add_robot(get_piper_config())

    # Add table obstacle
    monitor.add_box_obstacle(
        name="table",
        pose=np.array([[1, 0, 0, 0.3], [0, 1, 0, 0.0], [0, 0, 1, -0.05], [0, 0, 0, 1]]),
        dimensions=(0.6, 0.4, 0.02),
    )

    monitor.finalize()
    monitor.start_state_monitor(robot_id)

    if monitor.get_meshcat_url():
        print(f"   Meshcat URL: {monitor.get_meshcat_url()}")

    # =========================================================================
    # Step 3: Set initial state
    # =========================================================================
    print("\n3. Setting initial state...")

    from dimos.msgs.sensor_msgs import JointState

    q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    msg = JointState(
        ts=time.time(),
        frame_id="robot",
        name=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        position=q_start.tolist(),
        velocity=[0.0] * 6,
        effort=[0.0] * 6,
    )
    monitor.on_joint_state(msg, robot_id)
    monitor.publish_visualization()

    print(f"   Start: {q_start}")

    # =========================================================================
    # Step 4: Solve IK using WorldMonitor's world
    # =========================================================================
    print("\n4. Solving IK...")

    target_pose = np.array([[1, 0, 0, 0.25], [0, -1, 0, 0.1], [0, 0, -1, 0.2], [0, 0, 0, 1]])

    # Use monitor's world for IK
    ik_result = kinematics.solve(
        world=monitor.world,
        robot_id=robot_id,
        target_pose=target_pose,
        seed=q_start,
    )

    if not ik_result.is_success():
        print(f"   IK failed: {ik_result.message}")
        return

    q_goal = ik_result.joint_positions
    print(f"   Goal: {q_goal}")

    # =========================================================================
    # Step 5: Plan path
    # =========================================================================
    print("\n5. Planning path...")

    plan_result = planner.plan_joint_path(
        world=monitor.world,
        robot_id=robot_id,
        q_start=q_start,
        q_goal=q_goal,
    )

    if not plan_result.is_success():
        print(f"   Planning failed: {plan_result.message}")
        return

    print(f"   Found path: {len(plan_result.path)} waypoints")

    # =========================================================================
    # Step 6: Execute path (simulated)
    # =========================================================================
    print("\n6. Executing path (simulated)...")

    interpolated = interpolate_path(plan_result.path, resolution=0.03)

    for i, waypoint in enumerate(interpolated):
        # Simulate receiving joint state from robot
        msg = JointState(
            ts=time.time(),
            frame_id="robot",
            name=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            position=waypoint.tolist(),
            velocity=[0.0] * 6,
            effort=[0.0] * 6,
        )
        monitor.on_joint_state(msg, robot_id)
        monitor.publish_visualization()

        if i % 20 == 0:
            print(f"   Progress: {i}/{len(interpolated)}")

        time.sleep(0.02)

    print("   Execution complete!")

    # =========================================================================
    # Cleanup
    # =========================================================================
    monitor.stop_all_monitors()

    if "--sim" not in sys.argv:
        input("\nPress Enter to exit...")


# =============================================================================
# Main
# =============================================================================


def main():
    """Run examples."""
    print("\nWorldSpec Integration Examples")
    print("=" * 60)
    print("1. Basic Planning (factory + world + kinematics + planner)")
    print("2. WorldMonitor (reactive state updates)")
    print("3. Full Integration (monitor + kinematics + planner)")
    print()

    if "--sim" in sys.argv:
        # Quick demo
        choice = "1"
    else:
        choice = input("Select example (1/2/3): ").strip()

    if choice == "1":
        example_basic_planning()
    elif choice == "2":
        example_with_monitor()
    elif choice == "3":
        example_full_integration()
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
