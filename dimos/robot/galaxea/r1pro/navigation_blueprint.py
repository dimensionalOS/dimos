# Copyright 2026 Dimensional Inc.
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

"""ACT packing, native KronkNav, and holonomic coordinator base execution."""

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.policy.lerobot.module import R1ProPackingPolicy
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.galaxea.r1pro.grasping_blueprint import build_r1pro_manipulation
from dimos.robot.galaxea.r1pro.learning import R1PRO_PACKING_TASK
from dimos.robot.galaxea.r1pro.navigation_sim import (
    NAV_BASE_ID,
    NAV_FRAME,
    NAV_TASK,
    R1ProNavigationSim,
)

CONTROLLER_ARTIFACT = Path(__file__).with_name("navigation_controller.json")


def build_r1pro_packing_navigation(
    *,
    scene_path: Path,
    artifact: str,
    headless: bool = False,
    simulator: type[R1ProNavigationSim] = R1ProNavigationSim,
    coordinator_type: type[PathFollowingCoordinator] = PathFollowingCoordinator,
    prepare_scene_on_build: bool = False,
) -> Blueprint:
    """Give the learned arms and velocity-driven base disjoint coordinator resources."""
    joints = make_twist_base_joints(NAV_BASE_ID)
    manipulation = build_r1pro_manipulation(
        scene_path=scene_path,
        artifact=artifact,
        device="cuda",
        headless=headless,
        simulator=simulator,
        policy_module=R1ProPackingPolicy,
        task_description=R1PRO_PACKING_TASK,
        background_camera_rendering=True,
        viewer_lookat=(0.2, -0.25, 1.0),
        viewer_distance=1.8,
        viewer_azimuth=225.0,
        viewer_elevation=-30.0,
        coordinator_type=coordinator_type,
        prepare_scene_on_build=prepare_scene_on_build,
        velocity_base=HardwareComponent(
            hardware_id=NAV_BASE_ID,
            hardware_type=HardwareType.BASE,
            joints=joints,
            adapter_type="transport_lcm",
            auto_enable=True,
        ),
        navigation_task=TaskConfig(
            name=NAV_TASK,
            type="holonomic_pose_follower",
            joint_names=joints,
            priority=30,
            params={
                "artifact_path": str(CONTROLLER_ARTIFACT),
                "speed": 0.6,
                "lookahead": 0.025,
                "regulate_horizon": 0.2,
                "goal_tolerance": 0.006,
                "orientation_tolerance": 0.006,
                "approach_decel": 0.4,
                "stop_hold_s": 1.0,
            },
        ),
    )
    return autoconnect(
        manipulation,
        MLSPlannerNative.blueprint(
            world_frame="world",
            base_frame=NAV_FRAME,
            voxel_size=0.06,
            robot_height=1.70,
            surface_closing_radius=0.12,
            node_spacing_m=0.3,
            wall_clearance_m=0.35,
            wall_buffer_m=0.65,
            wall_buffer_weight=20.0,
            step_threshold_m=0.07,
            goal_tolerance=0.01,
            viz_publish_hz=1.0,
            worker_threads=2,
        ),
    ).remappings(
        [
            (MLSPlannerNative, "path", "planned_path"),
            (MLSPlannerNative, "tf", "navigation_tf"),
            # Only collision-validated, frame-converted paths reach the task via RPC.
            (coordinator_type, "path", "execution_path"),
            (simulator, "base_cmd_vel", f"/{NAV_BASE_ID}/cmd_vel"),
            (simulator, "base_odom", f"/{NAV_BASE_ID}/odom"),
        ]
    )
