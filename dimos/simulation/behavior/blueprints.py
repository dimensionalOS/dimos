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

"""R1 Pro teleop, navigation, manipulation, and task execution."""

import rerun.blueprint as rrb

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.r1pro_bridge import BehaviorCoordinator, BehaviorR1ProBridge
from dimos.simulation.behavior.r1pro_model import simulation_model, simulation_model_config
from dimos.simulation.behavior.skills import SYSTEM_PROMPT, BehaviorSkills
from dimos.simulation.behavior.types import TaskSelection
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module


def _view() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image"),
                rrb.Spatial2DView(origin="world/left_wrist_image"),
                rrb.Spatial2DView(origin="world/right_wrist_image"),
            ),
        )
    )


behavior_teleop = autoconnect(
    BehaviorConnection.blueprint(),
    vis_module(global_config.viewer, rerun_config={"blueprint": _view}).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")]
    ),
).global_config(transport="zenoh")

# KronkNav's current mapping, global/local planning, and holonomic tracking chain.
_behavior_navigation = autoconnect(
    BehaviorConnection.blueprint(publish_scan=True, allow_task_changes=False).remappings(
        [
            (BehaviorConnection, "registered_scan", "raw_scan"),
            (BehaviorConnection, "left_wrist_scan", "raw_scan"),
            (BehaviorConnection, "right_wrist_scan", "raw_scan"),
        ]
    ),
    PointCloudSelfFilter.blueprint(
        model=simulation_model(),
        voxel_size=0.05,
        tf_tolerance_s=0.1,
    ).remappings(
        [
            (PointCloudSelfFilter, "pointcloud", "raw_scan"),
            (PointCloudSelfFilter, "filtered_pointcloud", "registered_scan"),
        ]
    ),
    RayTracingVoxelMap.blueprint(voxel_size=0.05, max_range=5.0, world_frame="world").remappings(
        [(RayTracingVoxelMap, "lidar", "registered_scan")]
    ),
    MLSPlannerNative.blueprint(
        world_frame="world",
        base_frame="base_link",
        voxel_size=0.05,
        robot_height=1.5,
        wall_clearance_m=0.35,
        step_threshold_m=0.06,
        viz_publish_hz=2.0,
    ).remappings(
        [
            (MLSPlannerNative, "local_map", "local_map_unused"),
            (MLSPlannerNative, "region_bounds", "region_bounds_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    DanLocalPlanner.blueprint(lock_replan=0.4, resample_spacing_m=0.0),
    DanHolonomicTC.blueprint(
        run_profile="walk", speed_m_s=0.2, goal_tolerance=0.15, control_frequency=10.0
    ),
    MovementManager.blueprint(),
).global_config(transport="zenoh", n_workers=6, robot_width=0.7, robot_rotation_diameter=0.9)


behavior_nav = autoconnect(
    _behavior_navigation,
    vis_module(global_config.viewer, rerun_config={"blueprint": _view}),
)


behavior_task = autoconnect(
    behavior_teleop,
    BehaviorConnection.blueprint(task=TaskSelection()),
).global_config(transport="zenoh")

behavior_agentic = autoconnect(
    behavior_task,
    BehaviorSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=SYSTEM_PROMPT),
).global_config(transport="zenoh")


_upper_body = [coordinator_name(name) for name in UPPER_BODY_JOINTS]
behavior_r1pro = (
    autoconnect(
        _behavior_navigation,
        BehaviorConnection.blueprint(
            task=TaskSelection(), headless=False, publish_scan=True, allow_task_changes=False
        ),
        BehaviorR1ProBridge.blueprint(),
        BehaviorCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tick_rate=30,
            hardware=[
                HardwareComponent(
                    hardware_id="r1pro",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=_upper_body,
                    adapter_type="transport_lcm",
                )
            ],
            tasks=[joint_trajectory_task(_upper_body)],
        ).remappings([(BehaviorCoordinator, "joint_command", "coordinator_joint_command")]),
        ManipulationModule.blueprint(
            model=simulation_model_config(),
            # Search only the selected arm/torso joints, retaining measured base context.
            planner=RRTConnectPlannerConfig(),
            planning_timeout=10.0,
            visualization=ViserVisualizationConfig(host=global_config.listen_host),
        ).remappings(
            [
                (ManipulationModule, "coordinator_joint_state", "planning_joint_state"),
                (ManipulationModule, "tf", "planning_tf"),
            ]
        ),
    )
    .transports(
        {
            ("motor_states", JointState): ZenohTransport.spec(
                ZenohTopic("dimos/r1pro/motor_states", JointState)
            ),
            ("motor_command", MotorCommandArray): ZenohTransport.spec(
                ZenohTopic("dimos/r1pro/motor_command", MotorCommandArray)
            ),
        }
    )
    .global_config(transport="zenoh", n_workers=8)
)
