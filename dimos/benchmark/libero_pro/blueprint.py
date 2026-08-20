"""Complete policy-only DimOS blueprint for one LIBERO-PRO trial."""

from __future__ import annotations

from pathlib import Path

from dimos.benchmark.libero_pro.connection import LiberoConnection, LiberoRecorder
from dimos.benchmark.libero_pro.video import LiberoVideoRecorder
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasp_execution import GraspExecutionModule
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.memory2.module import OnExisting
from dimos.perception.grounded_segmentation import GroundedSegmentationModule
from dimos.robot.manipulators._modeling import base_pose, coordinator_joint_mapping, joint_names
from dimos.robot.manipulators.common.blueprints import trajectory_task

PANDA_MODEL_PATH = Path(__file__).with_name("panda.urdf")
PANDA_GRASP_FRAME_TO_TCP = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.097),
    (0.0, 0.0, 0.0, 1.0),
)


def _panda_model() -> RobotModelConfig:
    local_joints = joint_names(7)
    return RobotModelConfig(
        name="panda",
        model_path=PANDA_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=local_joints,
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joints),
                base_link="base",
                tip_link="tcp",
            )
        ],
        joint_limits_lower=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
        joint_limits_upper=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
        # The connection turns each target into LIBERO's native OSC pose action.
        # Keep planner timing effectively instantaneous so real-time policies
        # control motion through measured setpoint updates rather than a second
        # host-side rate limiter.
        velocity_limits=[100.0] * 7,
        max_velocity=100.0,
        max_acceleration=200.0,
        joint_name_mapping=coordinator_joint_mapping("panda", 7),
        gripper_hardware_id="panda",
    )


def libero_trial_blueprint(
    *,
    policy_endpoint: str,
    discovery_address: str,
    memory_path: Path,
    video_path: Path,
) -> Blueprint:
    panda = HardwareComponent(
        hardware_id="panda",
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints("panda", 7),
        gripper_joints=["panda/gripper"],
        adapter_type="sim_mujoco",
        address=discovery_address,
        gripper_open_position=0.04,
        gripper_closed_position=0.0,
    )
    return autoconnect(
        LiberoConnection.blueprint(
            endpoint=policy_endpoint,
            discovery_address=discovery_address,
        ),
        ControlCoordinator.blueprint(
            tick_rate=20.0,
            hardware=[panda],
            tasks=[
                trajectory_task(
                    panda,
                    start_position_tolerance=0.2,
                    # LIBERO tracks each commanded joint vector as its
                    # equivalent OSC end-effector setpoint. A redundant arm
                    # can reach that pose without returning to the same joint
                    # vector, so completion is the nominal trajectory duration
                    # while the final OSC setpoint remains held.
                    goal_position_tolerance=10.0,
                )
            ],
        ),
        ManipulationModule.blueprint(robots=[_panda_model()]),
        GraspExecutionModule.blueprint(),
        GroundedSegmentationModule.blueprint(),
        GraspGenXModule.blueprint(
            gripper={
                "extents_open": (0.08, 0.04, 0.10),
                "offset_open": (0.0, 0.0, 0.05),
                "extents_half_open": (0.04, 0.04, 0.10),
                "offset_half_open": (0.0, 0.0, 0.05),
                "fingertip_depth": 0.10,
                "family": "parallel_2f",
            },
            grasp_frame_to_tcp=PANDA_GRASP_FRAME_TO_TCP,
            max_candidates=25,
        ),
        LiberoRecorder.blueprint(
            db_path=memory_path,
            on_existing=OnExisting.OVERWRITE,
            record_tf=True,
            stream_codecs={
                "agentview_depth_image": "pickle",
                "eye_in_hand_depth_image": "pickle",
            },
            poseless_streams=[
                "joint_state",
                "agentview_color_image",
                "agentview_depth_image",
                "agentview_camera_info",
                "eye_in_hand_color_image",
                "eye_in_hand_depth_image",
                "eye_in_hand_camera_info",
            ],
        ),
        LiberoVideoRecorder.blueprint(output_path=video_path),
    )
