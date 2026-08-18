"""Blueprint surface tests for a complete LIBERO-PRO policy run."""

from pathlib import Path
import xml.etree.ElementTree as ET

from dimos.benchmark.libero_pro.blueprint import (
    PANDA_GRASP_FRAME_TO_TCP,
    PANDA_MODEL_PATH,
    libero_trial_blueprint,
)
from dimos.benchmark.libero_pro.connection import LiberoConnection, LiberoRecorder
from dimos.benchmark.libero_pro.video import LiberoVideoRecorder
from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.grasp_execution import GraspExecutionModule
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.perception.grounded_segmentation import GroundedSegmentationModule


def test_trial_blueprint_exposes_normal_control_manipulation_and_memory(
    tmp_path: Path,
) -> None:
    blueprint = libero_trial_blueprint(
        policy_endpoint="127.0.0.1:50051",
        discovery_address=str(tmp_path / "panda-shm"),
        memory_path=tmp_path / "recording.db",
        video_path=tmp_path / "trial.mp4",
    )
    atoms = {atom.module: atom for atom in blueprint.blueprints}

    assert {
        LiberoConnection,
        ControlCoordinator,
        ManipulationModule,
        GraspExecutionModule,
        GroundedSegmentationModule,
        GraspGenXModule,
        LiberoRecorder,
        LiberoVideoRecorder,
    } <= atoms.keys()
    hardware = atoms[ControlCoordinator].kwargs["hardware"]
    assert len(hardware) == 1
    assert hardware[0].adapter_type == "sim_mujoco"
    assert hardware[0].all_joints == [
        "panda/joint1",
        "panda/joint2",
        "panda/joint3",
        "panda/joint4",
        "panda/joint5",
        "panda/joint6",
        "panda/joint7",
        "panda/gripper",
    ]
    assert atoms[ControlCoordinator].kwargs["tick_rate"] == 20.0
    trajectory = atoms[ControlCoordinator].kwargs["tasks"][0]
    assert trajectory.params["start_position_tolerance"] == 0.2
    assert trajectory.params["goal_position_tolerance"] == 10.0
    robot_model = atoms[ManipulationModule].kwargs["robots"][0]
    assert robot_model.planning_groups[0].tip_link == "tcp"
    assert atoms[LiberoVideoRecorder].kwargs["output_path"] == tmp_path / "trial.mp4"
    assert atoms[LiberoRecorder].kwargs["record_tf"] is True
    assert atoms[LiberoRecorder].kwargs["stream_codecs"] == {
        "agentview_depth_image": "pickle",
        "eye_in_hand_depth_image": "pickle",
    }
    assert atoms[GraspGenXModule].kwargs["gripper"]["extents_open"] == (0.08, 0.04, 0.10)
    assert atoms[GraspGenXModule].kwargs["grasp_frame_to_tcp"] == PANDA_GRASP_FRAME_TO_TCP
    assert PANDA_GRASP_FRAME_TO_TCP[2][3] == 0.097
    assert atoms[GraspGenXModule].kwargs["max_candidates"] == 25


def test_recorder_has_no_privileged_evaluation_streams() -> None:
    public_inputs = {name for name in LiberoRecorder.__annotations__ if not name.startswith("_")}

    assert public_inputs == {
        "joint_state",
        "agentview_color_image",
        "agentview_depth_image",
        "agentview_camera_info",
        "eye_in_hand_color_image",
        "eye_in_hand_depth_image",
        "eye_in_hand_camera_info",
    }
    assert public_inputs.isdisjoint(
        {"reward", "success", "goal_predicates", "terminal_reason", "native_result"}
    )


def test_panda_collision_cylinders_use_franka_link_local_poses() -> None:
    root = ET.parse(PANDA_MODEL_PATH).getroot()
    expected = {
        "link1": ("0 0 -0.1915", "0.06", "0.283"),
        "link2": ("0 0 0", "0.06", "0.12"),
        "link3": ("0 0 -0.145", "0.06", "0.15"),
        "link4": ("0 0 0", "0.06", "0.12"),
        "link5": ("0 0 -0.26", "0.06", "0.10"),
        "link6": ("0 0 -0.03", "0.05", "0.08"),
        "link7": ("0 0 0.01", "0.04", "0.14"),
    }

    for name, (origin, radius, length) in expected.items():
        link = root.find(f"./link[@name='{name}']")
        assert link is not None
        collision_origin = link.find("./collision/origin")
        cylinder = link.find("./collision/geometry/cylinder")
        assert collision_origin is not None
        assert cylinder is not None
        assert collision_origin.attrib["xyz"] == origin
        assert cylinder.attrib == {"radius": radius, "length": length}
