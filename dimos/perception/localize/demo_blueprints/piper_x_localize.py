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

"""AgileX Piper X + Orbbec Gemini 335 live perception localize, teleop, and grasping blueprint.

`dimos run piper-x-localize` checks the Piper USB-CAN adapter and the Gemini 335,
brings `can0` up at 1 Mbit/s, then runs the camera, ManipulationLocalize, Rerun,
SpaceMouse teleop while `Ctrl` is held, planning, grasping, and McpServer.
"""

from __future__ import annotations

from dataclasses import replace
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.orbbec.camera import OrbbecCamera
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.common.blueprints import eef_twist_task, trajectory_task
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.piper.config import make_piper_hardware, make_piper_model_config
from dimos.teleop.spacemouse.spacemouse_teleop_module import SpaceMouseTeleopModule
from dimos.visualization.vis_module import vis_module

PIPER_CAN_USB_ID = "1d50:606f"
GEMINI_335_USB_ID = "2bc5:0800"
PIPER_CAN = "can0"
REPO_ROOT = Path(__file__).resolve().parents[4]

# Table scan: base faces forward (+X), gripper_base 35 cm up,
# gripper and camera pointing down at the table. joint5 stops at 70 degrees.
PIPER_X_TABLE_SCAN_JOINTS: list[float] = [
    0.0,
    math.radians(60.0),
    math.radians(-60.0),
    0.0,
    math.radians(65.0),
    0.0,
]

# gripper_base to the Gemini 335 camera_link (its left IR imager).
# Solved 2026-11-08 from 6 scans of a resting marker from different wrist poses; the arm
# sits on a 10 mm base plate, so the table is 0.010 m below base_link.
PIPER_X_WRIST_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=-0.0653, y=-0.0090, z=0.0374),
    rotation=Quaternion(-0.00845, -0.57440, -0.00943, 0.81847),
    frame_id="gripper_base",
    child_frame_id="camera_link",
)

# In GraspGenX's convention (approach along +Z, jaws closing along X) with origin on
# gripper_base: fingertips reach 0.140 m (measured), the distal pad spans 0.035 m back
# from the tip, and the adapter opens the jaws to 0.08 m.
PIPER_X_GRIPPER_SWEEP_VOLUME: dict[str, Any] = {
    "extents_open": (0.080, 0.030, 0.035),
    "offset_open": (0.0, 0.0, 0.1225),
    "extents_half_open": (0.040, 0.030, 0.035),
    "offset_half_open": (0.0, 0.0, 0.1225),
    "fingertip_depth": 0.140,
}

# Maps GraspGenX's grasp frame (+Z approach, X closing) to gripper_base (+Z approach, Y closing).
PIPER_X_GRASP_FRAME_TO_TCP: tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
] = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


OPTICAL_FRAME = "camera_color_optical_frame"


def _rerun_blueprint() -> Any:
    """Rerun layout: live Camera + Segmented 2D views beside the 3D scene & object clouds."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.Spatial2DView(origin="world/segmented_image", name="Segmented Objects"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.1)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="expanded"),
        rrb.SelectionPanel(state="hidden"),
    )


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(image_topic="/world/color_image", optical_frame=OPTICAL_FRAME)


def _cloud(cloud: Any) -> Any:
    # The default spheres are 5 cm across; at 5 mm the table no longer buries small objects.
    return cloud.to_rerun(voxel_size=0.005)


_rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/depth_image": None,
        # the camera cloud, and the points of the detected objects
        "world/pointcloud": _cloud,
        "world/detected_pointcloud": _cloud,
    },
    "max_hz": {"world/color_image": 10},
    "memory_limit": "10%",
    "tf_axes": 0.1,
}


def check_usb_hardware() -> None:
    """Exit unless the Piper USB-CAN adapter and the Gemini 335 are on USB."""
    lsusb = subprocess.run(["lsusb"], capture_output=True, text=True, check=False).stdout.lower()
    missing = [
        name
        for name, usb_id in (
            ("Piper USB-CAN adapter", PIPER_CAN_USB_ID),
            ("Orbbec Gemini 335", GEMINI_335_USB_ID),
        )
        if usb_id not in lsusb
    ]
    if missing:
        print(f"ERROR: not connected over USB: {', '.join(missing)}", file=sys.stderr)
        sys.exit(1)


def setup_can_interface() -> None:
    """Bring `can0` up at 1 Mbit/s and give this user read access to `/dev/input/event*`."""
    cmd = (
        f"sudo ip link set {PIPER_CAN} down "
        f"&& sudo ip link set {PIPER_CAN} type can bitrate 1000000 "
        f"&& sudo ip link set {PIPER_CAN} up "
        '&& (sudo setfacl -m u:"$(id -un)":r /dev/input/event* 2>/dev/null '
        "|| sudo chmod a+r /dev/input/event*)"
    )
    print(f"[piper-x-localize] $ {cmd}", flush=True)
    proc = subprocess.run(["bash", "-c", cmd], cwd=str(REPO_ROOT), check=False)
    if proc.returncode != 0:
        print(f"ERROR: Piper CAN setup failed with exit code {proc.returncode}.", file=sys.stderr)
        sys.exit(proc.returncode or 1)


_GUARDS_COMPLETED = False


def run_hardware_guards() -> None:
    global _GUARDS_COMPLETED
    if _GUARDS_COMPLETED:
        return
    check_usb_hardware()
    setup_can_interface()
    _GUARDS_COMPLETED = True


def _hardware_guard_requirement() -> str | None:
    if (
        "PYTEST_VERSION" not in os.environ
        and not global_config.simulation
        and not global_config.replay
        and os.environ.get("DIMOS_PIPER_SKIP_GUARDS") != "1"
    ):
        run_hardware_guards()
    return None


def _should_auto_run_guards_on_import() -> bool:
    """True when invoked via `dimos run piper-x-localize` in the main CLI process."""
    if "PYTEST_VERSION" in os.environ or global_config.simulation or global_config.replay:
        return False
    if os.environ.get("DIMOS_PIPER_SKIP_GUARDS") == "1":
        return False
    argv = sys.argv[1:]
    if "--help" in argv or "-h" in argv or "list" in argv:
        return False
    return "run" in argv and ("piper-x-localize" in argv or "piper_x_localize" in argv)


if _should_auto_run_guards_on_import():
    run_hardware_guards()


_piper_base_model = make_piper_model_config(home_joints=PIPER_X_TABLE_SCAN_JOINTS)
_piper_model = _piper_base_model.model_copy(
    update={
        # The folded arm rests a few hundredths of a radian past these zero limits.
        "model": _piper_base_model.model.with_joint_position_limits(
            "joint2", lower=-0.06, upper=math.pi
        ).with_joint_position_limits("joint3", lower=-2.9670597, upper=0.06),
        "base_pose": PoseStamped(frame_id="world"),
        # At the folded rest the wrist (link5) sits on the base (link1).
        "collision_exclusion_pairs": [
            *_piper_base_model.collision_exclusion_pairs,
            ("link1", "link5"),
        ],
        "gripper": GripperConfig(
            pad_links=("gripper_link1", "gripper_link2"),
            # gripper_joint1/2 mimic this jaw-opening joint.
            finger_joints={"gripper": (0.08, 0.0)},
            closing_steps=8,
        ),
    }
)
_piper_hw = make_piper_hardware("arm", adapter_type="piper", address=PIPER_CAN)

piper_x_localize = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        OrbbecCamera.blueprint(enable_pointcloud=True, rotate_180=True),
        ManipulationLocalize.blueprint(
            world_frame="world",
            depth_scale=0.001,
            params=replace(
                LocalizeParams(),
                fuse_voxel_m=0.005,
                min_camera_range_m=0.15,
                min_views=1,
                # 44% of the tape's lifted points were the table, seen at its mask edges.
                support_clearance_m=0.002,
            ),
        ),
        SpaceMouseTeleopModule.blueprint(
            deadman_key="ctrl",
            translation_only=True,
            decouple_arm_and_gripper=True,
            linear_speed=0.10,
            angular_speed=0.35,
            deadzone=0.08,
        ),
        ArmTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[_piper_hw],
            tasks=[
                eef_twist_task(
                    _piper_hw,
                    robot_model=_piper_model,
                    target_frame="gripper_base",
                    max_joint_velocity_rad_s=1.2,
                    feedback_limit_tolerance=0.15,
                    command_limit_margin=0.015,
                    pink=PinkKinematicsConfig(
                        position_cost=5.0,
                        orientation_cost=0.25,
                        posture_cost=1e-2,
                        damping=1e-4,
                        lm_damping=1e-3,
                    ),
                ),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
                trajectory_task(_piper_hw, priority=20, start_position_tolerance=0.15),
            ],
        ),
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=_piper_model,
            static_transforms=[PIPER_X_WRIST_CAMERA_TRANSFORM],
            planning_timeout=10.0,
            world_frame="world",
        ),
        GraspGenXModule.blueprint(
            gripper=PIPER_X_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=PIPER_X_GRASP_FRAME_TO_TCP,
            num_samples=200,
            max_candidates=200,
            planner="graspmoe",
        ),
        PickAndPlaceModule.blueprint(
            planning_frame="world",
            robot=_piper_model,
            pregrasp_offset=0.06,
            grasp_depth_search=0.02,
            grasp_obstacle_margin=0.001,
            grasp_scene_from_answer=True,
            grasp_verification=GraspVerificationConfig(empty_epsilon=0.04),
        ),
        ManipulationSkills.blueprint(),
        McpServer.blueprint(),
    )
    .requirements(_hardware_guard_requirement)
    .global_config(n_workers=6)
)


if __name__ == "__main__":
    run_hardware_guards()
    ModuleCoordinator.build(piper_x_localize).loop()
