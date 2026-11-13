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

"""Galaxea A1Z + Intel RealSense D405 live perception localize, teleop, and grasping blueprint.

When run in the terminal (`dimos run galaxea-a1z-localize` or
`uv run python -m dimos.perception.localize.demo_blueprints.galaxea_a1z_localize`),
this blueprint executes in sequence:

1. USB Hardware Guard: verifies that both the Galaxea A1Z USB-CANFD Adapter
   (`a8fa:8598`) and the Intel RealSense D405 Depth Camera (`8086:0b5b`) are
   connected over USB, printing their bus/device lines (or exiting with
   diagnostic output if either is missing).
2. CAN Setup Guard: runs `./bin/hardware/a1z/setup --yes` and
   `dimos hardware a1z configure-can --yes` (prompting for `sudo` when needed)
   to bring up and probe the `a1zcan` SocketCAN interface at 1 Mbit/s.
3. Launches the full stack:
   - `RealSenseCamera` streaming live RGB, aligned depth, camera_info, and pointcloud.
   - `ManipulationLocalize` with 5 mm voxel fusion, ghost-fragment filtering,
     plane-aware target exclusion + occluded support-plane shadow fill, and
     live 2D segmented mask + 3D object pointcloud Rerun streams.
   - `vis_module` (`dimos-viewer` / Rerun) displaying the live camera feed,
     2D segmented object overlays, and 3D detected object point clouds + boxes.
   - `SpaceMouseTeleopModule` mapping the 3Dconnexion SpaceMouse (`spacenavd`)
     to 6-DoF end-effector twist control **only while the global `Ctrl` key is
     held down**, immediately locking the arm in place in the air on release.
   - `GraspGenXModule` + `PickAndPlaceModule` + `ManipulationModule` +
     `ManipulationSkills` + `McpServer` for interactive `localize` and grasping
     via `dimos shell` or `dimos mcp call`.
"""

from __future__ import annotations

from dataclasses import replace
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
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.localize.types import LocalizeParams
from dimos.robot.manipulators.a1z.config import (
    A1Z_G1Z_MODEL_PATH,
    A1Z_GRASP_FRAME_TO_TCP,
    A1Z_GRIPPER_SWEEP_VOLUME,
    a1z_hardware,
    make_a1z_model_config,
)
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.teleop.spacemouse.spacemouse_teleop_module import SpaceMouseTeleopModule
from dimos.visualization.vis_module import vis_module

A1Z_USB_ID = "a8fa:8598"
D405_USB_ID = "8086:0b5b"
SPACEMOUSE_USB_VENDOR = "256f:"
OPTICAL_FRAME = "camera_color_optical_frame"
REPO_ROOT = Path(__file__).resolve().parents[4]

# Hand-eye transform from the A1Z wrist (arm_link6) to the D405 camera_link.
# Solved 2026-11-03 from 5 scans of a resting marker from different wrist poses.
A1Z_WRIST_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.0938, y=0.0098, z=0.0479),
    rotation=Quaternion(-0.01118, -0.04634, -0.00443, 0.99885),
    frame_id="arm_link6",
    child_frame_id="camera_link",
)


def check_usb_hardware(lsusb_output: str | None = None) -> tuple[str, str]:
    """Verify that the Galaxea A1Z USB-CANFD adapter and RealSense D405 are connected.

    Prints the matched `lsusb` lines and returns `(a1z_line, d405_line)`.
    Prints diagnostic details and exits with status 1 if either device is missing.
    """
    if lsusb_output is None:
        try:
            proc = subprocess.run(
                ["lsusb"],
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            print("ERROR: `lsusb` utility not found (`usbutils` package).", file=sys.stderr)
            sys.exit(1)
        if proc.returncode != 0:
            print(
                f"ERROR: `lsusb` failed (exit {proc.returncode}): {proc.stderr.strip()}",
                file=sys.stderr,
            )
            sys.exit(1)
        lsusb_output = proc.stdout

    lines = [line.strip() for line in lsusb_output.splitlines() if line.strip()]
    a1z_line = next((line for line in lines if A1Z_USB_ID in line.lower()), None)
    d405_line = next((line for line in lines if D405_USB_ID in line.lower()), None)
    spacemouse_line = next((line for line in lines if SPACEMOUSE_USB_VENDOR in line.lower()), None)

    print("• Galaxea A1Z USB-CANFD Adapter:")
    if a1z_line is not None:
        print(f"  {a1z_line}")
    else:
        print(f"  MISSING (expected USB ID {A1Z_USB_ID} Com Equipment CANFD Analyser)")

    print("• Intel RealSense D405 Depth Camera:")
    if d405_line is not None:
        print(f"  {d405_line}")
    else:
        print(
            f"  MISSING (expected USB ID {D405_USB_ID} Intel Corp. Intel(R) RealSense(TM) Depth Camera 405)"
        )

    if spacemouse_line is not None:
        print("• 3Dconnexion SpaceMouse:")
        print(f"  {spacemouse_line}")

    if a1z_line is None or d405_line is None:
        print(
            "\nERROR: Required hardware USB devices are not connected. "
            "Check USB cables and power for the Galaxea A1Z and RealSense D405.",
            file=sys.stderr,
        )
        sys.exit(1)

    return a1z_line, d405_line


def setup_can_interface(repo_root: Path = REPO_ROOT) -> None:
    """Run A1Z setup, SocketCAN (`a1zcan`) configuration, and `/dev/input/event*` read ACLs."""
    cmd = (
        "./bin/hardware/a1z/setup --yes "
        "&& source .venv/bin/activate "
        "&& dimos hardware a1z configure-can --yes "
        '&& (sudo -n setfacl -m u:"$(id -un)":r /dev/input/event* 2>/dev/null '
        '|| sudo setfacl -m u:"$(id -un)":r /dev/input/event* 2>/dev/null '
        "|| sudo chmod a+r /dev/input/event*)"
    )
    print(
        f"\n[galaxea-a1z-localize] Configuring CAN interface and input permissions in {repo_root}:"
    )
    print(f"  $ {cmd}", flush=True)
    proc = subprocess.run(
        ["bash", "-c", cmd],
        cwd=str(repo_root),
        check=False,
    )
    if proc.returncode != 0:
        print(
            f"\nERROR: Galaxea A1Z CAN setup failed with exit code {proc.returncode}.",
            file=sys.stderr,
        )
        sys.exit(proc.returncode or 1)
    print(
        "[galaxea-a1z-localize] CAN interface `a1zcan` and global SpaceMouse/Ctrl-key input ready.\n",
        flush=True,
    )


_GUARDS_COMPLETED = False


def run_hardware_guards() -> None:
    """Execute USB and CAN guards once before starting the blueprint."""
    global _GUARDS_COMPLETED
    if _GUARDS_COMPLETED:
        return
    check_usb_hardware()
    setup_can_interface()
    _GUARDS_COMPLETED = True


def _should_auto_run_guards_on_import() -> bool:
    """True when invoked via `dimos run galaxea-a1z-localize` in the main CLI process."""
    if "PYTEST_VERSION" in os.environ or global_config.simulation or global_config.replay:
        return False
    if os.environ.get("DIMOS_GALAXEA_SKIP_GUARDS") == "1":
        return False
    argv = sys.argv[1:]
    if "--help" in argv or "-h" in argv or "list" in argv:
        return False
    return "run" in argv and ("galaxea-a1z-localize" in argv or "galaxea_a1z_localize" in argv)


if _should_auto_run_guards_on_import():
    run_hardware_guards()


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


def _convert_detected_cloud(cloud: Any) -> Any:
    # The detected objects' points at the 5 mm they are fused at; the default spheres are 5 cm.
    return cloud.to_rerun(voxel_size=0.005)


rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/detected_pointcloud": _convert_detected_cloud,
        "world/depth_image": None,
    },
    "max_hz": {"world/color_image": 10},
    "memory_limit": "10%",
    "tf_axes": 0.1,
}

# Safe tabletop ready pose (~27 cm forward, ~35 cm up, slightly pitched down)
# with all 6 joints well away from the folded [0, 0, 0, ...] singularity and limits.
A1Z_TABLE_READY_JOINTS: list[float] = [0.0, 1.0, -1.0, 0.3, 0.0, 0.0]

_a1z_hw = a1z_hardware(
    "arm",
    has_gripper=True,
    address="a1zcan",
    dynamics_urdf_path=A1Z_G1Z_MODEL_PATH,
)
_a1z_model = make_a1z_model_config(
    has_gripper=True,
    home_joints=A1Z_TABLE_READY_JOINTS,
    base_pose=PoseStamped(frame_id="world"),
    tf_extra_links=["arm_link6"],
)


def _hardware_guard_requirement() -> str | None:
    if (
        "PYTEST_VERSION" not in os.environ
        and not global_config.simulation
        and not global_config.replay
        and os.environ.get("DIMOS_GALAXEA_SKIP_GUARDS") != "1"
    ):
        run_hardware_guards()
    return None


galaxea_a1z_localize = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
        RealSenseCamera.blueprint(enable_pointcloud=True),
        ManipulationLocalize.blueprint(
            world_frame="world",
            depth_scale=0.0001,
            params=replace(
                LocalizeParams(),
                fuse_voxel_m=0.005,
                min_camera_range_m=0.07,
                min_views=1,
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
            hardware=[_a1z_hw],
            tasks=[
                eef_twist_task(
                    _a1z_hw,
                    robot_model=_a1z_model,
                    target_frame="gripper_eef_link",
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
                trajectory_task(_a1z_hw, priority=20, start_position_tolerance=0.15),
            ],
        ),
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=_a1z_model,
            static_transforms=[A1Z_WRIST_CAMERA_TRANSFORM],
            planning_timeout=10.0,
            world_frame="world",
        ),
        GraspGenXModule.blueprint(
            gripper=A1Z_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=A1Z_GRASP_FRAME_TO_TCP,
            max_candidates=500,
        ),
        PickAndPlaceModule.blueprint(
            planning_frame="world",
            robot=_a1z_model,
            pregrasp_offset=0.06,
            grasp_depth_search=0.02,
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
    ModuleCoordinator.build(galaxea_a1z_localize).loop()
