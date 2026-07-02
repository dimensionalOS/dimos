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

"""Registry of samplable arms for the reachability capability map.

Each :class:`ArmModel` names a MuJoCo-loadable model (MJCF *or* URDF)
plus the joints and end-effector body that define one arm. construct /
evaluate / viewer are robot-agnostic — this module is the only place that
knows about a specific robot, so adding one is a single registry entry.

The capability map needs nothing from a model but kinematics + collision
geometry, which both MJCF and URDF carry. URDF models are loaded
collision-only (``discardvisual``), with ``package://`` references
expanded and ``strippath`` disabled so the real mesh paths survive
MuJoCo's compile step.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import TYPE_CHECKING

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.reachability.capability_map import MapParams
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    import mujoco


@dataclass(frozen=True)
class ArmModel:
    """One samplable arm: a MuJoCo-loadable model + the chain that defines it."""

    key: str
    model_path: Path
    joint_names: tuple[str, ...]  # the arm's actuated joints, as named in the model
    ee_body: str  # body whose frame is the tool flange
    base_link: str
    is_urdf: bool = False
    # For MJCF whose <compiler meshdir> is unset/relative (e.g. the G1 keeps
    # its STLs in data/g1_urdf/meshes). Ignored for URDF.
    model_meshdir: Path | None = None
    # For URDF: {ros_package_name: absolute_dir} to expand package:// refs.
    package_roots: dict[str, str | Path] = field(default_factory=dict)
    # TCP offset from ee_body's origin, in the ee_body local frame.
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Body-name pairs whose contact is a constant structural mesh overlap
    # (e.g. a link seated in its mount), not a real self-collision — excluded
    # from the sampler's rejection, exactly like a URDF's disabled adjacency.
    collision_exclude: tuple[tuple[str, str], ...] = ()
    # Height of the map-frame origin above ground. For a floating-base
    # humanoid the base is pinned here; for a table arm it is just where the
    # viewer stands the robot. 0 for fixed-base arms.
    base_height: float = 0.0
    # Explicit grid; None means construct() auto-sizes it from the workspace.
    params: MapParams | None = None
    # Optional URDF for the viewer's robot overlay (cells render without it).
    viewer_urdf: Path | None = None


def _g1(side: str) -> ArmModel:
    """One G1 arm, rooted at the pelvis (the floating base the WBC owns)."""
    joints = tuple(
        f"{side}_{j}"
        for j in (
            "shoulder_pitch_joint",
            "shoulder_roll_joint",
            "shoulder_yaw_joint",
            "elbow_joint",
            "wrist_roll_joint",
            "wrist_pitch_joint",
            "wrist_yaw_joint",
        )
    )
    # Calibrated grasp-center offsets from the wrist_yaw_link origin (the link
    # sits ~13 cm behind the palm); mirrored across the sagittal plane.
    grasp = (0.12, -0.05, 0.0) if side == "left" else (0.12, 0.05, 0.0)
    return ArmModel(
        key=f"g1-{side}",
        model_path=LfsPath("mujoco_sim/g1_gear_wbc.xml"),
        model_meshdir=LfsPath("g1_urdf/meshes"),
        joint_names=joints,
        ee_body=f"{side}_wrist_yaw_link",
        base_link="pelvis",
        grasp_offset=grasp,
        base_height=0.74,
        # The validated G1 grid: pelvis at WBC height, TCP up to ~1.8 m.
        params=MapParams(),
        viewer_urdf=LfsPath("g1_urdf/g1.urdf"),
        # g1.urdf references meshes as package://unitree_g1/meshes/... .
        package_roots={"unitree_g1": LfsPath("g1_urdf")},
    )


_REGISTRY: dict[str, ArmModel] = {
    "g1-left": _g1("left"),
    "g1-right": _g1("right"),
    "xarm7": ArmModel(
        key="xarm7",
        # Sample from the same description the viewer renders so the cells and
        # the robot overlay share one kinematic frame (the Menagerie MJCF sits
        # ~12 cm off the xarm_description URDF used for display).
        model_path=LfsPath("xarm_description/urdf/xarm7/xarm7.urdf"),
        is_urdf=True,
        joint_names=tuple(f"joint{i}" for i in range(1, 8)),
        ee_body="link7",
        base_link="link_base",
        # xarm gripper fingertips sit at link7-local [0, ±0.07, 0.10] — grasp
        # center is one finger-length out along the flange normal (+z).
        grasp_offset=(0.0, 0.0, 0.10),
        viewer_urdf=LfsPath("xarm_description/urdf/xarm7/xarm7.urdf"),
        package_roots={"xarm_description": LfsPath("xarm_description")},
    ),
    "piper": ArmModel(
        key="piper",
        model_path=LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml"),
        joint_names=tuple(f"joint{i}" for i in range(1, 7)),
        ee_body="link6",
        base_link="link1",
        # Grasp center one gripper-length out along link6's flange normal (+z).
        grasp_offset=(0.0, 0.0, 0.10),
        # link1's collision mesh is seated in the base; constant overlap.
        collision_exclude=(("link1", "world"),),
        viewer_urdf=LfsPath("piper_description/urdf/piper_no_gripper_description.urdf"),
        package_roots={"piper_description": LfsPath("piper_description")},
    ),
    "a750": ArmModel(
        key="a750",
        model_path=LfsPath("a750_description/urdf/a750_rev1_no_gripper.urdf"),
        is_urdf=True,
        package_roots={"a750_description": LfsPath("a750_description")},
        joint_names=tuple(f"joint{i}" for i in range(1, 7)),
        ee_body="link6",
        base_link="base_link",
        # a750 gripper fingers sit at link6-local [0.076, 0, 0] — its flange
        # normal is +x (not +z), so the grasp center is one length out along x.
        grasp_offset=(0.10, 0.0, 0.0),
        viewer_urdf=LfsPath("a750_description/urdf/a750_rev1_no_gripper.urdf"),
    ),
    "openarm": ArmModel(
        key="openarm",
        model_path=LfsPath("openarm_description/urdf/robot/openarm_v10_single.urdf"),
        is_urdf=True,
        package_roots={"openarm_description": LfsPath("openarm_description")},
        joint_names=tuple(f"openarm_left_joint{i}" for i in range(1, 8)),
        ee_body="openarm_left_link7",
        base_link="openarm_body_link0",
        # No hand in the arm-only model; grasp center one hand-length out along
        # the wrist flange normal (+z).
        grasp_offset=(0.0, 0.0, 0.10),
        viewer_urdf=LfsPath("openarm_description/urdf/robot/openarm_v10_single.urdf"),
        # The simplified (_symp) collision meshes of link5/link7 overlap
        # across the short link6 at every configuration.
        collision_exclude=(("openarm_left_link5", "openarm_left_link7"),),
    ),
}


def list_robots() -> list[str]:
    """Registry keys, in registration order (the CLI ``--robot`` choices)."""
    return list(_REGISTRY)


def arm_model(key: str) -> ArmModel:
    if key not in _REGISTRY:
        raise KeyError(f"unknown robot {key!r}; known: {', '.join(_REGISTRY)}")
    return _REGISTRY[key]


def robot_model_config(key: str, params: MapParams | None = None) -> RobotModelConfig:
    """Return a planning ``RobotModelConfig`` for a registered reachability arm."""
    arm = arm_model(key)
    map_params = params or arm.params or MapParams.at_base_height(arm.base_height)
    return RobotModelConfig(
        name=key,
        model_path=arm.model_path,
        base_pose=map_params.base_link_pose,
        joint_names=list(arm.joint_names),
        end_effector_link=arm.ee_body,
        base_link=arm.base_link,
        package_paths={pkg: Path(str(root)) for pkg, root in arm.package_roots.items()},
        collision_exclusion_pairs=list(arm.collision_exclude),
    )


def compile_model(
    model_path: str | Path,
    *,
    is_urdf: bool = False,
    model_meshdir: str | Path | None = None,
    package_roots: dict[str, str | Path] | None = None,
) -> mujoco.MjModel:
    """Compile an MJCF or URDF arm model into a ``mujoco.MjModel``.

    URDF is loaded collision-only: ``package://`` refs are expanded to the
    filesystem, ``strippath="false"`` keeps those absolute paths through
    compile, and ``discardvisual="true"`` drops visual meshes (often .dae,
    which MuJoCo can't read) — reachability only needs collision geometry.
    """
    import mujoco

    if is_urdf:
        text = Path(str(model_path)).read_text()
        for pkg, root in (package_roots or {}).items():
            text = text.replace(f"package://{pkg}/", f"{root}/")
        if "<mujoco>" not in text:
            text = re.sub(
                r"(<robot\b[^>]*>)",
                r'\1<mujoco><compiler strippath="false" discardvisual="true"/></mujoco>',
                text,
                count=1,
            )
        spec = mujoco.MjSpec.from_string(text)
    else:
        spec = mujoco.MjSpec.from_file(str(model_path))
        if model_meshdir:
            spec.meshdir = str(model_meshdir)
        else:
            spec.meshdir = str((Path(str(model_path)).parent / (spec.meshdir or "")).resolve())
    return spec.compile()


__all__ = ["ArmModel", "arm_model", "compile_model", "list_robots", "robot_model_config"]
