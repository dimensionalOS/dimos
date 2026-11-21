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

"""A three-joint toy arm (base yaw, two pitch joints) for the calibration math tests."""

from __future__ import annotations

from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.spec import WorkspaceGuard, build_arm_spec
from dimos.robot.assets.model import RobotModel


def _link(name: str, mass: float, com_x: float) -> str:
    return (
        f'<link name="{name}"><inertial><origin xyz="{com_x} 0 0.01"/><mass value="{mass}"/>'
        '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>'
    )


def _joint(name: str, parent: str, child: str, axis: str, x: float, roll: float = 0.0) -> str:
    rpy = f' rpy="{roll!r} 0 0"' if roll else ""
    return (
        f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{x} 0 0"{rpy}/><axis xyz="{axis}"/>'
        '<limit lower="-1.5" upper="1.5" effort="10" velocity="1"/></joint>'
    )


def toy_urdf(tilt_rad: float = 0.0) -> str:
    """The toy URDF, its base rolled by ``tilt_rad`` (a URDF that writes pi/2 as 1.5708 tilts
    by 3.7e-6 rad)."""
    return f"""<?xml version="1.0"?>
<robot name="toy">
  <link name="base"/>
  {_link("turntable", 0.4, 0.0)}
  {_link("upper", 1.0, 0.15)}
  {_link("fore", 0.5, 0.1)}
  <link name="tcp"/>
  {_joint("yaw", "base", "turntable", "0 0 1", 0.0, tilt_rad)}
  {_joint("shoulder", "turntable", "upper", "0 1 0", 0.0)}
  {_joint("elbow", "upper", "fore", "0 1 0", 0.3)}
  <joint name="tcp_mount" type="fixed"><parent link="fore"/><child link="tcp"/>
    <origin xyz="0.2 0 0"/></joint>
</robot>
"""


TOY_URDF = toy_urdf()


def toy_arm(tilt_rad: float = 0.0) -> ArmModel:
    """The toy arm on a bench; its yaw, tilted or not, is never measured."""
    guard = WorkspaceGuard.for_bench(floor_z_m=-0.45, min_x_m=-1.0)
    model = RobotModel.from_urdf_string(toy_urdf(tilt_rad))
    spec = build_arm_spec(
        "toy",
        model,
        ["yaw", "shoulder", "elbow"],
        measured_joints=["shoulder", "elbow"],
        workspace=guard,
    )
    arm = ArmModel(spec)
    arm.clearance_m = 0.02
    return arm
