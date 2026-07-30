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

"""Contract tests for the pinned RoboPlan TOPP-RA Python binding."""

from pathlib import Path
import sys

import numpy as np
import pytest

roboplan_core = pytest.importorskip("roboplan.core")
roboplan_toppra = pytest.importorskip("roboplan.toppra")

pytestmark = pytest.mark.self_hosted


def _scene(tmp_path: Path, *, acceleration: float | None) -> object:
    acceleration_attribute = "" if acceleration is None else f' acceleration="{acceleration}"'
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        f"""\
<robot name="contract_robot" version="1.2">
  <link name="base"/>
  <link name="tip"/>
  <joint name="joint" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="3"{acceleration_attribute}/>
  </joint>
</robot>
"""
    )
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(
        """\
<robot name="contract_robot">
  <group name="arm">
    <joint name="joint"/>
  </group>
</robot>
"""
    )
    return roboplan_core.Scene("contract_robot", urdf, srdf, [])


def test_roboplan_051_toppra_options_and_native_trajectory_contract(tmp_path: Path) -> None:
    scene = _scene(tmp_path, acceleration=2.0)
    options = roboplan_toppra.TOPPRAOptions(
        dt=0.02,
        mode=roboplan_toppra.SplineFittingMode.LinearBlend,
        velocity_scale=0.5,
        acceleration_scale=0.25,
        max_adaptive_iterations=7,
        max_adaptive_step_size=0.03,
        max_blend_deviation=0.01,
    )
    path = roboplan_core.JointPath()
    path.joint_names = ["joint"]
    path.positions = [
        np.asarray([0.0], dtype=np.float64),
        np.asarray([0.2], dtype=np.float64),
        np.asarray([0.4], dtype=np.float64),
    ]

    trajectory = roboplan_toppra.PathParameterizerTOPPRA(scene, "arm").generate(path, options)

    assert list(roboplan_toppra.SplineFittingMode) == [
        roboplan_toppra.SplineFittingMode.Hermite,
        roboplan_toppra.SplineFittingMode.Cubic,
        roboplan_toppra.SplineFittingMode.Adaptive,
        roboplan_toppra.SplineFittingMode.LinearBlend,
    ]
    assert options.dt == 0.02
    assert options.mode is roboplan_toppra.SplineFittingMode.LinearBlend
    assert options.velocity_scale == 0.5
    assert options.acceleration_scale == 0.25
    assert options.max_adaptive_iterations == 7
    assert options.max_adaptive_step_size == 0.03
    assert options.max_blend_deviation == 0.01
    assert trajectory.joint_names == ["joint"]
    assert len(trajectory.times) == len(trajectory.positions)
    assert len(trajectory.velocities) == len(trajectory.positions)
    assert len(trajectory.accelerations) == len(trajectory.positions)
    assert trajectory.times[0] == 0.0
    assert trajectory.times[-1] > 0.0
    assert np.allclose(trajectory.positions[0], [0.0])
    assert np.allclose(trajectory.positions[-1], [0.4])


def test_roboplan_051_missing_urdf_acceleration_is_effectively_unbounded(
    tmp_path: Path,
) -> None:
    scene = _scene(tmp_path, acceleration=None)

    lower, upper = scene.getAccelerationLimitVectors("arm")

    assert lower.tolist() == [-sys.float_info.max]
    assert upper.tolist() == [sys.float_info.max]
