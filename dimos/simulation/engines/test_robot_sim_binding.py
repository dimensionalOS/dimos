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

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

mujoco = pytest.importorskip("mujoco")

from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.robot_sim_binding import (
    RobotSimSpec,
    resolve_robot_sim_binding,
)
from dimos.simulation.utils.xml_parser import build_joint_mappings

pytestmark = pytest.mark.mujoco


def _write_scene_then_robot_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="binding-test">
  <option timestep="0.01"/>
  <worldbody>
    <body name="scene_prop" pos="1 0 0.2">
      <freejoint name="scene_free"/>
      <geom name="scene_geom" type="sphere" size="0.05" mass="1.0"/>
    </body>
    <body name="pelvis" pos="0 0 0.7">
      <freejoint name="floating_base_joint"/>
      <geom name="pelvis_geom" type="sphere" size="0.07" mass="1.0"/>
      <site name="imu_site" pos="0 0 0"/>
      <body name="hip_link" pos="0 0 -0.1">
        <joint name="hip_pitch_joint" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.03" mass="1.0"/>
      </body>
      <body name="knee_link" pos="0.1 0 -0.1">
        <joint name="knee_joint" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.03" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="knee_motor" joint="knee_joint"/>
    <motor name="hip_motor" joint="hip_pitch_joint"/>
  </actuator>
  <sensor>
    <framequat name="pelvis-orientation" objtype="body" objname="pelvis"/>
    <gyro name="pelvis-gyro" site="imu_site"/>
    <accelerometer name="pelvis-accel" site="imu_site"/>
    <velocimeter name="pelvis-linvel" site="imu_site"/>
  </sensor>
</mujoco>
""".strip()
    )


def _robot_spec(**overrides: Any) -> RobotSimSpec:
    kwargs: dict[str, Any] = dict(
        robot_id="testbot",
        hardware_joints=("testbot/hip_pitch", "testbot/knee"),
        root_body_names=("pelvis",),
        root_joint_names=("floating_base_joint",),
        require_floating_base=True,
        model_joint_names=("hip_pitch_joint", "knee_joint"),
        model_actuator_names=("hip_motor", "knee_motor"),
        imu_quat_names=("pelvis-orientation",),
        imu_gyro_names=("pelvis-gyro",),
        imu_accel_names=("pelvis-accel",),
        imu_linvel_names=("pelvis-linvel",),
        require_imu=True,
    )
    kwargs.update(overrides)
    return RobotSimSpec(**kwargs)


def test_robot_binding_ignores_scene_freejoint_and_uses_policy_order(tmp_path: Path) -> None:
    xml_path = tmp_path / "robot_scene.xml"
    _write_scene_then_robot_xml(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    binding = resolve_robot_sim_binding(model, _robot_spec(), build_joint_mappings(xml_path, model))

    assert binding.root_qpos_adr == 7
    assert binding.root_qvel_adr == 6
    assert binding.joint_qpos_adrs == (14, 15)
    assert binding.joint_qvel_adrs == (12, 13)
    assert binding.actuator_ids == (1, 0)
    assert [mapping.name for mapping in binding.joint_mappings] == [
        "hip_pitch_joint",
        "knee_joint",
    ]
    assert binding.imu_quat_slice == slice(0, 4)
    assert binding.imu_gyro_slice == slice(4, 7)
    assert binding.imu_accel_slice == slice(7, 10)
    assert binding.imu_linvel_slice == slice(10, 13)


def test_mujoco_engine_uses_robot_binding_joint_order(tmp_path: Path) -> None:
    xml_path = tmp_path / "robot_scene.xml"
    _write_scene_then_robot_xml(xml_path)

    engine = MujocoEngine(config_path=xml_path, headless=True, robot_sim_spec=_robot_spec())

    assert engine.robot_binding is not None
    assert engine.robot_binding.root_qpos_adr == 7
    assert engine.joint_names == ["hip_pitch_joint", "knee_joint"]
    assert engine.has_root_freejoint


def test_mujoco_engine_reset_joint_positions_sets_qpos_and_targets(tmp_path: Path) -> None:
    xml_path = tmp_path / "robot_scene.xml"
    _write_scene_then_robot_xml(xml_path)
    engine = MujocoEngine(config_path=xml_path, headless=True, robot_sim_spec=_robot_spec())

    engine.reset_joint_positions([0.25, -0.5])

    assert engine.joint_positions == [0.25, -0.5]
    assert engine.get_position_target(0) == 0.25
    assert engine.get_position_target(1) == -0.5
    assert engine.data.qpos[14] == pytest.approx(0.25)
    assert engine.data.qpos[15] == pytest.approx(-0.5)


def test_robot_binding_requires_configured_imu(tmp_path: Path) -> None:
    xml_path = tmp_path / "robot_scene.xml"
    _write_scene_then_robot_xml(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    mappings = build_joint_mappings(xml_path, model)

    with pytest.raises(ValueError, match="requires IMU"):
        resolve_robot_sim_binding(
            model,
            _robot_spec(imu_gyro_names=("missing-gyro",)),
            mappings,
        )


def test_robot_binding_rejects_bad_sensor_dimension(tmp_path: Path) -> None:
    xml_path = tmp_path / "robot_scene.xml"
    _write_scene_then_robot_xml(xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    mappings = build_joint_mappings(xml_path, model)

    with pytest.raises(ValueError, match="dim 3, expected 4"):
        resolve_robot_sim_binding(
            model,
            _robot_spec(imu_quat_names=("pelvis-gyro",)),
            mappings,
        )
