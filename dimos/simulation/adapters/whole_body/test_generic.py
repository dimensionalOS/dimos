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

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import time

import pytest

pytest.importorskip("mujoco")

from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter
from dimos.simulation.adapters.whole_body.generic import SimMujocoWholeBodyAdapter
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_shm import ManipShmWriter, shm_key_from_path
from dimos.simulation.engines.mujoco_sim_module import _WholeBodySimHooks
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec

pytestmark = pytest.mark.mujoco

# Two 2-dof chains plus one slider each, matching the dual-arm shape the
# generic adapter exists for. Position actuators close their own loop; the
# "*_motor" variant below exercises the PD-with-feedforward path instead.
_SCENE = """
<mujoco model="dual-chain">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    {bodies}
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
</mujoco>
"""

_ARM = """
    <body name="{side}_base" pos="0 {y} 0">
      <body name="{side}_link1">
        <joint name="{side}_joint1" type="hinge" axis="0 0 1" range="-2 2"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.5"/>
        <body name="{side}_link2" pos="0.2 0 0">
          <joint name="{side}_joint2" type="hinge" axis="0 1 0" range="-2 2"/>
          <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.5"/>
          <body name="{side}_finger" pos="0.2 0 0">
            <joint name="{side}_slide" type="slide" axis="0 1 0" range="0 0.05"/>
            <geom type="box" size="0.01 0.01 0.01" mass="0.1"/>
          </body>
        </body>
      </body>
    </body>
"""

_JOINTS = (
    "left_joint1",
    "left_joint2",
    "left_slide",
    "right_joint1",
    "right_joint2",
    "right_slide",
)
_HARDWARE_JOINTS = (
    "left/joint1",
    "left/joint2",
    "left_arm/gripper",
    "right/joint1",
    "right/joint2",
    "right_arm/gripper",
)


def _write_scene(path: Path, *, actuator: str) -> None:
    bodies = "".join(_ARM.format(side=side, y=y) for side, y in (("left", 0.3), ("right", -0.3)))
    actuators = "\n".join(
        f'<{actuator} name="{joint}_act" joint="{joint}"'
        + (' kp="200" kv="10"/>' if actuator == "position" else "/>")
        for joint in _JOINTS
    )
    path.write_text(_SCENE.format(bodies=bodies, actuators=actuators).strip())


def _spec() -> RobotSimSpec:
    return RobotSimSpec(
        robot_id="dual_chain",
        hardware_joints=_HARDWARE_JOINTS,
        model_joint_names=_JOINTS,
        model_actuator_names=tuple(f"{joint}_act" for joint in _JOINTS),
    )


class _Sim:
    """Sim side of the SHM bridge: engine + writer + the module's step hooks."""

    def __init__(self, xml_path: Path) -> None:
        self.shm = ManipShmWriter(shm_key_from_path(xml_path))
        self.engine = MujocoEngine(config_path=xml_path, headless=True, robot_sim_spec=_spec())
        hooks = _WholeBodySimHooks(self.shm, dof=len(_JOINTS))
        self.engine.set_step_hooks(before=hooks.pre_step, after=self._post_step)
        self._hooks = hooks
        assert self.engine.connect()

    def _post_step(self, engine: MujocoEngine) -> None:
        self._hooks.post_step(engine)
        self.shm.signal_ready(num_joints=len(engine.joint_names), arm_joints=len(_JOINTS))

    def close(self) -> None:
        self.engine.disconnect()
        self.shm.cleanup()


@pytest.fixture
def position_sim(tmp_path: Path) -> Iterator[tuple[_Sim, Path]]:
    xml_path = tmp_path / "position.xml"
    _write_scene(xml_path, actuator="position")
    sim = _Sim(xml_path)
    yield sim, xml_path
    sim.close()


def _settle(adapter: SimMujocoWholeBodyAdapter, index: int, target: float) -> float:
    deadline = time.monotonic() + 5.0
    reached = adapter.read_motor_states()[index].q
    while time.monotonic() < deadline and abs(reached - target) > 1e-3:
        time.sleep(0.02)
        reached = adapter.read_motor_states()[index].q
    return reached


def test_position_commands_drive_every_configured_joint(
    position_sim: tuple[_Sim, Path],
) -> None:
    _, xml_path = position_sim
    adapter = SimMujocoWholeBodyAdapter(
        address=xml_path, num_motors=len(_JOINTS), command_mode="position"
    )
    assert isinstance(adapter, WholeBodyAdapter)
    assert adapter.connect()

    targets = [0.4, -0.3, 0.04, -0.4, 0.3, 0.02]
    assert adapter.write_motor_commands([MotorCommand(q=q) for q in targets])
    for index, target in enumerate(targets):
        assert _settle(adapter, index, target) == pytest.approx(target, abs=5e-3)

    states = adapter.read_motor_states()
    assert len(states) == len(_JOINTS)
    # Arm-only MJCFs publish no IMU, so the adapter falls back to the default.
    assert adapter.read_imu().quaternion == (1.0, 0.0, 0.0, 0.0)
    adapter.disconnect()


def test_pd_commands_reach_a_torque_actuated_sim(tmp_path: Path) -> None:
    xml_path = tmp_path / "motor.xml"
    _write_scene(xml_path, actuator="motor")
    sim = _Sim(xml_path)
    adapter = SimMujocoWholeBodyAdapter(address=xml_path, num_motors=len(_JOINTS))
    try:
        assert adapter.connect()
        target = 0.35
        assert adapter.write_motor_commands(
            [MotorCommand(q=target, kp=60.0, kd=4.0) for _ in _JOINTS]
        )
        assert _settle(adapter, 0, target) == pytest.approx(target, abs=2e-2)
    finally:
        adapter.disconnect()
        sim.close()


def test_stop_sentinel_holds_the_measured_position(position_sim: tuple[_Sim, Path]) -> None:
    _, xml_path = position_sim
    adapter = SimMujocoWholeBodyAdapter(
        address=xml_path, num_motors=len(_JOINTS), command_mode="position"
    )
    assert adapter.connect()
    assert adapter.write_motor_commands([MotorCommand(q=0.3) for _ in _JOINTS])
    held = _settle(adapter, 0, 0.3)

    assert adapter.write_motor_commands([MotorCommand() for _ in _JOINTS])
    time.sleep(0.2)
    assert adapter.read_motor_states()[0].q == pytest.approx(held, abs=5e-3)
    adapter.disconnect()


def test_require_imu_rejects_a_sim_without_one(position_sim: tuple[_Sim, Path]) -> None:
    _, xml_path = position_sim
    adapter = SimMujocoWholeBodyAdapter(
        address=xml_path, num_motors=len(_JOINTS), command_mode="position", require_imu=True
    )
    assert not adapter.connect()


def test_configuration_errors_are_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="address"):
        SimMujocoWholeBodyAdapter(num_motors=4)
    with pytest.raises(ValueError, match="num_motors"):
        SimMujocoWholeBodyAdapter(address=tmp_path / "x.xml")
    with pytest.raises(ValueError, match="command_mode"):
        SimMujocoWholeBodyAdapter(address=tmp_path / "x.xml", num_motors=4, command_mode="torque")
