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

"""Motor commands retain their hardware semantics inside a robosuite simulation."""

from contextlib import ExitStack

import numpy as np
import pytest
from robosuite.models.robots.robot_model import RobotModel
from robosuite.utils.binding_utils import MjSim

from dimos.sim2.control import MotorController


@pytest.fixture
def motor_model(tmp_path):
    path = tmp_path / "motors.xml"
    path.write_text("""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">
      <freejoint name="floating"/>
      <geom type="box" size=".1 .1 .1" mass="1"/>
      <body name="arm" pos="0 0 .3">
        <joint name="shoulder" axis="0 1 0" range="-1 1"/>
        <geom type="capsule" size=".03 .1" mass=".5"/>
        <body name="finger" pos="0 0 .2">
          <joint name="slide" type="slide" axis="1 0 0" range="0 .04"/>
          <geom type="box" size=".02 .02 .02" mass=".1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="shoulder" gear="2" ctrlrange="-8 8"/>
    <position name="grip" joint="slide" kp="50" ctrlrange="0 .04"/>
  </actuator>
</mujoco>
""")
    return RobotModel(str(path), idn="test")


@pytest.fixture
def simulations():
    with ExitStack() as stack:

        def create(model):
            sim = MjSim(model.get_model())
            stack.callback(sim.free)
            return sim

        yield create


@pytest.fixture
def motor_sim(motor_model, simulations):
    return simulations(motor_model)


@pytest.fixture
def controller(motor_sim, motor_model):
    return MotorController(motor_sim, motor_model, scale=[1, -0.04 / 850], offset=[0, 0.04])


def test_pd_uses_all_five_fields_in_actuator_order_and_excludes_the_free_base(controller):
    sim = controller.sim
    sim.data.qpos[controller.qpos] = [0.2, 0.01]
    sim.data.qvel[controller.dofs] = [0.1, 0]
    action = np.array([[0.5, 425], [0.7, 0], [3, 0], [2, 0], [0.4, 0]])

    controller.set_goal(action.ravel())
    controls = controller.run_controller()

    # Joint effort is 3*(.5-.2) + 2*(.7-.1) + .4 = 2.5, divided by gear 2.
    assert controls == pytest.approx([1.25, 0.02])
    assert controller.positions == pytest.approx([0.2, 637.5])
    assert controller.velocities == pytest.approx([0.1, 0])
    assert controller.qpos.tolist() == [7, 8]


def test_virtual_pd_target_is_not_joint_clamped_but_motor_output_is(controller):
    controller.set_goal(np.array([[100, -850], [0, 0], [1, 0], [0, 0], [0, 0]]))

    assert controller.run_controller() == pytest.approx([8, 0.04])


def test_native_position_servo_uses_its_mjcf_gain_once(controller):
    sim = controller.sim
    sim.data.qpos[controller.qpos] = [0, 0.01]
    controller.set_goal(np.array([[0, 212.5], [0, 0], [0, 0], [0, 0], [0, 0]]))
    sim.data.ctrl[controller.actuators] = controller.run_controller()
    sim.forward()

    assert sim.data.qfrc_actuator[controller.dofs] == pytest.approx([0, 1])


@pytest.mark.parametrize("row", [1, 2, 3, 4])
def test_native_servo_rejects_velocity_gain_and_effort_overrides(controller, row):
    previous = np.array([[0.5, 425], [0, 0], [0, 0], [0, 0], [2, 0]])
    controller.set_goal(previous)
    invalid = previous.copy()
    invalid[row, 1] = 1

    with pytest.raises(ValueError, match="position targets only"):
        controller.set_goal(invalid)

    assert controller.run_controller() == pytest.approx([1, 0.02])


@pytest.mark.parametrize(
    "action",
    [
        np.zeros(9),
        np.full(10, np.nan),
        np.array([[0, 0], [0, 0], [-1, 0], [0, 0], [0, 0]]),
        np.array([[0, 0], [0, 0], [0, 0], [-1, 0], [0, 0]]),
    ],
)
def test_invalid_action_preserves_previous_goal(controller, action):
    controller.set_goal(np.array([[0, 425], [0, 0], [0, 0], [0, 0], [2, 0]]))

    with pytest.raises(ValueError):
        controller.set_goal(action)

    assert controller.run_controller() == pytest.approx([1, 0.02])


def test_controller_uses_current_joint_feedback_on_every_physics_step(controller):
    controller.set_goal(np.array([[0.5, 425], [0, 0], [4, 0], [0, 0], [0, 0]]))
    assert controller.run_controller()[0] == pytest.approx(1)

    controller.sim.data.qpos[controller.qpos[0]] = 0.25

    assert controller.run_controller()[0] == pytest.approx(0.5)


def test_reset_clears_effort_and_holds_native_servo_at_measured_position(controller):
    controller.set_goal(np.array([[0.5, 425], [0, 0], [4, 0], [2, 0], [1, 0]]))
    controller.sim.data.qpos[controller.qpos] = [0.25, 0.01]

    controller.reset_goal()

    assert controller.run_controller() == pytest.approx([0, 0.01])


def test_unsupported_actuator_behavior_is_not_treated_as_a_position_servo(motor_model, simulations):
    position = motor_model.actuator.find("position")
    position.tag = "velocity"
    position.attrib.pop("kp")
    position.set("kv", "50")
    sim = simulations(motor_model)
    with pytest.raises(ValueError, match="effort motors and native position servos"):
        MotorController(sim, motor_model)
