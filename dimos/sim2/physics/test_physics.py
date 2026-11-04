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

from dimos_lcm.control_msgs import BodyWrench
import numpy as np
import pytest

from dimos.sim2.geometry.poses import pose_array, poses_by_id


def test_command_pushes_box_and_does_not_move_other_robot(simulation, command):
    simulation.submit(command(simulation, positions=(0.65,), ttl=5))
    simulation.advance(1000)
    poses, joints = simulation.inspect(("left/slide_x", "right/slide_x"))
    assert joints.position == pytest.approx([0.65, 0], abs=0.001)
    assert poses_by_id(poses)["box"].position.x > 0.15


def test_point_applied_wrench_moves_body_and_torque_moves_passive_joint(simulation):
    for body, force, torque, point in (
        ("box", [5, 0, 0], [0, 0, 0], [0, 0, 0.055]),
        ("door/panel", [0, 0, 0], [0, 0, 0.2], [0.6, 0.4, 0]),
    ):
        simulation.submit(
            BodyWrench(
                header=simulation.header(body),
                body_id=body,
                apply_at_ns=0,
                valid_until_ns=300_000_000,
                force_world=force,
                torque_world=torque,
                point_world=point,
            )
        )
    simulation.advance(150)
    poses, joints = simulation.inspect(("door/hinge",))
    assert poses_by_id(poses)["box"].position.x > 0.1
    assert joints.position[0] > 0.02


def test_reset_rejects_old_epoch_and_expired_servo_holds(simulation, command):
    simulation.submit(command(simulation, positions=(0.6,), ttl=0.02))
    simulation.advance(610)
    assert simulation.inspect(("left/slide_x",))[1].position[0] < 0.2
    simulation.submit(command(simulation, names=("right/slide_x",), positions=(0.7,)))
    old_epoch = simulation.epoch
    simulation.reset()
    simulation.advance(100)
    assert simulation.epoch == old_epoch + 1
    assert simulation.status()["rejected_commands"] == 1
    assert simulation.inspect(("right/slide_x",))[1].position == pytest.approx([0], abs=1e-7)


def test_invalid_batch_is_atomic_and_placement_is_privileged(simulation, command):
    simulation.submit(
        command(simulation, names=("left/slide_x", "door/hinge"), positions=(0.5, 0.5))
    )
    simulation.advance(100)
    assert simulation.status()["rejected_commands"] == 1
    assert simulation.inspect(("left/slide_x",))[1].position == pytest.approx([0])
    with pytest.raises(ValueError, match="floating"):
        simulation.place("left/base_link", (1, 0, 0, 0, 0, 0, 1))
    assert simulation.epoch == 0
    simulation.place("box", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    assert pose_array(poses_by_id(simulation.inspect()[0])["box"])[:3] == pytest.approx(
        [0.2, 0.2, 0.4]
    )
    assert simulation.time_ns == simulation.backend.data.time == 0


def test_motor_law_applies_velocity_gains_and_feedforward_each_step(simulation, command):
    action = command(simulation, kp=40, kd=3, effort=1.5)
    action.targets[0].velocity = 0.2
    simulation.submit(action)
    simulation.advance()
    joints = simulation.inspect(("left/slide_x",))[1]
    assert joints.effort[0] == pytest.approx(
        40 * (0.2 - joints.position[0]) + 3 * (0.2 - joints.velocity[0]) + 1.5
    )


def test_sample_is_owned_and_complete(simulation, command):
    sample = simulation.inspect()[0]
    before = sample.lcm_encode()
    simulation.submit(command(simulation))
    simulation.advance(100)
    assert sample.lcm_encode() == before
    assert set(poses_by_id(sample)) == set(simulation.backend.body_ids)


@pytest.mark.parametrize(
    "field,value", [("kp", -1), ("kd", float("nan")), ("feedforward", float("inf"))]
)
def test_bad_motor_batch_leaves_actuator_unchanged(simulation, command, field, value):
    action = command(simulation)
    setattr(action.targets[0], field, value)
    before = simulation.backend.data.ctrl.copy()
    simulation.submit(action)
    simulation.prepare()
    np.testing.assert_array_equal(simulation.backend.data.ctrl, before)
    assert simulation.status()["rejected_commands"] == 1


def test_wrong_clock_and_duplicate_commands_are_rejected(simulation, command):
    action = command(simulation)
    action.header.clock_id = "other"
    simulation.submit(action)
    simulation.prepare()
    good = command(simulation)
    simulation.submit(good)
    simulation.submit(good)
    simulation.prepare()
    assert simulation.status()["rejected_commands"] == 2
