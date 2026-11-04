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

from dimos_lcm.control_msgs import BodyWrench, JointTarget
import numpy as np
import pytest

from dimos.sim2.geometry.poses import poses_by_id


def test_targets_push_box_and_do_not_move_other_robot(backend, poses):
    backend.set_joint_targets([JointTarget(joint_id="left/slide_x", position=0.65, kp=200, kd=20)])
    for _ in range(1000):
        backend.step(())
    joints = backend.read_joints(("left/slide_x", "right/slide_x"))
    assert joints.position == pytest.approx([0.65, 0], abs=0.001)
    assert poses_by_id(poses())["box"].position.x > 0.15


def test_point_applied_wrench_moves_body_and_torque_moves_passive_joint(backend, poses):
    wrenches = tuple(
        BodyWrench(body_id=body, force_world=force, torque_world=torque, point_world=point)
        for body, force, torque, point in (
            ("box", [5, 0, 0], [0, 0, 0], [0, 0, 0.055]),
            ("door/panel", [0, 0, 0], [0, 0, 0.2], [0.6, 0.4, 0]),
        )
    )
    for _ in range(150):
        backend.step(wrenches)
    assert poses_by_id(poses())["box"].position.x > 0.1
    assert backend.read_joints(("door/hinge",)).position[0] > 0.02


def test_invalid_target_batch_is_atomic(backend):
    with pytest.raises(ValueError, match="not a driven joint"):
        backend.set_joint_targets(
            [
                JointTarget(joint_id="left/slide_x", position=0.5, kp=200, kd=20),
                JointTarget(joint_id="door/hinge", position=0.5, kp=200, kd=20),
            ]
        )
    for _ in range(100):
        backend.step(())
    assert backend.read_joints(("left/slide_x",)).position == pytest.approx([0])


def test_motor_law_applies_velocity_gains_and_feedforward_each_step(backend):
    backend.set_joint_targets(
        [
            JointTarget(
                joint_id="left/slide_x", position=0.2, velocity=0.2, kp=40, kd=3, feedforward=1.5
            )
        ]
    )
    backend.step(())
    joints = backend.read_joints(("left/slide_x",))
    assert joints.effort[0] == pytest.approx(
        40 * (0.2 - joints.position[0]) + 3 * (0.2 - joints.velocity[0]) + 1.5
    )


def test_sample_is_owned_and_complete(backend, poses):
    sample = poses()
    before = sample.lcm_encode()
    backend.set_joint_targets([JointTarget(joint_id="left/slide_x", position=0.2, kp=200, kd=20)])
    for _ in range(100):
        backend.step(())
    assert sample.lcm_encode() == before
    assert set(poses_by_id(sample)) == set(backend.body_ids)


@pytest.mark.parametrize(
    "field,value", [("kp", -1), ("kd", float("nan")), ("feedforward", float("inf"))]
)
def test_bad_motor_batch_leaves_actuator_unchanged(backend, field, value):
    target = JointTarget(joint_id="left/slide_x", position=0.2, kp=200, kd=20)
    setattr(target, field, value)
    before = backend.data.ctrl.copy()
    with pytest.raises(ValueError, match="finite with nonnegative gains"):
        backend.set_joint_targets([target])
    np.testing.assert_array_equal(backend.data.ctrl, before)
