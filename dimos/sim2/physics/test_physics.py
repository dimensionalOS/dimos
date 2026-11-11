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

from dimos_lcm.control_msgs import CborCommand
import numpy as np
import pytest

from dimos.sim2.geometry.poses import poses_by_id


def test_targets_push_box_and_do_not_move_other_robot(backend, poses, motor_commands):
    backend.apply_action(motor_commands(positions=(0.65,)), [])
    for _ in range(1000):
        backend.step()
    joints = backend.read_joints(("left/slide_x", "right/slide_x"))
    assert joints.position == pytest.approx([0.65, 0], abs=0.001)
    assert poses_by_id(poses())["box"].position.x > 0.15


def test_unsupported_cbor_rejects_whole_action_before_joint_mutation(backend, motor_commands):
    before = backend.data.ctrl.copy()
    with pytest.raises(ValueError, match="unsupported CBOR command"):
        backend.apply_action(
            motor_commands(positions=(0.5,)),
            [
                CborCommand(
                    entity_id="world",
                    key="unknown",
                    value_length=1,
                    value=b"\x01",
                )
            ],
        )
    np.testing.assert_array_equal(backend.data.ctrl, before)
    for _ in range(100):
        backend.step()
    assert backend.read_joints(("left/slide_x",)).position == pytest.approx([0])


def test_invalid_target_batch_is_atomic(backend, motor_commands):
    with pytest.raises(ValueError, match="not a driven joint"):
        backend.apply_action(
            motor_commands(names=("left/slide_x", "door/hinge"), positions=(0.5, 0.5)),
            [],
        )
    for _ in range(100):
        backend.step()
    assert backend.read_joints(("left/slide_x",)).position == pytest.approx([0])


@pytest.mark.parametrize("side,direction", [(0, -1), (1, 1)])
def test_joint_limit_roundoff_is_clamped_but_out_of_range_is_rejected(
    backend, motor_commands, side, direction
):
    limit = backend.model.joint("left/slide_x").range[side]
    backend.apply_action(motor_commands(positions=(limit + direction * 3e-12,), kp=100), [])
    aid = backend.model.actuator("left/slide_x").id
    assert backend.data.ctrl[aid] == 100 * limit
    before = backend.data.ctrl.copy()

    with pytest.raises(ValueError, match="target outside joint limits"):
        backend.apply_action(motor_commands(positions=(limit + direction * 1e-6,)), [])
    np.testing.assert_array_equal(backend.data.ctrl, before)


def test_motor_law_applies_velocity_gains_and_feedforward_each_step(backend, motor_commands):
    backend.apply_action(
        motor_commands(velocity=0.2, kp=40, kd=3, effort=1.5),
        [],
    )
    backend.step()
    joints = backend.read_joints(("left/slide_x",))
    assert joints.effort[0] == pytest.approx(
        40 * (0.2 - joints.position[0]) + 3 * (0.2 - joints.velocity[0]) + 1.5
    )


def test_sample_is_owned_and_complete(backend, poses, motor_commands):
    sample = poses()
    before = sample.lcm_encode()
    backend.apply_action(motor_commands(positions=(0.2,)), [])
    for _ in range(100):
        backend.step()
    assert sample.lcm_encode() == before
    assert set(poses_by_id(sample)) == set(backend.body_ids)


@pytest.mark.parametrize(
    "field,value", [("kp", -1), ("kd", float("nan")), ("effort", float("inf"))]
)
def test_bad_motor_batch_leaves_actuator_unchanged(backend, motor_commands, field, value):
    targets = motor_commands()
    next(row for row in targets if row.interface_name == field).values[0] = value
    before = backend.data.ctrl.copy()
    with pytest.raises(ValueError, match="finite with nonnegative gains"):
        backend.apply_action(targets, [])
    np.testing.assert_array_equal(backend.data.ctrl, before)


@pytest.mark.parametrize(
    "damage", ["missing", "duplicate", "unknown", "count", "values", "names", "different_joints"]
)
def test_malformed_interfaces_reject_entire_action(backend, motor_commands, damage):
    rows = motor_commands()
    if damage == "missing":
        rows.pop()
    elif damage == "duplicate":
        rows.append(rows[0])
    elif damage == "unknown":
        rows[0].interface_name = "stiffness_typo"
    elif damage == "count":
        rows[0].values_length = 2
    elif damage == "values":
        rows[0].values = []
        rows[0].values_length = 0
    elif damage == "names":
        rows[0].joint_names = [""]
    else:
        rows[1].joint_names = ["right/slide_x"]
    ctrl, gain, bias = (
        backend.data.ctrl.copy(),
        backend.model.actuator_gainprm.copy(),
        backend.model.actuator_biasprm.copy(),
    )
    with pytest.raises(ValueError):
        backend.apply_action(rows, [])
    np.testing.assert_array_equal(backend.data.ctrl, ctrl)
    np.testing.assert_array_equal(backend.model.actuator_gainprm, gain)
    np.testing.assert_array_equal(backend.model.actuator_biasprm, bias)


def test_interfaces_resolve_names_even_when_rows_are_reordered(backend, motor_commands):
    rows = motor_commands(names=("left/slide_x", "right/slide_x"), positions=(0.2, 0.1))
    rows[0].joint_names.reverse()
    rows[0].values.reverse()
    backend.apply_action(list(reversed(rows)), [])
    for _ in range(500):
        backend.step()
    assert backend.read_joints(("left/slide_x", "right/slide_x")).position == pytest.approx(
        [0.2, 0.1], abs=0.001
    )
