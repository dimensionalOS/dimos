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

"""Scene commands are acknowledged, applied at ticks, expired and reset together."""

from dataclasses import replace

import pytest

from dimos.sim2.physics import BodyWrench, CommandBatch, JointDrive, KinematicTarget


def test_one_step_wrench_reports_application_then_expires(worlds):
    world = worlds()
    reports = []
    world.publish_report = reports.append
    batch = CommandBatch("push", world.episode, (BodyWrench("box", force=(10, 0, 0)),))

    receipt = world.submit(batch)
    assert receipt.status == "accepted"
    assert world.last_state.bodies["box"].linear_velocity[0] == 0
    world.step()
    applied_velocity = world.last_state.bodies["box"].linear_velocity[0]
    world.step()

    # Passive dynamics cause tiny drift; holding the force would add 0.05 m/s.
    assert applied_velocity > 0
    assert world.last_state.bodies["box"].linear_velocity[0] == pytest.approx(
        applied_velocity, abs=1e-5
    )
    assert [report.status for report in reports] == ["accepted", "applied", "expired"]
    assert reports[1].tick > receipt.tick
    assert world.last_state.wall_time > 0


def test_duration_uses_sim_time_and_does_not_accumulate_a_held_force(worlds):
    world = worlds()
    batch = CommandBatch(
        "push",
        world.episode,
        (BodyWrench("box", force=(10, 0, 0)),),
        duration=2 * world.step_seconds,
    )
    assert world.submit(batch).status == "accepted"
    world.step()
    first = world.last_state.bodies["box"].linear_velocity[0]
    world.step()
    second = world.last_state.bodies["box"].linear_velocity[0]
    world.step()

    assert second == pytest.approx(2 * first, abs=1e-5)
    assert world.last_state.bodies["box"].linear_velocity[0] == pytest.approx(second, abs=1e-5)


def test_reset_clears_active_commands_and_rejects_old_epoch(worlds):
    world = worlds()
    batch = CommandBatch("push", world.episode, (BodyWrench("box", force=(10, 0, 0)),), duration=10)
    assert world.submit(batch).status == "accepted"
    world.step()
    old_epoch = world.episode
    world.reset()

    assert world.episode > old_epoch
    assert world.submit(batch).status == "rejected"
    world.step()
    assert world.last_state.bodies["box"].linear_velocity == pytest.approx((0, 0, 0))
    assert world.submit(replace(batch, epoch=world.episode)).status == "accepted"


def test_invalid_batch_is_rejected_without_applying_its_valid_prefix(worlds):
    world = worlds()
    batch = CommandBatch(
        "invalid", world.episode, (BodyWrench("box", force=(10, 0, 0)), JointDrive("missing"))
    )

    report = world.submit(batch)
    world.step()

    assert report.status == "rejected"
    assert "missing" in report.reason
    assert world.last_state.bodies["box"].linear_velocity == pytest.approx((0, 0, 0))


@pytest.mark.parametrize("target", ["joint", "kinematic"])
def test_conflicting_batches_reject_atomically_and_release_expired_targets(
    worlds, scene_path, target
):
    scene_path.write_text(
        scene_path.read_text().replace(
            "</worldbody>",
            '<body name="prop" mocap="true" pos="0 3 1"><geom type="sphere" size=".1"/></body></worldbody>',
        )
    )
    world = worlds()
    reports = []
    world.publish_report = reports.append
    first, second = (
        (JointDrive("left/hinge", effort=0.4), JointDrive("left/hinge", effort=0.8))
        if target == "joint"
        else (
            KinematicTarget("prop", (1, 3, 1), (0, 0, 0, 1)),
            KinematicTarget("prop", (2, 3, 1), (0, 0, 0, 1)),
        )
    )
    batch = CommandBatch("target", world.episode, (first,), duration=2 * world.step_seconds)
    assert world.submit(batch).status == "accepted"
    conflict = CommandBatch(
        "conflict", world.episode, (BodyWrench("box", force=(10, 0, 0)), second)
    )
    assert world.submit(conflict).status == "rejected"
    world.step()
    if target == "joint":
        assert world.last_state.joints["left/hinge"].actuator_effort == pytest.approx((0.4,))
    else:
        assert world.last_state.bodies["prop"].position == pytest.approx((1, 3, 1))
    assert world.last_state.bodies["box"].linear_velocity == pytest.approx((0, 0, 0))

    world.step()
    # Admission releases a completed command's ID and target even before the next tick.
    assert world.submit(replace(batch, commands=(second,))).status == "accepted"
    assert [report.status for report in reports[-2:]] == ["expired", "accepted"]
    world.step()
    if target == "joint":
        assert world.last_state.joints["left/hinge"].actuator_effort == pytest.approx((0.8,))
    else:
        assert world.last_state.bodies["prop"].position == pytest.approx((2, 3, 1))


def test_concurrent_wrench_batches_add_forces_on_the_same_body(worlds):
    world = worlds()
    for name, force in (("first", 10), ("second", 20)):
        batch = CommandBatch(name, world.episode, (BodyWrench("box", force=(force, 0, 0)),))
        assert world.submit(batch).status == "accepted"
    world.step()
    assert world.last_state.bodies["box"].linear_velocity[0] == pytest.approx(
        30 * world.step_seconds, abs=1e-5
    )
