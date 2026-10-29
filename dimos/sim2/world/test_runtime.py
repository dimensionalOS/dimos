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

"""Physics applies standard motor commands, holds without them, and publishes coherent feedback."""

from dataclasses import replace
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


def _calibrate_motor(model, prefix):
    joint = model.joint(prefix + "hinge").id
    model.dof_damping[model.jnt_dofadr[joint]] = 2.75


def command(**overrides):
    values = dict(q=[0.3, 425], dq=[0.2, 0], kp=[2, 0], kd=[3, 0], tau=[0.4, 0])
    values.update(overrides)
    return MotorCommandArray(**values)


def test_compiled_calibration_is_scoped_to_its_robot(worlds, world_config):
    instance = world_config.robots["left"]
    fitted = replace(instance.config, configure_physics=_calibrate_motor)
    world = worlds(
        config=replace(
            world_config,
            robots={
                "left": replace(instance, config=fitted),
                "right": replace(instance, xyz=(2, 0, 0.3)),
            },
        )
    )
    left = world.robots["left"].dofs[0]
    right = world.robots["right"].dofs[0]
    assert world.model.dof_damping[[left, right]] == pytest.approx([2.75, 1])


def test_command_drives_motors_and_feedback_is_one_tick(worlds, feedback):
    world = worlds()
    samples = feedback(world)
    assert world.accept_command("left", command())

    world.step()

    binding = world.robots["left"]
    assert world.data.ctrl[binding.actuators] == pytest.approx([1.4, 0.02])
    robot_id, joints, imu, odom = samples[-1]
    assert robot_id == "left"
    assert joints.name == ["shoulder", "gripper"]
    assert joints.frame_id == imu.frame_id == "left/base"
    assert joints.ts == imu.ts == odom.ts
    assert imu.orientation.to_tuple() == pytest.approx([0, 0, 0, 1])
    assert odom.frame_id == "world"
    assert odom.position.to_tuple() == pytest.approx((0, 0, 0.3))
    # Snapshots for sensor workers are published every stride, not every tick.
    for _ in range(world.snapshot_stride - 1):
        world.step()
    snapshot = world.last_snapshot
    assert snapshot.metadata.physics_tick == world.snapshot_stride
    assert snapshot.metadata.sim_time == pytest.approx(world.data.time)


def test_native_servo_moves_and_preserves_gripper_units(worlds, feedback):
    world = worlds()
    samples = feedback(world)
    for _ in range(300):
        world.accept_command(
            "left", command(q=[0.25, 0], dq=[0, 0], kp=[10, 0], kd=[1, 0], tau=[0, 0])
        )
        world.step()
    joints = samples[-1][1]
    assert joints.position[0] == pytest.approx(0.25, abs=0.02)
    assert joints.position[1] == pytest.approx(0, abs=1)


@pytest.mark.parametrize(
    "overrides",
    [
        dict(q=[0.3], dq=[0.0], kp=[2.0], kd=[3.0], tau=[0.4]),
        dict(kp=[float("nan"), 0]),
        dict(kd=[-1, 0]),
        dict(tau=[0, 1]),
        dict(kp=[2, 5]),
        dict(timestamp=0.0),
    ],
)
def test_malformed_or_stale_commands_are_rejected_and_do_not_replace_the_held_target(
    worlds, overrides
):
    world = worlds()
    assert world.accept_command("left", command())
    assert not world.accept_command("left", command(**overrides))
    world.step()
    assert world.data.ctrl[world.robots["left"].actuators] == pytest.approx([1.4, 0.02])


def test_expired_command_falls_back_to_holding_the_last_pose(worlds, mocker):
    world = worlds()
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    binding = world.robots["left"]
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(3)
    position = world.data.qpos[binding.qpos[0]]
    velocity = world.data.qvel[binding.dofs[0]]
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)

    world.step()

    assert binding.action is None
    assert not binding.command_active
    assert binding.idle_position[0] == pytest.approx(position)
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(10 * 0 - velocity, abs=1e-6)


def test_connection_timeout_keeps_damping_until_reset_or_a_new_command(
    worlds, world_config, mocker
):
    instance = replace(world_config.robots["left"], halt_damping=4.0)
    world = worlds(config=replace(world_config, robots={"left": instance}))
    binding = world.robots["left"]
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)
    world.step()  # The command expires.

    # Moving away from the last pose must not restore startup stiffness.
    world.data.qpos[binding.qpos[0]] += 0.2
    world.data.qvel[binding.dofs[0]] = 0.2
    world.step()
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(-0.8)

    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[2, 0])
    )
    world.step()
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(2.0)

    world.reset()
    world.data.qpos[binding.qpos[0]] += 0.2
    world.data.qvel[binding.dofs[0]] = 0.0
    world.step()
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(-2.0)


def test_calibrated_actuator_lag_filters_the_control(worlds, world_config):
    instance = world_config.robots["left"]
    joint, gripper = instance.config.joints
    robot = replace(instance.config, joints=(replace(joint, actuator_time_constant=0.02), gripper))
    world = worlds(config=replace(world_config, robots={"left": replace(instance, config=robot)}))
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    aid = world.robots["left"].actuators[0]
    assert world.data.ctrl[aid] == pytest.approx(0.6)
    world.step()
    assert world.data.ctrl[aid] == pytest.approx(1.08)


def test_startup_wait_holds_until_the_first_command(worlds, world_config, feedback):
    instance = replace(world_config.robots["left"], wait_for_first_command=True)
    world = worlds(config=replace(world_config, robots={"left": instance}))
    samples = feedback(world)
    for _ in range(20):
        world.step()
    assert world.tick == 0
    assert len(samples) == 20
    assert samples[-1][1].position == pytest.approx([0.1, 850])

    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[10, 0], kd=[1, 0], tau=[0, 0])
    )
    world.step()

    assert world.tick == 1
    assert not world.waiting_for_control


def test_reset_restores_authored_state_and_starts_a_new_episode(worlds):
    world = worlds()
    snapshots = []
    world.publish_snapshot = snapshots.append
    world.accept_command("left", command())
    for _ in range(10):
        world.step()
    assert world.data.time > 0
    episode = world.episode
    previous = world.last_snapshot

    world.reset()

    current = world.last_snapshot
    assert current is snapshots[-1]
    assert world.episode == episode + 1
    assert current.metadata.episode_id == previous.metadata.episode_id + 1
    assert current.metadata.sequence > previous.metadata.sequence
    assert current.metadata.sim_time == 0
    assert world.robots["left"].action is None
    assert world.data.qpos[world.robots["left"].qpos[0]] == pytest.approx(0.1)
    # A command stamped before the reset is stale.
    assert not world.accept_command("left", command(timestamp=time.time() - 10))
    assert world.accept_command("left", command(timestamp=time.time()))


def test_pause_does_not_advance_physics_and_resume_starts_a_new_episode(worlds):
    world = worlds()
    before = world.data.qpos.copy()
    episode = world.episode
    world.set_paused(True)
    world.step()
    np.testing.assert_array_equal(world.data.qpos, before)
    assert world.tick == 0
    assert world.episode == episode
    world.set_paused(False)
    assert world.episode == episode + 1
    world.step()
    assert world.tick == 1


def test_post_step_snapshot_and_odometry_are_the_same_tick(worlds, world_config, feedback):
    instance = world_config.robots["left"]
    robot = replace(instance.config, floating=True)
    robot.model.write_text(
        robot.model.read_text().replace('<body name="base">', '<body name="base"><freejoint/>')
    )
    world = worlds(config=replace(world_config, robots={"left": replace(instance, config=robot)}))
    samples = feedback(world)
    root_joint = world.model.body("left/base").jntadr[0]
    qpos = world.model.jnt_qposadr[root_joint]
    dof = world.model.jnt_dofadr[root_joint]
    world.data.qvel[dof] = 1.0
    world.step()
    odom = samples[-1][3]
    assert world.data.qpos[qpos] > 0
    assert odom.position.x == pytest.approx(world.data.qpos[qpos])
    for _ in range(world.snapshot_stride - 1):
        world.step()
    assert world.last_snapshot.metadata.sim_time == pytest.approx(world.data.time)
    assert samples[-1][3].ts == pytest.approx(world.last_snapshot.values["wall_time"][0])


def test_robot_cannot_be_named_world(worlds):
    config = worlds().config
    with pytest.raises(ValueError, match="'world' is reserved"):
        replace(config, robots={"world": config.robots["left"]})
