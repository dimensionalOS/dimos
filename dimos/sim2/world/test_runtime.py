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
from functools import partial
import time

import mujoco
import numpy as np
import pytest
import robosuite
from robosuite.wrappers.domain_randomization_wrapper import DomainRandomizationWrapper

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.environment import MotorScene
from dimos.sim2.world.runtime import SimulationRuntime


def _fitted_model(factory, damping, position):
    model = factory()
    model.set_joint_attribute("damping", np.array([damping, 1.0]))
    model.worldbody.find("body").set("pos", position)
    return model


def command(**overrides):
    values = dict(q=[0.3, 425], dq=[0.2, 0], kp=[2, 0], kd=[3, 0], tau=[0.4, 0])
    values.update(overrides)
    return MotorCommandArray(**values)


def test_upstream_model_calibration_and_placement_are_scoped_to_each_robot(
    worlds, world_config, scene_path, robot_factory
):
    left = partial(_fitted_model, robot_factory, 2.75, "0 0 .3")
    right = partial(
        _fitted_model,
        partial(robot_factory.func, *robot_factory.args, idn="right"),
        1.0,
        "2 0 .3",
    )
    config = world_config | {
        "environment": partial(MotorScene, scene_path, {"left": left, "right": right}),
        "robots": {
            "left": replace(world_config["robots"]["left"], model=left),
            "right": replace(
                world_config["robots"]["left"], robot_id="right", model=right, frame_id="right/base"
            ),
        },
        "sensors": (),
    }
    world = worlds(config=config)
    left_dof = world.robots["left"].controller.dofs[0]
    right_dof = world.robots["right"].controller.dofs[0]
    assert world.model.dof_damping[[left_dof, right_dof]] == pytest.approx([2.75, 1])
    assert world.data.body("robotright_base").xpos == pytest.approx([2, 0, 0.3])


def test_motor_commands_follow_robot_ids_when_model_and_binding_order_differ(
    worlds, world_config, scene_path, robot_factory
):
    right_factory = partial(
        _fitted_model,
        partial(robot_factory.func, *robot_factory.args, idn="right"),
        1.0,
        "2 0 .3",
    )
    left = world_config["robots"]["left"]
    right = replace(left, robot_id="right", model=right_factory, frame_id="right/base")
    world = worlds(
        config=world_config
        | {
            "environment": partial(
                MotorScene, scene_path, {"left": robot_factory, "right": right_factory}
            ),
            "robots": {"right": right, "left": left},
            "sensors": (),
        }
    )
    for name, torque in (("left", 1.0), ("right", 2.0)):
        assert world.accept_command(
            name, command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[torque, 0])
        )

    world.step()

    assert world.data.ctrl[world.robots["left"].controller.actuators[0]] == pytest.approx(1.0)
    assert world.data.ctrl[world.robots["right"].controller.actuators[0]] == pytest.approx(2.0)


def test_command_drives_motors_and_feedback_is_one_tick(worlds, feedback):
    world = worlds()
    samples = feedback(world)
    assert world.accept_command("left", command())

    world.step()

    binding = world.robots["left"]
    controller = binding.controller
    assert world.data.ctrl[controller.actuators] == pytest.approx([1.4, 0.02])
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
    assert world.data.ctrl[world.robots["left"].controller.actuators] == pytest.approx([1.4, 0.02])


def test_expired_command_falls_back_to_holding_the_last_pose(worlds, mocker):
    world = worlds()
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    binding = world.robots["left"]
    controller = binding.controller
    assert world.data.ctrl[controller.actuators[0]] == pytest.approx(3)
    position = world.data.qpos[controller.qpos[0]]
    velocity = world.data.qvel[controller.dofs[0]]
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)

    world.step()

    assert binding.action is None
    assert not binding.command_active
    assert binding.idle_position[0] == pytest.approx(position)
    assert world.data.ctrl[controller.actuators[0]] == pytest.approx(10 * 0 - velocity, abs=1e-6)


def test_connection_timeout_keeps_damping_until_reset_or_a_new_command(
    worlds, world_config, mocker
):
    instance = replace(world_config["robots"]["left"], halt_damping=4.0)
    world = worlds(config=world_config | {"robots": {"left": instance}})
    binding = world.robots["left"]
    controller = binding.controller
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)
    world.step()  # The command expires.

    # Moving away from the last pose must not restore startup stiffness.
    world.data.qpos[controller.qpos[0]] += 0.2
    world.data.qvel[controller.dofs[0]] = 0.2
    world.step()
    assert world.data.ctrl[controller.actuators[0]] == pytest.approx(-0.8)

    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[2, 0])
    )
    world.step()
    assert world.data.ctrl[controller.actuators[0]] == pytest.approx(2.0)

    world.reset()
    world.data.qpos[controller.qpos[0]] += 0.2
    world.data.qvel[controller.dofs[0]] = 0.0
    world.step()
    assert world.data.ctrl[controller.actuators[0]] == pytest.approx(-2.0)


def test_authored_native_actuator_dynamics_are_used(worlds, robot_path):
    robot_path.write_text(
        robot_path.read_text().replace(
            '<motor name="motor"',
            '<general dyntype="filterexact" dynprm=".02" name="motor"',
        )
    )
    world = worlds()
    assert world.accept_command(
        "left", command(q=[0.1, 850], dq=[0, 0], kp=[0, 0], kd=[0, 0], tau=[3, 0])
    )
    world.step()
    actuator = world.robots["left"].controller.actuators[0]
    activation = world.model.actuator_actadr[actuator]
    assert world.data.ctrl[actuator] == pytest.approx(3)
    assert world.data.act[activation] == pytest.approx(3 * (1 - np.exp(-0.005 / 0.02)))
    world.step()
    assert world.data.act[activation] == pytest.approx(3 * (1 - np.exp(-0.01 / 0.02)))


def test_startup_wait_holds_until_the_first_command(worlds, world_config, feedback):
    instance = replace(world_config["robots"]["left"], wait_for_first_command=True)
    world = worlds(config=world_config | {"robots": {"left": instance}})
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


def test_reset_restores_authored_state_and_starts_a_new_episode(worlds, mocker):
    world = worlds()
    snapshots = []
    mocker.patch.object(world, "publish_snapshot", side_effect=snapshots.append)
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
    assert world.data.qpos[world.robots["left"].controller.qpos[0]] == pytest.approx(0.1)
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


def test_post_step_snapshot_and_odometry_are_the_same_tick(worlds, robot_path, feedback):
    robot_path.write_text(
        robot_path.read_text().replace(
            '<body name="base" pos="0 0 .3">',
            '<body name="base" pos="0 0 .3"><freejoint/>',
        )
    )
    world = worlds()
    samples = feedback(world)
    root_joint = world.model.body("robotleft_base").jntadr[0]
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


def test_robot_cannot_be_named_world(world_config):
    with pytest.raises(ValueError, match="'world' is reserved"):
        replace(world_config["robots"]["left"], robot_id="world")


def test_stock_panda_task_preserves_upstream_actions_observations_and_episode_lifecycle(mocker):
    environment = partial(
        robosuite.make,
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
        horizon=2,
        hard_reset=False,
    )
    results = []
    with SimulationRuntime(environment) as world:
        mocker.patch.object(world, "publish_environment", side_effect=results.append)
        assert world.robots == {}
        assert "cube_pos" in world.result.observations
        assert "robot0_joint_pos" in world.result.observations
        world.accept_action([0.0] * world.environment.action_dim)

        world.step()

        assert world.data.time == pytest.approx(0.05)
        assert world.tick == 25
        assert results[-1].reward == 0.0
        assert not results[-1].done
        assert np.isfinite(world.data.qpos).all()
        world.step()
        assert results[-1].done
        assert world.data.time == pytest.approx(0.1)
        world.step()
        assert world.data.time == pytest.approx(0.1)
        episode = world.episode
        world.reset()
        assert world.episode == episode + 1
        assert world.data.time == 0
        assert not world.result.done
        world.step()
        assert world.data.time == pytest.approx(0.05)


@pytest.mark.parametrize("action", [[], [float("nan")] * 7, [2.0] * 7])
def test_native_task_rejects_invalid_actions(action):
    environment = partial(
        robosuite.make,
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        hard_reset=False,
    )
    with SimulationRuntime(environment) as world:
        with pytest.raises(ValueError, match="action"):
            world.accept_action(action)


def test_native_task_snapshot_rate_uses_elapsed_time_across_physics_substeps(mocker):
    environment = partial(
        robosuite.make,
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        hard_reset=False,
        control_freq=50,
    )
    snapshots = []
    with SimulationRuntime(environment, snapshot_hz=10) as world:
        mocker.patch.object(world, "publish_snapshot", side_effect=snapshots.append)
        for _ in range(10):
            world.step()
        assert [frame.metadata.sim_time for frame in snapshots] == pytest.approx([0.1, 0.2])


def test_task_reward_and_done_are_published_when_observation_dictionary_is_empty(
    scene_path, mocker
):
    environment = partial(MotorScene, scene_path, {})
    results = []
    with SimulationRuntime(environment) as world:
        world.environment.ignore_done = False
        world.environment.horizon = 1
        mocker.patch.object(world.environment, "reward", return_value=3.5)
        mocker.patch.object(world, "publish_environment", side_effect=results.append)

        world.step()

        assert results[-1].observations == {}
        assert results[-1].reward == 3.5
        assert results[-1].done
        final_result = world.result
        world.step()
        assert world.result is final_result
        assert len(results) == 1


@pytest.mark.parametrize("motor_bound", [False, True])
def test_upstream_domain_randomization_reaches_physics_and_snapshot_model(
    world_config, motor_bound
):
    factory = (
        world_config["environment"]
        if motor_bound
        else partial(
            robosuite.make,
            "Lift",
            robots="Panda",
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            hard_reset=False,
        )
    )

    def randomized():
        return DomainRandomizationWrapper(
            factory(),
            seed=13,
            randomize_color=False,
            randomize_camera=True,
            randomize_lighting=True,
            randomize_dynamics=True,
            randomize_every_n_steps=0,
        )

    kwargs = {k: world_config[k] for k in ("robots", "sensors")} if motor_bound else {}
    with SimulationRuntime(randomized, **kwargs) as world:
        masses = world.model.body_mass.copy()
        if not motor_bound:
            world.accept_action([0.0] * len(world.environment.action_spec[0]))
        world.step()
        assert world.result.metadata.sim_time > 0
        assert world.result.observations
        previous_model = world.last_snapshot.model_path
        world.reset()
        assert not np.array_equal(world.model.body_mass, masses)
        assert world.last_snapshot.model_path != previous_model
        reader_model = mujoco.MjModel.from_binary_path(world.last_snapshot.model_path)
        np.testing.assert_array_equal(reader_model.body_mass, world.model.body_mass)
        assert world.result.metadata.sim_time == 0
        if motor_bound:
            assert set(world.robots) == {"left"}
