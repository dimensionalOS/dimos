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

from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.module import SimulationModule
from dimos.sim2.world.evidence import file_identity
from dimos.sim2.world.scene_types import SceneUpdate


def _calibrate_motor(model, prefix):
    joint = model.joint(prefix + "hinge").id
    model.dof_damping[model.jnt_dofadr[joint]] = 2.75


def test_run_evidence_survives_shutdown_without_copying_assets(world_config, tmp_path):
    instance = world_config.robots["left"]
    world = replace(
        world_config,
        robots={
            "left": replace(
                instance, controller={"implementation": "test-controller", "revision": "abc123"}
            )
        },
        calibration_evidence={"left.shoulder.damping": "capture-sha256/example-fit-v1"},
        sampled_variation={
            "left.xyz": {"distribution": "fixed", "seed": 123, "value": instance.xyz}
        },
    )
    module = SimulationModule(world=world, evidence_dir=tmp_path / "evidence")
    try:
        module.build()
        description = module.describe()
        record_path = Path(description["run_record"])
        record = json.loads(record_path.read_text())
        assert (
            record["compiled_model"]["sha256"]
            == file_identity(Path(description["model"]))["sha256"]
        )
        assert record["resolved_world"]["robots"]["left"]["controller"]["revision"] == "abc123"
        assert record["resolved_world"]["calibration_evidence"] == world.calibration_evidence
        assert record["resolved_world"]["sampled_variation"]["left.xyz"]["seed"] == 123
        assert record["initial_state"]["values"]
        assert module.describe_run() == record
    finally:
        module.stop()
    assert record_path.is_file()
    assert list(record_path.parent.iterdir()) == [record_path]
    assert not Path(description["model"]).exists()


def test_cli_run_uses_existing_log_directory_and_makes_no_calibration_claim(
    world_config, tmp_path, monkeypatch
):
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path))
    module = SimulationModule(world=world_config)
    try:
        module.build()
        path = Path(module.describe()["run_record"])
        assert path.parent == tmp_path / "sim2"
        assert module.describe_run()["resolved_world"]["calibration_evidence"] == {}
        assert module.describe_run()["resolved_world"]["sampled_variation"] == {}
    finally:
        module.stop()


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


def test_calibrated_actuator_lag_and_immediate_disable(worlds, clients, world_config):
    instance = world_config.robots["left"]
    joint, gripper = instance.config.joints
    robot = replace(instance.config, joints=(replace(joint, actuator_time_constant=0.02), gripper))
    world = worlds(config=replace(world_config, robots={"left": replace(instance, config=robot)}))
    client = clients(world, writable=True)
    client.write_motors(
        MotorCommandArray(q=[0.1, 850], kp=[0, 0], kd=[0, 0], tau=[3, 0]),
        observed=client.read(),
    )
    world.step()
    aid = world.robots["left"].actuators[0]
    assert world.data.ctrl[aid] == pytest.approx(0.6)
    world.step()
    assert world.data.ctrl[aid] == pytest.approx(1.08)
    client.disable(observed=client.read())
    world.step()
    assert world.data.ctrl[aid] == 0.0


def test_startup_wait_keeps_feedback_live_then_releases_once(worlds, clients, world_config):
    instance = world_config.robots["left"]
    instance = replace(instance, wait_for_first_command=True)
    world = worlds(config=replace(world_config, robots={"left": instance}))
    client = clients(world, writable=True)
    before = client.read()
    for _ in range(20):
        world.step()
    current = client.read()
    assert world.tick == 0
    assert current.metadata.sequence > before.metadata.sequence
    np.testing.assert_array_equal(current.values["position"], before.values["position"])
    client.write_motors(MotorCommandArray(q=[0.1, 850], kp=[10, 0], kd=[1, 0]), observed=current)
    world.step()
    assert world.tick == 1
    assert not world.waiting_for_control
    client.disable(observed=client.read())
    world.step()
    assert world.tick == 2
    assert not world.robots["left"].enabled


def test_expired_motor_command_stops_using_feedforward(worlds, clients, mocker):
    world = worlds()
    client = clients(world, writable=True)
    client.write_motors(
        MotorCommandArray(q=[0.1, 850], kp=[0, 0], kd=[0, 0], tau=[3, 0]),
        observed=client.read(),
    )
    world.step()
    binding = world.robots["left"]
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(3)
    velocity = world.data.qvel[binding.dofs[0]]
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)

    world.step()

    assert binding.applied_action_sequence == 0
    assert world.data.ctrl[binding.actuators[0]] == pytest.approx(-velocity)


def test_post_step_snapshot_and_derived_robot_pose_are_the_same_tick(worlds, clients, world_config):
    instance = world_config.robots["left"]
    robot = replace(instance.config, floating=True)
    robot.model.write_text(
        robot.model.read_text().replace('<body name="base">', '<body name="base"><freejoint/>')
    )
    world = worlds(config=replace(world_config, robots={"left": replace(instance, config=robot)}))
    client = clients(world)
    root_joint = world.model.body("left/base").jntadr[0]
    qpos = world.model.jnt_qposadr[root_joint]
    dof = world.model.jnt_dofadr[root_joint]
    world.data.qvel[dof] = 1.0
    world.step()
    sample = client.read()
    assert world.data.qpos[qpos] > 0
    assert sample.values["root_position"][0] == pytest.approx(world.data.qpos[qpos])
    assert sample.metadata.sim_time == world.data.time


@pytest.mark.parametrize("disabled", [False, True])
def test_scene_edit_preserves_robot_pose_and_motor_enable(worlds, clients, disabled):
    world = worlds()
    client = clients(world, writable=True)
    for _ in range(200):
        client.write_motors(
            MotorCommandArray(q=[0.4, 850], kp=[10, 0], kd=[1, 0]),
            observed=client.read(),
        )
        world.step()
    position = client.read().values["position"].copy()
    assert position[0] == pytest.approx(0.4, abs=0.03)
    if disabled:
        client.disable(observed=client.read())
        world.step()
        position = client.read().values["position"].copy()

    state = world.set_scene_state(SceneUpdate(poses={"box": Pose(2, 0, 0.3)}))
    assert client.read().values["position"] == pytest.approx(position)
    world.step()

    assert state.entities["box"].pose.position.to_tuple() == pytest.approx((2, 0, 0.3))
    assert client.read().values["position"][0] == pytest.approx(position[0], abs=0.002)
    assert world.robots["left"].idle_position[0] == pytest.approx(position[0])
    assert world.robots["left"].enabled is not disabled
    assert world.robots["left"].applied_action_sequence == 0


def test_disabled_effort_outputs_do_not_reenable_when_command_expires(worlds, clients, mocker):
    world = worlds()
    client = clients(world, writable=True)
    client.disable(observed=client.read())
    world.step()
    mocker.patch("dimos.sim2.world.runtime.time.monotonic", return_value=time.monotonic() + 1)

    world.step()

    assert not world.robots["left"].enabled
    assert world.data.ctrl[world.robots["left"].actuators[0]] == 0


def test_scene_update_is_atomic_and_reset_restores_authored_state(worlds):
    world = worlds()
    before = world.scene_state()
    with pytest.raises(ValueError, match="outside joint limits"):
        world.set_scene_state(SceneUpdate(poses={"box": Pose(2, 0, 0.3)}, joints={"door": 5}))
    assert world.scene_state().entities["box"] == before.entities["box"]
    assert world.scene_state().episode == before.episode
    assert world.scene_state().generation == before.generation

    changed = world.set_scene_state(
        SceneUpdate(poses={"box": Pose(2, 0, 0.3)}, joints={"door": 0.5})
    )
    assert changed.joints["door"] == pytest.approx(0.5)
    assert changed.episode == before.episode
    assert changed.generation == before.generation + 1
    reset = world.reset()
    assert reset.entities["box"] == before.entities["box"]
    assert reset.joints["door"] == 0
    assert reset.generation == changed.generation + 1
    assert reset.episode == before.episode + 1


def test_edit_and_pause_resume_keep_the_task_episode(worlds):
    world = worlds()
    world.step()
    before = world.scene_state()

    edited = world.set_scene_state(SceneUpdate(poses={"box": Pose(2, 0, 0.3)}))
    world.set_paused(True)
    world.set_paused(False)
    resumed = world.scene_state()

    assert edited.episode == resumed.episode == before.episode
    assert edited.sim_time == resumed.sim_time == before.sim_time
    assert resumed.generation == before.generation + 2
    assert world.snapshots.read_observation().metadata.episode_id == resumed.generation
    reset = world.reset()
    assert reset.episode == before.episode + 1
    assert reset.generation == resumed.generation + 1
    assert reset.sim_time == 0


def test_reset_publishes_snapshot_before_next_step(worlds):
    world = worlds()
    world.step()
    previous = world.snapshots.read_observation()

    world.reset()

    current = world.snapshots.read_observation()
    assert current.metadata.episode_id == previous.metadata.episode_id + 1
    assert current.metadata.sequence > previous.metadata.sequence
    assert current.metadata.sim_time == 0


def test_pause_does_not_advance_physics(worlds):
    world = worlds()
    before = world.data.qpos.copy()
    world.set_paused(True)
    world.step()
    np.testing.assert_array_equal(world.data.qpos, before)
    assert world.tick == 0
    world.set_paused(False)
    world.step()
    assert world.tick == 1


def test_robot_cannot_replace_world_snapshot_channel(worlds):
    config = worlds().config
    with pytest.raises(ValueError, match="'world' is reserved"):
        replace(config, robots={"world": config.robots["left"]})


@pytest.mark.parametrize("disabled", [False, True])
def test_resume_preserves_pose_and_motor_enable_but_invalidates_old_work(
    worlds, clients, mocker, disabled
):
    world = worlds()
    client = clients(world, writable=True)
    if disabled:
        client.disable(observed=client.read())
    else:
        client.write_motors(
            MotorCommandArray(q=[0.4, 850], kp=[10, 0], kd=[1, 0]), observed=client.read()
        )
    world.step()
    old_frame = client.read()
    pose = world.data.qpos.copy()
    sim_time = float(world.data.time)
    previous = world.snapshots.read_observation()
    world.set_paused(True)
    mocker.patch(
        "dimos.sim2.world.runtime.time.time", return_value=previous.values["wall_time"][0] + 10
    )

    world.set_paused(False)

    current = world.snapshots.read_observation()
    assert current.metadata.episode_id == previous.metadata.episode_id + 1
    assert current.metadata.sim_time == sim_time
    assert current.values["wall_time"][0] == previous.values["wall_time"][0] + 10
    np.testing.assert_array_equal(world.data.qpos, pose)
    assert world.robots["left"].enabled is not disabled
    assert not world.robots["left"].command_active
    with pytest.raises(RuntimeError, match="reset"):
        client.disable(observed=old_frame)
    world.set_paused(False)
    assert world.generation == current.metadata.episode_id
    world.step()
    assert world.robots["left"].applied_action_sequence == 0
    assert world.robots["left"].enabled is not disabled


def test_pause_cannot_restore_faulted_world(worlds):
    world = worlds()
    world.snapshots.set_lifecycle("faulted")
    with pytest.raises(RuntimeError, match="faulted"):
        world.set_paused(False)
    assert world.snapshots.lifecycle == "faulted"
