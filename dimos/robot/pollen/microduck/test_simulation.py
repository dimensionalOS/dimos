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

import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.microduck_policy_task.microduck_policy_task import (
    MicroDuckPolicyTask,
    MicroDuckPolicyTaskConfig,
)
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.pollen.microduck.config import (
    MICRODUCK_HOME,
    MICRODUCK_JOINT_SUFFIXES,
    MICRODUCK_JOINTS,
    MICRODUCK_SIM_SPEC,
)
from dimos.simulation.engines.robot_sim_binding import (
    RobotSimBinding,
    resolve_robot_sim_binding,
)
from dimos.simulation.utils.xml_parser import build_joint_mappings
from dimos.utils.data import get_data


@pytest.mark.mujoco
def test_official_scene_has_exact_policy_binding_and_physics_step() -> None:
    scene = Path(get_data("microduck/scene.xml"))
    model = mujoco.MjModel.from_xml_path(str(scene))

    binding = resolve_robot_sim_binding(
        model,
        MICRODUCK_SIM_SPEC,
        build_joint_mappings(scene, model),
    )

    assert model.opt.timestep == pytest.approx(0.005)
    assert binding.root_qpos_adr == 0
    assert binding.imu_quat_slice is not None
    assert binding.imu_gyro_slice is not None
    assert binding.imu_accel_slice is not None
    assert (
        tuple(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            for joint_id in binding.joint_ids
        )
        == MICRODUCK_JOINT_SUFFIXES
    )
    assert (
        tuple(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            for actuator_id in binding.actuator_ids
        )
        == MICRODUCK_JOINT_SUFFIXES
    )


def _policy_state(t_now: float = 1.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(zip(MICRODUCK_JOINTS, MICRODUCK_HOME, strict=True)),
            joint_velocities={name: 0.0 for name in MICRODUCK_JOINTS},
            joint_efforts={name: 0.0 for name in MICRODUCK_JOINTS},
        ),
        imu={
            "microduck": IMUState(
                quaternion=(1.0, 0.0, 0.0, 0.0),
                gyroscope=(0.0, 0.0, 0.0),
            )
        },
        t_now=t_now,
        dt=0.02,
    )


def _mujoco_state(data: mujoco.MjData, binding: RobotSimBinding, t_now: float) -> CoordinatorState:
    assert binding.imu_quat_slice is not None
    assert binding.imu_gyro_slice is not None
    quaternion = data.sensordata[binding.imu_quat_slice]
    gyroscope = data.sensordata[binding.imu_gyro_slice]
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={
                name: float(data.qpos[address])
                for name, address in zip(MICRODUCK_JOINTS, binding.joint_qpos_adrs, strict=True)
            },
            joint_velocities={
                name: float(data.qvel[address])
                for name, address in zip(MICRODUCK_JOINTS, binding.joint_qvel_adrs, strict=True)
            },
            joint_efforts={name: 0.0 for name in MICRODUCK_JOINTS},
        ),
        imu={
            "microduck": IMUState(
                quaternion=tuple(float(value) for value in quaternion),
                gyroscope=tuple(float(value) for value in gyroscope),
            )
        },
        t_now=t_now,
        dt=0.02,
    )


@pytest.mark.mujoco
def test_bundled_policy_set_runs_every_public_motion_with_finite_targets() -> None:
    task = MicroDuckPolicyTask(
        "microduck_policy",
        MicroDuckPolicyTaskConfig(
            policy_dir=Path(get_data("microduck/policies")),
            joint_names=list(MICRODUCK_JOINTS),
        ),
    )
    task.start()
    state = _policy_state()

    stand = task.compute(state)
    assert stand is not None
    assert stand.positions is not None
    assert np.all(np.isfinite(stand.positions))
    assert task.get_status()["current_policy"] == "stand"

    twist = Twist()
    twist.linear.x = 0.4
    assert task.on_twist_command(twist, state.t_now)
    walk = task.compute(state)
    assert walk is not None
    assert walk.positions is not None
    assert np.all(np.isfinite(walk.positions))
    assert task.get_status()["current_policy"] == "walk"

    for skill in ("ground_pick", "kick_left", "kick_right", "roulade"):
        assert task.reset_runtime_state(reactivate=True)
        assert task.run_skill(skill)["accepted"] is True
        output = task.compute(state)
        assert output is not None
        assert output.positions is not None
        assert np.all(np.isfinite(output.positions))
        assert task.get_status()["current_policy"] == skill


@pytest.mark.mujoco
def test_headless_closed_loop_is_finite_for_sixty_simulated_seconds() -> None:
    scene = Path(get_data("microduck/scene.xml"))
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    binding = resolve_robot_sim_binding(
        model,
        MICRODUCK_SIM_SPEC,
        build_joint_mappings(scene, model),
    )
    assert binding.root_qpos_adr is not None
    for address, position in zip(binding.joint_qpos_adrs, MICRODUCK_HOME, strict=True):
        data.qpos[address] = position
    data.qpos[binding.root_qpos_adr + 2] = 0.125
    mujoco.mj_forward(model, data)

    task = MicroDuckPolicyTask(
        "microduck_policy",
        MicroDuckPolicyTaskConfig(
            policy_dir=Path(get_data("microduck/policies")),
            joint_names=list(MICRODUCK_JOINTS),
        ),
    )
    task.start()
    twist = Twist()
    twist.linear.x = 0.2
    actuator_ids = np.asarray(binding.actuator_ids, dtype=np.int32)
    root_xy_at_walk_start: np.ndarray[Any, np.dtype[np.float64]] | None = None
    maximum_walk_displacement = 0.0
    minimum_root_z = math.inf
    seen_policies: set[str] = set()

    policy_dt = 1.0 / 50.0
    physics_steps_per_policy_tick = round(policy_dt / model.opt.timestep)
    assert physics_steps_per_policy_tick == 4
    for tick in range(round(60.0 / policy_dt)):
        t_now = tick * policy_dt
        if 10.0 <= t_now < 30.0 and tick % 5 == 0:
            assert task.on_twist_command(twist, t_now)

        output = task.compute(_mujoco_state(data, binding, t_now))
        assert output is not None
        assert output.positions is not None
        assert np.all(np.isfinite(output.positions))
        data.ctrl[actuator_ids] = output.positions
        for _ in range(physics_steps_per_policy_tick):
            mujoco.mj_step(model, data)

        assert np.all(np.isfinite(data.qpos))
        assert np.all(np.isfinite(data.qvel))
        root_xy = data.qpos[binding.root_qpos_adr : binding.root_qpos_adr + 2]
        root_z = float(data.qpos[binding.root_qpos_adr + 2])
        minimum_root_z = min(minimum_root_z, root_z)
        status = task.get_status()
        policy = status["current_policy"]
        assert isinstance(policy, str)
        seen_policies.add(policy)
        if 10.0 <= t_now < 30.0:
            if root_xy_at_walk_start is None:
                root_xy_at_walk_start = root_xy.copy()
            maximum_walk_displacement = max(
                maximum_walk_displacement,
                float(np.linalg.norm(root_xy - root_xy_at_walk_start)),
            )

    assert seen_policies >= {"stand", "walk"}
    assert task.get_status()["current_policy"] == "stand"
    assert task.get_status()["command_age_s"] is not None
    assert task.get_status()["command_age_s"] > 0.5
    assert minimum_root_z > 0.08
    assert maximum_walk_displacement > 1e-4
