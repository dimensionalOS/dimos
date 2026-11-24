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

"""Run the real xArm7 coordinator and Pink IK against headless MuJoCo."""

from contextlib import ExitStack
from dataclasses import dataclass, replace
from pathlib import Path
from queue import Queue
import shutil

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.xarm.blueprints.policy_teleop_sim import (
    demo_policy_teleop_human_sim,
    demo_policy_teleop_policy_sim,
    policy_teleop_sim,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.policy.adapters import TcpPoseAdapter
from dimos.teleop.policy.demo_sources import SyntheticVelocitySources
from dimos.teleop.policy.module import PolicyTeleopModule
from dimos.utils.testing.waiting import wait_until


@pytest.mark.parametrize(
    "blueprint", [policy_teleop_sim, demo_policy_teleop_human_sim, demo_policy_teleop_policy_sim]
)
def test_simulation_has_one_motion_task_and_both_sources_go_through_blender(blueprint):
    # Robot model materialization belongs to the self-hosted simulation tests.
    config_only = blueprint.disabled_modules(ArmTwistCoordinator)
    parsed = BlueprintConfigParser(config_only).parse(
        ["--mujocosimmodule.headless=true"], environ={}
    )
    assert parsed.global_config["simulation"] == "mujoco"
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    coordinator = atoms[ArmTwistCoordinator]
    assert [task.type for task in coordinator.kwargs["tasks"]] == ["eef_twist"]
    assert coordinator.kwargs["publish_frame_poses"]
    assert coordinator.kwargs["tasks"][0].params["timeout"] > 0
    assert set(atoms) == {
        ArmTwistCoordinator,
        MujocoSimModule,
        TcpPoseAdapter,
        PolicyTeleopModule,
        SyntheticVelocitySources,
    }


@dataclass
class _Simulation:
    coordinator: ArmTwistCoordinator
    blender: PolicyTeleopModule
    source: SyntheticVelocitySources
    poses: Queue[Transform]
    commands: Queue[JointState]


@pytest.fixture
def simulation(modules, tmp_path, mocker):
    # Writer and reader share a process here. Keep its ownership registered so
    # the reader cannot unregister the writer from their shared resource tracker.
    mocker.patch("dimos.utils.shm.unregister", side_effect=lambda shm: shm)
    atoms = {atom.module: atom for atom in policy_teleop_sim.active_blueprints}
    sim_kwargs = dict(atoms[MujocoSimModule].kwargs)
    # Unique scene path -> unique shared-memory key; never attach to a live demo.
    scene = Path(sim_kwargs["address"])
    copied = tmp_path / "scene"
    shutil.copytree(scene.parent, copied)
    scene = copied / scene.name
    sim_kwargs.update(address=str(scene), headless=True)
    sim = modules(MujocoSimModule, **sim_kwargs)
    coord_kwargs = dict(atoms[ArmTwistCoordinator].kwargs)
    coord_kwargs["hardware"] = [replace(hw, address=str(scene)) for hw in coord_kwargs["hardware"]]
    coordinator = modules(ArmTwistCoordinator, **coord_kwargs)
    adapter = modules(TcpPoseAdapter)
    blender = modules(**atoms[PolicyTeleopModule].kwargs)
    source = modules(SyntheticVelocitySources)
    poses, commands = Queue(), Queue()
    with ExitStack() as stack:
        for output, callback in (
            (coordinator.tf, adapter.tf.transport.publish),
            (adapter.current_pose, blender.current_pose.transport.publish),
            (adapter.current_pose, source.current_pose.transport.publish),
            (adapter.current_pose, poses.put),
            (source.teleop_command, blender.teleop_command.transport.publish),
            (source.policy_command, blender.policy_command.transport.publish),
            (blender.ee_twist_command, coordinator.ee_twist_command.transport.publish),
            (coordinator.coordinator_joint_command, commands.put),
        ):
            stack.callback(output.subscribe(callback))
        sim.start()
        adapter.start()
        blender.start()
        coordinator.start()
        yield _Simulation(coordinator, blender, source, poses, commands)


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    "blueprint", [policy_teleop_sim, demo_policy_teleop_human_sim, demo_policy_teleop_policy_sim]
)
def test_real_ik_moves_in_simulation_and_blender_shutdown_stops_it(simulation, blueprint):
    sim = simulation
    atoms = {atom.module: atom for atom in blueprint.active_blueprints}
    sim.blender.config.takeover_fn = atoms[PolicyTeleopModule].kwargs["takeover_fn"]
    initial = sim.poses.get(timeout=10)
    sim.source.start()

    # Wait for actual FK feedback showing a lift and world-Z rotation.
    def moved():
        pose = sim.poses.get(timeout=2)
        rotation = (
            Rotation.from_quat(pose.rotation.to_numpy())
            * Rotation.from_quat(initial.rotation.to_numpy()).inv()
        )
        return (
            pose.translation.z - initial.translation.z > 0.001 and rotation.as_rotvec()[2] > 0.005
        )

    wait_until(moved, timeout=5, interval=0.01)
    assert sim.blender.get_fault() is None
    assert sim.commands.qsize() > 0
    sim.blender.stop()
    wait_until(lambda: not sim.coordinator.get_active_tasks(), timeout=2, interval=0.01)
    assert sim.coordinator.list_tasks() == [EEF_TWIST_TASK_NAME]
    before = sim.coordinator.get_joint_positions()
    tick = sim.coordinator.get_tick_count()
    wait_until(lambda: sim.coordinator.get_tick_count() >= tick + 30, timeout=2, interval=0.01)
    after = sim.coordinator.get_joint_positions()
    np.testing.assert_allclose(list(after.values()), list(before.values()), atol=0.01)


@pytest.mark.self_hosted
def test_coordinator_estop_still_blocks_all_blended_commands(simulation):
    sim = simulation
    sim.source.start()
    wait_until(lambda: bool(sim.coordinator.get_active_tasks()), timeout=5, interval=0.01)
    assert sim.coordinator.set_estop(True)
    count = sim.commands.qsize()
    tick = sim.coordinator.get_tick_count()
    wait_until(lambda: sim.coordinator.get_tick_count() >= tick + 30, timeout=2, interval=0.01)
    assert sim.commands.qsize() == count
    assert not sim.coordinator.get_active_tasks()
    assert sim.coordinator.set_estop(False)
    wait_until(lambda: sim.commands.qsize() > count, timeout=3, interval=0.01)
