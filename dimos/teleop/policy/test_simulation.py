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

"""Exercise shared joint control with real Pink IK and headless MuJoCo."""

from contextlib import ExitStack
from dataclasses import dataclass, replace
from pathlib import Path
from queue import Queue
import shutil

import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.xarm.blueprints.policy_teleop_sim import (
    JointPolicyDemo,
    policy_teleop_sim,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule, pygame
from dimos.utils.testing.waiting import wait_until


@dataclass
class _Simulation:
    coordinator: ArmTwistCoordinator
    policy: JointPolicyDemo
    commands: Queue[JointState]
    poses: Queue[Transform]


@pytest.fixture
def simulation(modules, tmp_path, mocker):
    mocker.patch("dimos.utils.shm.unregister", side_effect=lambda shm: shm)
    atoms = {atom.module: atom for atom in policy_teleop_sim.active_blueprints}
    sim_kwargs = dict(atoms[MujocoSimModule].kwargs)
    scene = Path(sim_kwargs["address"])
    copied = tmp_path / "scene"
    shutil.copytree(scene.parent, copied)
    scene = copied / scene.name
    sim_kwargs.update(address=str(scene), headless=True)
    sim = modules(MujocoSimModule, **sim_kwargs)
    coord_kwargs = dict(atoms[ArmTwistCoordinator].kwargs)
    coord_kwargs["hardware"] = [replace(hw, address=str(scene)) for hw in coord_kwargs["hardware"]]
    coordinator = modules(ArmTwistCoordinator, **coord_kwargs)
    policy = modules(JointPolicyDemo, **atoms[JointPolicyDemo].kwargs)
    mocker.patch.object(policy, "_control", coordinator, create=True)
    commands = Queue()
    poses = Queue()

    def on_tf(message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id == "link_tcp":
                poses.put(transform)

    with ExitStack() as stack:
        stack.callback(
            coordinator.coordinator_joint_state.subscribe(
                policy.coordinator_joint_state.transport.publish
            )
        )
        stack.callback(coordinator.coordinator_joint_command.subscribe(commands.put))
        stack.callback(coordinator.tf.subscribe(on_tf))
        sim.start()
        coordinator.start()
        wait_until(lambda: bool(coordinator.get_joint_positions()), timeout=10, interval=0.01)
        yield _Simulation(coordinator, policy, commands, poses)


@pytest.mark.self_hosted
def test_operator_can_join_and_leave_a_running_policy_in_simulation(simulation):
    sim = simulation
    initial = sim.poses.get(timeout=5)
    sim.policy.start()
    wait_until(
        lambda: sim.coordinator.get_joint_positions()["joint1"] > 0.4,
        timeout=5,
        interval=0.01,
    )
    pose = sim.poses.get(timeout=2)
    # Drain feedback queued while waiting for the visibly larger policy sweep.
    while not sim.poses.empty():
        pose = sim.poses.get_nowait()
    assert pose.translation.y - initial.translation.y > 0.15
    intervention_start = pose.ts
    while pose.ts - intervention_start < 1.5:
        sim.coordinator.ee_twist_command.transport.publish(TwistStamped(linear=[0, 0, 0.12]))
        pose = sim.poses.get(timeout=2)
    # Holding Q changes height while the policy continues its lateral sweep.
    assert pose.translation.z - initial.translation.z > 0.03
    status = sim.coordinator.task_invoke("policy_teleop", "get_status")
    assert status["operator_present"]
    assert min(status["weights"].values()) == pytest.approx(0.5)
    assert sim.coordinator.task_invoke("joint_trajectory", "get_state") is TrajectoryState.EXECUTING
    sim.coordinator.ee_twist_command.transport.publish(TwistStamped())
    wait_until(
        lambda: not sim.coordinator.task_invoke("policy_teleop", "get_status")["operator_present"],
        timeout=2,
        interval=0.01,
    )
    wait_until(
        lambda: all(
            weight == 1
            for weight in sim.coordinator.task_invoke("policy_teleop", "get_status")[
                "weights"
            ].values()
        ),
        timeout=2,
        interval=0.01,
    )
    assert sim.coordinator.task_invoke("joint_trajectory", "get_state") is TrajectoryState.EXECUTING
    wait_until(
        lambda: abs(sim.poses.get(timeout=2).translation.z - initial.translation.z) < 0.02,
        timeout=3,
        interval=0.01,
    )
    wait_until(
        lambda: sim.coordinator.get_joint_positions()["joint1"] < -0.4,
        timeout=8,
        interval=0.01,
    )
    assert sim.commands.qsize() > 0


@pytest.mark.self_hosted
def test_keyboard_q_event_reaches_shared_control(simulation, modules, monkeypatch):
    # Exercise the actual pygame loop without opening a window on the desktop.
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    sim = simulation
    atoms = {atom.module: atom for atom in policy_teleop_sim.active_blueprints}
    keyboard = modules(KeyboardTeleopModule, **atoms[KeyboardTeleopModule].kwargs)
    initial = sim.poses.get(timeout=5)
    with ExitStack() as stack:
        stack.callback(
            keyboard.ee_twist_command.subscribe(sim.coordinator.ee_twist_command.transport.publish)
        )
        keyboard.start()
        wait_until(
            lambda: pygame.display.get_init() and pygame.display.get_surface() is not None,
            timeout=3,
            interval=0.01,
        )
        sim.policy.start()
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_q))
        wait_until(
            lambda: sim.poses.get(timeout=2).translation.z - initial.translation.z > 0.03,
            timeout=5,
            interval=0.01,
        )
        status = sim.coordinator.task_invoke("policy_teleop", "get_status")
        assert status["operator_present"]
        assert min(status["weights"].values()) == pytest.approx(0.5)

        pygame.event.post(pygame.event.Event(pygame.KEYUP, key=pygame.K_q))
        wait_until(
            lambda: not sim.coordinator.task_invoke("policy_teleop", "get_status")[
                "operator_present"
            ],
            timeout=2,
            interval=0.01,
        )
        assert (
            sim.coordinator.task_invoke("joint_trajectory", "get_state")
            is TrajectoryState.EXECUTING
        )


@pytest.mark.self_hosted
def test_estop_stops_policy_and_discards_cached_targets_in_simulation(simulation):
    sim = simulation
    sim.policy.start()
    wait_until(lambda: sim.commands.qsize() > 0, timeout=5, interval=0.01)
    assert sim.coordinator.set_estop(True)
    assert sim.coordinator.task_invoke("policy_teleop", "get_status")["policy_present"] == []
    count = sim.commands.qsize()
    tick = sim.coordinator.get_tick_count()
    wait_until(lambda: sim.coordinator.get_tick_count() >= tick + 10, timeout=2, interval=0.01)
    assert sim.commands.qsize() == count
    assert sim.coordinator.set_estop(False)
    tick = sim.coordinator.get_tick_count()
    wait_until(lambda: sim.coordinator.get_tick_count() >= tick + 10, timeout=2, interval=0.01)
    assert sim.commands.qsize() == count
