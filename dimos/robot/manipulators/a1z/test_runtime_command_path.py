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

"""Headless integration coverage for the A1Z teleoperation command path."""

from __future__ import annotations

from collections.abc import Callable
import time

import pytest
from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tick_loop import TickLoop
from dimos.robot.manipulators.a1z.blueprints.teleop import _build_a1z_keyboard_components
from dimos.robot.manipulators.a1z.config import A1Z_DOF, A1Z_FK_MODEL, make_a1z_hardware
from dimos.robot.manipulators.a1z.simulation import A1Z_SCENE_PATH, _A1ZMujocoSimModule
from dimos.robot.manipulators.common.blueprints import eef_twist_task
from dimos.simulation.engines.mujoco_shm import ManipShmWriter, shm_key_from_path
from dimos.teleop.keyboard.keyboard_teleop_module import (
    GRIPPER_CLOSED_POSITION,
    GRIPPER_OPEN_POSITION,
    KeyboardTeleopModule,
)


def test_a1z_mujoco_teleop_blueprint_contains_the_runtime_command_path() -> None:
    components = _build_a1z_keyboard_components("mujoco")
    module_types = {atom.module for component in components for atom in component.blueprints}

    assert KeyboardTeleopModule in module_types
    assert _A1ZMujocoSimModule in module_types
    assert ControlCoordinator in module_types


@pytest.mark.timeout(5)
def test_a1z_keyboard_commands_reach_mujoco_shm_and_twist_task() -> None:
    """Exercise the real in-process seams without starting a viewer or daemon.

    The MuJoCo module owns SHM in a separate worker in production.  This test
    owns the same SHM writer in-process, while using the real adapter,
    coordinator, servo task, and keyboard output.
    """
    hardware = make_a1z_hardware(
        "arm",
        adapter_type="sim_mujoco",
        address=str(A1Z_SCENE_PATH),
        home_joints=list((0.0, 0.7, -1.2, 0.5, 0.0, 0.0)),
    )
    tasks = [
        eef_twist_task(hardware, model_path=A1Z_FK_MODEL, ee_joint_id=A1Z_DOF),
        # This is the sim-only task added by the A1Z teleop blueprint.
        TaskConfig(
            name="servo_gripper",
            type="servo",
            joint_names=["arm/gripper"],
            priority=20,
            params={"timeout": 0.0, "default_positions": [0.0]},
        ),
    ]
    coordinator = ControlCoordinator(
        tick_rate=200.0,
        publish_joint_state=False,
        hardware=[hardware],
        tasks=tasks,
    )
    keyboard = KeyboardTeleopModule()
    writer = ManipShmWriter(shm_key_from_path(A1Z_SCENE_PATH))
    subscriptions: list[Disposable] = []

    try:
        writer.signal_ready(A1Z_DOF + 1)
        coordinator._setup_from_config()
        subscriptions.append(
            Disposable(keyboard.joint_command.subscribe(coordinator._on_joint_command))
        )
        subscriptions.append(
            Disposable(
                keyboard.coordinator_ee_twist_command.subscribe(coordinator._on_ee_twist_command)
            )
        )

        coordinator._tick_loop = TickLoop(
            tick_rate=200.0,
            hardware=coordinator._hardware,
            hardware_lock=coordinator._hardware_lock,
            tasks=coordinator._tasks,
            task_lock=coordinator._task_lock,
            joint_to_hardware=coordinator._joint_to_hardware,
        )

        keyboard._set_gripper_position(GRIPPER_OPEN_POSITION)
        coordinator._tick_loop._tick()
        assert _wait_for(lambda: writer.read_gripper_command() == GRIPPER_OPEN_POSITION)
        keyboard._set_gripper_position(GRIPPER_CLOSED_POSITION)
        coordinator._tick_loop._tick()
        assert _wait_for(lambda: writer.read_gripper_command() == GRIPPER_CLOSED_POSITION)

        keyboard._publish_twist(tasks[0].name, linear=(1.0, -2.0, 0.0), angular=(0.0, 0.0, 0.5))
        eef_task = coordinator.get_task(tasks[0].name)
        assert eef_task is not None
        assert eef_task.is_active()
    finally:
        for subscription in subscriptions:
            subscription.dispose()
        coordinator.stop()
        keyboard.stop()
        writer.cleanup()


def _wait_for(check: Callable[[], bool], timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return True
        time.sleep(0.005)
    return check()
