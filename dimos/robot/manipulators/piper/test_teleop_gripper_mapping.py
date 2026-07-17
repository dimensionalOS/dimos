# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.control.coordinator import ControlCoordinator
from dimos.robot.manipulators.piper.blueprints.teleop import coordinator_teleop_piper


def test_piper_quest_gripper_maps_neutral_open_and_full_closed() -> None:
    coordinator_kwargs = next(
        atom.kwargs
        for atom in coordinator_teleop_piper.blueprints
        if atom.module is ControlCoordinator
    )
    task = coordinator_kwargs["tasks"][0]

    assert task.params["gripper_open_pos"] == 0.035
    assert task.params["gripper_closed_pos"] == 0.0
