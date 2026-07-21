# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.core.global_config import global_config
from dimos.robot.manipulators.xarm.config import xarm6_hardware, xarm7_hardware


def test_xarm_mock_factories_configure_gripper_endpoints(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "xarm6_ip", "")
    monkeypatch.setattr(global_config, "xarm7_ip", "")

    for hardware in (
        xarm6_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
        xarm7_hardware(
            gripper=True,
            gripper_open_position=0.85,
            gripper_closed_position=0.0,
            mock_without_address=True,
        ),
    ):
        assert (hardware.gripper_closed_position, hardware.gripper_open_position) == (0.0, 0.85)
