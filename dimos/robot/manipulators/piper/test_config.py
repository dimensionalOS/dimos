# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.core.global_config import global_config
from dimos.robot.manipulators.piper.config import (
    PIPER_HOME_JOINTS,
    PIPER_SIM_PATH,
    make_piper_model_config,
    piper_hardware,
)


def test_piper_model_config_exposes_default_home_preset() -> None:
    config = make_piper_model_config()

    assert config.preset_poses["home"] == config.home_joints == PIPER_HOME_JOINTS
    assert len(config.preset_poses["home"]) == 6


def test_piper_model_config_exposes_supplied_home_preset() -> None:
    home_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    config = make_piper_model_config(home_joints=home_joints)

    assert config.preset_poses["home"] == home_joints
    assert config.home_joints == home_joints
    assert len(config.preset_poses["home"]) == 6
    assert config.preset_poses["home"] is not home_joints


def test_piper_defaults_to_mock_without_can_port(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")

    for can_port in (None, ""):
        monkeypatch.setattr(global_config, "can_port", can_port)
        hardware = piper_hardware()

        assert hardware.adapter_type == "mock"
        assert hardware.address is None


def test_piper_uses_configured_can_port(monkeypatch) -> None:
    can_port = "can7"
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", can_port)

    hardware = piper_hardware()

    assert hardware.adapter_type == "piper"
    assert hardware.address == can_port


def test_piper_simulation_selection_is_unchanged(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "can_port", "can7")

    hardware = piper_hardware()

    assert hardware.adapter_type == "sim_mujoco"
    assert hardware.address == str(PIPER_SIM_PATH)
