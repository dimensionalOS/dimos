"""Offline checks for the DimOS joint-offset hardware runner."""

import argparse
import io
import math

from demo_dimos_joint_offset import (
    ArmOnlyDimOSAdapter,
    RawMonitor,
    Runner,
    check_path,
    load_workspace_checker,
)
import numpy as np
import pytest

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig


def test_raw_decoder_keeps_position_bytes_that_resemble_parameter_opcodes() -> None:
    for low_byte in (0x33, 0x55):
        state = RawMonitor._decode(0x11, bytes([0x01, 0x80, low_byte, 0x7F, 0xF7, 0xFF, 28, 27]))
        assert state is not None
        assert state["joint"] == 1
        assert state["temperature_mos"] == 28
        assert state["q"] == pytest.approx(((0x80 << 8) | low_byte) * 25 / 65535 - 12.5)
    assert RawMonitor._decode(0x11, bytes([1, 0, 0x33, 21, 0, 0, 0, 0])) is None


def test_near_zero_test_path_is_checked_against_workspace() -> None:
    checker = load_workspace_checker()
    start = np.zeros(12)
    for joint_index in range(12):
        target = start.copy()
        target[joint_index] = math.radians(2)
        check_path(checker, start, target)
    bad = start.copy()
    bad[0] = 100.0
    with pytest.raises(ValueError, match="Workspace check rejected"):
        check_path(checker, start, bad)


def test_preflight_accepts_enabled_motors_without_disabling_them(mocker) -> None:
    args = argparse.Namespace(left_can="can1", right_can="can0")
    monitor = mocker.Mock()
    monitor.error = None
    monitor.snapshot.return_value = {
        side: {
            joint: {
                "q": 0.0,
                "status": 1,
                "temperature_mos": 28,
                "temperature_rotor": 27,
            }
            for joint in range(1, 7)
        }
        for side in ("left", "right")
    }
    checker = mocker.Mock()
    checker.check.return_value = (True, {})
    registers = {21: 12.5, 22: 10.0, 23: 28.0, 10: 1}
    mocker.patch(
        "demo_dimos_joint_offset.read_register",
        side_effect=lambda _port, joint, register, *, integer: (
            (30.0 if register == 22 else 10.0)
            if joint > 3 and register in (22, 23)
            else registers[register]
        ),
    )
    adapter = mocker.patch("demo_dimos_joint_offset.ArmOnlyDimOSAdapter")

    result = Runner(args, checker, monitor, io.StringIO()).preflight()

    assert result.tolist() == [0.0] * 12
    monitor.refresh.assert_called_once_with()
    adapter.assert_not_called()


def test_handoff_releases_script_adapter_without_disabling_motors(mocker) -> None:
    adapter = ArmOnlyDimOSAdapter(runtime_config=DamiaoRuntimeConfig())
    robot = mocker.Mock()
    adapter._robot = robot
    adapter._connected = True
    adapter._active = True

    adapter.disconnect_preserving_enable()

    robot.disable.assert_not_called()
    assert not adapter._connected


def test_enabled_handoff_adopts_motors_and_starts_holding_current_pose(mocker) -> None:
    args = argparse.Namespace(left_can="can1", right_can="can0")
    monitor = mocker.Mock()
    runner = Runner(args, mocker.Mock(), monitor, io.StringIO())
    runner.adapter = mocker.Mock()
    current = np.full(12, 0.2)
    mocker.patch("demo_dimos_joint_offset.raw_vectors", return_value=(current, 30))
    mocker.patch("demo_dimos_joint_offset.check_path")
    sample = mocker.patch.object(runner, "sample")

    runner.activate(already_enabled=True)

    runner.adapter.adopt_enabled_motors.assert_called_once_with()
    runner.adapter.activate.assert_not_called()
    assert runner.reference.tolist() == [0.2] * 12
    sample.assert_called_once()
    assert sample.call_args.args[1] == "handoff:hold"


def test_switching_to_candidate_keeps_the_adapter_active(mocker) -> None:
    args = argparse.Namespace(left_can="can1", right_can="can0")
    runner = Runner(args, mocker.Mock(), mocker.Mock(), io.StringIO())
    runner.adapter = mocker.Mock()
    runner.adapter._active = True

    runner.switch_mode("candidate")

    assert runner.adapter._runtime_config.tracking_comp
    assert runner.adapter._runtime_config.openyam_anvil_parity
    runner.adapter.deactivate.assert_not_called()
    runner.adapter.disconnect.assert_not_called()
