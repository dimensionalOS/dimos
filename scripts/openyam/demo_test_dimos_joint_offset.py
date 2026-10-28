"""Offline checks for the DimOS joint-offset hardware runner."""

import argparse
import io
import json
import math

import demo_dimos_joint_offset as offset
from demo_dimos_joint_offset import (
    RawMonitor,
    Runner,
    check_path,
    load_workspace_checker,
)
import numpy as np
import pytest


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


def test_preflight_reads_enabled_motors_without_connecting_an_adapter(mocker) -> None:
    monitor = mocker.Mock()
    monitor.snapshot.return_value = {
        side: {joint: {"status": 1} for joint in range(1, 7)} for side in ("left", "right")
    }
    checker = mocker.Mock()
    checker.check.return_value = (True, {})
    mocker.patch("demo_dimos_joint_offset.raw_vectors", return_value=(np.zeros(12), 28))
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
    args = argparse.Namespace(left_can="can1", right_can="can0")

    pose, enabled = Runner(args, checker, monitor, io.StringIO()).preflight()

    assert pose.tolist() == [0.0] * 12
    assert enabled is True
    monitor.refresh.assert_called_once_with()
    adapter.assert_not_called()


@pytest.mark.parametrize(
    ("already_enabled", "enable_calls", "adopt_calls"),
    [(False, 1, 0), (True, 0, 1)],
)
def test_connect_handles_disabled_and_enabled_arms(
    mocker, already_enabled, enable_calls, adopt_calls
) -> None:
    args = argparse.Namespace(left_can="can1", right_can="can0")
    monitor = mocker.Mock()
    checker = mocker.Mock()
    checker.check.return_value = (True, {})
    adapter = mocker.patch("demo_dimos_joint_offset.ArmOnlyDimOSAdapter").return_value
    adapter.connect.return_value = True
    mocker.patch("demo_dimos_joint_offset.raw_vectors", return_value=(np.zeros(12), 28))
    runner = Runner(args, checker, monitor, io.StringIO())
    mocker.patch.object(runner, "sample")

    runner.connect("baseline", already_enabled=already_enabled)

    assert adapter.activate.call_count == enable_calls
    assert adapter.adopt_enabled_motors.call_count == adopt_calls
    runner.sample.assert_called_once()


def test_main_positions_at_supplied_pose_before_offset_trial(mocker, tmp_path) -> None:
    target_deg = [0.0] * 12
    target_deg[8] = 30.0
    output_dir = tmp_path / "offset"
    args = argparse.Namespace(
        delta_deg=3.0,
        target_deg=target_deg,
        mode="baseline",
        side="right",
        joint=3,
        left_can="can1",
        right_can="can0",
        hold_seconds=None,
        output_dir=output_dir,
        execute=True,
    )
    mocker.patch.object(offset, "arguments", return_value=args)
    mocker.patch.object(offset, "load_workspace_checker")
    mocker.patch.object(offset, "RawMonitor")
    checked_paths = mocker.patch.object(offset, "check_path")
    mocker.patch.object(offset, "wait_disabled", return_value=np.zeros(12))
    mocker.patch.object(offset.signal, "signal")
    runner = mocker.patch.object(offset, "Runner").return_value
    runner.preflight.side_effect = [(np.zeros(12), False), (np.zeros(12), False)]
    runner.reference = None
    runner.trial.return_value = {
        "mode": "baseline",
        "side": "right",
        "joint": 3,
        "direction": 1,
    }

    result = offset.main()

    assert result == 0
    expected_pose = np.radians(target_deg)
    assert np.allclose(checked_paths.call_args_list[1].args[1], expected_pose)
    assert checked_paths.call_args_list[1].args[2][8] == pytest.approx(math.radians(33))
    assert np.allclose(runner.move.call_args_list[0].args[0], expected_pose)
    assert runner.trial.call_args.args[:2] == ("right", 3)
    assert np.allclose(runner.move.call_args_list[1].args[0], np.zeros(12))
    assert json.loads((output_dir / "manifest.json").read_text())["target_deg"] == target_deg
