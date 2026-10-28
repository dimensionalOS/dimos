"""Offline checks for the DimOS joint-offset hardware runner."""

import math

from demo_dimos_joint_offset import (
    RawMonitor,
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


