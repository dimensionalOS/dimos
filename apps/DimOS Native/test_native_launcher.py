# Copyright 2026 dimos contributors
# SPDX-License-Identifier: MIT

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SWIFT_SOURCE = (PROJECT_ROOT / "apps/DimOS Native/main.swift").read_text()
DOUBLE_CLICK_LAUNCHER = (PROJECT_ROOT / "scripts/start_dimos_go2.command").read_text()
CONFLICT_HELPER = (PROJECT_ROOT / "scripts/stop_dimos_native_conflicts.sh").read_text()


def test_hardware_mode_explicitly_enables_motion_without_auto_stand() -> None:
    assert '"go2connection.movement_enabled=true"' in SWIFT_SOURCE
    assert '"go2connection.auto_stand=false"' in SWIFT_SOURCE
    assert "只读连接（不能遥控）" in SWIFT_SOURCE  # noqa: RUF001
    assert 'button(\n            "连接并启用遥控",' in SWIFT_SOURCE
    assert "hardwareConfirmation" not in SWIFT_SOURCE
    assert "运动确认词" not in SWIFT_SOURCE


def test_native_app_owns_the_only_runtime() -> None:
    assert "stop_dimos_native_conflicts.sh" in SWIFT_SOURCE
    assert 'nohup "${DIMOS_BIN}"' not in DOUBLE_CLICK_LAUNCHER


def test_native_app_can_display_actual_odometry_from_the_active_mcp() -> None:
    assert "查看实际轨迹" in SWIFT_SOURCE
    assert '["mcp", "call", "get_robot_summary"]' in SWIFT_SOURCE
    assert "真实里程计 / actual path" in SWIFT_SOURCE


def test_conflict_helper_targets_legacy_mcp_and_all_control_ports() -> None:
    assert "dimos_dog_mcp.blueprint" in CONFLICT_HELPER
    for port in (7779, 9990, 3030, 9877):
        assert str(port) in CONFLICT_HELPER
