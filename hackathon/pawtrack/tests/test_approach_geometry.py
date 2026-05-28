"""Tests for the pure approach geometry helpers."""

import math

import pytest

from pawtrack.approach_geometry import (
    approach_velocity,
    centering_yaw,
    drive_to_position,
    hold_distance_vx,
)


def test_centering_yaw_opposes_error_and_clamps():
    # Subject right of center (positive error) -> turn the other way (negative).
    assert centering_yaw(0.5, 1.0, 0.8) == -0.5
    assert centering_yaw(-0.5, 1.0, 0.8) == 0.5
    assert centering_yaw(2.0, 1.0, 0.8) == -0.8  # clamped to the yaw limit


def test_hold_distance_vx_holds_with_deadband_and_safe_reverse():
    # Too far -> drive forward (clamped to the speed limit).
    assert hold_distance_vx(
        3.0, 1.5, forward_gain=0.8, max_forward_mps=0.5) == 0.5
    # Inside the deadband around the standoff -> hold (no hunting in/out).
    assert hold_distance_vx(1.55, 1.5, deadband_m=0.15) == 0.0
    assert hold_distance_vx(1.45, 1.5, deadband_m=0.15) == 0.0
    # Too close (beyond the deadband) -> back off to a safe gap, clamped.
    assert hold_distance_vx(
        1.0, 1.5, deadband_m=0.15, forward_gain=0.8, max_reverse_mps=0.3) == -0.3


def test_approach_velocity_holds_standoff_and_centers():
    vx, wz = approach_velocity(
        3.0, 1.5, 0.0, forward_gain=0.8, max_forward_mps=0.5)
    assert vx == 0.5 and wz == 0.0  # far -> forward, centered

    vx, _ = approach_velocity(1.5, 1.5, 0.0)  # at standoff
    assert vx == 0.0  # hold

    vx, wz = approach_velocity(
        1.7, 1.5, 0.3, deadband_m=0.15, forward_gain=0.8, turn_gain=1.0,
        max_yaw_radps=0.8)
    assert vx == pytest.approx(0.8 * 0.2)  # 0.2 > deadband -> ease forward
    assert wz == pytest.approx(-0.3)  # turn to recenter


def test_drive_to_position_faces_then_advances_to_standoff():
    # Target straight ahead (+x), robot facing +x: drive forward, no yaw.
    vx, wz = drive_to_position(
        (0.0, 0.0), 0.0, (3.0, 0.0), 1.0, forward_gain=0.8, max_forward_mps=0.5)
    assert vx == 0.5 and wz == pytest.approx(0.0)  # 0.8*(3-1)=1.6 -> clamp 0.5

    # Target to the left, robot facing +x: turn toward it, do not lunge sideways.
    vx, wz = drive_to_position(
        (0.0, 0.0), 0.0, (0.0, 3.0), 1.0, turn_gain=1.0, max_yaw_radps=0.8,
        heading_tol_rad=0.5)
    assert vx == 0.0  # heading error ~pi/2 > tol -> turn in place first
    assert wz == pytest.approx(0.8)  # positive yaw toward +y, clamped

    # Inside the standoff already: hold (no forward), roughly facing.
    vx, wz = drive_to_position((0.0, 0.0), 0.0, (0.5, 0.0), 1.0)
    assert vx == 0.0  # range < standoff


def test_drive_to_position_wraps_heading_error():
    # Target behind the robot: should turn (yaw != 0), not drive forward.
    vx, wz = drive_to_position(
        (0.0, 0.0), 0.0, (-3.0, 0.001), 1.0, heading_tol_rad=0.5)
    assert vx == 0.0
    assert abs(wz) > 0.0 and abs(math.pi - abs(wz / 1.0)) >= 0  # turning around
