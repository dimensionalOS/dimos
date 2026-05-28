"""Pure approach geometry for the greeter (no DimOS).

Small velocity helpers the greeter container composes with the phase machine to
drive toward a person and hold a polite standoff:

- :func:`hold_distance_vx` -- forward/back speed that settles at the standoff
  distance (deadbanded, backs off if too close).
- :func:`centering_yaw` -- yaw rate that turns to keep the subject centered.
- :func:`approach_velocity` -- the two combined for the approach drive.

The Go2 ``cmd_vel`` is a body twist ``(vx forward, vy strafe, wz yaw)``; the
greeter uses only ``vx`` and ``wz`` (no orbiting). Pure: stdlib only, unit
tested.
"""

from __future__ import annotations

import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def _wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def centering_yaw(
    image_error_x: float, turn_gain: float, max_yaw_radps: float
) -> float:
    """Yaw rate that turns the robot to recenter the subject.

    Args:
        image_error_x: Normalized horizontal centering error in ``[-1, 1]``.
        turn_gain: Proportional gain from error to yaw rate.
        max_yaw_radps: Symmetric clamp on the yaw command.

    Returns:
        Yaw rate (rad/s); positive turns toward decreasing ``image_error_x``.
    """
    return _clamp(-turn_gain * image_error_x, -max_yaw_radps, max_yaw_radps)


def hold_distance_vx(
    distance_m: float,
    standoff_m: float,
    *,
    deadband_m: float = 0.15,
    forward_gain: float = 0.8,
    max_forward_mps: float = 0.5,
    max_reverse_mps: float = 0.3,
) -> float:
    """Forward/back speed that holds a standoff distance, with a deadband.

    Drives forward when too far and gently backs up when too close, so the robot
    keeps a safe gap (accounting for its own size) even if it overshoots toward
    the subject. The deadband is the key to *not* oscillating in/out: inside
    ``standoff_m +/- deadband_m`` the command is exactly zero, so once the robot
    is roughly at the standoff it holds still instead of hunting back and forth.

    Args:
        distance_m: Floor distance to the subject.
        standoff_m: Target polite distance to hold.
        deadband_m: Half-width of the no-correction band around the standoff.
        forward_gain: Proportional gain from distance error to speed.
        max_forward_mps: Clamp on driving forward (too far).
        max_reverse_mps: Clamp on backing up (too close).

    Returns:
        Forward speed (m/s); positive = forward, negative = back up, 0 in band.
    """
    error = distance_m - standoff_m
    if abs(error) <= deadband_m:
        return 0.0
    return _clamp(forward_gain * error, -max_reverse_mps, max_forward_mps)


def approach_velocity(
    distance_m: float,
    standoff_m: float,
    image_error_x: float,
    *,
    deadband_m: float = 0.15,
    forward_gain: float = 0.8,
    turn_gain: float = 1.0,
    max_forward_mps: float = 0.5,
    max_reverse_mps: float = 0.3,
    max_yaw_radps: float = 0.8,
) -> tuple[float, float]:
    """Velocity to hold the standoff distance while centering the subject.

    Forward/back from :func:`hold_distance_vx` (deadbanded, so it settles at the
    standoff without oscillating and backs off if it gets too close) plus a yaw
    to recenter the subject.

    Args:
        distance_m: Floor distance to the subject.
        standoff_m: Target polite distance to hold.
        image_error_x: Normalized horizontal centering error in ``[-1, 1]``.
        deadband_m: No-correction band around the standoff.
        forward_gain: Gain from distance error to forward speed.
        turn_gain: Gain from centering error to yaw rate.
        max_forward_mps: Clamp on the forward command.
        max_reverse_mps: Clamp on backing up.
        max_yaw_radps: Symmetric clamp on the yaw command.

    Returns:
        ``(vx, wz)`` to publish this tick.
    """
    vx = hold_distance_vx(
        distance_m, standoff_m, deadband_m=deadband_m, forward_gain=forward_gain,
        max_forward_mps=max_forward_mps, max_reverse_mps=max_reverse_mps)
    wz = centering_yaw(image_error_x, turn_gain, max_yaw_radps)
    return (vx, wz)


def drive_to_position(
    robot_xy: tuple[float, float],
    robot_yaw: float,
    target_xy: tuple[float, float],
    standoff_m: float,
    *,
    forward_gain: float = 0.8,
    turn_gain: float = 1.0,
    max_forward_mps: float = 0.5,
    max_yaw_radps: float = 0.8,
    heading_tol_rad: float = 0.5,
) -> tuple[float, float]:
    """Body twist to drive from an odom pose toward ``target_xy``, to the standoff.

    Yaws to face the target and drives forward only when roughly facing it
    (within ``heading_tol_rad``) and still beyond the standoff, so it turns
    first rather than lunging sideways. Pure geometry over the robot's odom
    pose -- used to reach a subject's last-known floor position when the visual
    tracker has dropped it (dead reckoning), so a brief loss does not abandon
    the engagement.

    Args:
        robot_xy: Robot ``(x, y)`` in the world/odom frame.
        robot_yaw: Robot heading (rad) in the same frame.
        target_xy: Target ``(x, y)`` in the same frame.
        standoff_m: Distance to stop short of the target.
        forward_gain: Proportional gain from range error to forward speed.
        turn_gain: Proportional gain from heading error to yaw rate.
        max_forward_mps: Clamp on the forward command.
        max_yaw_radps: Symmetric clamp on the yaw command.
        heading_tol_rad: Only drive forward when the heading error is within
            this (else turn in place first).

    Returns:
        ``(vx, wz)`` to publish this tick.
    """
    dx = target_xy[0] - robot_xy[0]
    dy = target_xy[1] - robot_xy[1]
    range_m = math.hypot(dx, dy)
    heading_error = _wrap_angle(math.atan2(dy, dx) - robot_yaw)
    wz = _clamp(turn_gain * heading_error, -max_yaw_radps, max_yaw_radps)
    if range_m > standoff_m and abs(heading_error) < heading_tol_rad:
        vx = _clamp(forward_gain * (range_m - standoff_m), 0.0, max_forward_mps)
    else:
        vx = 0.0
    return (vx, wz)
