# Copyright 2025 Dimensional Inc.
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

"""Validation utilities for manipulator drivers."""

from typing import cast


def validate_joint_limits(
    positions: list[float],
    lower_limits: list[float],
    upper_limits: list[float],
    tolerance: float = 0.0,
) -> tuple[bool, str | None]:
    """Validate joint positions are within limits.

    Args:
        positions: Joint positions to validate (radians)
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)
        tolerance: Optional tolerance for soft limits (radians)

    Returns:
        Tuple of (is_valid, error_message)
        If valid, error_message is None
    """
    if len(positions) != len(lower_limits) or len(positions) != len(upper_limits):
        return False, f"Dimension mismatch: {len(positions)} positions, {len(lower_limits)} limits"

    for i, pos in enumerate(positions):
        lower = lower_limits[i] - tolerance
        upper = upper_limits[i] + tolerance

        if pos < lower:
            return False, f"Joint {i} position {pos:.3f} below limit {lower_limits[i]:.3f}"

        if pos > upper:
            return False, f"Joint {i} position {pos:.3f} above limit {upper_limits[i]:.3f}"

    return True, None


def validate_velocity_limits(
    velocities: list[float], max_velocities: list[float], scale_factor: float = 1.0
) -> tuple[bool, str | None]:
    """Validate joint velocities are within limits.

    Args:
        velocities: Joint velocities to validate (rad/s)
        max_velocities: Maximum allowed velocities (rad/s)
        scale_factor: Optional scaling factor (0-1) to reduce max velocity

    Returns:
        Tuple of (is_valid, error_message)
        If valid, error_message is None
    """
    if len(velocities) != len(max_velocities):
        return (
            False,
            f"Dimension mismatch: {len(velocities)} velocities, {len(max_velocities)} limits",
        )

    if scale_factor <= 0 or scale_factor > 1:
        return False, f"Invalid scale factor: {scale_factor} (must be in (0, 1])"

    for i, vel in enumerate(velocities):
        max_vel = max_velocities[i] * scale_factor

        if abs(vel) > max_vel:
            return False, f"Joint {i} velocity {abs(vel):.3f} exceeds limit {max_vel:.3f}"

    return True, None


def validate_acceleration_limits(
    accelerations: list[float], max_accelerations: list[float], scale_factor: float = 1.0
) -> tuple[bool, str | None]:
    """Validate joint accelerations are within limits.

    Args:
        accelerations: Joint accelerations to validate (rad/s²)
        max_accelerations: Maximum allowed accelerations (rad/s²)
        scale_factor: Optional scaling factor (0-1) to reduce max acceleration

    Returns:
        Tuple of (is_valid, error_message)
        If valid, error_message is None
    """
    if len(accelerations) != len(max_accelerations):
        return (
            False,
            f"Dimension mismatch: {len(accelerations)} accelerations, {len(max_accelerations)} limits",
        )

    if scale_factor <= 0 or scale_factor > 1:
        return False, f"Invalid scale factor: {scale_factor} (must be in (0, 1])"

    for i, acc in enumerate(accelerations):
        max_acc = max_accelerations[i] * scale_factor

        if abs(acc) > max_acc:
            return False, f"Joint {i} acceleration {abs(acc):.3f} exceeds limit {max_acc:.3f}"

    return True, None


def validate_trajectory(
    trajectory: list[dict[str, float | list[float]]],
    lower_limits: list[float],
    upper_limits: list[float],
    max_velocities: list[float] | None = None,
    max_accelerations: list[float] | None = None,
) -> tuple[bool, str | None]:
    """Validate a joint trajectory.

    Args:
        trajectory: List of waypoints, each with:
                   - 'positions': list[float] in radians
                   - 'velocities': Optional list[float] in rad/s
                   - 'time': float seconds from start
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)
        max_velocities: Optional maximum velocities (rad/s)
        max_accelerations: Optional maximum accelerations (rad/s²)

    Returns:
        Tuple of (is_valid, error_message)
        If valid, error_message is None
    """
    if not trajectory:
        return False, "Empty trajectory"

    # Check first waypoint starts at time 0
    if trajectory[0].get("time", 0) != 0:
        return False, "Trajectory must start at time 0"

    # Check waypoints are time-ordered
    prev_time: float = -1.0
    for i, waypoint in enumerate(trajectory):
        curr_time = cast("float", waypoint.get("time", 0))
        if curr_time <= prev_time:
            return False, f"Waypoint {i} time {curr_time} not after previous {prev_time}"
        prev_time = curr_time

    # Validate each waypoint
    for i, waypoint in enumerate(trajectory):
        # Check required fields
        if "positions" not in waypoint:
            return False, f"Waypoint {i} missing positions"

        positions = cast("list[float]", waypoint["positions"])

        # Validate position limits
        valid, error = validate_joint_limits(positions, lower_limits, upper_limits)
        if not valid:
            return False, f"Waypoint {i}: {error}"

        # Validate velocity limits if provided
        if "velocities" in waypoint and max_velocities:
            velocities = cast("list[float]", waypoint["velocities"])
            valid, error = validate_velocity_limits(velocities, max_velocities)
            if not valid:
                return False, f"Waypoint {i}: {error}"

    # Check acceleration limits between waypoints
    if max_accelerations and len(trajectory) > 1:
        for i in range(1, len(trajectory)):
            prev = trajectory[i - 1]
            curr = trajectory[i]

            dt = cast("float", curr["time"]) - cast("float", prev["time"])
            if dt <= 0:
                continue

            # Estimate acceleration from position change
            prev_pos = cast("list[float]", prev["positions"])
            curr_pos = cast("list[float]", curr["positions"])
            for j in range(len(prev_pos)):
                pos_change = curr_pos[j] - prev_pos[j]
                pos_change / dt

                # If velocities provided, use them for better estimate
                if "velocities" in prev and "velocities" in curr:
                    prev_vel = cast("list[float]", prev["velocities"])
                    curr_vel = cast("list[float]", curr["velocities"])
                    vel_change = curr_vel[j] - prev_vel[j]
                    acc = vel_change / dt
                    if abs(acc) > max_accelerations[j]:
                        return (
                            False,
                            f"Acceleration between waypoint {i - 1} and {i} joint {j}: {abs(acc):.3f} exceeds limit {max_accelerations[j]:.3f}",
                        )

    return True, None


def scale_velocities(
    velocities: list[float], max_velocities: list[float], scale_factor: float = 0.8
) -> list[float]:
    """Scale velocities to stay within limits.

    Args:
        velocities: Desired velocities (rad/s)
        max_velocities: Maximum allowed velocities (rad/s)
        scale_factor: Safety factor (0-1) to stay below limits

    Returns:
        Scaled velocities that respect limits
    """
    if not velocities or not max_velocities:
        return velocities

    # Find the joint that requires most scaling
    max_scale = 1.0
    for vel, max_vel in zip(velocities, max_velocities, strict=False):
        if max_vel > 0 and abs(vel) > 0:
            required_scale = abs(vel) / (max_vel * scale_factor)
            max_scale = max(max_scale, required_scale)

    # Apply uniform scaling to maintain direction
    if max_scale > 1.0:
        return [v / max_scale for v in velocities]

    return velocities


def clamp_positions(
    positions: list[float], lower_limits: list[float], upper_limits: list[float]
) -> list[float]:
    """Clamp positions to stay within limits.

    Args:
        positions: Desired positions (radians)
        lower_limits: Lower joint limits (radians)
        upper_limits: Upper joint limits (radians)

    Returns:
        Clamped positions within limits
    """
    clamped = []
    for pos, lower, upper in zip(positions, lower_limits, upper_limits, strict=False):
        clamped.append(max(lower, min(upper, pos)))
    return clamped
