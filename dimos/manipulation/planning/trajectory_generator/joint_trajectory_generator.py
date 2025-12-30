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

"""
Joint Trajectory Generator

Generates time-parameterized joint trajectories from waypoints using
trapezoidal velocity profiles.

Trapezoidal Profile:
    velocity
       ^
       |    ____________________
       |   /                    \
       |  /                      \
       | /                        \
       |/                          \
       +------------------------------> time
        accel    cruise      decel
"""

import math

from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryPoint


class JointTrajectoryGenerator:
    """
    Generates joint trajectories with trapezoidal velocity profiles.

    For each segment between waypoints:
    1. Determines the limiting joint (one that takes longest)
    2. Applies trapezoidal velocity profile based on limits
    3. Scales other joints to complete in the same time
    4. Generates trajectory points with proper timing

    Usage:
        generator = JointTrajectoryGenerator(num_joints=6)
        generator.set_limits(max_velocity=1.0, max_acceleration=2.0)
        trajectory = generator.generate(waypoints)
    """

    def __init__(
        self,
        num_joints: int = 6,
        max_velocity: list[float] | float = 1.0,
        max_acceleration: list[float] | float = 2.0,
        points_per_segment: int = 50,
    ) -> None:
        """
        Initialize trajectory generator.

        Args:
            num_joints: Number of joints
            max_velocity: rad/s (single value applies to all joints, or per-joint list)
            max_acceleration: rad/s^2 (single value or per-joint list)
            points_per_segment: Number of intermediate points per waypoint segment
        """
        self.num_joints = num_joints
        self.points_per_segment = points_per_segment

        # Initialize limits
        self.max_velocity: list[float] = []
        self.max_acceleration: list[float] = []
        self.set_limits(max_velocity, max_acceleration)

    def set_limits(
        self,
        max_velocity: list[float] | float,
        max_acceleration: list[float] | float,
    ) -> None:
        """
        Set velocity and acceleration limits.

        Args:
            max_velocity: rad/s (single value applies to all joints, or per-joint)
            max_acceleration: rad/s^2 (single value or per-joint)
        """
        if isinstance(max_velocity, (int, float)):
            self.max_velocity = [float(max_velocity)] * self.num_joints
        else:
            self.max_velocity = list(max_velocity)

        if isinstance(max_acceleration, (int, float)):
            self.max_acceleration = [float(max_acceleration)] * self.num_joints
        else:
            self.max_acceleration = list(max_acceleration)

    def generate(self, waypoints: list[list[float]]) -> JointTrajectory:
        """
        Generate a trajectory through waypoints with trapezoidal velocity profile.

        Args:
            waypoints: List of joint positions [q1, q2, ..., qn] in radians
                       First waypoint is start, last is goal

        Returns:
            JointTrajectory with time-parameterized points
        """
        if not waypoints or len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        all_points: list[TrajectoryPoint] = []
        current_time = 0.0

        # Add first waypoint
        all_points.append(
            TrajectoryPoint(
                time_from_start=0.0,
                positions=list(waypoints[0]),
                velocities=[0.0] * self.num_joints,
            )
        )

        # Process each segment
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            # Generate segment with trapezoidal profile
            segment_points, segment_duration = self._generate_segment(start, end, current_time)

            # Add points (skip first as it duplicates previous endpoint)
            all_points.extend(segment_points[1:])
            current_time += segment_duration

        return JointTrajectory(points=all_points)

    def _generate_segment(
        self,
        start: list[float],
        end: list[float],
        start_time: float,
    ) -> tuple[list[TrajectoryPoint], float]:
        """
        Generate trajectory points for a single segment using trapezoidal profile.

        Args:
            start: Starting joint positions
            end: Ending joint positions
            start_time: Time offset for this segment

        Returns:
            Tuple of (list of TrajectoryPoints, segment duration)
        """
        # Calculate displacement for each joint
        displacements = [end[j] - start[j] for j in range(self.num_joints)]

        # Find the limiting joint (one that takes longest)
        segment_duration = 0.0
        for j in range(self.num_joints):
            t = self._compute_trapezoidal_time(
                abs(displacements[j]),
                self.max_velocity[j],
                self.max_acceleration[j],
            )
            segment_duration = max(segment_duration, t)

        # Ensure minimum duration
        segment_duration = max(segment_duration, 0.01)

        # Generate points along the segment
        points: list[TrajectoryPoint] = []

        for i in range(self.points_per_segment + 1):
            # Normalized time [0, 1]
            s = i / self.points_per_segment
            t = start_time + s * segment_duration

            # Compute position and velocity for each joint
            positions = []
            velocities = []

            for j in range(self.num_joints):
                # Compute scaled limits for this joint to fit in segment_duration
                v_scaled, a_scaled = self._compute_scaled_limits(
                    abs(displacements[j]),
                    segment_duration,
                    self.max_velocity[j],
                    self.max_acceleration[j],
                )

                pos, vel = self._trapezoidal_interpolate(
                    s,
                    start[j],
                    end[j],
                    segment_duration,
                    v_scaled,
                    a_scaled,
                )
                positions.append(pos)
                velocities.append(vel)

            points.append(
                TrajectoryPoint(
                    time_from_start=t,
                    positions=positions,
                    velocities=velocities,
                )
            )

        return points, segment_duration

    def _compute_trapezoidal_time(
        self,
        distance: float,
        v_max: float,
        a_max: float,
    ) -> float:
        """
        Compute time to travel a distance with trapezoidal velocity profile.

        Two cases:
        1. Triangle profile: Can't reach v_max (short distance)
        2. Trapezoidal profile: Reaches v_max with cruise phase

        Args:
            distance: Absolute distance to travel
            v_max: Maximum velocity
            a_max: Maximum acceleration

        Returns:
            Time to complete the motion
        """
        if distance < 1e-9:
            return 0.0

        # Time to accelerate to v_max
        t_accel = v_max / a_max

        # Distance covered during accel + decel (both at a_max)
        d_accel = 0.5 * a_max * t_accel**2
        d_total_ramp = 2 * d_accel  # accel + decel

        if distance <= d_total_ramp:
            # Triangle profile - can't reach v_max
            # d = 2 * (0.5 * a * t^2) = a * t^2
            # t = sqrt(d / a)
            t_ramp = math.sqrt(distance / a_max)
            return 2 * t_ramp
        else:
            # Trapezoidal profile - has cruise phase
            d_cruise = distance - d_total_ramp
            t_cruise = d_cruise / v_max
            return 2 * t_accel + t_cruise

    def _compute_scaled_limits(
        self,
        distance: float,
        duration: float,
        v_max: float,
        a_max: float,
    ) -> tuple[float, float]:
        """
        Compute scaled velocity and acceleration to travel distance in given duration.

        This scales down the profile so the joint travels its distance in the
        same time as the limiting joint.

        Args:
            distance: Absolute distance to travel
            duration: Required duration (from limiting joint)
            v_max: Maximum velocity limit
            a_max: Maximum acceleration limit

        Returns:
            Tuple of (scaled_velocity, scaled_acceleration)
        """
        if distance < 1e-9 or duration < 1e-9:
            return v_max, a_max

        # Compute optimal time for this joint
        t_opt = self._compute_trapezoidal_time(distance, v_max, a_max)

        if t_opt >= duration - 1e-9:
            # This is the limiting joint or close to it
            return v_max, a_max

        # Need to scale down to fit in longer duration
        # Use simple scaling: scale both v and a by the same factor
        # This preserves the profile shape
        scale = t_opt / duration

        # For a symmetric trapezoidal/triangular profile:
        # If we scale time by k, we need to scale velocity by 1/k
        # But we also need to ensure we travel the same distance

        # Simpler approach: compute the average velocity needed
        distance / duration

        # For trapezoidal profile, v_avg = v_peak * (1 - t_accel/duration)
        # For simplicity, use a heuristic: scale velocity so trajectory fits

        # Check if we can use a triangle profile
        # Triangle: d = 0.5 * v_peak * T, so v_peak = 2 * d / T
        v_peak_triangle = 2 * distance / duration
        a_for_triangle = 4 * distance / (duration * duration)

        if v_peak_triangle <= v_max and a_for_triangle <= a_max:
            # Use triangle profile with these params
            return v_peak_triangle, a_for_triangle

        # Use trapezoidal with reduced velocity
        # Solve: distance = v * t_cruise + v^2/a
        # where t_cruise = duration - 2*v/a
        # This is complex, so use iterative scaling
        v_scaled = v_max * scale
        a_scaled = a_max * scale * scale  # acceleration scales with square of time scale

        # Verify and adjust
        t_check = self._compute_trapezoidal_time(distance, v_scaled, a_scaled)
        if abs(t_check - duration) > 0.01 * duration:
            # Fallback: use triangle profile scaled to fit
            v_scaled = 2 * distance / duration
            a_scaled = 4 * distance / (duration * duration)

        return min(v_scaled, v_max), min(a_scaled, a_max)

    def _trapezoidal_interpolate(
        self,
        s: float,
        start: float,
        end: float,
        duration: float,
        v_max: float,
        a_max: float,
    ) -> tuple[float, float]:
        """
        Interpolate position and velocity using trapezoidal profile.

        Args:
            s: Normalized time [0, 1]
            start: Start position
            end: End position
            duration: Total segment duration
            v_max: Max velocity for this joint (scaled)
            a_max: Max acceleration for this joint (scaled)

        Returns:
            Tuple of (position, velocity)
        """
        distance = abs(end - start)
        direction = 1.0 if end >= start else -1.0

        if distance < 1e-9 or duration < 1e-9:
            return end, 0.0

        # Handle endpoint exactly
        if s >= 1.0 - 1e-9:
            return end, 0.0
        if s <= 1e-9:
            return start, 0.0

        # Current time
        t = s * duration

        # Compute profile parameters for this joint
        t_accel = v_max / a_max if a_max > 1e-9 else duration / 2
        d_accel = 0.5 * a_max * t_accel**2
        d_total_ramp = 2 * d_accel

        if distance <= d_total_ramp + 1e-9:
            # Triangle profile
            t_peak = duration / 2
            v_peak = 2 * distance / duration
            a_eff = v_peak / t_peak if t_peak > 1e-9 else a_max

            if t <= t_peak:
                # Accelerating
                pos_offset = 0.5 * a_eff * t * t
                vel = direction * a_eff * t
            else:
                # Decelerating
                dt = t - t_peak
                pos_offset = distance / 2 + v_peak * dt - 0.5 * a_eff * dt * dt
                vel = direction * max(0.0, v_peak - a_eff * dt)
        else:
            # Trapezoidal profile
            d_cruise = distance - d_total_ramp
            t_cruise = d_cruise / v_max if v_max > 1e-9 else 0

            if t <= t_accel:
                # Accelerating phase
                pos_offset = 0.5 * a_max * t * t
                vel = direction * a_max * t
            elif t <= t_accel + t_cruise:
                # Cruise phase
                dt = t - t_accel
                pos_offset = d_accel + v_max * dt
                vel = direction * v_max
            else:
                # Decelerating phase
                dt = t - t_accel - t_cruise
                pos_offset = d_accel + d_cruise + v_max * dt - 0.5 * a_max * dt * dt
                vel = direction * max(0.0, v_max - a_max * dt)

        position = start + direction * pos_offset

        # Clamp to ensure we don't overshoot
        if direction > 0:
            position = min(position, end)
        else:
            position = max(position, end)

        return position, vel

    def preview(self, trajectory: JointTrajectory) -> str:
        """
        Generate a text preview of the trajectory.

        Args:
            trajectory: Generated trajectory to preview

        Returns:
            Formatted string showing trajectory details
        """
        lines = [
            "Trajectory Preview",
            "=" * 60,
            f"Duration: {trajectory.duration:.3f}s",
            f"Points: {len(trajectory.points)}",
            "",
            "Waypoints (time -> positions):",
            "-" * 60,
        ]

        # Show key points (first, last, and evenly spaced)
        indices = [0]
        step = max(1, len(trajectory.points) // 5)
        indices.extend(range(step, len(trajectory.points) - 1, step))
        indices.append(len(trajectory.points) - 1)
        indices = sorted(set(indices))

        for i in indices:
            pt = trajectory.points[i]
            pos_str = ", ".join(f"{p:+.3f}" for p in pt.positions)
            vel_str = ", ".join(f"{v:+.3f}" for v in pt.velocities)
            lines.append(f"  t={pt.time_from_start:6.3f}s: pos=[{pos_str}]")
            lines.append(f"           vel=[{vel_str}]")

        lines.append("-" * 60)
        return "\n".join(lines)
