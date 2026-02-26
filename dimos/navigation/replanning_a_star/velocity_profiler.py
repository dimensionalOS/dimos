# Copyright 2025-2026 Dimensional Inc.
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

"""Velocity profiler for path-aware speed control."""

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs import Path


class VelocityProfiler:
    """Computes optimal velocity profile along a path based on curvature and constraints.
    
    The profiler:
    1. Calculates curvature at each path point
    2. Determines max safe speed based on curvature (centripetal force limits)
    3. Applies acceleration/deceleration constraints
    4. Returns speed at each point
    """
    
    def __init__(
        self,
        max_linear_speed: float = 0.8,  # ~3-4x current 0.55 m/s
        max_angular_speed: float = 1.5,
        max_linear_accel: float = 1.0,  # m/s^2
        max_linear_decel: float = 2.0,  # m/s^2 (braking can be faster)
        max_centripetal_accel: float = 1.0,  # m/s^2 (lateral acceleration limit)
        min_speed: float = 0.05,  # Minimum speed to avoid stalling
    ):
        """Initialize velocity profiler.
        
        Args:
            max_linear_speed: Maximum forward speed (m/s)
            max_angular_speed: Maximum angular speed (rad/s)
            max_linear_accel: Maximum linear acceleration (m/s^2)
            max_linear_decel: Maximum linear deceleration (m/s^2)
            max_centripetal_accel: Maximum centripetal acceleration (m/s^2)
            min_speed: Minimum speed to maintain (m/s)
        """
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._max_linear_accel = max_linear_accel
        self._max_linear_decel = max_linear_decel
        self._max_centripetal_accel = max_centripetal_accel
        self._min_speed = min_speed
    
    def compute_profile(self, path: Path) -> NDArray[np.float64]:
        """Compute velocity profile for entire path.
        
        Args:
            path: Path to profile
            
        Returns:
            Array of velocities (m/s) for each path point
        """
        if len(path.poses) < 2:
            return np.array([self._min_speed])
        
        # Convert path to numpy array
        path_points = np.array([[p.position.x, p.position.y] for p in path.poses])
        
        # Calculate curvatures
        curvatures = self._compute_curvatures(path_points)
        
        # Calculate max speeds based on curvature
        max_speeds = self._compute_max_speeds_from_curvature(curvatures)
        
        # Apply acceleration constraints (forward pass)
        velocities = self._apply_acceleration_constraints(
            path_points, max_speeds, forward=True
        )
        
        # Apply deceleration constraints (backward pass)
        velocities = self._apply_acceleration_constraints(
            path_points, velocities, forward=False
        )
        
        # Ensure minimum speed
        velocities = np.maximum(velocities, self._min_speed)
        
        return velocities
    
    def get_velocity_at_index(self, path: Path, index: int) -> float:
        """Get velocity at specific path index.
        
        Args:
            path: Path
            index: Path point index
            
        Returns:
            Velocity at that point (m/s)
        """
        velocities = self.compute_profile(path)
        idx = min(max(0, index), len(velocities) - 1)
        return float(velocities[idx])
    
    def _compute_curvatures(self, path_points: NDArray[np.float64]) -> NDArray[np.float64]:
        """Compute curvature at each path point.
        
        Curvature κ = |dθ/ds| where θ is heading and s is arc length.
        For discrete points, we use: κ = |Δθ| / |Δs|
        
        Args:
            path_points: Array of [x, y] points
            
        Returns:
            Array of curvatures (1/m) at each point
        """
        n = len(path_points)
        if n < 3:
            return np.zeros(n)
        
        curvatures = np.zeros(n)
        
        # First point: use first segment
        if n > 1:
            ds1 = np.linalg.norm(path_points[1] - path_points[0])
            if n > 2:
                ds2 = np.linalg.norm(path_points[2] - path_points[1])
                dtheta = self._angle_between_segments(
                    path_points[0], path_points[1], path_points[2]
                )
                if ds1 + ds2 > 1e-6:
                    curvatures[0] = abs(dtheta) / (ds1 + ds2)
        
        # Middle points: use three-point method
        for i in range(1, n - 1):
            p0 = path_points[i - 1]
            p1 = path_points[i]
            p2 = path_points[i + 1]
            
            ds1 = np.linalg.norm(p1 - p0)
            ds2 = np.linalg.norm(p2 - p1)
            dtheta = self._angle_between_segments(p0, p1, p2)
            
            if ds1 + ds2 > 1e-6:
                curvatures[i] = abs(dtheta) / (ds1 + ds2)
        
        # Last point: use last segment
        if n > 1:
            ds1 = np.linalg.norm(path_points[n - 1] - path_points[n - 2])
            if n > 2:
                ds2 = np.linalg.norm(path_points[n - 2] - path_points[n - 3])
                dtheta = self._angle_between_segments(
                    path_points[n - 3], path_points[n - 2], path_points[n - 1]
                )
                if ds1 + ds2 > 1e-6:
                    curvatures[n - 1] = abs(dtheta) / (ds1 + ds2)
        
        return curvatures
    
    def _angle_between_segments(
        self, p0: NDArray[np.float64], p1: NDArray[np.float64], p2: NDArray[np.float64]
    ) -> float:
        """Compute angle change between two path segments.
        
        Args:
            p0, p1, p2: Three consecutive points
            
        Returns:
            Angle change in radians
        """
        v1 = p1 - p0
        v2 = p2 - p1
        
        # Normalize
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        
        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0
        
        v1 = v1 / norm1
        v2 = v2 / norm2
        
        # Angle between vectors
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # Determine sign using cross product (for 2D)
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if cross < 0:
            angle = -angle
        
        return float(angle)
    
    def _compute_max_speeds_from_curvature(
        self, curvatures: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Compute maximum safe speed based on curvature.
        
        From centripetal acceleration: a = v²/r = v²·κ
        Therefore: v_max = sqrt(a_max / κ)
        
        Args:
            curvatures: Curvature at each point (1/m)
            
        Returns:
            Maximum safe speeds (m/s)
        """
        max_speeds = np.full(len(curvatures), self._max_linear_speed)
        
        # For non-zero curvature, limit speed
        nonzero_curvature = curvatures > 1e-6
        if np.any(nonzero_curvature):
            # v_max = sqrt(a_max / κ)
            curvature_limited = np.sqrt(
                self._max_centripetal_accel / curvatures[nonzero_curvature]
            )
            max_speeds[nonzero_curvature] = np.minimum(
                max_speeds[nonzero_curvature], curvature_limited
            )
        
        return max_speeds
    
    def _apply_acceleration_constraints(
        self,
        path_points: NDArray[np.float64],
        max_speeds: NDArray[np.float64],
        forward: bool = True,
    ) -> NDArray[np.float64]:
        """Apply acceleration/deceleration constraints.
        
        Args:
            path_points: Path points
            max_speeds: Maximum speeds at each point
            forward: If True, apply forward (acceleration), else backward (deceleration)
            
        Returns:
            Velocities with acceleration constraints applied
        """
        velocities = max_speeds.copy()
        max_accel = self._max_linear_accel if forward else self._max_linear_decel
        
        if forward:
            # Forward pass: can't accelerate too fast
            for i in range(1, len(path_points)):
                ds = np.linalg.norm(path_points[i] - path_points[i - 1])
                if ds > 1e-6:
                    # v² = v₀² + 2·a·s
                    # v = sqrt(v₀² + 2·a·s)
                    v_prev = velocities[i - 1]
                    v_max_from_accel = np.sqrt(v_prev**2 + 2 * max_accel * ds)
                    velocities[i] = min(velocities[i], v_max_from_accel)
        else:
            # Backward pass: can't decelerate too fast
            for i in range(len(path_points) - 2, -1, -1):
                ds = np.linalg.norm(path_points[i + 1] - path_points[i])
                if ds > 1e-6:
                    # v² = v₀² + 2·a·s (backward)
                    v_next = velocities[i + 1]
                    v_max_from_decel = np.sqrt(v_next**2 + 2 * max_accel * ds)
                    velocities[i] = min(velocities[i], v_max_from_decel)
        
        return velocities