# Copyright 2026 Dimensional Inc.
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

"""The controller seam: pose + Path in, body-frame Twist out.

The protocol is the deployment seam — on the robot the same object consumes
the pose off the odometry message and the planner topic; here the episode
runner feeds it the simulated equivalents. There is no tf lookup on either
side. The laws themselves live in
:mod:`dimos.navigation.motion.control.laws`, one module each, and a *track*
names one through :mod:`dimos.navigation.motion.control.tracks` — nothing
outside that map should name a law directly.
"""

from __future__ import annotations

from collections.abc import Callable
from importlib import import_module
import math
from typing import Any, Protocol

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.service.spec import BaseConfig


def angle_diff(a: float, b: float) -> float:
    return math.remainder(a - b, math.tau)


class ControllerConfig(BaseConfig):
    lookahead: float = 0.35  # carrot distance along the path (m)
    max_speed: float = 0.5  # planar clamp (m/s)
    max_yaw_rate: float = 1.4  # rad/s
    k_pos: float = 2.0  # body-frame position error gain (1/s)
    k_yaw: float = 2.0  # yaw error gain (1/s)
    # yaw-per-meter above this is a commanded rotation (fan), not a curve --
    # matches the referee's fan detection threshold (sim.py _fan_marks)
    fan_yaw_per_m: float = 3.0
    # while a fan segment is being executed, hold position and rotate until
    # the yaw error drops under this (rad)
    fan_yaw_done: float = 0.25
    # speed governor over the optional clearance annotation, the curve of
    # profile.py: cruise at max_speed with
    # speed_clearance of room, creep at min_speed at the precision floor,
    # linear between; judged over the next speed_lookahead metres of path
    min_speed: float = 0.2
    speed_clearance: float = 0.35
    speed_floor_clearance: float = 0.05  # the embodiment precision floor
    speed_lookahead: float = 2.0
    # Gait calibration, read by the blind law only (laws/blind.py).
    # The twist is a request to a walking policy that under-delivers it; these
    # inverse the measured deficit. They are properties of the POLICY BLOB,
    # not of any law: the defaults were measured off the freewalk gait blob by
    # the referee's walk-slip probe (with the sim, see README). That blob is NOT
    # in this repo and is not ours to redistribute. On a different gait these
    # are a ~23% over-speed, so re-probe before driving hardware.
    walk_gain: float = 0.964
    walk_slip: float = 0.132
    walk_slip_ramp: float = 0.08  # below this intended speed the correction fades out
    # Read by the hinted law only (laws/hinted.py).
    # Centred window the tangent feedforward reads the plan's direction over.
    tangent_preview: float = 0.15
    # The governor's pinch-escape leg: below escape_clearance of room (read over
    # escape_preview of arc, not the ramp's speed_lookahead) the lower anchor
    # rises toward escape_speed, because what kills in a gap is dwell.
    escape_clearance: float = 0.10
    escape_preview: float = 1.00
    escape_speed: float = 0.75
    # Brake-feasible preview: a previewed waypoint imposes only what it can,
    # given the body may decelerate on the way there. brake_accel 0 reproduces
    # the seed's flat minimum exactly.
    brake_accel: float = 0.8
    brake_margin: float = 0.15

    @property
    def law_params(self) -> tuple[float, ...]:
        """The numeric fields the rust laws take, in declaration order.

        The gait calibration travels separately (:attr:`walk_params`) because
        only one law reads it.
        """
        return (
            self.lookahead,
            self.max_speed,
            self.max_yaw_rate,
            self.k_pos,
            self.k_yaw,
            self.fan_yaw_per_m,
            self.fan_yaw_done,
            self.min_speed,
            self.speed_clearance,
            self.speed_floor_clearance,
            self.speed_lookahead,
        )

    @property
    def walk_params(self) -> tuple[float, float, float]:
        return (self.walk_gain, self.walk_slip, self.walk_slip_ramp)

    @property
    def hinted_params(self) -> tuple[float, float, float, float, float, float]:
        return (
            self.tangent_preview,
            self.escape_clearance,
            self.escape_preview,
            self.escape_speed,
            self.brake_accel,
            self.brake_margin,
        )


class TrajectoryController(Protocol):
    config: ControllerConfig

    def reset(self) -> None: ...

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist: ...


# name -> "module:factory"; arbitrary "module:factory" strings load too, so
# a candidate plugs in without registering (planners/base does the same).
# Tracks name these; see tracks.py.
REGISTRY = {
    "seed": "dimos.navigation.motion.control.laws.seed:make",
    "seed-rs": "dimos.navigation.motion.control.laws.seed:make_rust",
    "blind": "dimos.navigation.motion.control.laws.blind:make",
    "blind-rs": "dimos.navigation.motion.control.laws.blind:make_rust",
    "hinted": "dimos.navigation.motion.control.laws.hinted:make",
    "hinted-rs": "dimos.navigation.motion.control.laws.hinted:make_rust",
}

BUILD_CMD = (
    "uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml"
)


def load(name: str) -> Callable[..., TrajectoryController]:
    """Resolve a registry name or a dotted "module:factory" string."""
    target = REGISTRY.get(name, name)
    mod, _, attr = target.partition(":")
    factory: Any = getattr(import_module(mod), attr or "make")
    return factory  # type: ignore[no-any-return]


def load_extension() -> Any:
    """The built rust extension, or an ImportError naming the build command."""
    try:
        import dimos_motion2_tc
    except ImportError as e:
        raise ImportError(f"dimos_motion2_tc is not built; run: {BUILD_CMD}") from e
    return dimos_motion2_tc


def path_xy_yaw(path: Path) -> np.ndarray:
    """The plan as a contiguous (N, 3) float64 array, the rust marshalling."""
    return np.ascontiguousarray(
        np.array(
            [[p.position.x, p.position.y, p.yaw] for p in path.poses], dtype=np.float64
        ).reshape(-1, 3)
    )
