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
names one by its ``"module:factory"`` through
:mod:`dimos.navigation.motion.control.tracks`.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import TypeAdapter

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.navigation.motion.embodiment.base import Embodiment


def angle_diff(a: float, b: float) -> float:
    return math.remainder(a - b, math.tau)


class ControllerConfig(BaseConfig):
    """A law's tuning. Everything the plant dictates -- speeds, the governor's
    band, the yaw rate, slew, slip -- is the embodiment's and is read from it;
    what is here is what a search over a fixed body may move -- and so
    nothing here has a default: a body brings the numbers searched on it
    (`embodiment/go2.py::GO2.control`), or explicitly borrows another's.
    """

    lookahead: float  # carrot distance along the path (m)
    k_pos: float  # body-frame position error gain (1/s)
    k_yaw: float  # yaw error gain (1/s)
    # yaw-per-meter above this is a commanded rotation (fan), not a curve
    fan_yaw_per_m: float
    # while a fan segment is being executed, hold position and rotate until
    # the yaw error drops under this (rad)
    fan_yaw_done: float
    # the governor (the embodiment's curve, control/profile.py) is judged over
    # the next speed_lookahead metres of path
    speed_lookahead: float
    # Read by the hinted law only (laws/hinted.py).
    # Centred window the tangent feedforward reads the plan's direction over.
    tangent_preview: float
    # The governor's pinch-escape leg: below escape_clearance of room (read over
    # escape_preview of arc, not the ramp's speed_lookahead) the lower anchor
    # rises toward escape_speed, because what kills in a gap is dwell.
    escape_clearance: float
    escape_preview: float
    escape_speed: float
    # Brake-feasible preview: a previewed waypoint imposes only what it can,
    # given the body may decelerate on the way there. brake_accel 0 reproduces
    # the seed's flat minimum exactly.
    brake_accel: float
    brake_margin: float


class TrajectoryController(Protocol):
    config: ControllerConfig

    def reset(self) -> None: ...

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: NDArray[np.float64] | None = None
    ) -> Twist: ...


BUILD_CMD = (
    "uv run maturin develop --uv --release -m dimos/navigation/motion/control/rust/Cargo.toml"
)


def load_extension() -> Any:
    """The built rust extension, or an ImportError naming the build command."""
    try:
        import dimos_motion2_tc
    except ImportError as e:
        raise ImportError(f"dimos_motion2_tc is not built; run: {BUILD_CMD}") from e
    return dimos_motion2_tc


def emb_json(emb: Embodiment) -> str:
    """The body as the rust laws take it: the same JSON dict the planner crate reads."""
    from dimos.navigation.motion.embodiment.base import Embodiment  # cyclic at import time

    return TypeAdapter(Embodiment).dump_json(emb).decode()


def path_xy_yaw(path: Path) -> NDArray[np.float64]:
    """The plan as a contiguous (N, 3) float64 array, the rust marshalling."""
    return np.ascontiguousarray(
        np.array(
            [[p.position.x, p.position.y, p.yaw] for p in path.poses], dtype=np.float64
        ).reshape(-1, 3)
    )
