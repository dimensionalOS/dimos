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

"""Spot camera/depth/odometry viewer.

Powers on, stands the robot, and streams the five fisheye + five depth cameras
and body odometry into a Rerun viewer laid out as two vertical columns: the five
depth cameras on the left, the five grayscale cameras on the right. Unlike the
full `spot` blueprint there is no teleop, so the robot stands but does not drive.

Everything rides the default Zenoh transport, which the Rerun bridge subscribes
to, so the viewer receives and renders the frames.

Usage:
    dimos run spot-cameras \
        -o spothighlevel.username=admin -o spothighlevel.password=<password>
"""

from __future__ import annotations

import rerun.blueprint as rrb

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.bosdyn.spot.effectors.high_level import SpotHighLevel
from dimos.visualization.rerun.bridge import RerunBridgeModule

_NUM_CAMERAS = 5


def spot_camera_layout() -> rrb.Blueprint:
    """Depth column on the left, grayscale column on the right."""
    depth_column = rrb.Vertical(
        *[
            rrb.Spatial2DView(origin=f"world/depth_image_{index}", name=f"depth {index}")
            for index in range(1, _NUM_CAMERAS + 1)
        ]
    )
    grayscale_column = rrb.Vertical(
        *[
            rrb.Spatial2DView(origin=f"world/grayscale_image_{index}", name=f"camera {index}")
            for index in range(1, _NUM_CAMERAS + 1)
        ]
    )
    return rrb.Blueprint(rrb.Horizontal(depth_column, grayscale_column), collapse_panels=True)


spot_cameras = autoconnect(
    SpotHighLevel.blueprint(),
    RerunBridgeModule.blueprint(blueprint=spot_camera_layout),
)
