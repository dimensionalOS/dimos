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

"""M20 navigation driving the simulated walking M20 to a clicked goal."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    _m20_nav,
    m20_nav_overrides,
    m20_nav_static,
)
from dimos.simulation.m20_sim.world import SimM20World
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D


def _scene_lines(scene: LineSegments3D) -> Archetype:
    return scene.to_rerun(radii=0.01)


def _rerun_blueprint() -> Blueprint:
    # rerun is heavy, loaded only in the viewer's worker
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="M20 sim",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "tf_axes": 0.35,
    "static": m20_nav_static(),
    "visual_override": {**m20_nav_overrides(), "world/scene": _scene_lines},
}

m20_sim_nav = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
    SimM20World.blueprint().remappings(
        [(SimM20World, "lidar", "slam_body_points"), (SimM20World, "odometry", "slam_odom")]
    ),
    _m20_nav,
    # gossip off until zenoh fixes its pending-connection bug: with it on, native modules
    # spawned together never link
).global_config(transport="zenoh", zenoh_gossip=False, n_workers=8)
