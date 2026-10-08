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

"""The legged Go2 with its Mid-360 in a generated office, driven from the rerun viewer."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.global_planner.viz import body_on_base_link
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.go2_mid360_static_transforms import Go2Mid360StaticTf
from dimos.simulation.go2_sim.world import SimGo2World
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D


def _scene_lines(scene: LineSegments3D) -> Archetype:
    return scene.to_rerun(radii=0.01)


def rerun_blueprint(hidden: tuple[str, ...] = ()) -> Blueprint:
    """One 3D view. Hidden entities stay in the entity tree, tickable in the viewer."""
    # rerun is heavy, loaded only in the viewer's worker
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            overrides={entity: rrb.EntityBehavior(visible=False) for entity in hidden},
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config: dict[str, Any] = {
    "blueprint": rerun_blueprint,
    "tf_axes": 0.3,
    "static": {
        "world/robot_body": partial(
            body_on_base_link, length=ROBOT_LENGTH, width=ROBOT_WIDTH, height=ROBOT_HEIGHT
        )
    },
    "visual_override": {"world/scene": _scene_lines},
}

go2_sim = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    SimGo2World.blueprint(),
    Go2Mid360StaticTf.blueprint(),
    MovementManager.blueprint(),
    # gossip off until zenoh fixes its pending-connection bug: with it on, native modules
    # spawned together never link, which the motion stack composed on this blueprint needs
).global_config(transport="zenoh", zenoh_gossip=False, n_workers=5, robot_model="unitree_go2")
