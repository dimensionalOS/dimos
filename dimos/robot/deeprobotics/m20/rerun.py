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

"""Rerun views for the M20 blueprints: the navigation overlays, the cameras, and the real and sim layouts."""

from __future__ import annotations

from collections.abc import Callable
from types import ModuleType
from typing import TYPE_CHECKING, Any

from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.viz import planner_visual_override
from dimos.navigation.global_planner.viz import render_path
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    PLANNER_VIZ_HZ,
    VOXEL_SIZE_M,
    WALL_CLEARANCE_M,
)
from dimos.robot.deeprobotics.m20.constants import (
    BASE_LINK_HEIGHT_M,
    BODY_LENGTH_M,
    BODY_WIDTH_M,
    PLANNING_HEIGHT_M,
)

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    from dimos_messages.dimos_msgs.msg import LineSegments3D
    from dimos_messages.foxglove_msgs.msg import CompressedVideo
    from dimos_messages.sensor_msgs.msg import CameraInfo
    from dimos.visualization.rerun.bridge import VisualOverride


def _robot_body(rr: ModuleType) -> list[Archetype]:
    return [
        rr.Boxes3D(
            half_sizes=[
                BODY_LENGTH_M * 0.5,
                BODY_WIDTH_M * 0.5,
                PLANNING_HEIGHT_M * 0.5,
            ],
            centers=[0.0, 0.0, PLANNING_HEIGHT_M * 0.5 - BASE_LINK_HEIGHT_M],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def m20_nav_static() -> dict[str, Callable[[ModuleType], list[Archetype]]]:
    """Bridge static entities for navigation: the body box on base_link."""
    return {"world/robot_body": _robot_body}


def m20_nav_overrides() -> dict[str, VisualOverride]:
    """Bridge overrides for the navigation maps, paths and planner debug, with the raw clouds hidden."""
    return {
        "world/lidar": None,
        "world/slam_body_points": None,
        "world/planner_path": render_path,
        **motion_visual_override(embodiment=M20),
        **planner_visual_override(
            PLANNER_VIZ_HZ, voxel_size=VOXEL_SIZE_M, wall_clearance_m=WALL_CLEARANCE_M
        ),
    }


def _render_h265(msg: CompressedVideo) -> Any:
    import rerun as rr

    return rr.VideoStream(codec=rr.VideoCodec.H265, sample=msg.data.tobytes())


def _render_front_camera_info(msg: CameraInfo) -> Any:
    from dimos.visualization.rerun.cdr import camera_pinhole

    return [("world/front_camera", camera_pinhole(msg, "front_camera_optical"))]


def _render_rear_camera_info(msg: CameraInfo) -> Any:
    from dimos.visualization.rerun.cdr import camera_pinhole

    return [("world/rear_camera", camera_pinhole(msg, "rear_camera_optical"))]


def _m20_rerun_blueprint() -> Any:
    """Go2-style navigation layout with both M20 camera streams."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/front_camera", name="Front Camera"),
                rrb.Spatial2DView(origin="world/rear_camera", name="Rear Camera"),
                row_shares=[1, 1],
            ),
            rrb.Spatial3DView(
                origin="world",
                name="M20 nav",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def m20_rerun_config() -> dict[str, Any]:
    """The real robot's view: navigation plus both cameras."""
    return {
        "pubsubs": [Zenoh()],
        "blueprint": _m20_rerun_blueprint,
        "tf_axes": 0.35,
        "max_hz": {
            "world/local_map": 0.5,
        },
        "visual_override": {
            **m20_nav_overrides(),
            "world/front_camera": _render_h265,
            "world/rear_camera": _render_h265,
            "world/front_camera_info": _render_front_camera_info,
            "world/rear_camera_info": _render_rear_camera_info,
        },
        "static": m20_nav_static(),
    }


def _scene_lines(scene: LineSegments3D) -> Archetype:
    from dimos.visualization.rerun.cdr import lines_archetype

    return lines_archetype(scene, radii=0.01)


def _sim_rerun_blueprint() -> Blueprint:
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


def m20_sim_rerun_config() -> dict[str, Any]:
    """The sim's view: navigation plus the scene's outlines."""
    return {
        "blueprint": _sim_rerun_blueprint,
        "tf_axes": 0.35,
        "static": m20_nav_static(),
        "visual_override": {**m20_nav_overrides(), "world/scene": _scene_lines},
    }
