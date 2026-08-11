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

"""Unitree G1 office watering demo: hardware GR00T WBC + manipulation + camera.

Hardware only. The GR00T WBC coordinator (policy at priority 50, arm
trajectories at 30 over the servo hold at 10), the manipulation module with
Viser off, and the head RealSense publishing JPEG color for the Rerun
stream. Comes up unarmed + dry-run with the base blueprint's 10 s
activation ramp.

The plant pose arrives on ``/object_pose`` from perception (the same message
sim publishes from ``SimBodyPose``); this blueprint does not publish it yet.

Driving is the viewer's own teleop, wired straight through: the viewer's
``tele_cmd_vel`` is remapped onto ``cmd_vel``, which is the coordinator's
``twist_command``. No MovementManager, no mux — the ControlCoordinator is the
only arbitration authority, and no panel runs on the robot.

No nav stack at all: no lidar, no voxel map, no costmap, no planner. Nothing
publishes odometry, so the Rerun robot mesh (rooted under ``world/odometry``)
sits at the origin — the camera and joint streams are what this demo shows.

Usage (on the G1; ``--rerun-open none`` because it has no display):
    dimos --rerun-open none --rerun-host 0.0.0.0 run unitree-g1-water-demo
Laptop viewer:
    dimos-viewer --connect rerun+http://<robot>:9877/proxy --ws-url ws://<robot>:3030/ws
"""

from __future__ import annotations

from typing import Any, cast

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _backend,
    _G1GrootCoordinator,
    _rerun_config,
    g1_groot_coordinator,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc_manip import (
    _ARM_TRAJECTORY_TASK,
    g1_manipulation,
)
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

if global_config.simulation:
    raise ValueError(
        "unitree-g1-water-demo is hardware-only; use unitree-g1-groot-wbc-manip for sim"
    )

# Both viewer surfaces drive the policy directly. The coordinator arbitrates;
# there is no mux in between.
_demo_remappings = [
    (_G1GrootCoordinator, "twist_command", "cmd_vel"),
    (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
    (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
]

_CAMERA_ENTITY = "world/color_compressed"


def _water_demo_rerun_blueprint() -> Any:
    """3D scene beside the head camera; the base layout is 3D-only."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                name="G1 water demo",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            rrb.Spatial2DView(origin=_CAMERA_ENTITY, name="Head camera"),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(state="collapsed"),
    )


# Camera caps are non-negotiable on the Jetson: uncapped image streams have
# eaten 20 GB of Rerun RAM before. coordinator_joint_state is the coordinator's
# full 29-joint aggregate at the 100 Hz tick — it saturates the viewer's gRPC
# channel on its own, and the per-robot /g1/joints stream already drives the mesh.
_demo_rerun_config: dict[str, Any] = {
    **_rerun_config,
    "blueprint": _water_demo_rerun_blueprint,
    "max_hz": {**_rerun_config["max_hz"], _CAMERA_ENTITY: 5.0},
    "visual_override": {
        **_rerun_config["visual_override"],
        "world/coordinator_joint_state": None,
    },
    "memory_limit": "5%",
}

unitree_g1_water_demo = (
    autoconnect(
        _backend,
        g1_groot_coordinator(extra_tasks=(_ARM_TRAJECTORY_TASK,)),
        # JPEG over the wire and straight into Rerun as an EncodedImage: the
        # robot never decodes, and raw color would cost ~18 MB/s of egress.
        RealSenseCamera.blueprint(enable_depth=False, enable_pointcloud=False, compress_color=True),
        g1_manipulation(visualization=ViserVisualizationConfig(host="0.0.0.0")),
        vis_module(viewer_backend=global_config.viewer, rerun_config=_demo_rerun_config),
    )
    .remappings(cast("Any", _demo_remappings))
    # One worker per module: the connection pump, the 100 Hz tick, the camera
    # capture loop and the Rerun bridge each need their own process.
    .global_config(robot_model="unitree_g1", n_workers=7)
)
