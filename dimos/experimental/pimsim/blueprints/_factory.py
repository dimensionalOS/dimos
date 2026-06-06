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

"""Blueprint factories for the pimsim sim + nav stack.

These are plain functions (no module-level blueprint construction), so importing
this module — e.g. from the cross-wall test or the demo harnesses — does NOT
resolve a scene or cook anything. The registered ``babylon-smoketest`` /
``babylon-nav`` blueprints (in their own modules) call these at import time;
that's the point at which a cooked scene is actually required.
"""

from __future__ import annotations

from importlib import resources
import os
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.experimental.pimsim.module import BabylonSceneViewerModule
from dimos.experimental.pimsim.odometry_adapter import (
    LIDAR_SENSOR_X,
    LIDAR_SENSOR_Z,
    OdomTfBroadcaster,
    PoseStampedToOdometry,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.simulation.scene_assets.cook import cook_scene_package
from dimos.simulation.scene_assets.mesh_scene import SceneMeshAlignment
from dimos.simulation.scene_assets.spec import BrowserVisualSpec, MujocoSceneSpec
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.simulation.sensors.scene_lidar import SceneLidarModule
from dimos.visualization.vis_module import vis_module

# Default scene is the cooked dimos-office; override with a scene name or a path
# to a scene.meta.json (e.g. an open floor for nav tests) via DIMOS_PIMSIM_SCENE.
DEFAULT_SCENE = os.getenv("DIMOS_PIMSIM_SCENE", "dimos-office")

# Box-proxy MJCF: a single free-jointed body with a couple of render geoms.
# Babylon owns the kinematic base in enable_sim mode, so this is purely what the
# viewer draws — no actuated joints, no STL meshes.
_PROXY_MJCF = str(resources.files("dimos.robot.unitree.go2").joinpath("go2_proxy.xml"))

# The 300+ MB visual GLB renders on CPU swiftshader in a headless browser and
# starves the physics step loop. Load it only when a human wants the viewer.
_LOAD_VISUAL = os.getenv("DIMOS_PIMSIM_VISUAL", "0").lower() in {"1", "true", "yes", "on"}

FLOOR_SCENE_DIR = Path.home() / ".cache" / "dimos" / "scene_packages" / "pimsim_flat_floor"
WAYPOINT_THRESHOLD_M = 0.6

# Initial rerun 3D camera framing the cross-wall scene (wall at y=2, route in
# x[-1,3]). Overridable via env for quick tuning. Z-up world frame.
_RERUN_EYE = os.getenv("DIMOS_RERUN_EYE", "2.0,-2.5,5.0")
_RERUN_TARGET = os.getenv("DIMOS_RERUN_TARGET", "1.0,2.0,0.2")


def build_babylon_sim(scene: str | None = None) -> Blueprint:
    """Box-proxy viewer + rust lidar on a cooked scene (name or scene.meta.json)."""
    package = resolve_scene_package(scene or DEFAULT_SCENE)
    if package is None or package.browser_collision_path is None:
        raise RuntimeError(
            f"babylon sim needs scene {scene or DEFAULT_SCENE!r} cooked with a "
            "browser collision mesh (python -m dimos.simulation.scene_assets.cook ...)."
        )

    viewer = BabylonSceneViewerModule.blueprint(
        mjcf_path=_PROXY_MJCF,
        scene_path=str(package.visual_path) if (_LOAD_VISUAL and package.visual_path) else None,
        browser_collision_path=str(package.browser_collision_path),
        scene_scale=package.alignment.scale,
        scene_translation=package.alignment.translation,
        scene_rotation_zyx_deg=package.alignment.rotation_zyx_deg,
        scene_y_up=package.alignment.y_up,
        initial_entities=package.entities,
        enable_sim=True,
        sim_rate=100.0,
        vehicle_height=0.40,
        step_offset=0.10,
        support_floor=True,
        lock_z=True,
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("entity_state_batch", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    )

    lidar = SceneLidarModule.blueprint(
        build_command="cargo build --release",
        scene_metadata_path=str(package.metadata_path),
        collision_path=str(package.browser_collision_path),
        scan_model="mid360",
        frame_id="lidar_link",
        hz=10.0,
        point_rate=200_000,
        horizontal_samples=720,
        vertical_samples=16,
        elevation_min_deg=-52.0,
        elevation_max_deg=52.0,
        min_range=0.1,
        max_range=40.0,
        sensor_x=LIDAR_SENSOR_X,
        sensor_z=LIDAR_SENSOR_Z,
        output_voxel_size=0.03,
        support_floor=True,
    ).transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("entity_states", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    )
    return autoconnect(viewer, lidar)


def _vec3(text: str) -> list[float]:
    return [float(part) for part in text.split(",")]


def _nav_rerun_blueprint() -> Any:
    """Rerun blueprint: framed 3D pointcloud view, panels collapsed."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[12, 14, 20]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            eye_controls=rrb.EyeControls3D(
                position=_vec3(_RERUN_EYE),
                look_target=_vec3(_RERUN_TARGET),
                eye_up=[0.0, 0.0, 1.0],
            ),
        ),
        # collapse_panels is the field _with_graph_tab preserves; it hides the
        # left/right/bottom panels for a clean full-window 3D pointcloud.
        collapse_panels=True,
    )


def ensure_flat_floor_scene() -> str:
    """Cook (once) a 40x40 m flat floor scene; return its scene.meta.json path."""
    meta = FLOOR_SCENE_DIR / "scene.meta.json"
    if meta.exists():
        return str(meta)
    import trimesh

    floor = trimesh.creation.box(extents=[40.0, 40.0, 0.1])
    floor.apply_translation([0.0, 0.0, -0.05])
    glb = FLOOR_SCENE_DIR.parent / "pimsim_flat_floor.glb"
    glb.parent.mkdir(parents=True, exist_ok=True)
    trimesh.Scene(floor).export(str(glb))
    package = cook_scene_package(
        glb,
        output_dir=FLOOR_SCENE_DIR,
        alignment=SceneMeshAlignment(scale=1.0, y_up=False),
        visual_spec=BrowserVisualSpec(optimizer="copy"),
        mujoco_spec=MujocoSceneSpec(enabled=False),
    )
    return str(package.metadata_path)


def build_babylon_nav(scene: str | None = None, *, with_vis: bool = False) -> Blueprint:
    """pimsim sim (open floor) + odom/TF adapters + FAR nav stack."""
    sim = build_babylon_sim(scene or ensure_flat_floor_scene())
    odom_adapter = PoseStampedToOdometry.blueprint().transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("odometry", Odometry): LCMTransport("/odometry", Odometry),
        }
    )
    tf_broadcaster = OdomTfBroadcaster.blueprint().transports(
        {("pose", PoseStamped): LCMTransport("/odom", PoseStamped)}
    )
    nav_stack = create_nav_stack(
        planner="simple",
        vehicle_height=0.40,
        max_speed=0.8,
        waypoint_threshold=WAYPOINT_THRESHOLD_M,
    ).transports({("registered_scan", PointCloud2): LCMTransport("/lidar", PointCloud2)})
    movement_manager = MovementManager.blueprint()
    parts = [sim, odom_adapter, tf_broadcaster, nav_stack, movement_manager]
    if with_vis:
        parts.append(
            vis_module(global_config.viewer, rerun_config={"blueprint": _nav_rerun_blueprint})
        )
    return (
        autoconnect(*parts)
        .remappings([(MovementManager, "way_point", "_mgr_way_point_unused")])
        .global_config(simulation=True)
    )
