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

"""Minimal browser-physics nav smoketest. No MuJoCo runtime, no robot.

The smallest useful pimsim: a box-proxy robot whose kinematic base is
integrated in the browser from incoming velocity commands, plus the
rust ``SceneLidarModule`` raycasting the cooked ``dimos-office``
collision mesh (and any dynamic entities). It is deliberately stripped
of whole-body control, camera streaming, and policy sims.

Topic flow (everything bus-level rides the browser ``/lcm-ws`` bridge):
- ``/nav_cmd_vel`` (Twist) in -> browser integrates it into the base
- ``/odom`` (PoseStamped) out <- browser publishes integrated base pose
- ``/lidar`` (PointCloud2) out <- rust raycaster, posed by ``/odom``
- ``/entity_state_batch`` <-> dynamic obstacles, folded into the lidar

Usage::

    dimos run babylon-smoketest

Then open http://localhost:8091/ (or drive headless via
``dimos.experimental.pimsim.headless.HeadlessBrowser`` +
``dimos.experimental.pimsim.client.PimSimClient``).
"""

from __future__ import annotations

from importlib import resources
import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.experimental.pimsim.module import BabylonSceneViewerModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.simulation.sensors.scene_lidar import SceneLidarModule

_SCENE_NAME = "dimos-office"

# Box-proxy MJCF: a single free-jointed body with a couple of render
# geoms. Babylon owns the kinematic base in enable_sim mode, so this is
# purely what the viewer draws — no actuated joints, no STL meshes.
_PROXY_MJCF = str(resources.files("dimos.robot.unitree.go2").joinpath("go2_proxy.xml"))

# The 300+ MB visual GLB renders on CPU swiftshader in a headless browser and
# starves the physics step loop. Load it only when a human wants the viewer.
_LOAD_VISUAL = os.getenv("DIMOS_PIMSIM_VISUAL", "0").lower() in {"1", "true", "yes", "on"}

_scene = resolve_scene_package(_SCENE_NAME)
if _scene is None or _scene.browser_collision_path is None:
    raise RuntimeError(
        f"babylon-smoketest needs the {_SCENE_NAME!r} scene cooked with a "
        "browser collision mesh. Cook it with:\n"
        "  python -m dimos.simulation.scene_assets.cook "
        "data/dimos_office_mesh/dimos_office_mesh.glb "
        "--output-dir ~/.cache/dimos/scene_packages/dimos_office --no-y-up "
        "--cook-spec data/dimos_office_mesh/dimos_office_mesh.cook.json "
        "--visual-optimizer blender --no-mujoco"
    )

_viewer = BabylonSceneViewerModule.blueprint(
    mjcf_path=_PROXY_MJCF,
    scene_path=str(_scene.visual_path) if (_LOAD_VISUAL and _scene.visual_path) else None,
    browser_collision_path=str(_scene.browser_collision_path),
    scene_scale=_scene.alignment.scale,
    scene_translation=_scene.alignment.translation,
    scene_rotation_zyx_deg=_scene.alignment.rotation_zyx_deg,
    scene_y_up=_scene.alignment.y_up,
    initial_entities=_scene.entities,
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

_lidar = SceneLidarModule.blueprint(
    build_command="cargo build --release",
    scene_metadata_path=str(_scene.metadata_path),
    collision_path=str(_scene.browser_collision_path),
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
    sensor_x=0.15,
    sensor_z=0.10,
    output_voxel_size=0.03,
    support_floor=True,
).transports(
    {
        ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
        ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("entity_states", EntityStateBatch): LCMTransport("/entity_state_batch", EntityStateBatch),
    }
)

babylon_smoketest = autoconnect(_viewer, _lidar)

__all__ = ["babylon_smoketest"]
