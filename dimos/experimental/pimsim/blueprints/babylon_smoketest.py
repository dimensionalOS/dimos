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

"""Browser-physics-only nav smoketest. No MuJoCo, no robot connection.

Composes ``BabylonSceneViewerModule`` (Havok in the browser, integrated
kinematic base, WASD drive) with the rust ``SceneLidarModule`` against
the cooked ``dimos-office`` scene package. Robot pose comes from the
browser's kinematic integration; the rust lidar consumes that pose
plus the entity batch and publishes ``/lidar``.

Usage:
    dimos run babylon-smoketest

Then open http://localhost:8091/, click Drive, WASD to walk the robot
around, click Add to drop a Havok box. The robot pushes the box, and
the lidar pointcloud picks it up.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.experimental.pimsim.module import BabylonSceneViewerModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.simulation.sensors.scene_lidar import SceneLidarModule
from dimos.utils.data import LfsPath

# G1 mesh is reused only for the rendered robot in the viewer — no
# physics dependency on MuJoCo. Any URDF/MJCF the pimsim mesh loader
# understands would work here; G1 is what's bundled.
_G1_MJCF = LfsPath("mujoco_sim/g1_gear_wbc.xml")
_SCENE_NAME = "dimos-office"

_scene = resolve_scene_package(_SCENE_NAME)
if _scene is None or _scene.browser_collision_path is None:
    raise RuntimeError(
        f"babylon-smoketest needs the {_SCENE_NAME!r} scene package cooked. "
        "Run `python -m dimos.simulation.scene_assets.cook --source <path>` "
        "or pull the cached bundle into ~/.cache/dimos/scene_packages/."
    )

babylon_smoketest = autoconnect(
    BabylonSceneViewerModule.blueprint(
        mjcf_path=str(_G1_MJCF),
        scene_path=str(_scene.visual_path) if _scene.visual_path else None,
        browser_collision_path=str(_scene.browser_collision_path),
        scene_scale=_scene.alignment.scale,
        scene_translation=_scene.alignment.translation,
        scene_rotation_zyx_deg=_scene.alignment.rotation_zyx_deg,
        scene_y_up=_scene.alignment.y_up,
        enable_sim=True,
        sim_rate=100.0,
        vehicle_height=0.75,
        init_x=0.0,
        init_y=0.0,
        init_yaw=0.0,
        lock_z=True,
    ).transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("sim_odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("entity_state_batch", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    ),
    SceneLidarModule.blueprint(
        build_command="cargo build --release",
        scene_metadata_path=str(_scene.metadata_path),
        collision_path=str(_scene.browser_collision_path),
        hz=10.0,
        horizontal_samples=720,
        vertical_samples=16,
        max_range=10.0,
        sensor_z=1.0,
        output_voxel_size=0.03,
    ).transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("entity_states", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    ),
)

__all__ = ["babylon_smoketest"]
