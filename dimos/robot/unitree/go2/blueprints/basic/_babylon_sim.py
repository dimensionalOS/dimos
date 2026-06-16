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

"""Babylon / pimsim viewer composition for the Go2.

Mirrors the G1 ``_babylon_blueprint`` helper in shape: the viewer
integrates ``/cmd_vel`` locally, publishes ``sim_odom`` as the
canonical ``/odom`` for downstream consumers, and (when a scene
package is configured) overlays the cooked visual GLB + browser
collision mesh. Compared to G1 we use a small box proxy MJCF — Go2's
real-hardware low-level loop runs on Unitree firmware via SPORT_MOD,
so the simulator only needs a kinematic base to track.
"""

from __future__ import annotations

from importlib import resources
import importlib.util
import os
from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_GO2_PROXY_MJCF = resources.files("dimos.robot.unitree.go2").joinpath("go2_proxy.xml")
_DEFAULT_BABYLON_PORT = 8091

# Go2 L1 LiDAR is mounted on the head module, ~15 cm forward and ~10 cm
# above the trunk center. The menagerie XML names the trunk "base"; sensor
# offsets here are relative to that body, since /odom tracks the trunk pose.
_GO2_LIDAR_FRAME_ID = "lidar_link"
_GO2_LIDAR_SENSOR_X_M = 0.15
_GO2_LIDAR_SENSOR_Y_M = 0.0
_GO2_LIDAR_SENSOR_Z_M = 0.10
_GO2_LIDAR_SENSOR_ROLL_DEG = 0.0
_GO2_LIDAR_SENSOR_PITCH_DEG = 0.0
_GO2_LIDAR_SENSOR_YAW_DEG = 0.0
# Use Mid-360-style scan as a stand-in; the real Go2 ships a Livox L1,
# but the rust raycaster's scan_model just picks azimuth/elevation
# distributions and the mid360 pattern is dense enough for nav.
_GO2_LIDAR_SCAN_MODEL = "mid360"
_GO2_LIDAR_POINT_RATE = 200_000
_GO2_LIDAR_ELEVATION_MIN_DEG = -52.0
_GO2_LIDAR_ELEVATION_MAX_DEG = 52.0
_GO2_LIDAR_MIN_RANGE_M = 0.1
_GO2_LIDAR_MAX_RANGE_M = 40.0


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    return default if raw is None else int(raw)


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    return default if raw is None else float(raw)


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.lower() in {"1", "true", "yes", "on"}


def _babylon_enabled() -> bool:
    return _env_bool("DIMOS_ENABLE_BABYLON", True)


def _go2_menagerie_mjcf() -> Path | None:
    spec = importlib.util.find_spec("mujoco_playground")
    if spec is None or not spec.submodule_search_locations:
        return None
    root = Path(next(iter(spec.submodule_search_locations)))
    path = root / "external_deps" / "mujoco_menagerie" / "unitree_go2" / "go2.xml"
    return path if path.exists() else None


def _viewer_mjcf_path() -> str:
    override = os.getenv("DIMOS_GO2_VIEWER_MJCF")
    if override:
        return override

    menagerie = _go2_menagerie_mjcf()
    if menagerie is not None:
        return str(menagerie)

    logger.warning("Go2 babylon: bundled menagerie model not found; using box proxy")
    with resources.as_file(_GO2_PROXY_MJCF) as proxy_path:
        return str(proxy_path)


def _scene_package() -> Any:
    """Load the cooked scene package if one is configured, else ``None``."""
    scene = getattr(global_config, "scene", "")
    if not scene:
        return None
    # Reuse the same catalog as the G1 path. The Go2 doesn't care about
    # the MuJoCo wrapper artifact (Babylon owns physics in pimsim mode),
    # so robot_mjcf_path stays None — same cooked package serves both.
    from dimos.simulation.scenes.catalog import resolve_scene_package

    try:
        return resolve_scene_package(scene, robot_mjcf_path=None, meshdir=None)
    except (FileNotFoundError, ValueError) as exc:
        logger.warning("Go2 babylon: cannot load scene %r: %s", scene, exc)
        return None


def go2_babylon_blueprint() -> Blueprint | None:
    """Build the BabylonSceneViewerModule blueprint for the Go2 in pimsim mode.

    Returns ``None`` when ``DIMOS_ENABLE_BABYLON=0``.  Caller is
    responsible for only invoking this when
    ``global_config.simulation in ("babylon", "pimsim")``.
    """
    if not _babylon_enabled():
        return None

    from dimos.experimental.pimsim.babylon.module import BabylonSceneViewerModule

    kwargs: dict[str, Any] = dict(
        mjcf_path=_viewer_mjcf_path(),
        port=_env_int("DIMOS_BABYLON_PORT", _DEFAULT_BABYLON_PORT),
        # Babylon integrates cmd_vel directly — no MuJoCo at runtime.
        enable_sim=True,
        sim_rate=_env_float("DIMOS_BABYLON_SIM_RATE_HZ", 100.0),
        vehicle_height=_env_float("DIMOS_GO2_VEHICLE_HEIGHT", 0.40),
        step_offset=_env_float("DIMOS_BABYLON_STEP_OFFSET", 0.10),
        support_floor=_env_bool("DIMOS_BABYLON_SUPPORT_FLOOR", True),
        support_floor_z=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_Z", 0.0),
        support_floor_size=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_SIZE", 0.0),
        lock_z=True,
    )

    scene_package = _scene_package()
    if scene_package is not None and scene_package.visual_path is not None:
        browser_collision_path = (
            str(scene_package.browser_collision_path)
            if scene_package.browser_collision_path is not None
            else None
        )
        kwargs.update(
            scene_path=str(scene_package.visual_path),
            scene_scale=scene_package.alignment.scale,
            scene_translation=scene_package.alignment.translation,
            scene_rotation_zyx_deg=scene_package.alignment.rotation_zyx_deg,
            scene_y_up=scene_package.alignment.y_up,
            browser_collision_path=browser_collision_path,
            initial_entities=scene_package.entities,
        )

    return BabylonSceneViewerModule.blueprint(**kwargs).transports(
        {
            # /joint_state and /odom are still consumed server-side because FK
            # runs in Python (and emits the binary robot_pose frame). Browser
            # also subscribes via /lcm-ws for HUD sliders. When FK moves to
            # the browser, these two go too.
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            # entity_state_batch is still server-published from JSON entity_states
            # multi-tab replay until the entity world migrates to the bridge.
            ("entity_state_batch", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    )


def go2_sim_mapping_blueprint() -> Blueprint | None:
    """Voxel→global_map mapper that converts /lidar into /global_map.

    Lives in the sim path because real-hardware Go2 already has its own
    mapping chain via the higher-level ``unitree_go2`` blueprint. In
    ``unitree-go2-basic --simulation pimsim`` (the headless / smoke
    path) we still want a populated ``BabylonSceneViewerModule.point\
    cloud_overlay`` so the user sees what the raycaster is doing.

    Returns ``None`` if lidar is disabled — /lidar wouldn't have any
    publisher anyway.
    """
    if _env_bool("DIMOS_DISABLE_LIDAR", False):
        return None
    scene_package = _scene_package()
    if scene_package is None or scene_package.browser_collision_path is None:
        return None

    from dimos.mapping.voxels import VoxelGridMapper

    voxel_size = _env_float("DIMOS_GLOBAL_MAP_VOXEL_SIZE", 0.05)
    return VoxelGridMapper.blueprint(voxel_size=voxel_size, emit_every=5).transports(
        {
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_map", PointCloud2): LCMTransport("/global_map", PointCloud2),
        }
    )


def go2_scene_lidar_blueprint() -> Blueprint | None:
    """Wire the rust SceneLidarModule when a cooked scene is configured.

    Real Go2 hardware publishes ``/lidar`` from the onboard L1 over
    WebRTC; ``PimSimConnection.lidar_stream()`` is an empty ``Subject``
    by design. So in pimsim mode the rust raycaster against the cooked
    browser collision mesh becomes the publisher. Returns ``None`` when
    lidar is explicitly disabled, no scene is cooked, or the rust
    binary isn't available.
    """
    if _env_bool("DIMOS_DISABLE_LIDAR", False):
        return None
    scene_package = _scene_package()
    if scene_package is None or scene_package.browser_collision_path is None:
        return None
    if not _env_bool("DIMOS_ENABLE_NATIVE_SCENE_LIDAR", True):
        return None

    from dimos.experimental.pimsim.sensors.scene_lidar import SceneLidarModule

    return SceneLidarModule.blueprint(
        scene_metadata_path=str(scene_package.metadata_path),
        collision_path=str(scene_package.browser_collision_path),
        scan_model=os.environ.get("DIMOS_SCENE_LIDAR_SCAN_MODEL", _GO2_LIDAR_SCAN_MODEL),
        frame_id=os.environ.get("DIMOS_SCENE_LIDAR_FRAME_ID", _GO2_LIDAR_FRAME_ID),
        hz=_env_float("DIMOS_SCENE_LIDAR_HZ", 10.0),
        point_rate=_env_int("DIMOS_SCENE_LIDAR_POINT_RATE", _GO2_LIDAR_POINT_RATE),
        horizontal_samples=_env_int("DIMOS_SCENE_LIDAR_HORIZONTAL_SAMPLES", 720),
        vertical_samples=_env_int("DIMOS_SCENE_LIDAR_VERTICAL_SAMPLES", 16),
        elevation_min_deg=_env_float(
            "DIMOS_SCENE_LIDAR_ELEVATION_MIN_DEG", _GO2_LIDAR_ELEVATION_MIN_DEG
        ),
        elevation_max_deg=_env_float(
            "DIMOS_SCENE_LIDAR_ELEVATION_MAX_DEG", _GO2_LIDAR_ELEVATION_MAX_DEG
        ),
        min_range=_env_float("DIMOS_SCENE_LIDAR_MIN_RANGE", _GO2_LIDAR_MIN_RANGE_M),
        max_range=_env_float("DIMOS_SCENE_LIDAR_MAX_RANGE", _GO2_LIDAR_MAX_RANGE_M),
        sensor_x=_env_float("DIMOS_SCENE_LIDAR_SENSOR_X", _GO2_LIDAR_SENSOR_X_M),
        sensor_y=_env_float("DIMOS_SCENE_LIDAR_SENSOR_Y", _GO2_LIDAR_SENSOR_Y_M),
        sensor_z=_env_float("DIMOS_SCENE_LIDAR_SENSOR_Z", _GO2_LIDAR_SENSOR_Z_M),
        sensor_roll_deg=_env_float("DIMOS_SCENE_LIDAR_SENSOR_ROLL_DEG", _GO2_LIDAR_SENSOR_ROLL_DEG),
        sensor_pitch_deg=_env_float(
            "DIMOS_SCENE_LIDAR_SENSOR_PITCH_DEG", _GO2_LIDAR_SENSOR_PITCH_DEG
        ),
        sensor_yaw_deg=_env_float("DIMOS_SCENE_LIDAR_SENSOR_YAW_DEG", _GO2_LIDAR_SENSOR_YAW_DEG),
        yaw_offset_deg=_env_float("DIMOS_SCENE_LIDAR_YAW_OFFSET_DEG", 0.0),
        output_voxel_size=_env_float("DIMOS_SCENE_LIDAR_OUTPUT_VOXEL_SIZE", 0.03),
        support_floor=_env_bool("DIMOS_SCENE_LIDAR_SUPPORT_FLOOR", True),
        support_floor_z=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_Z", 0.0),
        support_floor_size=_env_float("DIMOS_SCENE_SUPPORT_FLOOR_SIZE", 0.0),
    ).transports(
        {
            ("pose", PoseStamped): LCMTransport("/odom", PoseStamped),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
            # Picks up dynamic entities (Add-button spawns / wall RPCs) so
            # the lidar pointcloud folds them into per-ray intersections.
            ("entity_states", EntityStateBatch): LCMTransport(
                "/entity_state_batch", EntityStateBatch
            ),
        }
    )


__all__ = [
    "go2_babylon_blueprint",
    "go2_scene_lidar_blueprint",
    "go2_sim_mapping_blueprint",
]
