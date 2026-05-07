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

from __future__ import annotations

from collections import deque
import math
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Grid constants matching the original C++
TERRAIN_VOXEL_WIDTH = 41
TERRAIN_VOXEL_HALF_WIDTH = (TERRAIN_VOXEL_WIDTH - 1) // 2  # 20
TERRAIN_VOXEL_NUM = TERRAIN_VOXEL_WIDTH * TERRAIN_VOXEL_WIDTH

PLANAR_VOXEL_WIDTH = 101
PLANAR_VOXEL_HALF_WIDTH = (PLANAR_VOXEL_WIDTH - 1) // 2  # 50
PLANAR_VOXEL_NUM = PLANAR_VOXEL_WIDTH * PLANAR_VOXEL_WIDTH


class TerrainMapExtConfig(ModuleConfig):
    world_frame: str = "map"

    # Scan voxel size for downsampling (PCL VoxelGrid leaf size equivalent)
    scan_voxel_size: float = 0.1

    # Decay time for accumulated points (seconds; C++ default 10.0, launch override 4.0)
    decay_time: float = 4.0

    # Points within this distance never decay
    no_decay_distance: float = 0.0

    # Distance for manual clearing
    clearing_distance: float = 30.0

    # Ground estimation
    use_sorting: bool = False
    quantile_z: float = 0.25

    # Vehicle dimensions
    vehicle_height: float = 1.5

    # Height bounds relative to vehicle z, with distance-scaled expansion
    lower_bound_z: float = -1.5
    upper_bound_z: float = 1.0
    distance_ratio_z: float = 0.1

    # Voxel update thresholds (triggers downsample pass)
    voxel_point_update_threshold: int = 100
    voxel_time_update_threshold: float = 2.0

    # Terrain connectivity BFS
    check_terrain_connectivity: bool = True
    terrain_under_vehicle: float = -0.75
    terrain_connectivity_threshold: float = 0.5
    ceiling_filtering_threshold: float = 2.0

    # Local terrain map merge radius (meters)
    local_terrain_map_radius: float = 4.0

    # Merge local terrain_map points within local_terrain_map_radius into output.
    # The original C++ DOES this (lines 542-551 of terrainAnalysisExt.cpp).
    # Set to False to only publish points beyond the radius.
    merge_local_terrain: bool = True

    # Terrain voxel size (rolling grid cell size in meters)
    terrain_voxel_size: float = 2.0

    # Planar voxel size (ground estimation grid cell size in meters)
    planar_voxel_size: float = 0.4


def _voxel_index(x: float, y: float, voxel_size: float, half_width: int) -> int:
    """Compute voxel grid index matching the C++ int-cast-with-negative-correction."""
    offset = x - y + voxel_size / 2
    ind = int(offset / voxel_size) + half_width
    if offset < 0:
        ind -= 1
    return ind


class TerrainMapExt(Module):
    """Extended terrain map: accumulates local terrain into a wider, slower-decaying map.

    Subscribes to registered_scan directly (parallel to TerrainAnalysis),
    estimates ground elevation per planar voxel column, runs BFS connectivity
    to reject ceiling/overhead structures, and publishes filtered terrain.
    """

    config: TerrainMapExtConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: In[PointCloud2]
    terrain_map_ext: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Vehicle state
        self._vehicle_x = 0.0
        self._vehicle_y = 0.0
        self._vehicle_z = 0.0

        # System time tracking (matching C++ systemInitTime pattern)
        self._system_init_time = 0.0
        self._system_inited = False
        self._laser_cloud_time = 0.0

        # Rolling terrain voxel grid: list of point arrays (x, y, z, intensity)
        # intensity stores (laserCloudTime - systemInitTime) for decay
        self._terrain_voxel_cloud: list[list[list[float]]] = [[] for _ in range(TERRAIN_VOXEL_NUM)]
        self._terrain_voxel_update_num = [0] * TERRAIN_VOXEL_NUM
        self._terrain_voxel_update_time = [0.0] * TERRAIN_VOXEL_NUM

        # Grid shift tracking for rolling
        self._terrain_voxel_shift_x = 0
        self._terrain_voxel_shift_y = 0

        # Pending scan data
        self._new_laser_cloud = False
        self._laser_cloud_crop: list[list[float]] = []

        # Local terrain map (from TerrainAnalysis, for merge within radius)
        self._terrain_cloud_local: np.ndarray = np.zeros((0, 3), dtype=np.float32)

        # Clearing state
        self._clearing_cloud = False

    @rpc
    def start(self) -> None:
        super().start()
        self._terrain_voxel_cloud = [[] for _ in range(TERRAIN_VOXEL_NUM)]
        self._terrain_voxel_update_num = [0] * TERRAIN_VOXEL_NUM
        self._terrain_voxel_update_time = [0.0] * TERRAIN_VOXEL_NUM
        self._terrain_voxel_shift_x = 0
        self._terrain_voxel_shift_y = 0
        self._system_inited = False
        self._new_laser_cloud = False
        self._clearing_cloud = False

        self.register_disposable(Disposable(self.registered_scan.subscribe(self._on_scan)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_local_terrain)))

        self._running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._vehicle_x = msg.pose.position.x
            self._vehicle_y = msg.pose.position.y
            self._vehicle_z = msg.pose.position.z

    def _on_local_terrain(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        with self._lock:
            self._terrain_cloud_local = (
                points[:, :3].copy() if len(points) > 0 else np.zeros((0, 3), dtype=np.float32)
            )

    def _on_scan(self, cloud: PointCloud2) -> None:
        """Handle incoming registered_scan: crop by height bounds and store for processing."""
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        laser_cloud_time = cloud.ts if cloud.ts is not None else time.time()

        with self._lock:
            if not self._system_inited:
                self._system_init_time = laser_cloud_time
                self._system_inited = True

            self._laser_cloud_time = laser_cloud_time

            vehicle_x = self._vehicle_x
            vehicle_y = self._vehicle_y
            vehicle_z = self._vehicle_z

        config = self.config
        terrain_voxel_size = config.terrain_voxel_size
        max_range = terrain_voxel_size * (TERRAIN_VOXEL_HALF_WIDTH + 1)

        # Crop points by height bounds (matching laserCloudHandler in C++)
        cropped: list[list[float]] = []
        time_offset = laser_cloud_time - self._system_init_time

        for i in range(len(points)):
            point_x = float(points[i, 0])
            point_y = float(points[i, 1])
            point_z = float(points[i, 2])

            distance = math.sqrt((point_x - vehicle_x) ** 2 + (point_y - vehicle_y) ** 2)

            relative_z = point_z - vehicle_z
            lower = config.lower_bound_z - config.distance_ratio_z * distance
            upper = config.upper_bound_z + config.distance_ratio_z * distance

            if relative_z > lower and relative_z < upper and distance < max_range:
                cropped.append([point_x, point_y, point_z, time_offset])

        with self._lock:
            self._laser_cloud_crop = cropped
            self._new_laser_cloud = True

    def _process_loop(self) -> None:
        """Main processing loop — runs at ~100Hz, processes when new scan arrives."""
        while self._running:
            with self._lock:
                has_new = self._new_laser_cloud
            if not has_new:
                time.sleep(0.01)
                continue

            with self._lock:
                self._new_laser_cloud = False
                cropped = self._laser_cloud_crop
                self._laser_cloud_crop = []
                vehicle_x = self._vehicle_x
                vehicle_y = self._vehicle_y
                vehicle_z = self._vehicle_z
                laser_cloud_time = self._laser_cloud_time
                local_terrain = self._terrain_cloud_local.copy()
                clearing_cloud = self._clearing_cloud

            config = self.config
            terrain_voxel_size = config.terrain_voxel_size
            time_offset = laser_cloud_time - self._system_init_time

            # --- Terrain voxel rollover ---
            self._roll_terrain_grid(vehicle_x, vehicle_y, terrain_voxel_size)

            # --- Stack cropped scan into terrain voxels ---
            for point in cropped:
                ind_x = _voxel_index(
                    point[0], vehicle_x, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH
                )
                ind_y = _voxel_index(
                    point[1], vehicle_y, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH
                )

                if 0 <= ind_x < TERRAIN_VOXEL_WIDTH and 0 <= ind_y < TERRAIN_VOXEL_WIDTH:
                    flat_idx = TERRAIN_VOXEL_WIDTH * ind_x + ind_y
                    self._terrain_voxel_cloud[flat_idx].append(point)
                    self._terrain_voxel_update_num[flat_idx] += 1

            # --- Downsample / evict voxels that exceed thresholds ---
            for ind in range(TERRAIN_VOXEL_NUM):
                if (
                    self._terrain_voxel_update_num[ind] >= config.voxel_point_update_threshold
                    or time_offset - self._terrain_voxel_update_time[ind]
                    >= config.voxel_time_update_threshold
                    or clearing_cloud
                ):
                    cell_points = self._terrain_voxel_cloud[ind]
                    if not cell_points:
                        self._terrain_voxel_update_num[ind] = 0
                        self._terrain_voxel_update_time[ind] = time_offset
                        continue

                    # Voxel grid downsample
                    downsampled = self._voxel_downsample(cell_points, config.scan_voxel_size)

                    # Re-filter: height bounds, decay, clearing
                    filtered: list[list[float]] = []
                    for point in downsampled:
                        distance = math.sqrt(
                            (point[0] - vehicle_x) ** 2 + (point[1] - vehicle_y) ** 2
                        )
                        relative_z = point[2] - vehicle_z
                        lower = config.lower_bound_z - config.distance_ratio_z * distance
                        upper = config.upper_bound_z + config.distance_ratio_z * distance
                        point_age = time_offset - point[3]

                        in_height = relative_z > lower and relative_z < upper
                        in_time = (
                            point_age < config.decay_time or distance < config.no_decay_distance
                        )
                        not_cleared = not (distance < config.clearing_distance and clearing_cloud)

                        if in_height and in_time and not_cleared:
                            filtered.append(point)

                    self._terrain_voxel_cloud[ind] = filtered
                    self._terrain_voxel_update_num[ind] = 0
                    self._terrain_voxel_update_time[ind] = time_offset

            # --- Gather terrain cloud from central 21x21 cells ---
            terrain_cloud: list[list[float]] = []
            for ind_x in range(TERRAIN_VOXEL_HALF_WIDTH - 10, TERRAIN_VOXEL_HALF_WIDTH + 11):
                for ind_y in range(TERRAIN_VOXEL_HALF_WIDTH - 10, TERRAIN_VOXEL_HALF_WIDTH + 11):
                    terrain_cloud.extend(
                        self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y]
                    )

            # --- Ground elevation estimation on planar grid ---
            planar_voxel_elev = [0.0] * PLANAR_VOXEL_NUM
            planar_point_elev: list[list[float]] = [[] for _ in range(PLANAR_VOXEL_NUM)]
            planar_voxel_conn = [0] * PLANAR_VOXEL_NUM

            planar_voxel_size = config.planar_voxel_size

            for point in terrain_cloud:
                distance = math.sqrt((point[0] - vehicle_x) ** 2 + (point[1] - vehicle_y) ** 2)
                relative_z = point[2] - vehicle_z
                lower = config.lower_bound_z - config.distance_ratio_z * distance
                upper = config.upper_bound_z + config.distance_ratio_z * distance

                if relative_z > lower and relative_z < upper:
                    ind_x = _voxel_index(
                        point[0], vehicle_x, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
                    )
                    ind_y = _voxel_index(
                        point[1], vehicle_y, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
                    )

                    # Spread to 3x3 neighborhood
                    for delta_x in range(-1, 2):
                        for delta_y in range(-1, 2):
                            nx = ind_x + delta_x
                            ny = ind_y + delta_y
                            if 0 <= nx < PLANAR_VOXEL_WIDTH and 0 <= ny < PLANAR_VOXEL_WIDTH:
                                planar_point_elev[PLANAR_VOXEL_WIDTH * nx + ny].append(point[2])

            # Estimate ground elevation per planar voxel
            if config.use_sorting:
                for i in range(PLANAR_VOXEL_NUM):
                    elevations = planar_point_elev[i]
                    if elevations:
                        elevations.sort()
                        quantile_id = int(config.quantile_z * len(elevations))
                        quantile_id = max(0, min(quantile_id, len(elevations) - 1))
                        planar_voxel_elev[i] = elevations[quantile_id]
            else:
                for i in range(PLANAR_VOXEL_NUM):
                    elevations = planar_point_elev[i]
                    if elevations:
                        planar_voxel_elev[i] = min(elevations)

            # --- BFS terrain connectivity check ---
            if config.check_terrain_connectivity:
                center_ind = PLANAR_VOXEL_WIDTH * PLANAR_VOXEL_HALF_WIDTH + PLANAR_VOXEL_HALF_WIDTH
                if not planar_point_elev[center_ind]:
                    planar_voxel_elev[center_ind] = vehicle_z + config.terrain_under_vehicle

                queue: deque[int] = deque()
                queue.append(center_ind)
                planar_voxel_conn[center_ind] = 1

                while queue:
                    front = queue.popleft()
                    planar_voxel_conn[front] = 2

                    front_x = front // PLANAR_VOXEL_WIDTH
                    front_y = front % PLANAR_VOXEL_WIDTH

                    for delta_x in range(-10, 11):
                        for delta_y in range(-10, 11):
                            nx = front_x + delta_x
                            ny = front_y + delta_y
                            if 0 <= nx < PLANAR_VOXEL_WIDTH and 0 <= ny < PLANAR_VOXEL_WIDTH:
                                neighbor_ind = PLANAR_VOXEL_WIDTH * nx + ny
                                if (
                                    planar_voxel_conn[neighbor_ind] == 0
                                    and planar_point_elev[neighbor_ind]
                                ):
                                    elev_diff = abs(
                                        planar_voxel_elev[front] - planar_voxel_elev[neighbor_ind]
                                    )
                                    if elev_diff < config.terrain_connectivity_threshold:
                                        queue.append(neighbor_ind)
                                        planar_voxel_conn[neighbor_ind] = 1
                                    elif elev_diff > config.ceiling_filtering_threshold:
                                        planar_voxel_conn[neighbor_ind] = -1

            # --- Build output: points beyond local radius with ground/connectivity filter ---
            output_points: list[list[float]] = []

            for point in terrain_cloud:
                distance = math.sqrt((point[0] - vehicle_x) ** 2 + (point[1] - vehicle_y) ** 2)
                relative_z = point[2] - vehicle_z
                lower = config.lower_bound_z - config.distance_ratio_z * distance
                upper = config.upper_bound_z + config.distance_ratio_z * distance

                if (
                    relative_z > lower
                    and relative_z < upper
                    and distance > config.local_terrain_map_radius
                ):
                    ind_x = _voxel_index(
                        point[0], vehicle_x, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
                    )
                    ind_y = _voxel_index(
                        point[1], vehicle_y, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
                    )

                    if 0 <= ind_x < PLANAR_VOXEL_WIDTH and 0 <= ind_y < PLANAR_VOXEL_WIDTH:
                        flat_ind = PLANAR_VOXEL_WIDTH * ind_x + ind_y
                        elevation_distance = abs(point[2] - planar_voxel_elev[flat_ind])
                        connected = planar_voxel_conn[flat_ind] == 2
                        if elevation_distance < config.vehicle_height and (
                            connected or not config.check_terrain_connectivity
                        ):
                            # intensity = elevation distance from ground
                            output_points.append([point[0], point[1], point[2], elevation_distance])

            # --- Merge local terrain map within localTerrainMapRadius ---
            # NOTE: The original C++ does NOT do this — it only publishes points
            # beyond the radius. Controlled by merge_local_terrain config flag.
            if config.merge_local_terrain:
                for i in range(len(local_terrain)):
                    point_x = float(local_terrain[i, 0])
                    point_y = float(local_terrain[i, 1])
                    point_z = float(local_terrain[i, 2])
                    distance = math.sqrt((point_x - vehicle_x) ** 2 + (point_y - vehicle_y) ** 2)
                    if distance <= config.local_terrain_map_radius:
                        output_points.append([point_x, point_y, point_z, 0.0])

            with self._lock:
                self._clearing_cloud = False

            # --- Publish with intensity (elevation distance from ground) ---
            if output_points:
                arr = np.array(output_points, dtype=np.float32)
                self.terrain_map_ext.publish(
                    PointCloud2.from_numpy(
                        arr[:, :3],
                        frame_id=config.world_frame,
                        timestamp=laser_cloud_time,
                        intensities=arr[:, 3],
                    )
                )

    def _roll_terrain_grid(
        self, vehicle_x: float, vehicle_y: float, terrain_voxel_size: float
    ) -> None:
        """Roll the terrain voxel grid to keep it centered on the vehicle."""
        terrain_voxel_cen_x = terrain_voxel_size * self._terrain_voxel_shift_x
        terrain_voxel_cen_y = terrain_voxel_size * self._terrain_voxel_shift_y

        # Roll in -X direction
        # NOTE: The C++ does NOT shift terrainVoxelUpdateNum/Time during rollover.
        # Only the point cloud pointers are shifted. Counters stay at their indices.
        while vehicle_x - terrain_voxel_cen_x < -terrain_voxel_size:
            for ind_y in range(TERRAIN_VOXEL_WIDTH):
                for ind_x in range(TERRAIN_VOXEL_WIDTH - 1, 0, -1):
                    self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * (ind_x - 1) + ind_y]
                    )
                self._terrain_voxel_cloud[ind_y] = []
            self._terrain_voxel_shift_x -= 1
            terrain_voxel_cen_x = terrain_voxel_size * self._terrain_voxel_shift_x

        # Roll in +X direction
        while vehicle_x - terrain_voxel_cen_x > terrain_voxel_size:
            for ind_y in range(TERRAIN_VOXEL_WIDTH):
                for ind_x in range(TERRAIN_VOXEL_WIDTH - 1):
                    self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * (ind_x + 1) + ind_y]
                    )
                self._terrain_voxel_cloud[
                    TERRAIN_VOXEL_WIDTH * (TERRAIN_VOXEL_WIDTH - 1) + ind_y
                ] = []
            self._terrain_voxel_shift_x += 1
            terrain_voxel_cen_x = terrain_voxel_size * self._terrain_voxel_shift_x

        # Roll in -Y direction
        while vehicle_y - terrain_voxel_cen_y < -terrain_voxel_size:
            for ind_x in range(TERRAIN_VOXEL_WIDTH):
                for ind_y in range(TERRAIN_VOXEL_WIDTH - 1, 0, -1):
                    self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + (ind_y - 1)]
                    )
                self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x] = []
            self._terrain_voxel_shift_y -= 1
            terrain_voxel_cen_y = terrain_voxel_size * self._terrain_voxel_shift_y

        # Roll in +Y direction
        while vehicle_y - terrain_voxel_cen_y > terrain_voxel_size:
            for ind_x in range(TERRAIN_VOXEL_WIDTH):
                for ind_y in range(TERRAIN_VOXEL_WIDTH - 1):
                    self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self._terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + (ind_y + 1)]
                    )
                self._terrain_voxel_cloud[
                    TERRAIN_VOXEL_WIDTH * ind_x + (TERRAIN_VOXEL_WIDTH - 1)
                ] = []
            self._terrain_voxel_shift_y += 1
            terrain_voxel_cen_y = terrain_voxel_size * self._terrain_voxel_shift_y

    @staticmethod
    def _voxel_downsample(points: list[list[float]], voxel_size: float) -> list[list[float]]:
        """Voxel grid downsampling: one point per 3D cell, keeping newest timestamp.

        For each occupied voxel cell, keeps the point with the most recent timestamp
        (element [3]). This prevents the decay filter from prematurely evicting points
        that share a cell with newer observations — matching PCL VoxelGrid's centroid
        behavior where averaged timestamps are younger than the oldest point.

        Uses absolute grid binning. Preserves original point positions (no centroid
        shift) to avoid changing planar voxel assignments in ground estimation.
        """
        if not points:
            return []

        inverse_size = 1.0 / voxel_size
        seen: dict[tuple[int, int, int], list[float]] = {}
        for point in points:
            key = (
                math.floor(point[0] * inverse_size),
                math.floor(point[1] * inverse_size),
                math.floor(point[2] * inverse_size),
            )
            if key not in seen or point[3] > seen[key][3]:
                seen[key] = point
        return list(seen.values())
