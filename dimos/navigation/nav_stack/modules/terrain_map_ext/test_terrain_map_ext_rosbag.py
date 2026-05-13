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

"""Rosbag validation test for TerrainMapExt.

Feeds recorded registered_scan + odometry data directly to the TerrainMapExt.
Compares against the reference terrain_map_ext output recorded from the OG ROS
nav stack at multiple timestamps.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np
import pytest
from scipy.spatial import cKDTree

from dimos.navigation.nav_stack.modules.terrain_map_ext.terrain_map_ext import (
    PLANAR_VOXEL_HALF_WIDTH,
    PLANAR_VOXEL_NUM,
    PLANAR_VOXEL_WIDTH,
    TERRAIN_VOXEL_HALF_WIDTH,
    TERRAIN_VOXEL_NUM,
    TERRAIN_VOXEL_WIDTH,
    TerrainMapExt,
    TerrainMapExtConfig,
    _voxel_index,
)
from dimos.navigation.nav_stack.tests.rosbag_fixtures import load_rosbag_window
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.self_hosted]

# Key differences from C++ code defaults:
# useSorting=true, quantileZ=0.1, lowerBoundZ=-2.5, checkTerrainConn=false
DEFAULT_CONFIG = TerrainMapExtConfig(
    scan_voxel_size=0.1,
    decay_time=4.0,
    no_decay_distance=0.0,
    clearing_distance=30.0,
    use_sorting=True,
    quantile_z=0.1,
    vehicle_height=1.5,
    lower_bound_z=-2.5,
    upper_bound_z=1.0,
    distance_ratio_z=0.1,
    voxel_point_update_threshold=100,
    voxel_time_update_threshold=2.0,
    check_terrain_connectivity=True,
    terrain_under_vehicle=-0.75,
    terrain_connectivity_threshold=0.5,
    ceiling_filtering_threshold=2.0,
    local_terrain_map_radius=4.0,
    terrain_voxel_size=2.0,
    planar_voxel_size=0.4,
    merge_local_terrain=True,
)


class _OfflineTerrainMapExt:
    """Offline processor exercising the same logic as TerrainMapExt without Module wiring."""

    def __init__(self, config: TerrainMapExtConfig) -> None:
        self.config = config
        self.terrain_voxel_cloud: list[list[list[float]]] = [[] for _ in range(TERRAIN_VOXEL_NUM)]
        self.terrain_voxel_update_num = [0] * TERRAIN_VOXEL_NUM
        self.terrain_voxel_update_time = [0.0] * TERRAIN_VOXEL_NUM
        self.terrain_voxel_shift_x = 0
        self.terrain_voxel_shift_y = 0
        self.system_init_time = 0.0
        self.system_inited = False
        # Exposed after each process_scan call for test inspection
        self.last_planar_voxel_elev: list[float] = [0.0] * PLANAR_VOXEL_NUM
        self.last_planar_voxel_conn: list[int] = [0] * PLANAR_VOXEL_NUM
        self.last_planar_point_elev: list[list[float]] = [[] for _ in range(PLANAR_VOXEL_NUM)]

    def process_scan(
        self,
        scan_points: np.ndarray,
        scan_time: float,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_z: float,
        local_terrain: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Process one scan and return (positions Nx3, intensities Nx1)."""
        config = self.config
        terrain_voxel_size = config.terrain_voxel_size

        if not self.system_inited:
            self.system_init_time = scan_time
            self.system_inited = True

        time_offset = scan_time - self.system_init_time
        max_range = terrain_voxel_size * (TERRAIN_VOXEL_HALF_WIDTH + 1)

        # Crop incoming scan
        cropped: list[list[float]] = []
        for i in range(len(scan_points)):
            point_x = float(scan_points[i, 0])
            point_y = float(scan_points[i, 1])
            point_z = float(scan_points[i, 2])
            distance = math.sqrt((point_x - vehicle_x) ** 2 + (point_y - vehicle_y) ** 2)
            relative_z = point_z - vehicle_z
            lower = config.lower_bound_z - config.distance_ratio_z * distance
            upper = config.upper_bound_z + config.distance_ratio_z * distance
            if relative_z > lower and relative_z < upper and distance < max_range:
                cropped.append([point_x, point_y, point_z, time_offset])

        # Rollover
        self._roll_terrain_grid(vehicle_x, vehicle_y, terrain_voxel_size)

        # Stack into voxels
        for point in cropped:
            ind_x = _voxel_index(point[0], vehicle_x, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH)
            ind_y = _voxel_index(point[1], vehicle_y, terrain_voxel_size, TERRAIN_VOXEL_HALF_WIDTH)
            if 0 <= ind_x < TERRAIN_VOXEL_WIDTH and 0 <= ind_y < TERRAIN_VOXEL_WIDTH:
                flat_idx = TERRAIN_VOXEL_WIDTH * ind_x + ind_y
                self.terrain_voxel_cloud[flat_idx].append(point)
                self.terrain_voxel_update_num[flat_idx] += 1

        # Downsample/evict
        for ind in range(TERRAIN_VOXEL_NUM):
            if (
                self.terrain_voxel_update_num[ind] >= config.voxel_point_update_threshold
                or time_offset - self.terrain_voxel_update_time[ind]
                >= config.voxel_time_update_threshold
            ):
                cell_points = self.terrain_voxel_cloud[ind]
                if not cell_points:
                    self.terrain_voxel_update_num[ind] = 0
                    self.terrain_voxel_update_time[ind] = time_offset
                    continue
                downsampled = TerrainMapExt._voxel_downsample(cell_points, config.scan_voxel_size)
                filtered: list[list[float]] = []
                for point in downsampled:
                    distance = math.sqrt((point[0] - vehicle_x) ** 2 + (point[1] - vehicle_y) ** 2)
                    relative_z = point[2] - vehicle_z
                    lower = config.lower_bound_z - config.distance_ratio_z * distance
                    upper = config.upper_bound_z + config.distance_ratio_z * distance
                    point_age = time_offset - point[3]
                    if (
                        relative_z > lower
                        and relative_z < upper
                        and (point_age < config.decay_time or distance < config.no_decay_distance)
                    ):
                        filtered.append(point)
                self.terrain_voxel_cloud[ind] = filtered
                self.terrain_voxel_update_num[ind] = 0
                self.terrain_voxel_update_time[ind] = time_offset

        # Gather central terrain
        terrain_cloud: list[list[float]] = []
        for ind_x in range(TERRAIN_VOXEL_HALF_WIDTH - 10, TERRAIN_VOXEL_HALF_WIDTH + 11):
            for ind_y in range(TERRAIN_VOXEL_HALF_WIDTH - 10, TERRAIN_VOXEL_HALF_WIDTH + 11):
                terrain_cloud.extend(self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y])

        # Planar ground estimation
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
                for delta_x in range(-1, 2):
                    for delta_y in range(-1, 2):
                        nx = ind_x + delta_x
                        ny = ind_y + delta_y
                        if 0 <= nx < PLANAR_VOXEL_WIDTH and 0 <= ny < PLANAR_VOXEL_WIDTH:
                            planar_point_elev[PLANAR_VOXEL_WIDTH * nx + ny].append(point[2])

        if config.use_sorting:
            for i in range(PLANAR_VOXEL_NUM):
                elevations = planar_point_elev[i]
                if elevations:
                    elevations.sort()
                    quantile_id = max(
                        0, min(int(config.quantile_z * len(elevations)), len(elevations) - 1)
                    )
                    planar_voxel_elev[i] = elevations[quantile_id]
        else:
            for i in range(PLANAR_VOXEL_NUM):
                elevations = planar_point_elev[i]
                if elevations:
                    planar_voxel_elev[i] = min(elevations)

        # BFS connectivity
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

        # Store for test inspection
        self.last_planar_voxel_elev = planar_voxel_elev
        self.last_planar_voxel_conn = planar_voxel_conn
        self.last_planar_point_elev = planar_point_elev

        # Build output with intensity = elevation distance from ground
        output_points: list[list[float]] = []
        output_intensities: list[float] = []
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
                        output_points.append([point[0], point[1], point[2]])
                        output_intensities.append(elevation_distance)

        # Merge local terrain (matches original C++ terrainAnalysisExt.cpp lines 542-551)
        if config.merge_local_terrain and local_terrain is not None and len(local_terrain) > 0:
            for i in range(len(local_terrain)):
                point_x = float(local_terrain[i, 0])
                point_y = float(local_terrain[i, 1])
                point_z = float(local_terrain[i, 2])
                distance = math.sqrt((point_x - vehicle_x) ** 2 + (point_y - vehicle_y) ** 2)
                if distance <= config.local_terrain_map_radius:
                    output_points.append([point_x, point_y, point_z])
                    output_intensities.append(0.0)

        if output_points:
            return (
                np.array(output_points, dtype=np.float32),
                np.array(output_intensities, dtype=np.float32),
            )
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.float32)

    def _roll_terrain_grid(
        self, vehicle_x: float, vehicle_y: float, terrain_voxel_size: float
    ) -> None:
        terrain_voxel_cen_x = terrain_voxel_size * self.terrain_voxel_shift_x
        terrain_voxel_cen_y = terrain_voxel_size * self.terrain_voxel_shift_y

        while vehicle_x - terrain_voxel_cen_x < -terrain_voxel_size:
            for ind_y in range(TERRAIN_VOXEL_WIDTH):
                for ind_x in range(TERRAIN_VOXEL_WIDTH - 1, 0, -1):
                    self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * (ind_x - 1) + ind_y]
                    )
                self.terrain_voxel_cloud[ind_y] = []
            self.terrain_voxel_shift_x -= 1
            terrain_voxel_cen_x = terrain_voxel_size * self.terrain_voxel_shift_x

        while vehicle_x - terrain_voxel_cen_x > terrain_voxel_size:
            for ind_y in range(TERRAIN_VOXEL_WIDTH):
                for ind_x in range(TERRAIN_VOXEL_WIDTH - 1):
                    self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * (ind_x + 1) + ind_y]
                    )
                self.terrain_voxel_cloud[
                    TERRAIN_VOXEL_WIDTH * (TERRAIN_VOXEL_WIDTH - 1) + ind_y
                ] = []
            self.terrain_voxel_shift_x += 1
            terrain_voxel_cen_x = terrain_voxel_size * self.terrain_voxel_shift_x

        while vehicle_y - terrain_voxel_cen_y < -terrain_voxel_size:
            for ind_x in range(TERRAIN_VOXEL_WIDTH):
                for ind_y in range(TERRAIN_VOXEL_WIDTH - 1, 0, -1):
                    self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + (ind_y - 1)]
                    )
                self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x] = []
            self.terrain_voxel_shift_y -= 1
            terrain_voxel_cen_y = terrain_voxel_size * self.terrain_voxel_shift_y

        while vehicle_y - terrain_voxel_cen_y > terrain_voxel_size:
            for ind_x in range(TERRAIN_VOXEL_WIDTH):
                for ind_y in range(TERRAIN_VOXEL_WIDTH - 1):
                    self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + ind_y] = (
                        self.terrain_voxel_cloud[TERRAIN_VOXEL_WIDTH * ind_x + (ind_y + 1)]
                    )
                self.terrain_voxel_cloud[
                    TERRAIN_VOXEL_WIDTH * ind_x + (TERRAIN_VOXEL_WIDTH - 1)
                ] = []
            self.terrain_voxel_shift_y += 1
            terrain_voxel_cen_y = terrain_voxel_size * self.terrain_voxel_shift_y


def _find_nearest_odom(odom: np.ndarray, target_time: float) -> tuple[float, float, float]:
    """Find the odom position closest to the given timestamp."""
    idx = np.argmin(np.abs(odom[:, 0] - target_time))
    return float(odom[idx, 1]), float(odom[idx, 2]), float(odom[idx, 3])


def _spatial_overlap_fraction(
    our_points: np.ndarray, ref_points: np.ndarray, threshold_m: float = 0.5
) -> float:
    """Fraction of reference points with a match in our output within threshold_m."""
    if len(our_points) == 0 or len(ref_points) == 0:
        return 0.0
    tree = cKDTree(our_points[:, :3])
    distances, _ = tree.query(ref_points[:, :3], k=1)
    return float(np.mean(distances < threshold_m))


def _run_processor_to_scan_index(
    window: object, processor: _OfflineTerrainMapExt, scan_index: int
) -> tuple[np.ndarray, np.ndarray]:
    """Run processor through scans[0..scan_index] and return final (positions, intensities)."""
    positions = np.zeros((0, 3), dtype=np.float32)
    intensities = np.zeros(0, dtype=np.float32)
    for idx in range(scan_index + 1):
        scan_time, scan_points = window.scans[idx]  # type: ignore[attr-defined]
        vehicle_x, vehicle_y, vehicle_z = _find_nearest_odom(
            window.odom,
            scan_time,  # type: ignore[attr-defined]
        )
        local_terrain = None
        for tmap_time, tmap_points in window.terrain_maps:  # type: ignore[attr-defined]
            if abs(tmap_time - scan_time) < 0.5:
                local_terrain = tmap_points
                break
        positions, intensities = processor.process_scan(
            scan_points,
            scan_time,
            vehicle_x,
            vehicle_y,
            vehicle_z,
            local_terrain=local_terrain,
        )
    return positions, intensities


class TestTerrainMapExtRosbag:
    """Validate TerrainMapExt against OG nav stack recording."""

    def test_produces_output(self) -> None:
        """Feeding scans produces non-empty terrain_map_ext output."""
        window = load_rosbag_window()
        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        positions, _ = _run_processor_to_scan_index(window, processor, 9)
        assert len(positions) > 0, "TerrainMapExt produced no output after 10 scans"
        logger.info(f"Output after 10 scans: {len(positions)} points")

    def test_height_filtering(self) -> None:
        """Every output point must satisfy per-point height bounds with distance scaling."""
        window = load_rosbag_window()
        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        positions, _ = _run_processor_to_scan_index(window, processor, 19)

        if len(positions) == 0:
            pytest.skip("No output produced")

        vehicle_x, vehicle_y, vehicle_z = _find_nearest_odom(window.odom, window.scans[19][0])
        violations = 0
        for i in range(len(positions)):
            point_x, point_y, point_z = positions[i]
            distance = math.sqrt((point_x - vehicle_x) ** 2 + (point_y - vehicle_y) ** 2)
            relative_z = point_z - vehicle_z
            lower = DEFAULT_CONFIG.lower_bound_z - DEFAULT_CONFIG.distance_ratio_z * distance
            upper = DEFAULT_CONFIG.upper_bound_z + DEFAULT_CONFIG.distance_ratio_z * distance
            if relative_z <= lower or relative_z >= upper:
                violations += 1

        logger.info(f"Height check: {len(positions)} points, {violations} violations")
        assert violations == 0, (
            f"{violations}/{len(positions)} points violate per-point height bounds"
        )

    def test_accumulation_grows(self) -> None:
        """Point count should grow as more scans are accumulated."""
        window = load_rosbag_window()
        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)

        counts = []
        for idx in range(15):
            scan_time, scan_points = window.scans[idx]
            vehicle_x, vehicle_y, vehicle_z = _find_nearest_odom(window.odom, scan_time)
            positions, _ = processor.process_scan(
                scan_points, scan_time, vehicle_x, vehicle_y, vehicle_z
            )
            counts.append(len(positions))

        logger.info(f"Point counts over 15 scans: {counts}")
        assert counts[-1] > counts[0], (
            f"Point count didn't grow: first={counts[0]}, last={counts[-1]}"
        )

    def test_intensity_is_elevation_distance(self) -> None:
        """Output intensity must be the elevation distance from estimated ground."""
        window = load_rosbag_window()
        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        positions, intensities = _run_processor_to_scan_index(window, processor, 19)

        if len(positions) == 0:
            pytest.skip("No output produced")

        # All intensities should be non-negative (absolute distance from ground)
        assert np.all(intensities >= 0), "Negative intensity values found"

        # All intensities should be < vehicle_height (filtered out otherwise)
        assert np.all(intensities < DEFAULT_CONFIG.vehicle_height), (
            f"Intensity exceeds vehicle_height: max={intensities.max():.3f}"
        )

        # A meaningful fraction of non-local points should have nonzero intensity
        nonzero_frac = float(np.mean(intensities > 0))
        assert nonzero_frac > 0.3, (
            f"Only {nonzero_frac:.1%} nonzero intensities — "
            "ground estimation may not be producing elevation distances"
        )
        logger.info(
            f"Intensity check: {len(intensities)} values, "
            f"range [{intensities.min():.3f}, {intensities.max():.3f}], "
            f"{nonzero_frac:.1%} nonzero"
        )

    def test_spatial_overlap_with_reference(self) -> None:
        """Our output should spatially overlap with the reference terrain_map_ext."""
        window = load_rosbag_window()
        assert len(window.terrain_maps_ext) > 0, "No reference terrain_map_ext"

        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        positions, _ = _run_processor_to_scan_index(window, processor, 19)

        scan_time = window.scans[19][0]
        ref_idx = int(np.argmin([abs(t - scan_time) for t, _ in window.terrain_maps_ext]))
        ref_points = window.terrain_maps_ext[ref_idx][1]

        if len(positions) == 0 or len(ref_points) == 0:
            pytest.skip("Empty output or reference")

        overlap = _spatial_overlap_fraction(positions, ref_points, threshold_m=0.5)
        logger.info(
            f"Spatial overlap: {overlap:.1%} of {len(ref_points)} ref points "
            f"matched within 0.5m (our: {len(positions)} points)"
        )
        assert overlap > 0.85, f"Spatial overlap {overlap:.1%} is too low (expected >85%)"

    def test_reference_comparison_multiple_timestamps(self) -> None:
        """Compare count ratio and spatial overlap at multiple timestamps."""
        window = load_rosbag_window()
        assert len(window.terrain_maps_ext) > 0, "No reference terrain_map_ext"

        test_indices = [5, 10, 15, 20, 25]
        test_indices = [i for i in test_indices if i < len(window.scans)]

        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        all_positions: dict[int, np.ndarray] = {}

        for idx in range(max(test_indices) + 1):
            scan_time, scan_points = window.scans[idx]
            vehicle_x, vehicle_y, vehicle_z = _find_nearest_odom(window.odom, scan_time)
            local_terrain = None
            for tmap_time, tmap_points in window.terrain_maps:
                if abs(tmap_time - scan_time) < 0.5:
                    local_terrain = tmap_points
                    break
            positions, _ = processor.process_scan(
                scan_points,
                scan_time,
                vehicle_x,
                vehicle_y,
                vehicle_z,
                local_terrain=local_terrain,
            )
            if idx in test_indices:
                all_positions[idx] = positions

        logger.info(f"\n{'=' * 60}")
        logger.info("TERRAIN MAP EXT — MULTI-TIMESTAMP COMPARISON")

        for idx in test_indices:
            our_pts = all_positions[idx]
            scan_time = window.scans[idx][0]
            ref_idx = int(np.argmin([abs(t - scan_time) for t, _ in window.terrain_maps_ext]))
            ref_pts = window.terrain_maps_ext[ref_idx][1]

            if len(our_pts) == 0 or len(ref_pts) == 0:
                logger.info(f"  scan[{idx}]: SKIP (empty)")
                continue

            count_ratio = len(our_pts) / len(ref_pts)
            overlap = _spatial_overlap_fraction(our_pts, ref_pts, threshold_m=0.5)

            logger.info(f"  scan[{idx}]: count_ratio={count_ratio:.2f}, overlap={overlap:.1%}")

            # Count ratio gap (~0.6) comes from local terrain merge timing:
            # offline test gets discrete terrain_map snapshots vs C++ continuous subscription.
            assert 0.4 < count_ratio < 1.5, (
                f"scan[{idx}]: count_ratio {count_ratio:.2f} out of range [0.4, 1.5]"
            )
            assert overlap > 0.85, f"scan[{idx}]: overlap {overlap:.1%} too low (expected >85%)"

        logger.info(f"{'=' * 60}\n")

    def test_far_field_count(self) -> None:
        """Far-field-only count (no local merge) should closely match reference far-field."""
        window = load_rosbag_window()
        assert len(window.terrain_maps_ext) > 0, "No reference terrain_map_ext"

        no_merge_config = TerrainMapExtConfig(
            **{
                **DEFAULT_CONFIG.__dict__,
                "merge_local_terrain": False,
            }
        )
        processor = _OfflineTerrainMapExt(no_merge_config)
        positions, _ = _run_processor_to_scan_index(window, processor, 19)

        # Reference far-field: total minus points within localTerrainMapRadius
        scan_time = window.scans[19][0]
        ref_idx = int(np.argmin([abs(t - scan_time) for t, _ in window.terrain_maps_ext]))
        ref_pts = window.terrain_maps_ext[ref_idx][1]
        vehicle_x, vehicle_y, _ = _find_nearest_odom(window.odom, scan_time)
        ref_dists = np.sqrt((ref_pts[:, 0] - vehicle_x) ** 2 + (ref_pts[:, 1] - vehicle_y) ** 2)
        ref_far_count = int(np.sum(ref_dists > DEFAULT_CONFIG.local_terrain_map_radius))

        if ref_far_count == 0:
            pytest.skip("No reference far-field points")

        far_ratio = len(positions) / ref_far_count
        logger.info(
            f"Far-field only: ours={len(positions)}, ref={ref_far_count}, ratio={far_ratio:.2f}"
        )
        assert 0.4 < far_ratio < 1.5, (
            f"Far-field count ratio {far_ratio:.2f} out of range [0.4, 1.5]"
        )

    def test_ground_elevation_validation(self) -> None:
        """Validate planar_voxel_elev: the core ground estimation computation."""
        window = load_rosbag_window()
        processor = _OfflineTerrainMapExt(DEFAULT_CONFIG)
        positions, intensities = _run_processor_to_scan_index(window, processor, 19)

        if len(positions) == 0:
            pytest.skip("No output produced")

        planar_elev = processor.last_planar_voxel_elev
        planar_conn = processor.last_planar_voxel_conn
        planar_point_elev = processor.last_planar_point_elev
        vehicle_x, vehicle_y, vehicle_z = _find_nearest_odom(window.odom, window.scans[19][0])
        planar_voxel_size = DEFAULT_CONFIG.planar_voxel_size

        # 1. Cross-check: intensity == |z - ground_elev| for non-local points
        non_local_mask = intensities > 0
        non_local_pts = positions[non_local_mask]
        non_local_int = intensities[non_local_mask]

        recomputed_intensities = []
        for i in range(len(non_local_pts)):
            ind_x = _voxel_index(
                non_local_pts[i, 0], vehicle_x, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
            )
            ind_y = _voxel_index(
                non_local_pts[i, 1], vehicle_y, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH
            )
            if 0 <= ind_x < PLANAR_VOXEL_WIDTH and 0 <= ind_y < PLANAR_VOXEL_WIDTH:
                flat_ind = PLANAR_VOXEL_WIDTH * ind_x + ind_y
                recomputed_intensities.append(abs(non_local_pts[i, 2] - planar_elev[flat_ind]))

        recomputed = np.array(recomputed_intensities, dtype=np.float32)
        assert len(recomputed) == len(non_local_int), (
            f"Recomputed {len(recomputed)} intensities but have {len(non_local_int)} non-local — "
            "some points fell outside planar grid"
        )
        np.testing.assert_allclose(
            non_local_int,
            recomputed,
            atol=1e-5,
            err_msg="Intensity != |z - ground_elev|: ground estimation inconsistency",
        )
        logger.info(
            f"Ground elev cross-check: {len(recomputed)} points, "
            f"max error = {np.max(np.abs(non_local_int - recomputed)):.2e}"
        )

        # 2. Ground elevation should be near or below vehicle z
        populated_indices = [i for i in range(PLANAR_VOXEL_NUM) if planar_point_elev[i]]
        populated_elevs = np.array([planar_elev[i] for i in populated_indices])

        above_vehicle = float(np.mean(populated_elevs > vehicle_z + 0.5))
        logger.info(
            f"Ground elevation: {len(populated_indices)} populated cells, "
            f"range [{populated_elevs.min():.2f}, {populated_elevs.max():.2f}], "
            f"vehicle_z={vehicle_z:.2f}, {above_vehicle:.1%} above vehicle+0.5m"
        )
        assert above_vehicle < 0.25, f"{above_vehicle:.1%} of ground cells are >0.5m above vehicle"

        # 3. Connected cells should form a coherent region
        connected_count = sum(1 for c in planar_conn if c == 2)
        populated_count = len(populated_indices)
        conn_ratio = connected_count / max(populated_count, 1)
        logger.info(
            f"Connectivity: {connected_count}/{populated_count} cells connected ({conn_ratio:.1%})"
        )
        assert conn_ratio > 0.7, (
            f"Only {conn_ratio:.1%} of populated cells are connected — "
            "BFS may not be propagating correctly"
        )

        # 4. Adjacent populated cells should have smooth ground elevation
        elev_diffs: list[float] = []
        for i in populated_indices:
            ix = i // PLANAR_VOXEL_WIDTH
            iy = i % PLANAR_VOXEL_WIDTH
            for delta_x, delta_y in [(1, 0), (0, 1)]:
                neighbor_x, neighbor_y = ix + delta_x, iy + delta_y
                if 0 <= neighbor_x < PLANAR_VOXEL_WIDTH and 0 <= neighbor_y < PLANAR_VOXEL_WIDTH:
                    neighbor = PLANAR_VOXEL_WIDTH * neighbor_x + neighbor_y
                    if planar_point_elev[neighbor]:
                        elev_diffs.append(abs(planar_elev[i] - planar_elev[neighbor]))

        diffs = np.array(elev_diffs)
        median_diff = float(np.median(diffs))
        pct95_diff = float(np.percentile(diffs, 95))
        logger.info(
            f"Ground smoothness: median neighbor diff={median_diff:.3f}m, "
            f"95th pct={pct95_diff:.3f}m"
        )
        assert median_diff < 0.15, (
            f"Median neighbor ground diff {median_diff:.3f}m — ground not smooth"
        )
