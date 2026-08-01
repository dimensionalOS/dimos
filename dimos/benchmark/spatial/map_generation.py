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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Deterministic offline lidar-to-voxel-map generation for spatial snapshots."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import hashlib
from heapq import heappop, heappush
from itertools import pairwise
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import unary_union

from dimos.benchmark.spatial.collision_oracle import (
    CandidateDisposition,
    GeometryCandidateRejectedError,
    SquareFootprintCollisionOracle,
)
from dimos.benchmark.spatial.config import (
    SPATIAL_BENCHMARK_V1,
    LidarNoiseProfile,
    SpatialBenchmarkConfig,
)
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    FrameConventionRecord,
    FreeSpaceModel,
    MapperConfigurationRecord,
    Point2D,
    Polygon2D,
    Pose2D,
    Snapshot,
    Trajectory,
)
from dimos.benchmark.spatial.utilities import canonical_json, hash_file_sha256, stable_opaque_id
from dimos.mapping.voxels.module import VoxelGridMapper, VoxelGridMapperConfig
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_DETERMINISTIC_EPOCH_S = 1_700_000_000.0


@dataclass(frozen=True)
class PoseAlignment:
    """True-to-estimated pose transform sampled at a trajectory waypoint."""

    waypoint_index: int
    true_pose: Pose2D
    estimated_pose: Pose2D
    scan_retained: bool


@dataclass(frozen=True)
class VariantAlignment:
    """Deterministic local projection from oracle/world geometry to map frame.

    Each physical query point/pose uses the true->estimated transform attached
    to the nearest true trajectory waypoint.  Equal-distance ties are resolved
    by waypoint order through ``waypoint_index``.  This is a local alignment for
    maps accumulated from drifting scans, not a single global rigid transform.
    """

    trace: tuple[PoseAlignment, ...]

    def project_point(self, point: Point2D) -> Point2D:
        alignment = self._nearest(point.x_m, point.y_m)
        return _project_point_between_poses(point, alignment.true_pose, alignment.estimated_pose)

    def project_pose(self, pose: Pose2D) -> Pose2D:
        alignment = self._nearest(pose.x_m, pose.y_m)
        point = _project_point_between_poses(
            Point2D(x_m=pose.x_m, y_m=pose.y_m), alignment.true_pose, alignment.estimated_pose
        )
        return Pose2D(
            x_m=point.x_m,
            y_m=point.y_m,
            yaw_rad=pose.yaw_rad + alignment.estimated_pose.yaw_rad - alignment.true_pose.yaw_rad,
        )

    def _nearest(self, x_m: float, y_m: float) -> PoseAlignment:
        if not self.trace:
            raise ValueError("variant alignment trace must not be empty")
        return min(
            self.trace,
            key=lambda item: (
                (item.true_pose.x_m - x_m) ** 2 + (item.true_pose.y_m - y_m) ** 2,
                item.waypoint_index,
            ),
        )

    def has_unique_nearest(self, x_m: float, y_m: float, tie_band_m: float) -> bool:
        """Return whether a point has one nearest waypoint outside a tie band."""

        if tie_band_m < 0.0:
            raise ValueError("tie_band_m must be non-negative")
        distances = sorted(
            math.hypot(item.true_pose.x_m - x_m, item.true_pose.y_m - y_m) for item in self.trace
        )
        if len(distances) < 2:
            return bool(distances)
        return distances[1] - distances[0] > tie_band_m


@dataclass(frozen=True)
class GeneratedMap:
    """The native final mapper output and metadata needed to write a snapshot."""

    pointcloud: PointCloud2
    terminal_pose: Pose2D
    seed: int
    profile: LidarNoiseProfile
    alignment: VariantAlignment


class _IterableStream(Stream[PointCloud2]):
    """Small finite stream adapter used to exercise a module's public pipeline."""

    def __init__(self, observations: tuple[Observation[PointCloud2], ...]) -> None:
        super().__init__()
        self._observations = observations

    def __iter__(self) -> Iterator[Observation[PointCloud2]]:
        return iter(self._observations)


def generate_full_coverage_trajectory(
    scene_id: str,
    free_space: FreeSpaceModel,
    *,
    spacing_m: float = 1.0,
    frame_id: str = "world",
) -> Trajectory:
    """Create the sole deterministic boustrophedon trajectory retained per scene.

    Samples are only retained strictly inside authoritative floor polygons.  A
    spacing no greater than the configured lidar range is required so every
    retained walkable component has an observation within sensing range.
    """

    if not 0.0 < spacing_m <= float(SPATIAL_BENCHMARK_V1.lidar_profiles[0].max_range_m):
        raise ValueError("spacing_m must be positive and no greater than lidar range")
    walkable, navigable = _coverage_geometries(free_space)
    components = tuple(sorted(_polygon_components(navigable), key=lambda item: item.bounds))
    collision_oracle = SquareFootprintCollisionOracle(
        floor_regions=free_space.floor_regions,
        blocked_regions=(),
        barriers=free_space.barriers,
        footprint=SPATIAL_BENCHMARK_V1.footprint,
        tolerances=SPATIAL_BENCHMARK_V1.geometry_tolerances,
    )
    points = [
        point for component in components for point in _coverage_samples(component, spacing_m)
    ]
    points = _stable_pose_points(points, collision_oracle)
    if not points:
        raise ValueError("footprint-eroded free space has no trajectory samples")
    route = _route_coverage_points(points, free_space)
    waypoints = tuple(
        Pose2D(
            x_m=x,
            y_m=y,
            yaw_rad=math.atan2(next_y - y, next_x - x) if index + 1 < len(route) else 0.0,
        )
        for index, ((x, y), (next_x, next_y)) in enumerate(
            zip(route, [*route[1:], (route[-1][0] + 1.0, route[-1][1])], strict=True)
        )
    )
    connector_edges = _connector_coordinate_edges(
        tuple((waypoint.x_m, waypoint.y_m) for waypoint in waypoints), free_space
    )
    for waypoint in waypoints:
        if collision_oracle.evaluate_pose(waypoint).disposition is CandidateDisposition.COLLISION:
            raise ValueError("coverage waypoint is not collision-free")
    for first, second in pairwise(waypoints):
        if _coordinate_edge((first.x_m, first.y_m), (second.x_m, second.y_m)) in connector_edges:
            continue
        distance = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        if (
            collision_oracle.evaluate_translation(first, distance).disposition
            is CandidateDisposition.COLLISION
        ):
            raise ValueError("coverage route is not continuously collision-free")
    verify_full_coverage(
        free_space,
        waypoints,
        resolution_m=spacing_m,
        max_range_m=float(SPATIAL_BENCHMARK_V1.lidar_profiles[0].max_range_m),
    )
    trajectory_id = stable_opaque_id("trajectory", {"scene": scene_id, "policy": "coverage-v1"})
    return Trajectory(
        trajectory_id=trajectory_id,
        scene_id=scene_id,
        policy_version="coverage-v1",
        frame_id=frame_id,
        waypoints=waypoints,
    )


def raycast_lidar(
    pose: Pose2D,
    barriers: tuple[BarrierSegment, ...],
    profile: LidarNoiseProfile,
    *,
    sensor_height_m: float = 1.0,
) -> np.ndarray:
    """Raycast horizontal beams against opening-subtracted 2-D barriers.

    A horizontal lidar is explicitly modelled as observing barriers from floor
    through ``sensor_height_m``; imported 3-D walls remain the provenance for
    these authoritative 2-D segments, while door spans are never ray targets.
    """

    resolution = math.radians(float(profile.angular_resolution_deg))
    count = max(1, round((2.0 * math.pi) / resolution))
    hits: list[tuple[float, float, float]] = []
    origin = np.array((pose.x_m, pose.y_m, sensor_height_m), dtype=np.float64)
    for index in range(count):
        angle = pose.yaw_rad + index * 2.0 * math.pi / count
        direction = np.array((math.cos(angle), math.sin(angle), 0.0), dtype=np.float64)
        distances = tuple(
            distance
            for barrier in barriers
            if (distance := _ray_barrier_hit(origin[:2], direction[:2], barrier)) is not None
        )
        if distances:
            distance = min(distances)
            if distance <= float(profile.max_range_m):
                hits.append(tuple(origin + distance * direction))
    return np.asarray(hits, dtype=np.float32).reshape((-1, 3))


def world_from_sensor(
    points: np.ndarray, pose: Pose2D, *, sensor_height_m: float = 1.0
) -> np.ndarray:
    """Transform sensor-frame points into the benchmark's +z, CCW-yaw world frame."""

    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    rotation = np.array(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))
    transformed: np.ndarray = points @ rotation.T + np.array((pose.x_m, pose.y_m, sensor_height_m))
    return transformed


def sensor_from_world(
    points: np.ndarray, pose: Pose2D, *, sensor_height_m: float = 1.0
) -> np.ndarray:
    """Invert :func:`world_from_sensor` without changing units or handedness."""

    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    rotation = np.array(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))
    transformed: np.ndarray = (points - np.array((pose.x_m, pose.y_m, sensor_height_m))) @ rotation
    return transformed


def generate_map(
    trajectory: Trajectory,
    free_space: FreeSpaceModel,
    profile: LidarNoiseProfile,
    seed: int,
    *,
    config: SpatialBenchmarkConfig = SPATIAL_BENCHMARK_V1,
) -> GeneratedMap:
    """Simulate true-pose lidar, then map estimated-pose world points via pipeline."""

    rng = np.random.default_rng(seed)
    drift_xy = np.zeros(2, dtype=np.float64)
    drift_yaw = 0.0
    if trajectory.frame_id != config.mapper.frame_id:
        raise ValueError("trajectory must be expressed in the mapper world frame")
    observations: list[Observation[PointCloud2]] = []
    trace: list[PoseAlignment] = []
    terminal_pose = trajectory.waypoints[-1]
    for index, true_pose in enumerate(trajectory.waypoints):
        drift_xy, drift_yaw = _next_drift(drift_xy, drift_yaw, profile, rng)
        estimated_pose = Pose2D(
            x_m=true_pose.x_m + float(drift_xy[0]),
            y_m=true_pose.y_m + float(drift_xy[1]),
            yaw_rad=true_pose.yaw_rad + drift_yaw,
        )
        terminal_pose = estimated_pose
        timestamp = _DETERMINISTIC_EPOCH_S + index
        scan_retained = rng.random() <= float(profile.coverage_probability)
        trace.append(PoseAlignment(index, true_pose, estimated_pose, scan_retained))
        if not scan_retained:
            continue
        # Raycast uses the physical pose.  Range/dropout are sensor-frame
        # effects; only then does estimated localization place points in world.
        sensor_points = sensor_from_world(
            raycast_lidar(true_pose, free_space.barriers, profile), true_pose
        )
        sensor_points = _perturb_sensor_ranges(sensor_points, profile, rng)
        if len(sensor_points):
            world_points = world_from_sensor(sensor_points, estimated_pose)
            cloud = PointCloud2.from_numpy(
                world_points, frame_id=config.mapper.frame_id, timestamp=timestamp
            )
            observations.append(Observation(ts=timestamp, data_type=PointCloud2, _data=cloud))
    if not observations:
        raise ValueError("final map has no lidar observations")
    # ``pipeline`` is intentionally exercised without starting module runtime
    # transports: this finite offline route needs only the public transform API.
    mapper = object.__new__(VoxelGridMapper)
    mapper.config = VoxelGridMapperConfig(
        voxel_size=float(config.mapper.voxel_size),
        block_count=config.mapper.block_count,
        device=config.mapper.device,
        carve_columns=config.mapper.carve_columns,
        frame_id=config.mapper.frame_id,
        emit_every=config.mapper.emit_every,
    )
    outputs = tuple(mapper.pipeline(_IterableStream(tuple(observations))))
    if not outputs:
        raise ValueError("VoxelGridMapper pipeline emitted no final map")
    # The final emitted observation is the public mapper result and carries the
    # exact timestamp of the final accepted input observation.
    return GeneratedMap(
        outputs[-1].data, terminal_pose, seed, profile, VariantAlignment(tuple(trace))
    )


def _project_point_between_poses(
    point: Point2D, true_pose: Pose2D, estimated_pose: Pose2D
) -> Point2D:
    local = sensor_from_world(
        np.array(((point.x_m, point.y_m, 0.0),), dtype=np.float64), true_pose, sensor_height_m=0.0
    )
    projected = world_from_sensor(local, estimated_pose, sensor_height_m=0.0)[0]
    return Point2D(x_m=float(projected[0]), y_m=float(projected[1]))


def write_snapshot(
    generated: GeneratedMap,
    directory: Path,
    *,
    scene_id: str,
    trajectory_id: str,
    mapper_revision: str,
    config: SpatialBenchmarkConfig = SPATIAL_BENCHMARK_V1,
) -> Snapshot:
    """Write exact LCM bytes and canonical strict snapshot JSON."""

    directory.mkdir(parents=True, exist_ok=True)
    artifact = directory / "global_map.pc2.lcm"
    artifact.write_bytes(generated.pointcloud.lcm_encode(frame_id=config.mapper.frame_id))
    config_digest = hashlib.sha256(canonical_json(config.model_dump(mode="json"))).hexdigest()
    snapshot = Snapshot(
        snapshot_id=stable_opaque_id(
            "snapshot",
            {
                "scene": scene_id,
                "trajectory": trajectory_id,
                "variant": generated.profile.name.value,
            },
        ),
        scene_id=scene_id,
        trajectory_id=trajectory_id,
        variant=generated.profile.name,
        terminal_pose=generated.terminal_pose,
        map_artifact_path="global_map.pc2.lcm",
        map_artifact_sha256=hash_file_sha256(artifact),
        mapper_revision=mapper_revision,
        mapper_configuration_digest=config_digest,
        noise_profile_version=f"lidar-{generated.profile.name.value}-v1",
        seed=generated.seed,
        frame_id=config.mapper.frame_id,
        mapper_configuration=MapperConfigurationRecord(
            voxel_size_m=float(config.mapper.voxel_size),
            block_count=config.mapper.block_count,
            device=config.mapper.device,
            carve_columns=config.mapper.carve_columns,
            frame_id=config.mapper.frame_id,
            emit_every=config.mapper.emit_every,
        ),
        frame_contract=FrameConventionRecord(
            frame_id=config.mapper.frame_id,
            space=config.world_frame.space,
            units=config.world_frame.units,
            handedness=config.world_frame.handedness,
            gravity_axis=config.world_frame.gravity_axis,
            x_axis=config.world_frame.x_axis,
            y_axis=config.world_frame.y_axis,
            yaw_direction=config.world_frame.yaw_direction,
            yaw_units=config.world_frame.yaw_units,
        ),
    )
    (directory / "snapshot.json").write_bytes(
        canonical_json(snapshot.model_dump(mode="json")) + b"\n"
    )
    return snapshot


def load_snapshot_map(directory: Path, snapshot: Snapshot) -> PointCloud2:
    """Verify a snapshot artifact then decode its native PointCloud2 bytes."""

    artifact = directory / snapshot.map_artifact_path
    if hash_file_sha256(artifact) != snapshot.map_artifact_sha256:
        raise ValueError("snapshot map artifact SHA-256 does not match")
    pointcloud = PointCloud2.lcm_decode(artifact.read_bytes())
    if pointcloud.frame_id != snapshot.frame_contract.frame_id:
        raise ValueError("decoded PointCloud2 frame does not match snapshot frame contract")
    return pointcloud


def _next_drift(
    drift_xy: np.ndarray,
    drift_yaw: float,
    profile: LidarNoiseProfile,
    rng: np.random.Generator,
) -> tuple[np.ndarray, float]:
    correlation = float(profile.drift_correlation)
    innovation = math.sqrt(1.0 - correlation**2)
    xy = correlation * drift_xy + innovation * rng.normal(
        0.0, float(profile.planar_drift_sigma_m), 2
    )
    yaw = correlation * drift_yaw + innovation * float(
        rng.normal(0.0, float(profile.yaw_drift_sigma_rad))
    )
    return xy, yaw


def _perturb_sensor_ranges(
    points: np.ndarray,
    profile: LidarNoiseProfile,
    rng: np.random.Generator,
) -> np.ndarray:
    if not len(points):
        return points
    original_ranges = np.linalg.norm(points[:, :2], axis=1)
    keep = rng.random(len(points)) >= float(profile.dropout_probability)
    ranges = original_ranges + rng.normal(0.0, float(profile.range_sigma_m), len(points)).astype(
        np.float32
    )
    ranges = np.clip(ranges, 0.0, float(profile.max_range_m))
    unit = points[:, :2] / np.maximum(original_ranges[:, None], np.finfo(np.float32).eps)
    output = points.copy()
    output[:, :2] = unit * ranges[:, None]
    return output[keep]


def verify_full_coverage(
    free_space: FreeSpaceModel,
    waypoints: tuple[Pose2D, ...],
    *,
    resolution_m: float,
    max_range_m: float,
) -> None:
    """Prove each deterministic navigable-cell target has finite-range clear sight.

    Targets are a fixed lattice plus each component representative, so narrow and
    concave regions cannot disappear merely because no lattice intersection lands
    inside them.  A barrier intersection invalidates sight even when floor union
    containment alone would otherwise permit the segment.
    """
    if not waypoints or resolution_m <= 0.0 or max_range_m <= 0.0:
        raise ValueError("coverage verifier requires waypoints and positive resolution and range")
    walkable, navigable = _coverage_geometries(free_space)
    targets = [
        target
        for component in _polygon_components(navigable)
        for target in _coverage_samples(component, resolution_m)
    ]
    barriers = tuple(
        LineString(((barrier.start.x_m, barrier.start.y_m), (barrier.end.x_m, barrier.end.y_m)))
        for barrier in free_space.barriers
    )
    for target in targets:
        if not any(
            math.dist(target, (waypoint.x_m, waypoint.y_m)) <= max_range_m
            and walkable.covers(LineString((target, (waypoint.x_m, waypoint.y_m))))
            and not any(
                LineString((target, (waypoint.x_m, waypoint.y_m))).crosses(barrier)
                for barrier in barriers
            )
            for waypoint in waypoints
        ):
            raise ValueError(
                "coverage verifier found a navigable target without finite-range line of sight"
            )


def _coverage_geometries(
    free_space: FreeSpaceModel,
) -> tuple[Polygon | MultiPolygon, Polygon | MultiPolygon]:
    floors = unary_union(tuple(_polygon(region) for region in free_space.floor_regions))
    if not isinstance(floors, (Polygon, MultiPolygon)):
        raise ValueError("walkable union must be polygonal")
    clearance = (
        float(SPATIAL_BENCHMARK_V1.footprint.side_length_m) / 2.0
        + float(SPATIAL_BENCHMARK_V1.footprint.safety_margin_m)
        + float(SPATIAL_BENCHMARK_V1.geometry_tolerances.collision_uncertainty_margin_m)
    )
    barriers = unary_union(
        tuple(
            LineString(
                ((barrier.start.x_m, barrier.start.y_m), (barrier.end.x_m, barrier.end.y_m))
            ).buffer(clearance, cap_style="flat")
            for barrier in free_space.barriers
        )
    )
    navigable = floors.buffer(-clearance).difference(barriers)
    if not isinstance(navigable, (Polygon, MultiPolygon)) or navigable.is_empty:
        raise ValueError("footprint-eroded free space is empty")
    return floors, navigable


def _polygon(region: Polygon2D) -> Polygon:
    """Convert an immutable record only at the geometry boundary."""
    return Polygon(
        tuple((vertex.x_m, vertex.y_m) for vertex in region.vertices),
        tuple(tuple((vertex.x_m, vertex.y_m) for vertex in hole) for hole in region.holes),
    )


def _polygon_components(geometry: Polygon | MultiPolygon) -> tuple[Polygon, ...]:
    return (geometry,) if isinstance(geometry, Polygon) else tuple(geometry.geoms)


def _coverage_samples(component: Polygon, spacing_m: float) -> list[tuple[float, float]]:
    """Return one deterministic interior target for every occupied grid cell."""
    min_x, min_y, max_x, max_y = component.bounds
    samples: list[tuple[float, float]] = []
    for row in range(math.floor(min_y / spacing_m), math.ceil(max_y / spacing_m) + 1):
        row_samples: list[tuple[float, float]] = []
        for column in range(math.floor(min_x / spacing_m), math.ceil(max_x / spacing_m) + 1):
            cell = Polygon(
                (
                    (column * spacing_m, row * spacing_m),
                    ((column + 1) * spacing_m, row * spacing_m),
                    ((column + 1) * spacing_m, (row + 1) * spacing_m),
                    (column * spacing_m, (row + 1) * spacing_m),
                )
            )
            occupied = component.intersection(cell)
            if occupied.area > 0.0:
                representative = occupied.representative_point()
                row_samples.append((float(representative.x), float(representative.y)))
        samples.extend(reversed(row_samples) if row % 2 else row_samples)
    return samples


def _stable_pose_points(
    points: list[tuple[float, float]], oracle: SquareFootprintCollisionOracle
) -> list[tuple[float, float]]:
    retained: list[tuple[float, float]] = []
    for x_m, y_m in points:
        try:
            if oracle.pose_is_collision_free(Pose2D(x_m=x_m, y_m=y_m, yaw_rad=0.0)):
                retained.append((x_m, y_m))
        except GeometryCandidateRejectedError:
            continue
    return retained


def _route_coverage_points(
    points: list[tuple[float, float]], free_space: FreeSpaceModel
) -> list[tuple[float, float]]:
    _, navigable = _coverage_geometries(free_space)
    connector_edges = _connector_index_edges(tuple(points), free_space)
    edges: list[list[tuple[int, float]]] = [[] for _ in points]
    for first, start in enumerate(points):
        for second in range(first + 1, len(points)):
            distance = math.dist(start, points[second])
            is_connector_edge = frozenset((first, second)) in connector_edges
            if is_connector_edge or navigable.covers(LineString((start, points[second]))):
                edges[first].append((second, distance))
                edges[second].append((first, distance))
    route = [0]
    current = 0
    for target in range(1, len(points)):
        path = _shortest_path(edges, current, target)
        if path is None:
            raise ValueError("coverage visibility graph cannot connect navigable samples")
        route.extend(path[1:])
        current = target
    return [points[index] for index in route]


def _connector_index_edges(
    points: tuple[tuple[float, float], ...], free_space: FreeSpaceModel
) -> set[frozenset[int]]:
    edges: set[frozenset[int]] = set()
    for connector in free_space.coverage_connectors:
        start = _nearest_point_index(points, (connector.start.x_m, connector.start.y_m))
        end = _nearest_point_index(points, (connector.end.x_m, connector.end.y_m))
        if start != end:
            edges.add(frozenset((start, end)))
    return edges


def _connector_coordinate_edges(
    points: tuple[tuple[float, float], ...], free_space: FreeSpaceModel
) -> set[frozenset[tuple[float, float]]]:
    return {
        _coordinate_edge(points[first], points[second])
        for edge in _connector_index_edges(points, free_space)
        for first, second in (tuple(edge),)
    }


def _coordinate_edge(
    first: tuple[float, float], second: tuple[float, float]
) -> frozenset[tuple[float, float]]:
    return frozenset((first, second))


def _nearest_point_index(
    points: tuple[tuple[float, float], ...], target: tuple[float, float]
) -> int:
    return min(
        range(len(points)),
        key=lambda index: (math.dist(points[index], target), index),
    )


def _shortest_path(
    edges: list[list[tuple[int, float]]], start: int, target: int
) -> list[int] | None:
    queue: list[tuple[float, int]] = [(0.0, start)]
    previous: dict[int, int] = {}
    distances = {start: 0.0}
    while queue:
        distance, node = heappop(queue)
        if node == target:
            path = [node]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return list(reversed(path))
        if distance != distances[node]:
            continue
        for neighbor, cost in edges[node]:
            candidate = distance + cost
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                previous[neighbor] = node
                heappush(queue, (candidate, neighbor))
    return None


def _ray_barrier_hit(
    origin: np.ndarray, direction: np.ndarray, barrier: BarrierSegment
) -> float | None:
    start = np.array((barrier.start.x_m, barrier.start.y_m), dtype=np.float64)
    segment = np.array((barrier.end.x_m - barrier.start.x_m, barrier.end.y_m - barrier.start.y_m))
    determinant = direction[0] * -segment[1] - direction[1] * -segment[0]
    if abs(determinant) < 1e-10:
        return None
    t, u = np.linalg.solve(
        np.array(((direction[0], -segment[0]), (direction[1], -segment[1]))), start - origin
    )
    return float(t) if t > 0.0 and 0.0 <= u <= 1.0 else None
