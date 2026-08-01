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

"""Hermetic regressions for offline spatial map generation."""

from pathlib import Path

import numpy as np
import pytest

from dimos.benchmark.spatial.config import SPATIAL_BENCHMARK_V1
from dimos.benchmark.spatial.map_generation import (
    PoseAlignment,
    VariantAlignment,
    _perturb_sensor_ranges,
    generate_full_coverage_trajectory,
    generate_map,
    load_snapshot_map,
    raycast_lidar,
    sensor_from_world,
    verify_full_coverage,
    world_from_sensor,
    write_snapshot,
)
from dimos.benchmark.spatial.models import (
    BarrierSegment,
    CoverageConnector,
    FreeSpaceModel,
    MapVariant,
    Point2D,
    Polygon2D,
    Pose2D,
)
from dimos.benchmark.spatial.structured3d import PlaneGeometry


def _wall(x: float) -> PlaneGeometry:
    return PlaneGeometry(
        source_plane_id=1,
        semantic_type="wall",
        contours_m=(((x, -2.0, 0.0), (x, 2.0, 0.0), (x, 2.0, 2.0), (x, -2.0, 2.0)),),
    )


def _free_space() -> FreeSpaceModel:
    return FreeSpaceModel(
        floor_regions=(
            Polygon2D(
                vertices=(
                    Point2D(x_m=-1, y_m=-1),
                    Point2D(x_m=3, y_m=-1),
                    Point2D(x_m=3, y_m=1),
                    Point2D(x_m=-1, y_m=1),
                )
            ),
        ),
        barriers=(BarrierSegment(start=Point2D(x_m=2.5, y_m=-2), end=Point2D(x_m=2.5, y_m=2)),),
    )


def test_raycast_uses_nearest_finite_wall_for_occlusion() -> None:
    profile = SPATIAL_BENCHMARK_V1.lidar_profiles[0].model_copy(
        update={"angular_resolution_deg": 360.0}
    )
    hits = raycast_lidar(
        pose=Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
        barriers=(
            BarrierSegment(start=Point2D(x_m=1, y_m=-2), end=Point2D(x_m=1, y_m=2)),
            BarrierSegment(start=Point2D(x_m=2, y_m=-2), end=Point2D(x_m=2, y_m=2)),
        ),
        profile=profile,
    )
    assert hits.shape == (1, 3)
    assert np.isclose(hits[0, 0], 1.0)


def test_regeneration_serialization_decode_and_frame_contract(tmp_path: Path) -> None:
    scene_id = "scene_" + "a" * 64
    trajectory = generate_full_coverage_trajectory(scene_id, _free_space(), spacing_m=2.0)
    profile = SPATIAL_BENCHMARK_V1.lidar_profiles[0].model_copy(
        update={"angular_resolution_deg": 90.0}
    )
    first = generate_map(trajectory, _free_space(), profile, seed=7)
    second = generate_map(trajectory, _free_space(), profile, seed=7)
    first_snapshot = write_snapshot(
        first,
        tmp_path / "first",
        scene_id=scene_id,
        trajectory_id=trajectory.trajectory_id,
        mapper_revision="test",
    )
    second_snapshot = write_snapshot(
        second,
        tmp_path / "second",
        scene_id=scene_id,
        trajectory_id=trajectory.trajectory_id,
        mapper_revision="test",
    )
    assert (tmp_path / "first" / "global_map.pc2.lcm").read_bytes() == (
        tmp_path / "second" / "global_map.pc2.lcm"
    ).read_bytes()
    decoded = load_snapshot_map(tmp_path / "first", first_snapshot)
    assert decoded.frame_id == "world"
    assert decoded.ts == 1_700_000_000.0 + len(trajectory.waypoints) - 1
    assert first_snapshot.variant is MapVariant.CLEAN
    assert first_snapshot.frame_contract.units == "m"
    assert first_snapshot.frame_contract.x_axis == "forward"
    assert first_snapshot.frame_contract.yaw_direction == "counterclockwise"
    assert first_snapshot.mapper_configuration.voxel_size_m == 0.05
    assert first_snapshot.mapper_configuration.frame_id == "world"
    assert first_snapshot.mapper_configuration_digest == second_snapshot.mapper_configuration_digest
    assert first_snapshot.map_artifact_sha256 == second_snapshot.map_artifact_sha256
    assert len(first.alignment.trace) == len(trajectory.waypoints)
    assert any(not item.scan_retained for item in first.alignment.trace) or all(
        item.scan_retained for item in first.alignment.trace
    )


def test_variant_alignment_projects_by_nearest_true_waypoint() -> None:
    alignment = VariantAlignment(
        (
            PoseAlignment(
                0,
                Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
                Pose2D(x_m=1.0, y_m=0.0, yaw_rad=0.0),
                False,
            ),
            PoseAlignment(
                1,
                Pose2D(x_m=10.0, y_m=0.0, yaw_rad=0.0),
                Pose2D(x_m=10.0, y_m=2.0, yaw_rad=0.5),
                True,
            ),
        )
    )

    point = alignment.project_point(Point2D(x_m=0.2, y_m=0.0))
    pose = alignment.project_pose(Pose2D(x_m=10.2, y_m=0.0, yaw_rad=0.25))

    assert point == Point2D(x_m=1.2, y_m=0.0)
    assert np.isclose(pose.yaw_rad, 0.75)
    assert pose.y_m > 2.0


def test_variant_alignment_tie_breaks_by_waypoint_order_and_reports_ambiguity() -> None:
    alignment = VariantAlignment(
        (
            PoseAlignment(
                0,
                Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0),
                Pose2D(x_m=1.0, y_m=0.0, yaw_rad=0.0),
                True,
            ),
            PoseAlignment(
                1,
                Pose2D(x_m=2.0, y_m=0.0, yaw_rad=0.0),
                Pose2D(x_m=2.0, y_m=10.0, yaw_rad=0.0),
                True,
            ),
        )
    )

    assert alignment.project_point(Point2D(x_m=1.0, y_m=0.0)) == Point2D(x_m=2.0, y_m=0.0)
    assert not alignment.has_unique_nearest(1.0, 0.0, tie_band_m=0.01)
    assert alignment.has_unique_nearest(0.25, 0.0, tie_band_m=0.01)


def test_asymmetric_sensor_world_transform_round_trip_preserves_frame_contract() -> None:
    pose = Pose2D(x_m=3.25, y_m=-1.75, yaw_rad=0.37)
    sensor_points = np.array(((0.4, -1.2, 0.3), (-2.1, 0.7, 1.4)), dtype=np.float64)
    world_points = world_from_sensor(sensor_points, pose, sensor_height_m=0.8)
    assert np.allclose(sensor_from_world(world_points, pose, sensor_height_m=0.8), sensor_points)
    assert world_points[0, 2] == 1.1


def test_asymmetric_absolute_world_frame_uses_right_handed_ccw_yaw() -> None:
    world_points = world_from_sensor(
        np.array(((2.0, 0.0, 0.0),), dtype=np.float64),
        Pose2D(x_m=3.0, y_m=-4.0, yaw_rad=np.pi / 2),
        sensor_height_m=1.5,
    )
    assert np.allclose(world_points, ((3.0, -2.0, 1.5)))


def test_noise_ranges_are_deterministic_and_clipped_to_sensor_range() -> None:
    profile = SPATIAL_BENCHMARK_V1.lidar_profiles[1].model_copy(
        update={"range_sigma_m": 100.0, "dropout_probability": 0.0, "max_range_m": 2.0}
    )
    points = np.array(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), dtype=np.float32)
    first = _perturb_sensor_ranges(points, profile, np.random.default_rng(9))
    second = _perturb_sensor_ranges(points, profile, np.random.default_rng(9))
    assert np.array_equal(first, second)
    assert np.all(np.linalg.norm(first[:, :2], axis=1) <= 2.0)
    assert np.all(np.linalg.norm(first[:, :2], axis=1) >= 0.0)


def test_connected_multi_room_opening_has_collision_free_coverage_path() -> None:
    rooms = (
        Polygon2D(
            vertices=(
                Point2D(x_m=0, y_m=0),
                Point2D(x_m=4, y_m=0),
                Point2D(x_m=4, y_m=4),
                Point2D(x_m=0, y_m=4),
            )
        ),
        Polygon2D(
            vertices=(
                Point2D(x_m=4, y_m=0),
                Point2D(x_m=8, y_m=0),
                Point2D(x_m=8, y_m=4),
                Point2D(x_m=4, y_m=4),
            )
        ),
    )
    free_space = FreeSpaceModel(
        floor_regions=rooms,
        barriers=(
            BarrierSegment(start=Point2D(x_m=4, y_m=0), end=Point2D(x_m=4, y_m=1)),
            BarrierSegment(start=Point2D(x_m=4, y_m=3), end=Point2D(x_m=4, y_m=4)),
        ),
    )
    trajectory = generate_full_coverage_trajectory("scene_" + "b" * 64, free_space)
    verify_full_coverage(free_space, trajectory.waypoints, resolution_m=1.0, max_range_m=12.0)
    assert any(waypoint.x_m > 4.0 for waypoint in trajectory.waypoints)


def test_disconnected_footprint_free_space_is_rejected() -> None:
    free_space = FreeSpaceModel(
        floor_regions=(
            Polygon2D(
                vertices=(
                    Point2D(x_m=0, y_m=0),
                    Point2D(x_m=2, y_m=0),
                    Point2D(x_m=2, y_m=2),
                    Point2D(x_m=0, y_m=2),
                )
            ),
            Polygon2D(
                vertices=(
                    Point2D(x_m=4, y_m=0),
                    Point2D(x_m=6, y_m=0),
                    Point2D(x_m=6, y_m=2),
                    Point2D(x_m=4, y_m=2),
                )
            ),
        ),
        barriers=(),
    )
    with pytest.raises(ValueError, match="cannot connect|visibility graph"):
        generate_full_coverage_trajectory("scene_" + "c" * 64, free_space)


def test_thin_portal_connector_routes_disconnected_eroded_components() -> None:
    left = Polygon2D(
        vertices=(
            Point2D(x_m=0, y_m=0),
            Point2D(x_m=4, y_m=0),
            Point2D(x_m=4, y_m=4),
            Point2D(x_m=0, y_m=4),
        )
    )
    right = Polygon2D(
        vertices=(
            Point2D(x_m=4.05, y_m=0),
            Point2D(x_m=8.05, y_m=0),
            Point2D(x_m=8.05, y_m=4),
            Point2D(x_m=4.05, y_m=4),
        )
    )
    portal = Polygon2D(
        vertices=(
            Point2D(x_m=4.0, y_m=1.0),
            Point2D(x_m=4.05, y_m=1.0),
            Point2D(x_m=4.05, y_m=3.0),
            Point2D(x_m=4.0, y_m=3.0),
        )
    )
    free_space = FreeSpaceModel(
        floor_regions=(left, right, portal),
        barriers=(),
        coverage_connectors=(
            CoverageConnector(start=Point2D(x_m=3.5, y_m=2.0), end=Point2D(x_m=4.55, y_m=2.0)),
        ),
    )

    trajectory = generate_full_coverage_trajectory("scene_" + "d" * 64, free_space)

    assert any(waypoint.x_m < 4.0 for waypoint in trajectory.waypoints)
    assert any(waypoint.x_m > 4.05 for waypoint in trajectory.waypoints)
