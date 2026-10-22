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

from collections.abc import Callable

import mujoco
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]
import pytest

from dimos.simulation.mujoco.constants import LIDAR_RESOLUTION, MAX_HEIGHT, MAX_RANGE, MIN_RANGE
from dimos.simulation.mujoco.raycast_lidar import (
    EmbreeLidar,
    scan_directions,
    voxel_down_sample,
)


@pytest.fixture
def scene() -> tuple[mujoco.MjModel, mujoco.MjData, int]:
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="10 10 0.1"/>
            <geom name="box" type="box" pos="2 0 1" size="0.25 0.5 0.5" group="0"/>
            <body name="robot" pos="0 0 0.5">
              <freejoint/>
              <geom type="sphere" pos="0 0.75 0.5" size="0.1" group="3"/>
              <body name="mount" pos="0 0 0.5">
                <site name="lidar"/>
                <geom type="sphere" pos="0 -0.75 0" size="0.1" group="1"/>
              </body>
            </body>
            <body name="person" pos="-2 0 1" mocap="true">
              <geom type="box" size="0.2 0.2 0.5" group="1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, model.site("lidar").id


@pytest.fixture
def lidar_factory(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
) -> Callable[[NDArray[np.float64]], EmbreeLidar]:
    model, data, site_id = scene

    def build(directions: NDArray[np.float64]) -> EmbreeLidar:
        return EmbreeLidar(model, data, site_id, directions)

    return build


@pytest.fixture
def floor_scene() -> tuple[mujoco.MjModel, mujoco.MjData, int]:
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom type="plane" size="0 0 0.1"/>
            <body name="robot" pos="0 0 0.5">
              <freejoint/>
              <geom type="sphere" size="0.03"/>
              <body name="mount" pos="0.3 0.1 0.3">
                <site name="lidar"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    return model, mujoco.MjData(model), model.site("lidar").id


@pytest.mark.parametrize("position", [(0.0, 0.0), (1.25, -0.75)])
@pytest.mark.parametrize("yaw_deg", [0, 45, 90])
def test_lidar_box_follows_base_translation_without_rotating_or_rounding_corners(
    floor_scene: tuple[mujoco.MjModel, mujoco.MjData, int],
    position: tuple[float, float],
    yaw_deg: float,
) -> None:
    model, data, site_id = floor_scene
    data.qpos[:2] = position
    half_yaw = np.deg2rad(yaw_deg) / 2
    data.qpos[3:7] = [np.cos(half_yaw), 0, 0, np.sin(half_yaw)]
    mujoco.mj_forward(model, data)
    # Eight points on the square's boundary, plus four just outside it.
    targets = np.array(
        [
            [-3.2, -3.2, 0],
            [-3.2, 3.2, 0],
            [3.2, -3.2, 0],
            [3.2, 3.2, 0],
            [-3.2, 0, 0],
            [3.2, 0, 0],
            [0, -3.2, 0],
            [0, 3.2, 0],
            [-3.21, 0, 0],
            [3.21, 0, 0],
            [0, -3.21, 0],
            [0, 3.21, 0],
        ]
    )
    targets[:, :2] += position
    directions_world = targets - data.site_xpos[site_id]
    directions_world /= np.linalg.norm(directions_world, axis=1, keepdims=True)
    directions_local = directions_world @ data.site_xmat[site_id].reshape(3, 3)

    points = EmbreeLidar(model, data, site_id, directions_local).cast_box(data, 3.2)

    assert np.all(points[:, :2] >= (np.array(position) - 3.2).astype(np.float32))
    assert np.all(points[:, :2] <= (np.array(position) + 3.2).astype(np.float32))

    # Remove ray-intersection roundoff before sorting points with equal X/Y.
    actual = _sorted(np.round(points.astype(np.float64), decimals=6))
    np.testing.assert_allclose(actual, _sorted(targets[:8]), atol=1e-6)


@pytest.mark.parametrize("elevation_min,elevation_max", [(-7, 52), (-80, 10)])
def test_scan_directions_covers_full_azimuth_and_elevation(
    elevation_min: float, elevation_max: float
) -> None:
    directions = scan_directions(4, elevation_min, elevation_max, 3)

    assert directions.shape == (12, 3)
    assert directions.dtype == np.float64
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0)
    elevations = np.rad2deg(np.arcsin(directions[:, 2])).reshape(3, 4)
    np.testing.assert_allclose(
        elevations[:, 0], [elevation_min, (elevation_min + elevation_max) / 2, elevation_max]
    )
    horizontal = directions[:, :2] / np.linalg.norm(directions[:, :2], axis=1, keepdims=True)
    np.testing.assert_allclose(
        horizontal, np.tile([[1, 0], [0, 1], [-1, 0], [0, -1]], (3, 1)), atol=1e-15
    )


def test_cast_lidar_hits_box_faces(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
    lidar_factory: Callable[[NDArray[np.float64]], EmbreeLidar],
) -> None:
    _, data, _ = scene
    directions = np.array([[1.0, 0, 0], [1, 0.1, 0.1], [1, -0.1, -0.1]])
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)

    lidar = lidar_factory(directions)
    points = lidar.cast(data, MAX_RANGE)

    assert points.dtype == np.float32
    np.testing.assert_allclose(
        points, [[1.75, 0, 1], [1.75, 0.175, 1.175], [1.75, -0.175, 0.825]], atol=1e-3
    )


def test_cast_lidar_excludes_robot_subtree_and_tracks_person_behind_it(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
) -> None:
    model, data, site_id = scene
    directions = scan_directions(4, 0, 0, 1)

    lidar = EmbreeLidar(model, data, site_id, directions)
    points = lidar.cast(data, MAX_RANGE)

    # The +y/-y rays pass through the robot's root/mount; neither returns a hit.
    np.testing.assert_allclose(points, [[1.75, 0, 1], [-1.8, 0, 1]], atol=1e-3)

    qpos = data.qpos.copy()
    data.mocap_pos[0, 0] = -2.5
    mujoco.mj_forward(model, data)
    moved = lidar.cast(data, MAX_RANGE)

    np.testing.assert_allclose(moved, [[1.75, 0, 1], [-2.3, 0, 1]], atol=1e-3)

    data.mocap_pos[0] = [0, 1.5, 1]
    mujoco.mj_forward(model, data)
    behind_robot = lidar.cast(data, MAX_RANGE)

    # Excluding robot geometry must also allow seeing the person behind it.
    np.testing.assert_allclose(behind_robot, [[1.75, 0, 1], [0, 1.3, 1]], atol=1e-3)
    np.testing.assert_array_equal(data.qpos, qpos)


@pytest.mark.parametrize(
    "distance,expected",
    [
        (MIN_RANGE / 2, []),
        (MIN_RANGE, [[MIN_RANGE, 0, 1]]),
        (MIN_RANGE - 1e-4, []),
        (MIN_RANGE + 1e-4, [[MIN_RANGE + 1e-4, 0, 1]]),
        (MAX_RANGE, [[MAX_RANGE, 0, 1]]),
        (MAX_RANGE - 1e-4, [[MAX_RANGE - 1e-4, 0, 1]]),
        (MAX_RANGE + 1e-4, []),
        # The box's bounding sphere overlaps the cutoff, but its face is beyond it.
        (MAX_RANGE + 0.1, []),
    ],
)
def test_cast_lidar_enforces_range_on_hit_distance(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
    distance: float,
    expected: list[list[float]],
) -> None:
    model, data, site_id = scene
    model.geom_pos[model.geom("box").id, 0] = distance + 0.25
    mujoco.mj_forward(model, data)

    points = EmbreeLidar(model, data, site_id, np.array([[1.0, 0, 0]])).cast(data, MAX_RANGE)

    np.testing.assert_allclose(points, np.asarray(expected).reshape(-1, 3), atol=1e-3)
    distances = np.linalg.norm(points - data.site_xpos[site_id], axis=1)
    assert np.all(distances >= MIN_RANGE)
    assert np.all(distances <= MAX_RANGE)


@pytest.mark.parametrize(
    "height,expected",
    [
        (MAX_HEIGHT, [[0.25, -0.5, 0]]),
        (MAX_HEIGHT - 1e-4, [[0.25, -0.5, 0]]),
        (MAX_HEIGHT + 1e-4, []),
        (MAX_HEIGHT + 0.1, []),
    ],
)
def test_cast_lidar_uses_site_pose_and_world_height_band(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
    height: float,
    expected: list[list[float]],
) -> None:
    model, data, site_id = scene
    data.qpos[:3] = [-0.25, -0.5, height]
    # Local +x now points straight down; local z cannot implement the world height filter.
    data.qpos[3:7] = [np.sqrt(0.5), 0, np.sqrt(0.5), 0]
    mujoco.mj_forward(model, data)

    points = EmbreeLidar(model, data, site_id, np.array([[1.0, 0, 0]])).cast(data, MAX_RANGE)

    np.testing.assert_allclose(points, np.asarray(expected).reshape(-1, 3), atol=1e-3)


def test_cast_lidar_misses_and_empty_scan_return_empty_cloud(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
) -> None:
    model, data, site_id = scene
    misses = EmbreeLidar(model, data, site_id, np.array([[0.0, 0, 1]])).cast(data, MAX_RANGE)
    empty = EmbreeLidar(model, data, site_id, np.empty((0, 3))).cast(data, MAX_RANGE)

    np.testing.assert_array_equal(misses, np.empty((0, 3), dtype=np.float32))
    np.testing.assert_array_equal(empty, np.empty((0, 3), dtype=np.float32))


def _reference_points(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    directions: NDArray[np.float64],
    max_range: float,
) -> NDArray[np.float64]:
    origin = data.site_xpos[site_id]
    world = np.einsum("ij,kj->ik", directions, data.site_xmat[site_id].reshape(3, 3))
    root = model.body_rootid[model.site_bodyid[site_id]]
    robot = model.body_rootid[model.geom_bodyid] == root
    groups = model.geom_group.copy()
    model.geom_group[robot] = 5
    geom_id = np.empty(1, dtype=np.int32)
    try:
        distances = np.array(
            [
                mujoco.mj_ray(
                    model,
                    data,
                    origin,
                    ray,
                    np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8),
                    1,
                    -1,
                    geom_id,
                )
                for ray in world
            ]
        )
    finally:
        model.geom_group[:] = groups
    valid = (distances >= MIN_RANGE) & (distances <= max_range)
    valid &= np.abs(world[:, 2] * distances) <= MAX_HEIGHT
    return origin + world[valid] * distances[valid, None]


def test_mixed_scene_matches_mj_ray_and_leaves_simulation_state_unchanged(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
) -> None:
    model, data, site_id = scene
    directions = scan_directions(72, -80, 52, 20)
    qpos = data.qpos.copy()
    mocap_pos = data.mocap_pos.copy()
    xpos = data.geom_xpos.copy()
    expected = _reference_points(model, data, site_id, directions, MAX_RANGE)

    actual = EmbreeLidar(model, data, site_id, directions).cast(data, MAX_RANGE)

    np.testing.assert_allclose(actual, expected, atol=1e-4)
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.mocap_pos, mocap_pos)
    np.testing.assert_array_equal(data.geom_xpos, xpos)


def test_person_yaw_is_applied_to_cached_geometry(
    scene: tuple[mujoco.MjModel, mujoco.MjData, int],
) -> None:
    model, data, site_id = scene
    directions = scan_directions(72, -15, 15, 5)
    lidar = EmbreeLidar(model, data, site_id, directions)
    before = lidar.cast(data, MAX_RANGE)
    data.mocap_quat[0] = [np.cos(0.3), 0, 0, np.sin(0.3)]
    mujoco.mj_forward(model, data)
    expected = _reference_points(model, data, site_id, directions, MAX_RANGE)

    actual = lidar.cast(data, MAX_RANGE)

    assert not np.array_equal(before, actual)
    np.testing.assert_allclose(actual, expected, atol=1e-4)


@pytest.mark.parametrize("attributes", ['rgba="1 1 1 0"', 'group="3"', 'material="hidden"'])
def test_invisible_geometry_does_not_occlude_visible_hits(attributes: str) -> None:
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <asset><material name="hidden" rgba="1 1 1 0"/></asset>
          <worldbody>
            <body name="robot"><freejoint/><geom type="sphere" size="0.01"/>
              <site name="lidar"/>
            </body>
            <geom type="box" size="0.1 0.5 0.5" pos="1 0 0" {attributes}/>
            <geom type="box" size="0.1 0.5 0.5" pos="2 0 0" group="2"/>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lidar = EmbreeLidar(model, data, model.site("lidar").id, np.array([[1.0, 0, 0]]))

    np.testing.assert_allclose(lidar.cast(data, MAX_RANGE), [[1.9, 0, 0]], atol=1e-4)


@pytest.mark.parametrize("body_attributes,joint", [('mocap="true"', ""), ("", "<freejoint/>")])
def test_dynamic_descendant_moves_into_and_out_of_range(body_attributes: str, joint: str) -> None:
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <worldbody>
            <body name="robot"><freejoint/><geom type="sphere" size="0.01"/>
              <site name="lidar"/>
            </body>
            <body name="moving" pos="20 0 0" {body_attributes}>{joint}
              <body pos="0.25 0 0">
                <geom type="box" size="0.1 0.3 0.3"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lidar = EmbreeLidar(model, data, model.site("lidar").id, np.array([[1.0, 0, 0]]))
    empty = lidar.cast(data, MAX_RANGE)
    # Both layouts expose one position array for the moving body's translation.
    position = data.mocap_pos[0] if body_attributes else data.qpos[7:10]
    position[0] = 1.0
    mujoco.mj_forward(model, data)
    near = lidar.cast(data, MAX_RANGE)
    position[0] = 20.0
    mujoco.mj_forward(model, data)
    far = lidar.cast(data, MAX_RANGE)

    np.testing.assert_array_equal(empty, np.empty((0, 3), dtype=np.float32))
    np.testing.assert_allclose(near, [[1.15, 0, 0]], atol=1e-4)
    np.testing.assert_array_equal(far, empty)


def test_scene_containing_only_robot_returns_no_points() -> None:
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody><body><freejoint/>
          <geom type="sphere" size="1"/><site name="lidar"/>
        </body></worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lidar = EmbreeLidar(model, data, model.site("lidar").id, scan_directions(4, 0, 0, 1))

    np.testing.assert_array_equal(lidar.cast(data, MAX_RANGE), np.empty((0, 3), dtype=np.float32))


def _sorted(points: NDArray[np.float64]) -> NDArray[np.float64]:
    return points[np.lexsort((points[:, 2], points[:, 1], points[:, 0]))]


def test_voxel_down_sample_matches_open3d() -> None:
    rng = np.random.default_rng(1)
    points = np.vstack([rng.uniform(-3, 3, size=(20_000, 3)), rng.normal(0, 0.02, size=(5_000, 3))])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    expected = np.asarray(cloud.voxel_down_sample(voxel_size=LIDAR_RESOLUTION).points)

    actual = voxel_down_sample(points, LIDAR_RESOLUTION)

    assert actual.shape == expected.shape
    np.testing.assert_allclose(_sorted(actual), _sorted(expected), atol=1e-9)
    assert voxel_down_sample(np.empty((0, 3)), LIDAR_RESOLUTION).shape == (0, 3)
