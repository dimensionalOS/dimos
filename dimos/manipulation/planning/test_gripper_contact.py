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

"""Grasp contact checks on the sim xArm7 and its gripper, in a RoboPlan world.

Every scene stands on a table just below z = 0, in front of the arm, world
frame, Z up, metres. Grasps come from above: the tool tip's z axis points down.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.manipulation.planning.gripper_contact import GraspContactChecker, scene_obstacles
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.transform_utils import matrix_to_pose

CONTACT_MARGIN = 0.005
OBSTACLE_MARGIN = 0.003
# Where the objects stand, in reach of the arm.
X = 0.4
# The table's points; its octree cells fill z -6 to 0 mm.
TABLE_Z = -0.001


def _top_down(x: float, y: float, z: float) -> Pose:
    """A tool tip pose at (X + x, y, z) pointing straight down; its y axis points along world -y."""
    pose = np.eye(4)
    pose[:3, :3] = np.diag([1.0, -1.0, -1.0])
    pose[:3, 3] = (X + x, y, z)
    return matrix_to_pose(pose)


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> NDArray:
    """Points 2 mm apart on the surface of an axis-aligned box centred at (X, 0, 0) + ``center``."""
    half = np.asarray(size) / 2.0
    faces = []
    for axis in range(3):
        u, v = (a for a in range(3) if a != axis)
        grid_u = np.arange(-half[u], half[u] + 1e-9, 0.002)
        grid_v = np.arange(-half[v], half[v] + 1e-9, 0.002)
        mesh_u, mesh_v = np.meshgrid(grid_u, grid_v)
        for side in (-1.0, 1.0):
            face = np.zeros((mesh_u.size, 3))
            face[:, u], face[:, v], face[:, axis] = (
                mesh_u.ravel(),
                mesh_v.ravel(),
                side * half[axis],
            )
            faces.append(face)
    return np.concatenate(faces) + np.asarray(center) + (X, 0.0, 0.0)


def _table(hole: float) -> NDArray:
    """Table points 1 cm apart, as the camera cloud has them, minus those within ``hole``
    (metres, along x and y) of the spot where an object stands."""
    x, y = np.meshgrid(np.arange(-0.3, 0.3, 0.01), np.arange(-0.3, 0.3, 0.01))
    points = np.stack([x.ravel(), y.ravel(), np.full(x.size, TABLE_Z)], axis=1)
    under = (np.abs(points[:, 0]) < hole) & (np.abs(points[:, 1]) < hole)
    return points[~under] + (X, 0.0, 0.0)


def _clear(checker: GraspContactChecker, grasp: Pose, target: NDArray, *scene: NDArray) -> bool:
    """Whether one grasp passes, with every scene point set merged."""
    points = np.concatenate([np.empty((0, 3)), *scene])
    return checker.check([grasp], target, points, "world", CONTACT_MARGIN, OBSTACLE_MARGIN)[0]


def _xarm(closing_steps: int = 4) -> GraspContactChecker:
    from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config

    config = make_xarm7_sim_robot_config()
    assert config.gripper is not None
    gripper = config.gripper.model_copy(update={"closing_steps": closing_steps})
    return GraspContactChecker(config.model_copy(update={"gripper": gripper}), "manipulator")


@pytest.fixture(scope="module")
def xarm() -> GraspContactChecker:
    return _xarm()


@pytest.fixture(scope="module")
def xarm_open() -> GraspContactChecker:
    """Checks the open fingers only."""
    return _xarm(closing_steps=0)


# A 4 x 4 x 6 cm box.
BOX = _box((0.0, 0.0, 0.03), (0.04, 0.04, 0.06))
BOX_TABLE = _table(hole=0.02)
# Open xArm fingertips sit this far (metres) behind the tool tip, along the approach.
XARM_TIP_BEHIND_TCP = 0.0099
# The left finger's centre, along the tool tip's y axis, metres.
XARM_LEFT_FINGER_Y = 0.06


@pytest.mark.self_hosted
def test_xarm_keeps_a_grasp_with_the_pads_on_the_box_sides(xarm: GraspContactChecker) -> None:
    assert _clear(xarm, _top_down(0.0, 0.0, 0.03), BOX, BOX_TABLE)


@pytest.mark.self_hosted
def test_xarm_rejects_a_fingertip_resting_on_top_of_the_box(xarm: GraspContactChecker) -> None:
    # The left finger straight over the box, its tip on the box's top face.
    resting = _top_down(0.0, XARM_LEFT_FINGER_Y, 0.06 - XARM_TIP_BEHIND_TCP)

    assert not _clear(xarm, resting, BOX)


@pytest.mark.self_hosted
def test_xarm_rejects_pads_pressing_into_a_box_wider_than_the_open_jaws(
    xarm: GraspContactChecker,
) -> None:
    # The open pads stand 8.89 cm apart; this box is 9.2 cm across them.
    wide = _box((0.0, 0.0, 0.03), (0.04, 0.092, 0.06))

    assert not _clear(xarm, _top_down(0.0, 0.0, 0.03), wide)


@pytest.mark.self_hosted
def test_xarm_closing_sweep_catches_fingertips_swinging_into_the_table(
    xarm: GraspContactChecker, xarm_open: GraspContactChecker
) -> None:
    # A narrow block: the fingertips swing forward as the fingers close, into the
    # table before the pads reach the block. Open, they clear the table by 5 mm.
    block = _box((0.0, 0.0, 0.0125), (0.02, 0.02, 0.025))
    table = _table(hole=0.01)
    grasp = _top_down(0.0, 0.0, 0.005 - XARM_TIP_BEHIND_TCP)

    assert _clear(xarm, grasp, block)
    assert not _clear(xarm, grasp, block, table)
    assert _clear(xarm_open, grasp, block, table)


@pytest.mark.self_hosted
def test_xarm_rejects_the_table_and_a_neighbour(
    xarm: GraspContactChecker, xarm_open: GraspContactChecker
) -> None:
    # Too low on a 3 cm box: the open fingertips are already 2 mm into the table.
    short_box = _box((0.0, 0.0, 0.015), (0.04, 0.04, 0.03))
    low = _top_down(0.0, 0.0, -0.002 - XARM_TIP_BEHIND_TCP)
    # A neighbour 3.5 cm from the box, right where the left finger comes down.
    neighbour = _box((0.0, -0.07, 0.03), (0.04, 0.03, 0.06))

    assert _clear(xarm_open, low, short_box)
    assert not _clear(xarm_open, low, short_box, BOX_TABLE)
    assert not _clear(xarm, _top_down(0.0, 0.0, 0.03), BOX, BOX_TABLE, neighbour)


def test_gripper_config_steps_from_open_to_closed() -> None:
    gripper = GripperConfig(
        pad_links=("pad",), open_joints={"drive": 0.0}, closed_joints={"drive": 0.8}
    )

    assert [step["drive"] for step in gripper.joint_positions()] == pytest.approx(
        [0.0, 0.2, 0.4, 0.6, 0.8]
    )
    with pytest.raises(ValueError, match="same joints"):
        GripperConfig(pad_links=("pad",), open_joints={"drive": 0.0})


def _cloud(*points: tuple[float, float, float]) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)), frame_id="world", timestamp=1.0
    )


def _xyz(points: np.ndarray) -> set[tuple[float, float, float]]:
    return {tuple(round(float(v), 3) for v in p) for p in points}


def test_the_scene_keeps_what_is_near_the_object_but_not_on_it() -> None:
    table_z = 0.745
    # A can whose lowest seen point is 5 mm above the table.
    target = _cloud(*((0.5, 0.0, table_z + z) for z in (0.005, 0.05, 0.08)))
    table = [(0.5 + dx, dy, table_z) for dx in (-0.1, -0.05, 0.05, 0.1) for dy in (-0.1, 0.0, 0.1)]
    neighbour = (0.6, 0.0, table_z + 0.05)
    scene = _cloud(
        *table,
        neighbour,
        (0.5, 0.008, table_z + 0.05),  # the can's own edge, missed by its mask: dropped
        (0.5, 0.0, table_z + 0.25),  # the robot's own finger near its tool tip: dropped
        (1.2, 0.0, table_z),  # out of the gripper's reach: dropped
    )

    found = scene_obstacles(scene, target, [Vector3(0.5, 0.0, table_z + 0.3)])

    assert _xyz(found) == _xyz(np.asarray([*table, neighbour], dtype=np.float32))
