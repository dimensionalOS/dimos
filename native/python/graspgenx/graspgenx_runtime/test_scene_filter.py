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

"""The xArm gripper against a synthetic table, through GraspGenX's own collision filter.

The model is mocked; the collision filter is real and runs on the CPU. Grasps are
written as xArm tool-tip (TCP) poses, turned into the grasp frame GraspGenX
predicts, and must come back out as the same TCP poses.
"""

from __future__ import annotations

from collections.abc import Callable
import importlib

import graspgenx_runtime.runtime as runtime_module
from graspgenx_runtime.runtime import GraspGenXRuntimeModule
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, gripper_body_points
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.manipulators.xarm.grasp_geometry import (
    XARM_GRASP_FRAME_TO_TCP,
    XARM_GRIPPER_BODY,
    XARM_GRIPPER_SWEEP_VOLUME,
)

# The target: a 6 x 6 x 10 cm box standing on the table top (z = 0) at the origin.
TARGET_HALF = np.array([0.03, 0.03, 0.05])
TARGET_CENTER = np.array([0.0, 0.0, 0.05])
# A neighbour of the same size 4 cm away along +X.
NEIGHBOUR_CENTER = np.array([0.10, 0.0, 0.05])
# Directions in the world: the TCP's Z is where the gripper approaches, its Y the
# way the jaws close.
UP, DOWN = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])
X, Y = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])

# name -> (tool tip position, approach direction, closing direction, kept)
GRASPS: dict[str, tuple[tuple[float, float, float], np.ndarray, np.ndarray, bool]] = {
    # Straight down on the top, jaws closing along Y: clear.
    "top_down": ((0.0, 0.0, 0.05), DOWN, Y, True),
    # Straight down, jaws closing along X: the +X finger lands in the neighbour.
    "top_down_into_neighbour": ((0.0, 0.0, 0.05), DOWN, X, False),
    # Straight down so deep the fingertips reach the table.
    "top_down_to_the_table": ((0.0, 0.0, -0.002), DOWN, Y, False),
    # From the side at mid height, jaws closing level: clear.
    "side": ((0.0, 0.0, 0.06), X, Y, True),
    # From the side 2 cm above the table: the palm drives into it.
    "low_side": ((0.0, 0.0, 0.02), X, Y, False),
    # From the side with the jaws closing up and down: the lower finger is under the table.
    "side_closing_vertically": ((0.0, 0.0, 0.05), X, UP, False),
    # From below, up through the table.
    "from_below": ((0.0, 0.0, 0.03), UP, Y, False),
}


def tcp_pose(
    position: tuple[float, float, float], approach: np.ndarray, closing: np.ndarray
) -> np.ndarray:
    """A 4x4 xArm tool-tip pose from where it is and which ways it points."""
    pose = np.eye(4)
    pose[:3, 0] = np.cross(closing, approach)
    pose[:3, 1] = closing
    pose[:3, 2] = approach
    pose[:3, 3] = position
    return pose


def box_surface(center: np.ndarray, half: np.ndarray, spacing: float) -> np.ndarray:
    """Points on the faces of an axis-aligned box, as a depth camera would see them."""
    axes = [np.arange(-h, h + 1e-9, spacing) for h in half]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    on_surface = np.any(np.isclose(np.abs(grid), half, atol=spacing / 2), axis=1)
    return grid[on_surface] + center


def scene() -> np.ndarray:
    """A 1 cm table top and the neighbour, without the target or the table it hides.

    The table under the target, and a 1.5 cm band around it, are left out the way
    the camera and the target's mask leave them out.
    """
    xs = np.arange(-0.25, 0.2501, 0.01)
    table = np.stack(np.meshgrid(xs, xs, [0.0], indexing="ij"), axis=-1).reshape(-1, 3)
    hidden = np.all(np.abs(table[:, :2]) <= TARGET_HALF[:2] + 0.015, axis=1)
    near_neighbour = np.all(np.abs(table[:, :2] - NEIGHBOUR_CENTER[:2]) <= TARGET_HALF[:2], axis=1)
    table = table[~hidden & ~near_neighbour]
    neighbour = box_surface(NEIGHBOUR_CENTER, TARGET_HALF, 0.01)
    return np.concatenate([table, neighbour]).astype(np.float32)


@pytest.fixture(scope="module")
def collision_free(tmp_path_factory: pytest.TempPathFactory) -> Callable[..., np.ndarray]:
    # GraspGenX clones its assets on import unless these name existing directories.
    empty = tmp_path_factory.mktemp("graspgenx")
    with pytest.MonkeyPatch.context() as env:
        env.setenv("GRASPGENX_CHECKPOINT_DIR", str(empty))
        env.setenv("GRASPGENX_GRIPPER_CFG_DIR", str(empty))
        scene_filter = importlib.import_module("graspgenx_runtime.scene_filter")
    check: Callable[..., np.ndarray] = scene_filter.collision_free
    return check


def config() -> GraspGenXConfig:
    return GraspGenXConfig(
        gripper=XARM_GRIPPER_SWEEP_VOLUME,  # type: ignore[arg-type]
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
        gripper_body=XARM_GRIPPER_BODY,  # type: ignore[arg-type]
    )


def test_the_xarm_gripper_body_is_checked_in_the_grasp_frame(
    mocker: MockerFixture, collision_free: Callable[..., np.ndarray]
) -> None:
    names = list(GRASPS)
    tcp = np.stack([tcp_pose(*GRASPS[name][:3]) for name in names])
    # What GraspGenX would predict for these tool tips.
    grasp_frame = tcp @ np.linalg.inv(np.asarray(XARM_GRASP_FRAME_TO_TCP))
    scores = np.linspace(1.0, 0.5, len(names))
    cfg = config()
    points = gripper_body_points(cfg.gripper_body)
    backend = mocker.patch.object(runtime_module, "_create_runtime").return_value
    backend.infer.return_value = (grasp_frame, scores)
    backend.collision_free.side_effect = lambda scene_points, poses, threshold: collision_free(
        scene_points, poses, points, threshold, device="cpu"
    )
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True,
        **cfg.model_dump(exclude={"rpc_transport", "tf_transport", "g"}),
    )
    target = PointCloud2.from_numpy(
        box_surface(TARGET_CENTER, TARGET_HALF, 0.005).astype(np.float32),
        frame_id="world",
        timestamp=1.0,
    )
    try:
        module.start()
        result = module.propose_grasps_in_scene(
            target, PointCloud2.from_numpy(scene(), frame_id="world", timestamp=1.0), 0.01
        )
    finally:
        module.stop()

    kept = [name for name in names if GRASPS[name][3]]
    assert result.removed == len(names) - len(kept)
    by_score = {round(float(score), 6): name for name, score in zip(names, scores, strict=True)}
    assert [by_score[round(c.score, 6)] for c in result.grasps] == kept
    for candidate in result.grasps:
        name = by_score[round(candidate.score, 6)]
        expected = tcp[names.index(name)]
        assert candidate.pose.position.to_numpy() == pytest.approx(expected[:3, 3], abs=1e-6)
        rotation = candidate.pose.orientation.to_rotation_matrix()
        assert np.asarray(rotation) == pytest.approx(expected[:3, :3], abs=1e-6)


def test_an_empty_scene_keeps_every_grasp(collision_free: Callable[..., np.ndarray]) -> None:
    points = gripper_body_points(config().gripper_body)
    poses = np.repeat(np.eye(4)[None], 3, axis=0)

    clear = collision_free(np.empty((0, 3)), poses, points, 0.01, device="cpu")

    assert clear.tolist() == [True, True, True]


def test_the_fingers_close_along_the_tool_tip_y_axis() -> None:
    # Gripper points in the TCP frame: the fingers lie along +-Y, just behind the tip.
    points = gripper_body_points(config().gripper_body).astype(np.float64)
    in_tcp = (
        np.linalg.inv(np.asarray(XARM_GRASP_FRAME_TO_TCP)) @ np.c_[points, np.ones(len(points))].T
    ).T
    fingers = in_tcp[in_tcp[:, 2] > -0.045]

    assert np.abs(fingers[:, 1]).min() == pytest.approx(0.0445, abs=1e-3)
    assert np.abs(fingers[:, 1]).max() == pytest.approx(0.0765, abs=1e-3)
    assert np.abs(fingers[:, 0]).max() == pytest.approx(0.016, abs=1e-3)
    assert fingers[:, 2].max() == pytest.approx(-0.0098, abs=1e-3)
