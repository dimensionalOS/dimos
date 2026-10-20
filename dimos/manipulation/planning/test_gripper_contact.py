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

"""Grasp contact checks on the xArm Gripper and on a made-up three-finger hand.

Every scene stands on a table at z = 0, world frame, Z up, metres. Grasps come
from above: the tool tip's z axis points down.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.planning.gripper_contact import GripperContactModel, approach_offsets
from dimos.manipulation.planning.spec.gripper import ContactPatch, GraspContact, GripperSpec
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import RobotModel
from dimos.utils.transform_utils import pose_to_matrix

CONTACT_MARGIN = 0.005
OBSTACLE_MARGIN = 0.003
CLEAR = GraspContact()


def _top_down(x: float, y: float, z: float, yaw: float = 0.0) -> NDArray[np.float64]:
    """A tool tip pose at (x, y, z) pointing straight down, turned ``yaw`` radians about Z."""
    c, s = np.cos(yaw), np.sin(yaw)
    pose = np.eye(4)
    # Half a turn about X (tool z down), then the yaw about world Z.
    pose[:3, :3] = np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])
    pose[:3, 3] = (x, y, z)
    return pose


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> NDArray:
    """Points 2 mm apart on the surface of an axis-aligned box."""
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
    return np.concatenate(faces) + np.asarray(center)


def _cylinder(radius: float, height: float) -> NDArray:
    """Points about 2 mm apart on an upright cylinder's side and top, standing on the table."""
    angles = np.linspace(0.0, 2 * np.pi, int(2 * np.pi * radius / 0.002), endpoint=False)
    heights = np.arange(0.0, height + 1e-9, 0.002)
    a, h = np.meshgrid(angles, heights)
    side = np.stack([radius * np.cos(a.ravel()), radius * np.sin(a.ravel()), h.ravel()], axis=1)
    r, a = np.meshgrid(np.arange(0.0, radius, 0.002), angles)
    top = np.stack([(r * np.cos(a)).ravel(), (r * np.sin(a)).ravel(), np.full(r.size, height)], 1)
    return np.concatenate([side, top])


def _table(*, hole: tuple[float, float, float, float] | None = None) -> NDArray:
    """Table points 1 cm apart, as the camera cloud has them, minus the points under ``hole``
    (x_low, x_high, y_low, y_high), where an object stands."""
    x, y = np.meshgrid(np.arange(-0.3, 0.3, 0.01), np.arange(-0.3, 0.3, 0.01))
    points = np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], axis=1)
    if hole is not None:
        x0, x1, y0, y1 = hole
        under = (
            (points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
        )
        points = points[~under]
    return points


def _check(
    model: GripperContactModel,
    grasp: NDArray,
    target: NDArray,
    **obstacles: NDArray,
) -> GraspContact:
    return model.check(grasp[None], target, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN)[0]


# The xArm Gripper, from the xArm description the planner loads.


@pytest.fixture(scope="module")
def xarm() -> GripperContactModel:
    from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config

    config = make_xarm7_sim_robot_config()
    assert config.gripper is not None
    return GripperContactModel(config.model.load(), config.gripper, "link_tcp")


# A 4 x 4 x 6 cm box standing at the origin.
BOX = _box((0.0, 0.0, 0.03), (0.04, 0.04, 0.06))
BOX_TABLE = _table(hole=(-0.02, 0.02, -0.02, 0.02))
# Open xArm fingertips sit this far (metres) behind the tool tip, along the approach.
XARM_TIP_BEHIND_TCP = 0.0099
# The left finger's centre, along the tool tip's y axis, metres.
XARM_LEFT_FINGER_Y = 0.06


@pytest.mark.self_hosted
def test_xarm_keeps_a_grasp_with_the_pads_on_the_box_sides(xarm: GripperContactModel) -> None:
    assert _check(xarm, _top_down(0.0, 0.0, 0.03), BOX, table=BOX_TABLE) == CLEAR


@pytest.mark.self_hosted
def test_xarm_rejects_a_fingertip_resting_on_top_of_the_box(xarm: GripperContactModel) -> None:
    # The left finger straight over the box, its tip on the box's top face. The
    # tool tip's y axis points along world -y when it points down.
    resting = _top_down(0.0, XARM_LEFT_FINGER_Y, 0.06 - XARM_TIP_BEHIND_TCP)

    assert _check(xarm, resting, BOX, table=BOX_TABLE) == GraspContact(("target",))


@pytest.mark.self_hosted
def test_xarm_rejects_pads_pressing_into_a_box_wider_than_the_open_jaws(
    xarm: GripperContactModel,
) -> None:
    # The open pads stand 8.89 cm apart; this box is 9.2 cm across them.
    wide = _box((0.0, 0.0, 0.03), (0.04, 0.092, 0.06))
    table = _table(hole=(-0.02, 0.02, -0.046, 0.046))

    assert _check(xarm, _top_down(0.0, 0.0, 0.03), wide, table=table) == GraspContact(("target",))


@pytest.mark.self_hosted
def test_xarm_closing_sweep_catches_fingertips_swinging_into_the_table(
    xarm: GripperContactModel,
) -> None:
    # A narrow block: the pads meet it only near the end of the close, after the
    # fingertips have swung forward about 1.3 cm. Open, they clear the table by 5 mm.
    block = _box((0.0, 0.0, 0.0175), (0.02, 0.02, 0.035))
    table = _table(hole=(-0.01, 0.01, -0.01, 0.01))
    grasp = _top_down(0.0, 0.0, 0.005 - XARM_TIP_BEHIND_TCP)

    assert _check(xarm, grasp, block, table=table) == GraspContact(("table",), while_closing=True)


@pytest.mark.self_hosted
def test_xarm_rejects_the_table_and_a_neighbour(xarm: GripperContactModel) -> None:
    # Too low on a 3 cm box: the open fingertips are already 2 mm into the table.
    short_box = _box((0.0, 0.0, 0.015), (0.04, 0.04, 0.03))
    low = _top_down(0.0, 0.0, -0.002 - XARM_TIP_BEHIND_TCP)
    # A neighbour 3.5 cm from the box, right where the left finger comes down.
    neighbour = _box((0.0, -0.07, 0.03), (0.04, 0.03, 0.06))

    assert _check(xarm, low, short_box, table=BOX_TABLE) == GraspContact(("table",))
    assert _check(xarm, _top_down(0.0, 0.0, 0.03), BOX, table=BOX_TABLE, objects=neighbour) == (
        GraspContact(("objects",))
    )


@pytest.mark.self_hosted
def test_xarm_checks_a_hundred_grasps_at_once(xarm: GripperContactModel) -> None:
    heights = np.linspace(0.0, 0.12, 100)
    grasps = np.stack([_top_down(0.0, 0.0, z, yaw=z * 20.0) for z in heights])

    results = xarm.check(grasps, BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN)

    assert len(results) == 100
    # Low grasps press the knuckles onto the box top; high ones close above it.
    assert "target" in results[0].touches
    assert results[-1] == CLEAR


@pytest.mark.self_hosted
def test_xarm_approach_offsets_match_the_finger_mesh(xarm: GripperContactModel) -> None:
    from dimos.robot.manipulators.xarm.config import (
        XARM_APPROACH_OFFSETS,
        make_xarm7_sim_robot_config,
    )

    config = make_xarm7_sim_robot_config()
    assert config.gripper is not None
    offsets = approach_offsets(config.model.load(), config.gripper, "link_tcp")
    # Measured on the left finger mesh, in its frame (see xarm_gripper_spec):
    # the pad face's centre and the fingertip's end.
    face = np.array([0.0, -0.026, 0.041, 1.0])
    fingertip = np.array([0.0, 0.0, 0.061, 1.0])
    closed = xarm.poses - 1
    pad_behind = -(xarm.tip_from_link(closed, "left_finger") @ face)[2]
    reach = max((xarm.tip_from_link(s, "left_finger") @ fingertip)[2] for s in range(xarm.poses))

    assert pad_behind == pytest.approx(0.0175, abs=0.0005)
    assert reach == pytest.approx(0.003, abs=0.0005)
    assert offsets.pad_behind_tcp == pytest.approx(pad_behind, abs=0.001)
    assert offsets.fingertip_reach == pytest.approx(reach, abs=0.001)
    assert np.allclose(offsets, XARM_APPROACH_OFFSETS, atol=0.0001)


def _cloud(points: NDArray) -> PointCloud2:
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id="world", timestamp=1.0)


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    "target, radius, aimed_only",
    [
        # A tall box, 5 x 5 x 16 cm: at mid-height the tool tip leaves the palm
        # on the box's top.
        (_box((0.0, 0.0, 0.08), (0.05, 0.05, 0.16)), 0.025, True),
        # A short can, 6 cm across and 3.3 cm tall.
        (_cylinder(0.03, 0.033), 0.03, False),
        # A mug-like cylinder, 8 cm across and 10 cm tall.
        (_cylinder(0.04, 0.10), 0.04, False),
    ],
    ids=["tall box", "short can", "mug"],
)
def test_xarm_heuristic_grasps_aiming_the_pads_pass_the_contact_check(
    xarm: GripperContactModel, target: NDArray, radius: float, aimed_only: bool
) -> None:
    from dimos.robot.manipulators.xarm.config import XARM_APPROACH_OFFSETS

    table = _table(hole=(-radius, radius, -radius, radius))

    def checked(**config: Any) -> list[GraspContact]:
        module = HeuristicGraspModule(**config)
        try:
            grasps = module.propose_grasps(_cloud(target)).candidates
        finally:
            module.stop()
        poses = np.stack([pose_to_matrix(grasp.pose) for grasp in grasps])
        return xarm.check(poses, target, {"table": table}, CONTACT_MARGIN, OBSTACLE_MARGIN)

    aimed = checked(**XARM_APPROACH_OFFSETS._asdict())
    at_tool_tip = checked(heuristic_aim_pads=False)

    assert aimed == [CLEAR] * len(aimed)
    assert (at_tool_tip[0] != CLEAR) is aimed_only


# A made-up three-finger hand: a palm and three fingers 120 degrees apart, each
# hinged at the palm's rim. Open, the fingers splay 0.4 rad outward; closed, they
# lean 0.4 rad inward, so their tips swing forward and back as they close.

_FINGER_LENGTH = 0.08
_HAND_URDF = """<?xml version="1.0"?>
<robot name="hand">
  <link name="palm">
    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.1 0.1 0.02"/></geometry></collision>
  </link>
  <link name="tool"/>
  <joint name="tool_joint" type="fixed">
    <parent link="palm"/><child link="tool"/><origin xyz="0 0 0.09" rpy="0 0 0"/>
  </joint>
  {fingers}
</robot>
"""
_FINGER = """
  <link name="mount_{i}"/>
  <joint name="mount_{i}_joint" type="fixed">
    <parent link="palm"/><child link="mount_{i}"/><origin xyz="0 0 0" rpy="0 0 {yaw}"/>
  </joint>
  <link name="finger_{i}">
    <collision>
      <origin xyz="0 0 0.04" rpy="0 0 0"/><geometry><box size="0.01 0.015 0.08"/></geometry>
    </collision>
  </link>
  <joint name="finger_{i}_joint" type="revolute">
    <parent link="mount_{i}"/><child link="finger_{i}"/>
    <origin xyz="0.04 0 0.01" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-0.4" upper="0.4" effort="1" velocity="1"/>
    {mimic}
  </joint>
"""


@pytest.fixture(scope="module")
def hand_urdf(tmp_path_factory: pytest.TempPathFactory) -> Path:
    fingers = "".join(
        _FINGER.format(
            i=i,
            yaw=i * 2.0 * np.pi / 3.0,
            mimic="" if i == 0 else '<mimic joint="finger_0_joint" multiplier="1" offset="0"/>',
        )
        for i in range(3)
    )
    path: Path = tmp_path_factory.mktemp("hand") / "hand.urdf"
    path.write_text(_HAND_URDF.format(fingers=fingers))
    return path


def _hand(path: Path, closing_steps: int = 4) -> GripperContactModel:
    # Each finger's inner face lies at x = -0.005 in its own frame. The patch
    # covers it from 2 cm up to the tip, grown 6 mm along the face and past
    # the tip, 8 mm out toward the object and 4 mm into the finger.
    spec = GripperSpec(
        contact_patches=tuple(
            ContactPatch(
                link=f"finger_{i}", center=(-0.007, 0.0, 0.053), size=(0.012, 0.027, 0.066)
            )
            for i in range(3)
        ),
        open_joints={"finger_0_joint": -0.4},
        closed_joints={"finger_0_joint": 0.4},
        closing_steps=closing_steps,
    )
    return GripperContactModel(RobotModel.from_file(path).load(), spec, "tool")


@pytest.fixture(scope="module")
def hand(hand_urdf: Path) -> GripperContactModel:
    return _hand(hand_urdf)


# A can 4 cm across and 5 cm tall, standing at the origin.
CAN = _cylinder(0.02, 0.05)
CAN_TABLE = _table(hole=(-0.02, 0.02, -0.02, 0.02))


def _lowest_fingertip(hand: GripperContactModel, step: int) -> float:
    """How far (metres) the fingers reach past the tool tip, along the approach, at one pose."""
    corners = [
        (hand.tip_from_link(step, link) @ np.array([x, y, z, 1.0]))[2]
        for link in hand.links
        if link.startswith("finger")
        for x in (-0.005, 0.005)
        for y in (-0.0075, 0.0075)
        for z in (0.0, _FINGER_LENGTH)
    ]
    return float(max(corners))


def test_three_finger_hand_keeps_a_grasp_around_the_can(hand: GripperContactModel) -> None:
    assert hand.links == ["finger_0", "finger_1", "finger_2", "palm"]
    assert _check(hand, _top_down(0.0, 0.0, 0.04), CAN, table=CAN_TABLE) == CLEAR


def test_three_finger_hand_rejects_the_palm_on_the_can(hand: GripperContactModel) -> None:
    tall_can = _cylinder(0.02, 0.10)
    # The palm's face is 8 cm behind the tool tip: 1 mm above the can's top.
    assert _check(hand, _top_down(0.0, 0.0, 0.021), tall_can, table=CAN_TABLE) == GraspContact(
        ("target",)
    )


def test_three_finger_hand_sweep_catches_the_fingertips_swinging_into_the_table(
    hand: GripperContactModel, hand_urdf: Path
) -> None:
    swing = _lowest_fingertip(hand, 2) - _lowest_fingertip(hand, 0)
    assert swing > 0.004
    # Open, the fingertips clear the table by 4 mm; halfway through the close they reach it.
    grasp = _top_down(0.0, 0.0, 0.004 + _lowest_fingertip(hand, 0))
    open_only = _hand(hand_urdf, closing_steps=0)

    assert _check(hand, grasp, CAN, table=CAN_TABLE) == GraspContact(("table",), while_closing=True)
    assert _check(open_only, grasp, CAN, table=CAN_TABLE) == CLEAR


def test_approach_offsets_of_the_three_finger_hand(hand_urdf: Path) -> None:
    spec = GripperSpec(
        contact_patches=tuple(
            ContactPatch(
                link=f"finger_{i}", center=(-0.007, 0.0, 0.053), size=(0.012, 0.027, 0.066)
            )
            for i in range(3)
        ),
        open_joints={"finger_0_joint": -0.4},
        closed_joints={"finger_0_joint": 0.4},
    )

    offsets = approach_offsets(RobotModel.from_file(hand_urdf).load(), spec, "tool")

    # The tool tip is 9 cm above the palm's centre and each finger hinges 1 cm
    # above it. Closed, a finger leans 0.4 rad inward, taking its patch centre
    # (x -0.007, z 0.053 in the finger's frame) to this height above the hinge.
    patch_up = 0.053 * math.cos(0.4) - 0.007 * math.sin(0.4)
    assert offsets.pad_behind_tcp == pytest.approx(0.09 - 0.01 - patch_up)
    # Upright the fingertip is level with the tool tip; leaning a little, its
    # 1 cm wide end reaches a hair farther.
    assert offsets.fingertip_reach == pytest.approx(math.hypot(0.08, 0.005) - 0.08, abs=1e-5)
    # The palm's face, 1 cm above its centre.
    assert offsets.palm_behind_tcp == pytest.approx(0.08)


def test_three_finger_hand_rejects_a_neighbour_under_a_finger(hand: GripperContactModel) -> None:
    # Finger 0 hangs along +x; a neighbour stands right where it comes down.
    neighbour = _box((0.07, 0.0, 0.03), (0.03, 0.03, 0.06))

    assert _check(
        hand, _top_down(0.0, 0.0, 0.04), CAN, table=CAN_TABLE, objects=neighbour
    ) == GraspContact(("objects",))


def test_a_spec_naming_a_link_the_urdf_lacks_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "hand.urdf"
    path.write_text(_HAND_URDF.format(fingers=""))
    spec = GripperSpec(
        contact_patches=(ContactPatch(link="finger_9", center=(0, 0, 0), size=(1, 1, 1)),)
    )

    with pytest.raises(ValueError, match="finger_9"):
        GripperContactModel(RobotModel.from_file(path).load(), spec, "tool")


def test_gripper_spec_steps_from_open_to_closed() -> None:
    spec = GripperSpec(
        contact_patches=(ContactPatch(link="pad", center=(0, 0, 0), size=(0.01, 0.01, 0.01)),),
        open_joints={"drive": 0.0},
        closed_joints={"drive": 0.8},
    )

    assert [step["drive"] for step in spec.joint_positions()] == pytest.approx(
        [0.0, 0.2, 0.4, 0.6, 0.8]
    )
    with pytest.raises(ValueError, match="same joints"):
        GripperSpec(contact_patches=spec.contact_patches, open_joints={"drive": 0.0})
