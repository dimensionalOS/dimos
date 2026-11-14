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

"""The simulated connection's ports, joined to the test by in-memory transports."""

from collections.abc import Callable, Iterator
from dataclasses import dataclass
import math
from typing import Any

import numpy as np
import pytest

from dimos.control.contract.keys import Interface
from dimos.core.coordination.blueprints import BlueprintAtom
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_CAMERA_TRANSFORM,
    DUAL_OPENYAM_TABLE_TOP_Z,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection
from dimos.utils.testing.waiting import wait_until

JOINTS = [f"arm/{name}" for name in DUAL_OPENYAM.joint_names]
CAMERA_PORTS = (
    "color_image",
    "depth_image",
    "camera_info",
    "depth_camera_info",
    "left_wrist_color_image",
    "left_wrist_camera_info",
    "right_wrist_color_image",
    "right_wrist_camera_info",
    "tf",
)


@dataclass
class Deployed:
    """A started connection, where to send it commands and all it has published per port."""

    connection: DualOpenYamSimConnection
    commands: LocalTransport[JointState]
    published: dict[str, list[Any]]

    def positions(self) -> dict[str, float]:
        reading = self.published["joint_state"][-1]
        return dict(zip(reading.name, reading.position, strict=True))

    def command(self, targets: dict[str, float]) -> None:
        self.commands.broadcast(
            None, JointState(name=list(targets), position=list(targets.values()))
        )


Deploy = Callable[..., Deployed]


@pytest.fixture
def deploy(no_rpc: None) -> Iterator[Deploy]:
    """Start a simulated connection whose every port has an in-memory transport."""
    started = []

    def start(**config: Any) -> Deployed:
        connection = DualOpenYamSimConnection(source="arm", **config)
        started.append(connection)
        commands = LocalTransport[JointState]()
        connection.set_transport("position_command", commands)
        published: dict[str, list[Any]] = {}
        for name in connection.outputs:
            port = LocalTransport[Any]()
            published[name] = []
            port.subscribe(published[name].append)
            connection.set_transport(name, port)
        connection.start()
        wait_until(lambda: bool(published["joint_state"]), timeout=10.0)
        return Deployed(connection, commands, published)

    yield start
    for connection in started:
        connection.stop()


def test_stands_in_for_the_damiao_connection_port_for_port(no_rpc: None) -> None:
    real = {
        stream.name: stream for stream in BlueprintAtom.create(DualOpenYamConnection, {}).streams
    }
    simulated = BlueprintAtom.create(DualOpenYamSimConnection, {}).streams

    assert set(real) == {"position_command", "joint_state"}
    assert [stream for stream in simulated if stream.name in real] == list(real.values())
    assert {stream.name for stream in simulated} - set(real) == {*CAMERA_PORTS, "pointcloud"}
    assert DualOpenYamSimConnection.robot is DualOpenYamConnection.robot

    connection = DualOpenYamSimConnection()
    try:
        assert connection.config.source == "dual_openyam"
        assert connection.config.model is DUAL_OPENYAM.model
        assert connection.config.joint_commands == {
            name: (Interface.POSITION,) for name in DUAL_OPENYAM.joint_names
        }
    finally:
        connection.stop()


def test_accepts_position_commands_only(no_rpc: None) -> None:
    with pytest.raises(ValueError, match="position commands only"):
        DualOpenYamSimConnection(joint_commands={"left_joint1": (Interface.VELOCITY,)})


@pytest.mark.self_hosted
def test_reports_every_joint_and_follows_position_commands(deploy: Deploy) -> None:
    rig = deploy(cameras=False, deadman_timeout_s=30.0)

    first = rig.published["joint_state"][0]
    assert first.name == JOINTS
    assert first.frame_id == "dual_openyam_base"
    assert first.position == pytest.approx([*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0], abs=1e-3)
    assert len(first.velocity) == len(first.effort) == len(JOINTS)

    target = {
        "arm/left_joint1": 0.5,
        "arm/right_joint1": -0.5,
        "arm/left_gripper": 1.0,
        "arm/right_gripper": 0.4,
    }
    rig.command(target)

    wait_until(
        lambda: all(abs(rig.positions()[name] - goal) < 0.01 for name, goal in target.items()),
        timeout=10.0,
    )
    # The joints the command left out stay where they were.
    assert rig.positions()["arm/left_joint2"] == pytest.approx(
        DUAL_OPENYAM_HOME_JOINTS[1], abs=1e-3
    )
    assert rig.connection.status().last_rejection is None


@pytest.mark.self_hosted
def test_refuses_a_command_past_a_joint_limit(deploy: Deploy) -> None:
    rig = deploy(cameras=False, deadman_timeout_s=30.0)

    # The elbow's range starts at 0 rad.
    rig.command({"arm/left_joint3": -0.5})

    wait_until(lambda: rig.connection.status().last_rejection is not None, timeout=10.0)
    assert rig.positions()["arm/left_joint3"] == pytest.approx(
        DUAL_OPENYAM_HOME_JOINTS[2], abs=1e-3
    )


@pytest.mark.self_hosted
def test_holds_where_it_is_when_commands_stop(deploy: Deploy) -> None:
    rig = deploy(cameras=False, deadman_timeout_s=0.1)

    # One command to swing the left arm far out, then silence.
    rig.command({"arm/left_joint1": 1.5})

    wait_until(lambda: rig.connection.status().deadman_fired, timeout=10.0)
    wait_until(
        lambda: max(abs(speed) for speed in rig.published["joint_state"][-1].velocity) < 0.01,
        timeout=10.0,
    )
    assert 0.05 < rig.positions()["arm/left_joint1"] < 1.3


@pytest.mark.self_hosted
def test_says_where_an_object_is_and_stands_it_somewhere_else(deploy: Deploy) -> None:
    connection = deploy(cameras=False, deadman_timeout_s=30.0).connection

    # The cube's frame is at its centre, half its 5.7 cm above the table.
    resting = DUAL_OPENYAM_TABLE_TOP_Z + 0.0285
    assert connection.body_position("rubiks_cube") == pytest.approx(
        [0.37, -0.09, resting], abs=2e-3
    )

    connection.place_body("rubiks_cube", [0.30, 0.10, resting], math.radians(45.0))

    assert connection.body_position("rubiks_cube") == pytest.approx([0.30, 0.10, resting], abs=2e-3)
    # An arm's link is a body too, but not one to pick up and put down.
    with pytest.raises(ValueError, match="no free body"):
        connection.place_body("left_gripper", [0.30, 0.10, resting])
    with pytest.raises(ValueError, match="no free body"):
        connection.place_body("mug", [0.30, 0.10, resting])


@pytest.mark.self_hosted
def test_publishes_each_camera_on_its_realsense_port_and_frames(
    deploy: Deploy, mujoco_gl: None
) -> None:
    published = deploy(deadman_timeout_s=30.0).published
    wait_until(lambda: all(published[name] for name in CAMERA_PORTS), timeout=20.0)

    color, depth = published["color_image"][-1], published["depth_image"][-1]
    assert color.frame_id == depth.frame_id == "camera_color_optical_frame"
    assert (color.format, color.data.shape, color.data.dtype) == (
        ImageFormat.RGB,
        (480, 640, 3),
        np.uint8,
    )
    assert (depth.format, depth.data.shape, depth.data.dtype) == (
        ImageFormat.DEPTH,
        (480, 640),
        np.float32,
    )
    info = published["camera_info"][-1]
    assert (info.width, info.height, info.frame_id) == (640, 480, "camera_color_optical_frame")
    # A D405's 58 degree vertical field of view at 480 rows.
    assert info.K == pytest.approx([433.0, 0.0, 320.0, 0.0, 433.0, 240.0, 0.0, 0.0, 1.0], abs=0.1)
    assert published["depth_camera_info"][-1].K == info.K
    for side in ("left", "right"):
        wrist = published[f"{side}_wrist_color_image"][-1]
        assert wrist.frame_id == f"{side}_wrist/camera_color_optical_frame"
        assert wrist.data.shape == (480, 640, 3)
        assert published[f"{side}_wrist_camera_info"][-1].frame_id == wrist.frame_id

    edges: dict[str, Transform] = {
        transform.child_frame_id: transform for transform in published["tf"][0].transforms
    }
    assert {child: edge.frame_id for child, edge in edges.items()} == {
        "camera_link": "world",
        "camera_color_frame": "camera_link",
        "camera_color_optical_frame": "camera_color_frame",
        "camera_depth_frame": "camera_link",
        "camera_depth_optical_frame": "camera_depth_frame",
        "left_wrist/camera_link": "world",
        "left_wrist/camera_color_frame": "left_wrist/camera_link",
        "left_wrist/camera_color_optical_frame": "left_wrist/camera_color_frame",
        "right_wrist/camera_link": "world",
        "right_wrist/camera_color_frame": "right_wrist/camera_link",
        "right_wrist/camera_color_optical_frame": "right_wrist/camera_color_frame",
    }
    # The overhead camera's camera_link sits where the rig's calibration puts it.
    mount = edges["camera_link"]
    calibrated = DUAL_OPENYAM_CAMERA_TRANSFORM
    assert mount.translation.to_numpy() == pytest.approx(
        calibrated.translation.to_numpy(), abs=1e-6
    )
    turn = calibrated.rotation.to_numpy() / np.linalg.norm(calibrated.rotation.to_numpy())
    assert abs(np.dot(mount.rotation.to_numpy(), turn)) == pytest.approx(1.0, abs=1e-9)
    assert edges["camera_color_optical_frame"].rotation == OPTICAL_ROTATION
    # Depth, intrinsics and tf agree: the pixel at the image centre is a point of the table top.
    row, column = 240, 320
    in_optical = float(depth.data[row, column]) * np.array(
        [(column + 0.5 - info.K[2]) / info.K[0], (row + 0.5 - info.K[5]) / info.K[4], 1.0]
    )
    optical_in_world = mount.rotation.to_rotation_matrix() @ OPTICAL_ROTATION.to_rotation_matrix()
    on_table = mount.translation.to_numpy() + optical_in_world @ in_optical
    assert on_table[2] == pytest.approx(DUAL_OPENYAM_TABLE_TOP_Z, abs=1e-3)
    # Each wrist camera rides above its own arm's gripper at the home posture.
    assert edges["left_wrist/camera_link"].translation.y > 0.15
    assert edges["right_wrist/camera_link"].translation.y < -0.15
    assert edges["left_wrist/camera_link"].translation.z > 0.4
    # One stamp covers a frame set: its images and every edge of its tf tree.
    first = published["color_image"][0]
    assert {edge.ts for edge in edges.values()} == {first.ts}
    assert published["left_wrist_color_image"][0].ts == first.ts


@pytest.mark.self_hosted
def test_publishes_the_overhead_view_as_a_realsense_builds_its_point_cloud(
    deploy: Deploy, mujoco_gl: None
) -> None:
    published = deploy(deadman_timeout_s=30.0, enable_pointcloud=True, depth_trunc_m=0.6).published
    wait_until(
        lambda: len(published["pointcloud"]) >= 3 and bool(published["camera_info"]), timeout=20.0
    )

    cloud = published["pointcloud"][-1]
    # It has the frame and the stamp of the depth image it was built from.
    [depth] = [image for image in published["depth_image"] if image.ts == cloud.ts]
    [color] = [image for image in published["color_image"] if image.ts == cloud.ts]
    assert cloud.frame_id == depth.frame_id == "camera_color_optical_frame"
    # Every second pixel is put back through the pinhole, and nothing past the cut-off.
    info = published["camera_info"][-1]
    rows, columns = np.mgrid[0:480:2, 0:640:2]
    distance = depth.data[::2, ::2]
    near = distance <= 0.6
    assert 0.3 < near.mean() < 0.95
    points = cloud.points_f32()
    assert points == pytest.approx(
        np.column_stack(
            [
                (columns[near] - info.K[2]) * distance[near] / info.K[0],
                (rows[near] - info.K[5]) * distance[near] / info.K[4],
                distance[near],
            ]
        ),
        abs=1e-6,
    )
    colors = cloud.pointcloud_tensor.point["colors"].numpy()
    assert colors == pytest.approx(color.data[::2, ::2][near] / 255.0, abs=1e-6)
    # Through the camera's mount the points are the scene: an empty stretch of
    # the table, out to the picture's edge, lies flat at the table's height.
    mount = DUAL_OPENYAM_CAMERA_TRANSFORM
    optical_in_world = mount.rotation.to_rotation_matrix() @ OPTICAL_ROTATION.to_rotation_matrix()
    in_world = points @ optical_in_world.T + mount.translation.to_numpy()
    on_table = (in_world[:, 0] > 0.2) & (in_world[:, 0] < 0.3) & (np.abs(in_world[:, 1]) < 0.15)
    assert on_table.sum() > 1000
    assert in_world[on_table, 2] == pytest.approx(DUAL_OPENYAM_TABLE_TOP_Z, abs=1.5e-3)
    # Clouds come at their own rate, 5 Hz by default, not with every frame.
    stamps = [cloud.ts for cloud in published["pointcloud"]]
    assert np.diff(stamps).min() >= 0.2


@pytest.mark.self_hosted
def test_leaves_the_overhead_cameras_mount_to_the_planner_when_told_to(
    deploy: Deploy, mujoco_gl: None
) -> None:
    published = deploy(deadman_timeout_s=30.0, publish_overhead_mount=False).published
    wait_until(lambda: bool(published["tf"]), timeout=20.0)

    parents = {edge.child_frame_id: edge.frame_id for edge in published["tf"][0].transforms}
    # The imagers hang off camera_link as before; where that is, the planner says.
    assert "camera_link" not in parents
    assert parents["camera_color_frame"] == parents["camera_depth_frame"] == "camera_link"
    # The wrist cameras ride the arms, and only the rig knows where they are.
    assert parents["left_wrist/camera_link"] == parents["right_wrist/camera_link"] == "world"
    assert not published["pointcloud"]
