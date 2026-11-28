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

import asyncio
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("pxr")
pytest.importorskip("mujoco")

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.body_sim import G1_BODY, G1_IMU, G1_ODOM_TF, G1BodySim
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
from dimos.sim2.actions import JointCommand, read_actions
from dimos.sim2.body import is_site
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.drives import usd_drives
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.sim import compose_usds, fetched_usd
from dimos.sim2.world_observations import WorldWriter

SCENE = Path(__file__).parents[3] / "sim2" / "mujoco" / "assets" / "scene.usda"
MOUNT = "scene/g1"
MOUNT_XYZ = (1.0, -2.0, 0.5)
MOUNT_YAW = 0.7


@pytest.fixture
def g1() -> Any:
    body = G1BodySim(attached_to="g1", instance_name=f"{MOUNT}/G1BodySim")
    yield body
    body.stop()


def _providing_hardware_files(module: Any) -> str:
    """Wire ``module``'s hardware_file streams to the topics a blueprint gives them and answer on them; its namespace."""
    from dimos.core.hardware_files import hardware_file_topics
    from dimos.core.transport_factory import make_transport
    from dimos.msgs.files.FileInfo import FileInfo

    request_topic, response_topic = hardware_file_topics(module.namespace)
    module.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    module.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    module.provide_hardware_files()
    return str(module.namespace)


def test_its_usd_drives_every_joint_the_controller_commands(g1: G1BodySim) -> None:
    usd = g1.config.usd
    assert set(usd_drives(usd.usd, usd.asset_root)) == {
        G1BodySim.usd_joint(name) for name in G1_JOINT_NAMES
    }


def test_its_lidar_and_camera_mounts_and_its_pelvis_imu_are_isaac_sim_sites(
    g1: G1BodySim,
) -> None:
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr

    usd = g1.config.usd
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a robot", usd.asset_root))
    for frame in [*G1BodySim.mounts, G1_IMU]:
        assert is_site(stage.GetPrimAtPath(f"/{frame}")), frame
    assert set(G1BodySim.mounts) <= usd.frame_names()
    assert g1.frame.lidar == f"{MOUNT}/lidar"
    assert g1.frame.camera == f"{MOUNT}/camera"


@pytest.mark.self_hosted  # MuJoCo compiles the G1 from its Git LFS meshes
def test_odom_is_from_the_mount_so_the_pelvis_stands_above_it(g1: G1BodySim) -> None:
    scene = UsdModel.from_file(SCENE).with_frames(
        {"g1": ("scene", MOUNT_XYZ, (0.0, 0.0, MOUNT_YAW))}
    )
    world = compose_usds([(scene.usd, None), (fetched_usd(_providing_hardware_files(g1)), MOUNT)])
    backend = MujocoBackend(world, [g1.simulated_robot()])
    g1._world.apply(backend.frame())
    base = g1._world.entity(g1.namespace)
    assert base is not None and base.pose is not None
    assert base.velocity is not None
    odom = g1.odometry(base.pose, base.velocity, 0.0)

    odom_pelvis = Transform.from_pose(g1.body_frame, odom.to_pose_stamped()).to_matrix()
    mount = np.eye(4)
    mount[:3, 3] = MOUNT_XYZ
    c, s = math.cos(MOUNT_YAW), math.sin(MOUNT_YAW)
    mount[:2, :2] = [[c, -s], [s, c]]
    # MuJoCo names a robot's body by its namespace and the prim path's last segment.
    pelvis = backend.data.xpos[backend.model.body(f"{MOUNT}/{G1_BODY.rpartition('/')[2]}").id]
    # tf: the mount holds odom by identity, so mount @ odom is where the pelvis is.
    assert (mount @ odom_pelvis)[:3, 3] == pytest.approx(pelvis, abs=1e-4)
    assert odom.z == pytest.approx(0.793, abs=1e-4)
    assert odom.frame_id == g1.attach_frame


def test_a_position_command_drives_its_joints_with_groot_s_gains_once_the_world_shows_it(
    g1: G1BodySim,
) -> None:
    knee = G1_JOINT_NAMES.index("g1/left_knee")
    command = JointState(name=[G1_JOINT_NAMES[knee]], position=[0.3])
    sent: list[Any] = []
    g1.world_actions.publish = sent.append  # type: ignore[method-assign,assignment]
    asyncio.run(g1.handle_position_command(command))
    assert sent == []
    writer = WorldWriter()
    writer.start_ns = 0
    writer.add(MOUNT, model="g1.usda")
    writer.add(f"{MOUNT}/left_knee_joint")
    g1._world.apply(writer.end_frame(1.0))
    asyncio.run(g1.handle_position_command(command))

    assert [read_actions(action, writer) for action in sent] == [
        [
            JointCommand(
                MOUNT,
                ("left_knee_joint",),
                position=(0.3,),
                velocity=(0.0,),
                effort=(0.0,),
                kp=(G1_GROOT_KP[knee],),
                kd=(G1_GROOT_KD[knee],),
            )
        ]
    ]


def test_its_mount_holds_its_odom_and_nothing_static_holds_its_body(g1: G1BodySim) -> None:
    static = {(t.frame_id, t.child_frame_id) for t in g1.static_transforms()}

    assert (MOUNT, g1.attach_frame) in static
    assert not any(child == g1.body_frame for _, child in static), sorted(static)
    # Point-LIO puts its namespace in front of its frame_id itself.
    assert g1.attach_frame == f"{MOUNT}/{G1_ODOM_TF['tf_frames_local']['odom']}"


def test_the_odom_tf_beside_it_holds_its_body_in_its_odom(g1: G1BodySim) -> None:
    odom_tf = OdomTf(instance_name=f"{MOUNT}/OdomTf", **G1_ODOM_TF)
    try:
        assert odom_tf.tf_edge_ids == {(g1.attach_frame, g1.body_frame)}
    finally:
        odom_tf.stop()


def test_the_pelvis_imu_is_where_the_usd_puts_its_site_on_the_pelvis(g1: G1BodySim) -> None:
    imu_in_body = g1._imu_in_body

    # The site sits below the pelvis origin, unturned, as the vendored MJCF puts it.
    assert np.allclose(imu_in_body[:3, :3], np.eye(3))
    assert imu_in_body[2, 3] < 0
    assert np.linalg.norm(imu_in_body[:3, 3]) < 0.2
