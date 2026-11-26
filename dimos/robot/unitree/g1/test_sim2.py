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

import math
from pathlib import Path
import time
from typing import Any

import numpy as np
import pytest

pytest.importorskip("pxr")
pytest.importorskip("mujoco")

SCENE = Path(__file__).parents[3] / "sim2" / "mujoco" / "assets" / "scene.usda"
MOUNT = "scene/g1"
MOUNT_XYZ = (1.0, -2.0, 0.5)
MOUNT_YAW = 0.7
# An IMU off the body's origin and turned on it, so the lever arm and the mounting both show.
IMU_XYZ = (0.1, 0.0, -0.1)
IMU_YAW = math.pi / 2


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


def _imu_in_body(yaw: float = 0.0) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    imu = np.eye(4)
    imu[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    imu[:3, 3] = IMU_XYZ
    return imu


@pytest.mark.self_hosted  # MuJoCo compiles the G1 from its Git LFS meshes
def test_odom_is_from_the_mount_so_the_pelvis_stands_above_it() -> None:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.unitree.g1.sim2 import G1_BODY, G1_USD, G1Sim
    from dimos.sim2.mujoco.backend import MujocoBackend
    from dimos.sim2.sim import compose_usds, fetched_usd
    from dimos.sim2.spec import SimulatedRobot
    from dimos.sim2.world_observations import WorldMirror

    scene = UsdModel.from_file(SCENE).with_frames(
        {"g1": ("scene", MOUNT_XYZ, (0.0, 0.0, MOUNT_YAW))}
    )
    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    try:
        g1 = fetched_usd(_providing_hardware_files(sim))
        world = compose_usds([(scene.usd, None), (g1, MOUNT)])
        backend = MujocoBackend(world, [SimulatedRobot(MOUNT, G1_USD)])
        sim._world = WorldMirror()
        sim._world.apply(backend.frame())
        odom = sim._odom(time.time())
        assert odom is not None
        odom_pelvis = Transform.from_pose(sim.tf_frames["body"], odom).to_matrix()
        mount = Transform.from_matrix(np.eye(4)).to_matrix()
        mount[:3, 3] = MOUNT_XYZ
        c, s = math.cos(MOUNT_YAW), math.sin(MOUNT_YAW)
        mount[:2, :2] = [[c, -s], [s, c]]
        # MuJoCo names a robot's body by its namespace and the prim path's last segment.
        pelvis = backend.data.xpos[backend.model.body(f"{MOUNT}/{G1_BODY.rpartition('/')[2]}").id]
        # tf: the mount holds odom by identity, so mount @ odom is where the pelvis is.
        assert (mount @ odom_pelvis)[:3, 3] == pytest.approx(pelvis, abs=1e-4)
        assert odom.position.z == pytest.approx(0.793, abs=1e-4)
    finally:
        sim.stop()


def test_odom_is_in_the_frame_a_real_g1_reports_it_in() -> None:
    from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig
    from dimos.robot.unitree.g1.sim2 import G1Sim

    sim = G1Sim(attached_to="g1", instance_name="g1/G1Sim")
    try:
        # Point-LIO puts its namespace in front of its frame_id itself.
        assert sim.attach_frame == f"g1/{PointLioConfig.model_fields['frame_id'].default}"
    finally:
        sim.stop()


def test_tf_goes_out_at_the_rate_a_real_g1_publishes_it() -> None:
    import asyncio
    from collections.abc import AsyncIterator

    from dimos.robot.unitree.g1.g1_tf_publisher import G1TfPublisherConfig
    from dimos.robot.unitree.g1.sim2 import G1Sim

    rates: list[float] = []

    class _World:
        async def ticks(self, rate_hz: float) -> AsyncIterator[float]:
            rates.append(rate_hz)
            for _ in ():
                yield 0.0

    sim = G1Sim(attached_to="g1", instance_name="g1/G1Sim")
    try:
        sim._world = _World()  # type: ignore[assignment]
        asyncio.run(sim._publish_tf())
    finally:
        sim.stop()
    assert rates == [G1TfPublisherConfig.model_fields["publish_hz"].default]


def test_the_simulated_body_is_the_root_link_of_the_g1_usd() -> None:
    from dimos.robot.unitree.g1.sim2 import G1_BODY, G1_ODOM_TF, G1_USD, G1SimConfig

    assert G1_USD.root_link == G1_BODY == G1_ODOM_TF["tf_frames_local"]["body"]
    assert G1SimConfig().tf_frames_local["body"] == G1_BODY


def test_a_g1_publishes_its_odometry_on_odom_and_only_the_joints_that_move_on_tf() -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.robot.unitree.g1.sim2 import G1Sim
    from dimos.sim2.world_observations import WorldMirror, WorldWriter

    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    try:
        writer, sim._world = WorldWriter(), WorldMirror()
        writer.start_ns = 0
        writer.add(MOUNT, model="g1.usda")
        writer.body_state(MOUNT, [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0], [0.0] * 6, [0.0] * 6)
        sim._world.apply(writer.end_frame(1.0))
        sim._tf = sim._create_tfbuffer(None)
        published: list[set[tuple[str, str]]] = []
        odometry: list[PoseStamped] = []
        sim._tf.publish = lambda *ts: published.append(  # type: ignore[method-assign]
            {(t.frame_id, t.child_frame_id) for t in ts}
        )
        sim.odom.publish = odometry.append  # type: ignore[method-assign,assignment]
        sim._publish_tf_tick()
        moving = {(t.frame_id, t.child_frame_id) for t in sim.moving_transforms()}
    finally:
        sim.stop()

    assert moving
    # Odometry is an estimate, so it goes out on odom for an OdomTf, never on the hardware's tf.
    assert published == [moving]
    assert [reading.frame_id for reading in odometry] == [sim.attach_frame]


def test_a_position_command_drives_its_joints_with_groot_s_gains_once_the_world_shows_the_g1() -> (
    None
):
    import asyncio

    from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
    from dimos.robot.unitree.g1.sim2 import G1Sim, usd_joint
    from dimos.sim2.actions import JointCommand, read_actions
    from dimos.sim2.world_observations import WorldWriter

    knee = G1_JOINT_NAMES.index("g1/left_knee")
    command = JointState(name=[G1_JOINT_NAMES[knee]], position=[0.3])
    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    sent: list[Any] = []
    try:
        sim.world_actions.publish = sent.append  # type: ignore[method-assign,assignment]
        asyncio.run(sim.handle_position_command(command))
        assert sent == []
        writer = WorldWriter()
        writer.start_ns = 0
        writer.add(MOUNT, model="g1.usda")
        writer.add(f"{MOUNT}/{usd_joint(G1_JOINT_NAMES[knee])}")
        sim._world.apply(writer.end_frame(1.0))
        asyncio.run(sim.handle_position_command(command))
    finally:
        sim.stop()

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


def test_a_mounted_g1_s_mount_holds_its_odom_and_nothing_static_holds_its_body() -> None:
    from dimos.robot.unitree.g1.sim2 import G1Sim

    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    try:
        body = sim.tf_frames[sim.config.usd.root_link]
        static = {(t.frame_id, t.child_frame_id) for t in sim.static_transforms()}
        odom = sim.attach_frame
    finally:
        sim.stop()

    assert (MOUNT, odom) in static
    assert not any(f"{body}/".startswith(f"{child}/") for _, child in static), sorted(static)


def test_the_odom_tf_beside_a_g1_holds_its_body_in_its_odom() -> None:
    from dimos.robot.unitree.g1.sim2 import G1_ODOM_TF, G1Sim
    from dimos.sim2.odom_tf import OdomTf

    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    odom_tf = OdomTf(instance_name=f"{MOUNT}/OdomTf", **G1_ODOM_TF)
    try:
        assert odom_tf.tf_edge_ids == {(sim.attach_frame, sim.tf_frames["body"])}
    finally:
        sim.stop()
        odom_tf.stop()


def test_the_pelvis_imu_is_where_the_usd_puts_its_site_on_the_pelvis() -> None:
    from dimos.robot.unitree.g1.sim2 import G1Sim

    sim = G1Sim(attached_to=MOUNT, instance_name=f"{MOUNT}/G1Sim")
    try:
        imu_in_body = sim._imu_in_body
    finally:
        sim.stop()

    # The site sits below the pelvis origin, unturned, as the vendored MJCF puts it.
    assert np.allclose(imu_in_body[:3, :3], np.eye(3))
    assert imu_in_body[2, 3] < 0
    assert np.linalg.norm(imu_in_body[:3, 3]) < 0.2


def test_a_g1_at_rest_reads_gravity_up_and_no_spin() -> None:
    from dimos.sim2.mujoco.sensor_world import imu_reading

    gyro, accelerometer, orientation = imu_reading(
        np.array([1.0, 2.0, 0.8, 0.0, 0.0, 0.0, 1.0]), np.zeros(6), np.zeros(6), _imu_in_body()
    )

    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    assert orientation == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_a_spinning_g1_s_gyro_reads_the_spin_in_the_imu_s_axes() -> None:
    from dimos.sim2.mujoco.sensor_world import imu_reading

    spin = np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0])
    # The IMU is yawed a quarter turn on the body, so the world's x spin is about its -y.
    gyro, _, _ = imu_reading(
        np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]), spin, np.zeros(6), _imu_in_body(IMU_YAW)
    )

    assert gyro == pytest.approx([0.0, -0.3, 0.0])


def test_a_spinning_g1_s_accelerometer_feels_the_pull_toward_the_axis() -> None:
    from dimos.sim2.mujoco.sensor_world import imu_reading

    rate = 2.0
    spin = np.array([0.0, 0.0, 0.0, 0.0, 0.0, rate])
    _, accelerometer, _ = imu_reading(
        np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]), spin, np.zeros(6), _imu_in_body()
    )

    # Centripetal: rate^2 * radius toward the axis, from an IMU IMU_XYZ[0] out along x.
    assert accelerometer == pytest.approx([-(rate**2) * IMU_XYZ[0], 0.0, 9.81])


def test_a_tilted_g1_s_imu_is_turned_as_its_root_is_and_feels_gravity_tilted() -> None:
    from scipy.spatial.transform import Rotation

    from dimos.sim2.mujoco.sensor_world import imu_reading

    tilt = Rotation.from_euler("xyz", (0.3, -0.2, 1.1))
    gyro, accelerometer, orientation = imu_reading(
        np.array([0.0, 0.0, 0.8, *tilt.as_quat()]), np.zeros(6), np.zeros(6), _imu_in_body(IMU_YAW)
    )

    expected = tilt * Rotation.from_euler("z", IMU_YAW)
    assert Rotation.from_quat(orientation).approx_equal(expected, atol=1e-9)
    assert accelerometer == pytest.approx(expected.inv().apply([0.0, 0.0, 9.81]))
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
