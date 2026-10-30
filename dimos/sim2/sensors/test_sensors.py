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

from contextlib import ExitStack
from dataclasses import replace
from functools import partial
import threading
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import ZenohTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.blueprint import simulation
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Mount


def _description(world, model_path):
    mujoco.mj_saveModel(world.backend.model, str(model_path), None)
    return {**world.describe(), "model": str(model_path)}


def _with_sensors(config, sensors):
    factory = config["environment"]
    return {
        **config,
        "sensors": tuple(sensors),
        "environment": partial(factory.func, **{**factory.keywords, "sensors": tuple(sensors)}),
    }


@pytest.fixture
def reader(worlds, tmp_path, mocker):
    world = worlds()
    observer = WorldReader(_description(world, tmp_path / "world.mjb"))
    mocker.patch.object(world, "publish_snapshot", side_effect=observer.push)
    observer.push(world.last_snapshot)
    return world, observer


def test_lidar_uses_its_own_model_and_sees_scene_mutations(reader):
    world, observer = reader
    assert observer.update()
    prefix = observer.model_prefix("left")
    groups = world.backend.model.geom_group.copy()
    raycaster = Raycaster(observer.model, observer.model.body(prefix + "base").id)
    rays = np.array([[1.0, 0, 0]])
    origin = np.array([0.0, 0, 0.3])
    assert raycaster.ranges(observer.data, origin, rays, 10) == pytest.approx([1.3])
    box = world.backend.model.jnt_qposadr[world.backend.model.body("box").jntadr[0]]
    world.backend.data.qpos[box] = 2.5
    for _ in range(world.describe()["snapshot_stride"]):
        world.step()
    assert observer.update()
    assert raycaster.ranges(observer.data, origin, rays, 10) == pytest.approx([2.3])
    np.testing.assert_array_equal(world.backend.model.geom_group, groups)
    assert observer.model is not world.backend.model
    assert prefix == "robotleft_"


def test_reader_refreshes_model_parameters_and_generation_on_reset(reader):
    world, observer = reader
    assert observer.update()
    generation = observer.episode
    world.backend.model.geom_friction[:] *= 1.5
    world.reset()
    assert observer.update()
    assert observer.episode == generation + 1
    np.testing.assert_array_equal(observer.model.geom_friction, world.backend.model.geom_friction)
    assert len(observer.frames()) == 1
    assert observer.data.time == 0


def test_reader_reloads_model_and_discards_history_for_a_hard_reset(reader, tmp_path):
    world, observer = reader
    assert observer.update()
    previous = observer.model
    new_model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="new"><freejoint/>'
        '<geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>'
    )
    data = mujoco.MjData(new_model)
    state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(new_model, state_kind))
    mujoco.mj_getState(new_model, data, state, state_kind)
    path = tmp_path / "new.mjb"
    mujoco.mj_saveModel(new_model, str(path), None)
    observer.push(
        StateFrame(
            FrameMetadata(observer.sequence + 1, observer.episode + 1, 0, 0.0),
            {"state": state, "wall_time": np.array([1000.0])},
            model_path=str(path),
        )
    )
    assert observer.update()
    assert observer.model is not previous
    assert observer.model.nq == 7
    assert observer.model.body("new").id == 1
    assert len(observer.frames()) == 1
    assert observer.timestamp == 1000.0


def test_composition_has_one_world_and_thin_camera_bridges(world_config):
    sensors = (
        *world_config["sensors"],
        ("left", Camera("camera", "view")),
        ("left", Camera("inspection", "view")),
    )
    blueprint = simulation(**_with_sensors(world_config, sensors))
    atoms = blueprint.blueprints
    types = [atom.module for atom in atoms]
    assert types.count(SimulationModule) == 1
    assert types.count(SimRGBDCameraModule) == 2
    assert types.count(LidarModule) == 1
    cameras = [atom for atom in atoms if atom.module is SimRGBDCameraModule]
    assert all("observations" in {port.name for port in atom.streams} for atom in cameras)
    assert all("snapshots" not in {port.name for port in atom.streams} for atom in cameras)


def test_world_and_robot_cameras_have_distinct_output_topics(world_config):
    sensors = (
        *world_config["sensors"],
        ("left", Camera("camera", "view")),
        (None, Camera("camera", Mount("world", xyz=(0, 0, 3)))),
    )
    blueprint = simulation(**_with_sensors(world_config, sensors))
    assert blueprint.remapping_map["world_camera", "color_image"] == "world/camera/color_image"
    assert blueprint.remapping_map["left_camera", "color_image"] == "left/camera/color_image"


def test_unselected_imu_is_an_observation_bridge(world_config):
    original = world_config["robots"]["left"]
    robot = replace(original, model=replace(original.model, control_imu=None))
    blueprint = simulation(**{**world_config, "robots": {"left": robot}})
    assert sum(atom.module is SimImuModule for atom in blueprint.blueprints) == 1
    assert blueprint.remapping_map["left_imu", "imu"] == "left/imu/imu"


def test_lidar_retains_modeled_delivery_and_acquisition_timestamps(reader, world_config):
    world, observer = reader
    descriptor = next(sensor for _, sensor in world_config["sensors"] if isinstance(sensor, Lidar))
    device = LidarDevice(observer, replace(descriptor, rate_hz=10, latency=0.025), "left", "base")
    try:
        assert device.poll() == []
        assert device.status()["pending_deliveries"] > 0
        for _ in range(6):
            world.step()
        measurements = device.poll()
        assert measurements
        assert {measurement.acquisition_end for measurement in measurements} == {0.0}
        assert {measurement.available_at for measurement in measurements} == {0.025}
        device.published(measurements[0], at=measurements[0].available_timestamp + 0.02)
        assert device.status()["lateness_seconds"] == pytest.approx(0.02, abs=1e-6)
        assert device.poll() == []
    finally:
        device.close()


def test_reset_discards_pending_lidar_deliveries(reader, world_config):
    world, observer = reader
    descriptor = next(sensor for _, sensor in world_config["sensors"] if isinstance(sensor, Lidar))
    device = LidarDevice(observer, replace(descriptor, rate_hz=10, latency=0.05), "left", "base")
    try:
        assert device.poll() == []
        episode = world.episode
        world.reset()
        assert device.poll() == []
        for _ in range(12):
            world.step()
        samples = device.poll()
        assert samples
        assert all(sample.episode > episode for sample in samples)
    finally:
        device.close()


@pytest.mark.parametrize("external_camera", [False, True])
@pytest.mark.self_hosted
def test_blueprint_deploys_world_observations_and_rolling_workers(world_config, external_camera):
    image_ready, cloud_ready = threading.Event(), threading.Event()
    images, clouds, depth_clouds = {}, [], []
    depth_cloud_ready = threading.Event()
    image_frames = {"left/camera_optical"}
    if external_camera:
        image_frames.add("world/overhead_optical")

    def on_image(value):
        images[value.frame_id] = value
        if image_frames <= images.keys():
            image_ready.set()

    def on_cloud(value):
        if not clouds:
            clouds.append(value)
            cloud_ready.set()

    def on_depth_cloud(value):
        depth_clouds.append(value)
        depth_cloud_ready.set()

    sensors = (*world_config["sensors"], ("left", Camera("camera", "view", width=160, height=120)))
    if external_camera:
        sensors += (
            (
                None,
                Camera(
                    "overhead",
                    Mount("world", xyz=(1.5, 0, 3)),
                    width=160,
                    height=120,
                    pointcloud=True,
                ),
            ),
        )
    with ExitStack() as stack:
        prefix = "sim2-test/" + uuid4().hex
        image = ZenohTransport(prefix + "/image", Image)
        cloud = ZenohTransport(prefix + "/cloud", PointCloud2)
        stack.callback(image.stop)
        stack.callback(cloud.stop)
        stack.callback(image.subscribe(on_image))
        stack.callback(cloud.subscribe(on_cloud))
        image_port = "world/overhead/color_image" if external_camera else "left/camera/color_image"
        transports = {(image_port, Image): image, ("left/lidar/pointcloud", PointCloud2): cloud}
        if external_camera:
            depth_cloud = ZenohTransport(prefix + "/depth-cloud", PointCloud2)
            stack.callback(depth_cloud.stop)
            stack.callback(depth_cloud.subscribe(on_depth_cloud))
            transports["world/overhead/pointcloud", PointCloud2] = depth_cloud
            robot_image = ZenohTransport(prefix + "/robot-image", Image)
            stack.callback(robot_image.stop)
            stack.callback(robot_image.subscribe(on_image))
            transports["left/camera/color_image", Image] = robot_image
        blueprint = simulation(**_with_sensors(world_config, sensors))
        coordinator = ModuleCoordinator.build(
            blueprint.transports(transports).global_config(
                n_workers=1,
                viewer="none",
                transport="zenoh",
            )
        )
        stack.callback(coordinator.stop)
        sim = coordinator.get_instance(SimulationModule)
        assert image_ready.wait(30), "camera observation bridge did not publish"
        assert cloud_ready.wait(30), "lidar worker did not publish"
        assert images.keys() == image_frames
        assert all(value.data.std() > 15 for value in images.values())
        assert clouds[0].frame_id == "world"
        if external_camera:
            assert depth_cloud_ready.wait(15), "RGB-D bridge did not publish a point cloud"
            assert depth_clouds[0].frame_id == "world/overhead_optical"
            assert len(depth_clouds[0].pointcloud.points) > 100
        assert sim.status()["running"]
        assert sim.status()["steps"] > 0
        episode = sim.status()["episode"]
        sim.reset()
        assert sim.status()["episode"] == episode + 1


def test_reader_history_is_bounded_and_cleared_at_reset(reader):
    world, observer = reader
    for _ in range(100):
        world.step()
    frames = observer.frames()
    assert len(frames) == world.describe()["snapshot_slots"]
    assert all(frame.metadata.episode_id == world.episode for frame in frames)
    world.reset()
    frames = observer.frames()
    assert len(frames) == 1
    assert frames[0].metadata.sim_time == 0
    assert frames[0].metadata.episode_id == world.episode
