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
import multiprocessing
import threading
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.blueprint import simulation
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import StreamDescriptor
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.device import CameraDevice
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.device import ImuDevice
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.world.scene_types import SceneUpdate


@pytest.fixture
def reader(worlds, tmp_path):
    world = worlds()
    model_path = tmp_path / "world.mjb"
    mujoco.mj_saveModel(world.model, str(model_path), None)
    observer = WorldReader(
        {"model": str(model_path), "snapshot": world.snapshot_descriptor.to_dict()}
    )
    try:
        yield world, observer
    finally:
        observer.close()


def _render_snapshot(description, camera_name, pipe):
    observer = WorldReader(description)
    try:
        assert observer.update()
        sensor = Camera("test", camera_name, width=160, height=120)
        renderer = CameraDevice(observer, sensor)
        try:
            messages = {m.port: m.value for m in renderer.poll()}
            pipe.send((messages["color_image"].data, messages["depth_image"].data))
        finally:
            renderer.close()
    finally:
        observer.close()
        pipe.close()


@pytest.fixture
def capture(tmp_path):
    # Keep native graphics contexts out of the test coordinator, as in deployment.
    def render(world, camera_name):
        model_path = tmp_path / "capture.mjb"
        mujoco.mj_saveModel(world.model, str(model_path), None)
        description = {"model": str(model_path), "snapshot": world.snapshot_descriptor.to_dict()}
        context = multiprocessing.get_context("spawn")
        reader, writer = context.Pipe(duplex=False)
        process = context.Process(target=_render_snapshot, args=(description, camera_name, writer))
        try:
            process.start()
            writer.close()
            assert reader.poll(15), "renderer did not produce a frame"
            result = reader.recv()
            process.join(5)
            assert process.exitcode == 0
            return result
        finally:
            if process.is_alive():
                process.terminate()
                process.join(5)
            reader.close()
            writer.close()

    return render


@pytest.mark.self_hosted
def test_world_snapshot_renders_nonblank_rgb_and_metric_depth(worlds, capture):
    world = worlds()
    rgb, depth = capture(world, "left/view")
    assert rgb.shape == (120, 160, 3)
    assert rgb.std() > 15
    assert depth is not None
    assert depth.shape == (120, 160)
    assert np.count_nonzero((depth > 0.1) & (depth < 10)) > 1000
    assert world.tick == 0


def test_lidar_uses_its_own_model_and_sees_scene_mutations(reader):
    world, observer = reader
    assert observer.update()
    groups = world.model.geom_group.copy()
    raycaster = Raycaster(observer.model, observer.model.body("left/base").id)
    rays = np.array([[1.0, 0, 0]])
    origin = np.array([0.0, 0, 0.3])
    before = raycaster.ranges(observer.data, origin, rays, 10)
    assert before == pytest.approx([1.3])
    world.set_scene_state(SceneUpdate(poses={"box": Pose(2.5, 0, 0.3)}))
    assert observer.update()
    after = raycaster.ranges(observer.data, origin, rays, 10)
    assert after == pytest.approx([2.3])
    np.testing.assert_array_equal(world.model.geom_group, groups)
    assert observer.model is not world.model


def test_reader_observes_reset_generation_without_recompiling(reader):
    world, observer = reader
    assert observer.update()
    model = observer.model
    generation = observer.episode
    world.reset()
    assert observer.update()
    assert observer.episode == generation + 1
    assert observer.model is model


def test_composition_contains_one_world_and_no_public_sim_connections(world_config):
    instance = world_config.robots["left"]
    robot = instance.config
    extra_camera = replace(robot.sensors[1], name="inspection")
    robot = robot.with_sensor(extra_camera)
    blueprint = simulation(
        world=replace(
            world_config,
            robots={
                "left": replace(instance, config=robot),
                "right": replace(instance, xyz=(0, 2, 0.3)),
            },
        ),
    )
    atoms = blueprint.blueprints
    types = [atom.module for atom in atoms]
    assert types.count(SimulationModule) == 1
    assert types.count(SimRGBDCameraModule) == 3
    assert types.count(LidarModule) == 2
    assert len(types) == 6
    assert len({atom.name for atom in atoms}) == 6


@pytest.mark.self_hosted
def test_scene_devices_attach_without_a_fake_robot(world_config, worlds, capture):
    camera = Camera("overhead", Mount("world", xyz=(1.5, 0, 3)), width=160, height=120)
    lidar = Lidar("monitor", Mount("world", xyz=(0, 0, 0.3)), Fibonacci)
    world = worlds(config=replace(world_config, robots={}, sensors=(camera, lidar)))
    assert world.robots == {}
    camera_id = world.model.camera("sensor/overhead").id
    assert world.data.cam_xpos[camera_id] == pytest.approx([1.5, 0, 3])
    site_id = world.model.site("sensor/monitor").id
    assert world.data.site_xpos[site_id] == pytest.approx([0, 0, 0.3])
    rgb, depth = capture(world, "sensor/overhead")
    assert rgb.std() > 15
    assert np.count_nonzero((depth > 0.1) & (depth < 5)) > 1000
    caster = Raycaster(world.model)
    assert caster.ranges(
        world.data, world.data.site_xpos[site_id], np.array([[1.0, 0, 0]]), 10
    ) == pytest.approx([1.3])


def test_world_device_can_follow_a_movable_scene_body(world_config, worlds):
    camera = Camera("inspection", Mount("box", xyz=(0, 0, 1)))
    world = worlds(config=replace(world_config, sensors=(camera,)))
    camera_id = world.model.camera("sensor/inspection").id
    assert world.data.cam_xpos[camera_id] == pytest.approx([1.5, 0, 1.3])
    world.set_scene_state(SceneUpdate(poses={"box": Pose(2.5, 0, 0.3)}))
    assert world.data.cam_xpos[camera_id] == pytest.approx([2.5, 0, 1.3])


def test_world_and_robot_cameras_have_distinct_output_topics(world_config):
    camera = Camera("camera", Mount("world", xyz=(0, 0, 3)))
    blueprint = simulation(world=replace(world_config, sensors=(camera,)))
    assert blueprint.remapping_map["world_camera", "color_image"] == "world/camera/color_image"
    assert blueprint.remapping_map["left_camera", "color_image"] == "left/camera/color_image"
    assert sum(a.module is SimulationModule for a in blueprint.blueprints) == 1


def test_adding_devices_preserves_existing_routes_and_explicit_world_settings(world_config):
    before = simulation(world=world_config)
    extra = Camera("overhead", Mount("world", xyz=(0, 0, 3)))
    world = replace(world_config, sensors=(extra,), snapshot_hz=91)
    after = simulation(world=world)
    assert after.blueprints[0].kwargs["world"] is world
    assert after.remapping_map.items() >= before.remapping_map.items()
    assert before.remapping_map["left_camera", "color_image"] == "left/camera/color_image"


def test_an_unselected_imu_is_an_independent_device(world_config):
    instance = replace(world_config.robots["left"], control_imu=None)
    blueprint = simulation(world=replace(world_config, robots={"left": instance}))
    assert sum(a.module is SimImuModule for a in blueprint.blueprints) == 1
    assert blueprint.remapping_map["left_imu", "imu"] == "left/imu/imu"


@pytest.mark.parametrize("device_type,sensor_index", [(ImuDevice, 0), (LidarDevice, 2)])
def test_devices_acquire_without_a_module_and_wait_for_modeled_delivery(
    reader, device_type, sensor_index
):
    world, observer = reader
    sensor = replace(
        world.config.robots["left"].config.sensors[sensor_index], rate_hz=10, latency=0.025
    )
    device = device_type(observer, sensor, "left", "base")
    try:
        assert device.poll() == []
        assert device.status()["pending_deliveries"] > 0
        for _ in range(6):
            world.step()
        measurements = device.poll()
        assert measurements
        assert {m.acquisition_end for m in measurements} == {0.0}
        assert {m.available_at for m in measurements} == {0.025}
        assert all(m.episode == world.generation for m in measurements)
        device.published(measurements[0], at=measurements[0].available_timestamp + 0.02)
        assert device.status()["lateness_seconds"] == pytest.approx(0.02, abs=1e-6)
        assert device.poll() == []
    finally:
        device.close()


def test_slow_polling_reports_drops_without_changing_acquisition_clock(reader):
    world, observer = reader
    sensor = replace(world.config.robots["left"].config.sensors[0], rate_hz=10)
    device = ImuDevice(observer, sensor, "left")
    try:
        assert device.poll()[0].acquisition_end == 0
        for _ in range(68):
            world.step()
        sample = device.poll()[0]
        assert sample.acquisition_end == pytest.approx(0.3)
        assert device.status()["dropped_acquisitions"] == 2
        assert sample.value.orientation_covariance[0] == -1
    finally:
        device.close()


def test_pause_and_reset_discard_old_deliveries(reader):
    world, observer = reader
    sensor = replace(world.config.robots["left"].config.sensors[0], rate_hz=10, latency=0.05)
    device = ImuDevice(observer, sensor, "left")
    try:
        assert device.poll() == []
        old_episode = world.generation
        world.set_paused(True)
        world.step()
        assert device.poll() == []
        assert world.data.time == 0
        world.set_paused(False)
        world.reset()
        assert device.poll() == []
        for _ in range(12):
            world.step()
        samples = device.poll()
        assert samples
        assert all(m.episode > old_episode for m in samples)
    finally:
        device.close()


def test_capture_timestamps_do_not_follow_host_polling_jitter(reader, mocker):
    world, observer = reader
    sensor = replace(world.config.robots["left"].config.sensors[0], rate_hz=10)
    device = ImuDevice(observer, sensor, "left")
    try:
        first = device.poll()[0]
        mocker.patch("dimos.sim2.world.runtime.time.time", return_value=first.value.ts + 10)
        for _ in range(22):
            world.step()
        second = device.poll()[0]
        assert second.value.ts - first.value.ts == pytest.approx(0.1, abs=1e-6)
        device.published(second, at=first.value.ts + 10)
        assert device.status()["lateness_seconds"] > 9
    finally:
        device.close()


@pytest.mark.parametrize("external_camera", [False, True])
@pytest.mark.self_hosted
def test_blueprint_deploys_physics_camera_and_lidar_in_workers(world_config, external_camera):
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

    with ExitStack() as stack:
        prefix = "sim2-test/" + uuid4().hex
        image = ZenohTransport(prefix + "/image", Image)
        cloud = ZenohTransport(prefix + "/cloud", PointCloud2)
        stack.callback(image.stop)
        stack.callback(cloud.stop)
        stack.callback(image.subscribe(on_image))
        stack.callback(cloud.subscribe(on_cloud))
        sensors = (
            (
                Camera(
                    "overhead",
                    Mount("world", xyz=(1.5, 0, 3)),
                    width=160,
                    height=120,
                    pointcloud=True,
                ),
            )
            if external_camera
            else ()
        )
        image_port = "world/overhead/color_image" if external_camera else "left/camera/color_image"
        cloud_port = "left/lidar/pointcloud"
        transports = {(image_port, Image): image, (cloud_port, PointCloud2): cloud}
        if external_camera:
            depth_cloud = ZenohTransport(prefix + "/depth-cloud", PointCloud2)
            stack.callback(depth_cloud.stop)
            stack.callback(depth_cloud.subscribe(on_depth_cloud))
            transports["world/overhead/pointcloud", PointCloud2] = depth_cloud
            robot_image = ZenohTransport(prefix + "/robot-image", Image)
            stack.callback(robot_image.stop)
            stack.callback(robot_image.subscribe(on_image))
            transports["left/camera/color_image", Image] = robot_image
        blueprint = simulation(world=replace(world_config, sensors=sensors))
        coordinator = ModuleCoordinator.build(
            blueprint.transports(transports).global_config(
                n_workers=1, viewer="none", transport="zenoh"
            )
        )
        stack.callback(coordinator.stop)
        sim = coordinator.get_instance(SimulationModule)
        assert image_ready.wait(15), "camera worker did not publish"
        assert cloud_ready.wait(15), "lidar worker did not publish"
        assert images.keys() == image_frames
        assert all(value.data.std() > 15 for value in images.values())
        assert clouds[0].frame_id == "world"
        if external_camera:
            assert depth_cloud_ready.wait(15), "RGB-D worker did not publish a point cloud"
            assert depth_clouds[0].frame_id == "world/overhead_optical"
            assert len(depth_clouds[0].pointcloud.points) > 100
        assert sim.status()["running"]
        assert sim.status()["steps"] > 0
        descriptor = StreamDescriptor.from_dict(sim.describe()["robots"]["left"])
        with SimRobotClient(descriptor) as client:
            before = client.read()
            sim.reset()
            after = client.read()
            assert after.metadata.episode_id == before.metadata.episode_id + 1

    with pytest.raises(TimeoutError):
        SimRobotClient(descriptor).connect(timeout=0.1)
