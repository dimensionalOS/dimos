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

"""Real worker/transport checks for composed cameras sharing a rendering owner."""

from collections import defaultdict
from contextlib import ExitStack
from queue import Queue
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.hardware_module import Mount, Mounts
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.sim2.description import compose_scene, scene_revision
from dimos.sim2.hardware import SimHardwareModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import (
    SimCameraModule,
    SimColorCameraModule,
    SimCompressedCameraModule,
    SimDepthCameraModule,
)
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted


def mount(name):
    return f"world/world/{name}_mount/cartesian_pusher/base_link/carriage/tool/camera"


class CameraScene(SimHardwareModule):
    mounts = Mounts(mount("left"), mount("right").replace("/camera", "/lidar"))


# Mounts are named by their last segment. Use distinct existing mounts.
def camera_mount(name):
    return mount(name) if name == "left" else mount(name).replace("/camera", "/lidar")


def device_name(name):
    return camera_mount(name) + "/" + name


def camera_frame(name):
    return camera_mount(name) + "/camera/view"


def composed(deployment):
    return compose_scene(
        [
            deployment.get_instance(d.instance_name)
            for d in deployment.list_modules()
            if "get_usd" in d.rpc_names
        ]
    )


@pytest.fixture
def camera_blueprint(world, request):
    def camera(name, hz, device=SimCameraModule):
        color, depth = device is not SimDepthCameraModule, device is not SimColorCameraModule
        ports = [p for p, has in (("color_image", color), ("depth_image", depth)) if has]
        return device.attach(
            Mount(camera_mount(name)),
            instance_name=name,
            sensor=CameraConfig("view", hz=hz, color=color, depth=depth),
        ).remappings([(device, port, f"{name}/{port}") for port in (*ports, "camera_info")])

    left = camera("left", 10)
    right = camera("right", 5, getattr(request, "param", SimCameraModule))
    return autoconnect(
        CameraScene.blueprint(instance_name="scene", usd=world),
        left,
        right,
        CameraRenderModule.blueprint(
            renderer=MujocoRenderer, cameras=(device_name("left"), device_name("right"))
        ),
    ).global_config(
        viewer="none",
        transport="zenoh",
        n_workers=5,
        zenoh_domain="sim2-cameras-" + uuid.uuid4().hex,
    )


@pytest.fixture
def cameras(camera_blueprint, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = camera_blueprint.solidify()
    parsed = BlueprintConfigParser(blueprint).parse()
    deployed = ModuleCoordinator.build(blueprint, parsed_config=parsed)
    try:
        yield deployed
    finally:
        deployed.stop()


def test_two_mounted_cameras_share_one_scene_and_keep_topics_rates_and_timestamps(
    cameras, tmp_path
):
    render = cameras.get_instance(CameraRenderModule)
    wait_until(lambda: render.statistics().get("scene_loads") == 1, timeout=20)
    scene = composed(cameras)
    messages = defaultdict(Queue)
    with ExitStack() as cleanup:
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        observations = cameras.transports[("observations", WorldState)]
        cleanup.callback(observations.stop)
        for name in ("left", "right"):
            for port, kind in (
                ("color_image", Image),
                ("depth_image", Image),
                ("camera_info", CameraInfo),
            ):
                transport = cameras.transports[(f"{name}/{port}", kind)]
                cleanup.callback(transport.stop)
                cleanup.callback(transport.subscribe(messages[name, port].put))
        samples = defaultdict(list)
        for step in range(5):
            bodies = backend.read_bodies()
            observations.publish(
                WorldState(
                    header=SampleHeader(
                        clock_id="camera-test", time_ns=step * 100_000_000, frame_id="world"
                    ),
                    model_revision=scene_revision(scene),
                    bodies_length=len(bodies),
                    bodies=bodies,
                )
            )
            names = ("left", "right") if step % 2 == 0 else ("left",)
            for name in names:
                for port in ("color_image", "depth_image", "camera_info"):
                    msg = messages[name, port].get(timeout=10)
                    assert msg.frame_id == camera_frame(name)
                    assert msg.ts == pytest.approx(step / 10)
                    samples[name, port].append(msg)
    assert render.statistics()["scene_loads"] == 1
    assert render.statistics()["cameras"] == 2
    assert render.statistics()["error"] == ""
    assert [m.ts for m in samples["left", "color_image"]] == [0, 0.1, 0.2, 0.3, 0.4]
    assert [m.ts for m in samples["right", "color_image"]] == [0, 0.2, 0.4]
    assert cameras.get_instance(device_name("left")).statistics()["acquisitions"] == 5
    assert cameras.get_instance(device_name("right")).statistics()["acquisitions"] == 3
    assert samples["left", "color_image"][0].shape == (240, 320, 3)
    assert samples["right", "depth_image"][0].shape == (240, 320)
    np.testing.assert_allclose(
        samples["left", "camera_info"][0].K, [188.36, 0, 160, 0, 188.36, 120, 0, 0, 1]
    )


@pytest.mark.parametrize("camera_blueprint", [SimCompressedCameraModule], indirect=True)
def test_compressed_camera_sends_jpeg_color_beside_a_raw_camera(cameras, tmp_path):
    render = cameras.get_instance(CameraRenderModule)
    wait_until(lambda: render.statistics().get("scene_loads") == 1, timeout=20)
    scene = composed(cameras)
    messages = defaultdict(Queue)
    with ExitStack() as cleanup:
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        observations = cameras.transports[("observations", WorldState)]
        cleanup.callback(observations.stop)
        for stream, kind in (
            ("left/color_image", Image),
            ("right/color_image", CompressedImage),
            ("right/depth_image", Image),
        ):
            transport = cameras.transports[(stream, kind)]
            cleanup.callback(transport.stop)
            cleanup.callback(transport.subscribe(messages[stream].put))
        bodies = backend.read_bodies()
        observations.publish(
            WorldState(
                header=SampleHeader(clock_id="camera-test", time_ns=300_000_000, frame_id="world"),
                model_revision=scene_revision(scene),
                bodies_length=len(bodies),
                bodies=bodies,
            )
        )
        raw = messages["left/color_image"].get(timeout=10)
        jpeg = messages["right/color_image"].get(timeout=10)
        depth = messages["right/depth_image"].get(timeout=10)
    assert isinstance(raw, Image) and raw.shape == (240, 320, 3)
    assert isinstance(jpeg, CompressedImage) and jpeg.format == "jpeg"
    assert jpeg.frame_id == camera_frame("right") and jpeg.ts == pytest.approx(0.3)
    # Far smaller than the raw pixels, and still a picture of the scene.
    assert len(jpeg.data) < 240 * 320 * 3 / 4
    decoded = jpeg.decode()
    assert decoded.shape == (240, 320, 3) and decoded.data.std() > 5
    # JPEG cannot carry metric depth: it stays raw.
    assert depth.shape == (240, 320) and depth.data.dtype == np.float32
    assert cameras.get_instance(device_name("right")).jpeg_quality() == 75
    assert cameras.get_instance(device_name("left")).jpeg_quality() is None


@pytest.mark.parametrize(
    "camera_blueprint", [SimColorCameraModule, SimDepthCameraModule], indirect=True
)
def test_a_camera_with_one_image_has_only_that_stream(cameras, camera_blueprint, tmp_path):
    render = cameras.get_instance(CameraRenderModule)
    wait_until(lambda: render.statistics().get("scene_loads") == 1, timeout=20)
    scene = composed(cameras)
    right = next(
        a for a in camera_blueprint.solidify().active_blueprints if a.name == device_name("right")
    )
    color_only = right.module is SimColorCameraModule
    own = ("right/color_image", CompressedImage) if color_only else ("right/depth_image", Image)
    messages = defaultdict(Queue)
    with ExitStack() as cleanup:
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        observations = cameras.transports[("observations", WorldState)]
        cleanup.callback(observations.stop)
        # The RGB-D camera on the left is served as before, beside it.
        for stream, kind in (own, ("left/color_image", Image), ("left/depth_image", Image)):
            transport = cameras.transports[(stream, kind)]
            cleanup.callback(transport.stop)
            cleanup.callback(transport.subscribe(messages[stream].put))
        bodies = backend.read_bodies()
        observations.publish(
            WorldState(
                header=SampleHeader(clock_id="camera-test", time_ns=300_000_000, frame_id="world"),
                model_revision=scene_revision(scene),
                bodies_length=len(bodies),
                bodies=bodies,
            )
        )
        image = messages[own[0]].get(timeout=10)
        assert messages["left/color_image"].get(timeout=10).shape == (240, 320, 3)
        assert messages["left/depth_image"].get(timeout=10).shape == (240, 320)
    assert image.frame_id == camera_frame("right") and image.ts == pytest.approx(0.3)
    if color_only:
        decoded = image.decode()
        assert decoded.shape == (240, 320, 3) and decoded.data.std() > 5
    else:
        assert image.shape == (240, 320) and image.data.dtype == np.float32
    ports = {
        port.name for port in cameras.get_instance(device_name("right")).get_stream_descriptors()
    }
    assert {"color_image", "depth_image"} & ports == {own[0].split("/")[1]}
    assert "camera_info" in ports
    assert cameras.get_instance(device_name("right")).statistics()["acquisitions"] == 1


def test_a_depth_device_cannot_have_a_color_only_sensor(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = autoconnect(
        SimCameraModule.blueprint(sensor=CameraConfig("camera", depth=False)),
        CameraRenderModule.blueprint(
            scene=world, renderer=MujocoRenderer, cameras=("SimCameraModule",)
        ),
    ).global_config(
        viewer="none", n_workers=1, zenoh_domain="sim2-depth-mismatch-" + uuid.uuid4().hex
    )
    with pytest.raises(ValueError, match="publishes depth_image exactly when"):
        ModuleCoordinator.build(blueprint)


def test_renderer_cannot_take_a_camera_assigned_to_another_owner(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = autoconnect(
        SimCameraModule.blueprint(sensor=CameraConfig("camera"), renderer_name="other"),
        CameraRenderModule.blueprint(
            scene=world, renderer=MujocoRenderer, cameras=("SimCameraModule",)
        ),
    ).global_config(
        viewer="none", n_workers=1, zenoh_domain="sim2-wrong-renderer-" + uuid.uuid4().hex
    )
    with pytest.raises(ValueError, match="no camera renderer named"):
        ModuleCoordinator.build(blueprint)
