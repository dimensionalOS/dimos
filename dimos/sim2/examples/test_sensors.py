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
from queue import Queue
import uuid

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.hardware_module import Mounts
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.hardware import SimHardwareModule
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted


_TOOL = "world/world/left_mount/cartesian_pusher/base_link/carriage/tool"
_JOINT = "world/world/left_mount/cartesian_pusher/joints/slide_x"


class SensorScene(SimHardwareModule):
    mounts = Mounts(*(_TOOL + "/" + kind for kind in ("camera", "imu", "lidar")))


@pytest.fixture
def deployed_world(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    imu = SimImuModule.attach(SensorScene.mounts.imu, sensor=ImuConfig("imu")).remappings(
        [(SimImuModule, "imu", "imu")]
    )
    camera = SimCameraModule.attach(SensorScene.mounts.camera, sensor=CameraConfig("camera"))
    lidar = SimLidarModule.attach(
        SensorScene.mounts.lidar, sensor=LidarConfig("lidar"), raycaster=MujocoRaycaster
    )
    blueprint = autoconnect(
        SensorScene.blueprint(usd=world, instance_name="scene"),
        SimModule.blueprint(
            gravity_world=(0, 0, -1.62), backend=MujocoBackend, joint_ids=(_JOINT,)
        ),
        imu,
        camera,
        lidar,
        SimJointEncoderModule.blueprint(sensor=EncoderConfig((_JOINT,))),
        CameraRenderModule.blueprint(
            renderer=MujocoRenderer, cameras=tuple(a.name for a in camera.active_blueprints)
        ),
    ).global_config(
        viewer="none",
        transport="zenoh",
        n_workers=4,
        zenoh_domain="sim2-test-" + uuid.uuid4().hex,
        zenoh_scout_addr="224.0.0.227:7446",
    )
    deployment = ModuleCoordinator.build(blueprint)
    try:
        yield deployment
    finally:
        deployment.stop()


def test_separate_sensor_modules_run_with_standard_timestamps(deployed_world, command):
    sim = deployed_world.get_instance(SimModule)
    camera = deployed_world.get_instance(SimCameraModule)
    lidar = deployed_world.get_instance(SimLidarModule)
    wait_until(lambda: camera.statistics()["acquisitions"] >= 2, timeout=20)

    def acquired():
        stats = lidar.statistics()
        assert not stats["error"], stats
        return stats["acquisitions"] >= 2

    wait_until(acquired, timeout=30)
    # The deployed IMU receives non-Earth gravity through its ordinary simulation_config RPC.
    samples = Queue()
    transport = deployed_world.transports[("imu", Imu)]
    with ExitStack() as cleanup:
        cleanup.callback(transport.stop)
        cleanup.callback(transport.subscribe(samples.put))
        sample = samples.get(timeout=5)
    assert sample.linear_acceleration.z == pytest.approx(1.62, abs=1e-8)
    before = sim.inspect()
    h = before.header
    sim.submit(command(h, positions=(0.65,)))
    wait_until(
        lambda: poses_by_id(sim.inspect())["world/world/box_mount/box/box"].position.x > 0.1,
        timeout=5,
    )
    count = camera.statistics()["acquisitions"]
    wait_until(lambda: camera.statistics()["acquisitions"] > count, timeout=5)
    assert camera.statistics()["error"] == lidar.statistics()["error"] == ""
    assert sim.status()["error"] == sim.status()["publication_error"] == ""
