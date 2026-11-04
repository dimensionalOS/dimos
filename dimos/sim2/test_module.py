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

from dataclasses import replace
from queue import Queue
import uuid

from dimos_lcm.control_msgs import BodyWrench
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import ControllerConfig, SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def deployed_world(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    # Consumers know instance IDs, placements and drives; only the manager has models.
    scene = replace(world, instances=tuple(replace(i, model=None) for i in world.instances))
    blueprint = autoconnect(
        UrdfManager.blueprint(models={i.id: i.model for i in world.instances}),
        SimModule.blueprint(
            scene=scene,
            backend=MujocoBackend,
            motion_frames=("left/imu",),
            joint_ids=("left/slide_x",),
        ),
        SimImuModule.blueprint(sensor=ImuConfig("left/imu")),
        SimJointEncoderModule.blueprint(sensor=EncoderConfig(("left/slide_x",))),
        SimCameraModule.blueprint(
            scene=scene, sensor=CameraConfig("left/camera"), renderer=MujocoRenderer
        ),
        SimLidarModule.blueprint(
            scene=scene, sensor=LidarConfig("left/lidar"), raycaster=MujocoRaycaster
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


def test_separate_sensor_modules_run_reset_and_recover(deployed_world):
    sim = deployed_world.get_instance(SimModule)
    camera = deployed_world.get_instance(SimCameraModule)
    lidar = deployed_world.get_instance(SimLidarModule)
    wait_until(lambda: camera.statistics()["acquisitions"] >= 2, timeout=20)
    wait_until(lambda: lidar.statistics()["acquisitions"] >= 2, timeout=20)
    before = sim.inspect()[0]
    h = before.header
    sim.submit(
        BodyWrench(
            header=h,
            body_id="box",
            apply_at_ns=h.time_ns,
            valid_until_ns=h.time_ns + 300_000_000,
            force_world=[5, 0, 0],
            torque_world=[0, 0, 0],
            point_world=[0, 0, 0.055],
        )
    )
    wait_until(lambda: poses_by_id(sim.inspect()[0])["box"].position.x > 0.1, timeout=5)
    reset = sim.reset()[0]
    assert reset.header.epoch == h.epoch + 1
    assert poses_by_id(reset)["box"].position.z == pytest.approx(0.055)
    count = camera.statistics()["acquisitions"]
    wait_until(lambda: camera.statistics()["acquisitions"] > count, timeout=5)
    assert camera.statistics()["error"] == lidar.statistics()["error"] == ""
    assert sim.status()["error"] == sim.status()["publication_error"] == ""


@pytest.fixture
def controlled(make_sim):
    return make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        joint_ids=("left/slide_x",),
    )


def test_missing_controller_pauses_without_stepping_or_resetting(make_sim):
    sim = make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        startup_timeout_s=0.5,
        control_timeout_s=0.1,
    )
    wait_until(lambda: sim.status()["status"] == "fault", timeout=3)
    status = sim.status()
    assert status["time_ns"] == 0
    assert status["epoch"] == 0
    assert "deadline" in status["error"]


def test_control_uses_exact_boundary_then_waits_for_next(controlled, command):
    controlled.submit(command(controlled.inspect()[0].header))
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    assert controlled.status()["epoch"] == 0
    assert controlled.inspect()[0].header.time_ns == 20_000_000


def test_nondivisible_export_rate_has_no_rounding_drift(make_sim, command):
    stamps = Queue()

    def receive(sample):
        if sample.header.epoch == 1:
            stamps.put(sample.header.time_ns)

    sim = make_sim(
        controllers=(ControllerConfig("test", 20_000_000, ("left/slide_x",)),),
        on_poses=receive,
    )
    baseline = sim.reset()[0]
    # Wait for the reset sample to arrive before releasing the controller boundary.
    assert stamps.get(timeout=3) == 0
    sim.submit(command(baseline.header))
    assert [stamps.get(timeout=3) for _ in range(4)] == [
        6_000_000,
        10_000_000,
        16_000_000,
        20_000_000,
    ]


def test_wrong_clock_and_duplicate_commands_are_rejected(controlled, command):
    header = controlled.inspect()[0].header
    wrong = command(header)
    wrong.header.clock_id = "other-simulation"
    good = command(header)
    controlled.submit(wrong)
    controlled.submit(good)
    controlled.submit(good)
    wait_until(lambda: controlled.status()["rejected_commands"] == 2, timeout=3)
    wait_until(lambda: controlled.status()["time_ns"] == 20_000_000, timeout=3)
    assert controlled.status()["error"] == ""


def test_command_expiry_holds_motor_and_reset_rejects_previous_epoch(make_sim, command):
    sim = make_sim(
        controllers=(ControllerConfig("test", 1_220_000_000, ("right/slide_x",)),),
    )
    header = sim.inspect()[0].header
    short = command(header, positions=(0.6,), ttl=0.02)
    short.header.source_id = "uncontrolled"
    sim.submit(short)
    sim.submit(command(header, names=("right/slide_x",), positions=(0.0,), ttl=1.22))
    wait_until(lambda: sim.status()["time_ns"] == 1_220_000_000, timeout=5)
    poses, joints = sim.inspect(("left/slide_x",))
    assert joints.position[0] < 0.2
    stale = command(poses.header, names=("right/slide_x",), positions=(0.0,), ttl=1.22)
    reset = sim.reset()[0]
    sim.submit(stale)
    wait_until(lambda: sim.status()["rejected_commands"] == 1, timeout=3)
    assert reset.header.epoch == header.epoch + 1
    assert reset.header.time_ns == 0
    assert sim.inspect(("right/slide_x",))[1].position[0] == pytest.approx(0)


def test_place_is_atomic_and_invalidates_old_samples(controlled):
    before = controlled.inspect()[0]
    with pytest.raises(ValueError, match="free root"):
        controlled.place("left/tool", (0.2, 0.2, 0.4, 0, 0, 0, 1))
    assert controlled.inspect()[0].header.epoch == before.header.epoch
    placed = controlled.place("box", (0.2, 0.2, 0.4, 0, 0, 0, 1))[0]
    position = poses_by_id(placed)["box"].position
    assert (position.x, position.y, position.z) == pytest.approx((0.2, 0.2, 0.4))
    assert placed.header.epoch == before.header.epoch + 1
    assert placed.header.time_ns == 0
