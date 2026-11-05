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

"""Real G1 geometry and GR00T weights through ordinary worker/transport lifecycle."""

from collections import defaultdict, deque
from contextlib import ExitStack
from dataclasses import replace
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import (
    JOINTS,
    LIDAR,
    MODELS,
    SCENE,
    unitree_g1_sim2,
    unitree_g1_sim2_pointlio,
)
from dimos.robot.unitree.g1.sim2_policy import G1PolicyModule
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.native_e2e


@pytest.fixture
def deployed_g1(tmp_path, monkeypatch, g1_configuration):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    blueprint = g1_configuration[0].global_config(zenoh_domain="sim2-g1-" + uuid.uuid4().hex)
    # Exercise the CLI configuration round-trip, including sensor factories.
    parsed = BlueprintConfigParser(blueprint).parse()
    deployment = ModuleCoordinator.build(blueprint, parsed_config=parsed)
    try:
        yield deployment
    finally:
        deployment.stop()


@pytest.fixture(
    params=[(unitree_g1_sim2, "world"), (unitree_g1_sim2_pointlio, "odom")],
    ids=["truth-map", "pointlio-map"],
)
def g1_configuration(request):
    return request.param


@pytest.fixture
def g1_measurements(deployed_g1):
    samples = {}
    with ExitStack() as cleanup:
        for topic, message in (
            ("global_map", PointCloud2),
            ("lidar_raw", PointCloud2),
            ("filtered_pointcloud", PointCloud2),
            ("imu_raw", Imu),
            ("tf", TFMessage),
        ):
            samples[topic] = deque(maxlen=100)
            transport = deployed_g1.transports[(topic, message)]
            cleanup.callback(transport.stop)
            cleanup.callback(transport.subscribe(samples[topic].append))
        yield samples


def test_g1_walks_and_moves_an_arm_with_standard_sensor_messages(
    deployed_g1, g1_measurements, g1_configuration
):
    sim = deployed_g1.get_instance(SimModule)
    policy = deployed_g1.get_instance(G1PolicyModule)
    camera = deployed_g1.get_instance(SimCameraModule)
    lidar = deployed_g1.get_instance(SimLidarModule)
    wait_until(lambda: sim.status()["time_ns"] > 2_000_000_000, timeout=40)
    wait_until(lambda: camera.statistics()["acquisitions"] >= 3, timeout=20)

    def scans_ready():
        stats = lidar.statistics()
        assert not stats["error"], stats
        return stats["acquisitions"] >= 3

    wait_until(scans_ready, timeout=20, message=str(lidar.statistics()))
    wait_until(lambda: bool(g1_measurements["global_map"]), timeout=40)
    cloud = g1_measurements["global_map"][-1]
    assert len(cloud) > 300
    assert cloud.frame_id == g1_configuration[1]
    assert g1_measurements["filtered_pointcloud"]
    assert cloud.ts is not None
    assert 0 <= cloud.ts <= sim.inspect().header.time_ns / 1e9
    raw = g1_measurements["lidar_raw"][-1]
    assert raw.frame_id == "g1/mid360_link"
    assert set(np.unique(raw.lines_u8())) == {0, 1, 2, 3}
    assert g1_measurements["imu_raw"][-1].frame_id == "g1/mid360_link"
    parents = defaultdict(set)
    for message in list(g1_measurements["tf"]):
        for transform in message.transforms:
            parents[transform.child_frame_id].add(transform.frame_id)
    assert all(len(p) == 1 for p in parents.values()), dict(parents)
    expected_parent = "odom" if g1_configuration[1] == "odom" else "g1/torso_link"
    assert parents["g1/mid360_link"] == {expected_parent}
    before = sim.inspect()
    before_inferences = policy.status()["inferences"]
    assert poses_by_id(before)["g1/pelvis"].position.z > 0.68
    policy.arm_joint("g1/left_elbow_joint", 1.4)
    policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=20)
    wait_until(lambda: sim.status()["time_ns"] > before.header.time_ns + 3_000_000_000, timeout=40)
    after = sim.inspect()
    joints = after.joint_truth
    policy.move(Twist(), duration=1)
    assert poses_by_id(after)["g1/pelvis"].position.z > 0.68
    assert (
        poses_by_id(after)["g1/pelvis"].position.x - poses_by_id(before)["g1/pelvis"].position.x
        > 0.2
    )
    assert joints.position[joints.name.index("g1/left_elbow_joint")] == pytest.approx(1.4, abs=0.12)
    assert poses_by_id(sim.inspect())["g1/pelvis"].position.z > 0.68
    assert (
        sim.status()["error"] == camera.statistics()["error"] == lidar.statistics()["error"] == ""
    )
    assert policy.status()["inferences"] >= before_inferences + 150


def test_g1_mid360_mount_observes_the_floor_instead_of_its_head():
    scene = replace(SCENE, instances=tuple(replace(i, model=MODELS[i.id]) for i in SCENE.instances))
    with ExitStack() as cleanup:
        backend = MujocoBackend()
        cleanup.callback(backend.close)
        backend.load(scene, 2_000_000)
        lidar = SimLidarModule(scene=scene, sensor=LIDAR, raycaster=MujocoRaycaster)
        cleanup.callback(lidar.dispose)
        lidar.open_sensor(scene)
        cleanup.callback(lidar.close_sensor)
        self_filter = UrdfManagerPointClearing(
            models=MODELS, padding_m=0.05, filter_geometry="visual", publish_joint_tf=False
        )
        cleanup.callback(self_filter.dispose)
        for tick in range(51):
            bodies = backend.read_bodies()
            state = WorldState(
                header=SampleHeader(clock_id="g1-mount-test", time_ns=tick * 2_000_000),
                model_revision=scene.revision,
                bodies_length=len(bodies),
                bodies=bodies,
                joint_truth=backend.read_joints(JOINTS),
            )
            self_filter._on_observations(state)
            scan = lidar.measure(state)
        assert scan is not None
        raw, world = scan
        floor = world.points_f32()[np.abs(world.points_f32()[:, 2]) < 0.05]
        assert len(floor) > 700
        assert np.ptp(floor[:, 0]) > 4
        assert np.ptp(floor[:, 1]) > 4
        assert np.median(raw.points_f32()[:, 2]) > 0

        filtered_raw = self_filter.filter_cloud(raw)
        filtered_world = self_filter.filter_cloud(world)
        assert filtered_raw is not None and filtered_world is not None
        assert len(filtered_raw) == len(filtered_world) < len(raw)
        points = filtered_world.points_f32()
        assert not np.any((np.linalg.norm(points[:, :2], axis=1) < 0.8) & (points[:, 2] > 0.08))
        # Preserve distant environment returns, rather than hiding the robot from rays.
        outside = np.linalg.norm(world.points_f32()[:, :2], axis=1) > 0.8
        kept_outside = np.linalg.norm(points[:, :2], axis=1) > 0.8
        np.testing.assert_allclose(points[kept_outside], world.points_f32()[outside])


@pytest.mark.parametrize("g1_configuration", [(unitree_g1_sim2, "world")], indirect=True)
def test_g1_truth_map_has_no_body_trail_after_walking(deployed_g1, g1_measurements):
    sim = deployed_g1.get_instance(SimModule)
    policy = deployed_g1.get_instance(G1PolicyModule)
    wait_until(lambda: sim.status()["time_ns"] > 2_000_000_000, timeout=40)
    start = sim.inspect().header.time_ns
    policy.arm_joint("g1/left_elbow_joint", 1.4)
    policy.move(Twist(linear=Vector3(0.2, 0, 0)), duration=20)
    wait_until(lambda: sim.status()["time_ns"] > start + 3_000_000_000, timeout=40)
    policy.move(Twist(), duration=1)
    wait_until(
        lambda: bool(g1_measurements["global_map"])
        and g1_measurements["global_map"][-1].ts > start / 1e9 + 3,
        timeout=20,
    )
    p = g1_measurements["global_map"][-1].points_f32()
    # Only floor lies along this route; the desk and box are outside the corridor.
    corridor = (p[:, 0] > -0.5) & (p[:, 0] < 0.9) & (np.abs(p[:, 1]) < 0.5)
    assert np.count_nonzero(corridor) > 20
    assert not np.any(corridor & (p[:, 2] > 0.15))
