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

"""Integrated IMU delivery must not wait for rolling lidar ray acquisition."""

from dataclasses import replace
from functools import partial

import numpy as np
import pytest

from dimos.sim2.environment import MotorScene
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.observations import ObservationFrame


@pytest.fixture
def lidar_module(request):
    rate, latency = getattr(request, "param", (200, 0))
    module = LidarModule(
        robot_id="left",
        sensor=Lidar(
            "lidar",
            "scan",
            Mid360,
            imu=Imu("lidar_imu", "scan", rate_hz=rate, latency=latency),
            frame_id="mid360_link",
            imu_frame_id="imu_link",
            truth_outputs=False,
        ),
    )
    yield module
    module.stop()


def test_integrated_imu_delivers_every_physics_sample_without_ray_acquisition(
    world_config, worlds, scene_path, robot_factory, lidar_module, mocker
):
    sensors = (("left", lidar_module.config.sensor),)
    config = {
        **world_config,
        "sensors": sensors,
        "environment": partial(MotorScene, scene_path, {"left": robot_factory}, sensors=sensors),
    }
    world = worlds(config=config)
    mocker.patch.object(world, "publish_observation", side_effect=lidar_module._on_observation)
    publish = mocker.patch.object(lidar_module.imu, "publish")
    # No lidar worker or ray query runs: IMU acquisition belongs to the backend.
    for _ in range(40):
        world.step()

    messages = [call.args[0] for call in publish.call_args_list]
    assert len(messages) == 40
    assert {message.frame_id for message in messages} == {"imu_link"}
    assert np.diff([message.ts for message in messages]) == pytest.approx([0.005] * 39, abs=3e-7)
    assert messages[-1].angular_velocity.to_tuple() == pytest.approx((0, 0, 0))
    assert messages[-1].linear_acceleration.to_tuple() == pytest.approx((0, 0, 0))
    assert lidar_module.sensor_status()["dropped_imu_samples"] == 0


def test_integrated_imu_filters_other_devices_and_old_epochs(lidar_module, mocker):
    publish = mocker.patch.object(lidar_module.imu, "publish")
    frame = ObservationFrame(
        FrameMetadata(5, 2, 3, 0.015),
        "left",
        "lidar_imu",
        1000.0,
        {"imu": np.arange(6), "metadata": np.array([0.015])},
    )
    lidar_module._on_observation(frame)
    lidar_module._on_observation(frame)
    lidar_module._on_observation(replace(frame, robot_id="right"))
    lidar_module._on_observation(replace(frame, sensor_name="other_imu"))
    lidar_module._on_observation(replace(frame, metadata=FrameMetadata(6, 1, 4, 0.02)))
    publish.assert_called_once()
    message = publish.call_args.args[0]
    assert message.angular_velocity.to_tuple() == (0, 1, 2)
    assert message.linear_acceleration.to_tuple() == (3, 4, 5)
    assert message.ts == 1000.0
    lidar_module._on_observation(
        replace(
            frame,
            metadata=FrameMetadata(7, 3, 0, 0),
            timestamp=2000.0,
            values={"imu": np.arange(6), "metadata": np.array([0.0])},
        )
    )
    assert publish.call_count == 2
    assert lidar_module.sensor_status()["dropped_imu_samples"] == 0


@pytest.mark.parametrize("lidar_module", [(200, 0.025), (10, 0.025)], indirect=True)
def test_integrated_imu_latency_uses_physics_deadlines_without_slowing_acquisition(
    world_config, worlds, scene_path, robot_factory, lidar_module, mocker
):
    sensors = (("left", lidar_module.config.sensor),)
    world = worlds(
        config={
            **world_config,
            "sensors": sensors,
            "environment": partial(
                MotorScene, scene_path, {"left": robot_factory}, sensors=sensors
            ),
        }
    )
    acquired = {}
    delivered = []

    def receive(frame):
        acquired[frame.timestamp] = float(frame.values["metadata"][0])
        lidar_module._on_observation(frame)

    def publish(message):
        delivered.append((world.sim_time, message.ts))

    mocker.patch.object(world, "publish_observation", side_effect=receive)
    mocker.patch.object(world, "publish_snapshot", side_effect=lidar_module._on_snapshot)
    mocker.patch.object(lidar_module.imu, "publish", side_effect=publish)
    for _ in range(40):
        world.step()

    assert len(delivered) >= 2
    assert [time - acquired[stamp] for time, stamp in delivered] == pytest.approx(
        [0.025] * len(delivered), abs=1e-9
    )
    assert lidar_module.sensor_status()["dropped_imu_samples"] == 0
    assert len(acquired) >= len(delivered)


@pytest.mark.parametrize("lidar_module", [(200, 0.025)], indirect=True)
def test_reset_snapshot_discards_delayed_imu_and_rejects_late_old_observations(
    lidar_module, mocker
):
    publish = mocker.patch.object(lidar_module.imu, "publish")
    old = ObservationFrame(
        FrameMetadata(1, 1, 1, 0.005),
        "left",
        "lidar_imu",
        1000.005,
        {"imu": np.arange(6), "metadata": np.array([0.005])},
    )
    lidar_module._on_observation(old)
    assert lidar_module.sensor_status()["pending_imu_samples"] == 1
    lidar_module._on_snapshot(StateFrame(FrameMetadata(2, 2, 0, 0), {}))
    lidar_module._on_observation(replace(old, metadata=FrameMetadata(3, 1, 2, 0.010)))
    lidar_module._on_snapshot(StateFrame(FrameMetadata(4, 2, 8, 0.040), {}))

    publish.assert_not_called()
    assert lidar_module.sensor_status()["pending_imu_samples"] == 0
    current = replace(
        old,
        metadata=FrameMetadata(5, 2, 9, 0.045),
        timestamp=2000.045,
        values={"imu": np.arange(6), "metadata": np.array([0.045])},
    )
    lidar_module._on_observation(current)
    lidar_module._on_snapshot(StateFrame(FrameMetadata(6, 2, 14, 0.070), {}))
    publish.assert_called_once()
    assert publish.call_args.args[0].ts == 2000.045
