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
import multiprocessing

import numpy as np
import pytest
from robosuite.utils.binding_utils import MjRenderContextOffscreen, MjSim
from robosuite.utils.observables import (
    Observable,
    create_deterministic_corrupter,
    create_deterministic_delayer,
    sensor as observable_sensor,
)

from dimos.sim2.messages import FrameMetadata
from dimos.sim2.sensors.camera.messages import camera_messages
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.messages import imu_message
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.observations import (
    ObservationFrame,
    collect_observations,
    create_observables,
)

XML = """
<mujoco>
  <option timestep=".005" gravity="0 0 0"/>
  <worldbody>
    <light pos="0 0 4"/>
    <geom name="floor" type="plane" size="5 5 .1" rgba=".3 .5 .7 1"/>
    <body name="robotarm_base" pos="0 0 .3">
      <geom type="box" size=".2 .2 .3" mass="1" rgba="1 .1 .1 1"/>
      <site name="robotarm_imu"/>
      <camera name="robotarm_view" pos="2 -2 2" xyaxes=".707 .707 0 -.408 .408 .816"/>
    </body>
  </worldbody>
  <sensor>
    <gyro name="robotarm_sensor/imu/gyro" site="robotarm_imu"/>
    <accelerometer name="robotarm_sensor/imu/accel" site="robotarm_imu"/>
  </sensor>
</mujoco>
"""


@pytest.fixture
def sim():
    value = MjSim.from_xml_string(XML)
    value.forward()
    try:
        yield value
    finally:
        value.free()


def _update(observables, cache, timestep, *, force=False):
    for observable in observables.values():
        observable.update(timestep, cache, force=force)
    return {name: observable.obs for name, observable in observables.items()}


def test_imu_uses_upstream_sampling_and_delay_semantics(sim):
    descriptor = Imu("imu", "imu", rate_hz=10, latency=0.025)
    observables = create_observables(sim, descriptor, "robotarm_", "arm")

    @observable_sensor(modality="time")
    def clock(cache):
        return np.array([sim.data.time])

    reference = Observable(
        "clock", clock, sampling_rate=10, delayer=create_deterministic_delayer(0.025)
    )
    cache, reference_cache, previous = {}, {}, {}
    frames = []
    for step in range(60):
        sim.step()
        values = _update(observables, cache, 0.005)
        reference.update(0.005, reference_cache)
        assert values["arm/imu/metadata"][1] == pytest.approx(reference.obs)
        frames.extend(
            collect_observations(
                values,
                (("arm", descriptor),),
                FrameMetadata(step, 1, step, sim.data.time),
                1000.0,
                previous,
            )
        )
    assert 2 <= len(frames) <= 4
    assert len({frame.timestamp for frame in frames}) == len(frames)
    assert all(
        frame.timestamp == pytest.approx(1000 + frame.values["metadata"][0]) for frame in frames
    )


def test_imu_observable_corruption_reaches_the_standard_message(sim):
    descriptor = Imu("imu", "imu")
    observables = create_observables(sim, descriptor, "robotarm_", "arm")
    observables["arm/imu/imu"].set_corrupter(create_deterministic_corrupter(0.25))
    values = _update(observables, {}, 0.005, force=True)
    (frame,) = collect_observations(
        values,
        (("arm", descriptor),),
        FrameMetadata(1, 1, 0, 0),
        1234.0,
        {},
    )
    message = imu_message(frame)
    assert message.angular_velocity.to_tuple() == pytest.approx((0.25, 0.25, 0.25))
    assert message.linear_acceleration.to_tuple() == pytest.approx((0.25, 0.25, 0.25))
    assert message.orientation_covariance[0] == -1
    assert message.frame_id == "arm/imu"
    assert message.ts == 1234.0


def test_changing_upstream_rate_keeps_payload_and_acquisition_time_together(sim):
    descriptor = Imu("imu", "imu", rate_hz=10)
    observables = create_observables(sim, descriptor, "robotarm_", "arm")
    payload = observables["arm/imu/imu"]
    payload.set_sampling_rate(20)
    payload.set_filter(lambda value: np.full_like(value, sim.data.time))
    cache, previous, frames = {}, {}, []
    for step in range(40):
        sim.step()
        values = _update(observables, cache, 0.005)
        frames.extend(
            collect_observations(
                values,
                (("arm", descriptor),),
                FrameMetadata(step, 1, step, sim.data.time),
                1000.0,
                previous,
            )
        )
    assert len(frames) >= 3
    assert all(frame.values["imu"][0] == frame.values["metadata"][0] for frame in frames)


@pytest.mark.parametrize("image_rate,depth_rate", [(20, 5), (5, 20)])
def test_rgb_depth_keep_individual_capture_times_when_observable_rates_differ(
    sim, mocker, image_rate, depth_rate
):
    descriptor = Camera("camera", "view", width=2, height=2, pointcloud=True)
    mocker.patch.object(
        sim,
        "render",
        return_value=(np.ones((2, 2, 3), dtype=np.uint8), np.full((2, 2), 0.5)),
    )
    cloud = mocker.patch("dimos.sim2.sensors.camera.messages.PointCloud2.from_rgbd")
    observables = create_observables(sim, descriptor, "robotarm_", "arm")
    observables["arm/camera/image"].set_sampling_rate(image_rate)
    observables["arm/camera/image"].set_corrupter(create_deterministic_corrupter(0.25))
    observables["arm/camera/depth"].set_sampling_rate(depth_rate)
    observables["arm/camera/depth"].set_delayer(create_deterministic_delayer(0.02))
    cache, previous, frames = {}, {}, []
    for step in range(60):
        sim.step()
        values = _update(observables, cache, 0.005)
        frames.extend(
            collect_observations(
                values,
                (("arm", descriptor),),
                FrameMetadata(step, 1, step, sim.data.time),
                1000.0,
                previous,
            )
        )
    mismatched = [
        frame
        for frame in frames
        if (
            "depth" in frame.values
            and frame.values["timestamps"]["image"] != frame.values["timestamps"]["depth"]
        )
    ]
    assert mismatched
    for frame in mismatched:
        messages = camera_messages(descriptor, frame)
        for name, port in (("image", "color_image"), ("depth", "depth_image")):
            assert (port in messages) == (name in frame.values["changed"])
            if port in messages:
                assert messages[port].ts == frame.values["timestamps"][name]
        assert "pointcloud" not in messages
    cloud.assert_not_called()
    counts = {
        name: sum(name in frame.values["changed"] for frame in frames)
        for name in ("image", "depth")
    }
    assert (counts["image"] > counts["depth"]) == (image_rate > depth_rate)
    # A slower environment observer may receive two independently new captures.
    (latest,) = collect_observations(
        values,
        (("arm", descriptor),),
        FrameMetadata(61, 1, 60, sim.data.time),
        1000.0,
        {},
    )
    messages = camera_messages(descriptor, latest)
    assert messages["color_image"].data.dtype == np.uint8
    assert {transform.ts for transform in messages["tf"]} == set(
        latest.values["timestamps"].values()
    )
    assert len(messages["tf"]) == 2


def test_collection_keeps_sample_time_and_accepts_reset_generation(sim):
    descriptor = Imu("imu", "imu", rate_hz=10)
    observables = create_observables(sim, descriptor, "robotarm_", "arm")
    assert (
        collect_observations(
            {name: observable.obs for name, observable in observables.items()},
            (("arm", descriptor),),
            FrameMetadata(0, 1, 0, 0),
            2000.0,
            {},
        )
        == []
    )
    values = _update(observables, {}, 0.005, force=True)
    previous = {}
    (first,) = collect_observations(
        values,
        (("arm", descriptor),),
        FrameMetadata(1, 1, 0, 0),
        2000.0,
        previous,
    )
    assert (
        collect_observations(
            values,
            (("arm", descriptor),),
            FrameMetadata(2, 1, 1, 0.005),
            2000.0,
            previous,
        )
        == []
    )
    (reset,) = collect_observations(
        values,
        (("arm", descriptor),),
        FrameMetadata(3, 2, 0, 0),
        3000.0,
        previous,
    )
    assert first.timestamp == 2000.0
    assert reset.timestamp == 3000.0
    assert reset.metadata.episode_id == 2


@pytest.fixture
def imu_bridge():
    descriptor = Imu("imu", "imu")
    module = SimImuModule(sensor=descriptor, robot_id="arm")
    try:
        yield module
    finally:
        module.stop()


def test_imu_bridge_drops_duplicate_old_and_other_sensor_frames(mocker, imu_bridge):
    module = imu_bridge
    publish = mocker.patch.object(module.imu, "publish")
    frame = ObservationFrame(
        FrameMetadata(2, 2, 3, 0.015),
        "arm",
        "imu",
        1234.0,
        {"imu": np.arange(6)},
    )
    module._receive(frame)
    module._receive(frame)
    module._receive(replace(frame, metadata=FrameMetadata(1, 1, 0, 0)))
    module._receive(replace(frame, robot_id="other"))
    publish.assert_called_once()
    assert publish.call_args.args[0].ts == 1234.0
    module._receive(replace(frame, metadata=FrameMetadata(3, 3, 0, 0)))
    assert publish.call_count == 2


def _render(pipe):
    sim = MjSim.from_xml_string(XML)
    MjRenderContextOffscreen(sim, device_id=-1)
    try:
        descriptor = Camera("camera", "view", width=160, height=120)
        observables = create_observables(sim, descriptor, "robotarm_", "arm")
        values = _update(observables, {}, 0.005, force=True)
        (frame,) = collect_observations(
            values,
            (("arm", descriptor),),
            FrameMetadata(1, 1, 0, 0),
            1000.0,
            {},
        )
        messages = camera_messages(descriptor, frame)
        pipe.send(
            (
                messages["color_image"].data,
                messages["depth_image"].data,
                messages["camera_info"].frame_id,
                messages["color_image"].ts,
            )
        )
    finally:
        sim.free()
        pipe.close()


@pytest.mark.self_hosted
def test_upstream_camera_produces_rgb_and_metric_depth():
    context = multiprocessing.get_context("spawn")
    reader, writer = context.Pipe(duplex=False)
    process = context.Process(target=_render, args=(writer,))
    try:
        process.start()
        writer.close()
        assert reader.poll(30), "upstream camera did not render"
        rgb, depth, frame_id, timestamp = reader.recv()
        process.join(5)
        assert process.exitcode == 0
        assert rgb.shape == (120, 160, 3)
        assert rgb.std() > 15
        assert depth.shape == (120, 160)
        assert np.count_nonzero((depth > 0.1) & (depth < 10)) > 1000
        assert frame_id == "arm/camera_optical"
        assert timestamp == 1000.0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        reader.close()
        writer.close()
