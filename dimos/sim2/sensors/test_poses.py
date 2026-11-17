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
import threading

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import pytest
from reactivex.subject import Subject

from dimos.robot.assets.model import UsdModel
from dimos.sim2.sensors.camera.render_module import CameraRenderModule


@pytest.fixture
def sensor_input(mocker):
    # This exercises the input queue; opening the rendering scene is mocked below.
    scene = UsdModel.from_layer('#usda 1.0\ndef Xform "world" {}\n')
    module = CameraRenderModule(scene=scene, renderer=mocker.Mock(), cameras=("camera",))
    received = []
    consuming, release, drained = threading.Event(), threading.Event(), threading.Event()

    def acquire(sample, *, latest):
        received.append(sample.header.time_ns)
        if len(received) == 1:
            consuming.set()
            assert release.wait(5)
        if latest and sample.header.time_ns > 0:
            drained.set()

    mocker.patch.object(module, "open_sensor")
    mocker.patch.object(module, "close_sensor")
    mocker.patch.object(module, "acquire", side_effect=acquire)
    with ExitStack() as cleanup:
        source = Subject()
        cleanup.callback(source.dispose)
        mocker.patch.object(
            module.observations, "subscribe", side_effect=lambda cb: source.subscribe(cb).dispose
        )
        cleanup.callback(module.stop)
        module.start()
        blocked, unblock = threading.Event(), threading.Event()

        async def busy_loop():
            blocked.set()
            assert unblock.wait(5)

        future = module.spawn(busy_loop())
        cleanup.callback(lambda: future.result(timeout=5))
        cleanup.callback(unblock.set)
        cleanup.callback(release.set)
        assert blocked.wait(5)
        yield module, source, received, consuming, release, drained


def test_sensor_retains_ordered_samples_while_dispatch_and_acquisition_are_busy(sensor_input):
    module, source, received, consuming, release, drained = sensor_input
    source.on_next(WorldState(header=SampleHeader(time_ns=0)))
    assert consuming.wait(5)
    for tick in range(1, 51):
        source.on_next(WorldState(header=SampleHeader(time_ns=tick * 2_000_000)))
    release.set()
    assert drained.wait(5)
    assert received == [tick * 2_000_000 for tick in range(51)]
    assert module.statistics()["dropped_inputs"] == 0


def test_sensor_bounds_backlog_and_reports_each_overflow(sensor_input):
    module, source, received, consuming, release, drained = sensor_input
    source.on_next(WorldState(header=SampleHeader(time_ns=0)))
    assert consuming.wait(5)
    for tick in range(1, 131):
        source.on_next(WorldState(header=SampleHeader(time_ns=tick * 2_000_000)))
    release.set()
    assert drained.wait(5)
    assert received == [0, *[tick * 2_000_000 for tick in range(3, 131)]]
    assert module.statistics()["dropped_inputs"] == 2
