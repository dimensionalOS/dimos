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
from pathlib import Path
import struct
import threading
from uuid import uuid4

import numpy as np
import pytest

from dimos.sim2.connections.whole_body import WholeBodyConnection
from dimos.sim2.control.interface import descriptor
from dimos.sim2.ipc.channel import ChannelFrame, FrameMetadata, RobotChannel
from dimos.sim2.sensors.spec import Imu
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig


@pytest.fixture
def connection():
    definition = RobotConfig(
        model=Path("unused.xml"),
        root_body="base",
        control=ControlInterface.WHOLE_BODY,
        joints=(Joint("joint", "joint", "motor", home=0.25),),
        sensors=(Imu("imu", "imu"),),
    )
    module = WholeBodyConnection(definition=definition, address="test/robot", robot_id="robot")
    yield module
    module.stop()


def test_transient_contention_skips_sample_and_resumes_publication(connection, mocker):
    desc = connection._device.definition
    values = {
        "position": np.array([j.home for j in desc.joints]),
        "velocity": np.zeros(1),
        "effort": np.zeros(1),
        "wall_time": np.array([3.0]),
        "imu_quaternion": np.array([1.0, 0, 0, 0]),
        "imu_gyroscope": np.zeros(3),
        "imu_accelerometer": np.array([0.0, 0, 9.81]),
        "root_position": np.array([0.0, 0, 0.3]),
        "root_quaternion": np.array([1.0, 0, 0, 0]),
    }
    sample = mocker.patch.object(
        connection._device,
        "sample",
        side_effect=[
            BlockingIOError("writer busy"),
            ChannelFrame(FrameMetadata(1, 1, 1, 0, 1), values),
        ],
    )
    wait = mocker.patch.object(connection._stop, "wait", return_value=False)
    joints = mocker.patch.object(connection.motor_states, "publish")
    imu = mocker.patch.object(connection.imu, "publish")
    mocker.patch.object(connection.odom, "publish")
    mocker.patch.object(connection.tf, "publish", side_effect=lambda _: connection._stop.set())
    error = mocker.patch("dimos.sim2.connections.whole_body.logger.exception")

    connection._publish()

    assert sample.call_count == 2
    wait.assert_any_call(1.0 / connection.config.rate_hz)
    assert joints.call_count == imu.call_count == 1
    assert joints.call_args.args[0].ts == imu.call_args.args[0].ts == 3.0
    assert joints.call_args.args[0].position == values["position"].tolist()
    error.assert_not_called()


def test_non_contention_error_is_not_suppressed(connection, mocker):
    sample = mocker.patch.object(connection._device, "sample", side_effect=RuntimeError("closed"))
    publish = mocker.patch.object(connection.motor_states, "publish")
    error = mocker.patch("dimos.sim2.connections.whole_body.logger.exception")

    connection._publish()

    sample.assert_called_once_with()
    publish.assert_not_called()
    error.assert_called_once_with("sim2 whole-body connection stopped")


@pytest.fixture
def channel(request):
    desc = replace(
        descriptor(uuid4().hex + "/robot", ControlInterface.WHOLE_BODY, 1),
        observation_slots=request.param,
    )
    with RobotChannel.create(desc) as owner:
        yield owner


@pytest.mark.parametrize(
    ("direction", "sequence_offset", "channel"),
    [("action", 24, 2), ("observation", 32, 2), ("observation", 32, 42)],
    indirect=["channel"],
)
def test_read_retains_committed_frame_while_writer_switches_slots(
    channel, direction, sequence_offset, mocker
):
    layout = getattr(channel.descriptor, direction + "_layout")
    publish = getattr(channel, "publish_" + direction)
    read = getattr(channel, "read_" + direction)
    old = {field.name: np.ones(field.shape) for field in layout.fields}
    new = {field.name: np.full(field.shape, 2) for field in layout.fields}
    publish(old, FrameMetadata(0, 1, 1, 0, 0.01))
    pending, resume, finished = threading.Event(), threading.Event(), threading.Event()
    pack = struct.pack_into

    def pause_before_commit(fmt, buffer, offset, *values):
        pack(fmt, buffer, offset, *values)
        if offset == sequence_offset:
            pending.set()
            assert resume.wait(5), "reader did not release the writer"

    def writer():
        publish(new, FrameMetadata(0, 1, 2, 0, 0.02))
        finished.set()

    mocker.patch("dimos.sim2.ipc.channel.struct.pack_into", side_effect=pause_before_commit)
    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    try:
        assert pending.wait(5), "writer did not reach the commit boundary"
        frame = read(retries=1)
        assert frame.metadata.physics_tick == 1
        np.testing.assert_array_equal(frame.values["position"], old["position"])
    finally:
        resume.set()
        thread.join(timeout=5)
    assert finished.is_set()
    frame = read(retries=1)
    assert frame.metadata.physics_tick == 2
    np.testing.assert_array_equal(frame.values["position"], new["position"])


@pytest.mark.parametrize("channel", [2], indirect=True)
def test_overwritten_slot_is_busy_not_returned_as_feedback(channel):
    values = {
        field.name: np.ones(field.shape) for field in channel.descriptor.observation_layout.fields
    }
    channel.publish_observation(values, FrameMetadata(0, 1, 1, 0, 0.01))
    active = struct.unpack_from("<I", channel._buffer, 16)[0]
    slot = (
        channel.descriptor.observation_offset
        + active * channel.descriptor.observation_layout.slot_size
    )
    struct.pack_into("<Q", channel._buffer, slot, 0)
    with pytest.raises(BlockingIOError, match="coherent observation"):
        channel.read_observation(retries=1)
