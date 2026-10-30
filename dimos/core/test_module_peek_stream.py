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

from concurrent.futures import ThreadPoolExecutor

import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.module import Module, PeekNotFound
from dimos.core.stream import IO, In, Out
from dimos.core.transport import LCMTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.testing.waiting import wait_until


class PeekModule(Module):
    odom: Out[PoseStamped]
    incoming: In[PoseStamped]
    duplex: IO[PoseStamped]


@pytest.fixture
def module(mocker):
    scheduler = ThreadPoolScheduler(max_workers=2)
    mocker.patch("dimos.utils.reactive.get_scheduler", return_value=scheduler)
    try:
        with PeekModule() as instance:
            yield instance
    finally:
        scheduler.executor.shutdown(wait=True)


@pytest.mark.flaky(reruns=1)
@pytest.mark.parametrize("stream_name", ["odom", "incoming", "duplex"])
def test_peek_observes_external_publications(module, each_transport, stream_name):
    topic = f"/peek/{stream_name}"
    module.set_transport(stream_name, make_transport(topic, PoseStamped))
    expected = PoseStamped(ts=123.0, frame_id="world", position=(1.0, 2.0, 3.0))

    with make_transport(topic, PoseStamped) as publisher, ThreadPoolExecutor() as executor:
        result = executor.submit(module.peek_stream, stream_name, 3.0)

        def publish_until_done():
            publisher.publish(expected)
            return result.done()

        wait_until(publish_until_done, timeout=5.0, interval=0.02)
        pose = result.result()

    assert isinstance(pose, PoseStamped)
    assert pose.ts == expected.ts
    assert pose.frame_id == expected.frame_id
    assert (pose.x, pose.y, pose.z) == (1.0, 2.0, 3.0)


def send_only_transport(mocker) -> LCMTransport:
    # A send-only transport (e.g. WebRTC video) never echoes local publishes.
    transport = LCMTransport("/peek/send_only", PoseStamped)
    mocker.patch.object(transport, "broadcast")
    mocker.patch.object(transport, "subscribe", return_value=mocker.Mock())
    return transport


@pytest.mark.flaky(reruns=1)
@pytest.mark.parametrize("has_transport", [False, True])
def test_peek_preserves_local_output_without_transport_loopback(module, mocker, has_transport):
    if has_transport:
        module.set_transport("odom", send_only_transport(mocker))
    expected = PoseStamped(ts=123.0, position=(4.0, 5.0, 6.0))

    with ThreadPoolExecutor() as executor:
        result = executor.submit(module.peek_stream, "odom", 3.0)

        def publish_until_done():
            module.odom.publish(expected)
            return result.done()

        wait_until(publish_until_done, timeout=5.0, interval=0.02)

    assert result.result() is expected


def test_one_publication_reaches_every_pending_peek_and_subscriber(module, mocker):
    # A peek completes inside Out.publish(); dropping its callback there must not
    # make publish() skip the subscribers registered after it.
    transport = send_only_transport(mocker)
    module.set_transport("odom", transport)
    received = []
    expected = PoseStamped(ts=123.0, position=(4.0, 5.0, 6.0))

    with ThreadPoolExecutor() as executor:
        peeks = [executor.submit(module.peek_stream, "odom", 3.0) for _ in range(2)]
        # Each peek subscribes locally before it subscribes to the transport.
        wait_until(lambda: transport.subscribe.call_count == 2, timeout=5.0, interval=0.02)
        module.odom.subscribe(received.append)
        module.odom.publish(expected)
        results = [peek.result(timeout=5.0) for peek in peeks]

    assert all(result is expected for result in results)
    assert received == [expected]


@pytest.mark.parametrize(
    "stream_name",
    [
        "odom",
        pytest.param(
            "incoming", marks=pytest.mark.skip(reason="flaky in CI, disabled until fixed")
        ),
        "duplex",
    ],
)
def test_peek_timeout_releases_transport_subscription(module, mocker, stream_name):
    transport = LCMTransport("/peek/idle", PoseStamped)
    unsubscribe = mocker.Mock()
    mocker.patch.object(transport, "subscribe", return_value=unsubscribe)
    module.set_transport(stream_name, transport)

    assert module.peek_stream(stream_name, timeout=0.05) is None
    unsubscribe.assert_called_once_with()


def test_peek_unknown_stream(module):
    assert isinstance(module.peek_stream("missing", timeout=0.05), PeekNotFound)
