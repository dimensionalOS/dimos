# Copyright 2025-2026 Dimensional Inc.
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

"""Generated CDR transport encoding, lifecycle, and deferred backend selection."""

from contextlib import ExitStack
import pickle
import threading

from dimos_cdrgen.registry import encode
import pytest

import dimos.core.cdr_transport as transport_module
from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import global_config
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Float32, Int32
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, ZenohQoS


@pytest.fixture(params=["lcm", "zenoh"])
def raw_transport(request, mocker):
    name = "LCMPubSubBase" if request.param == "lcm" else "ZenohPubSubBase"
    factory = mocker.patch.object(transport_module, name, autospec=True)
    transport = CdrTransport("/example", Int32, backend=request.param)
    yield transport, factory
    transport.stop()


def test_raw_payload_and_canonical_channel(raw_transport):
    transport, factory = raw_transport
    message = Int32(42)
    factory.assert_not_called()
    transport.publish(message)
    factory.return_value.publish.assert_called_once_with(transport.topic, encode(message))
    assert transport.channel == "example/cdr/std_msgs/msg/Int32"
    transport.start()
    factory.return_value.start.assert_called_once_with()
    transport.stop()
    factory.return_value.stop.assert_called_once_with()
    transport.stop()
    factory.return_value.stop.assert_called_once_with()


def test_subscriber_delivers_owned_value_skips_malformed_and_disposes(raw_transport, mocker):
    transport, factory = raw_transport
    callback = mocker.Mock()
    dispose = transport.subscribe(callback)
    receive = factory.return_value.subscribe.call_args.args[1]
    receive(encode(Int32(17)), transport.topic)
    callback.assert_called_once_with(Int32(17))
    assert type(callback.call_args.args[0]) is Int32
    receive(b"invalid", transport.topic)
    callback.assert_called_once_with(Int32(17))
    dispose()
    factory.return_value.subscribe.return_value.assert_called_once_with()


def test_wrong_message_type_is_rejected_before_start(raw_transport):
    transport, factory = raw_transport
    with pytest.raises(TypeError, match="declared type"):
        transport.publish(Float32(1.0))
    factory.assert_not_called()


def test_failed_start_cleans_up_and_can_retry(raw_transport):
    transport, factory = raw_transport
    factory.return_value.start.side_effect = RuntimeError("unavailable")
    with pytest.raises(RuntimeError, match="unavailable"):
        transport.start()
    factory.return_value.stop.assert_called_once_with()
    factory.return_value.start.side_effect = None
    transport.start()
    assert factory.return_value.start.call_count == 2


def test_pickling_preserves_declared_type_session_and_qos():
    qos = ZenohQoS(reliability="reliable", congestion_control="block")
    value = CdrTransport("/example", Int32, backend="zenoh", qos=qos, scouting=False)
    restored = pickle.loads(pickle.dumps(value))
    assert restored.channel == value.channel
    assert restored.publish_qos == {"reliability": "reliable", "congestion_control": "block"}
    assert restored._config.scouting is False


def test_unregistered_type_is_not_accepted_as_a_message():
    with pytest.raises(TypeError, match="registry-owned"):
        CdrTransport("example", bytes, backend="zenoh")


def test_lcm_does_not_silently_ignore_zenoh_qos():
    with pytest.raises(ValueError, match="only to Zenoh"):
        CdrTransport("example", Int32, backend="lcm", qos=ZenohQoS(reliability="reliable"))


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_actual_raw_transport_delivers_cdr_without_external_network(backend):
    received = []
    ready = threading.Event()

    def receive(value):
        received.append(value)
        ready.set()

    config = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    transport = CdrTransport("example", Int32, backend=backend, **config)
    with ExitStack() as resources:
        resources.callback(transport.stop)
        resources.callback(transport.subscribe(receive))
        transport.publish(Int32(29))
        assert ready.wait(5), "local subscriber did not receive the CDR payload"
        assert received == [Int32(29)]
        assert type(received[0]) is Int32


def test_deferred_cdr_spec_resolves_backend_namespace_and_qos_at_build(monkeypatch):
    monkeypatch.setattr(global_config, "transport", "lcm")
    spec = CdrTransport.spec("camera", Image)
    fixed = CdrTransport.spec("literal", Int32, backend="lcm")
    monkeypatch.setattr(global_config, "transport", "zenoh")
    zenoh = spec.build()
    assert zenoh.channel == "dimos/camera/cdr/sensor_msgs/msg/Image"
    assert zenoh.publish_qos == QOS_LATEST_WINS.to_wire()
    assert fixed.build().channel == "literal/cdr/std_msgs/msg/Int32"
    monkeypatch.setattr(global_config, "transport", "lcm")
    lcm = spec.build()
    assert lcm.channel == "camera/cdr/sensor_msgs/msg/Image"
    assert lcm.publish_qos is None


@pytest.mark.parametrize("topic", ["", "/", "camera/*", "camera?", "camera#"])
def test_default_backend_rejects_invalid_logical_topic(topic, monkeypatch):
    monkeypatch.setattr(global_config, "transport", "zenoh")
    with pytest.raises(ValueError, match="concrete, nonempty"):
        CdrTransport(topic, Int32)
