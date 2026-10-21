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

"""Codec contracts and channel metadata, without network access."""

import pickle

import pytest

from dimos.messages.geometry_msgs.msg import Vector3 as CDRVector3
from dimos.messages.std_msgs.msg import Int32, String
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.codec import describe_message
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic, _key_expr_to_topic


@pytest.mark.parametrize(
    "payload", [b"", b"bad!", b"\x00\x01\x00\x00", b"\x00\x03\x00\x00" + b"\x00" * 4]
)
def test_cdr_rejects_invalid_payload_without_fallback(payload):
    with pytest.raises(ValueError):
        describe_message(Int32).codec.decode(payload)


@pytest.mark.parametrize(
    "declared,message",
    [(Int32, String("wrong")), (Vector3, CDRVector3(1, 2, 3)), (CDRVector3, Vector3(1, 2, 3))],
)
def test_codec_rejects_wrong_declared_type(declared, message):
    with pytest.raises(TypeError, match="declared type"):
        describe_message(declared).codec.encode(message)


def test_unsupported_and_ambiguous_types_fail_at_binding(monkeypatch):
    with pytest.raises(TypeError, match="Unsupported"):
        Topic("test", message_type=dict)
    monkeypatch.setattr(Int32, "lcm_encode", lambda self: b"", raising=False)
    monkeypatch.setattr(Int32, "lcm_decode", lambda data: None, raising=False)
    with pytest.raises(TypeError, match="Ambiguous"):
        Topic("test", message_type=Int32)


@pytest.mark.parametrize("topic_type", [Topic, ZenohTopic])
def test_generic_and_legacy_constructor_names_are_compatible(topic_type):
    topic = topic_type("test", message_type=Int32)
    assert topic == topic_type("test", Int32)
    assert pickle.loads(pickle.dumps(topic)) == topic
    with pytest.raises(TypeError, match="Conflicting"):
        topic_type("test", Vector3, message_type=Int32)


def test_legacy_channel_names_and_plain_cdr_topics():
    assert str(Topic("test", Vector3)) == "test#geometry_msgs.Vector3"
    assert ZenohTopic("test", Vector3).key_expr == "test/geometry_msgs.Vector3"
    cdr = Topic("test", message_type=Int32)
    assert str(cdr) == "test"
    assert Topic.from_channel_str(str(cdr), Int32) == cdr
    zenoh = ZenohTopic("test", message_type=Int32)
    assert zenoh.key_expr == "test"
    assert _key_expr_to_topic(zenoh.key_expr, Int32) == zenoh
