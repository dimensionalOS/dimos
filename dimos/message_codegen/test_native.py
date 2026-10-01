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

"""Run against the generated example extension; see its README build commands."""

import gc
import pickle
import weakref

import numpy as np
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

generated = pytest.importorskip("dimos_generated", reason="Build examples/message-codegen first")
if not hasattr(generated, "demo_msgs"):
    pytest.skip(
        "Build examples/message-codegen and add its extension to PYTHONPATH",
        allow_module_level=True,
    )
Telemetry = generated.demo_msgs.msg.Telemetry
ImageEnvelope = generated.demo_msgs.msg.ImageEnvelope
Image = generated.sensor_msgs.msg.Image


def test_all_generated_types_match_independent_codec():
    reference = get_typestore(Stores.ROS2_JAZZY)
    message_types = [
        value
        for package in vars(generated).values()
        if hasattr(package, "msg")
        for value in vars(package.msg).values()
        if isinstance(value, type) and hasattr(value, "msg_name")
    ]
    assert len(message_types) >= 142
    for message_type in message_types:
        reference.register(get_types_from_msg(message_type.schema, message_type.msg_name))
        for little_endian in (True, False):
            encoded = message_type().encode(little_endian=little_endian)
            decoded = reference.deserialize_cdr(encoded, message_type.msg_name)
            canonical = bytes(
                reference.serialize_cdr(decoded, message_type.msg_name, little_endian=little_endian)
            )
            assert encoded == canonical, message_type.msg_name
            assert message_type.decode(canonical).encode(little_endian=little_endian) == encoded


def test_native_defaults_match_definition():
    value = Telemetry()

    assert list(value.hops) == []
    assert list(value.axes) == [1.0, 2.0, 3.0]
    assert value.reading.temperature == 21.5
    assert value.reading.active is True
    assert value.label == "start"


def test_sequence_and_nested_edits_survive_serialization():
    value = Telemetry()
    value.hops.append(4)
    value.hops.extend([5, 6])
    value.axes[1] = -2.0
    value.reading.temperature = 37.5

    decoded = Telemetry.decode(value.encode())

    assert list(decoded.hops) == [4, 5, 6]
    assert list(decoded.axes) == [1.0, -2.0, 3.0]
    assert decoded.reading.temperature == 37.5


def test_value_equality_compares_nested_fields_and_arrays():
    first = Telemetry(hops=[1, 2], payload=b"pixels")
    second = Telemetry.decode(first.encode())

    assert first == second
    second.reading.temperature += 1
    assert first != second
    second.reading.temperature = first.reading.temperature
    second.hops[1] = 3
    assert first != second
    assert first != Image()


def test_numpy_views_are_readonly_and_share_native_storage():
    value = Image(data=np.arange(16, dtype=np.uint8))
    first = value.data.view()
    second = value.data.view()

    assert np.shares_memory(first, second)
    assert not first.flags.writeable
    np.testing.assert_array_equal(first, np.arange(16, dtype=np.uint8))
    with pytest.raises(ValueError, match="read-only"):
        first[0] = 7


def test_mutable_copy_is_independent():
    value = Image(data=[1, 2, 3])
    view = value.data.view()
    copied = value.data.copy()
    copied[0] = 9

    assert copied.flags.writeable
    assert not np.shares_memory(view, copied)
    assert list(value.data) == [1, 2, 3]


def test_view_retains_owner_until_released():
    value = Image(data=[1, 2, 3])
    owner = weakref.ref(value)
    view = value.data.view()
    del value
    gc.collect()

    assert owner() is not None
    np.testing.assert_array_equal(view, [1, 2, 3])
    del view
    gc.collect()
    assert owner() is None


def test_all_views_must_be_released_before_resize():
    value = Image(data=[1, 2, 3])
    first = value.data.view()
    second = value.data.view()

    with pytest.raises(BufferError, match="borrowed"):
        value.data.append(4)
    with pytest.raises(BufferError, match="borrowed"):
        value.data = [9, 8]
    del first
    with pytest.raises(BufferError, match="borrowed"):
        value.data.clear()
    del second
    value.data.append(4)
    assert list(value.data) == [1, 2, 3, 4]


def test_nested_buffer_prevents_parent_replacement():
    parent = ImageEnvelope(image=Image(data=[1, 2, 3]))
    view = parent.image.data.view()

    with pytest.raises(BufferError, match="borrowed"):
        parent.image = Image(data=[9])
    del parent
    gc.collect()
    np.testing.assert_array_equal(view, [1, 2, 3])


def test_fixed_array_view_tracks_element_edits():
    value = Telemetry()
    view = value.axes.view()
    value.axes[0] = 7

    assert view[0] == 7
    with pytest.raises(BufferError, match="borrowed"):
        value.axes = [4, 5, 6]


def test_bounds_are_checked_on_encoding():
    value = Telemetry()
    value.hops.extend(range(9))

    with pytest.raises(ValueError, match="sequence bound"):
        value.encode()
    value.hops.clear()
    value.label = "x" * 33
    with pytest.raises(ValueError, match="string bound"):
        value.encode()


def test_fixed_array_rejects_wrong_length():
    with pytest.raises((TypeError, ValueError, RuntimeError)):
        Telemetry(axes=[1, 2])


def test_pickle_preserves_values_for_worker_ipc():
    value = Telemetry(sequence=99, hops=[1, 2], payload=[0, 255])
    value.header.frame_id = "map"

    restored = pickle.loads(pickle.dumps(value))

    assert restored.encode() == value.encode()
    restored.hops.append(3)
    assert list(value.hops) == [1, 2]


def test_standard_string_message_round_trips():
    value = generated.std_msgs.msg.String(data="hello")

    assert value.decode(value.encode()).data == "hello"
