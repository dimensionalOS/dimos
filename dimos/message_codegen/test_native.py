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

"""Installed native value/catalog acceptance, without a generated demo extension."""

import gc
import pickle

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated_schemas.provider import message_types
from dimos_message_build.registry import decode, encode, initialize
import numpy as np
import pytest

STORE = initialize()
CLASSES = message_types()


def field_value(field):
    kind, spec = field
    if kind.name == "NAME":
        return value_for(spec)
    if kind.name == "BASE":
        name, _ = spec
        if name == "string":
            return "map"
        if name == "bool":
            return True
        return 1.25 if name.startswith("float") else 1
    item, count = spec
    values = [field_value(item) for _ in range(count if kind.name == "ARRAY" else 1)]
    if item[0].name == "BASE" and item[1][0] != "string":
        dtype = {"byte": "uint8", "char": "uint8"}.get(item[1][0], item[1][0])
        return np.array(values, dtype=dtype)
    return values


def value_for(name):
    fields = STORE.fielddefs[name][1]
    return (
        CLASSES[name](**{key: field_value(field) for key, field in fields})
        if fields
        else CLASSES[name](0)
    )


@pytest.mark.parametrize("little_endian", [False, True])
@pytest.mark.parametrize(
    "name",
    [
        pytest.param(
            name,
            marks=pytest.mark.xfail(
                strict=True,
                raises=NotImplementedError,
                reason="CDR-L07 catalog: docs/development/message-limitations.md#cdr-l07",
            ),
        )
        if name == "shape_msgs/msg/SolidPrimitive"
        else name
        for name in sorted(CLASSES)
    ],
)
def test_catalog_native_values_roundtrip_in_both_byte_orders(name, little_endian):
    value = value_for(name)
    wire = encode(value, little_endian=little_endian)
    restored = decode(wire, CLASSES[name])
    assert type(restored) is CLASSES[name]
    assert encode(restored, little_endian=little_endian) == wire


def test_large_image_preserves_header_type_data_and_worker_pickle():
    image = Image(
        header=Header(stamp=Time(sec=17, nanosec=123), frame_id="camera"),
        height=480,
        width=640,
        encoding="rgb8",
        is_bigendian=0,
        step=1920,
        data=np.arange(921600, dtype=np.uint8),
    )
    restored = decode(encode(image), Image)
    assert type(restored.header) is Header
    assert restored.header == image.header
    np.testing.assert_array_equal(restored.data, image.data)
    assert encode(pickle.loads(pickle.dumps(image))) == encode(image)


def test_native_array_copy_is_independent_and_view_survives_message_release():
    image = Image(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        height=1,
        width=3,
        encoding="mono8",
        is_bigendian=0,
        step=3,
        data=np.array([1, 2, 3], dtype=np.uint8),
    )
    view = image.data.view()
    copied = image.data.copy()
    copied[0] = 9
    np.testing.assert_array_equal(image.data, [1, 2, 3])
    assert not np.shares_memory(view, copied)
    del image
    gc.collect()
    np.testing.assert_array_equal(view, [1, 2, 3])
