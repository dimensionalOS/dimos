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

import cbor2
from dimos_lcm.sim_msgs import CborExtension
import pytest

from dimos.sim2.extensions import validate_cbor_entries


@pytest.fixture
def prop():
    value = cbor2.dumps({"temperature_c": 21.5})
    return CborExtension(
        entity_id="box",
        key="thermal",
        schema="example.thermal.v1",
        value_length=len(value),
        value=value,
    )


def test_optional_payload_keeps_its_schema_and_value(prop):
    validate_cbor_entries([prop], {"box"})
    decoded = CborExtension.lcm_decode(prop.lcm_encode())
    assert decoded.schema == "example.thermal.v1"
    assert cbor2.loads(bytes(decoded.value)) == {"temperature_c": 21.5}


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("entity_id", "missing", "known entity"),
        ("schema", "", "schema"),
        ("value_length", 999, "byte count"),
        ("value", b"\xff", "invalid CBOR"),
        ("value", b"\x01\x02", "exactly one"),
    ],
)
def test_invalid_extension_is_rejected(prop, field, value, error):
    setattr(prop, field, value)
    if field == "value":
        prop.value_length = len(value)
    with pytest.raises(ValueError, match=error):
        validate_cbor_entries([prop], {"box"})


def test_duplicate_property_is_ambiguous(prop):
    with pytest.raises(ValueError, match="duplicate"):
        validate_cbor_entries([prop, prop], {"box"})
