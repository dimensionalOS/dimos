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

"""Recording metadata shares owner schemas and can project late Header fields."""

from dimos_cdrgen.definitions import Definitions
from dimos_cdrgen.registry import schema, schema_roots
import pytest

from dimos.experimental.memory.cdr_schema import cdr_name, recording_schema, timestamp_layout
from dimos.messages.geometry_msgs.msg import PoseStamped
from dimos.messages.std_msgs.msg import String


def definitions(tmp_path, text):
    source = tmp_path / "custom_msgs/msg/Sample.msg"
    source.parent.mkdir(parents=True)
    source.write_text(text)
    return Definitions([tmp_path, *schema_roots()])


def test_registry_owner_and_complete_schema_are_reused():
    metadata = recording_schema(PoseStamped)
    assert cdr_name(PoseStamped) == "geometry_msgs/msg/PoseStamped"
    assert metadata["data"] == schema("geometry_msgs/msg/PoseStamped")
    assert "MSG: std_msgs/Header" in metadata["data"]
    assert "MSG: builtin_interfaces/Time" in metadata["data"]
    assert metadata["timestamp_prefix"] == {
        "kind": "struct",
        "fields": [{"kind": "struct", "fields": [{"kind": "stamp"}]}],
    }
    assert recording_schema(String)["timestamp_prefix"] is None
    with pytest.raises(TypeError, match="registry-owned"):
        recording_schema(bytes)


def test_late_header_layout_handles_variable_sequences_and_alignment(tmp_path):
    parsed = definitions(
        tmp_path, "string label\nfloat64[] values\nstd_msgs/Header header\nuint32 tail\n"
    )
    assert timestamp_layout(parsed, "custom_msgs/msg/Sample") == {
        "kind": "struct",
        "fields": [
            {"kind": "string"},
            {"kind": "sequence", "element": {"kind": "atom", "alignment": 8, "bytes": 8}},
            {"kind": "struct", "fields": [{"kind": "stamp"}]},
        ],
    }


def test_headerless_and_repeated_headers_use_reception_time(tmp_path):
    parsed = definitions(tmp_path, "std_msgs/Header[] header\n")
    assert timestamp_layout(parsed, "custom_msgs/msg/Sample") is None


def test_fixed_arrays_remain_distinct_from_bounded_sequences(tmp_path):
    parsed = definitions(tmp_path, "uint16[3] offsets\nuint8[<=4] flags\nstd_msgs/Header header\n")
    result = timestamp_layout(parsed, "custom_msgs/msg/Sample")
    assert result["fields"][0] == {
        "kind": "array",
        "length": 3,
        "element": {"kind": "atom", "alignment": 2, "bytes": 2},
    }
    assert result["fields"][1] == {
        "kind": "sequence",
        "element": {"kind": "atom", "alignment": 1, "bytes": 1},
    }


def test_numeric_ts_fallback_is_projected_after_other_fields(tmp_path):
    parsed = definitions(tmp_path, "uint8 flags\nfloat64 ts\n")
    assert timestamp_layout(parsed, "custom_msgs/msg/Sample") == {
        "kind": "struct",
        "fields": [
            {"kind": "atom", "alignment": 1, "bytes": 1},
            {"kind": "seconds", "scalar": "f64"},
        ],
    }
