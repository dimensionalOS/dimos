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

import json
import math
from pathlib import Path
import struct
import sys

import numpy as np
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.message_codegen.definitions import Definitions, parse_message


@pytest.fixture(autouse=True)
def restore_reference_types_module():
    # Independent oracle stores own a process-global module. Restore it so they
    # do not invalidate the canonical registry's native class pickle identities.
    previous = sys.modules.get("rosbags.usertypes")
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop("rosbags.usertypes", None)
        else:
            sys.modules["rosbags.usertypes"] = previous


def write_message(root: Path, name: str, text: str) -> Path:
    path = root / f"{name}.msg"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def test_resolves_standard_dependencies_without_ros(tmp_path):
    write_message(tmp_path, "example_msgs/msg/Telemetry", "sensor_msgs/Image image\nstring label\n")

    definitions = Definitions([tmp_path])
    closure = definitions.resolve(["example_msgs/msg/Telemetry"])

    assert [message.name for message in closure] == [
        "builtin_interfaces/msg/Time",
        "std_msgs/msg/Header",
        "sensor_msgs/msg/Image",
        "example_msgs/msg/Telemetry",
    ]
    reflected = get_types_from_msg(
        definitions.schema("example_msgs/msg/Telemetry"), "example_msgs/msg/Telemetry"
    )
    assert set(reflected) == {message.name for message in closure}


def test_preserves_defaults_constants_and_bounds(tmp_path):
    path = write_message(
        tmp_path,
        "example_msgs/msg/Telemetry",
        "uint8 MODE=7\nstring<=12 label 'hello'\nfloat64[3] xyz [1, 2, 3]\nint32[<=4] values [4]\n",
    )

    message = parse_message(path)

    assert message.constants[0].value == 7
    assert message.fields[0].default == "hello"
    assert message.fields[0].type.string_bound == 12
    assert message.fields[1].default == (1.0, 2.0, 3.0)
    assert message.fields[1].type.array_size == 3
    assert not message.fields[1].type.sequence
    assert message.fields[2].type.sequence
    assert message.fields[2].type.array_size == 4


def test_conflicting_definition_reports_both_sources(tmp_path):
    first = write_message(tmp_path / "one", "example_msgs/msg/Value", "int32 value\n")
    second = write_message(tmp_path / "two", "example_msgs/msg/Value", "float64 value\n")

    with pytest.raises(ValueError, match="Conflicting definition") as error:
        Definitions([tmp_path / "one", tmp_path / "two"], bundled=False)

    assert str(first) in str(error.value)
    assert str(second) in str(error.value)


def test_explicit_empty_sequence_default(tmp_path):
    path = write_message(tmp_path, "example_msgs/msg/Value", "int32[<=8] values []\n")

    message = parse_message(path)

    assert message.fields[0].default == ()


@pytest.mark.parametrize("name", ["A", "A0", "A_B2_C"])
def test_constant_name_validation_accepts_ros_names(tmp_path, name):
    path = write_message(tmp_path, "example_msgs/msg/Value", f"uint8 {name}=1\n")

    assert parse_message(path).constants[0].name == name


@pytest.mark.parametrize("name", ["A_", "A__B", "a", "A" + "0" * 10000 + "!"])
def test_constant_name_validation_rejects_invalid_names(tmp_path, name):
    path = write_message(tmp_path, "example_msgs/msg/Value", f"uint8 {name}=1\n")

    with pytest.raises(ValueError):
        parse_message(path)


def test_equivalent_definitions_ignore_comments(tmp_path):
    write_message(tmp_path / "one", "example_msgs/msg/Value", "int32 value # comment\n")
    write_message(tmp_path / "two", "example_msgs/msg/Value", "# another comment\nint32 value\n")

    closure = Definitions([tmp_path / "one", tmp_path / "two"], bundled=False).resolve()

    assert [message.name for message in closure] == ["example_msgs/msg/Value"]


def test_missing_dependency_reports_source(tmp_path):
    source = write_message(tmp_path, "example_msgs/msg/Value", "other_msgs/Absent value\n")

    with pytest.raises(
        ValueError, match="unresolved message dependency other_msgs/msg/Absent"
    ) as error:
        Definitions([tmp_path], bundled=False).resolve()

    assert str(source) in str(error.value)


def test_recursive_message_reports_chain(tmp_path):
    write_message(tmp_path, "example_msgs/msg/First", "Second second\n")
    write_message(tmp_path, "example_msgs/msg/Second", "First first\n")

    with pytest.raises(
        ValueError, match="First -> example_msgs/msg/Second -> example_msgs/msg/First"
    ):
        Definitions([tmp_path], bundled=False).resolve()


def test_invalid_field_reports_source_line(tmp_path):
    source = write_message(
        tmp_path, "example_msgs/msg/Value", "# title\nint32 first\nuint8 BADNAME\n"
    )

    with pytest.raises(ValueError) as error:
        parse_message(source)

    assert f"{source}:3:" in str(error.value)


def test_all_bundled_schemas_resolve():
    closure = Definitions([]).resolve()

    assert "sensor_msgs/msg/PointCloud2" in {message.name for message in closure}
    assert "visualization_msgs/msg/MarkerArray" in {message.name for message in closure}
    assert all(
        dependency in {item.name for item in closure}
        for message in closure
        for dependency in message.dependencies
    )


def test_full_upstream_generator_sources_are_pinned():
    manifest = json.loads((Path(__file__).parent / "native_sources.json").read_text())
    repositories = manifest["repositories"]
    assert repositories["rosidl"] == {
        "url": "https://github.com/ros2/rosidl.git",
        "revision": "85fa592b698b0f665e3120f48fac0d35e2f7d8a4",
    }
    for source in repositories.values():
        assert source["url"].startswith("https://github.com/")
        assert len(source["revision"]) == 40
        assert all(character in "0123456789abcdef" for character in source["revision"])
    assert not (Path(__file__).parent / "_vendor/rosidl").exists()


def test_native_python_empty_type_preserves_wire_sentinel(tmp_path):
    write_message(tmp_path, "example_msgs/msg/Empty", "uint8 MODE=7\n")
    definitions = Definitions([tmp_path])

    store = get_typestore(Stores.EMPTY)
    store.register(
        get_types_from_msg(definitions.schema("example_msgs/msg/Empty"), "example_msgs/msg/Empty")
    )
    cls = store.types["example_msgs/msg/Empty"]
    value = cls(0)
    assert value.MODE == 7
    assert bytes(store.serialize_cdr(value, value.__msgtype__)) == b"\0\1\0\0\0"
    assert (
        type(
            store.deserialize_cdr(store.serialize_cdr(value, value.__msgtype__), value.__msgtype__)
        )
        is cls
    )


def test_library_decoder_preserves_padding_and_nan(tmp_path):
    write_message(tmp_path, "probe_msgs/msg/Value", "uint8 prefix\nfloat32 sample\nbool[] flags\n")
    definitions = Definitions([tmp_path])

    store = get_typestore(Stores.EMPTY)
    store.register(
        get_types_from_msg(definitions.schema("probe_msgs/msg/Value"), "probe_msgs/msg/Value")
    )
    for little in (True, False):
        value = store.types["probe_msgs/msg/Value"](7, 1.0, np.array([False, True]))
        wire = bytearray(store.serialize_cdr(value, value.__msgtype__, little_endian=little))
        wire[5:8] = b"\xa5" * 3
        wire[8:12] = struct.pack("<I" if little else ">I", 0x7F800001)
        decoded = store.deserialize_cdr(bytes(wire), value.__msgtype__)
        assert math.isnan(decoded.sample)
        assert decoded.prefix == 7 and list(decoded.flags) == [False, True]


@pytest.mark.parametrize("little", [True, False])
@pytest.mark.xfail(
    strict=True,
    raises=pytest.fail.Exception,
    reason="CDR-L06 bool arrays: docs/development/message-limitations.md#cdr-l06",
)
def test_library_decoder_rejects_noncanonical_bool_array(tmp_path, little):
    write_message(tmp_path, "probe_msgs/msg/Value", "uint8 prefix\nfloat32 sample\nbool[] flags\n")
    definitions = Definitions([tmp_path])
    store = get_typestore(Stores.EMPTY)
    store.register(
        get_types_from_msg(definitions.schema("probe_msgs/msg/Value"), "probe_msgs/msg/Value")
    )
    value = store.types["probe_msgs/msg/Value"](7, 1.0, np.array([False, True]))
    wire = bytearray(store.serialize_cdr(value, value.__msgtype__, little_endian=little))
    wire[-1] = 2
    with pytest.raises(ValueError, match="bool"):
        store.deserialize_cdr(bytes(wire), value.__msgtype__)


def test_full_upstream_serialization_and_fastcdr_sources_are_pinned():
    manifest = json.loads((Path(__file__).parent / "native_sources.json").read_text())
    assert manifest["repositories"]["rosidl_typesupport_fastrtps"] == {
        "url": "https://github.com/ros2/rosidl_typesupport_fastrtps.git",
        "revision": "b883555055cce17982a22172a42ef557ef40bac5",
    }
    assert manifest["fastcdr"] == {
        "url": "https://github.com/eProsima/Fast-CDR/archive/refs/tags/v2.4.0.tar.gz",
        "sha256": "79d8466107dd6b7d1defe961c4aa31735038937cf9dd1175cf6b0da0df2209ab",
        "version": "2.4.0",
    }


def test_nix_archive_pins_cover_the_shared_upstream_revisions():
    root = Path(__file__).resolve().parents[2]
    sources = json.loads((Path(__file__).parent / "native_sources.json").read_text())
    archives = json.loads((root / "native/cpp/source-archives.json").read_text())
    assert archives.keys() == sources["repositories"].keys()
    for name, archive in archives.items():
        assert archive["revision"] == sources["repositories"][name]["revision"]
        assert len(archive["sha256"]) == 64
        assert all(character in "0123456789abcdef" for character in archive["sha256"])
