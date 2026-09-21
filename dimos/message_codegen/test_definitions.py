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

from pathlib import Path

import pytest
from rosbags.typesys import get_types_from_msg

from dimos.message_codegen.definitions import Definitions, parse_message


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
