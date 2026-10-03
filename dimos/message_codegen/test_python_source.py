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

"""Malformed wire data is rejected before constructing generated Python values."""

import os
from pathlib import Path
import struct
import subprocess
import sys

import pytest

generated = pytest.importorskip("dimos_generated", reason="Generate the source example first")
if not hasattr(generated, "demo_msgs"):
    pytest.skip("Generate the source example first", allow_module_level=True)
Telemetry = generated.demo_msgs.msg.Telemetry
Bool = generated.std_msgs.msg.Bool
String = generated.std_msgs.msg.String
UInt32MultiArray = generated.std_msgs.msg.UInt32MultiArray


@pytest.mark.parametrize("little", [True, False])
def test_every_truncation_of_nested_message_is_rejected(little):
    message = Telemetry(sequence=42, label="café", payload=[0, 255], hops=[-1, 2])
    encoded = message.encode(little_endian=little)
    for end in range(len(encoded)):
        with pytest.raises(ValueError):
            Telemetry.decode(encoded[:end])
    with pytest.raises(ValueError, match="Trailing"):
        Telemetry.decode(encoded + b"\0")
    assert Telemetry.decode(encoded).encode(little_endian=little) == encoded


@pytest.mark.parametrize("little", [True, False])
def test_string_requires_positive_length_terminator_and_valid_utf8(little):
    endian = "<" if little else ">"
    header = bytes([0, int(little), 0, 0])
    for length, payload in [(0, b""), (2, b"ab"), (2, b"\xff\0"), (0xFFFFFFFF, b"\0")]:
        with pytest.raises(ValueError):
            String.decode(header + struct.pack(endian + "I", length) + payload)
    value = String(data="café")
    assert String.decode(value.encode(little_endian=little)) == value


def test_invalid_encapsulation_bool_and_huge_sequence_are_rejected():
    with pytest.raises(ValueError, match="encapsulation"):
        Bool.decode(b"\0\3\0\0\1")
    with pytest.raises(ValueError, match="bool"):
        Bool.decode(b"\0\1\0\0\2")
    encoded = bytearray(UInt32MultiArray().encode())
    encoded[-4:] = struct.pack("<I", 0xFFFFFFFF)
    with pytest.raises(ValueError, match="sequence"):
        UInt32MultiArray.decode(bytes(encoded))


def test_decode_preserves_generated_class_type_for_python_consumers(tmp_path):
    consumer = tmp_path / "consumer.py"
    consumer.write_text(
        "from dimos_generated.demo_msgs.msg import Telemetry\n"
        "message: Telemetry = Telemetry.decode(Telemetry(sequence=7).encode())\n"
        "sequence: int = message.sequence\n"
    )
    result = subprocess.run(
        [sys.executable, "-m", "mypy", "--strict", "--follow-imports=silent", str(consumer)],
        capture_output=True,
        text=True,
        env={**os.environ, "MYPYPATH": str(Path(generated.__file__).parent.parent)},
    )
    assert result.returncode == 0, result.stdout + result.stderr
