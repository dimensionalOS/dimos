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
import subprocess
import sys

from dimos.message_codegen.generate import generate


def test_generated_interfaces_check_nested_fields_buffers_and_keyword_arguments(
    tmp_path, monkeypatch
):
    generate(
        [Path("examples/message-codegen")], tmp_path / "generated", ["demo_msgs/msg/Telemetry"]
    )
    monkeypatch.setenv("MYPYPATH", str(tmp_path / "generated/typing"))
    source = tmp_path / "consumer.py"
    source.write_text(
        "from dimos_generated.demo_msgs.msg import Telemetry\n"
        "from dimos_generated.geometry_msgs.msg import Point\n"
        "value = Telemetry(position=Point(x=1.5), hops=[1, 2], payload=b'abc')\n"
        "value.reading.temperature = 32.0\n"
        "value.hops.append(3)\n"
        "pixels = value.payload.view()\n"
        "result: float = Telemetry.decode(value.encode()).position.x\n"
    )
    command = [
        sys.executable,
        "-m",
        "mypy",
        "--strict",
        "--config-file=/dev/null",
        "--cache-dir",
        str(tmp_path / "cache"),
        str(source),
    ]

    valid = subprocess.run(command, capture_output=True, text=True, check=False)

    assert valid.returncode == 0, valid.stdout + valid.stderr

    source.write_text(
        source.read_text() + "value.reading.temperature = 'hot'\n"
        "value.axes.append(1.0)\n"
        "Telemetry(unknown_field=1)\n"
    )

    invalid = subprocess.run(command, capture_output=True, text=True, check=False)

    assert invalid.returncode != 0
    assert "[assignment]" in invalid.stdout
    assert "[attr-defined]" in invalid.stdout
    assert "[call-arg]" in invalid.stdout
