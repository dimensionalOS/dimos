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

import os
from pathlib import Path
import subprocess
import sys

from dimos.message_codegen.generate import generate


def test_generated_interfaces_check_nested_fields_buffers_and_keyword_arguments(
    tmp_path, monkeypatch
):
    definition = tmp_path / "interfaces/demo_msgs/msg/Reading.msg"
    definition.parent.mkdir(parents=True)
    definition.write_text(
        "std_msgs/Header header\ngeometry_msgs/Point position\nfloat64[] readings\nfloat64[3] axes\nuint8[] payload\n"
    )
    generate(
        [tmp_path / "interfaces"],
        tmp_path / "generated",
        ["demo_msgs/msg/Reading"],
        languages=("python",),
    )
    (tmp_path / "dimos_message_build").symlink_to(Path(__file__).parent, target_is_directory=True)
    monkeypatch.setenv(
        "MYPYPATH", os.pathsep.join([str(tmp_path / "generated/python"), str(tmp_path)])
    )
    source = tmp_path / "consumer.py"
    source.write_text(
        "import numpy as np\n"
        "from dimos_generated.demo_msgs.msg import Reading\n"
        "from dimos_generated.geometry_msgs.msg import Point\n"
        "from dimos_generated.std_msgs.msg import Header\n"
        "from dimos_generated.builtin_interfaces.msg import Time\n"
        "from dimos_message_build.registry import encode, decode\n"
        "value = Reading(header=Header(Time(1, 2), 'map'), position=Point(1.5, 0.0, 0.0), "
        "readings=np.array([1.0, 2.0]), axes=np.zeros(3), payload=np.frombuffer(b'abc', dtype=np.uint8))\n"
        "value.position.x = 32.0\n"
        "value.readings = np.append(value.readings, 3.0)\n"
        "pixels = value.payload.view()\n"
        "result: float = decode(encode(value), Reading).position.x\n"
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
        source.read_text() + "value.position.x = 'hot'\n"
        "value.axes.append(1.0)\n"
        "Reading(unknown_field=1)\n"
    )

    invalid = subprocess.run(command, capture_output=True, text=True, check=False)

    assert invalid.returncode != 0
    assert "[assignment]" in invalid.stdout
    assert "[attr-defined]" in invalid.stdout
    assert "[call-arg]" in invalid.stdout
