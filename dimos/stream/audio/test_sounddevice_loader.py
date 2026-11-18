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

import pytest

# Stands in for sounddevice on a machine where PortAudio cannot start.
_FAILING_SOUNDDEVICE = """
class PortAudioError(Exception):
    pass

class OutputStream:
    pass

raise PortAudioError("PulseAudio_Initialize: Can't connect to server")
"""


@pytest.mark.parametrize(
    "module",
    [
        "dimos.stream.audio.node_output",
        "dimos.robot.unitree.connection",
        "dimos.robot.unitree.unitree_skill_container",
    ],
)
def test_import_survives_portaudio_not_starting(module: str, tmp_path: Path) -> None:
    (tmp_path / "sounddevice.py").write_text(_FAILING_SOUNDDEVICE)
    env = os.environ | {"PYTHONPATH": str(tmp_path)}
    code = f"import {module}, sounddevice; sounddevice.OutputStream"

    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=120, env=env
    )

    assert result.returncode == 0, result.stderr
