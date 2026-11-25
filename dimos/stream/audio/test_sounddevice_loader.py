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

import pytest

_SOUNDDEVICE = """
from unittest.mock import Mock

class PortAudioError(Exception):
    pass

server_running = False
_initialized = 0

def _initialize():
    global _initialized
    if not server_running:
        raise PortAudioError("PulseAudio_Initialize: Can't connect to server")
    _initialized += 1

def query_devices():
    return ["speaker"] if _initialized else []

class OutputStream:
    start = Mock()
    write = Mock()
    stop = Mock()
    close = Mock()

    def __init__(self, **kwargs):
        if not _initialized:
            raise PortAudioError("Error querying device -1")

_initialize()
"""


@pytest.fixture(autouse=True)
def headless_audio(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    (tmp_path / "sounddevice.py").write_text(_SOUNDDEVICE)
    monkeypatch.setenv("PYTHONPATH", str(tmp_path))
    monkeypatch.setenv("DIMOS_LOG_LEVEL", "INFO")


def _run_python(code: str) -> str:
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


@pytest.mark.parametrize(
    "module",
    [
        "dimos.stream.audio.node_output",
        "dimos.robot.unitree.connection",
        "dimos.robot.unitree.unitree_skill_container",
    ],
)
def test_headless_import_is_quiet(module: str) -> None:
    assert _run_python(f"import {module}, sounddevice; sounddevice.OutputStream") == ""


def test_playback_recovers_when_audio_server_starts() -> None:
    _run_python("""
from unittest.mock import patch

import numpy as np
from reactivex.subject import Subject

from dimos.stream.audio.base import AudioEvent
from dimos.stream.audio.node_output import SounddeviceAudioOutput
import sounddevice

events = Subject()
frame = AudioEvent(np.ones((16, 1), dtype=np.float32), sample_rate=16000, timestamp=0)
output = SounddeviceAudioOutput()
try:
    output.consume_audio(events)
    with patch("dimos.stream.audio.node_output.monotonic", return_value=10) as clock:
        events.on_next(frame)
        sounddevice.OutputStream.write.assert_not_called()

        sounddevice.server_running = True
        clock.return_value = 20
        events.on_next(frame)
    sounddevice.OutputStream.write.assert_called_once_with(frame.data)
finally:
    output.stop()
""")


def test_cached_sounddevice_discovers_late_output_device() -> None:
    _run_python("""
import dimos.robot.unitree.connection
import sounddevice
from sounddevice import OutputStream, query_devices

sounddevice.server_running = True
assert query_devices() == ["speaker"]
OutputStream().close()
""")
