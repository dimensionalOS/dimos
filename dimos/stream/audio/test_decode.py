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

"""Failure behavior and runtime requirements for browser-audio decoding."""

import pytest

from dimos.stream.audio import decode as decode_module
from dimos.stream.audio.decode import decode_audio_bytes, ffmpeg_requirement


def test_garbage_returns_none() -> None:
    assert decode_audio_bytes(b"definitely not an audio container") is None


def test_ffmpeg_requirement_reports_missing_executable(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(decode_module.shutil, "which", lambda _name: None)
    assert ffmpeg_requirement() == (
        "ffmpeg is required for browser audio; install the ffmpeg system package"
    )
