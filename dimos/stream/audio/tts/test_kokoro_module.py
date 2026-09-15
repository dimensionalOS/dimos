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

import io
import wave

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.stream.audio.tts.kokoro_module import KokoroTTSConfig, KokoroTTSModule


@pytest.fixture
def module(tmp_path):
    module = KokoroTTSModule(
        enabled=True, model_path=tmp_path / "model.onnx", voices_path=tmp_path / "voices.bin"
    )
    try:
        yield module
    finally:
        module.stop()


@pytest.fixture
def engine(module, mocker):
    module.config.model_path.touch()
    module.config.voices_path.touch()
    kokoro = mocker.MagicMock()
    ort = mocker.MagicMock()
    mocker.patch(
        "dimos.stream.audio.tts.kokoro_module.importlib.import_module",
        side_effect={"kokoro_onnx": kokoro, "onnxruntime": ort}.__getitem__,
    )
    engine = kokoro.Kokoro.from_session.return_value
    engine.create.return_value = (np.array([-2.0, -0.5, 0.0, 0.5, 2.0]), 24000)
    module.start()
    return engine


def test_speech_is_pcm16_wav_and_repeated_text_is_cached(module, engine):
    audio = module.synthesize("Recording started")
    assert module.synthesize(" Recording started ") == audio

    with wave.open(io.BytesIO(audio)) as wav:
        assert (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) == (1, 2, 24000)
        assert np.frombuffer(wav.readframes(5), dtype="<i2").tolist() == [
            -32767,
            -16383,
            0,
            16383,
            32767,
        ]
    engine.create.assert_called_once_with(
        "Recording started", voice="af_sarah", speed=1.0, lang="en-us"
    )


def test_cache_evicts_least_recently_used_phrase(module, engine):
    for i in range(128):
        module.synthesize(f"Episode {i}")
    module.synthesize("Episode 0")
    module.synthesize("Episode 128")
    module.synthesize("Episode 0")
    assert engine.create.call_count == 129
    module.synthesize("Episode 1")
    assert engine.create.call_count == 130


@pytest.mark.parametrize("text", ["", " \n ", "a" * 501])
def test_invalid_text_is_rejected_before_synthesis(module, engine, text):
    with pytest.raises(ValidationError):
        module.synthesize(text)
    engine.create.assert_not_called()


def test_missing_assets_fail_without_downloading(module, mocker):
    load = mocker.patch("dimos.stream.audio.tts.kokoro_module.importlib.import_module")
    with pytest.raises(FileNotFoundError, match="Collection never downloads models"):
        module.start()
    load.assert_not_called()


def test_missing_optional_dependency_explains_installation(module, mocker):
    module.config.model_path.touch()
    module.config.voices_path.touch()
    mocker.patch(
        "dimos.stream.audio.tts.kokoro_module.importlib.import_module",
        side_effect=ModuleNotFoundError("kokoro_onnx"),
    )
    with pytest.raises(ImportError, match="uv sync --extra tts"):
        module.start()


def test_synthesis_error_does_not_poison_cache(module, engine):
    engine.create.side_effect = [RuntimeError("inference failed"), (np.zeros(5), 24000)]
    with pytest.raises(RuntimeError, match="inference failed"):
        module.synthesize("Episode saved")
    with wave.open(io.BytesIO(module.synthesize("Episode saved"))) as wav:
        assert wav.getnframes() == 5


def test_stopped_module_rejects_cached_speech(module, engine):
    module.synthesize("Recording started")
    module.stop()
    with pytest.raises(RuntimeError, match="not running"):
        module.synthesize("Recording started")


def test_disabled_tts_requires_no_engine_or_assets(module, mocker):
    module.config.enabled = False
    load = mocker.patch("dimos.stream.audio.tts.kokoro_module.importlib.import_module")
    assert KokoroTTSConfig.model_fields["enabled"].default is False
    module.start()
    assert module.is_enabled() is False
    load.assert_not_called()
    with pytest.raises(RuntimeError, match="not running"):
        module.synthesize("Hello")


def test_enabled_tts_reports_module_config(module, engine):
    assert module.is_enabled() is True
