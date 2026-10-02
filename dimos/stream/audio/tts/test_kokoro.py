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
import subprocess
import sys
import wave

import numpy as np
import pytest

from dimos.stream.audio.tts import kokoro
from dimos.stream.audio.tts.kokoro import KokoroTTS, KokoroTTSConfig
from dimos.utils import cache


@pytest.fixture(autouse=True)
def cache_state(tmp_path, monkeypatch):
    monkeypatch.setattr(cache, "_CACHE_LOCK_DIR", tmp_path / "state/cache-users")
    monkeypatch.setattr(cache, "_CACHE_GATE_PATH", tmp_path / "state/cache-clean.lock")


@pytest.fixture
def speech():
    speech = KokoroTTS(KokoroTTSConfig(enabled=True))
    try:
        yield speech
    finally:
        speech.close()


@pytest.fixture
def dependencies(mocker, tmp_path):
    library, torch = mocker.MagicMock(), mocker.MagicMock()
    mocker.patch.object(kokoro, "_load_dependencies", return_value=(library, torch))
    mocker.patch.object(
        kokoro, "download_hf_asset", side_effect=lambda **kw: tmp_path / kw["filename"]
    )
    return library, torch


@pytest.fixture
def engine(speech, dependencies, mocker):
    library, _ = dependencies
    audio = mocker.MagicMock()
    audio.detach.return_value.cpu.return_value.numpy.return_value = np.array(
        [-2.0, -0.5, 0.0, 0.5, 2.0]
    )
    engine = library.KPipeline.return_value
    engine.return_value = [("text", "phonemes", audio)]
    speech.prepare()
    return engine


def test_speech_combines_segments_into_pcm16_wav_and_caches_text(speech, engine):
    engine.return_value *= 2
    audio = speech.synthesize("Recording started")
    assert speech.synthesize(" Recording started ") == audio
    with wave.open(io.BytesIO(audio)) as wav:
        assert (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) == (1, 2, 24000)
        assert (
            np.frombuffer(wav.readframes(10), dtype="<i2").tolist()
            == [
                -32767,
                -16383,
                0,
                16383,
                32767,
            ]
            * 2
        )
    engine.assert_called_once()


def test_tokenizer_installer_exit_becomes_startup_error(speech, dependencies):
    library, _ = dependencies
    library.KPipeline.side_effect = SystemExit(1)
    with pytest.raises(RuntimeError, match="TTS tokenizer installation failed"):
        speech.prepare()
    with pytest.raises(RuntimeError, match="not running"):
        speech.synthesize("Recording started")
    with cache.cache_cleanup_guard():
        pass


def test_empty_synthesis_is_not_cached(speech, engine):
    engine.side_effect = [[], engine.return_value]
    with pytest.raises(RuntimeError, match="produced no speech"):
        speech.synthesize("Hello")
    assert speech.synthesize("Hello").startswith(b"RIFF")


def test_disabled_tts_requires_no_optional_imports_or_downloads():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import importlib.abc
import sys
class BlockOptionalImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'kokoro', 'torch', 'huggingface_hub', 'spacy', 'en_core_web_sm', 'sounddevice'}:
            raise AssertionError(f'Unexpected optional dependency: {fullname}')
sys.meta_path.insert(0, BlockOptionalImports())
from dimos.stream.audio.tts.kokoro import KokoroTTS, KokoroTTSConfig
from dimos.imitation.collection.prompts import CollectionSpeech
from dimos.stream.audio.wav_player import WavPlayer
from dimos.utils import cache
speech = KokoroTTS(KokoroTTSConfig())
speech.prepare()
speech.close()
CollectionSpeech(KokoroTTSConfig()).prepare()
""",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr


def test_preparation_holds_cache_guard_through_model_loading(speech, dependencies, mocker):
    library, _ = dependencies

    def loading(*args, **kwargs):
        with pytest.raises(cache.CacheInUseError), cache.cache_cleanup_guard():
            pass
        raise RuntimeError("load failed")

    mocker.patch.object(library, "KModel", side_effect=loading)
    with pytest.raises(RuntimeError, match="load failed"):
        speech.prepare()
    with cache.cache_cleanup_guard():
        pass
