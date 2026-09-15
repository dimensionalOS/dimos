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

"""Offline CPU speech synthesis with locally installed Kokoro INT8 assets."""

from collections import OrderedDict
import importlib
import io
from pathlib import Path
import threading
from typing import Annotated, Any
import wave

import numpy as np
from pydantic import BaseModel, StringConstraints

from dimos.stream.audio.tts.assets import (
    MODEL_FILENAME,
    TTS_CACHE_DIR,
    VOICES_FILENAME,
    ensure_asset,
)


def _load_dependencies() -> tuple[Any, Any]:
    # Optional inference imports stay behind enabled helper preparation.
    try:
        kokoro = importlib.import_module("kokoro_onnx")
        ort = importlib.import_module("onnxruntime")
    except ImportError as exc:
        raise ImportError("Install offline speech dependencies with uv sync --extra tts") from exc
    return kokoro, ort


class SpeechText(BaseModel):
    text: Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class KokoroTTSConfig(BaseModel):
    enabled: bool = False
    model_path: Path = TTS_CACHE_DIR / MODEL_FILENAME
    voices_path: Path = TTS_CACHE_DIR / VOICES_FILENAME
    voice: str = "af_sarah"


class KokoroTTS:
    """Generate WAV speech locally after preparing cached model assets."""

    def __init__(self, config: KokoroTTSConfig) -> None:
        self.config = config
        self._engine: Any = None
        self._synthesis_lock = threading.Lock()
        self._cache: OrderedDict[str, bytes] = OrderedDict()

    def prepare(self) -> None:
        if not self.config.enabled:
            return
        kokoro, ort = _load_dependencies()
        paths = (
            (self.config.model_path, MODEL_FILENAME),
            (self.config.voices_path, VOICES_FILENAME),
        )
        for path, filename in paths:
            if path != TTS_CACHE_DIR / filename and not path.is_file():
                raise FileNotFoundError(f"Missing custom TTS asset: {path}")
        for path, filename in paths:
            if path == TTS_CACHE_DIR / filename:
                ensure_asset(path, filename)

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(
            str(self.config.model_path), options, providers=["CPUExecutionProvider"]
        )
        with self._synthesis_lock:
            self._engine = kokoro.Kokoro.from_session(session, str(self.config.voices_path))
            self._cache.clear()

    def synthesize(self, text: str) -> bytes:
        """Synthesize up to 500 characters of English as mono PCM16 WAV."""
        text = SpeechText(text=text).text
        with self._synthesis_lock:
            if self._engine is None:
                raise RuntimeError("Speech synthesis is not running")
            if text in self._cache:
                self._cache.move_to_end(text)
                return self._cache[text]
            samples, sample_rate = self._engine.create(
                text, voice=self.config.voice, speed=1.0, lang="en-us"
            )
            pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2")
            output = io.BytesIO()
            with wave.open(output, "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(sample_rate)
                wav.writeframes(pcm.tobytes())
            audio = output.getvalue()
            self._cache[text] = audio
            if len(self._cache) > 128:
                self._cache.popitem(last=False)
            return audio

    def close(self) -> None:
        with self._synthesis_lock:
            self._engine = None
            self._cache.clear()
