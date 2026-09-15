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
from typing import Any, ClassVar
import wave

import numpy as np

from dimos.constants import CACHE_DIR
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.stream.audio.tts.spec import SpeechRequest


class KokoroTTSConfig(ModuleConfig):
    model_path: Path = CACHE_DIR / "tts" / "kokoro-v1.0.int8.onnx"
    voices_path: Path = CACHE_DIR / "tts" / "voices-v1.0.bin"
    voice: str = "af_sarah"


class KokoroTTSModule(Module):
    """Generate WAV speech without network access or local speaker playback."""

    config: KokoroTTSConfig
    dedicated_worker: ClassVar[bool] = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._engine: Any = None
        self._synthesis_lock = threading.Lock()
        self._cache: OrderedDict[str, bytes] = OrderedDict()

    @rpc
    def start(self) -> None:
        for path in (self.config.model_path, self.config.voices_path):
            if not path.is_file():
                raise FileNotFoundError(
                    f"Missing TTS asset: {path}. Install the quantized Kokoro assets; "
                    "see docs/usage/webxr-audio.md. Collection never downloads models."
                )
        # Load optional inference dependencies only when this module is started.
        try:
            kokoro = importlib.import_module("kokoro_onnx")
            ort = importlib.import_module("onnxruntime")
        except ImportError as exc:
            raise ImportError(
                "Install offline speech dependencies with uv sync --extra tts"
            ) from exc
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(
            str(self.config.model_path), options, providers=["CPUExecutionProvider"]
        )
        with self._synthesis_lock:
            self._engine = kokoro.Kokoro.from_session(session, str(self.config.voices_path))
            self._cache.clear()
        super().start()

    @rpc
    def synthesize(self, text: str) -> bytes:
        """Synthesize up to 500 characters of English as mono PCM16 WAV."""
        text = SpeechRequest(text=text).text
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

    @rpc
    def stop(self) -> None:
        with self._synthesis_lock:
            self._engine = None
            self._cache.clear()
        super().stop()


def optional_tts() -> tuple[Blueprint, ...]:
    """Include speech only when enabled before collection blueprint resolution."""
    return (KokoroTTSModule.blueprint(),) if global_config.tts else ()
