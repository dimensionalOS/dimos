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

"""Offline CPU speech synthesis with the official Kokoro model and library."""

from collections import OrderedDict
import importlib
import io
import threading
from typing import Annotated, Any
import wave

import numpy as np
from pydantic import BaseModel, StringConstraints

from dimos.utils.assets import download_hf_asset
from dimos.utils.cache import cache_usage_guard

MODEL_REPO = "hexgrad/Kokoro-82M"
MODEL_REVISION = "f3ff3571791e39611d31c381e3a41a3af07b4987"


def _load_dependencies() -> tuple[Any, Any]:
    # Optional inference imports stay behind enabled helper preparation.
    try:
        kokoro = importlib.import_module("kokoro")
        torch = importlib.import_module("torch")
    except ImportError as exc:
        raise ImportError("Install offline speech dependencies with uv sync --extra tts") from exc
    return kokoro, torch


class SpeechText(BaseModel):
    text: Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class KokoroTTSConfig(BaseModel):
    enabled: bool = False
    voice: Annotated[str, StringConstraints(pattern=r"^a[fm]_[a-z]+$")] = "af_sarah"


class KokoroTTS:
    """Generate WAV speech locally after preparing cached model assets."""

    def __init__(self, config: KokoroTTSConfig) -> None:
        self.config = config
        self._engine: Any = None
        self._voice: Any = None
        self._synthesis_lock = threading.Lock()
        self._cache: OrderedDict[str, bytes] = OrderedDict()

    def prepare(self) -> None:
        if not self.config.enabled:
            return
        with cache_usage_guard():
            kokoro, torch = _load_dependencies()
            config_path, model_path, voice_path = (
                download_hf_asset(repo_id=MODEL_REPO, revision=MODEL_REVISION, filename=filename)
                for filename in ("config.json", "kokoro-v1_0.pth", f"voices/{self.config.voice}.pt")
            )
            model = (
                kokoro.KModel(repo_id=MODEL_REPO, config=str(config_path), model=str(model_path))
                .to("cpu")
                .eval()
            )
            try:
                pipeline = kokoro.KPipeline(
                    lang_code="a", repo_id=MODEL_REPO, model=model, device="cpu"
                )
            except SystemExit as exc:
                # spaCy's lazy tokenizer installer can exit instead of raising an Exception.
                raise RuntimeError(
                    "TTS tokenizer installation failed. Check the installer output and "
                    "network access, then retry collection."
                ) from exc
            voice = torch.load(str(voice_path), map_location="cpu", weights_only=True)
            with self._synthesis_lock:
                self._engine = pipeline
                self._voice = voice
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
            chunks = [
                audio.detach().cpu().numpy()
                for _, _, audio in self._engine(text, voice=self._voice, speed=1.0)
            ]
            if not chunks:
                raise RuntimeError("Kokoro produced no speech")
            samples = np.concatenate(chunks)
            pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2")
            output = io.BytesIO()
            with wave.open(output, "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(24000)
                wav.writeframes(pcm.tobytes())
            audio = output.getvalue()
            self._cache[text] = audio
            if len(self._cache) > 128:
                self._cache.popitem(last=False)
            return audio

    def close(self) -> None:
        with self._synthesis_lock:
            self._engine = None
            self._voice = None
            self._cache.clear()
