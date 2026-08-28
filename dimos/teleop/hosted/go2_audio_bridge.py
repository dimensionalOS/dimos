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

"""Operator microphone audio to the Go2 speaker over Unitree WebRTC."""

from __future__ import annotations

import base64
from io import BytesIO
import json
import queue
import threading
import time
from typing import Any, Literal
import wave

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
from unitree_webrtc_connect.constants import AUDIO_API, RTC_TOPIC

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.stream.audio.base import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GET_AUDIO_LIST = AUDIO_API["GET_AUDIO_LIST"]
ENTER_MEGAPHONE = AUDIO_API["ENTER_MEGAPHONE"]
EXIT_MEGAPHONE = AUDIO_API["EXIT_MEGAPHONE"]
UPLOAD_MEGAPHONE = AUDIO_API["UPLOAD_MEGAPHONE"]
TARGET_SAMPLE_RATE = 44100
INT16_MIN = np.iinfo(np.int16).min
INT16_MAX = np.iinfo(np.int16).max
# Base64-character block size the Unitree upload API takes per request.
UPLOAD_CHUNK_CHARS = 4096


class Go2AudioBridgeConfig(ModuleConfig):
    speaker: Literal["auto", "enabled", "disabled"] = "auto"
    batch_ms: int = 100
    idle_timeout_sec: float = 1.0
    queue_frames: int = 100
    chunk_interval_sec: float = 0.05
    megaphone_enter_delay_sec: float = 0.2
    target_peak: int = 12000
    max_gain: float = 128.0
    noise_gate_peak: int = 32


class Go2AudioBridgeModule(Module):
    """Forward hosted operator audio when the Go2 audio route is enabled."""

    config: Go2AudioBridgeConfig
    go2: GO2ConnectionSpec
    operator_audio: In[AudioEvent]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._frames: queue.Queue[AudioEvent | None] = queue.Queue(maxsize=self.config.queue_frames)
        self._worker: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._speaker_available: bool | None = None
        self._megaphone_active = False

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        if self.config.speaker == "disabled":
            self._speaker_available = False
            return
        if self.config.speaker == "enabled":
            self._speaker_available = True
        self.register_disposable(Disposable(self.operator_audio.subscribe(self._on_audio)))
        self._worker = threading.Thread(target=self._run, daemon=True, name="go2-audio-bridge")
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._frames.put_nowait(None)
        except queue.Full:
            pass
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            if self._worker.is_alive():
                logger.error("Go2 audio bridge worker did not stop")
            else:
                self._worker = None
        if self._worker is None:
            self._exit_megaphone()
        super().stop()

    def _on_audio(self, frame: AudioEvent) -> None:
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            try:
                self._frames.get_nowait()
                self._frames.put_nowait(frame)
            except queue.Empty:
                pass

    def _run(self) -> None:
        pending: list[NDArray[np.int16]] = []
        pending_samples = 0
        last_audible_at: float | None = None
        target_samples = max(1, TARGET_SAMPLE_RATE * self.config.batch_ms // 1000)

        while not self._stop_event.is_set():
            try:
                frame = self._frames.get(timeout=self.config.idle_timeout_sec)
            except queue.Empty:
                self._flush(pending)
                pending.clear()
                pending_samples = 0
                self._exit_megaphone()
                last_audible_at = None
                continue
            if frame is None:
                break
            pcm = self._to_mono_target_rate(frame)
            if pcm.size == 0:
                continue
            now = time.monotonic()
            if self._peak(pcm) > self.config.noise_gate_peak:
                last_audible_at = now
            pending.append(pcm)
            pending_samples += pcm.size
            if pending_samples >= target_samples:
                self._flush(pending)
                pending.clear()
                pending_samples = 0
            if (
                self._megaphone_active
                and last_audible_at is not None
                and now - last_audible_at >= self.config.idle_timeout_sec
            ):
                self._flush(pending)
                pending.clear()
                pending_samples = 0
                self._exit_megaphone()
                last_audible_at = now

        if self._stop_event.is_set():
            self._exit_megaphone()
        else:
            self._flush(pending)

    def _ensure_speaker(self) -> bool:
        if self._speaker_available is not None:
            return self._speaker_available
        try:
            self._request(GET_AUDIO_LIST)
        except Exception as exc:
            logger.info("Go2 audio hub unavailable; speaker audio disabled", error=str(exc))
            self._speaker_available = False
        else:
            logger.info("Go2 audio hub detected; operator audio enabled")
            self._speaker_available = True
        return self._speaker_available

    def _flush(self, frames: list[NDArray[np.int16]]) -> None:
        if not frames or not self._ensure_speaker():
            return
        pcm = np.concatenate(frames).astype(np.int16, copy=False)
        pcm = self._normalize_level(pcm)
        if pcm.size == 0:
            return
        try:
            if not self._megaphone_active:
                self._request(ENTER_MEGAPHONE)
                self._megaphone_active = True
                if self._stop_event.is_set():
                    self._exit_megaphone()
                    return
                if self.config.megaphone_enter_delay_sec > 0:
                    if self._stop_event.wait(self.config.megaphone_enter_delay_sec):
                        self._exit_megaphone()
                        return
            wav_data = self._wav_bytes(pcm)
            self._upload_wav(wav_data)
        except Exception:
            logger.warning("Go2 speaker audio send failed", exc_info=True)
            self._exit_megaphone()
            if self.config.speaker == "auto":
                self._speaker_available = False

    def _upload_wav(self, wav_data: bytes) -> None:
        encoded = base64.b64encode(wav_data).decode("ascii")
        chunks = [
            encoded[i : i + UPLOAD_CHUNK_CHARS] for i in range(0, len(encoded), UPLOAD_CHUNK_CHARS)
        ]
        for index, chunk in enumerate(chunks, 1):
            if self._stop_event.is_set():
                return
            self._request(
                UPLOAD_MEGAPHONE,
                {
                    "current_block_size": len(chunk),
                    "block_content": chunk,
                    "current_block_index": index,
                    "total_block_number": len(chunks),
                },
            )
            if index < len(chunks) and self.config.chunk_interval_sec > 0:
                if self._stop_event.wait(self.config.chunk_interval_sec):
                    return

    def _exit_megaphone(self) -> None:
        if not self._megaphone_active:
            return
        try:
            self._request(EXIT_MEGAPHONE)
        except Exception:
            logger.warning("Failed to exit Go2 megaphone mode", exc_info=True)
        else:
            self._megaphone_active = False

    def _request(self, api_id: int, parameter: dict[str, Any] | None = None) -> dict[Any, Any]:
        response = self.go2.publish_request(
            RTC_TOPIC["AUDIO_HUB_REQ"],
            {"api_id": api_id, "parameter": json.dumps(parameter or {})},
        )
        if not response:
            raise RuntimeError(f"Go2 audio request {api_id} returned no response")
        code = self._response_code(response)
        if code not in (None, 0):
            raise RuntimeError(f"Go2 audio request {api_id} failed with code {code}")
        return response

    @staticmethod
    def _response_code(response: Any) -> int | None:
        """Extract Unitree's nested status code when firmware provides one."""
        if not isinstance(response, dict):
            return None
        candidates = [response.get("code")]
        data = response.get("data")
        if isinstance(data, dict):
            candidates.append(data.get("code"))
            header = data.get("header")
            if isinstance(header, dict):
                status = header.get("status")
                if isinstance(status, dict):
                    candidates.append(status.get("code"))
        for value in candidates:
            if isinstance(value, int):
                return value
        return None

    @staticmethod
    def _peak(pcm: NDArray[np.int16]) -> int:
        return int(np.max(np.abs(pcm.astype(np.int32))))

    def _normalize_level(self, pcm: NDArray[np.int16]) -> NDArray[np.int16]:
        """Gate near-silence and raise each segment toward a bounded peak."""
        if pcm.size == 0:
            return pcm
        peak = self._peak(pcm)
        if peak <= self.config.noise_gate_peak:
            return np.empty(0, dtype=np.int16)
        gain = min(self.config.max_gain, self.config.target_peak / peak)
        if gain == 1.0:
            return pcm
        amplified = np.clip(pcm.astype(np.float32) * gain, INT16_MIN, INT16_MAX)
        return amplified.astype(np.int16)

    @staticmethod
    def _to_mono_target_rate(frame: AudioEvent) -> NDArray[np.int16]:
        if frame.sample_rate <= 0 or frame.channels <= 0:
            return np.empty(0, dtype=np.int16)
        pcm = frame.to_int16().data.reshape(-1)
        if frame.channels > 1:
            usable = pcm.size - (pcm.size % frame.channels)
            pcm = pcm[:usable].reshape(-1, frame.channels).astype(np.int32).mean(axis=1)
        if frame.sample_rate != TARGET_SAMPLE_RATE and pcm.size:
            output_size = round(pcm.size * TARGET_SAMPLE_RATE / frame.sample_rate)
            pcm = np.interp(
                np.linspace(0, pcm.size - 1, output_size),
                np.arange(pcm.size),
                pcm,
            )
        return np.asarray(np.clip(pcm, INT16_MIN, INT16_MAX), dtype=np.int16)

    @staticmethod
    def _wav_bytes(pcm: NDArray[np.int16]) -> bytes:
        output = BytesIO()
        with wave.open(output, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(TARGET_SAMPLE_RATE)
            wav.writeframes(pcm.tobytes())
        return output.getvalue()
