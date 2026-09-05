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

"""Cockpit push-to-talk to agent text: the chat panel mic's robot side.

`audio_in` carries one utterance per button hold as ordered AudioChunk
slices (dimos/web/relay_bridge/audio_codec.py pins the wire contract). The
reassembled container bytes go through the owned local STT chain - ffmpeg
decode to PCM, AudioNormalizer, WhisperNode (the exact pipeline behind the
legacy WebInput upload) - and the transcript is published on `human_input`,
entering the agent conversation exactly like a typed chat message. A
cancelled hold simply stops sending; its half-built utterance is reaped
after `stale_utterance_s`.
"""

from dataclasses import dataclass, field
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from typing import Any

import reactivex as rx
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.stream.audio.base import AudioEvent
from dimos.stream.audio.decode import decode_audio_bytes, ffmpeg_requirement
from dimos.stream.audio.pipeline import whisper_pipeline
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.audio_codec import AudioChunk

logger = setup_logger()


@dataclass
class _Assembly:
    """One in-flight utterance: ordered slices of its container bytes."""

    expected_seq: int = 0
    total_bytes: int = 0
    last_at: float = 0.0
    parts: list[bytes] = field(default_factory=list)


class VoiceInputConfig(ModuleConfig):
    # A cancelled hold sends no final frame; reap its assembly after this.
    stale_utterance_s: float = 15.0
    max_utterance_bytes: int = 2 * 1024 * 1024
    max_pending_utterances: int = 4


class VoiceInput(Module):
    config: VoiceInputConfig

    audio_in: In[AudioChunk]
    human_input: Out[str]

    _assemblies: dict[str, _Assembly]
    _lock: Lock
    _queue: Queue[bytes]
    _thread: Thread
    _stop_event: Event
    _audio_subject: rx.subject.Subject[AudioEvent] | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._assemblies = {}
        self._lock = Lock()
        self._queue = Queue(maxsize=self.config.max_pending_utterances)
        self._thread = Thread(
            target=self._worker_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._stop_event = Event()
        self._audio_subject = None

    def __reduce__(self) -> Any:
        return (self.__class__, (), {})

    @rpc
    def start(self) -> None:
        requirement_error = ffmpeg_requirement()
        if requirement_error is not None:
            raise RuntimeError(requirement_error)
        super().start()
        self._audio_subject, transcripts = whisper_pipeline()
        self.register_disposable(
            transcripts.subscribe(
                on_next=self._publish_text,
                # Upstream stream failures still terminate the subscription.
                on_error=lambda e: logger.error("voice STT pipeline failed: %s", e),
            )
        )
        # Not handle_audio_in: the auto-bound handler mailbox is latest-wins
        # and would drop chunks; a manual subscription sees every one.
        self.register_disposable(Disposable(self.audio_in.subscribe(self._on_chunk)))
        if not self._thread.is_alive():
            self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_chunk(self, chunk: AudioChunk) -> None:
        """Transport-thread hot path: bookkeeping only, decode and STT queued."""
        now = time.monotonic()
        finished: bytes | None = None
        with self._lock:
            self._reap_stale_locked(now)
            assembly = self._assemblies.get(chunk.sid)
            if assembly is None:
                if chunk.seq != 0:
                    logger.warning("voice utterance %s dropped: no start chunk", chunk.sid)
                    return
                assembly = _Assembly()
                self._assemblies[chunk.sid] = assembly
            if chunk.seq != assembly.expected_seq:
                del self._assemblies[chunk.sid]
                logger.warning(
                    "voice utterance %s dropped: chunk gap (got seq %d, expected %d)",
                    chunk.sid,
                    chunk.seq,
                    assembly.expected_seq,
                )
                return
            assembly.expected_seq += 1
            assembly.last_at = now
            assembly.total_bytes += len(chunk.data)
            if assembly.total_bytes > self.config.max_utterance_bytes:
                del self._assemblies[chunk.sid]
                logger.warning(
                    "voice utterance %s dropped: over %d bytes",
                    chunk.sid,
                    self.config.max_utterance_bytes,
                )
                return
            if chunk.data:
                assembly.parts.append(chunk.data)
            if chunk.final:
                del self._assemblies[chunk.sid]
                data = b"".join(assembly.parts)
                if data:
                    finished = data
        if finished is not None:
            try:
                self._queue.put_nowait(finished)
            except Full:
                logger.warning("voice utterance dropped: transcription queue full")

    def _reap_stale(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._reap_stale_locked(now)

    def _reap_stale_locked(self, now: float) -> None:
        for sid, assembly in list(self._assemblies.items()):
            if now - assembly.last_at > self.config.stale_utterance_s:
                del self._assemblies[sid]
                logger.info("voice utterance %s reaped: cancelled or sender gone", sid)

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw = self._queue.get(timeout=0.5)
            except Empty:
                # Expiry must not depend on new traffic: a cancelled hold may
                # be the last thing a browser ever sends.
                self._reap_stale()
                continue
            try:
                event = decode_audio_bytes(raw)
                if event is not None and self._audio_subject is not None:
                    self._audio_subject.on_next(event)
            except Exception:
                # One bad clip must not end voice input for the process.
                logger.exception("voice transcription failed")

    def _publish_text(self, text: str) -> None:
        text = text.strip()
        if text:
            self.human_input.publish(text)
