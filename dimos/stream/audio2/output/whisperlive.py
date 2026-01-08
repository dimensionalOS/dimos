#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""WhisperLive speech-to-text output using streaming transcription."""

import threading
import time
from typing import Optional, Callable

import numpy as np
from pydantic import BaseModel, Field
from reactivex import Observable
from reactivex.abc import ObserverBase, DisposableBase
from reactivex.subject import Subject
from whisper_live.client import TranscriptionClient

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent, CompressedAudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.output.whisperlive")


class TranscriptionEvent(BaseModel):
    """Transcription result from WhisperLive."""

    text: str = Field(description="Transcribed text")
    is_final: bool = Field(default=False, description="Whether this is a final transcription")
    timestamp: float = Field(description="Timestamp of the transcription")
    language: Optional[str] = Field(default=None, description="Detected language")


class WhisperLiveConfig(BaseModel):
    """Configuration for WhisperLive STT output."""

    host: str = Field(default="localhost", description="WhisperLive server host")
    port: int = Field(default=9090, description="WhisperLive server port")
    lang: str = Field(default="en", description="Language for transcription")
    translate: bool = Field(default=False, description="Translate to English")
    model: str = Field(default="small", description="Whisper model size")
    use_vad: bool = Field(default=True, description="Use voice activity detection")
    max_clients: int = Field(default=4, description="Maximum concurrent clients")
    max_connection_time: int = Field(default=600, description="Max connection time in seconds")
    sample_rate: int = Field(default=16000, description="Required sample rate for Whisper")
    channels: int = Field(default=1, description="Required channels (mono)")


class WhisperLiveOutputNode:
    """Speech-to-text output using WhisperLive streaming server.

    This node sends audio to a WhisperLive server and receives transcriptions.
    It acts as both a sink (for audio) and a source (for transcriptions).

    WhisperLive expects 16kHz mono audio, so the audio2 pipeline should
    include resampling if needed.
    """

    def __init__(
        self,
        config: WhisperLiveConfig,
        on_transcription: Optional[Callable[[TranscriptionEvent], None]] = None,
    ):
        """Initialize WhisperLive output node.

        Args:
            config: WhisperLive configuration
            on_transcription: Optional callback for transcription events
        """
        self.config = config
        self._on_transcription = on_transcription

        # WhisperLive client
        self._client: Optional[TranscriptionClient] = None

        # Connection state
        self._is_connected = False
        self._lock = threading.Lock()

        # Audio buffer for resampling/conversion
        self._audio_buffer = []

        # Subject for reactive transcription stream
        self._transcription_subject = Subject()

    def _get_sink_name(self) -> str:
        """Get descriptive name for this sink."""
        return f"WhisperLiveSTT[{self.config.host}:{self.config.port}]"

    def _connect(self):
        """Connect to WhisperLive server."""
        if self._is_connected:
            return

        try:
            logger.info(f"{self._get_sink_name()}: Connecting to server...")

            # Create client with callback for transcriptions
            self._client = TranscriptionClient(
                host=self.config.host,
                port=self.config.port,
                lang=self.config.lang,
                translate=self.config.translate,
                model=self.config.model,
                use_vad=self.config.use_vad,
                max_clients=self.config.max_clients,
                max_connection_time=self.config.max_connection_time,
                transcription_callback=self._handle_transcription,
            )

            self._is_connected = True
            logger.info(f"{self._get_sink_name()}: Connected successfully")

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Failed to connect: {e}")
            raise

    def _disconnect(self):
        """Disconnect from WhisperLive server."""
        if not self._is_connected:
            return

        try:
            logger.info(f"{self._get_sink_name()}: Disconnecting...")

            if self._client:
                # Close client connection
                try:
                    if hasattr(self._client.client, "close_websocket"):
                        self._client.client.close_websocket()
                except Exception as e:
                    # Connection may already be closed
                    logger.debug(
                        f"{self._get_sink_name()}: WebSocket close error (may be already closed): {e}"
                    )

            self._is_connected = False
            self._client = None

            # Complete the transcription stream now that we're disconnected
            self._transcription_subject.on_completed()

            logger.info(f"{self._get_sink_name()}: Disconnected")

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Error during disconnect: {e}")

    def _handle_transcription(self, *args):
        """Handle transcription result from WhisperLive.

        Args:
            *args: WhisperLive passes (client_uid, result) or just (result,)
        """
        # WhisperLive passes (client_uid, result) to the callback
        if len(args) == 2:
            client_uid, result = args
        elif len(args) == 1:
            result = args[0]
        else:
            logger.error(f"{self._get_sink_name()}: Unexpected callback args: {args}")
            return
        try:
            # Parse WhisperLive result format
            # Result can be a dict or a list of segment dicts
            if isinstance(result, list):
                # List of segments - process each one
                for segment in result:
                    self._process_segment(segment)
                return
            elif isinstance(result, dict):
                self._process_segment(result)
                return

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Error handling transcription: {e}")

    def _process_segment(self, segment: dict):
        """Process a single transcription segment.

        Args:
            segment: Transcription segment dict
        """
        try:
            logger.debug(f"{self._get_sink_name()}: Processing segment: {segment}")

            text = segment.get("text", "")

            # WhisperLive uses "completed" field for finality
            is_final = segment.get("completed", False)

            # WhisperLive provides start timestamp as string
            start_str = segment.get("start", "0.0")
            timestamp = float(start_str) if isinstance(start_str, str) else start_str

            language = segment.get("language")

            if text.strip():
                event = TranscriptionEvent(
                    text=text,
                    is_final=is_final,
                    timestamp=timestamp,
                    language=language,
                )

                logger.info(f"{self._get_sink_name()}: Transcription: '{text}' (final={is_final})")

                # Emit to reactive stream
                self._transcription_subject.on_next(event)

                # Call user callback if provided (for backward compatibility)
                if self._on_transcription:
                    self._on_transcription(event)

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Error handling transcription: {e}")

    def _convert_audio_event(self, event: AudioEvent) -> np.ndarray:
        """Convert AudioEvent to format required by WhisperLive.

        WhisperLive expects 16kHz mono float32 audio.

        Args:
            event: Audio event to convert

        Returns:
            Audio data as float32 numpy array
        """
        if isinstance(event, RawAudioEvent):
            data = event.data

            # Ensure mono
            if event.channels > 1:
                # Average channels to mono
                data = np.mean(data.reshape(-1, event.channels), axis=1)

            # Convert to float32 if needed
            if data.dtype != np.float32:
                data = data.astype(np.float32)

            # Normalize to [-1, 1] range if needed
            if data.max() > 1.0 or data.min() < -1.0:
                data = data / np.max(np.abs(data))

            return data

        elif isinstance(event, CompressedAudioEvent):
            logger.warning(
                f"{self._get_sink_name()}: Compressed audio not supported, "
                "pipeline should output raw audio"
            )
            return np.array([], dtype=np.float32)

        return np.array([], dtype=np.float32)

    def on_next(self, event: AudioEvent):
        """Handle incoming audio event."""
        if not self._is_connected:
            self._connect()

        if not self._client:
            return

        try:
            # Convert audio to required format
            audio_data = self._convert_audio_event(event)

            if len(audio_data) == 0:
                return

            # Send audio to WhisperLive via internal client's websocket
            # The Client.send_packet_to_server expects bytes
            self._client.client.send_packet_to_server(audio_data.tobytes())

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Error processing audio: {e}")

    def on_error(self, error: Exception):
        """Handle stream error."""
        logger.error(f"{self._get_sink_name()}: Stream error: {error}")
        self._transcription_subject.on_error(error)
        self._disconnect()

    def on_completed(self):
        """Handle stream completion."""
        logger.info(f"{self._get_sink_name()}: Stream completed")

        # Send END_OF_AUDIO signal to WhisperLive to get final transcription
        if self._client and self._is_connected:
            try:
                from whisper_live.client import Client

                logger.info(f"{self._get_sink_name()}: Sending END_OF_AUDIO signal")
                self._client.client.send_packet_to_server(Client.END_OF_AUDIO)
            except Exception as e:
                logger.error(f"{self._get_sink_name()}: Error sending END_OF_AUDIO: {e}")

        # Give WhisperLive time to process the END_OF_AUDIO
        # Note: WhisperLive streaming mode sends completed=False for all segments
        time.sleep(0.5)
        self._disconnect()


def whisperlive_stt(
    host: str = "localhost",
    port: int = 9090,
    lang: str = "en",
    translate: bool = False,
    model: str = "small",
    use_vad: bool = True,
    on_transcription: Optional[Callable[[TranscriptionEvent], None]] = None,
    **kwargs,
):
    """Create a WhisperLive speech-to-text output.

    This creates an audio sink that sends audio to a WhisperLive server
    for real-time speech transcription.

    IMPORTANT: WhisperLive requires 16kHz mono audio. Make sure your pipeline
    resamples to 16kHz before this node.

    Args:
        host: WhisperLive server host (default: "localhost")
        port: WhisperLive server port (default: 9090)
        lang: Language code (default: "en")
        translate: Translate to English (default: False)
        model: Whisper model size - "tiny", "base", "small", "medium", "large" (default: "small")
        use_vad: Use voice activity detection (default: True)
        on_transcription: Callback function for transcription results
        **kwargs: Additional arguments passed to WhisperLiveConfig

    Returns:
        An operator function that consumes AudioEvents and produces transcriptions

    Examples:
        # Basic usage with microphone
        def on_text(event: TranscriptionEvent):
            print(f"Transcription: {event.text}")

        microphone(sample_rate=16000).pipe(
            whisperlive_stt(on_transcription=on_text)
        ).run()

        # With file input (requires resampling to 16kHz)
        from dimos.stream.audio2 import file_input

        file_input("speech.wav").pipe(
            # TODO: Add resample operator when available
            whisperlive_stt(model="base", on_transcription=on_text)
        ).run()

        # Remote server
        microphone(sample_rate=16000).pipe(
            whisperlive_stt(
                host="192.168.1.100",
                port=9090,
                model="medium",
                on_transcription=on_text
            )
        ).run()
    """
    from reactivex import create

    config = WhisperLiveConfig(
        host=host,
        port=port,
        lang=lang,
        translate=translate,
        model=model,
        use_vad=use_vad,
        **kwargs,
    )

    node = WhisperLiveOutputNode(config, on_transcription=on_transcription)

    # Return a dual-purpose object that can be used as operator or sink
    class WhisperLiveOperator:
        """Can be used as both an operator and a sink.

        Provides a .transcriptions property for reactive access to transcription events.
        """

        def __init__(self, sink_node):
            self._sink = sink_node

        @property
        def transcriptions(self) -> Observable:
            """Get the observable stream of transcription events.

            Returns:
                Observable that emits TranscriptionEvent objects

            Example:
                stt = whisperlive_stt()
                audio = microphone().pipe(stt, speaker())
                stt.transcriptions.subscribe(lambda t: print(t.text))
                audio.run()
            """
            return self._sink._transcription_subject

        def __call__(self, source):
            """Act as an operator for pipe()."""

            def subscribe(observer, scheduler=None):
                # Wrap sink callbacks
                def on_next_wrapper(value):
                    self._sink.on_next(value)
                    # Pass through audio for chaining
                    observer.on_next(value)

                def on_error_wrapper(error):
                    self._sink.on_error(error)
                    observer.on_error(error)

                def on_completed_wrapper():
                    logger.info("WhisperLiveOperator: on_completed received")
                    self._sink.on_completed()
                    observer.on_completed()

                # Subscribe to source
                return source.subscribe(
                    on_next=on_next_wrapper,
                    on_error=on_error_wrapper,
                    on_completed=on_completed_wrapper,
                    scheduler=scheduler,
                )

            return create(subscribe)

        # Allow using as a sink directly with subscribe()
        def on_next(self, value):
            self._sink.on_next(value)

        def on_error(self, error):
            self._sink.on_error(error)

        def on_completed(self):
            self._sink.on_completed()

    return WhisperLiveOperator(node)
