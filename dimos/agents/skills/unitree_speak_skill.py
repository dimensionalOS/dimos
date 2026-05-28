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

import base64
import hashlib
import io
import json
import math
import os
import struct
import threading
import time
from typing import Any
import wave

from openai import OpenAI
from unitree_webrtc_connect.constants import RTC_TOPIC

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_AUDIO_API = {
    "GET_AUDIO_LIST": 1001,
    "SELECT_START_PLAY": 1002,
    "PAUSE": 1003,
    "UNSUSPEND": 1004,
    "SET_PLAY_MODE": 1007,
    "UPLOAD_AUDIO_FILE": 2001,
    "ENTER_MEGAPHONE": 4001,
    "EXIT_MEGAPHONE": 4002,
    "UPLOAD_MEGAPHONE": 4003,
}
_PLAY_MODE_NO_CYCLE = "no_cycle"
_OPENAI_TTS_TIMEOUT_S = 60.0


class UnitreeSpeakSkill(Module):
    """Speak through the Unitree Go2 onboard speaker."""

    _connection: GO2ConnectionSpec
    _openai_client: OpenAI | None = None
    _speech_unavailable_reason: str | None = None
    _last_robot_audio_upload: dict[str, Any] | None = None
    _bg_threads: list[threading.Thread] = []
    _bg_threads_lock: threading.Lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        if not os.getenv("OPENAI_API_KEY"):
            self._speech_unavailable_reason = "OPENAI_API_KEY is not set"
            logger.warning("UnitreeSpeakSkill TTS disabled because OPENAI_API_KEY is not set")
            return
        self._openai_client = OpenAI(timeout=_OPENAI_TTS_TIMEOUT_S)

    @rpc
    def stop(self) -> None:
        with self._bg_threads_lock:
            threads = list(self._bg_threads)
        for thread in threads:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @skill
    def speak(self, text: str, blocking: bool = True) -> str:
        """Speak text out loud through the Unitree Go2 onboard speaker.

        Use this to communicate with nearby people from the robot body, not
        from the operator computer.

        Args:
            text: Text to synthesize and play on the robot.
            blocking: When true, wait for upload/play request completion.
        """
        if self._openai_client is None:
            reason = self._speech_unavailable_reason or "TTS not initialized"
            return f"Robot speech unavailable: {reason}"

        if not blocking:
            thread = threading.Thread(
                target=self._speak_bg,
                args=(text,),
                daemon=True,
                name="UnitreeSpeakSkill-bg",
            )
            with self._bg_threads_lock:
                self._bg_threads.append(thread)
            thread.start()
            return f"Speaking on robot (non-blocking): {text}"

        return self._speak_blocking(text)

    @skill
    def speech_status(self) -> str:
        """Report Unitree Go2 onboard speaker readiness without speaking."""
        with self._bg_threads_lock:
            active_background_threads = sum(1 for thread in self._bg_threads if thread.is_alive())
        if self._openai_client is None:
            reason = self._speech_unavailable_reason or "TTS not initialized"
            return (
                "UnitreeSpeakSkill status: tts=unavailable; "
                f"reason={reason}; robot_audio=missing; "
                f"background_speech_threads={active_background_threads}."
            )
        robot_audio = "connected" if getattr(self, "_connection", None) is not None else "missing"
        return (
            "UnitreeSpeakSkill status: tts=ready; "
            f"robot_audio={robot_audio}; "
            f"background_speech_threads={active_background_threads}."
        )

    @skill
    def play_robot_audio_test(self) -> str:
        """Play a short generated tone through the Unitree Go2 onboard speaker.

        Use this to test the robot audio upload/playback path without calling
        OpenAI TTS.
        """
        try:
            wav_data = _generate_tone_wav()
            filename = f"audio_test_{int(time.time() * 1000)}"
            unique_id = self._upload_audio_to_robot(wav_data, filename)
            self._play_audio_on_robot(unique_id)
            return (
                "Robot audio test sent: "
                f"filename={filename}; bytes={len(wav_data)}; unique_id={unique_id}."
            )
        except Exception as exc:
            logger.error("Unitree robot audio test failed", exc_info=True)
            return f"Error playing robot audio test: {exc}"

    @skill
    def play_robot_megaphone_test(self) -> str:
        """Play a generated tone through the Unitree Go2 megaphone audio path.

        Use this when normal uploaded audio returns a unique_id but no sound is
        heard from the robot.
        """
        try:
            wav_data = _generate_tone_wav()
            self._upload_and_play_megaphone(wav_data, duration_s=5.0)
            return f"Robot megaphone audio test sent: bytes={len(wav_data)}."
        except Exception as exc:
            logger.error("Unitree robot megaphone audio test failed", exc_info=True)
            return f"Error playing robot megaphone audio test: {exc}"

    def _speak_bg(self, text: str) -> None:
        try:
            self._speak_blocking(text)
        finally:
            with self._bg_threads_lock:
                self._bg_threads = [
                    thread
                    for thread in self._bg_threads
                    if thread is not threading.current_thread()
                ]

    def _speak_blocking(self, text: str) -> str:
        try:
            logger.info("Generating Unitree speech audio with OpenAI TTS")
            wav_data = self._generate_wav(text)
            logger.info("Uploading Unitree speech audio", bytes=len(wav_data))
            filename = f"speak_{int(time.time() * 1000)}"
            unique_id = self._upload_audio_to_robot(wav_data, filename)
            logger.info("Playing Unitree speech audio", unique_id=unique_id)
            self._play_audio_on_robot(unique_id)
            display_text = text[:50] + "..." if len(text) > 50 else text
            return f"Spoke on robot: {display_text}"
        except Exception as exc:
            logger.error("Unitree robot speech failed", exc_info=True)
            return f"Error speaking on robot: {exc}"

    def _generate_wav(self, text: str) -> bytes:
        if self._openai_client is None:
            raise RuntimeError("TTS not initialized")
        response = self._openai_client.audio.speech.create(
            model="tts-1",
            voice="echo",
            input=text,
            speed=1.2,
            response_format="wav",
        )
        return response.content

    def _webrtc_request(self, api_id: int, parameter: dict[str, Any] | None = None) -> Any:
        return self._connection.publish_request(
            RTC_TOPIC["AUDIO_HUB_REQ"],
            {
                "api_id": api_id,
                "parameter": json.dumps(parameter or {}),
            },
        )

    def _upload_audio_to_robot(self, audio_data: bytes, filename: str) -> str:
        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode("utf-8")
        chunk_size = 61440
        chunks = [b64_data[i : i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
        total_chunks = len(chunks)

        logger.info(
            "Uploading Unitree audio",
            filename=filename,
            bytes=len(audio_data),
            chunks=total_chunks,
        )
        for index, chunk in enumerate(chunks, 1):
            self._webrtc_request(
                _AUDIO_API["UPLOAD_AUDIO_FILE"],
                {
                    "file_name": filename,
                    "file_type": "wav",
                    "file_size": len(audio_data),
                    "current_block_index": index,
                    "total_block_number": total_chunks,
                    "block_content": chunk,
                    "current_block_size": len(chunk),
                    "file_md5": file_md5,
                    "create_time": int(time.time() * 1000),
                },
            )

        response: Any = None
        unique_id: str | None = None
        for _attempt in range(1, 6):
            response = self._webrtc_request(_AUDIO_API["GET_AUDIO_LIST"], {})
            unique_id = _find_uploaded_audio_id(response, filename)
            if unique_id is not None:
                break
            time.sleep(0.2)
        self._last_robot_audio_upload = {
            "filename": filename,
            "bytes": len(audio_data),
            "chunks": total_chunks,
            "unique_id": unique_id,
            "list_response": response,
        }
        if unique_id is None:
            logger.warning(
                "Could not find uploaded Unitree audio by filename",
                filename=filename,
                response=response,
            )
            return filename
        logger.info("Unitree audio uploaded", filename=filename, unique_id=unique_id)
        return unique_id

    def _play_audio_on_robot(self, unique_id: str) -> None:
        logger.info("Requesting Unitree audio playback", unique_id=unique_id)
        self._webrtc_request(_AUDIO_API["SET_PLAY_MODE"], {"play_mode": _PLAY_MODE_NO_CYCLE})
        time.sleep(0.1)
        self._webrtc_request(_AUDIO_API["UNSUSPEND"], {})
        time.sleep(0.1)
        self._webrtc_request(_AUDIO_API["SELECT_START_PLAY"], {"unique_id": unique_id})

    def _upload_and_play_megaphone(self, audio_data: bytes, *, duration_s: float) -> None:
        logger.info("Entering Unitree megaphone mode", bytes=len(audio_data))
        self._webrtc_request(_AUDIO_API["ENTER_MEGAPHONE"], {})
        try:
            time.sleep(0.2)
            b64_data = base64.b64encode(audio_data).decode("utf-8")
            chunk_size = 4096
            chunks = [b64_data[i : i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
            total_chunks = len(chunks)
            logger.info("Uploading Unitree megaphone audio", chunks=total_chunks)
            for index, chunk in enumerate(chunks, 1):
                self._webrtc_request(
                    _AUDIO_API["UPLOAD_MEGAPHONE"],
                    {
                        "current_block_size": len(chunk),
                        "block_content": chunk,
                        "current_block_index": index,
                        "total_block_number": total_chunks,
                    },
                )
                if index < total_chunks:
                    time.sleep(0.02)
            time.sleep(duration_s + 0.5)
        finally:
            logger.info("Exiting Unitree megaphone mode")
            self._webrtc_request(_AUDIO_API["EXIT_MEGAPHONE"], {})


def _find_uploaded_audio_id(response: Any, filename: str) -> str | None:
    if not isinstance(response, dict):
        return None
    data = response.get("data")
    if isinstance(data, dict):
        data = data.get("data")
    if not isinstance(data, str):
        return None
    try:
        audio_list = json.loads(data).get("audio_list", [])
    except json.JSONDecodeError:
        return None
    for audio in audio_list:
        if isinstance(audio, dict) and audio.get("CUSTOM_NAME") == filename:
            unique_id = audio.get("UNIQUE_ID")
            return unique_id if isinstance(unique_id, str) else None
    return None


def _generate_tone_wav(
    *,
    frequency_hz: float = 660.0,
    duration_s: float = 5.0,
    sample_rate: int = 22050,
) -> bytes:
    buffer = io.BytesIO()
    amplitude = 0.95
    total_samples = int(duration_s * sample_rate)
    with wave.open(buffer, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        for index in range(total_samples):
            value = int(
                32767
                * amplitude
                * math.sin(2.0 * math.pi * frequency_hz * index / sample_rate)
            )
            wav_file.writeframes(struct.pack("<h", value))
    return buffer.getvalue()
