# Copyright 2025 Dimensional Inc.
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
import json
import os
import tempfile
import time

import numpy as np
from openai import OpenAI
from pydantic import Field
import soundfile as sf  # type: ignore[import-untyped]
from unitree_webrtc_connect.constants import RTC_TOPIC  # type: ignore[import-untyped]

from dimos.skills.skills import AbstractRobotSkill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Audio API constants (from go2_webrtc_driver)
AUDIO_API = {
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

PLAY_MODES = {"NO_CYCLE": "no_cycle", "SINGLE_CYCLE": "single_cycle", "LIST_LOOP": "list_loop"}


class UnitreeSpeak(AbstractRobotSkill):
    """Speak text out loud through the robot's speakers using WebRTC audio upload."""

    text: str = Field(..., description="Text to speak")
    voice: str = Field(
        default="echo", description="Voice to use (alloy, echo, fable, onyx, nova, shimmer)"
    )
    speed: float = Field(default=1.2, description="Speech speed (0.25 to 4.0)")
    use_megaphone: bool = Field(
        default=False, description="Use megaphone mode for lower latency (experimental)"
    )

    def __init__(self, **data) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**data)
        self._openai_client = None

    def _get_openai_client(self):  # type: ignore[no-untyped-def]
        if self._openai_client is None:
            self._openai_client = OpenAI()  # type: ignore[assignment]
        return self._openai_client

    def _generate_audio(self, text: str) -> bytes:
        try:
            client = self._get_openai_client()  # type: ignore[no-untyped-call]
            response = client.audio.speech.create(
                model="tts-1", voice=self.voice, input=text, speed=self.speed, response_format="mp3"
            )
            return response.content  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Error generating audio: {e}")
            raise

    def _webrtc_request(self, api_id: int, parameter: dict | None = None):  # type: ignore[no-untyped-def, type-arg]
        if parameter is None:
            parameter = {}

        request_data = {"api_id": api_id, "parameter": json.dumps(parameter) if parameter else "{}"}

        return self._robot.connection.publish_request(RTC_TOPIC["AUDIO_HUB_REQ"], request_data)  # type: ignore[attr-defined]

    def _upload_audio_to_robot(self, audio_data: bytes, filename: str) -> str:
        try:
            file_md5 = hashlib.md5(audio_data).hexdigest()
            b64_data = base64.b64encode(audio_data).decode("utf-8")

            chunk_size = 61440
            chunks = [b64_data[i : i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
            total_chunks = len(chunks)

            logger.info(f"Uploading audio '{filename}' in {total_chunks} chunks (optimized)")

            for i, chunk in enumerate(chunks, 1):
                parameter = {
                    "file_name": filename,
                    "file_type": "wav",
                    "file_size": len(audio_data),
                    "current_block_index": i,
                    "total_block_number": total_chunks,
                    "block_content": chunk,
                    "current_block_size": len(chunk),
                    "file_md5": file_md5,
                    "create_time": int(time.time() * 1000),
                }

                logger.debug(f"Sending chunk {i}/{total_chunks}")
                self._webrtc_request(AUDIO_API["UPLOAD_AUDIO_FILE"], parameter)

            logger.info(f"Audio upload completed for '{filename}'")

            list_response = self._webrtc_request(AUDIO_API["GET_AUDIO_LIST"], {})

            if list_response and "data" in list_response:
                data_str = list_response.get("data", {}).get("data", "{}")
                audio_list = json.loads(data_str).get("audio_list", [])

                for audio in audio_list:
                    if audio.get("CUSTOM_NAME") == filename:
                        return audio.get("UNIQUE_ID")  # type: ignore[no-any-return]

            logger.warning(
                f"Could not find uploaded audio '{filename}' in list, using filename as UUID"
            )
            return filename

        except Exception as e:
            logger.error(f"Error uploading audio to robot: {e}")
            raise

    def _play_audio_on_robot(self, uuid: str):  # type: ignore[no-untyped-def]
        try:
            self._webrtc_request(AUDIO_API["SET_PLAY_MODE"], {"play_mode": PLAY_MODES["NO_CYCLE"]})
            time.sleep(0.1)

            parameter = {"unique_id": uuid}

            logger.info(f"Playing audio with UUID: {uuid}")
            self._webrtc_request(AUDIO_API["SELECT_START_PLAY"], parameter)

        except Exception as e:
            logger.error(f"Error playing audio on robot: {e}")
            raise

    def _stop_audio_playback(self) -> None:
        try:
            logger.debug("Stopping audio playback")
            self._webrtc_request(AUDIO_API["PAUSE"], {})
        except Exception as e:
            logger.warning(f"Error stopping audio playback: {e}")

    def _upload_and_play_megaphone(self, audio_data: bytes, duration: float):  # type: ignore[no-untyped-def]
        try:
            logger.debug("Entering megaphone mode")
            self._webrtc_request(AUDIO_API["ENTER_MEGAPHONE"], {})

            time.sleep(0.2)

            b64_data = base64.b64encode(audio_data).decode("utf-8")

            chunk_size = 4096
            chunks = [b64_data[i : i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
            total_chunks = len(chunks)

            logger.info(f"Uploading megaphone audio in {total_chunks} chunks")

            for i, chunk in enumerate(chunks, 1):
                parameter = {
                    "current_block_size": len(chunk),
                    "block_content": chunk,
                    "current_block_index": i,
                    "total_block_number": total_chunks,
                }

                logger.debug(f"Sending megaphone chunk {i}/{total_chunks}")
                self._webrtc_request(AUDIO_API["UPLOAD_MEGAPHONE"], parameter)

                if i < total_chunks:
                    time.sleep(0.05)

            logger.info("Megaphone audio upload completed, waiting for playback")

            time.sleep(duration + 1.0)

        except Exception as e:
            logger.error(f"Error in megaphone mode: {e}")
            try:
                self._webrtc_request(AUDIO_API["EXIT_MEGAPHONE"], {})
            except:
                pass
            raise
        finally:
            try:
                logger.debug("Exiting megaphone mode")
                self._webrtc_request(AUDIO_API["EXIT_MEGAPHONE"], {})
                time.sleep(0.1)
            except Exception as e:
                logger.warning(f"Error exiting megaphone mode: {e}")

    def __call__(self) -> str:
        super().__call__()  # type: ignore[no-untyped-call]

        if not self._robot:
            logger.error("No robot instance provided to UnitreeSpeak skill")
            return "Error: No robot instance available"

        try:
            display_text = self.text[:50] + "..." if len(self.text) > 50 else self.text
            logger.info(f"Speaking: '{display_text}'")

            logger.debug("Generating audio with OpenAI TTS")
            audio_data = self._generate_audio(self.text)

            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp_mp3:
                tmp_mp3.write(audio_data)
                tmp_mp3_path = tmp_mp3.name

            try:
                audio_array, sample_rate = sf.read(tmp_mp3_path)

                if audio_array.ndim > 1:
                    audio_array = np.mean(audio_array, axis=1)

                target_sample_rate = 22050
                if sample_rate != target_sample_rate:
                    logger.debug(f"Resampling from {sample_rate}Hz to {target_sample_rate}Hz")
                    old_length = len(audio_array)
                    new_length = int(old_length * target_sample_rate / sample_rate)
                    old_indices = np.arange(old_length)
                    new_indices = np.linspace(0, old_length - 1, new_length)
                    audio_array = np.interp(new_indices, old_indices, audio_array)
                    sample_rate = target_sample_rate

                audio_array = audio_array / np.max(np.abs(audio_array))

                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_wav:
                    sf.write(tmp_wav.name, audio_array, sample_rate, format="WAV", subtype="PCM_16")
                    tmp_wav.seek(0)
                    wav_data = open(tmp_wav.name, "rb").read()
                    os.unlink(tmp_wav.name)

                logger.info(
                    f"Audio size: {len(wav_data) / 1024:.1f}KB, duration: {len(audio_array) / sample_rate:.1f}s"
                )

            finally:
                os.unlink(tmp_mp3_path)

            if self.use_megaphone:
                logger.debug("Using megaphone mode for lower latency")
                duration = len(audio_array) / sample_rate
                self._upload_and_play_megaphone(wav_data, duration)

                return f"Spoke: '{display_text}' on robot successfully (megaphone mode)"
            else:
                filename = f"speak_{int(time.time() * 1000)}"

                logger.debug("Uploading audio to robot")
                uuid = self._upload_audio_to_robot(wav_data, filename)

                logger.debug("Playing audio on robot")
                self._play_audio_on_robot(uuid)

                duration = len(audio_array) / sample_rate
                logger.debug(f"Waiting {duration:.1f}s for playback to complete")
                # time.sleep(duration + 0.2)

                # self._stop_audio_playback()

                return f"Spoke: '{display_text}' on robot successfully"

        except Exception as e:
            logger.error(f"Error in speak skill: {e}")
            return f"Error speaking text: {e!s}"
