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

"""Text-to-speech providers for the Fetch prototype."""

from __future__ import annotations

import logging
import os
import struct
import uuid
from typing import Literal

logger = logging.getLogger("fetch.tts")

# Lazily-created, reused Cartesia HTTP client (avoids a TLS handshake per call on
# the latency-sensitive speech path). Cached per event loop and recreated if the
# running loop changes or the client is closed, since an httpx.AsyncClient is
# bound to the loop it was created on.
_cartesia_client: object | None = None
_cartesia_loop: object | None = None

TtsProvider = Literal["openai", "gemini", "cartesia"]

DEFAULT_GEMINI_TTS_MODEL = "gemini-3.1-flash-live-preview"
GEMINI_TTS_SAMPLE_RATE = 24000

DEFAULT_CARTESIA_TTS_MODEL = "sonic-3.5-2026-05-04"
CARTESIA_BASE_URL = "https://api.cartesia.ai"
CARTESIA_VERSION = "2026-03-01"
CARTESIA_SAMPLE_RATE = 24000
# Cartesia's recommended stable agent voice (Jameson); used as the default.
DEFAULT_CARTESIA_VOICE = "a5136bf9-224c-4d76-b823-52bd5efcffcc"
# Map the shared voice names to Cartesia voice ids (recommended/stable voices).
VOICE_MAP_OPENAI_TO_CARTESIA: dict[str, str] = {
    "alloy": "f786b574-daa5-4673-aa0c-cbe3e8534c02",   # Katie
    "echo": "a5136bf9-224c-4d76-b823-52bd5efcffcc",    # Jameson
    "fable": "db6b0ed5-d5d3-463d-ae85-518a07d3c2b4",   # Skylar
    "onyx": "630ed21c-2c5c-41cf-9d82-10a7fd668370",    # Corey
    "nova": "62ae83ad-4f6a-430b-af41-a9bede9286ca",    # Gemma
    "shimmer": "f786b574-daa5-4673-aa0c-cbe3e8534c02", # Katie
}

VOICE_MAP_OPENAI_TO_GEMINI: dict[str, str] = {
    "alloy": "Kore",
    "echo": "Charon",
    "fable": "Puck",
    "onyx": "Orus",
    "nova": "Aoede",
    "shimmer": "Led",
}

GEMINI_PREBUILT_VOICES = frozenset({
    "Achernar", "Achird", "Algieba", "Algenib", "Aoede", "Autonoe",
    "Capella", "Charon", "Despina", "Erinome", "Fenrir", "Kore",
    "Laomedeia", "Led", "Oberon", "Orus", "Pollux", "Puck",
    "Rasalgethi", "Sadachbia", "Sadaltager", "Schedar", "Sulafat",
    "Titania", "Umbriel", "Zephyr", "Zubenelgenubi",
})


def map_voice(voice: str, tts_provider: TtsProvider) -> str:
    if tts_provider == "gemini":
        if voice in GEMINI_PREBUILT_VOICES:
            return voice
        return VOICE_MAP_OPENAI_TO_GEMINI.get(voice, "Kore")
    if tts_provider == "cartesia":
        # Pass through an explicit Cartesia voice id (a UUID); otherwise map a
        # shared name, falling back to the default voice.
        if _is_uuid(voice):
            return voice
        return VOICE_MAP_OPENAI_TO_CARTESIA.get(voice, DEFAULT_CARTESIA_VOICE)
    return voice


def _is_uuid(value: str) -> bool:
    try:
        uuid.UUID(str(value))
        return True
    except ValueError:
        return False


def pcm_to_wav(
    pcm: bytes,
    sample_rate: int = GEMINI_TTS_SAMPLE_RATE,
    channels: int = 1,
    sample_width: int = 2,
) -> bytes:
    data_size = len(pcm)
    header = struct.pack(
        "<4sI4s4sIHHIIHH4sI",
        b"RIFF",
        36 + data_size,
        b"WAVE",
        b"fmt ",
        16,
        1,
        channels,
        sample_rate,
        sample_rate * channels * sample_width,
        channels * sample_width,
        sample_width * 8,
        b"data",
        data_size,
    )
    return header + pcm


def _gemini_api_key() -> tuple[str, str | None]:
    return (
        "GEMINI_API_KEY or GOOGLE_API_KEY",
        os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY"),
    )


def _cartesia_api_key() -> tuple[str, str | None]:
    return ("CARTESIA_API_KEY", os.getenv("CARTESIA_API_KEY"))


async def gemini_live_tts(text: str, voice: str = "Kore") -> bytes:
    key_name, api_key = _gemini_api_key()
    if not api_key:
        raise RuntimeError(f"{key_name} is not set")

    from google import genai
    from google.genai import types

    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(
                    voice_name=voice,
                )
            )
        ),
    )

    client = genai.Client(api_key=api_key)
    pcm_chunks: list[bytes] = []

    async with client.aio.live.connect(
        model=DEFAULT_GEMINI_TTS_MODEL,
        config=config,
    ) as session:
        await session.send_client_content(
            turns=[types.Content(
                role="user",
                parts=[types.Part.from_text(text=f"Speak exactly this text, nothing else: {text}")]
            )],
            turn_complete=True,
        )
        async for response in session.receive():
            server_content = getattr(response, "server_content", None)
            if server_content is None:
                continue
            model_turn = getattr(server_content, "model_turn", None)
            if model_turn is None:
                continue
            parts = getattr(model_turn, "parts", None)
            if parts is None:
                continue
            for part in parts:
                inline_data = getattr(part, "inline_data", None)
                if inline_data is None:
                    continue
                data = getattr(inline_data, "data", None)
                if data:
                    pcm_chunks.append(data)

    if not pcm_chunks:
        raise RuntimeError("Gemini Live TTS returned no audio data")

    return pcm_to_wav(b"".join(pcm_chunks))


async def cartesia_tts(
    text: str,
    voice: str = DEFAULT_CARTESIA_VOICE,
    model: str = DEFAULT_CARTESIA_TTS_MODEL,
) -> bytes:
    """Synthesize speech with Cartesia Sonic and return WAV bytes."""
    key_name, api_key = _cartesia_api_key()
    if not api_key:
        raise RuntimeError(f"{key_name} is not set")

    import asyncio

    import httpx

    global _cartesia_client, _cartesia_loop
    loop = asyncio.get_running_loop()
    if _cartesia_client is None or _cartesia_client.is_closed or _cartesia_loop is not loop:
        _cartesia_client = httpx.AsyncClient(timeout=30)
        _cartesia_loop = loop

    headers = {
        "X-API-Key": api_key,
        "Cartesia-Version": CARTESIA_VERSION,
        "Content-Type": "application/json",
    }
    body = {
        "model_id": model,
        "transcript": text,
        "voice": {"mode": "id", "id": voice},
        "output_format": {
            "container": "wav",
            "encoding": "pcm_s16le",
            "sample_rate": CARTESIA_SAMPLE_RATE,
        },
    }
    response = await _cartesia_client.post(
        f"{CARTESIA_BASE_URL}/tts/bytes", headers=headers, json=body
    )
    if response.status_code != 200:
        # Log the upstream body server-side; never surface it to the client.
        logger.error("Cartesia TTS HTTP %s: %s", response.status_code, response.text[:500])
        raise RuntimeError(f"Cartesia TTS failed (HTTP {response.status_code})")
    return response.content
