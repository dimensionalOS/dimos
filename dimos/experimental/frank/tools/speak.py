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

"""Say something out of the laptop's default audio output (the Bluetooth speaker on the robot).

ElevenLabs streaming text-to-speech, raw PCM so no mp3 decoding is needed. Audio starts
playing as the first chunk arrives. Phrases are cached by hash so repeats are instant.

    uv run python dimos/experimental/frank/tools/speak.py "Hi, I'm Frank."
    uv run python dimos/experimental/frank/tools/speak.py --voices      # list voices on the account
    uv run python dimos/experimental/frank/tools/speak.py --device      # show the current output device

Env: ELEVENLABS_API_KEY (required), ELEVENLABS_VOICE_ID (optional, defaults to a premade
voice). Read from `dimos/experimental/frank/.env`, then the repo root `.env`, then the environment.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import sys

from dotenv import load_dotenv
import numpy as np
import requests
import sounddevice as sd

SAMPLE_RATE = 24000
MODEL = os.environ.get(
    "ELEVENLABS_MODEL", "eleven_v3"
)  # v3 understands [chuckles], [pause], [laughs]
VOICE_SETTINGS = {"stability": 0.35, "similarity_boost": 0.8, "style": 0.6}
DEFAULT_VOICE = "pNInz6obpgDQGcFmaJgB"  # "Adam", an ElevenLabs premade voice
API = "https://api.elevenlabs.io/v1"
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent  # dimos/experimental/frank
CACHE = ROOT / "cache"


def _env() -> tuple[str, str]:
    load_dotenv(ROOT / ".env")
    load_dotenv()
    key = os.environ.get("ELEVENLABS_API_KEY")
    if not key:
        sys.exit(f"ELEVENLABS_API_KEY is not set. Put it in {ROOT / '.env'}")
    return key, os.environ.get("ELEVENLABS_VOICE_ID", DEFAULT_VOICE)


def voices() -> list[tuple[str, str]]:
    key, _ = _env()
    r = requests.get(f"{API}/voices", headers={"xi-api-key": key}, timeout=15)
    r.raise_for_status()
    return [(v["voice_id"], v["name"]) for v in r.json()["voices"]]


def output_device() -> str:
    return str(sd.query_devices(sd.default.device[1])["name"])


def _play_bytes(pcm: bytes) -> None:
    sd.play(np.frombuffer(pcm, dtype=np.int16), SAMPLE_RATE, blocking=True)


def speak(text: str) -> None:
    key, voice = _env()
    CACHE.mkdir(exist_ok=True)
    path = CACHE / (hashlib.sha1(f"{voice}:{MODEL}:{text}".encode()).hexdigest() + ".pcm")
    if path.exists():
        _play_bytes(path.read_bytes())
        return

    r = requests.post(
        f"{API}/text-to-speech/{voice}/stream",
        params={"output_format": f"pcm_{SAMPLE_RATE}"},
        headers={"xi-api-key": key},
        json={"text": text, "model_id": MODEL, "voice_settings": VOICE_SETTINGS},
        stream=True,
        timeout=30,
    )
    r.raise_for_status()

    chunks: list[bytes] = []
    carry = b""
    with sd.RawOutputStream(samplerate=SAMPLE_RATE, channels=1, dtype="int16") as out:
        for chunk in r.iter_content(chunk_size=4096):
            chunks.append(chunk)
            buf = carry + chunk
            usable = len(buf) - (len(buf) % 2)  # int16 alignment
            out.write(buf[:usable])
            carry = buf[usable:]
    path.write_bytes(b"".join(chunks))


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(2)
    if args[0] == "--voices":
        for vid, name in voices():
            print(f"{vid}  {name}")
        sys.exit(0)
    if args[0] == "--device":
        print(output_device())
        sys.exit(0)
    print(f"[speak] out: {output_device()}", file=sys.stderr)
    speak(" ".join(args))
