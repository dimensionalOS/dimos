#!/usr/bin/env python3
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

"""Measure real round-trip latency of the models Fetch uses.

This hits the live APIs, so it needs OPENAI_API_KEY, GEMINI_API_KEY (or
GOOGLE_API_KEY), and/or CARTESIA_API_KEY (loaded from the repo .env, same as the
server). Each provider whose key is missing is skipped. Run from the repo root:

    python3 scripts/latency_bench.py           # 3 runs each
    python3 scripts/latency_bench.py --runs 5

Use the interpreter that has both the openai and google-genai SDKs installed
(here that is python3, not the anaconda python).
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import os
import statistics
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
load_dotenv(ROOT / ".env")
load_dotenv()

import policy  # noqa: E402  (after sys.path + dotenv)
import tts  # noqa: E402

SAMPLE_IMAGE = ROOT / "static" / "idle-camera-feed.png"
SAMPLE_TEXT = (
    "Hey there. Grab an ice-cold Coke from my back, then hold it up for a quick photo."
)
# Dedicated Gemini TTS model (generate_content API), distinct from the Live model.
GEMINI_TTS_MODEL = "gemini-3.1-flash-tts-preview"

# Cartesia Sonic TTS (REST; no SDK needed). Voice = Skylar (en).
CARTESIA_BASE = "https://api.cartesia.ai"
CARTESIA_VERSION = "2026-03-01"
CARTESIA_MODEL = "sonic-3.5-2026-05-04"
CARTESIA_VOICE_ID = "db6b0ed5-d5d3-463d-ae85-518a07d3c2b4"


def _have_gemini() -> bool:
    return bool(os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY"))


def _have_openai() -> bool:
    return bool(os.getenv("OPENAI_API_KEY"))


def _have_cartesia() -> bool:
    return bool(os.getenv("CARTESIA_API_KEY"))


def _summary(samples: list[float]) -> str:
    return (
        f"min {min(samples):.2f}s  median {statistics.median(samples):.2f}s  "
        f"max {max(samples):.2f}s  (n={len(samples)})"
    )


def _image_data_url() -> tuple[str, int]:
    """Match what the browser sends: ~640px-wide JPEG at quality 0.72."""
    try:
        import io

        from PIL import Image

        img = Image.open(SAMPLE_IMAGE).convert("RGB")
        width = min(640, img.width)
        height = round(img.height * (width / img.width))
        img = img.resize((width, height))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=72)
        data = buf.getvalue()
        return "data:image/jpeg;base64," + base64.b64encode(data).decode("ascii"), len(data)
    except Exception:
        raw = SAMPLE_IMAGE.read_bytes()
        return "data:image/png;base64," + base64.b64encode(raw).decode("ascii"), len(raw)


def bench_vision(provider: str, runs: int, data_url: str) -> None:
    config = policy.FetchPolicyConfig(vision_provider=provider)
    pol = policy.FetchPolicy(config)
    print(f"\n[VISION] {provider} / {config.model}  (per-frame decision)")
    samples: list[float] = []
    for i in range(runs):
        start = time.perf_counter()
        try:
            decision = pol.analyze_frame(data_url)
            elapsed = time.perf_counter() - start
            samples.append(elapsed)
            print(f"  run {i + 1}: {elapsed:.2f}s  state={decision.get('state')}")
        except Exception as exc:  # real API errors propagate out of analyze_frame
            print(f"  run {i + 1}: ERROR  {exc!r}")
    if samples:
        print(f"  => {_summary(samples)}")


def bench_openai_tts(runs: int, model: str) -> None:
    from openai import OpenAI

    client = OpenAI()
    print(f"\n[TTS] openai / {model}  (full synthesis)")
    samples: list[float] = []
    for i in range(runs):
        start = time.perf_counter()
        try:
            resp = client.audio.speech.create(
                model=model, voice="echo", input=SAMPLE_TEXT, response_format="mp3"
            )
            elapsed = time.perf_counter() - start
            samples.append(elapsed)
            print(f"  run {i + 1}: {elapsed:.2f}s  {len(resp.content)} bytes")
        except Exception as exc:
            print(f"  run {i + 1}: ERROR  {exc!r}")
    if samples:
        print(f"  => {_summary(samples)}")


async def _gemini_live_once(model: str, voice: str) -> tuple[float, float]:
    """Return (time-to-first-audio-chunk, time-to-turn-complete) in seconds."""
    from google import genai
    from google.genai import types

    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    client = genai.Client(api_key=api_key)
    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice),
            )
        ),
    )
    start = time.perf_counter()
    first: float | None = None
    async with client.aio.live.connect(model=model, config=config) as session:
        await session.send_client_content(
            turns=[
                types.Content(
                    role="user",
                    parts=[types.Part.from_text(text=f"Speak exactly this text, nothing else: {SAMPLE_TEXT}")],
                )
            ],
            turn_complete=True,
        )
        async for resp in session.receive():
            server_content = getattr(resp, "server_content", None)
            model_turn = getattr(server_content, "model_turn", None) if server_content else None
            if model_turn is not None:
                for part in getattr(model_turn, "parts", None) or []:
                    inline = getattr(part, "inline_data", None)
                    if inline is not None and getattr(inline, "data", None) and first is None:
                        first = time.perf_counter() - start
            if server_content is not None and getattr(server_content, "turn_complete", False):
                break
    total = time.perf_counter() - start
    return (first if first is not None else total), total


def bench_gemini_live(runs: int, model: str) -> None:
    print(f"\n[TTS/LIVE] gemini / {model}  (time-to-first-audio + full turn)")
    firsts: list[float] = []
    totals: list[float] = []
    for i in range(runs):
        try:
            first, total = asyncio.run(_gemini_live_once(model, "Charon"))
            firsts.append(first)
            totals.append(total)
            print(f"  run {i + 1}: first-audio {first:.2f}s  full {total:.2f}s")
        except Exception as exc:
            print(f"  run {i + 1}: ERROR  {exc!r}")
    if firsts:
        print(f"  => first-audio  {_summary(firsts)}")
    if totals:
        print(f"  => full turn    {_summary(totals)}")


def bench_gemini_tts_model(runs: int, model: str) -> None:
    from google import genai
    from google.genai import types

    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    client = genai.Client(api_key=api_key)
    config = types.GenerateContentConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name="Charon"),
            )
        ),
    )
    print(f"\n[TTS] gemini / {model}  (full synthesis; no streaming)")
    samples: list[float] = []
    for i in range(runs):
        start = time.perf_counter()
        try:
            resp = client.models.generate_content(model=model, contents=SAMPLE_TEXT, config=config)
            audio = resp.candidates[0].content.parts[0].inline_data.data
            elapsed = time.perf_counter() - start
            samples.append(elapsed)
            print(f"  run {i + 1}: {elapsed:.2f}s  {len(audio)} bytes")
        except Exception as exc:
            print(f"  run {i + 1}: ERROR  {exc!r}")
    if samples:
        print(f"  => {_summary(samples)}")


def bench_cartesia_tts(runs: int, model: str) -> None:
    import json

    import httpx

    headers = {
        "X-API-Key": os.getenv("CARTESIA_API_KEY"),
        "Cartesia-Version": CARTESIA_VERSION,
        "Content-Type": "application/json",
    }
    base_body = {
        "model_id": model,
        "transcript": SAMPLE_TEXT,
        "voice": {"mode": "id", "id": CARTESIA_VOICE_ID},
    }
    print(f"\n[TTS] cartesia / {model}  (time-to-first-audio via SSE + full via bytes)")
    firsts: list[float] = []
    fulls: list[float] = []
    for i in range(runs):
        # Streaming first-audio (raw container is required for the SSE endpoint).
        first: float | None = None
        sse_body = {**base_body, "output_format": {"container": "raw", "encoding": "pcm_s16le", "sample_rate": 24000}}
        start = time.perf_counter()
        try:
            with httpx.stream("POST", f"{CARTESIA_BASE}/tts/sse", headers=headers, json=sse_body, timeout=60) as resp:
                if resp.status_code != 200:
                    raise RuntimeError(f"SSE HTTP {resp.status_code}: {resp.read().decode('utf-8', 'ignore')[:120]}")
                for line in resp.iter_lines():
                    if not line:
                        continue
                    payload = line[5:].strip() if line.startswith("data:") else line.strip()
                    try:
                        event = json.loads(payload)
                    except ValueError:
                        continue
                    if event.get("data") and first is None:
                        first = time.perf_counter() - start
                    if event.get("type") == "done" or event.get("done"):
                        break
        except Exception as exc:
            print(f"  run {i + 1}: SSE ERROR  {exc!r}")
        # Full synthesis (wav container).
        full: float | None = None
        nbytes = 0
        bytes_body = {**base_body, "output_format": {"container": "wav", "encoding": "pcm_s16le", "sample_rate": 24000}}
        start = time.perf_counter()
        try:
            resp = httpx.post(f"{CARTESIA_BASE}/tts/bytes", headers=headers, json=bytes_body, timeout=60)
            if resp.status_code != 200:
                raise RuntimeError(f"bytes HTTP {resp.status_code}: {resp.text[:120]}")
            full = time.perf_counter() - start
            nbytes = len(resp.content)
        except Exception as exc:
            print(f"  run {i + 1}: BYTES ERROR  {exc!r}")
        if first is not None and full is not None:
            firsts.append(first)
            fulls.append(full)
            print(f"  run {i + 1}: first-audio {first:.2f}s  full {full:.2f}s  ({nbytes} bytes)")
    if firsts:
        print(f"  => first-audio  {_summary(firsts)}")
    if fulls:
        print(f"  => full         {_summary(fulls)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Latency benchmark for Fetch models.")
    parser.add_argument("--runs", type=int, default=3, help="Calls per model (default 3).")
    parser.add_argument(
        "--gemini-tts-model",
        default=GEMINI_TTS_MODEL,
        help=f"Dedicated Gemini TTS model (default {GEMINI_TTS_MODEL}).",
    )
    parser.add_argument(
        "--cartesia-model",
        default=CARTESIA_MODEL,
        help=f"Cartesia Sonic TTS model (default {CARTESIA_MODEL}).",
    )
    args = parser.parse_args()

    print(f"Latency benchmark ({args.runs} runs/model). Sample image: {SAMPLE_IMAGE.name}")
    print(
        f"keys: openai={'yes' if _have_openai() else 'no'}  "
        f"gemini={'yes' if _have_gemini() else 'no'}  "
        f"cartesia={'yes' if _have_cartesia() else 'no'}"
    )

    data_url, frame_bytes = _image_data_url()
    print(f"vision frame: {frame_bytes // 1024} KB (production-sized JPEG)")

    if _have_openai():
        bench_vision("openai", args.runs, data_url)
    else:
        print("\n[VISION] openai: skipped (no OPENAI_API_KEY)")
    if _have_gemini():
        bench_vision("gemini", args.runs, data_url)
    else:
        print("\n[VISION] gemini: skipped (no GEMINI_API_KEY)")

    if _have_openai():
        bench_openai_tts(args.runs, "tts-1")
    else:
        print("\n[TTS] openai: skipped (no OPENAI_API_KEY)")
    if _have_gemini():
        bench_gemini_live(args.runs, tts.DEFAULT_GEMINI_TTS_MODEL)
        bench_gemini_tts_model(args.runs, args.gemini_tts_model)
    else:
        print("\n[TTS/LIVE] gemini: skipped (no GEMINI_API_KEY)")
    if _have_cartesia():
        bench_cartesia_tts(args.runs, args.cartesia_model)
    else:
        print("\n[TTS] cartesia: skipped (no CARTESIA_API_KEY)")

    print("\nNote: OpenAI Realtime (gpt-realtime-2) is a WebRTC route and is not benchmarked here.")


if __name__ == "__main__":
    main()
