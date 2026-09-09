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

"""Design a voice from a text description with ElevenLabs, audition the previews, save the winner.

    uv run python dimos/experimental/frank/tools/design_voice.py preview "confident, suave, funny ..." [--text "..."]
    uv run python dimos/experimental/frank/tools/design_voice.py play 2               # replay preview 2
    uv run python dimos/experimental/frank/tools/design_voice.py save 2 "Frank"       # create the voice, prints its id
    uv run python dimos/experimental/frank/tools/design_voice.py remix <voice_id> "warmer, slower" [--text "..."]

Previews are kept in `dimos/experimental/frank/cache/voice_design/` so play/save work across runs.
Needs ELEVENLABS_API_KEY (see speak.py).
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
import sys

import requests

sys.path.insert(0, str(Path(__file__).parent))
import speak

DIR = speak.CACHE / "voice_design"
STATE = DIR / "previews.json"


def _headers() -> dict[str, str]:
    key, _ = speak._env()
    return {"xi-api-key": key}


def preview(description: str, text: str | None, remix_of: str | None = None) -> None:
    body: dict[str, object] = {"voice_description": description, "model_id": "eleven_ttv_v3"}
    if text:
        body["text"] = text
    else:
        body["auto_generate_text"] = True
    endpoint = f"text-to-voice/{remix_of}/remix" if remix_of else "text-to-voice/design"
    r = requests.post(
        f"{speak.API}/{endpoint}",
        params={"output_format": f"pcm_{speak.SAMPLE_RATE}"},
        headers=_headers(),
        json=body,
        timeout=120,
    )
    if not r.ok:
        sys.exit(f"{r.status_code}: {r.text}")
    data = r.json()
    DIR.mkdir(parents=True, exist_ok=True)
    state = {"description": description, "text": data.get("text", text), "previews": []}
    for i, p in enumerate(data["previews"], 1):
        path = DIR / f"{i}.pcm"
        path.write_bytes(base64.b64decode(p["audio_base_64"]))
        state["previews"].append({"generated_voice_id": p["generated_voice_id"], "file": str(path)})
    STATE.write_text(json.dumps(state, indent=2))
    print(f"text: {state['text']}")
    for i in range(1, len(state["previews"]) + 1):
        print(f"--- preview {i}")
        play(i)


def play(i: int) -> None:
    speak._play_bytes((DIR / f"{i}.pcm").read_bytes())


def save(i: int, name: str) -> None:
    state = json.loads(STATE.read_text())
    body = {
        "voice_name": name,
        "voice_description": state["description"],
        "generated_voice_id": state["previews"][i - 1]["generated_voice_id"],
    }
    r = requests.post(f"{speak.API}/text-to-voice", headers=_headers(), json=body, timeout=60)
    if not r.ok:
        sys.exit(f"{r.status_code}: {r.text}")
    vid = r.json()["voice_id"]
    print(f"saved voice {name!r}: {vid}")
    print(f"add to {speak.ROOT / '.env'}:  ELEVENLABS_VOICE_ID={vid}")


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        sys.exit(2)
    if a[0] == "preview":
        text = a[a.index("--text") + 1] if "--text" in a else None
        preview(a[1], text)
    elif a[0] == "remix":
        text = a[a.index("--text") + 1] if "--text" in a else None
        preview(a[2], text, remix_of=a[1])
    elif a[0] == "play":
        play(int(a[1]))
    elif a[0] == "save":
        save(int(a[1]), a[2])
    else:
        print(__doc__)
        sys.exit(2)
