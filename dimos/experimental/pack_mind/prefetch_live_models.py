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

"""Pre-cache the HuggingFace models the live PACK MIND stack loads at runtime.

Run this ONCE on each demo laptop while it has real internet. Afterwards the demo
runs fully offline with ``HF_HUB_OFFLINE=1`` — every model loads from the local
cache, so a flaky venue network (or the dog's internet-less WiFi AP) can't stall a
model load mid-demo.

    uv run python -m dimos.experimental.pack_mind.prefetch_live_models

What these are (and why they bite if missing):
  - moondream2: the VLM behind ``look_out_for`` (GlobalConfig.detection_model =
    "moondream"). Loads lazily on the FIRST detection — uncached = crash mid-demo.
  - faster-whisper-base: WebInput's voice STT, loaded at module start — uncached +
    HF_HUB_OFFLINE=1 = the start-time LocalEntryNotFoundError.

CLIP and YOLO ship in ``data/`` (git-lfs), so they are not fetched here. The
LLM/agent brain (GPT-4o / Qwen-VL) is a hosted API, not a local download — it needs
internet at demo time regardless (run the coordinator laptop on a router with WAN).
"""

from __future__ import annotations

import os

# Force online for this one run, even if the shell exported HF_HUB_OFFLINE=1.
os.environ["HF_HUB_OFFLINE"] = "0"
os.environ["TRANSFORMERS_OFFLINE"] = "0"

from huggingface_hub import snapshot_download  # noqa: E402

# Plain snapshot is enough for these (loaded straight from the cached dir).
SNAPSHOT_MODELS: list[tuple[str, str]] = [
    ("Systran/faster-whisper-base", "WebInput voice STT (faster-whisper)"),
]


def main() -> None:
    for repo, why in SNAPSHOT_MODELS:
        print(f"\n== {repo}  — {why}")
        print(f"   cached: {snapshot_download(repo)}")

    # moondream2 is loaded via trust_remote_code, which pulls THREE things a plain
    # snapshot_download misses: the weights, the custom code modules
    # (transformers_modules/...), and a SEPARATE tokenizer repo it references
    # (moondream/starmie-v1). A full from_pretrained fetches all of them, so the
    # live look_out_for then loads fully offline.
    print("\n== vikhyatk/moondream2  — look_out_for VLM (full load for transitive deps)")
    from transformers import AutoModelForCausalLM

    AutoModelForCausalLM.from_pretrained("vikhyatk/moondream2", trust_remote_code=True)
    print("   moondream2 + code modules + starmie-v1 tokenizer cached")

    print("\nDone. Run the live demo with HF_HUB_OFFLINE=1 — models load from cache.")


if __name__ == "__main__":
    main()
