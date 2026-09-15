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

"""Pinned Kokoro assets shared by setup and runtime configuration."""

import hashlib
from pathlib import Path
import tempfile

import requests

from dimos.constants import CACHE_DIR

TTS_CACHE_DIR = CACHE_DIR / "tts"
MODEL_FILENAME = "kokoro-v1.0.int8.onnx"
VOICES_FILENAME = "voices-v1.0.bin"
RELEASE_URL = "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1"
ASSET_SHA256 = {
    MODEL_FILENAME: "ae315a79b623f244700e4afb9246c46a26066782e049ba174bf3ba433970ee9c",
    VOICES_FILENAME: "bca610b8308e8d99f32e6fe4197e7ec01679264efed0cac9140fe9c29f1fbf7d",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ensure_asset(destination: Path, filename: str) -> None:
    """Download a pinned asset only when its destination is missing or corrupt."""
    expected = ASSET_SHA256[filename]
    cache_dir = destination.parent
    cache_dir.mkdir(parents=True, exist_ok=True)
    if destination.is_file() and _sha256(destination) == expected:
        print(f"Verified existing {destination}", flush=True)
        return
    print(f"Downloading {filename} to {cache_dir} ...", flush=True)
    temporary: Path | None = None
    try:
        with requests.get(f"{RELEASE_URL}/{filename}", stream=True, timeout=(10, 60)) as response:
            response.raise_for_status()
            digest = hashlib.sha256()
            with tempfile.NamedTemporaryFile(
                dir=cache_dir, prefix=f".{filename}.", suffix=".part", delete=False
            ) as output:
                temporary = Path(output.name)
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    output.write(chunk)
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                raise ValueError(f"Checksum mismatch for {filename}; rerun collection to retry")
            temporary.replace(destination)
            print(f"Installed and verified {destination}", flush=True)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
