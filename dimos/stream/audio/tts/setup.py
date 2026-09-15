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

"""Download and verify local Kokoro assets without importing robot or inference code."""

import hashlib
from pathlib import Path
import sys
import tempfile

import requests

from dimos.stream.audio.tts.assets import ASSET_SHA256, RELEASE_URL, TTS_CACHE_DIR


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def setup_assets(cache_dir: Path = TTS_CACHE_DIR) -> None:
    """Install verified assets, reusing valid files and replacing corrupt ones."""
    cache_dir.mkdir(parents=True, exist_ok=True)
    for filename, expected in ASSET_SHA256.items():
        destination = cache_dir / filename
        if destination.is_file() and _sha256(destination) == expected:
            print(f"Verified existing {destination}", flush=True)
            continue
        print(f"Downloading {filename} to {cache_dir} ...", flush=True)
        temporary: Path | None = None
        try:
            with requests.get(
                f"{RELEASE_URL}/{filename}", stream=True, timeout=(10, 60)
            ) as response:
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
                    raise ValueError(f"Checksum mismatch for {filename}; rerun setup to retry")
                temporary.replace(destination)
                print(f"Installed and verified {destination}", flush=True)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    print("TTS assets ready. Collection can now synthesize speech offline.", flush=True)


def main() -> int:
    try:
        setup_assets()
    except (OSError, requests.RequestException, ValueError) as exc:
        print(f"TTS setup failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
