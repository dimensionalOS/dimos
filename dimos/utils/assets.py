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

"""Download model assets into the DimOS cache."""

import hashlib
import importlib
import logging
from pathlib import Path
from urllib.parse import urlparse

from filelock import FileLock
import requests

from dimos.constants import CACHE_DIR
from dimos.utils.cache import cache_usage_locked

logger = logging.getLogger(__name__)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@cache_usage_locked
def download_http_asset(url: str, *, sha256: str | None = None) -> Path:
    """Cache an HTTPS file, validating its checksum when supplied."""
    parsed = urlparse(url)
    if parsed.scheme != "https" or not parsed.netloc:
        raise ValueError("Model asset URL must use HTTPS")
    if sha256 is not None:
        sha256 = sha256.lower()
        if len(sha256) != 64 or any(c not in "0123456789abcdef" for c in sha256):
            raise ValueError("Asset SHA-256 must contain 64 hexadecimal characters")
    key = hashlib.sha256(f"{url}\0{sha256 or ''}".encode()).hexdigest()
    root = CACHE_DIR / "assets" / "http" / key
    root.mkdir(parents=True, exist_ok=True)
    name = Path(parsed.path).name
    files = root / "files"
    files.mkdir(exist_ok=True)
    destination = files / (name if name not in {"", ".", ".."} else "asset")
    partial = root / ".download.part"
    with FileLock(root / ".download.lock"):
        if destination.is_file() and (sha256 is None or _sha256(destination) == sha256):
            return destination
        logger.info("Downloading model asset: %s", url)
        try:
            with requests.get(url, stream=True, timeout=(10, 60)) as response:
                response.raise_for_status()
                digest = hashlib.sha256()
                with partial.open("wb") as stream:
                    for chunk in response.iter_content(chunk_size=1024 * 1024):
                        stream.write(chunk)
                        digest.update(chunk)
            if sha256 is not None and digest.hexdigest() != sha256:
                raise ValueError("SHA-256 mismatch")
            partial.replace(destination)
        except (OSError, ValueError) as exc:
            raise RuntimeError(f"Could not download model asset {url}: {exc}") from exc
        finally:
            partial.unlink(missing_ok=True)
    return destination


@cache_usage_locked
def download_hf_asset(*, repo_id: str, revision: str, filename: str) -> Path:
    """Cache a Hugging Face file at an explicit revision; cache hits stay offline."""
    if not revision.strip():
        raise ValueError("Hugging Face assets require an explicit revision")
    # HF remains optional for HTTPS-only consumers, including ABC.
    hub = importlib.import_module("huggingface_hub")
    cache_dir = CACHE_DIR / "assets" / "huggingface"
    cached = hub.try_to_load_from_cache(repo_id, filename, revision=revision, cache_dir=cache_dir)
    if isinstance(cached, str):
        return Path(cached)
    logger.info("Downloading Hugging Face asset: %s@%s/%s", repo_id, revision, filename)
    try:
        return Path(hub.hf_hub_download(repo_id, filename, revision=revision, cache_dir=cache_dir))
    except (OSError, ValueError) as exc:
        raise RuntimeError(
            f"Could not download Hugging Face asset {repo_id}@{revision}/{filename}: {exc}"
        ) from exc
