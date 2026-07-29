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

"""Resolve hosted replay references into verified local memory2 databases."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
from urllib.parse import unquote, urlparse

import requests

from dimos.hosted_data.repository import ReplayObject, sha256_file

_SCHEME = "dimos-replay"
_OBJECT_ID_RE = re.compile(r"^[0-9a-f]{64}$")
_DEFAULT_SERVER_URL = "http://127.0.0.1:8765"
_CONNECT_TIMEOUT_SECONDS = 10
_TRANSFER_TIMEOUT_SECONDS = 3600


class HostedReplayError(RuntimeError):
    """A hosted replay reference or transfer is invalid."""


@dataclass(frozen=True)
class HostedReplayReference:
    owner: str
    repository: str
    object_id: str

    @classmethod
    def parse(cls, value: str | Path) -> HostedReplayReference | None:
        text = str(value)
        parsed = urlparse(text)
        if parsed.scheme != _SCHEME:
            return None
        parts = [unquote(part) for part in parsed.path.split("/") if part]
        owner = unquote(parsed.netloc)
        if not owner or len(parts) != 2 or parsed.query or parsed.fragment:
            raise HostedReplayError(
                "hosted replay must be dimos-replay://OWNER/REPOSITORY/OBJECT_ID"
            )
        repository, object_id = parts
        if not _OBJECT_ID_RE.fullmatch(object_id):
            raise HostedReplayError("hosted replay object ID must be a lowercase SHA-256 digest")
        return cls(owner=owner, repository=repository, object_id=object_id)


def _cache_root() -> Path:
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return base / "dimos" / "hosted-replays"


def _objects_url(server_url: str, reference: HostedReplayReference) -> str:
    return (
        f"{server_url.rstrip('/')}/v1/repositories/{reference.owner}/{reference.repository}/objects"
    )


def _metadata(server_url: str, reference: HostedReplayReference) -> ReplayObject:
    try:
        response = requests.get(
            _objects_url(server_url, reference),
            timeout=_CONNECT_TIMEOUT_SECONDS,
        )
        response.raise_for_status()
    except requests.RequestException as exc:
        raise HostedReplayError(f"failed to list hosted replay repository: {exc}") from exc
    for raw in response.json():
        item = ReplayObject.from_dict(raw)
        if item.object_id == reference.object_id:
            if Path(item.filename).suffix.lower() not in {".db", ".sqlite", ".sqlite3"}:
                raise HostedReplayError(
                    f"hosted object {item.filename!r} is not a memory2 SQLite replay"
                )
            return item
    raise HostedReplayError(f"hosted replay object not found: {reference.object_id}")


def resolve_hosted_replay(
    value: str | Path,
    *,
    server_url: str | None = None,
    cache_root: Path | None = None,
) -> Path | None:
    """Download a ``dimos-replay://`` database once and return its cache path."""
    reference = HostedReplayReference.parse(value)
    if reference is None:
        return None

    target = (cache_root or _cache_root()) / f"{reference.object_id}.db"
    if target.is_file() and sha256_file(target) == reference.object_id:
        return target

    resolved_server_url = server_url or os.environ.get("DIMOS_DATA_SERVER_URL", _DEFAULT_SERVER_URL)
    item = _metadata(resolved_server_url, reference)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.part")
    try:
        response = requests.get(
            f"{_objects_url(resolved_server_url, reference)}/{reference.object_id}",
            stream=True,
            timeout=(_CONNECT_TIMEOUT_SECONDS, _TRANSFER_TIMEOUT_SECONDS),
        )
        response.raise_for_status()
        with temporary.open("wb") as destination:
            for chunk in response.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    destination.write(chunk)
        response.close()
        if temporary.stat().st_size != item.size_bytes:
            raise HostedReplayError("downloaded replay size does not match repository metadata")
        if sha256_file(temporary) != reference.object_id:
            raise HostedReplayError("downloaded replay failed SHA-256 verification")
        os.replace(temporary, target)
    except (OSError, requests.RequestException) as exc:
        raise HostedReplayError(f"failed to download hosted replay: {exc}") from exc
    finally:
        temporary.unlink(missing_ok=True)
    return target
