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

"""Deterministic identity, path-safety, and integrity helpers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import TypeAlias

JsonValue: TypeAlias = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]


def canonical_json(payload: JsonValue) -> bytes:
    """Serialize a JSON-compatible value into canonical UTF-8 bytes."""

    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def stable_opaque_id(namespace: str, payload: JsonValue) -> str:
    """Return a deterministic opaque identifier scoped to ``namespace``."""

    if not namespace or not namespace.replace("-", "").replace("_", "").isalnum():
        raise ValueError("namespace must contain only letters, digits, hyphens, and underscores")
    digest = hashlib.sha256(
        namespace.encode("utf-8") + b"\x00" + canonical_json(payload)
    ).hexdigest()
    return f"{namespace}_{digest}"


def validate_relative_path(value: str) -> str:
    """Validate a portable bundle-relative path without traversal components."""

    if not value or "\\" in value:
        raise ValueError("path must be a non-empty POSIX relative path")
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError("path must not be absolute or contain traversal components")
    return path.as_posix()


def hash_file_sha256(path: Path) -> str:
    """Return the lowercase SHA-256 digest for a file's exact bytes."""

    digest = hashlib.sha256()
    with path.open("rb") as artifact:
        for chunk in iter(lambda: artifact.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
