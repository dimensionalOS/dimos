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

"""Explicit acquisition and immutable selection of externally stored SPACE data."""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import time
from typing import Any, BinaryIO, cast

import requests

from dimos.constants import CACHE_DIR
from dimos.evals.suites.lib.space.constants import (
    DATA_BYTES,
    DATA_MEMBER,
    DATA_SHA256,
    DATA_URL,
    DOWNLOAD_TIMEOUT_S,
    MAX_DOWNLOAD_BYTES,
    SELECTED_INDICES,
    SPACE_REPOSITORY,
    SPACE_REVISION,
    TASK,
)
from dimos.evals.suites.lib.space.upstream import verify_source


@dataclass(frozen=True)
class SpacePaths:
    root: Path = CACHE_DIR / "evals" / "space"

    @property
    def source(self) -> Path:
        return self.root / "source"

    @property
    def questions(self) -> Path:
        return self.root / TASK / "qas.json"


@dataclass(frozen=True)
class Example:
    index: int
    qa: dict[str, Any]

    @property
    def case_id(self) -> str:
        return f"space-map-sketching-text-{self.index:03d}"

    @property
    def question(self) -> str:
        return cast("str", self.qa["question"])

    @property
    def question_sha256(self) -> str:
        return hashlib.sha256(self.question.encode()).hexdigest()

    @property
    def environment(self) -> str:
        return Path(self.qa["metadata"]["env_dir"]).name

    def identity(self) -> dict[str, Any]:
        return {
            "case_id": self.case_id,
            "index": self.index,
            "question_sha256": self.question_sha256,
            "environment": self.environment,
        }


class BoundedReader(io.RawIOBase):
    """Bound compressed bytes and elapsed time without materializing the archive."""

    def __init__(self, stream: BinaryIO, *, limit: int, timeout_s: float) -> None:
        self._stream = stream
        self._limit = limit
        self._deadline = time.monotonic() + timeout_s
        self.bytes_read = 0

    def read(self, size: int = -1) -> bytes:
        if time.monotonic() >= self._deadline:
            raise TimeoutError("SPACE download deadline exceeded")
        remaining = self._limit - self.bytes_read + 1
        requested = remaining if size < 0 else min(size, remaining)
        data = self._stream.read(requested)
        self.bytes_read += len(data)
        if self.bytes_read > self._limit:
            raise ValueError("SPACE archive exceeded the bounded download limit")
        return data


def read_member(stream: BinaryIO | BoundedReader) -> bytes:
    """Return only the pinned regular-file member; never extract archive paths."""
    with tarfile.open(fileobj=stream, mode="r|gz") as archive:
        for member in archive:
            if member.name != DATA_MEMBER:
                continue
            if not member.isfile() or member.size != DATA_BYTES:
                raise ValueError("Unexpected SPACE member type or size")
            extracted = archive.extractfile(member)
            assert extracted is not None
            payload = extracted.read(DATA_BYTES + 1)
            verify_data(payload)
            return payload
    raise ValueError(f"SPACE archive does not contain {DATA_MEMBER}")


def verify_data(payload: bytes) -> None:
    if len(payload) != DATA_BYTES or hashlib.sha256(payload).hexdigest() != DATA_SHA256:
        raise ValueError("SPACE data does not match the pinned file size and SHA-256")


def acquire_data(destination: Path) -> int:
    """Download explicitly; publish only a complete, verified upstream file."""
    if destination.exists():
        verify_data(destination.read_bytes())
        return 0
    with requests.get(
        DATA_URL, stream=True, timeout=(15, 20), headers={"Accept-Encoding": "identity"}
    ) as response:
        response.raise_for_status()
        reader = BoundedReader(
            cast("BinaryIO", response.raw), limit=MAX_DOWNLOAD_BYTES, timeout_s=DOWNLOAD_TIMEOUT_S
        )
        payload = read_member(reader)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".space-download-", dir=destination.parent
    ) as directory:
        staged = Path(directory) / "qas.json"
        staged.write_bytes(payload)
        staged.replace(destination)
    return reader.bytes_read


def setup(paths: SpacePaths) -> dict[str, Any]:
    """Acquire source and one unchanged data file outside the DimOS checkout."""
    paths.root.mkdir(parents=True, exist_ok=True)
    if not paths.source.exists():
        with tempfile.TemporaryDirectory(prefix=".space-source-", dir=paths.root) as directory:
            source = Path(directory) / "source"
            subprocess.run(["git", "init", "--quiet", str(source)], check=True, timeout=30)
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(source),
                    "fetch",
                    "--quiet",
                    "--depth=1",
                    SPACE_REPOSITORY,
                    SPACE_REVISION,
                ],
                check=True,
                timeout=180,
            )
            subprocess.run(
                ["git", "-C", str(source), "checkout", "--quiet", "--detach", "FETCH_HEAD"],
                check=True,
                timeout=30,
            )
            verify_source(source)
            source.rename(paths.source)
    verify_source(paths.source)
    downloaded = acquire_data(paths.questions)
    examples = load_examples(paths)
    return {
        "space_revision": SPACE_REVISION,
        "data_sha256": DATA_SHA256,
        "downloaded_compressed_bytes": downloaded,
        "selected_cases": [example.identity() for example in examples],
    }


def load_examples(
    paths: SpacePaths, indices: Sequence[int] = SELECTED_INDICES
) -> tuple[Example, ...]:
    if not paths.questions.is_file():
        raise FileNotFoundError(
            "SPACE data is missing. Run python -m dimos.evals.suites.lib.space.commands setup"
        )
    payload = paths.questions.read_bytes()
    verify_data(payload)
    rows = json.loads(payload)
    examples = tuple(Example(index, rows[index]) for index in indices)
    if len({e.case_id for e in examples}) != len(examples):
        raise ValueError("SPACE selection contains duplicate source indices")
    if tuple(indices) == SELECTED_INDICES:
        layouts = Counter(e.environment.rsplit("_", 1)[0] for e in examples)
        labels = Counter(e.qa["answer"] for e in examples)
        if (
            len({e.environment for e in examples}) != 20
            or len(layouts) != 10
            or set(layouts.values()) != {2}
            or labels != {1: 5, 2: 5, 3: 5, 4: 5}
        ):
            raise ValueError("SPACE subset does not satisfy the frozen selection contract")
    return examples
