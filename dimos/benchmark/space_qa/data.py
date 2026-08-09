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

"""The official SPACE data release: fetched from Apple, never redistributed.

The release is a 3.6 GB tarball under Apple's own terms. It is downloaded once
into the user cache, verified against a pinned digest, and read from there. No
question, answer or transcript from it belongs in this repository, so every
fixture in the tests beside this module is generated.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tarfile
import tempfile
from typing import Any

import requests

from dimos.benchmark.space_qa.source import space_cache_root
from dimos.benchmark.space_qa.tasks import SpaceTextTask

SPACE_DATA_URL = "https://ml-site.cdn-apple.com/datasets/space/space.tar.gz"
# Digest of the tarball this integration was developed against, recorded from a
# local copy. Apple publishes no checksum, so this is the evidence that a later
# download is the same release rather than a silently re-cut one.
SPACE_DATA_SHA256 = "fd4cc8964224f4732d8741ee298b8bbd43f11123911846ef78278b4a3f3ce82d"
# Points at an already-extracted release root instead of the cached one.
SPACE_DATA_ENV = "DIMOS_SPACE_DATA"

RELEASE_DIR_NAME = "SPACE_data_release"
PROVENANCE_NAME = "provenance.json"
DOWNLOAD_CHUNK_BYTES = 1 << 22
# Both bound a single blocking socket operation, not the transfer: 3.6 GB over
# a slow link legitimately streams for an hour, and neither timeout cuts that
# short. What they end is a host that never completes the handshake, and a
# connection that stops delivering bytes — a download that will never finish,
# which without a deadline waits forever instead of failing.
DOWNLOAD_CONNECT_TIMEOUT_SECONDS = 30.0
DOWNLOAD_READ_TIMEOUT_SECONDS = 120.0

SpaceRow = dict[str, Any]


@dataclass(frozen=True, kw_only=True)
class TaskQuestions:
    """A task's questions, and the digest of the bytes this run read them from.

    The two attest to different things and are not interchangeable. The release
    digest says which archive the cache was unpacked from, on the word of the
    record written beside it; ``sha256`` here is computed over the file that was
    actually opened, so it holds whatever a run really asked its questions from
    — including a release edited after it was unpacked.
    """

    rows: tuple[SpaceRow, ...]
    sha256: str


def space_data_dir() -> Path:
    return space_cache_root() / "data"


def resolve_space_data() -> Path:
    """Return the release root, downloading and unpacking it once if needed."""
    override = os.environ.get(SPACE_DATA_ENV)
    if override:
        path = Path(override).expanduser()
        if not path.is_dir():
            raise FileNotFoundError(f"{SPACE_DATA_ENV}={override} is not a directory")
        return path.resolve()
    release = space_data_dir() / RELEASE_DIR_NAME
    if release.is_dir():
        _verify_cached_release(release)
        return release
    return download_release(space_data_dir())


def release_sha256(release: Path) -> str | None:
    """The digest recorded in the ``provenance.json`` beside a release, if there is one.

    This reports the release's own claim about which archive it came from; it
    reads none of the release. A release this side unpacked has that record and
    reports it wherever it is reached from; one extracted by hand and pointed at
    with ``DIMOS_SPACE_DATA`` has none, and returns None rather than borrowing
    the pinned digest.
    """
    provenance = release.parent / PROVENANCE_NAME
    if not provenance.is_file():
        return None
    recorded = json.loads(provenance.read_text(encoding="utf-8"))
    # Valid JSON that is no record at all — `null`, a list, a bare number — says
    # as little about the release as a truncated one, and is read the same way.
    if not isinstance(recorded, dict):
        return None
    digest = recorded.get("sha256")
    return digest if isinstance(digest, str) else None


def download_release(target: Path) -> Path:
    """Fetch the 3.6 GB release, verify it, and unpack it into ``target``."""
    _require_extraction_filtering()
    target.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".space-data-", dir=target))
    try:
        archive = staging / "space.tar.gz"
        digest = _download(SPACE_DATA_URL, archive)
        if digest != SPACE_DATA_SHA256:
            raise RuntimeError(
                f"{SPACE_DATA_URL} hashed to {digest}, expected {SPACE_DATA_SHA256}; "
                "the published release changed and the pin has to be re-reviewed"
            )
        unpacked = staging / "unpacked"
        with tarfile.open(archive, "r:gz") as tar:
            # The `data` filter refuses absolute paths and escapes from `unpacked`.
            tar.extractall(unpacked, filter="data")
        archive.unlink()
        release = unpacked / RELEASE_DIR_NAME
        if not release.is_dir():
            raise RuntimeError(f"the release archive holds no {RELEASE_DIR_NAME}/ directory")
        published = target / RELEASE_DIR_NAME
        # The record goes down first. Losing the machine between the two leaves
        # a record with no release, which the next run cannot even see: it finds
        # no release directory, downloads again and overwrites the record. The
        # other order leaves a release with no record, which no run may read.
        _write_provenance(staging, target, digest)
        shutil.rmtree(published, ignore_errors=True)
        os.replace(release, published)
        return published
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def load_task_rows(task: SpaceTextTask, data_root: Path) -> TaskQuestions:
    """Read a task's questions, refusing a file that no longer has the shape we sampled.

    Row position is the only address SPACE gives a question, so a file that
    gained or lost rows would silently re-point every sampled index at a
    different question. That is a failed run, not a smaller one.

    The bytes are hashed on the way past, and the digest travels to the manifest.
    A release is only ever checked against what its provenance record claims, so
    this is the one place a run can say what it read rather than what it was told
    it would read.
    """
    path = data_root / task.qas_relative_path
    if not path.is_file():
        raise FileNotFoundError(f"{task.name} has no questions at {path}")
    payload = path.read_bytes()
    rows = json.loads(payload.decode("utf-8"))
    if not isinstance(rows, list):
        raise ValueError(f"{path} holds a {type(rows).__name__}, expected a list of rows")
    if len(rows) != task.expected_rows:
        raise ValueError(
            f"{path} holds {len(rows)} rows but {task.name} is registered with "
            f"{task.expected_rows}; every sampled row index would address a different question"
        )
    for ordinal, row in enumerate(rows):
        _validate_row(path, ordinal, row)
    return TaskQuestions(rows=tuple(rows), sha256=hashlib.sha256(payload).hexdigest())


def _verify_cached_release(release: Path) -> None:
    """Re-check the cache's claim against the pin, every run, not just the one that filled it.

    The bytes are hashed once, while downloading, and never again: rehashing
    3.6 GB before every run buys little against a 32-byte record it would be
    reading anyway. What outlives that run is the cache — the pin moves, a
    directory gets edited by hand, an older copy is restored — so the claim is
    re-read rather than assumed. What a run actually read is attested one layer
    down, by the digest ``load_task_rows`` takes of the question file itself.
    """
    try:
        recorded = release_sha256(release)
    except ValueError:
        # A record that will not decode or parse names no release either.
        recorded = None
    if recorded == SPACE_DATA_SHA256:
        return
    complaint = (
        f"was unpacked from {recorded}, not the pinned {SPACE_DATA_SHA256}"
        if recorded is not None
        else f"has no readable {PROVENANCE_NAME} beside it, so nothing says which release it is"
    )
    raise RuntimeError(
        f"the cached release at {release} {complaint}; delete {release.parent} and let the "
        f"next run fetch it again, or set {SPACE_DATA_ENV}={release} to read it as it is, "
        "unverified"
    )


def _validate_row(path: Path, ordinal: int, row: object) -> None:
    if not isinstance(row, dict):
        raise ValueError(f"{path} row {ordinal} is a {type(row).__name__}, expected an object")
    question = row.get("question")
    if not isinstance(question, str) or not question:
        raise ValueError(f"{path} row {ordinal} has no question text")
    answer = row.get("answer")
    if isinstance(answer, bool) or not isinstance(answer, int):
        raise ValueError(f"{path} row {ordinal} has a non-integer answer: {answer!r}")


def _require_extraction_filtering() -> None:
    """Refuse to fetch an archive this interpreter could not unpack safely."""
    if not hasattr(tarfile, "data_filter"):
        raise RuntimeError(
            f"Python {sys.version.split()[0]} cannot filter tar extraction, so unpacking "
            "the release would follow escaping paths; upgrade to a patch release with "
            "extraction filters (3.10.12+ or 3.11.4+)"
        )


def _download(url: str, destination: Path) -> str:
    """Stream the archive to disk, hashing it on the way past."""
    digest = hashlib.sha256()
    timeout = (DOWNLOAD_CONNECT_TIMEOUT_SECONDS, DOWNLOAD_READ_TIMEOUT_SECONDS)
    with requests.get(url, stream=True, timeout=timeout) as response:
        response.raise_for_status()
        with destination.open("wb") as sink:
            for chunk in response.iter_content(chunk_size=DOWNLOAD_CHUNK_BYTES):
                digest.update(chunk)
                sink.write(chunk)
    return digest.hexdigest()


def _write_provenance(staging: Path, target: Path, digest: str) -> None:
    """Publish the record with a rename, so it is never read half-written."""
    written = staging / PROVENANCE_NAME
    written.write_text(
        json.dumps({"url": SPACE_DATA_URL, "sha256": digest}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(written, target / PROVENANCE_NAME)
