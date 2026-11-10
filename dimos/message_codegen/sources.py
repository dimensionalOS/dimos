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

"""Provenance for the committed built-in message definitions.

``packages/dimos-messages/interfaces`` is the only message input of a build.
Those files are committed, so checkouts, sdists and editable installs never
download schemas. ``schema-sources.json`` records where each upstream file came
from: the repository revision, the archive hash and the hash of every vendored
file. ``verify()`` re-checks the committed bytes against that record without
network access. ``vendor()`` is the explicit maintainer refresh that downloads
the pinned archives and rewrites the upstream files; definitions owned by DimOS
(``interfaces/dimos_msgs``) are edited in place and never downloaded.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
from io import BytesIO
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import tarfile
from tempfile import TemporaryDirectory

PROJECT = Path(__file__).resolve().parents[2] / "packages/dimos-messages"
INTERFACES = PROJECT / "interfaces"
LOCK = PROJECT / "schema-sources.json"

# Message definition packages authored in this repository: not vendored.
LOCAL = ("control_msgs", "dimos_msgs")

_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _relative(value: str) -> str:
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or ".." in path.parts or value.endswith("/"):
        raise ValueError(f"Unsafe message input path: {value}")
    return value


def load(project: Path = PROJECT) -> dict:
    """Read and validate the provenance record."""
    lock = json.loads((project / "schema-sources.json").read_text())
    if lock.get("format") != 1 or not isinstance(lock.get("sources"), list):
        raise ValueError("schema-sources.json must be a format 1 source list")
    for source in lock["sources"]:
        if set(source) != {"repository", "revision", "url", "sha256", "prefix", "files"}:
            raise ValueError(f"Unexpected source fields: {sorted(source)}")
        if not isinstance(source["url"], str) or not isinstance(source["repository"], str):
            raise ValueError("Each source requires a repository and url")
        if not isinstance(source["revision"], str) or not source["revision"]:
            raise ValueError("Each source requires a revision")
        if not _SHA256.fullmatch(source["sha256"]):
            raise ValueError(f"Invalid archive hash for {source['repository']}")
        _relative(source["prefix"] + "archive-member")
        if not source["files"]:
            raise ValueError(f"No files pinned for {source['repository']}")
        for destination, entry in source["files"].items():
            _relative(destination)
            if destination.split("/")[0] in LOCAL:
                raise ValueError(f"Upstream source owns a local definition: {destination}")
            if set(entry) != {"path", "sha256"}:
                raise ValueError(f"Unexpected file fields for {destination}")
            _relative(entry["path"])
            if not _SHA256.fullmatch(entry["sha256"]):
                raise ValueError(f"Invalid file hash for {destination}")
    return lock


def pinned(lock: dict) -> dict[str, str]:
    """Committed path -> expected upstream hash."""
    return {
        destination: entry["sha256"]
        for source in lock["sources"]
        for destination, entry in source["files"].items()
    }


def local_files(project: Path = PROJECT) -> set[str]:
    """Committed paths authored in this repository."""
    interfaces = project / "interfaces"
    return {
        str(path.relative_to(interfaces))
        for name in LOCAL
        for path in (interfaces / name).rglob("*.msg")
        if path.is_file()
    }


def local_licenses(project: Path = PROJECT) -> set[str]:
    """License files required for the authored definition packages present."""
    interfaces = project / "interfaces"
    return {f"licenses/{name}-LICENSE" for name in LOCAL if (interfaces / name).is_dir()}


def verify(project: Path = PROJECT) -> list[str]:
    """Check the committed inputs against the provenance record; return problems."""
    lock = load(project)
    expected = pinned(lock)
    interfaces = project / "interfaces"
    problems = []
    for destination, digest in sorted(expected.items()):
        path = interfaces / destination
        if not path.is_file():
            problems.append(f"missing vendored definition: {destination}")
        elif sha256(path.read_bytes()).hexdigest() != digest:
            problems.append(f"vendored definition does not match its pin: {destination}")
    local = local_files(project)
    if not local:
        problems.append(f"no {', '.join(LOCAL)} definitions found")
    licenses = local_licenses(project)
    for license_file in sorted(licenses):
        if not (interfaces / license_file).is_file():
            problems.append(f"missing license: {license_file}")
    allowed = set(expected) | local | licenses
    for path in sorted(interfaces.rglob("*")):
        if path.is_file():
            relative = str(path.relative_to(interfaces))
            if relative not in allowed:
                problems.append(f"unexpected message input: {relative}")
    return problems


def _archive(source: dict, cache: Path) -> bytes:
    target = cache / (source["sha256"] + ".tar.gz")
    if target.is_file():
        data = target.read_bytes()
    else:
        from requests import get

        response = get(source["url"], timeout=120)
        response.raise_for_status()
        data = response.content
        cache.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    actual = sha256(data).hexdigest()
    if actual != source["sha256"]:
        raise ValueError(f"Archive hash mismatch for {source['repository']}: {actual}")
    return data


def _staged(source: dict, data: bytes, stage: Path) -> None:
    with tarfile.open(fileobj=BytesIO(data), mode="r:gz") as archive:
        for destination, entry in sorted(source["files"].items()):
            member = archive.extractfile(source["prefix"] + entry["path"])
            if member is None:
                raise ValueError(f"Archive lacks {entry['path']} in {source['repository']}")
            contents = member.read()
            actual = sha256(contents).hexdigest()
            if actual != entry["sha256"]:
                raise ValueError(f"Definition hash mismatch for {destination}: {actual}")
            target = stage / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(contents)


def vendor(project: Path = PROJECT, cache: Path | None = None) -> None:
    """Rewrite the vendored upstream definitions from their pinned archives."""
    lock = load(project)
    interfaces = project / "interfaces"
    cache = cache or Path("build/builtin-source-cache/archives")
    with TemporaryDirectory(prefix=".vendor-") as directory:
        stage = Path(directory) / "inputs"
        for source in lock["sources"]:
            _staged(source, _archive(source, cache), stage)
        expected = pinned(lock)
        # Everything verified before the committed inputs are touched.
        interfaces.mkdir(parents=True, exist_ok=True)
        for destination in expected:
            target = interfaces / destination
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(stage / destination, target)
        for path in sorted(interfaces.rglob("*"), reverse=True):
            relative = str(path.relative_to(interfaces))
            if (
                path.is_file()
                and relative not in expected
                and relative.split("/")[0] not in LOCAL
                and relative not in local_licenses(project)
            ):
                path.unlink()
            elif path.is_dir() and not any(path.iterdir()):
                path.rmdir()
    problems = verify(project)
    if problems:
        raise ValueError("Vendored inputs are inconsistent: " + "; ".join(problems))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--update",
        action="store_true",
        help="download the pinned archives and rewrite the vendored definitions",
    )
    parser.add_argument("--cache", type=Path, default=None, help="archive cache directory")
    arguments = parser.parse_args()
    if arguments.update:
        vendor(cache=arguments.cache)
        print("Vendored definitions updated and verified.")
        return
    problems = verify()
    if problems:
        raise SystemExit("\n".join(problems))
    print("Committed definitions match the provenance record.")


if __name__ == "__main__":
    main()
