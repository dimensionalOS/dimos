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

"""Explicit-root recording migration planning and batch execution; never downloads."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterator
import json
import os
from pathlib import Path
from typing import Any

from dimos.memory.convert_recording import convert, read_mcap, read_sqlite
from dimos.memory.utils.validation import validate_identifier

_INPUTS = {".mcap", ".db", ".sqlite"}
_RECORDINGS = _INPUTS | {".pickle", ".pkl", ".pcap", ".lcm"}


def inspect_recording(source: Path, output_format: str) -> dict[str, Any]:
    """Read declarations and count rows; no dynamic registry imports or payload execution."""
    if output_format not in {"mcap", "db"}:
        raise ValueError("Format must be mcap or db")
    if source.is_symlink():
        raise ValueError("Symlink recordings are not followed; provide their explicit source root")
    if source.suffix not in _INPUTS:
        raise ValueError(
            "Not an expanded MCAP/memory SQLite recording; archives, pickle and raw captures need explicit migration"
        )
    with source.open("rb") as file:
        if file.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError(
                "LFS pointer, not recording bytes; materialize it with the project data loader first"
            )
    reader = (
        read_mcap(source)
        if source.suffix == ".mcap"
        else read_sqlite(source, allow_vectors=output_format == "db")
    )
    with reader as (streams, rows):
        if not streams:
            raise ValueError("Recording has no declared streams")
        counts: Counter[str] = Counter({stream.name: 0 for stream in streams})
        types = {}
        for stream in streams:
            if output_format == "db":
                validate_identifier(stream.name)
            types[stream.name] = {
                "input_codec": stream.codec,
                "output_type": stream.target.__msgtype__,
            }
        for row in rows:
            counts[row.stream.name] += 1
    return {"counts": dict(counts), "total": sum(counts.values()), "types": types}


def _candidates(root: Path) -> Iterator[Path]:
    # os.walk does not follow directory symlinks. Explicitly report them rather than
    # traversing data outside the requested root or silently declaring it converted.
    def fail_walk(error: OSError) -> None:
        raise error

    for directory, subdirs, files in os.walk(root, followlinks=False, onerror=fail_walk):
        for name in sorted(subdirs):
            path = Path(directory) / name
            if path.is_symlink():
                yield path
        for name in sorted(files):
            path = Path(directory) / name
            uncompressed = name.removesuffix(".tar.gz")
            if (
                path.suffix in _RECORDINGS
                or Path(uncompressed).suffix in _RECORDINGS
                or name.endswith((".tar.gz", ".zip", ".tar", ".tgz"))
            ):
                yield path


def plan(source: Path, destination: Path, output_format: str = "mcap") -> dict[str, Any]:
    """Inventory only the explicitly supplied recording root, preserving relative paths."""
    if output_format not in {"mcap", "db"}:
        raise ValueError("Format must be mcap or db")
    source, destination = source.absolute(), destination.absolute()
    if not source.is_dir() or source.is_symlink():
        raise ValueError("Batch source must be an explicit real directory")
    source_resolved, output_resolved = source.resolve(), destination.resolve()
    if (
        source_resolved in output_resolved.parents
        or output_resolved in source_resolved.parents
        or source_resolved == output_resolved
    ):
        raise ValueError("Input and output directory trees must not overlap")
    if destination.exists() or destination.is_symlink():
        raise FileExistsError("Batch destination must be a new directory")
    entries: list[dict[str, Any]] = []
    destinations: set[str] = set()
    for path in sorted(_candidates(source)):
        relative = path.relative_to(source)
        # Keep the original suffix to avoid collisions between run.db and run.mcap.
        output = destination / relative.parent / (relative.name + ".cdr." + output_format)
        entry: dict[str, Any] = {"input": str(path), "output": str(output), "status": "ready"}
        try:
            if str(output) in destinations:
                raise ValueError("Duplicate destination")
            destinations.add(str(output))
            entry.update(inspect_recording(path, output_format))
        except Exception as exc:
            entry.update(status="blocked", reason=str(exc))
        entries.append(entry)
    if not entries:
        raise ValueError("No recognized recording files found in the supplied directory")
    return {
        "source": str(source),
        "destination": str(destination),
        "format": output_format,
        "preflight": "metadata, schema and row counts; payload integrity is checked during conversion",
        "files": entries,
        "blocked": sum(e["status"] == "blocked" for e in entries),
    }


def migrate(
    source: Path, destination: Path, *, output_format: str = "mcap", dry_run: bool = False
) -> dict[str, Any]:
    """Preflight every candidate before publishing any batch member; report every result."""
    result = plan(source, destination, output_format)
    if dry_run or result["blocked"]:
        return result
    destination.mkdir(parents=False, exist_ok=False)
    for entry in result["files"]:
        entry["status"] = "not_attempted"
    try:
        for entry in result["files"]:
            output = Path(entry["output"])
            output.parent.mkdir(parents=True, exist_ok=True)
            try:
                converted = convert(Path(entry["input"]), output)
                entry.update(status="converted", conversion=converted)
            except Exception as exc:
                entry.update(status="failed", reason=str(exc))
                result["blocked"] += 1
                break
    finally:
        # If a later file changes/fails after preflight, successes remain explicitly
        # listed; exit is nonzero and no caller can mistake the partial batch for done.
        with (destination / "migration-summary.json").open("x") as file:
            json.dump(result, file, indent=2)
    return result
