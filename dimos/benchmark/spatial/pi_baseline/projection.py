# Copyright 2026 Dimensional Inc.
"""Canonical public ``case.v1.json`` projection."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
from typing import TypeAlias, cast, overload
import uuid

from dimos.benchmark.spatial.corpus_loader import SpatialCorpusInstance
from dimos.benchmark.spatial.pi_baseline.records import StagingRecord
from dimos.benchmark.spatial.pi_baseline.topology import (
    PinnedDirectory,
    TopologyError,
    fresh_directory_cursor,
)
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, validate_relative_path

CaseV1: TypeAlias = dict[str, JsonValue]
_FORBIDDEN = {
    "oracle",
    "room_id",
    "source_scene_key",
    "source_artifact_sha256",
    "answer",
    "topology",
    "geometry",
}


class DescriptorMismatchError(ValueError):
    """Safe metadata for a staging descriptor-set mismatch."""

    def __init__(self, expected_entry_count: int, actual_entry_count: int, actual_entries_sha256: str) -> None:
        super().__init__()
        self.expected_entry_count = expected_entry_count
        self.actual_entry_count = actual_entry_count
        self.actual_entries_sha256 = actual_entries_sha256


def project_case(item: SpatialCorpusInstance) -> CaseV1:
    """Return only selected public records and release identity."""
    payload: CaseV1 = {
        "record_type": "case",
        "schema_version": "1.0",
        "release": _release_identity(item),
        "scene": item.scene.model_dump(mode="json"),
        "trajectory": item.trajectory.model_dump(mode="json"),
        "question": item.question.model_dump(mode="json"),
        "snapshot": item.snapshot.model_dump(mode="json"),
        "instance": item.instance.model_dump(mode="json"),
    }
    _reject_private(payload)
    return payload


def write_case(path: Path, item: SpatialCorpusInstance) -> str:
    validate_relative_path(path.name)
    data = canonical_json(project_case(item)) + b"\n"
    path.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


@overload
def stage_public_instance(
    corpus_root: Path,
    staging_parent: Path,
    *,
    scene_id: str,
    trajectory_id: str,
    question_id: str,
    variant: str,
    instance_id: str,
) -> Path: ...


@overload
def stage_public_instance(
    corpus_root: Path,
    staging_parent: PinnedDirectory,
    *,
    destination_name: str | None = None,
    scene_id: str,
    trajectory_id: str,
    question_id: str,
    variant: str,
    instance_id: str,
) -> PinnedDirectory: ...


def stage_public_instance(
    corpus_root: Path,
    staging_parent: Path | PinnedDirectory,
    *,
    destination_name: str | None = None,
    scene_id: str,
    trajectory_id: str,
    question_id: str,
    variant: str,
    instance_id: str,
) -> Path | PinnedDirectory:
    """Create one exclusive, minimal staging directory for an exact public instance."""
    from dimos.benchmark.spatial.pi_baseline.resolver import resolve_public_instance

    item = resolve_public_instance(
        corpus_root,
        scene_id=scene_id,
        trajectory_id=trajectory_id,
        question_id=question_id,
        variant=variant,
        instance_id=instance_id,
    )
    public_root = item.public_root
    source_map = item.variant_root / item.snapshot.map_artifact_path
    if source_map.is_symlink() or public_root not in source_map.parents or not source_map.is_file():
        raise ValueError("map artifact is not a regular file inside public corpus")
    legacy = isinstance(staging_parent, Path)
    parent = PinnedDirectory.open(staging_parent, create=False) if legacy else staging_parent
    name = destination_name or f"pi-public-{uuid.uuid4().hex}"
    child: PinnedDirectory | None = None
    try:
        parent.verify()
        os.mkdir(name, dir_fd=parent.fd)
        child = PinnedDirectory.open_at(parent, name)
        child.mkdir("cases")
        child.mkdir("maps")
        case_data = canonical_json(project_case(item)) + b"\n"
        cases = PinnedDirectory.open_at(child, "cases")
        try:
            cases.write_bytes("case.v1.json", case_data)
        finally:
            cases.close()
        case_hash = hashlib.sha256(case_data).hexdigest()
        map_data = _read_source(source_map)
        maps = PinnedDirectory.open_at(child, "maps")
        try:
            maps.write_bytes("map.lcm", map_data)
        finally:
            maps.close()
        map_hash = hashlib.sha256(map_data).hexdigest()
        schema_source = Path(__file__).parents[4] / "packages/pi-spatial-adapter/src/tool-definitions.v1.json"
        schema_data = schema_source.read_bytes()
        child.write_bytes("schema.v1.json", schema_data)
        schema_hash = hashlib.sha256(schema_data).hexdigest()
        release = _release_identity(item)
        provenance: dict[str, JsonValue] = {
            "record_type": "pi-provenance",
            "schema_version": "1.0",
            "release_id": release["release_id"],
            "release_version": release["release_version"],
            "case_sha256": case_hash,
            "map_sha256": map_hash,
            "schema_sha256": schema_hash,
        }
        manifest: dict[str, JsonValue] = {
            "record_type": "pi-staging-manifest",
            "schema_version": "1.0",
            "identity": {
                "scene_id": scene_id,
                "trajectory_id": trajectory_id,
                "question_id": question_id,
                "variant": variant,
                "instance_id": instance_id,
            },
            "release": release,
            "provenance": provenance,
        }
        _write_json(child, "provenance.v1.json", provenance)
        _write_json(child, "staging-manifest.v1.json", manifest)
        files = []
        for relative in (
            "cases/case.v1.json",
            "maps/map.lcm",
            "schema.v1.json",
            "staging-manifest.v1.json",
            "provenance.v1.json",
        ):
            files.append(
                {
                    "path": relative,
                    "sha256": hashlib.sha256(_read_owned(child, relative)).hexdigest(),
                }
            )
        _write_json(
            child,
            "inventory.v1.json",
            {"record_type": "pi-inventory", "schema_version": "1.0", "files": files},
        )
        # The record is checked here as well as by callers, while preserving the existing API.
        _verify_staging_descriptor(
            child,
            StagingRecord(
                case_path="cases/case.v1.json",
                map_path="maps/map.lcm",
                map_sha256=map_hash,
                schema_sha256=schema_hash,
            ),
        )
        child.verify()
        result: Path | PinnedDirectory = child
        if legacy:
            result = parent.path / name
            child.close()
        return result
    except Exception:
        if child is not None:
            _remove_tree(parent, name, child)
        raise
    finally:
        if legacy:
            parent.close()


def _write_json(directory: PinnedDirectory, name: str, payload: dict[str, JsonValue]) -> None:
    directory.write_bytes(name, canonical_json(cast("JsonValue", payload)) + b"\n")


def _read_source(path: Path) -> bytes:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise TopologyError("source map is not a regular file")
        return os.read(fd, info.st_size)
    finally:
        os.close(fd)


def _read_owned(directory: PinnedDirectory, relative: str) -> bytes:
    return directory.read_relative(relative)


def _verify_staging_descriptor(root: PinnedDirectory, staging: StagingRecord) -> None:
    root.verify()
    expected = {
        "cases/case.v1.json",
        "maps/map.lcm",
        "schema.v1.json",
        "staging-manifest.v1.json",
        "inventory.v1.json",
        "provenance.v1.json",
    }
    for relative in (staging.case_path, staging.map_path, staging.schema_path):
        root.read_relative(relative)
    if hashlib.sha256(root.read_relative(staging.map_path)).hexdigest() != staging.map_sha256:
        raise ValueError("staged map hash does not match its record")
    case = json.loads(root.read_relative(staging.case_path))
    manifest = json.loads(root.read_relative("staging-manifest.v1.json"))
    provenance = json.loads(root.read_relative("provenance.v1.json"))
    inventory = json.loads(root.read_relative("inventory.v1.json"))
    if not isinstance(case, dict) or not isinstance(manifest, dict) or not isinstance(provenance, dict):
        raise ValueError("invalid staging metadata")
    if manifest.get("record_type") != "pi-staging-manifest" or provenance.get("record_type") != "pi-provenance":
        raise ValueError("invalid staging metadata")
    if manifest.get("provenance") != provenance or provenance.get("map_sha256") != staging.map_sha256:
        raise ValueError("staging provenance does not match map")
    if provenance.get("case_sha256") != hashlib.sha256(root.read_relative(staging.case_path)).hexdigest():
        raise ValueError("staging provenance does not match case")
    if provenance.get("schema_sha256") != staging.schema_sha256 or hashlib.sha256(
        root.read_relative(staging.schema_path)
    ).hexdigest() != staging.schema_sha256:
        raise ValueError("staging schema does not match its record")
    entries = inventory.get("files") if isinstance(inventory, dict) else None
    if not isinstance(entries, list) or {entry.get("path") for entry in entries} != expected - {"inventory.v1.json"}:
        raise ValueError("staged inventory is incomplete or contains private artifacts")
    for entry in entries:
        relative = str(entry["path"])
        if hashlib.sha256(root.read_relative(relative)).hexdigest() != entry.get("sha256"):
            raise ValueError(f"staged inventory hash mismatch: {relative}")
    actual = _descriptor_files(root.fd)
    if actual != expected:
        actual_encoding = canonical_json(cast("JsonValue", sorted(actual)))
        raise DescriptorMismatchError(
            expected_entry_count=len(expected),
            actual_entry_count=len(actual),
            actual_entries_sha256=hashlib.sha256(actual_encoding).hexdigest(),
        )
    root.verify()


def _descriptor_files(fd: int, prefix: str = "") -> set[str]:
    result: set[str] = set()
    with fresh_directory_cursor(fd) as cursor_fd:
        with os.scandir(cursor_fd) as entries:
            for entry in entries:
                relative = f"{prefix}{entry.name}"
                info = os.stat(entry.name, dir_fd=cursor_fd, follow_symlinks=False)
                if stat.S_ISDIR(info.st_mode):
                    child = os.open(
                        entry.name,
                        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                        dir_fd=cursor_fd,
                    )
                    try:
                        result.update(_descriptor_files(child, relative + "/"))
                    finally:
                        os.close(child)
                elif stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
                    result.add(relative)
                else:
                    raise ValueError("staging contains an unsupported entry")
    return result


def _remove_tree(parent: PinnedDirectory, name: str, child: PinnedDirectory) -> None:
    """Remove a failed staging child using only retained directory descriptors."""
    try:
        with fresh_directory_cursor(child.fd) as cursor_fd:
            with os.scandir(cursor_fd) as entries:
                for entry in entries:
                    info = os.stat(entry.name, dir_fd=cursor_fd, follow_symlinks=False)
                    if stat.S_ISDIR(info.st_mode):
                        nested = PinnedDirectory.open_at(child, entry.name)
                        try:
                            _remove_tree(child, entry.name, nested)
                        finally:
                            nested.close()
                    else:
                        os.unlink(entry.name, dir_fd=child.fd)
        child.close()
        os.rmdir(name, dir_fd=parent.fd)
    except OSError:
        child.close()


def _release_identity(item: SpatialCorpusInstance) -> dict[str, JsonValue]:
    manifest = json.loads((item.corpus_root / "manifest.json").read_text())
    return {
        "release_id": str(manifest["release_id"]),
        "release_version": str(manifest["release_version"]),
        "generator_revision": str(manifest["generator_revision"]),
        "mapper_configuration_digest": str(manifest["mapper_configuration_digest"]),
        "source_dataset_revision": str(manifest["source_dataset_revision"]),
    }


def _reject_private(value: object) -> None:
    if isinstance(value, dict):
        if _FORBIDDEN.intersection(value):
            raise ValueError("case projection contains a forbidden/private field")
        for child in value.values():
            _reject_private(child)
    elif isinstance(value, list):
        for child in value:
            _reject_private(child)
