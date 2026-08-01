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

# Copyright 2026 Dimensional Inc.
"""Safe staging and byte-integrity checks for baseline inputs."""

from __future__ import annotations

import json
from pathlib import Path

from dimos.benchmark.spatial.pi_baseline.records import StagingRecord
from dimos.benchmark.spatial.utilities import hash_file_sha256

_MANIFEST = "staging-manifest.v1.json"
_INVENTORY = "inventory.v1.json"
_PROVENANCE = "provenance.v1.json"
_ALLOWED = {
    "cases/case.v1.json",
    "maps/map.lcm",
    "schema.v1.json",
    _MANIFEST,
    _INVENTORY,
    _PROVENANCE,
}


def verify_staging(root: Path, staging: StagingRecord) -> tuple[Path, Path]:
    """Verify every byte and provenance field in a self-contained staging area."""
    root = root.resolve()
    if root.is_symlink() or not root.is_dir():
        raise ValueError("staging root must be a real directory")
    case_path = (root / staging.case_path).resolve()
    map_path = (root / staging.map_path).resolve()
    schema_path = (root / staging.schema_path).resolve()
    for path in (case_path, map_path, schema_path):
        if path.is_symlink() or (path != root and root not in path.parents):
            raise ValueError("staged path escapes its root")
        if not path.is_file():
            raise ValueError(f"staged file does not exist: {path}")
    if hash_file_sha256(map_path) != staging.map_sha256:
        raise ValueError("staged map hash does not match its record")
    manifest = _read_json(root / _MANIFEST)
    inventory = _read_json(root / _INVENTORY)
    provenance = _read_json(root / _PROVENANCE)
    if (
        manifest.get("record_type") != "pi-staging-manifest"
        or manifest.get("schema_version") != "1.0"
    ):
        raise ValueError("invalid staging manifest")
    if (
        provenance.get("record_type") != "pi-provenance"
        or provenance.get("map_sha256") != staging.map_sha256
    ):
        raise ValueError("staging provenance does not match map")
    if manifest.get("provenance") != provenance:
        raise ValueError("staging manifest provenance does not match provenance record")
    if provenance.get("case_sha256") != hash_file_sha256(case_path):
        raise ValueError("staging provenance does not match case")
    if (
        not schema_path.is_file()
        or hash_file_sha256(schema_path) != staging.schema_sha256
        or provenance.get("schema_sha256") != staging.schema_sha256
    ):
        raise ValueError("staging schema does not match its record")
    entries = inventory.get("files")
    if not isinstance(entries, list) or {entry.get("path") for entry in entries} != _ALLOWED - {
        _INVENTORY
    }:
        raise ValueError("staged inventory is incomplete or contains private artifacts")
    for entry in entries:
        path = root / str(entry["path"])
        if path.is_symlink() or not path.is_file() or hash_file_sha256(path) != entry.get("sha256"):
            raise ValueError(f"staged inventory hash mismatch: {entry.get('path')}")
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() or path.is_symlink()
    }
    if actual != _ALLOWED:
        raise ValueError("staging contains unexpected files")
    return case_path, map_path


def _read_json(path: Path) -> dict[str, object]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"missing staging metadata: {path.name}")
    value = json.loads(path.read_bytes())
    if not isinstance(value, dict):
        raise ValueError(f"invalid staging metadata: {path.name}")
    return value
