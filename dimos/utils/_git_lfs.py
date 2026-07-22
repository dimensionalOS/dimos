# Copyright 2025-2026 Dimensional Inc.
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

import json
from pathlib import Path
import re
import subprocess

from pydantic import BaseModel, ValidationError


class _LFSFileEntry(BaseModel):
    name: str
    oid_type: str
    oid: str


class _LFSFilesResponse(BaseModel):
    files: list[_LFSFileEntry]


def get_committed_file_sha256(path: Path, repo_root: Path) -> str:
    """Return the committed Git LFS SHA256 OID for a repository file."""
    try:
        relative_name = path.relative_to(repo_root).as_posix()
    except ValueError as error:
        raise RuntimeError(
            f"Cannot look up committed Git LFS file for {path}: path is outside repository {repo_root}"
        ) from error

    command = [
        "git",
        "lfs",
        "ls-files",
        "--json",
        f"--include={relative_name}",
        "HEAD",
    ]
    try:
        result = subprocess.run(
            command,
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, OSError) as error:
        detail = getattr(error, "stderr", None) or str(error)
        raise RuntimeError(
            f"Failed to list committed Git LFS file for {path} with {' '.join(command)}: {detail}"
        ) from error

    try:
        response = _LFSFilesResponse.model_validate_json(result.stdout)
    except (ValidationError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Malformed Git LFS JSON response for {path}: {error}") from error

    if len(response.files) != 1:
        raise RuntimeError(
            f"Expected exactly one committed Git LFS entry for {relative_name}, found {len(response.files)}"
        )

    entry = response.files[0]
    if entry.name != relative_name:
        raise RuntimeError(f"Git LFS entry name mismatch for {relative_name}: got {entry.name}")
    if entry.oid_type != "sha256":
        raise RuntimeError(
            f"Git LFS entry for {relative_name} has unsupported oid_type {entry.oid_type!r}"
        )
    if re.fullmatch(r"[0-9a-f]{64}", entry.oid) is None:
        raise RuntimeError(f"Git LFS entry for {relative_name} has invalid sha256 oid")

    return entry.oid
