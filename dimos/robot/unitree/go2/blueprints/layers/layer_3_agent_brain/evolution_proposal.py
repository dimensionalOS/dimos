#!/usr/bin/env python3
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

from __future__ import annotations

from datetime import UTC, datetime
import json
import os
from pathlib import Path
import re
from typing import Any

SKILL_PROPOSAL_SCHEMA = "dimos.skill_proposal.v1"

_VALID_RISK_LEVELS = {"low", "medium", "high"}
_LARGE_ARTIFACT_EXTENSIONS = {
    ".avi",
    ".db",
    ".jpeg",
    ".jpg",
    ".mov",
    ".mp4",
    ".npy",
    ".npz",
    ".onnx",
    ".pkl",
    ".png",
    ".pt",
    ".sqlite",
}


def parse_skill_proposal_json(proposal_json: str) -> dict[str, Any]:
    """Parse the primitive MCP JSON argument into a proposal object."""
    try:
        proposal = json.loads(proposal_json or "{}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid proposal_json: {exc.msg}") from exc
    if not isinstance(proposal, dict):
        raise ValueError("proposal_json must decode to a JSON object")
    return proposal


def validate_skill_proposal(proposal: dict[str, Any]) -> list[str]:
    """Validate that a generated skill change remains reviewable and bounded."""
    errors: list[str] = []
    if not isinstance(proposal, dict):
        return ["proposal must be a JSON object"]

    if proposal.get("schema") != SKILL_PROPOSAL_SCHEMA:
        errors.append(f"schema must be {SKILL_PROPOSAL_SCHEMA}")
    if not str(proposal.get("proposal_id") or "").strip():
        errors.append("proposal_id is required")
    if not str(proposal.get("title") or "").strip():
        errors.append("title is required")
    if proposal.get("requires_human_review") is not True:
        errors.append("requires_human_review must be true")
    if proposal.get("risk_level") not in _VALID_RISK_LEVELS:
        errors.append("risk_level must be low, medium, or high")

    target_files = proposal.get("target_files")
    if not isinstance(target_files, list) or not target_files:
        errors.append("target_files must be a non-empty list")
    else:
        for index, target in enumerate(target_files):
            errors.extend(_validate_target_file(index, target))

    generated_patch = proposal.get("generated_patch", "")
    if generated_patch is not None and not isinstance(generated_patch, str):
        errors.append("generated_patch must be a string")
    return errors


def normalize_skill_proposal(
    proposal: dict[str, Any],
    commit_requested: bool = False,
    commit_sha: str = "",
    created_at: float | None = None,
) -> dict[str, Any]:
    """Validate and normalize a proposal before writing it to the ledger."""
    errors = validate_skill_proposal(proposal)
    if errors:
        raise ValueError("; ".join(errors))

    normalized = dict(proposal)
    normalized["proposal_id"] = str(normalized["proposal_id"]).strip()
    normalized["title"] = str(normalized["title"]).strip()
    normalized["summary"] = str(normalized.get("summary") or "").strip()
    normalized["target_files"] = [str(target).strip() for target in normalized["target_files"]]
    normalized["requires_human_review"] = True
    normalized.setdefault(
        "created_at",
        round(created_at if created_at is not None else datetime.now(UTC).timestamp(), 3),
    )
    normalized["git"] = {
        "commit_requested": commit_requested,
        "commit_sha": commit_sha,
    }
    return normalized


def write_skill_proposal_file(
    ledger_dir: Path,
    proposal: dict[str, Any],
    commit_requested: bool = False,
) -> Path:
    """Write a validated proposal under `.dimos/evolution/proposals`."""
    normalized = normalize_skill_proposal(proposal, commit_requested=commit_requested)
    proposal_dir = ledger_dir / "proposals"
    proposal_dir.mkdir(parents=True, exist_ok=True)
    _ensure_readme(ledger_dir)

    path = _unique_proposal_path(proposal_dir, normalized["proposal_id"])
    payload = json.dumps(normalized, indent=2, sort_keys=True) + "\n"
    tmp_path = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    tmp_path.write_text(payload, encoding="utf-8")
    tmp_path.replace(path)
    return path


def _validate_target_file(index: int, target: object) -> list[str]:
    errors: list[str] = []
    if not isinstance(target, str) or not target.strip():
        return [f"target_files[{index}] must be a non-empty string"]

    target_path = target.strip().replace("\\", "/")
    parts = [part for part in target_path.split("/") if part]
    suffix = Path(target_path).suffix.casefold()

    if Path(target_path).is_absolute():
        errors.append(f"target_files[{index}] must be a relative path")
    if ".." in parts:
        errors.append(f"target_files[{index}] must not contain '..'")
    if target_path.startswith(".dimos/evolution/events/"):
        errors.append(f"target_files[{index}] must not target evolution event logs")
    if suffix in _LARGE_ARTIFACT_EXTENSIONS:
        errors.append(f"target_files[{index}] must not target large binary artifacts")
    return errors


def _ensure_readme(ledger_dir: Path) -> None:
    readme = ledger_dir / "README.md"
    if readme.exists():
        return
    readme.write_text(
        "# DimOS Evolution Ledger\n\n"
        "This directory stores low-volume agent evolution events and human-reviewable "
        "proposals. It should not contain raw images, embeddings, database files, or "
        "large robot logs.\n",
        encoding="utf-8",
    )


def _safe_filename(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._-") or "proposal"


def _unique_proposal_path(proposal_dir: Path, proposal_id: str) -> Path:
    """Return a non-colliding proposal path, appending -1, -2, ... when needed.

    A reused stable ``proposal_id`` (e.g. ``improve_navigation``) must not
    silently overwrite an earlier review artifact. The JSON keeps the
    author's ``proposal_id`` verbatim; only the filename is disambiguated.
    """
    stem = _safe_filename(str(proposal_id))
    path = proposal_dir / f"{stem}.json"
    suffix = 1
    while path.exists():
        path = proposal_dir / f"{stem}-{suffix}.json"
        suffix += 1
    return path


__all__ = [
    "SKILL_PROPOSAL_SCHEMA",
    "normalize_skill_proposal",
    "parse_skill_proposal_json",
    "validate_skill_proposal",
    "write_skill_proposal_file",
]
