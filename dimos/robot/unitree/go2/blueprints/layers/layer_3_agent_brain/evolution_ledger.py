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

from pathlib import Path
import subprocess
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_event import (
    build_evolution_event,
    parse_payload_json,
    resolve_evolution_ledger_dir,
    write_evolution_event_file,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_proposal import (
    parse_skill_proposal_json,
    write_skill_proposal_file,
)
from dimos.spec.utils import Spec


class EvolutionLedgerSpec(Spec, Protocol):
    """RPC surface for Layer 3 modules that write low-volume evolution events."""

    def write_evolution_event(
        self,
        event_type: str,
        task: str = "",
        payload: dict[str, Any] | None = None,
        commit: bool = False,
    ) -> dict[str, Any]: ...

    def record_skill_proposal(self, proposal_json: str, commit: bool = False) -> SkillResult: ...

    def write_skill_proposal(
        self,
        proposal: dict[str, Any],
        commit: bool = False,
    ) -> dict[str, Any]: ...


class _Go2EvolutionLedger(Module):
    """Git-backed audit sink for low-volume Go2 agent evolution events."""

    @skill
    def record_evolution_event(
        self,
        event_type: str,
        task: str = "",
        payload_json: str = "{}",
        commit: bool = False,
    ) -> SkillResult:
        """Record a low-volume agent evolution event to the local ledger.

        Args:
            event_type: Event category, such as task_feasibility or context_feedback.
            task: Optional user task or situation that produced the event.
            payload_json: JSON object containing the event details.
            commit: Whether to create a local Git commit containing only this event file.
        """
        try:
            payload = parse_payload_json(payload_json)
        except ValueError as exc:
            return SkillResult.fail("INVALID_INPUT", str(exc))

        try:
            record = self.write_evolution_event(
                event_type=event_type,
                task=task,
                payload=payload,
                commit=commit,
            )
        except ValueError as exc:
            return SkillResult.fail("INVALID_INPUT", str(exc))
        except Exception as exc:
            return SkillResult.fail("EXECUTION_FAILED", f"failed to record event: {exc}")

        warning_count = len(record["warnings"])
        message = f"Recorded evolution event {record['event_type']}"
        if warning_count:
            message += f" with {warning_count} warning(s)"
        return SkillResult.ok(message, **record)

    @skill
    def record_skill_proposal(
        self,
        proposal_json: str = "{}",
        commit: bool = False,
    ) -> SkillResult:
        """Record a human-reviewable skill or interface proposal without applying it.

        Args:
            proposal_json: JSON object using dimos.skill_proposal.v1.
            commit: Whether to create a local Git commit containing only this proposal file.
        """
        try:
            proposal = parse_skill_proposal_json(proposal_json)
        except ValueError as exc:
            return SkillResult.fail("INVALID_INPUT", str(exc))

        try:
            record = self.write_skill_proposal(proposal=proposal, commit=commit)
        except ValueError as exc:
            return SkillResult.fail("INVALID_INPUT", str(exc))
        except Exception as exc:
            return SkillResult.fail("EXECUTION_FAILED", f"failed to record proposal: {exc}")

        warning_count = len(record["warnings"])
        message = f"Recorded skill proposal {record['proposal_id']}"
        if warning_count:
            message += f" with {warning_count} warning(s)"
        return SkillResult.ok(
            message,
            **record,
        )

    @rpc
    def write_evolution_event(
        self,
        event_type: str,
        task: str = "",
        payload: dict[str, Any] | None = None,
        commit: bool = False,
    ) -> dict[str, Any]:
        """Write one event to the ledger and optionally commit only that file."""
        event_payload = payload or {}
        event = build_evolution_event(
            event_type=event_type,
            task=task,
            payload=event_payload,
            commit_requested=commit,
        )
        ledger_dir = resolve_evolution_ledger_dir()
        event_path = write_evolution_event_file(ledger_dir=ledger_dir, event=event)

        warnings: list[str] = []
        commit_sha = ""
        if commit:
            commit_sha = _commit_event_file(
                ledger_dir=ledger_dir,
                event_path=event_path,
                event_type=event.event_type,
                warnings=warnings,
            )

        return {
            "schema": event.to_dict()["schema"],
            "event_type": event.event_type,
            "task": event.task,
            "ledger_dir": str(ledger_dir),
            "event_path": str(event_path),
            "commit_requested": commit,
            "commit_sha": commit_sha,
            "warnings": warnings,
            "event": event.to_dict(),
        }

    @rpc
    def write_skill_proposal(
        self,
        proposal: dict[str, Any],
        commit: bool = False,
    ) -> dict[str, Any]:
        """Write one validated skill proposal and optionally commit only that file."""
        ledger_dir = resolve_evolution_ledger_dir()
        proposal_path = write_skill_proposal_file(
            ledger_dir=ledger_dir,
            proposal=proposal,
            commit_requested=commit,
        )
        proposal_data = parse_skill_proposal_json(proposal_path.read_text(encoding="utf-8"))

        warnings: list[str] = []
        commit_sha = ""
        if commit:
            commit_sha = _commit_event_file(
                ledger_dir=ledger_dir,
                event_path=proposal_path,
                event_type="skill_proposal",
                warnings=warnings,
            )

        return {
            "schema": proposal_data["schema"],
            "proposal_id": proposal_data["proposal_id"],
            "ledger_dir": str(ledger_dir),
            "proposal_path": str(proposal_path),
            "commit_requested": commit,
            "commit_sha": commit_sha,
            "warnings": warnings,
            "proposal": proposal_data,
        }


def _commit_event_file(
    ledger_dir: Path,
    event_path: Path,
    event_type: str,
    warnings: list[str],
) -> str:
    root = _git_root_for_path(ledger_dir)
    if root is None:
        warnings.append("ledger directory is not inside a Git worktree; event was not committed")
        return ""

    ledger_dir = ledger_dir.resolve()
    event_path = event_path.resolve()
    if not event_path.is_relative_to(ledger_dir):
        warnings.append("event path is outside the ledger directory; event was not committed")
        return ""

    rel_event_path = event_path.relative_to(root).as_posix()
    add_result = _run_git(root, "add", "--", rel_event_path)
    if add_result.returncode != 0:
        warnings.append(f"git add failed: {add_result.stderr.strip()}")
        return ""

    message = f"Record evolution event: {event_type}"
    commit_result = _run_git(root, "commit", "-m", message, "--", rel_event_path)
    if commit_result.returncode != 0:
        warnings.append(f"git commit failed: {commit_result.stderr.strip()}")
        return ""

    sha_result = _run_git(root, "rev-parse", "HEAD")
    if sha_result.returncode != 0:
        warnings.append(f"git rev-parse failed: {sha_result.stderr.strip()}")
        return ""
    return sha_result.stdout.strip()


def _git_root_for_path(path: Path) -> Path | None:
    result = _run_git(path, "rev-parse", "--show-toplevel")
    if result.returncode != 0:
        return None
    return Path(result.stdout.strip()).resolve()


def _run_git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=False,
        capture_output=True,
        text=True,
    )


__all__ = ["EvolutionLedgerSpec", "_Go2EvolutionLedger"]
