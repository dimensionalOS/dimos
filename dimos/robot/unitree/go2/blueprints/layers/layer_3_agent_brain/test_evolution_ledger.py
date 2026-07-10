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

import json
from pathlib import Path
import subprocess

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
    _Go2EvolutionLedger,
)


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


def test_record_evolution_event_writes_json_event(monkeypatch, tmp_path: Path) -> None:
    ledger_dir = tmp_path / ".dimos" / "evolution"
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(ledger_dir))
    monkeypatch.setenv("DIMOS_RUN_ID", "run-123")
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_evolution_event(
            event_type="task_feasibility",
            task="walk to the kitchen",
            payload_json='{"feasible": "uncertain"}',
        )

        assert result.success is True
        event_path = Path(result.metadata["event_path"])
        assert event_path.exists()
        assert event_path.is_relative_to(ledger_dir)

        data = json.loads(event_path.read_text())
        assert data["schema"] == "dimos.evolution_event.v1"
        assert data["event_type"] == "task_feasibility"
        assert data["task"] == "walk to the kitchen"
        assert data["run_id"] == "run-123"
        assert data["payload"] == {"feasible": "uncertain"}
        assert data["git"] == {"commit_requested": False, "commit_sha": ""}
        assert (ledger_dir / "README.md").exists()
    finally:
        _stop_modules(ledger)


def test_record_evolution_event_rejects_invalid_payload_json(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(tmp_path / "ledger"))
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_evolution_event(
            event_type="context_feedback",
            payload_json="{not-json}",
        )

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
    finally:
        _stop_modules(ledger)


def test_record_evolution_event_commits_only_event_file(monkeypatch, tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    _git(repo, "init")
    _git(repo, "config", "user.email", "dimos@example.com")
    _git(repo, "config", "user.name", "DimOS Test")

    unrelated = repo / "unrelated.txt"
    unrelated.write_text("do not stage me")

    ledger_dir = repo / ".dimos" / "evolution"
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(ledger_dir))
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_evolution_event(
            event_type="context_feedback",
            task="inspect object",
            payload_json='{"source": "spatial_memory"}',
            commit=True,
        )

        assert result.success is True
        assert result.metadata["commit_sha"]

        committed_files = _git(repo, "show", "--name-only", "--pretty=format:", "HEAD")
        assert committed_files.splitlines() == [
            Path(result.metadata["event_path"]).relative_to(repo).as_posix()
        ]

        status = _git(repo, "status", "--short")
        assert "?? unrelated.txt" in status
        assert "A  unrelated.txt" not in status
    finally:
        _stop_modules(ledger)


def test_record_evolution_event_warns_when_commit_requested_outside_git(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(tmp_path / "ledger"))
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_evolution_event(
            event_type="task_feasibility",
            payload_json="{}",
            commit=True,
        )

        assert result.success is True
        assert result.metadata["commit_sha"] == ""
        assert result.metadata["warnings"]
    finally:
        _stop_modules(ledger)
