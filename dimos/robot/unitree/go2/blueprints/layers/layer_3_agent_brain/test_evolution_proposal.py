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

from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_ledger import (
    _Go2EvolutionLedger,
)
from dimos.robot.unitree.go2.blueprints.layers.layer_3_agent_brain.evolution_proposal import (
    SKILL_PROPOSAL_SCHEMA,
    validate_skill_proposal,
)


def _stop_modules(*modules: object) -> None:
    for module in modules:
        stop = getattr(module, "stop", None)
        if stop is not None:
            stop()


def _proposal(**overrides: object) -> dict[str, object]:
    proposal: dict[str, object] = {
        "schema": SKILL_PROPOSAL_SCHEMA,
        "proposal_id": "go2-context-evidence-policy",
        "title": "Tighten context evidence policy",
        "summary": "Require memory backend evidence before navigation planning.",
        "risk_level": "medium",
        "target_files": [
            "dimos/robot/unitree/go2/blueprints/layers/layer_3_agent_brain/context_provider.py"
        ],
        "requires_human_review": True,
        "generated_patch": "",
    }
    proposal.update(overrides)
    return proposal


def test_validate_skill_proposal_accepts_reviewable_relative_targets() -> None:
    assert validate_skill_proposal(_proposal()) == []


def test_validate_skill_proposal_rejects_direct_unreviewed_mutation() -> None:
    errors = validate_skill_proposal(_proposal(requires_human_review=False))

    assert "requires_human_review must be true" in errors


def test_validate_skill_proposal_rejects_unsafe_targets() -> None:
    errors = validate_skill_proposal(
        _proposal(
            target_files=[
                "/tmp/direct-write.py",
                "../outside.py",
                ".dimos/evolution/events/2026/07/04/raw.json",
                "dimos/data/raw_camera_frame.png",
            ]
        )
    )

    assert "target_files[0] must be a relative path" in errors
    assert "target_files[1] must not contain '..'" in errors
    assert "target_files[2] must not target evolution event logs" in errors
    assert "target_files[3] must not target large binary artifacts" in errors


def test_record_skill_proposal_writes_validated_proposal(monkeypatch, tmp_path: Path) -> None:
    ledger_dir = tmp_path / ".dimos" / "evolution"
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(ledger_dir))
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_skill_proposal(proposal_json=json.dumps(_proposal()))

        assert result.success is True
        proposal_path = Path(result.metadata["proposal_path"])
        assert proposal_path.exists()
        assert proposal_path.is_relative_to(ledger_dir / "proposals")

        data = json.loads(proposal_path.read_text())
        assert data["schema"] == SKILL_PROPOSAL_SCHEMA
        assert data["requires_human_review"] is True
        assert data["git"] == {"commit_requested": False, "commit_sha": ""}
    finally:
        _stop_modules(ledger)


def test_record_skill_proposal_preserves_reused_proposal_id(
    monkeypatch, tmp_path: Path
) -> None:
    ledger_dir = tmp_path / ".dimos" / "evolution"
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(ledger_dir))
    ledger = _Go2EvolutionLedger()
    try:
        first = ledger.record_skill_proposal(
            proposal_json=json.dumps(_proposal(title="First iteration of the proposal"))
        )
        second = ledger.record_skill_proposal(
            proposal_json=json.dumps(_proposal(title="Second iteration of the proposal"))
        )

        assert first.success is True
        assert second.success is True

        first_path = Path(first.metadata["proposal_path"])
        second_path = Path(second.metadata["proposal_path"])
        assert first_path != second_path
        assert first_path.exists()
        assert second_path.exists()

        first_data = json.loads(first_path.read_text())
        second_data = json.loads(second_path.read_text())
        # The author's proposal_id is preserved verbatim in both artifacts;
        # only the filename is disambiguated.
        assert first_data["proposal_id"] == second_data["proposal_id"]
        assert first_data["title"] == "First iteration of the proposal"
        assert second_data["title"] == "Second iteration of the proposal"
    finally:
        _stop_modules(ledger)


def test_record_skill_proposal_keeps_distinct_sanitized_ids_separate(
    monkeypatch, tmp_path: Path
) -> None:
    """Distinct proposal_ids that sanitize to the same slug must not collide.

    ``_safe_filename`` maps every non-alphanumeric character to ``_``, so
    ``skill-v2`` and ``skill.v2`` both reduce to ``skill_v2``. They are
    unrelated proposals and must land on distinct files — neither overwriting
    the other, nor conflated as iterations (``-1``) of a single proposal.
    """
    ledger_dir = tmp_path / ".dimos" / "evolution"
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(ledger_dir))
    ledger = _Go2EvolutionLedger()
    try:
        dash = ledger.record_skill_proposal(
            proposal_json=json.dumps(_proposal(proposal_id="skill-v2", title="Dash variant"))
        )
        dot = ledger.record_skill_proposal(
            proposal_json=json.dumps(_proposal(proposal_id="skill.v2", title="Dot variant"))
        )

        assert dash.success is True
        assert dot.success is True

        dash_path = Path(dash.metadata["proposal_path"])
        dot_path = Path(dot.metadata["proposal_path"])
        assert dash_path != dot_path
        assert dash_path.exists()
        assert dot_path.exists()
        # Neither is a ``-1`` iteration of the other: the digests anchored on
        # the original ids differ, so both keep their unsuffixed base name.
        assert dash_path.name != "skill_v2-1.json"
        assert dot_path.name != "skill_v2-1.json"

        dash_data = json.loads(dash_path.read_text())
        dot_data = json.loads(dot_path.read_text())
        assert dash_data["proposal_id"] == "skill-v2"
        assert dot_data["proposal_id"] == "skill.v2"
        assert dash_data["title"] == "Dash variant"
        assert dot_data["title"] == "Dot variant"
    finally:
        _stop_modules(ledger)


def test_record_skill_proposal_rejects_invalid_target(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("DIMOS_EVOLUTION_LEDGER_DIR", str(tmp_path / "ledger"))
    ledger = _Go2EvolutionLedger()
    try:
        result = ledger.record_skill_proposal(
            proposal_json=json.dumps(_proposal(target_files=["/tmp/direct-write.py"]))
        )

        assert result.success is False
        assert result.error_code == "INVALID_INPUT"
        assert "relative path" in result.message
    finally:
        _stop_modules(ledger)
