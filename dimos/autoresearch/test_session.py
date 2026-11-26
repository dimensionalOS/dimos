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

from pathlib import Path

import pytest

from dimos.autoresearch.session import Budget, Evaluation, Session, SessionInfo, Trial


@pytest.fixture
def session(tmp_path: Path) -> Session:
    return Session.create(
        tmp_path / "research",
        SessionInfo(
            task="Find a smaller encoding",
            experiment="test",
            metric="bytes",
            direction="minimize",
            budget=Budget(max_candidates=2, dev_trials=2, heldout_trials=3),
        ),
    )


def test_snapshot_budget_and_numeric_leaderboard(session: Session) -> None:
    first = session.submit("candidate.bin", b"\x00\xff", "baseline")
    second = session.submit("candidate.bin", b"\xff", "omit padding")

    def size(trial: Trial) -> Evaluation:
        return Evaluation(score=trial.artifact.stat().st_size + trial.index)

    session.evaluate(first, "dev", size)
    assert session.leaderboard()[-1].dev_score is None
    session.evaluate(second, "dev", size)

    (session.artifact(first).parent / "__pycache__").mkdir()
    other = Session(session.dir)
    assert other.artifact(first).read_bytes() == b"\x00\xff"
    assert [(s.candidate, s.dev_score) for s in other.leaderboard()] == [
        (second, 1.5),
        (first, 2.5),
    ]
    with pytest.raises(RuntimeError, match="budget"):
        other.submit("another", b"", "too late")
    with pytest.raises(RuntimeError, match="call freeze"):
        other.evaluate(second, "heldout", size)


def test_resume_interrupted_heldout_without_reusing_evidence_or_rescoring(session: Session) -> None:
    candidate = session.submit("answer.txt", b"plain text", "baseline")
    alternate = session.submit("answer.txt", b"different", "alternate")
    session.freeze(candidate)

    def fails_on_second(trial: Trial) -> Evaluation:
        (trial.directory / "partial").touch()
        if trial.index == 1:
            raise RuntimeError("evaluator unavailable")
        return Evaluation(score=-3)

    with pytest.raises(RuntimeError, match="unavailable"):
        session.evaluate(candidate, "heldout", fails_on_second)
    assert [r.score for r in session.results(candidate)] == [-3]
    assert session.info().status == "frozen"
    other = Session(session.dir)
    with pytest.raises(RuntimeError, match="frozen"):
        other.freeze(alternate)
    with pytest.raises(RuntimeError, match="frozen"):
        other.submit("third", b"", "late")
    with pytest.raises(RuntimeError, match="frozen"):
        other.evaluate(candidate, "dev", fails_on_second)

    indexes = []

    def resumed(trial: Trial) -> Evaluation:
        assert not (trial.directory / "partial").exists()
        indexes.append(trial.index)
        return Evaluation(score=0, note="measured")

    other.freeze(candidate)
    other.evaluate(candidate, "heldout", resumed)
    other.evaluate(candidate, "heldout", resumed)
    assert indexes == [1, 2]
    assert (session.info().status, session.info().heldout_score) == ("scored", -1)


def test_running_evaluation_excludes_other_writers_but_allows_status(session: Session) -> None:
    candidate = session.submit("data", b"", "baseline")
    other = Session(session.dir)

    def measure(trial: Trial) -> Evaluation:
        assert other.info().status == "running"
        with pytest.raises(RuntimeError, match="in progress"):
            other.freeze(candidate)
        with pytest.raises(RuntimeError, match="in progress"):
            other.evaluate(candidate, "dev", measure)
        return Evaluation(score=trial.index)

    session.evaluate(candidate, "dev", measure)
    assert session.info().status == "idle"


def test_library_note_survives_reopen(session: Session) -> None:
    note = session.note("Remove padding", "Keep trailing bytes only for partial blocks.")
    assert [note] == Session(session.dir).library()
    assert note.read_text().startswith("# Remove padding\n")
