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

"""Contract tests for the agent-facing belief skills.

The property that matters is that a failure survives the trip to the agent as
structure: reason, whether retrying helps, and what would fix it. If any of
that degrades to prose on the way out, the agent is back to guessing.
"""

from __future__ import annotations

import math
import time

import pytest

from dimos.experimental.memory_belief import skills as skills_module
from dimos.experimental.memory_belief.answer import UnknownReason
from dimos.experimental.memory_belief.skills import BeliefQuerySkills
from dimos.experimental.memory_belief.types import SCHEMA_VERSION, BeliefObservation, EvidenceRef
from dimos.experimental.memory_belief.write import append_belief, belief_stream
from dimos.memory.store.sqlite import SqliteStore


def record(**overrides) -> BeliefObservation:
    fields = {
        "schema_version": SCHEMA_VERSION,
        "target_ref": "chair-17",
        "label": "chair",
        "visibility": "present",
        "valid_ts": 0.0,
        "observed_ts": 0.0,
        "source": "fake",
        "place_ref": "kitchen",
        "evidence": (EvidenceRef(stream="color_image", observation_id=7, ts=0.0),),
        **overrides,
    }
    return BeliefObservation(**fields)


@pytest.fixture
def skills(tmp_path):
    """A module wired to a store holding one fresh sighting."""

    store = SqliteStore(path=str(tmp_path / "b.db"))
    stream = belief_stream(store)
    append_belief(stream, record(valid_ts=time.time(), observed_ts=time.time()))

    module = BeliefQuerySkills(store=store)
    try:
        yield module
    finally:
        module.dispose()
        store.stop()


class TestTheSkillCannotContradictTheEvidence:
    def test_retryable_follows_the_reason_rather_than_the_skill(self, skills):
        """``retryable`` is derived from the unknown reason, so a caller that
        retries is acting on what the data says, not on what a skill claimed.

        NEVER_COVERED is retryable because going and looking would change it;
        the two must not be settable apart.
        """
        result = skills.query(
            {"select": "entities", "where": [{"op": "label", "value": "no-such-thing"}]}
        )

        assert result.unknown_reason is UnknownReason.NEVER_COVERED
        assert result.retryable is not UnknownReason.NEVER_COVERED.is_terminal


class TestTheAgentCannotAskAboutTheFuture:
    """The asking time is the robot's, not the model's.

    ``as_of`` used to be a `setdefault`, so a model could name any moment it
    liked. Combined with a query engine that accepted `inf`, a runtime agent
    could answer a question about now from observations not yet made.
    """

    @staticmethod
    def _as_of_reaching_the_engine(skills, monkeypatch, asked):
        """The value `execute` is actually called with, not the one asked for."""
        seen = {}

        def spy(store, payload):
            seen["as_of"] = payload["as_of"]
            return {"status": "unknown", "reason": "NEVER_COVERED", "quality": {}}

        monkeypatch.setattr(skills_module, "execute", spy)
        skills.query({"select": "entities", "as_of": asked, "project": "locate"})
        return seen["as_of"]

    def test_a_later_as_of_is_clamped_to_now(self, skills, monkeypatch):
        asked = time.time() + 86_400
        assert self._as_of_reaching_the_engine(skills, monkeypatch, asked) < asked

    def test_an_earlier_as_of_is_honoured(self, skills, monkeypatch):
        """Looking back is a real question and must keep working."""
        asked = time.time() - 3600
        got = self._as_of_reaching_the_engine(skills, monkeypatch, asked)
        assert got == pytest.approx(asked)

    def test_a_non_finite_as_of_falls_back_to_now(self, skills, monkeypatch):
        got = self._as_of_reaching_the_engine(skills, monkeypatch, float("inf"))
        assert math.isfinite(got)


class TestReadOnly:
    def test_the_query_skill_holds_no_capabilities(self):
        """Nothing here moves the robot or writes belief, so a query can never
        be refused for conflicting with something that does."""
        assert BeliefQuerySkills.query.__skill_uses__ == []


class TestARefusalCarriesWhatWouldFixIt:
    """A refusal an agent cannot act on is barely better than a wrong answer.

    The two cases below differ only in whether this deployment registered a
    provider, and they warrant opposite next moves: go and look, versus give up
    and say so. Both are the resolver's, not the query's -- what a robot can do
    is not a property of what the data says.
    """

    def _refusal(self, module):
        result = module.query(
            {"select": "entities", "where": [{"op": "label", "value": "no-such-thing"}]}
        )
        assert not result.success
        return result.metadata["remedy"]

    def test_an_unprovided_capability_is_named_but_not_actionable(self, skills):
        remedy = self._refusal(skills)

        assert remedy["capability"] == "sweep_place"
        assert remedy["actionable"] is False
        assert "sweep_place" in remedy["note"]

    def test_a_provided_capability_becomes_actionable(self, tmp_path):
        store = SqliteStore(path=str(tmp_path / "r.db"))
        append_belief(belief_stream(store), record(valid_ts=time.time()))
        module = BeliefQuerySkills(store=store, remedies={"sweep_place": lambda: None})
        try:
            remedy = self._refusal(module)
        finally:
            module.dispose()
            store.stop()

        assert remedy["actionable"] is True
        assert remedy["needs_revisit"] is True
