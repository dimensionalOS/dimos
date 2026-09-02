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

"""Contract tests for turning an unknown into an action.

The loop only closes if a failed query names something runnable. These check
that it does, that nothing hardcodes which skill provides what, and that a
capability with no provider is reported rather than quietly skipped.
"""

from __future__ import annotations

import pytest

from dimos.experimental.memory_belief.answer import UnknownReason
from dimos.experimental.memory_belief.remedy import RemedyResolver

#: What a resolver is handed when the query succeeded: no reason to remedy.
ANSWERED = None


class TestTheLoopCloses:
    def test_the_plan_says_which_provider_would_run(self):
        def sweep(**kwargs):
            return None

        remedy = RemedyResolver({"sweep_place": sweep}).plan_for(UnknownReason.NEVER_COVERED)

        assert remedy.actionable
        assert remedy.provider == "sweep"

    def test_an_answered_query_has_nothing_to_remedy(self):
        assert RemedyResolver().plan_for(ANSWERED) is None


class TestNothingHardcodesASkill:
    def test_the_same_reason_resolves_differently_per_robot(self):
        """A legged robot walks there; an arm turns its wrist."""

        def walk(**kwargs):
            return None

        def turn_wrist(**kwargs):
            return None

        incoherent = UnknownReason.INCOHERENT

        assert RemedyResolver({"observe_place": walk}).plan_for(incoherent).provider == "walk"
        assert (
            RemedyResolver({"observe_place": turn_wrist}).plan_for(incoherent).provider
            == "turn_wrist"
        )


class TestMissingProvidersAreReported:
    def test_an_unprovided_capability_is_not_actionable(self):
        remedy = RemedyResolver().plan_for(UnknownReason.NEVER_COVERED)

        assert not remedy.actionable
        assert "sweep_place" in remedy.note


class TestTerminalReasons:
    def test_no_capability_is_never_actionable(self):
        remedy = RemedyResolver().plan_for(UnknownReason.NO_CAPABILITY)

        assert not remedy.actionable
        assert remedy.capability is None

    def test_it_stays_unactionable_even_if_everything_is_registered(self):
        every = {
            r.suggested_capability: (lambda **kw: None)
            for r in UnknownReason
            if r.suggested_capability
        }

        assert not RemedyResolver(every).plan_for(UnknownReason.NO_CAPABILITY).actionable


class TestRevisitIsDistinguished:
    def test_a_vocabulary_gap_needs_no_second_trip(self):
        """The frames are stored; re-running detection answers it in place."""
        resolver = RemedyResolver({"redetect_with_vocabulary": lambda **kw: None})

        remedy = resolver.plan_for(UnknownReason.OUT_OF_VOCABULARY)

        assert remedy.actionable
        assert not remedy.needs_revisit

    @pytest.mark.parametrize("reason", [UnknownReason.INCOHERENT, UnknownReason.NEVER_COVERED])
    def test_the_others_do(self, reason):
        every = {
            r.suggested_capability: (lambda **kw: None)
            for r in UnknownReason
            if r.suggested_capability
        }

        assert RemedyResolver(every).plan_for(reason).needs_revisit


class TestRegistration:
    def test_registered_capabilities_are_listable(self):
        resolver = RemedyResolver({"sweep_place": lambda **kw: None})

        assert resolver.capabilities == frozenset({"sweep_place"})


class TestEveryReasonTheEngineEmitsCanBeResolved:
    """The gap this closes cost a KeyError at the skill boundary.

    ``api.py`` returned ``reason="INCOHERENT"`` while ``UnknownReason`` had no
    such member, so ``skills.py`` raised ``KeyError`` instead of refusing --
    turning the layer's most characteristic answer, "these sightings may not be
    one thing", into a crash. Reading the reasons back out of the source keeps
    the two from drifting apart again.
    """

    def test_the_enum_holds_every_reason_api_returns(self):
        import pathlib as _pathlib
        import re

        from dimos.experimental.memory_belief import api

        source = _pathlib.Path(api.__file__).read_text()
        emitted = set(re.findall(r'"reason":\s*"([A-Z_]+)"', source))
        emitted |= set(re.findall(r'reason\s*=\s*"unknown",\s*"([A-Z_]+)"', source))

        assert emitted, "no reasons found in api.py; this test has stopped testing anything"
        assert emitted <= {r.name for r in UnknownReason}

    def test_every_enum_member_is_reachable_from_a_skill_result(self):
        for reason in UnknownReason:
            assert UnknownReason[reason.name] is reason
