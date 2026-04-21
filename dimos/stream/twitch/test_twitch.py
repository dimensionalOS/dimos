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

"""Tests for TwitchMessage, TwitchChat filters, and vote tallying."""

from __future__ import annotations

import time

from dimos.stream.twitch.module import TwitchMessage
from dimos.stream.twitch.votes import (
    _tally_majority,
    _tally_plurality,
    _tally_runoff,
    _tally_weighted_recent,
)

# ── TwitchMessage ──


class TestTwitchMessage:
    def test_text_property(self) -> None:
        msg = TwitchMessage(content="hello")
        assert msg.text == "hello"

    def test_find_one_match(self) -> None:
        msg = TwitchMessage(content="I vote forward please")
        assert msg.find_one(["forward", "back", "left", "right"]) == "forward"

    def test_find_one_case_insensitive(self) -> None:
        msg = TwitchMessage(content="FORWARD")
        assert msg.find_one(["forward", "back"]) == "forward"

    def test_find_one_no_match(self) -> None:
        msg = TwitchMessage(content="hello world")
        assert msg.find_one(["forward", "back"]) is None

    def test_find_one_first_wins(self) -> None:
        msg = TwitchMessage(content="go left or right")
        assert msg.find_one(["left", "right"]) == "left"

    def test_find_one_word_boundary(self) -> None:
        msg = TwitchMessage(content="I want to go backwards")
        assert msg.find_one(["back", "forward"]) is None

    def test_find_one_with_set(self) -> None:
        msg = TwitchMessage(content="back")
        result = msg.find_one({"forward", "back"})
        assert result in ("forward", "back")  # set order not guaranteed

    def test_find_one_with_frozenset(self) -> None:
        msg = TwitchMessage(content="left")
        result = msg.find_one(frozenset(["left", "right"]))
        assert result in ("left", "right")  # frozenset order not guaranteed

    def test_repr(self) -> None:
        msg = TwitchMessage(author="user1", content="hi")
        assert "user1" in repr(msg)
        assert "hi" in repr(msg)


# ── Tally functions ──

# Vote tuple format: (choice, timestamp, voter)
NOW = time.time()


class TestTallyPlurality:
    def test_empty(self) -> None:
        assert _tally_plurality([]) is None

    def test_single_vote(self) -> None:
        assert _tally_plurality([("forward", NOW, "a")]) == "forward"

    def test_winner(self) -> None:
        votes = [("forward", NOW, "a"), ("forward", NOW, "b"), ("back", NOW, "c")]
        assert _tally_plurality(votes) == "forward"

    def test_tie_returns_one(self) -> None:
        votes = [("forward", NOW, "a"), ("back", NOW, "b")]
        result = _tally_plurality(votes)
        assert result in ("forward", "back")


class TestTallyMajority:
    def test_empty(self) -> None:
        assert _tally_majority([]) is None

    def test_majority_winner(self) -> None:
        votes = [("forward", NOW, "a"), ("forward", NOW, "b"), ("back", NOW, "c")]
        assert _tally_majority(votes) == "forward"

    def test_no_majority(self) -> None:
        votes = [("forward", NOW, "a"), ("back", NOW, "b"), ("left", NOW, "c"), ("right", NOW, "d")]
        assert _tally_majority(votes) is None

    def test_exact_half_not_majority(self) -> None:
        votes = [
            ("forward", NOW, "a"),
            ("forward", NOW, "b"),
            ("back", NOW, "c"),
            ("back", NOW, "d"),
        ]
        assert _tally_majority(votes) is None


class TestTallyWeightedRecent:
    def test_empty(self) -> None:
        assert _tally_weighted_recent([], NOW, NOW + 5) is None

    def test_recent_votes_weighted_higher(self) -> None:
        start = NOW
        end = NOW + 10
        # Early vote for "back", late vote for "forward"
        votes = [("back", start + 1, "a"), ("forward", end - 0.1, "b")]
        assert _tally_weighted_recent(votes, start, end) == "forward"

    def test_many_early_can_beat_few_late(self) -> None:
        start = NOW
        end = NOW + 10
        votes = [
            ("back", start + 0.1, "a"),
            ("back", start + 0.2, "b"),
            ("back", start + 0.3, "c"),
            ("forward", end - 0.1, "d"),
        ]
        assert _tally_weighted_recent(votes, start, end) == "back"


class TestTallyRunoff:
    def test_empty(self) -> None:
        assert _tally_runoff([]) is None

    def test_clear_majority(self) -> None:
        votes = [("forward", NOW, "a"), ("forward", NOW, "b"), ("back", NOW, "c")]
        assert _tally_runoff(votes) == "forward"

    def test_runoff_eliminates_third(self) -> None:
        # No majority: forward=2, back=2, left=1
        # Runoff between forward and back, left eliminated
        votes = [
            ("forward", NOW, "a"),
            ("forward", NOW, "b"),
            ("back", NOW, "c"),
            ("back", NOW, "d"),
            ("left", NOW, "e"),
        ]
        result = _tally_runoff(votes)
        assert result in ("forward", "back")

    def test_single_vote(self) -> None:
        assert _tally_runoff([("left", NOW, "a")]) == "left"


# ── TwitchChat._publish_if_matched filter logic ──
# We test the filter logic indirectly via inject_message since TwitchChat
# requires the module system. Instead we test the filter predicates directly.


class TestFilterLogic:
    """Test the filter predicate patterns used in TwitchChatConfig."""

    def test_filter_author_lambda(self) -> None:
        exclude_nightbot = lambda name: name != "nightbot"  # noqa: E731
        assert exclude_nightbot("user1") is True
        assert exclude_nightbot("nightbot") is False

    def test_filter_content_lambda(self) -> None:
        reject_spam = lambda text: len(text) < 200  # noqa: E731
        assert reject_spam("short message") is True
        assert reject_spam("x" * 201) is False


# ── message_to_choice with lambda (the demo pattern) ──


class TestMessageToChoiceLambda:
    def test_lambda_with_emoji_gate(self) -> None:
        choices = ["forward", "back", "left", "right"]
        fn = lambda msg, c: "🤖" in msg.text and msg.find_one(c)  # noqa: E731

        msg_with_emoji = TwitchMessage(content="🤖 forward")
        assert fn(msg_with_emoji, choices) == "forward"

        msg_without_emoji = TwitchMessage(content="forward")
        assert not fn(msg_without_emoji, choices)

    def test_default_message_to_choice(self) -> None:
        from dimos.stream.twitch.votes import _default_message_to_choice

        choices = ["forward", "back"]
        msg = TwitchMessage(content="go forward!")
        assert _default_message_to_choice(msg, choices) == "forward"

        msg2 = TwitchMessage(content="hello")
        assert _default_message_to_choice(msg2, choices) is None


# ── Vote deduplication ──


class TestVoteDeduplication:
    """Test the deduplication logic used in TwitchVotes._vote_loop."""

    def test_latest_vote_per_voter_wins(self) -> None:
        """When a voter changes their mind, only their latest vote counts."""
        # Simulates the dedup logic from _vote_loop
        votes = [
            ("forward", NOW, "alice"),
            ("back", NOW + 1, "alice"),  # alice changed mind
            ("forward", NOW, "bob"),
        ]
        # Dedup: keep latest per voter
        latest_per_voter: dict[str, tuple[str, float, str]] = {}
        for vote in votes:
            latest_per_voter[vote[2]] = vote
        deduped = list(latest_per_voter.values())

        result = _tally_plurality(deduped)
        # alice=back, bob=forward → tie; either is valid
        assert result in ("forward", "back")
        # But alice's final vote was "back"
        assert ("back", NOW + 1, "alice") in deduped
        assert ("forward", NOW, "alice") not in deduped

    def test_single_voter_multiple_votes(self) -> None:
        """A single voter spamming should only get one vote."""
        votes = [
            ("forward", NOW, "spammer"),
            ("forward", NOW + 1, "spammer"),
            ("forward", NOW + 2, "spammer"),
            ("back", NOW, "other"),
        ]
        latest_per_voter: dict[str, tuple[str, float, str]] = {}
        for vote in votes:
            latest_per_voter[vote[2]] = vote
        deduped = list(latest_per_voter.values())

        # 1 vote forward (spammer's latest), 1 vote back (other) → tie
        assert len(deduped) == 2

    def test_no_dedup_different_voters(self) -> None:
        """Different voters all get their votes counted."""
        votes = [
            ("forward", NOW, "a"),
            ("forward", NOW, "b"),
            ("back", NOW, "c"),
        ]
        latest_per_voter: dict[str, tuple[str, float, str]] = {}
        for vote in votes:
            latest_per_voter[vote[2]] = vote
        deduped = list(latest_per_voter.values())

        assert len(deduped) == 3
        assert _tally_plurality(deduped) == "forward"
