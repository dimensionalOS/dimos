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

"""TwitchVotes: extends TwitchChat with vote tallying.

Each incoming message is passed through ``message_to_choice`` to extract a
choice string.  If the result is one of ``choices``, the vote is recorded.
At the end of each window the winning choice is published as a
:class:`TwitchChoice` on ``chat_vote_choice``.

Voting modes: plurality, majority, weighted_recent, runoff.
"""

from __future__ import annotations

from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.stream.twitch.module import TwitchChat, TwitchChatConfig, TwitchMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class TwitchChoice:
    winner: str = ""
    total_votes: int = 0
    timestamp: float = 0.0


class VoteMode(str, Enum):
    PLURALITY = "plurality"
    MAJORITY = "majority"
    WEIGHTED_RECENT = "weighted_recent"
    RUNOFF = "runoff"


def _default_message_to_choice(msg: TwitchMessage, choices: list[str]) -> str | None:
    return msg.find_one(choices)


class TwitchVotesConfig(TwitchChatConfig):
    # A vote is only counted if message_to_choice returns one of these.
    choices: list[str] = []
    # (msg, choices) -> choice string, or None to skip
    message_to_choice: Callable[[TwitchMessage, list[str]], str | None] = _default_message_to_choice

    vote_window_seconds: float = 5.0
    min_votes_threshold: int = 1
    vote_mode: VoteMode = VoteMode.PLURALITY


# ── Vote tallying ──


def _tally_plurality(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    if not counts:
        return None
    return counts.most_common(1)[0][0]


def _tally_majority(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    total = sum(counts.values())
    if total == 0:
        return None
    winner, count = counts.most_common(1)[0]
    return winner if count > total / 2 else None


def _tally_weighted_recent(
    votes: list[tuple[str, float, str]], window_start: float, window_end: float
) -> str | None:
    if not votes:
        return None
    duration = max(window_end - window_start, 0.001)
    weighted: dict[str, float] = {}
    for cmd, ts, _ in votes:
        weight = 0.5 + 0.5 * ((ts - window_start) / duration)
        weighted[cmd] = weighted.get(cmd, 0.0) + weight
    if not weighted:
        return None
    return max(weighted, key=weighted.__getitem__)


def _tally_runoff(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    total = sum(counts.values())
    if total == 0:
        return None
    winner, count = counts.most_common(1)[0]
    if count > total / 2:
        return winner

    top2 = {cmd for cmd, _ in counts.most_common(2)}
    if len(top2) < 2:
        return winner

    latest: dict[str, str] = {}
    for cmd, _, voter in votes:
        latest[voter] = cmd

    runoff_counts: Counter[str] = Counter()
    for _voter, cmd in latest.items():
        if cmd in top2:
            runoff_counts[cmd] += 1

    return runoff_counts.most_common(1)[0][0] if runoff_counts else winner


class TwitchVotes(TwitchChat):
    """Extends TwitchChat with vote tallying.

    Each incoming message is passed through ``message_to_choice``.  If the
    result is one of ``choices``, the vote is recorded.  At the end of each
    window the winning choice is published as a :class:`TwitchChoice` on
    ``chat_vote_choice``.
    """

    config: TwitchVotesConfig

    chat_vote_choice: Out[TwitchChoice]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._votes: deque[tuple[str, float, str]] = deque()
        self._votes_lock = threading.Lock()
        self._vote_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._valid_choices: frozenset[str] = frozenset()

    def _on_message_received(self, msg: TwitchMessage) -> None:
        choice = self.config.message_to_choice(msg, self.config.choices)
        if choice and choice in self._valid_choices:
            with self._votes_lock:
                self._votes.append((choice, time.time(), msg.author))

    @rpc
    def start(self) -> None:
        self._stop_event.clear()
        self._valid_choices = frozenset(self.config.choices)
        super().start()

        self._vote_thread = threading.Thread(
            target=self._vote_loop, daemon=True, name="twitch-vote"
        )
        self._vote_thread.start()
        logger.info(
            "[TwitchVotes] Vote loop started",
            vote_mode=self.config.vote_mode.value,
            window=self.config.vote_window_seconds,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._vote_thread is not None:
            self._vote_thread.join(timeout=2)
            self._vote_thread = None
        super().stop()

    def record_vote(self, choice: str, voter: str = "anonymous") -> None:
        """Record a vote programmatically (for testing)."""
        c = choice.lower().strip()
        if c not in self._valid_choices:
            return
        with self._votes_lock:
            self._votes.append((c, time.time(), voter))

    def _vote_loop(self) -> None:
        while not self._stop_event.is_set():
            window_start = time.time()
            self._stop_event.wait(timeout=self.config.vote_window_seconds)
            window_end = time.time()

            cutoff = window_end - self.config.vote_window_seconds
            with self._votes_lock:
                current_votes = [(c, ts, v) for c, ts, v in self._votes if ts >= cutoff]
                # Keep only votes that arrived after this window ended
                self._votes = deque((c, ts, v) for c, ts, v in self._votes if ts >= window_end)

            # Deduplicate: keep only the latest vote per voter
            latest_per_voter: dict[str, tuple[str, float, str]] = {}
            for vote in current_votes:
                latest_per_voter[vote[2]] = vote
            current_votes = list(latest_per_voter.values())

            if len(current_votes) < self.config.min_votes_threshold:
                continue

            winner = self._tally(current_votes, window_start, window_end)
            if winner is None:
                continue

            logger.info("[TwitchVotes] Winner: %s (%d votes)", winner, len(current_votes))
            self.chat_vote_choice.publish(
                TwitchChoice(winner=winner, total_votes=len(current_votes), timestamp=window_end)
            )

    def _tally(
        self,
        votes: list[tuple[str, float, str]],
        window_start: float,
        window_end: float,
    ) -> str | None:
        mode = self.config.vote_mode
        if mode == VoteMode.PLURALITY:
            return _tally_plurality(votes)
        elif mode == VoteMode.MAJORITY:
            return _tally_majority(votes)
        elif mode == VoteMode.WEIGHTED_RECENT:
            return _tally_weighted_recent(votes, window_start, window_end)
        elif mode == VoteMode.RUNOFF:
            return _tally_runoff(votes)
        return _tally_plurality(votes)


twitch_votes = TwitchVotes.blueprint
