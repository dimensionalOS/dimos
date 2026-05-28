#!/usr/bin/env python3
# Drop-in Guide reactive Q&A skills.
# Tracks the priming session (tagged + skipped objects) so visitors can ask
# the robot about its memory after priming completes. Pairs with the
# `tag_location` skill that already writes to dimOS spatial memory — these
# skills just maintain a parallel session log so we can answer summary
# questions like "what did you skip?" and "where can you take me?".

import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _human_age(seconds_ago: float) -> str:
    """Format an age in seconds as a human-readable phrase."""
    if seconds_ago < 30:
        return "just now"
    if seconds_ago < 90:
        return "about a minute ago"
    if seconds_ago < 60 * 60:
        return f"about {int(round(seconds_ago / 60))} minutes ago"
    if seconds_ago < 2 * 60 * 60:
        return "about an hour ago"
    return f"{int(round(seconds_ago / 3600))} hours ago"


class ReactiveQASkills(Module):
    """Session log + Q&A helpers for the priming workflow."""

    _tagged: list[dict[str, Any]] = []
    _skipped: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._tagged = []
        self._skipped = []

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def note_tagged(self, name: str) -> str:
        """Record that a location was tagged during priming. Call this RIGHT
        AFTER `tag_location` so we can later answer "where can you take me?"
        and "what places do you know?" with time-aware context.

        Args:
            name: the name used in the corresponding tag_location call
                  (e.g. "printer", "the kitchen").
        """
        self._tagged.append({"name": name.strip(), "ts": time.time()})
        return f"Noted tagged location: {name}"

    @skill
    def note_skipped(self, description: str) -> str:
        """Record that the operator chose to skip a proposed object during
        priming. Call this when the operator says "skip" / "no important" /
        "next one" instead of confirming a tag. Used later to answer
        "what did you skip?".

        Args:
            description: short description of what was visible but not tagged
                         (e.g. "a green plant on a stand to my right").
        """
        self._skipped.append({"description": description.strip(), "ts": time.time()})
        return f"Noted skip: {description}"

    def _format_tagged_entry(self, entry: dict[str, Any], now: float) -> str:
        age = _human_age(now - entry["ts"])
        return f"{entry['name']} (tagged {age})"

    @skill
    def list_tagged_places(self) -> str:
        """List every place I've tagged so far in this session, with how
        long ago each one was tagged. Use this when someone asks
        "where can you take me?", "what places do you know?", "what did
        you tag?", or similar overview questions.
        """
        if not self._tagged:
            return "I haven't tagged any places yet in this session."
        now = time.time()
        formatted = [self._format_tagged_entry(t, now) for t in self._tagged]
        if len(formatted) == 1:
            return (
                f"I know one place: {formatted[0]}. "
                f"You can ask me to take you there."
            )
        return (
            f"I know {len(formatted)} places: " + "; ".join(formatted) + ". "
            f"You can ask me to take you to any of them."
        )

    @skill
    def what_did_you_skip(self) -> str:
        """Report the objects the operator chose to skip during priming. Use
        this when someone asks "what did you skip?", "what didn't you tag?",
        or "anything you saw but ignored?".
        """
        if not self._skipped:
            return "I didn't skip anything during this priming session."
        now = time.time()
        descs = [
            f"{s['description']} ({_human_age(now - s['ts'])})" for s in self._skipped
        ]
        if len(descs) == 1:
            return f"I skipped one object during priming: {descs[0]}."
        return f"I skipped {len(descs)} objects during priming: " + "; ".join(descs) + "."

    @skill
    def narrate_tour(self) -> str:
        """Narrate a verbal tour of everything I've tagged during this
        session. Use this when someone says "give me a tour", "tell me
        what you know", "show me around", or asks for a general overview.
        Returns a complete sentence the agent should then `speak` aloud.
        """
        if not self._tagged:
            return (
                "I haven't tagged anything yet. Walk me around and I can "
                "build up a tour as we go."
            )
        now = time.time()
        names = [t["name"] for t in self._tagged]
        if len(names) == 1:
            return (
                f"Here's a quick tour: I have {names[0]}, "
                f"tagged {_human_age(now - self._tagged[0]['ts'])}. "
                f"Ask me to take you there anytime."
            )
        first_lines = [
            f"{t['name']} ({_human_age(now - t['ts'])})" for t in self._tagged[:-1]
        ]
        last = self._tagged[-1]
        last_line = (
            f"and {last['name']} ({_human_age(now - last['ts'])})"
        )
        return (
            f"Here's the tour. I know {len(names)} places: "
            + ", ".join(first_lines)
            + ", "
            + last_line
            + ". Pick any one and I'll take you there."
        )

    @skill
    def express_uncertainty(self, topic: str, reason: str) -> str:
        """Speak a calibrated uncertainty statement BEFORE acting on a
        low-confidence decision. Use this when:
        - You can't find an exact tagged match for a navigation query.
        - `describe_scene` returned ambiguous output.
        - The operator's instruction has multiple valid interpretations.

        The robot should call `speak` with the returned sentence so visitors
        hear the caveat instead of seeing silent failure.

        Args:
            topic: brief subject of the uncertainty (e.g. "the printer",
                   "your previous instruction").
            reason: one short clause explaining why you're uncertain
                    (e.g. "I haven't tagged it yet",
                    "the camera frame is blurred",
                    "there are two locations that match").
        """
        topic = topic.strip()
        reason = reason.strip().rstrip(".")
        return f"I'm not sure about {topic} — {reason}. Want me to make a best guess, or wait?"
