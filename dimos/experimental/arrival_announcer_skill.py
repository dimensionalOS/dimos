#!/usr/bin/env python3
# Drop-in Guide arrival announcer.
# Subscribes to the planner's /goal_reached topic and, when the robot arrives
# at a nav goal, injects a synthetic "you have arrived" message into the
# McpClient's human_input stream. Claude then responds with the configured
# arrival sequence (speak + Hello wave).
#
# Why a synthetic message rather than direct speak+wave: keeps the agent's
# conversation history coherent (the visitor sees one continuous dialogue),
# and lets the prompt determine arrival phrasing per-context.

from __future__ import annotations

import time

from dimos_lcm.std_msgs import Bool

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ARRIVAL_PROMPT = (
    "[SYSTEM ARRIVAL EVENT] The robot has just reached the navigation goal. "
    "Greet the visitor: call `speak` with ONE short arrival sentence (e.g. "
    "\"Here we are at the X. Anything else?\"), then call "
    "`execute_sport_command(command_name=\"Hello\")` to wave. Do not "
    "announce arrival a second time if you have already done so for this "
    "trip."
)

# After arrival, suppress duplicate triggers for this many seconds so a
# burst of goal_reached pulses doesn't double-fire the greeting.
DEDUP_WINDOW_S = 5.0


class ArrivalAnnouncerSkill(Module):
    """On planner goal_reached=True, inject an arrival prompt into the agent."""

    goal_reached: In[Bool]
    human_input: Out[str]

    _last_fire_ts: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.goal_reached.subscribe(self._on_goal_reached)
        logger.info("ArrivalAnnouncerSkill: subscribed to /goal_reached")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_goal_reached(self, msg: Bool) -> None:
        if not bool(getattr(msg, "data", False)):
            return
        now = time.time()
        if now - self._last_fire_ts < DEDUP_WINDOW_S:
            return
        self._last_fire_ts = now
        logger.info("ArrivalAnnouncerSkill: goal_reached=True -> injecting arrival prompt")
        self.human_input.publish(ARRIVAL_PROMPT)
