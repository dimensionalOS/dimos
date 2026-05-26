#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Patrol Dog skill container — HACKATHON TEMPLATE.

This mirrors the real DimOS pattern in
`dimos/robot/unitree/unitree_skill_container.py`: a `Module` subclass whose
`@skill` methods become the agent's tools. At the hackathon you fill in the
bodies (wiring to the real Go2 connection / navigation / perception specs) and
register this container in the Go2 agentic blueprint.

Design rules learned from the real codebase:
  * Each @skill method needs a CLEAR docstring (the LLM reads it as the tool
    description) and typed params with sensible defaults.
  * Return a short human-readable `str` (works on every dimos version). On
    newer `main` you may instead return `SkillResult(success=..., message=...,
    error_code=...)` for structured outcomes — but PyPI 0.0.12.post2 does NOT
    have SkillResult, so stick to `str` unless you've installed from main.
  * Keep skills idempotent-ish and fast; do heavy work behind a serialized
    service (the robot SDK is not concurrency-safe).

NOTE: this file is a TEMPLATE — the bodies are stubs/mocks so it reads and
imports without hardware. Wire the TODOs to real specs on Day 1.
"""

from __future__ import annotations

import datetime

from dimos.agents.annotation import skill
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PatrolDogConfig(ModuleConfig):
    # Named waypoints for the patrol route; fill with real map coords on Day 1.
    waypoints: list[str] = ["lobby", "corridor", "back_door"]
    site_name: str = "Booth 4"


class PatrolDogSkills(Module):
    """Skills for the '巡逻汪 / Patrol Dog' inspection-and-security agent.

    Wire these to the real Go2 specs (connection / navigation / perception)
    the way unitree_skill_container.py does, e.g.:

        nav: NavigationInterfaceSpec
        conn: GO2ConnectionSpec

    For now they are mocked so the container imports with no hardware.
    """

    config: PatrolDogConfig

    # --- mock state (replace with real subscriptions to /odom, costmap, etc.) ---
    _incidents: list[str] = []
    _current_wp: str = "start"

    @skill
    def patrol_route(self) -> str:
        """List the waypoints on the current patrol route, in order.

        Use this to tell the operator where the dog will go before it starts.
        """
        return "Patrol route: " + " -> ".join(self.config.waypoints)

    @skill
    def go_to_waypoint(self, name: str = "lobby") -> str:
        """Navigate the dog to a named waypoint on the patrol route.

        Args:
            name: One of the route waypoints (see patrol_route).
        """
        if name not in self.config.waypoints:
            return f"Unknown waypoint '{name}'. Known: {self.config.waypoints}"
        # TODO Day 1: publish a goal to the planner (see ReplanningAStarPlanner
        # /goal_request topic) and block until /goal_reached.
        self._current_wp = name
        return f"Arrived at '{name}'."

    @skill
    def describe_surroundings(self) -> str:
        """Describe what the dog currently sees at its location.

        On hardware this grabs the latest RGB frame and asks a VLM (Qwen /
        moondream) for a one-line description. Mocked here.
        """
        # TODO Day 1: pull latest /color_image, run the VLM detector, summarize.
        return f"[{self._current_wp}] Mock view: corridor clear, one closed door, lights on."

    @skill
    def check_for_anomaly(self) -> str:
        """Compare the current scene to the learned 'normal' baseline and report
        whether anything looks out of place (open door, unexpected person,
        misplaced object)."""
        # TODO Day 1: query spatial memory for the baseline at this pose; diff.
        return f"[{self._current_wp}] No anomaly vs. baseline."

    @skill
    def log_incident(self, description: str = "") -> str:
        """Record an incident (with location + timestamp) for the operator.

        Args:
            description: What was observed that warranted logging.
        """
        ts = datetime.datetime.now().isoformat(timespec="seconds")
        entry = f"{ts} @ {self._current_wp}: {description}"
        self._incidents.append(entry)
        logger.info("INCIDENT %s", entry)
        return f"Logged incident #{len(self._incidents)}: {entry}"

    @skill
    def incident_report(self) -> str:
        """Return the full list of incidents logged during this patrol."""
        if not self._incidents:
            return f"No incidents logged at {self.config.site_name}."
        return "Incident report:\n" + "\n".join(
            f"  {i + 1}. {e}" for i, e in enumerate(self._incidents)
        )


# How to run it (Day 1):
#   1. Add PatrolDogSkills to the Go2 agentic blueprint's skill containers
#      (model on dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py).
#   2. dimos --simulation run unitree-go2-agentic --daemon
#   3. dimos mcp list-tools           # confirm the skills appear as MCP tools
#      dimos mcp call go_to_waypoint --arg name=corridor   # call without an LLM
#      dimos agent-send "patrol the route and report anything unusual"  # via LLM
