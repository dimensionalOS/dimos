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

"""PACK MIND search skills — the bridge from a dog's LLM agent to the shared coordinator.

A DimOS ``Module`` that turns the laptop coordinator's HTTP API into @skill tools
the robot's agent can call. The dog never learns the other dog's coordinates — it
only asks "what zone next?" and reports cleared zones / findings by NAME. Every
call is wrapped in a short-timeout try/except so a coordinator outage degrades to
a calm string and NEVER raises into the agent loop.

Wire it into an agentic blueprint alongside the robot stack; point it at the
coordinator with the ``PACK_COORDINATOR_URL`` env var (or the constructor arg).
"""

from __future__ import annotations

import os
from typing import Any

import requests

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.experimental.pack_mind.pack_coordinator_server import DEFAULT_COORDINATOR_PORT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Built from localhost + the server's port constant — no hardcoded port literal.
DEFAULT_COORDINATOR_URL = f"http://localhost:{DEFAULT_COORDINATOR_PORT}"

# Short timeout: the agent loop must never block on a flaky coordinator.
_REQUEST_TIMEOUT = 5.0


class PackSearchSkills(Module):
    """Exposes the PACK MIND coordinator to one dog's LLM agent as tools.

    Args:
        dog_name: this dog's stable name (used as the ledger identity).
        coordinator_url: base URL of the coordinator server; defaults to the
            ``PACK_COORDINATOR_URL`` env var, else localhost + the default port.
    """

    def __init__(
        self,
        dog_name: str | None = None,
        coordinator_url: str | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        # Per-laptop identity: explicit arg › PACK_DOG_NAME env › "alpha". This lets
        # the SAME blueprint run on both laptops, differing only by env var.
        self._dog_name = dog_name or os.environ.get("PACK_DOG_NAME") or "alpha"
        self._url = (
            coordinator_url
            or os.environ.get("PACK_COORDINATOR_URL")
            or DEFAULT_COORDINATOR_URL
        ).rstrip("/")

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("PackSearchSkills ready", dog=self._dog_name, coordinator=self._url)

    @rpc
    def stop(self) -> None:
        super().stop()

    # -- HTTP plumbing -------------------------------------------------------

    def _post(self, path: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        """POST JSON; return the decoded dict, or None on any failure."""
        try:
            resp = requests.post(
                f"{self._url}{path}", json=payload, timeout=_REQUEST_TIMEOUT
            )
            resp.raise_for_status()
            data: Any = resp.json()
            return data if isinstance(data, dict) else None
        except (requests.RequestException, ValueError) as exc:
            logger.warning("coordinator POST failed", path=path, error=str(exc))
            return None

    def _get(self, path: str, params: dict[str, str]) -> dict[str, Any] | None:
        try:
            resp = requests.get(
                f"{self._url}{path}", params=params, timeout=_REQUEST_TIMEOUT
            )
            resp.raise_for_status()
            data: Any = resp.json()
            return data if isinstance(data, dict) else None
        except (requests.RequestException, ValueError) as exc:
            logger.warning("coordinator GET failed", path=path, error=str(exc))
            return None

    # -- skills (exposed to the LLM) -----------------------------------------

    @skill
    def start_search(self, target: str) -> str:
        """Begin a pack search for an object across the shared zones.

        Call this once at mission start. It resets the shared zone ledger so the
        whole pack hunts for the same target without re-searching areas.

        Args:
            target: A short description of what to find, e.g. "red backpack".
        """
        data = self._post("/start_search", {"target": target})
        if data is None:
            return f"Could not reach the pack coordinator to start searching for {target}."
        return str(data.get("status", f"Searching for {target}."))

    @skill
    def report_cleared(self, zone: str) -> str:
        """Tell the pack you finished searching a zone and the target was NOT there.

        Marks the zone cleared on the shared ledger so no teammate re-searches it.

        Args:
            zone: The name of the zone you just finished searching, e.g. "north".
        """
        data = self._post("/report_cleared", {"dog": self._dog_name, "zone": zone})
        if data is None:
            return f"Could not reach the pack coordinator to report {zone} cleared."
        return str(data.get("status", f"{zone} cleared."))

    @skill
    def report_finding(self, object: str, zone: str) -> str:
        """Tell the whole pack you found the target so everyone stops searching.

        Records the sighting by zone NAME on the shared blackboard; the first
        finding wins and halts all assignment.

        Args:
            object: What you found, e.g. "red backpack".
            zone: The zone name where you found it, e.g. "east".
        """
        data = self._post(
            "/report_finding",
            {"dog": self._dog_name, "object": object, "zone": zone},
        )
        if data is None:
            return f"Could not reach the pack coordinator to report finding {object}."
        finding = data.get("finding")
        if isinstance(finding, dict):
            return f"Pack notified: {finding.get('object')} found in {finding.get('zone')}."
        return f"Pack notified: {object} found in {zone}."

    @skill
    def where_is(self, object: str) -> str:
        """Ask the pack's shared memory where an object was found.

        Use this to act on a TEAMMATE's discovery — you may never have seen the
        object yourself. Returns the zone a packmate reported it in, or that it
        hasn't been found yet.

        Args:
            object: What you're asking about, e.g. "red backpack".
        """
        data = self._get("/where_is", {})
        if data is None:
            return "Could not reach the pack memory."
        if not data.get("found"):
            return f"No packmate has found the {object} yet."
        return (
            f"{data.get('by')} found the {data.get('object')} in {data.get('zone')}. "
            f"I can take you there."
        )

    # -- rpc (internal control, not exposed to the LLM) ----------------------

    @rpc
    def finding_zone(self) -> str:
        """The zone of the current finding (for navigation), or "" if none yet."""
        data = self._get("/where_is", {})
        if data is None or not data.get("found"):
            return ""
        zone = data.get("zone")
        return zone if isinstance(zone, str) else ""

    @rpc
    def release_dog(self, dog: str) -> bool:
        """Mark a packmate offline so the pack reclaims its unfinished zones.

        Operator/coordinator action (e.g. a dog dropped). The downed dog's
        findings persist; its claimed-but-uncleared zones return to the pool so a
        survivor inherits them. Returns True on success.
        """
        data = self._post("/release_dog", {"dog": dog})
        return data is not None

    @rpc
    def next_zone(self) -> str:
        """Ask the coordinator for the next unsearched zone for this dog.

        Returns the zone name, or "" when the search is over (found or no zones
        left) — skills cannot return None cleanly, so "" signals "stop".
        """
        data = self._post("/assign_zone", {"dog": self._dog_name})
        if data is None:
            return ""
        zone = data.get("zone")
        return zone if isinstance(zone, str) else ""

    @rpc
    def should_stop(self) -> bool:
        """True once any dog has found the target and the pack should converge."""
        data = self._get("/should_stop", {"dog": self._dog_name})
        if data is None:
            return False
        return bool(data.get("stop", False))


pack_search_skills = PackSearchSkills.blueprint
