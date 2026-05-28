"""Pure state machine for the greeter loop (no DimOS).

The greeter dog cycles through four phases:

- ``wander``: the DimOS patrol roams the room while the container scans for a
  person sitting on a chair. When it picks a random not-recently-greeted subject
  (see :mod:`pawtrack.visited_registry`) the patrol stops and we engage.
- ``approach``: drive to a polite standoff while keeping the subject centered.
- ``greet``: hold still and wave hello once -- whichever way the person faces.
- ``cooldown``: brief pause; the container marks the subject visited and resumes
  the patrol, then we return to ``wander``.

The greeting is unconditional: any seated person reached at a safe standoff gets
a wave -- there is no facing / orbit-to-front requirement (an earlier design
circled until the subject faced the camera, which often never resolved).

This module owns *which phase* the robot is in, given a per-tick observation;
the DimOS container performs the effects -- start/stop the patrol, run the
detector and tracker, drive ``cmd_vel``, fire the greeting, and mark the subject
visited -- reacting to the ``entered`` edges in the returned snapshot.

Pure: stdlib only, unit tested. Mirrors the ``MonitorState`` pattern in
:mod:`pawtrack.track_state`.
"""

from __future__ import annotations

import dataclasses
import json
from typing import Literal

State = Literal["wander", "approach", "greet", "cooldown"]

_MESSAGES: dict[State, str] = {
    "wander": "Patrolling, looking for a person sitting on a chair.",
    "approach": "Approaching the subject.",
    "greet": "Greeting the subject (waving hello).",
    "cooldown": "Greeted; marking visited and resuming the patrol.",
}


@dataclasses.dataclass(frozen=True)
class GreeterParams:
    """Thresholds for the greeter phase transitions.

    Attributes:
        standoff_m: Polite distance to hold from the subject before greeting.
        standoff_tolerance_m: Slack added to ``standoff_m`` when deciding the
            robot is "close enough" to stop approaching and wave.
        min_safe_distance_m: Hard floor on the distance at greeting time -- the
            robot will not wave if it is closer than this (accounts for the dog's
            footprint and the reach of the Hello animation). If it has drifted
            inside this, the standoff hold backs it out before it can greet.
        engage_timeout_s: Give up and resume patrol if approaching stalls this
            long without greeting (e.g. an unreachable spot, or a subject that
            left). The subject is then skipped so we do not immediately re-pick
            and re-stick on it. This bounds the whole engagement, including the
            dead-reckon-to-last-known phase after a track loss.
        greet_duration_s: How long the greeting phase lasts.
        cooldown_duration_s: How long to wait after greeting before resuming the
            patrol, so the just-greeted subject is not immediately re-engaged.
    """

    standoff_m: float = 1.5
    standoff_tolerance_m: float = 0.3
    min_safe_distance_m: float = 1.0
    engage_timeout_s: float = 10.0
    greet_duration_s: float = 3.0
    cooldown_duration_s: float = 5.0


@dataclasses.dataclass(frozen=True)
class GreeterObservation:
    """One tick of perception for the greeter machine.

    Attributes:
        target_acquired: A not-recently-greeted subject was selected this tick
            (the container has chosen one and started tracking it). Triggers
            ``wander -> approach``.
        subject_visible: Whether the engaged subject is currently tracked (or was
            re-detected on arrival). The greeting only fires while it is visible.
        distance_m: Estimated floor distance to the subject, or None if unknown.
            While the subject is tracked this is the camera-ranged distance;
            while it is lost it is the odom distance to its last-known position.
        reached_destination: While the subject is lost, whether the dog has
            driven to its last-known position (dead reckoning) and is still
            unable to see it -- the cue to give up that engagement.
    """

    target_acquired: bool = False
    subject_visible: bool = False
    distance_m: float | None = None
    reached_destination: bool = False


@dataclasses.dataclass(frozen=True)
class GreeterSnapshot:
    """The greeter phase after a tick, suitable for JSON status output.

    Attributes:
        state: The current phase.
        message: Human-readable description of the phase.
        entered: True if this tick transitioned into ``state`` -- a one-shot
            edge the container keys effects off: ``approach`` -> stop the patrol
            and start tracking, ``greet`` -> fire the greeting once, ``cooldown``
            -> mark the subject visited, ``wander`` -> resume the patrol.
        reason: Why this phase was entered, when it matters. ``"stuck"`` on a
            ``wander`` edge means an engagement timed out (vs a merely lost
            subject, ``None``), so the container can skip that subject.
    """

    state: State
    message: str
    entered: bool = False
    reason: str | None = None

    def to_json(self) -> str:
        """Serialize to compact JSON for a status stream."""
        return json.dumps(dataclasses.asdict(self), separators=(",", ":"))


class GreeterMachine:
    """Decides the greeter phase across ticks from per-tick observations.

    Wander -> approach (a target is acquired) -> greet (visible at a safe
    standoff) -> cooldown (timed) -> wander. Losing the tracker does not abandon
    the approach: the container dead-reckons to the subject's last-known
    position, and only once it has arrived there and still cannot see the subject
    (``reached_destination`` with ``subject_visible`` false) does the machine
    give up. ``engage_timeout_s`` bounds the whole engagement. Greet and cooldown
    are purely timed and ignore the observation.
    """

    def __init__(self, params: GreeterParams | None = None):
        self._params = params if params is not None else GreeterParams()
        self._state: State = "wander"
        self._state_since = 0.0
        self._pending_reason: str | None = None

    @property
    def state(self) -> State:
        """The current phase."""
        return self._state

    def reset(self) -> None:
        """Return to a clean initial ``wander`` state, for a fresh greeter run."""
        self._state = "wander"
        self._state_since = 0.0
        self._pending_reason = None

    def step(self, observation: GreeterObservation, now: float) -> GreeterSnapshot:
        """Fold one tick in and return the resulting phase snapshot.

        Args:
            observation: This tick's perception.
            now: Current monotonic timestamp.

        Returns:
            The phase snapshot; ``entered`` is True when the phase changed.
        """
        self._pending_reason = None
        nxt = self._next_state(observation, now)
        entered = nxt != self._state
        if entered:
            self._state = nxt
            self._state_since = now
        reason = self._pending_reason if entered else None
        return GreeterSnapshot(
            self._state, _MESSAGES[self._state], entered, reason)

    def _next_state(self, obs: GreeterObservation, now: float) -> State:
        if self._state == "wander":
            return "approach" if obs.target_acquired else "wander"
        if self._state == "approach":
            if self._engage_timed_out(now):
                return "wander"
            if obs.subject_visible and self._safe_to_greet(obs):
                return "greet"
            if obs.reached_destination and not obs.subject_visible:
                # Drove to the subject's last-known spot but it is not there
                # (it moved or was a bad fix). Skip it like a stalled engagement.
                self._pending_reason = "lost"
                return "wander"
            return "approach"
        if self._state == "greet":
            if now - self._state_since >= self._params.greet_duration_s:
                return "cooldown"
            return "greet"
        # cooldown
        if now - self._state_since >= self._params.cooldown_duration_s:
            return "wander"
        return "cooldown"

    def _within_standoff(self, obs: GreeterObservation) -> bool:
        if obs.distance_m is None:
            return False
        limit = self._params.standoff_m + self._params.standoff_tolerance_m
        return obs.distance_m <= limit

    def _safe_to_greet(self, obs: GreeterObservation) -> bool:
        """At the standoff and no closer than the safe minimum (dog footprint)."""
        if obs.distance_m is None:
            return False
        return obs.distance_m >= self._params.min_safe_distance_m and (
            self._within_standoff(obs))

    def _engage_timed_out(self, now: float) -> bool:
        if now - self._state_since > self._params.engage_timeout_s:
            self._pending_reason = "stuck"
            return True
        return False
