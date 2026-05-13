"""Object Memory Tracker — specification.

A memory2-backed object tracker that takes over the identity-management
role currently done in-RAM by ``ObjectDB``. Solves two practical
problems with the current per-frame tracker:

1. **Stable labels across scans.** YOLO classifies the same physical
   object as different classes between frames ("trash can" /
   "basket" / "container"). Today, ``Object.update_object``
   overwrites the name on every match (``object.py:88``), so the
   public label is most-recent-wins. The tracker votes across all
   detections that contributed to an identity.

2. **Memory between actions.** Today, ``ObjectDB`` has no lifecycle management on
   objects, no notion of confidence, and no persistence
   across process restarts. Move an object A → B and the system sees
   two objects. Lose one scan to camera occlusion and risk a duplicate
   identity. Restart the process and the workspace memory (due to storing in RAM) is gone.

   This tracker models object existence as a **continuous belief** that
   decays without detections and is reinforced by them. Memory2 is
   the source of truth; a synchronous replay on ``start()`` rebuilds
   the in-RAM cache from memory2 before any new detections arrive, so
   the robot has memory between sessions.


Architecture
------------

::

    camera
      |
      v
    ObjectSceneRegistrationModule          (detection only — no ObjectDB)
      |
      | raw_detections: list[DetObject]
      v
    ObjectMemoryTracker  --tracked_objects-->  PickAndPlaceModule
       |           ^
       |           | sync replay on start() rebuilds cache from memory2
       v           |
    +-- memory2 (source of truth) ------+
    | object_observations (dense)        |
    | object_events       (sparse)       |
    +------------------------------------+
                  |
                  v
            @skill recall(name)


Belief model
------------

For each tracked object::

    confidence(now) = exp(-(now - last_seen_ts) / time_constant_s)

Pure function of time since the last detection. Three thresholds split
the range::

    confident   c >= active_threshold  (0.5)   -> in published snapshot
    tentative   match_threshold <= c < active  -> still match-eligible
                                                   (this band gives
                                                    occlusion robustness)
    lost        c < lost_threshold     (0.1)   -> emit LOST event,
                                                   move to _lost bucket

With ``time_constant_s = 15``: a single missed scan at 1-5 s cadence
barely moves the needle; sustained ~30 s absence drops into tentative;
~45 s absence flips to lost.


Match — four tiers, Python over the cache
-----------------------------------------

Per detection, the matcher tries (in order, falling through on miss):

1. **track-id** — YOLOE's tracker assigns ids that survive short
   occlusions. O(1) lookup against ``_state``.
2. **tight spatial** — within ``distance_threshold`` of any object
   whose confidence is at least ``match_threshold`` (active OR
   tentative). Name-agnostic to absorb YOLO label flicker.
3. **drift** — wider ``reacquire_radius``, same voted name, against
   ``_state``. Handles "object moved slowly while watched."
4. **re-acquisition** — wider radius, same voted name, against
   ``_lost``. Handles "object briefly disappeared and came back
   somewhere else (moved)."

No match → APPEARED event, new identity. Matching is in-process
Python with ram cache; memory2 is only written to, not queried, on this path.


Data flow / streams
-------------------

Every matched detection appends to ``object_observations``. Lifecycle
transitions (APPEARED / PROMOTED / LABEL_CHANGED / MOVED / LOST)
append to ``object_events``. The in-RAM cache (``_state`` for active
identities, ``_lost`` for recently-lost) is updated **inline** on every
write — the emit helpers ``append`` to memory2 and then synchronously
apply the same record to the cache. memory2 holds the durable record;
the cache is a write-through index over it.

On process ``start()``, the cache is rebuilt by **synchronous replay**
of both streams (``stream.to_list()``), completing before the tracker
accepts any new detections. This gives cross-session memory without
any async-backfill race.

"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Protocol, runtime_checkable

from pydantic import Field

from dimos.memory2.module import MemoryModuleConfig

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.agents.annotation import skill 
    from dimos.core.stream import In, Out
    from dimos.perception.detection.type.detection3d.object import Object as DetObject


EventKind = Literal["APPEARED", "PROMOTED", "LABEL_CHANGED", "MOVED", "LOST"]


# --------------------------------------------------------------------- streams

STREAM_OBSERVATIONS = "object_observations"
STREAM_EVENTS = "object_events"


# ----------------------------------------------------------------- data shapes


@dataclass
class ObjectObservation:
    """Payload of one observation in the ``object_observations`` stream."""

    object_id: str           # tracker's identity (NOT detection.object_id)
    detection: DetObject     # perception Object — stored as-is


@dataclass
class ObjectEvent:
    """Payload of one event (lifecycle transition) in the ``object_events`` stream."""

    kind: EventKind
    object_id: str           # tracker's identity (NOT detection.object_id)
    name: str                # voted name at the time of the event
    detection: DetObject     # snapshot of the relevant Object — stored as-is
    details: dict[str, Any] = field(default_factory=dict)


# ----------------------------------------------------------------------- config


class ObjectMemoryTrackerConfig(MemoryModuleConfig):
    """Config for tracker."""

    # Spatial matching
    distance_threshold: float = 0.2          # Tier 2 match radius(m), the distance close enough to believe it's the same object
    reacquire_radius: float = 1.0            # Tier 3 & 4 match radius(m). Used for drift (object moved while watched) and re-acquisition (object came back after being lost or moved).

    # Identity promotion
    min_detections_for_permanent: int = 6    # minimum number of detections to promote an object

    # Continuous-belief existence model
    time_constant_s: float = 15.0            # decay time-constant in confidence(now) = exp(-Δt/τ)
    active_threshold: float = 0.5            # ≥ this: published in tracked_objects
    match_threshold: float = 0.2             # ≥ this: still match-eligible (tentative band)
    lost_threshold: float = 0.1              # < this: emit LOST, move to _lost
    recent_lost_window_s: float = 60.0       # how long _lost is searchable for reacquire

    # Persistence
    db_path: str | Path = Field(default="manipulation_object_memory.db")


# ---------------------------------------------------------------------- protocol


@runtime_checkable
class ObjectMemoryTrackerSpec(Protocol):
    """Protocol for the memory2-backed object lifecycle tracker.

    On start, opens two memory2 streams (observations + events), rebuilds
    its in-RAM cache via synchronous replay from past sessions, and begins
    consuming raw detections. Publishes confident objects to manipulation
    and exposes cross-session recall as an agent skill.
    """

    config: ObjectMemoryTrackerConfig

    # Inputs
    raw_detections: In[list[DetObject]]
    """Per-scan batch of detections."""

    # Outputs
    tracked_objects: Out[list[DetObject]]
    """Confident objects (confidence ≥ ``active_threshold``) published per scan"""

    object_events: Out[ObjectEvent]
    """Live lifecycle events."""

    def recall(self, name: str) -> str:
        """Where did I last see something I labelled <name>?

        Decorated ``@skill`` in the implementation. Queries
        ``object_events.tags(name=name).last()`` — works after process
        restart because memory2 is the source of truth, not the in-RAM
        cache.

        Returns a human-readable summary, or
        ``"I have no memory of any <name>."`` when the events stream
        has no matching entry.
        """
        ...
