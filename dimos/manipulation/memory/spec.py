""" Memory2-native perception for manipulation.


Architecture
------------

::

    camera
      |
      v
    RGBDCameraRecorder  ==>  color_image  ──auto-subscribe──┐
                        ==>  depth_image                    |
                        ==>  camera_info                    v
                                                      SemanticSearch
                                                      (continuous CLIP,
                                                       brightness/sharpness
                                                       filtered)
                                                            |
                                                            v
                                                     color_image_embedded
                                                            |
              ┌──────────── pulled on trigger ──────────────┘
              v
    LazyPerceptionModule  ◀── watched_names: set[str] ──┐
      | @skill find_objects(prompts)                    |
      | + startup scan (one-time on boot)               |
      | + 10s heartbeat:                                |
      |     scan(default_prompts + watched_names)       |
      | composes: CLIP search → peaks → VLM → 3D        |
      |                                                 |
      | raw_detections: list[DetObject]                 |
      v                                                 |
    ObjectMemoryTracker ─────────────────────────────────┘
      | object_observations (private)
      | object_events       (public API)
      |
      v
    tracked_objects: list[DetObject]
      |
      v
    PickAndPlaceModule   (manipulation — no API change)

    events ─→ @skill recall(name)

    All streams live in one shared SQLite store (recording.db).


The lazy pipeline (LazyPerceptionModule)
----------------------------------------

Inside ``find_objects(prompts)``, the heartbeat callback, and the
one-time startup scan:

1. Split comma-separated prompts → run one class at a time. Moondream's
   ``query_detections`` sets every detection's ``.name`` to the query
   string, so per-class is required for correct labeling.

2. CLIP-embed the prompt → query vector.

3. ``color_image_embedded.search(query_vec).order_by("ts").materialize()``
   — pull a similarity-ranked, time-sorted result set.

4. ``.transform(peaks(...))`` — extract temporal *peaks* (local maxima
   of similarity over time), so the VLM gets diverse candidate frames
   rather than near-duplicates of the same scene.

5. Filter peaks by ``lookback_window_s`` and ``min_peak_score``, cap
   at ``max_frames_per_call``.

6. For each surviving peak frame: pull aligned depth and the latest
   camera_info, run VLM detection, project via
   ``Object.from_2d_to_list`` (camera→world transform from the
   recorder's pose).

7. Publish the resulting ``list[Object]`` on ``detections_out``.


Startup workspace scan + dynamic watch list
-------------------------------------------

Two mechanisms keep the tracker's lifecycle model coherent for any
object in the scene, not just the ones the agent happens to query for.

**1. Startup scan (one-time on boot).** Shortly after ``start()``
(default ~5s, to let SemanticSearch populate the embedding index),
``LazyPerceptionModule`` runs a one-time scan with
``startup_prompts``. Each prompt is processed through the same lazy
pipeline. Detected objects flow into the tracker → seed identities
appear in ``tracked_objects`` immediately, without waiting for the
agent to ask.

``startup_prompts`` is blueprint-configured (curated for the
workspace, e.g. ``["cup", "bowl", "plate", "fork", "knife"]`` for a
kitchen setup). With an empty list, the startup scan is skipped.

**2. Dynamic watch list (every heartbeat).** The tracker publishes
``watched_names: Out[set[str]]`` — the union of voted names in
``_state`` (currently tracked) and ``_lost`` (within
``recent_lost_window_s``). ``LazyPerceptionModule`` subscribes via
its ``watched_names_in`` port and caches the latest set.

Each heartbeat scans the union ``default_prompts ∪ watched_names``.
So once any object enters the tracker — via agent call, startup
scan, or earlier heartbeat — its name is automatically added to the
heartbeat's watch list. LOST events fire when objects actually leave
the scene (heartbeat scanned and didn't find them), not just because
they fell off the static prompt list.

**Why this matters.** Without these two mechanisms, the lifecycle
model is technically there but operationally broken for any object
outside ``default_prompts``. The agent calls ``find_objects("fork")``,
fork appears, ~45s later LOST fires as a false negative because
nothing was looking for the fork anymore. The dynamic watch list
closes that gap; the startup scan ensures the tracker isn't empty on
boot.


The tracker (ObjectMemoryTracker) 
-----------------------------------------------------

The tracker is **detector-agnostic** — anything publishing
``list[DetObject]`` on ``raw_detections`` works. v3 just plugs
``LazyPerceptionModule`` in front of it; the contract is the same.

**Belief model.** For each tracked object::

    confidence(now) = exp(-(now - last_seen_ts) / time_constant_s)

Pure function of time since the last detection. Three bands::

    confident   c >= active_threshold (0.5)  -> in published snapshot
    tentative   match_threshold <= c < active -> still match-eligible
                                                  (occlusion robustness)
    lost        c < lost_threshold    (0.1)  -> emit LOST event,
                                                  move to _lost bucket

With ``time_constant_s = 15`` and ``background_scan_period_s = 10``:
LOST fires roughly ``tau * ln(10) + heartbeat_period ≈ 45s`` after an
object actually disappears, gated by ``lookback_window_s`` (the lazy
pipeline's recency filter on peak frames).

**Four-tier match.** Per detection, the matcher tries in order:

1. ``track-id`` — O(1) lookup. YOLOE assigned ids; under v3 VLM
   detections have ``track_id = -1`` so this tier is mostly inert.
2. ``tight spatial`` — within ``distance_threshold``, name-agnostic.
   Absorbs label flicker.
3. ``drift`` — wider ``reacquire_radius``, same voted name, against
   ``_state``. Silent (no event). Handles "object moved while
   watched."
4. ``re-acquisition`` — wider radius, same voted name, against
   ``_lost``. Emits ``MOVED``. Handles "object briefly disappeared
   and came back somewhere else."

No match → ``APPEARED`` event, new identity.


Streams — six total in one shared SQLite store
----------------------------------------------

Sensor streams (recorder writes, lazy reads, everyone replays):

    color_image            JPEG-encoded RGB frames + world pose
    depth_image            depth frames (one row per camera tick)
    camera_info            intrinsics (rarely change, .last() suffices)

Embedding stream (SemanticSearch writes, lazy reads):

    color_image_embedded   CLIP vector + the image bytes, vec0-indexed
                           for similarity search

Tracker streams:

    object_observations    private — one row per matched detection,
                           drives the voted-name histogram and the
                           cache rebuild on restart
    object_events          public API — APPEARED / PROMOTED /
                           LABEL_CHANGED / MOVED / LOST. Consumed by
                           recall(name), audit, monitoring.


Open vocab + cross-session memory
---------------------------------

The system is fully open-vocab. The agent passes any natural-language
description to ``find_objects``; the VLM handles it. The system is
*reactively* open-vocab on the agent side, and *proactively* open-vocab
on the heartbeat side once an object has entered the tracker (via the
dynamic watch list described above).

Three sources of prompts feed the lazy pipeline:

- **Agent-supplied** (per-call) — argument to ``find_objects``.
- **Static seed** — ``default_prompts`` from the blueprint config;
  scanned by every heartbeat regardless of tracker state. Use this
  for objects you want monitored even before they've been seen
  (alerting use cases).
- **Dynamic** — names currently in the tracker (``watched_names``);
  automatically added once an object is observed. Use this for
  "keep tracking whatever the agent discovered."

Cross-session memory: ``tracker.start()`` synchronously replays both
tracker streams (``stream.to_list()``) before accepting new detections.
The tracker recovers all identities, lost-bucket state, and label
histograms from the durable record. ``recall(name)`` queries
``object_events.tags(name=name).last()`` directly — works after
process restart because memory2 is the source of truth. After replay,
the tracker re-publishes ``watched_names`` so the heartbeat picks up
where the previous session left off.


"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Protocol, runtime_checkable

from pydantic import Field

from dimos.memory2.module import MemoryModuleConfig, RecorderConfig
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.agents.annotation import skill  # noqa: F401  (referenced in docstrings)
    from dimos.core.stream import In, Out
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection3d.object import Object as DetObject


EventKind = Literal["APPEARED", "PROMOTED", "LABEL_CHANGED", "MOVED", "LOST"]


# --------------------------------------------------------------------- streams

STREAM_OBSERVATIONS = "object_observations"
STREAM_EVENTS = "object_events"


# ----------------------------------------------------------------- data shapes


@dataclass
class ObjectObservation:
    """Payload of one observation in the ``object_observations`` stream. 
    **Private, internal for ObjectMemoryTracker**
    """

    object_id: str           # tracker's identity (NOT detection.object_id)
    detection: DetObject     # perception Object — stored as-is


@dataclass
class ObjectEvent:
    """Payload of one lifecycle event in the ``object_events`` stream.

    **Public API**
    """

    kind: EventKind
    object_id: str           # tracker's identity
    name: str                # voted name at the time of the event
    detection: DetObject     # snapshot of the relevant Object — stored as-is
    details: dict[str, Any] = field(default_factory=dict)


# =============================================================================
#  RGBDCameraRecorder — records color / depth / intrinsics to memory2
# =============================================================================


class RGBDCameraRecorderConfig(RecorderConfig):
    """Config for the generic RGBD-camera recorder."""

    db_path: str | Path = "recording.db"


@runtime_checkable
class RGBDCameraRecorderSpec(Protocol):
    """Protocol for the generic RGBD-camera recorder."""

    config: RGBDCameraRecorderConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]


# =============================================================================
#  LazyPerceptionModule — agent-callable lazy detector + heartbeat
# =============================================================================


class LazyPerceptionModuleConfig(MemoryModuleConfig):
    """Config for the lazy perception module."""

    db_path: str | Path = "recording.db"     # shared with Recorder + SemanticSearch

    # Models (blueprint can override)
    vlm_provider: type[VlModel] = MoondreamVlModel
    embedding_model: type[EmbeddingModel] = CLIPModel

    # Heartbeat — gives lifecycle events real-world timing.
    # Set to 0 to disable; lifecycle then fires only on agent calls.
    background_scan_period_s: float = 10.0

    # Static seed prompts the heartbeat ALWAYS scans for, regardless of
    # tracker state. Use for "alert me if X appears" patterns. The
    # heartbeat also scans tracker.watched_names dynamically, so curated
    # objects appearing in the scene get tracked even if not in this list.
    default_prompts: list[str] = Field(default_factory=list)

    # Startup workspace scan — runs once after start() to seed the tracker
    # with whatever is already in view. Disabled if startup_prompts is empty.
    startup_workspace_scan: bool = True
    startup_prompts: list[str] = Field(default_factory=list)
    startup_delay_s: float = 5.0             # wait this long after start() for SemanticSearch

    # Lazy pipeline tuning
    lookback_window_s: float = 60.0          # peaks must be within this recent window
    peak_distance: float = 1.0               # peaks() distance kwarg (seconds)
    peak_prominence: float = 0.02            # peaks() prominence kwarg
    min_peak_score: float = 0.20             # absolute floor on peak similarity
    max_frames_per_call: int = 2             # cap VLM calls per (scan × prompt class)


@runtime_checkable
class LazyPerceptionModuleSpec(Protocol):
    """Protocol for the lazy perception module."""

    config: LazyPerceptionModuleConfig

    # Inputs
    watched_names_in: In[set[str]]
    """Subscribed to ``ObjectMemoryTracker.watched_names``. Cached
    locally; on each heartbeat the latest cached set is unioned with
    ``default_prompts`` to determine what to scan for. Initial state
    is empty (heartbeat only scans ``default_prompts``) until the
    tracker publishes its first update."""

    # Outputs
    detections_out: Out[list[DetObject]]
    """Wired by the blueprint to ``ObjectMemoryTracker.raw_detections``."""

    def find_objects(self, prompts: str) -> str:
        """Run the lazy detection pipeline.

        Decorated ``@skill`` in the implementation. ``prompts`` is a
        comma-separated list of natural-language object classes; the
        pipeline splits, embeds, searches, peaks, VLM-detects, and
        publishes ``list[Object]`` for the tracker.

        Returns a human-readable summary string. Bounded to recent
        frames by ``lookback_window_s`` — for older lookups use the
        tracker's ``recall(name)`` skill.
        """
        ...


# =============================================================================
#  ObjectMemoryTracker — v1 lifecycle + new watched_names port for v3
# =============================================================================


class ObjectMemoryTrackerConfig(MemoryModuleConfig):
    """Config for the object lifecycle tracker."""

    # Spatial matching
    distance_threshold: float = 0.2          # Tier 2 match radius (m)
    reacquire_radius: float = 1.0            # Tier 3 & 4 match radius (m)

    # Identity promotion
    min_detections_for_permanent: int = 6    # observations before PROMOTED fires

    # Continuous-belief existence model
    time_constant_s: float = 15.0            # decay time-constant in confidence(now) = exp(-Δt/τ)
    active_threshold: float = 0.5            # ≥ this: published in tracked_objects
    match_threshold: float = 0.2             # ≥ this: still match-eligible (tentative band)
    lost_threshold: float = 0.1              # < this: emit LOST, move to _lost
    recent_lost_window_s: float = 60.0       # how long _lost is searchable for reacquire

    # Persistence
    db_path: str | Path = Field(default="recording.db")


@runtime_checkable
class ObjectMemoryTrackerSpec(Protocol):
    """Protocol for the memory2-backed object lifecycle tracker.

    On start, opens two memory2 streams (observations + events),
    rebuilds its in-RAM cache via synchronous replay, and begins
    consuming raw detections. Publishes confident objects to
    manipulation, exposes cross-session recall as an agent skill, and
    publishes the union of currently-tracked and recently-lost voted
    names so the upstream detector can scan for them automatically.

    Detector-agnostic: anything publishing ``list[DetObject]`` on
    ``raw_detections`` works.
    """

    config: ObjectMemoryTrackerConfig

    # Inputs
    raw_detections: In[list[DetObject]]
    """Per-scan batch of detections."""

    # Outputs
    tracked_objects: Out[list[DetObject]]
    """Confident objects (confidence ≥ ``active_threshold``) published
    per scan."""

    object_events: Out[ObjectEvent]
    """Live lifecycle events."""

    watched_names: Out[set[str]]
    """Union of voted names in ``_state`` (currently tracked) and
    ``_lost`` (within ``recent_lost_window_s``). Re-published on every
    cache update so subscribers (e.g., ``LazyPerceptionModule``'s
    heartbeat) automatically scan for newly-discovered objects.
    Republished once after the start-time replay so the heartbeat
    picks up cross-session state."""

    def recall(self, name: str) -> str:
        """Where did I last see something I labelled <name>?

        Decorated ``@skill`` in the implementation. Queries
        ``object_events.tags(name=name).last()`` — works after process
        restart because memory2 is the source of truth, not the
        in-RAM cache. Unbounded recency (full event history).

        Returns a human-readable summary, or
        ``"I have no memory of any <name>."`` when the events stream
        has no matching entry.
        """
        ...
