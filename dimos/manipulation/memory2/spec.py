""" Memory2-native perception for manipulation.


Architecture
------------

::

    camera
      |
      v
    RGBDCameraRecorder
      ├── records: color_image, depth_image, camera_info
      └── continuous CLIP embed (brightness/sharpness filtered)
                → color_image_embedded
              |
              ┌──────────── pulled on trigger (read-only) ──┐
              v                                             |
    LazyPerceptionModule                                    |
      | @skill find_objects(prompt)                         |
      |        .search(vec).filter(sim>=thr).order_by("ts", desc).first()
      |        → VLM detect → 3D project                    |
      | @skill find_objects_near(prompt, x, y, z, radius)   |
      |        .near((x,y,z), r).search(vec).filter(...).order_by(...).first()
      |        → VLM detect → 3D project                    |
      | @skill recall(name)                                 |
      |        .search(vec).filter(sim>=thr).order_by("ts", desc).first()
      |        → return camera pose + timestamp (no VLM, cheaper)
      |                                                     |
      | objects: Out[list[DetObject]]                       |
      v                                                     |
    PickAndPlaceModule  (manipulation — In[objects])        |
                                                            |
    Shared SQLite store (recording.db) ─────────────────────┘
    Recorder writes; LazyPerceptionModule opens its own SqliteStore
    on the same file (read-only via WAL).

    Why recording + embedding in ONE module: memory2's SubjectNotifier
    is in-memory per-store, so a separate-module embedder couldn't
    receive .live() notifications from the recorder's writes even on
    the same db_path. Keeping them in one module = one store = one
    notifier = live pipeline works. The LazyPerceptionModule reads
    only at skill-call time (no .live()), so cross-instance SQLite
    reads (WAL) are sufficient there.


Three skills, three query shapes
--------------------------------

Each skill is a one-line composition of memory2 primitives. Every
skill returns the **most recent confident match** along with its
timestamp — the agent sees how fresh the data is and decides if it's
fresh enough to act on. No time-window parameter on the API; "current"
is whatever memory2 has most recently.

- ``find_objects(prompt)`` — **semantic + most recent**. Search the
  embedding stream, filter to confident matches, take the most recent.
  Run VLM + 3D projection on that frame. Returns ``list[Object]`` (via
  the ``objects`` port) plus formatted summary with timestamp.

- ``find_objects_near(prompt, x, y, z, radius)`` — **spatial + semantic
  + most recent**. Same as ``find_objects`` plus a camera-pose proximity
  filter. NOTE on indexing: memory2 currently runs vector ``.search()``
  as a top-K vec0 query and applies the spatial ``.near()`` predicate in
  Python afterward — the R*Tree index does NOT pre-filter the embedding
  search. Correct results, but for very long recordings the top-K cap
  can elide spatially-near low-similarity frames. Pushing R*Tree
  pre-filtering through ``Backend._vector_search`` is a memory2 follow-up.

- ``recall(name)`` — **cheaper cousin of find_objects**. Same query
  shape but skips VLM detection and 3D projection; returns the camera
  pose at the matching frame plus timestamp. Use for "where was I when
  I saw X" questions where exact 3D object pose isn't needed.

**Freshness contract is in the response.** Every result includes
``(seen N seconds ago)`` so the agent can reason: "2 seconds is fresh
— act"; "5 minutes is stale — re-query or skip."


The lazy pipeline
------------------

All three skills share the same shape:

1. CLIP-embed the prompt → query vector.

2. Build a memory2 query on ``color_image_embedded`` by composing the
   relevant filters (``.near`` / ``.search`` / ``.filter`` /
   ``.order_by``). ``.search`` pushes through the vec0 vector index;
   tag/time/SQL-expressible filters push to SQL. ``.near`` runs as a
   Python post-filter after vector search (R*Tree pre-gating of vector
   search is a memory2 follow-up).

3. Take ``.first()`` — the most recent confident match. Single
   observation.

4. (find_objects / find_objects_near only:) Pull aligned depth via
   ``.at(obs.ts, tolerance)`` and latest intrinsics via ``.last()``.
   Run VLM detection on the color frame, project via
   ``Object.from_2d_to_list`` (camera→world transform from the
   recorder's pose).

5. Publish ``list[Object]`` on ``objects``. Return a formatted summary
   that **includes the observation timestamp** so the agent can read
   freshness.

**Snapshot contract.** Every publish on ``objects`` replaces
``PickAndPlaceModule._detection_snapshot`` — the cache ``pick`` /
``place`` read. So:

- A successful ``find_objects("cup")`` → snapshot = [cup objects]; ``pick("cup")`` works.
- An interleaved ``find_objects("apple")`` returning 0 matches → snapshot = []; later
  ``pick("cup")`` fails ("not found"). Re-query before acting.
- This is intentional. Each ``find_objects`` call is a fresh world model, not an
  accumulating one. The agent prompt makes this explicit.

``PickAndPlaceModule`` reads ``objects`` to act on named
targets. Multiple instances in the result are handled either by
prompt qualifiers ("cup on the right" → VLM returns one detection in
the frame) or by the agent reading the returned positions and
choosing.


Streams (recorded continuously)
-------------------------------

::

    color_image            JPEG-encoded RGB frames + world pose
    depth_image            depth frames, LOSSLESS lz4+lcm codec
    camera_info            intrinsics (rarely change, .last() suffices)
    color_image_embedded   CLIP vector + image bytes, vec0-indexed
                           for similarity search (populated by the
                           recorder's continuous embed pipeline)

**depth_image MUST be recorded losslessly.** memory2's codec
auto-dispatch maps every ``Image``-typed stream to ``JpegCodec``
(lossy 8-bit DCT) — correct for RGB color, but it shreds uint16
depth (millimeters). JPEG'd depth → no coherent 3D points →
``from_2d_to_list`` returns ``[]`` → ``find_objects`` silently
returns nothing even though CLIP + VLM succeeded. ``RGBDCameraRecorder``
overrides the depth stream's codec to ``lz4+lcm`` via memory2's
public per-stream ``codec=`` override (no memory2 changes). The codec
is persisted per-stream in the SQLite registry, so an existing
recording created before this fix keeps its JPEG depth — delete the
db file once to recreate the stream with the lossless codec.

**depth_image must be converted mm→m before projection.** RealSense
publishes ``DEPTH16`` (uint16 millimeters). The live OSR path converts
to float32 meters inline before ``from_2d_to_list``
(``object_scene_registration.py:295-301``). The memory2-native path
replays the *raw recorded* ``DEPTH16``, so ``LazyPerceptionModule``
replicates that exact conversion (format-aware: only ``DEPTH16`` is
divided by 1000) before projecting. Skipping it makes every point
exceed ``depth_trunc`` → empty pointcloud → ``find_objects`` returns
nothing. This is the replay-path analogue of the codec issue: anything
OSR does inline on live frames, the memory2-native path must redo on
replayed frames.


Open vocab + cross-session memory
---------------------------------

The agent passes any natural-language description to ``find_objects``
or ``find_objects_near`` (``"red mug with handle"``,
``"the screwdriver near the laptop"``). The VLM handles visual
disambiguation; the agent can encode spatial qualifiers in the prompt,
or call ``find_objects_near`` for camera-pose-bounded queries (Python
spatial post-filter; see the ``find_objects_near`` note above for the
indexing limitation).

Cross-session memory is implicit: memory2 persists every observation
in ``recording.db``. Skills query the full embedding history regardless
of when the current process started — no replay logic, no bespoke
loaders. ``recall(name)`` returning "(seen 3 hours ago)" is normal and
correct after a process restart; the agent reads the timestamp and
decides.

``RGBDCameraRecorder`` uses **resume-if-exists**: it opens ``recording.db``
if the file is there (cross-session memory works out of the box) and
creates it fresh if it isn't. ``RecorderConfig.overwrite`` is inert in
this subclass — see ``recorder.py``. Users wanting a clean slate delete
the file manually before launching.

"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.memory2.module import MemoryModuleConfig, RecorderConfig
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel

if TYPE_CHECKING:
    from dimos.agents.annotation import skill  # noqa: F401  (referenced in docstrings)
    from dimos.core.stream import In, Out
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.detection.type.detection3d.object import Object as DetObject


# =============================================================================
#  RGBDCameraRecorder — records color / depth / intrinsics to memory2
# =============================================================================


class RGBDCameraRecorderConfig(RecorderConfig):
    """Config for the RGBD-camera recorder + continuous embedder."""

    db_path: str | Path = "recording.db"
    embedding_model: type[EmbeddingModel] = CLIPModel


@runtime_checkable
class RGBDCameraRecorderSpec(Protocol):
    """Protocol for the RGBD-camera recorder + continuous embedder."""

    config: RGBDCameraRecorderConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]


# =============================================================================
#  LazyPerceptionModule — agent-callable open-vocab perception
# =============================================================================


class LazyPerceptionModuleConfig(MemoryModuleConfig):
    """Config for the lazy perception module."""

    db_path: str | Path = "recording.db"

    # Models (blueprint can override)
    vlm_provider: type[VlModel] = MoondreamVlModel
    embedding_model: type[EmbeddingModel] = CLIPModel

    # Similarity threshold — observations below this don't count as a
    # confident match for find_objects / find_objects_near.
    min_similarity: float = 0.20

    # 3D projection knobs threaded into ``Object.from_2d_to_list``.
    max_distance: float = 1.0
    use_aabb: bool = True
    max_obstacle_width: float = 0.06


@runtime_checkable
class LazyPerceptionModuleSpec(Protocol):
    """Protocol for the lazy perception module."""

    config: LazyPerceptionModuleConfig

    objects: Out[list[DetObject]]
    """Wired by the blueprint to ``PickAndPlaceModule.objects``. Cache
    of the most recent ``find_objects`` / ``find_objects_near`` result.
    Manipulation reads from here to act on the named target. Named
    ``objects`` to match the existing manipulation In port for
    autoconnect-by-name."""

    def find_objects(self, prompt: str) -> str:
        """Find objects matching ``prompt``. Returns the most recent
        confident match with timestamp.

        Decorated ``@skill`` in the implementation. Composes
        ``.search()`` + ``.filter(similarity >= min_similarity)`` +
        ``.order_by("ts", desc=True)`` + ``.first()`` over
        ``color_image_embedded``, then runs VLM detection on the
        matching frame and projects to 3D. Publishes ``list[Object]``
        on the ``objects`` port.

        Open-vocab: ``prompt`` can be any natural language
        (``"red mug"``, ``"the cup on the right"``). Comma-separated
        prompts are split and processed per class because VLM
        ``query_detections`` labels every result with the literal
        query string.

        Returns a human-readable summary including the observation
        timestamp (``"(seen 3s ago)"``) so the agent can judge
        freshness. Returns a no-match message when no observation
        meets ``min_similarity``.
        """
        ...

    def find_objects_near(
        self,
        prompt: str,
        x: float,
        y: float,
        z: float,
        radius: float = 1.0,
    ) -> str:
        """Find objects matching ``prompt``, in frames recorded when
        the camera was within ``radius`` meters of ``(x, y, z)``.

        Decorated ``@skill`` in the implementation. Same query shape
        as ``find_objects`` plus an extra ``.near((x, y, z), radius)``
        predicate. ``.near()`` runs as a Python post-filter today —
        memory2's R*Tree index does not pre-gate the vector search.
        Take most recent confident match → VLM → 3D project → publish
        on the ``objects`` port.

        ``.near()`` filters by the camera's pose at record time, NOT
        by the detected object's position. Use for "frames captured
        while looking at this workspace area"; object-position
        filtering is downstream of detection.
        """
        ...

    def recall(self, name: str) -> str:
        """Where did I last see something matching ``name``?

        Decorated ``@skill`` in the implementation. Cheaper cousin of
        ``find_objects``: composes ``.search()`` +
        ``.order_by("ts", desc=True)`` + ``.first()`` over
        ``color_image_embedded`` — returns the most recent confident
        semantic match across the full embedding history, but **does
        not run VLM**. Returns the camera pose at the matching frame
        plus timestamp.

        Works after process restart because memory2's SQLite store is
        the persistence layer; the query touches the same db that
        previous sessions wrote.

        Returns a human-readable summary with timestamp
        (``"Last saw 'cup' near (1.0, 0.5, 0.9) (seen 3min ago)"``), or
        ``"No memory of <name>."`` when nothing matches.
        """
        ...
