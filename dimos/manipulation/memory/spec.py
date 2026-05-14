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
    LazyPerceptionModule
      | @skill find_objects(prompt)
      |        .search(vec).filter(sim>=thr).order_by("ts", desc).first()
      |        → VLM detect → 3D project
      | @skill find_objects_near(prompt, x, y, z, radius)
      |        .near((x,y,z), r).search(vec).filter(sim>=thr).order_by("ts", desc).first()
      |        → VLM detect → 3D project
      | @skill recall(name)
      |        .search(vec).order_by("ts", desc).first()
      |        → return camera pose + timestamp (no VLM, cheaper)
      |
      | latest_detections: Out[list[DetObject]]
      v
    PickAndPlaceModule   (manipulation — reads latest_detections)

    All streams live in one shared SQLite store (recording.db).


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
  ``latest_detections``) plus formatted summary with timestamp.

- ``find_objects_near(prompt, x, y, z, radius)`` — **spatial + semantic
  + most recent**. Same as ``find_objects`` but pre-filters via
  memory2's R*Tree index to frames recorded when the camera pose was
  within ``radius`` of ``(x, y, z)``.

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
   ``.order_by``). All filters push down to SQL / R*Tree / vec0
   indexes via memory2's store backend.

3. Take ``.first()`` — the most recent confident match. Single
   observation.

4. (find_objects / find_objects_near only:) Pull aligned depth via
   ``.at(obs.ts, tolerance)`` and latest intrinsics via ``.last()``.
   Run VLM detection on the color frame, project via
   ``Object.from_2d_to_list`` (camera→world transform from the
   recorder's pose).

5. Publish ``list[Object]`` on ``latest_detections``. Return a
   formatted summary that **includes the observation timestamp** so
   the agent can read freshness.

``PickAndPlaceModule`` reads ``latest_detections`` to act on named
targets. Multiple instances in the result are handled either by
prompt qualifiers ("cup on the right" → VLM returns one detection in
the frame) or by the agent reading the returned positions and
choosing.


Streams (recorded continuously)
-------------------------------

::

    color_image            JPEG-encoded RGB frames + world pose
    depth_image            depth frames (one row per camera tick)
    camera_info            intrinsics (rarely change, .last() suffices)
    color_image_embedded   CLIP vector + image bytes, vec0-indexed
                           for similarity search (populated by SemanticSearch)


Open vocab + cross-session memory
---------------------------------

The agent passes any natural-language description to ``find_objects``
or ``find_objects_near`` (``"red mug with handle"``,
``"the screwdriver near the laptop"``). The VLM handles visual
disambiguation; the agent can encode spatial qualifiers in the prompt,
or call ``find_objects_near`` to use memory2's R*Tree index for
camera-pose-bounded queries.

Cross-session memory is implicit: memory2 persists every observation
in ``recording.db``. Skills query the full embedding history regardless
of when the current process started — no replay logic, no bespoke
loaders. ``recall(name)`` returning "(seen 3 hours ago)" is normal and
correct after a process restart; the agent reads the timestamp and
decides.

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


@runtime_checkable
class LazyPerceptionModuleSpec(Protocol):
    """Protocol for the lazy perception module."""

    config: LazyPerceptionModuleConfig

    latest_detections: Out[list[DetObject]]
    """Wired by the blueprint to the manipulation stack
    (e.g., ``PickAndPlaceModule.objects``). Cache of the most recent
    ``find_objects`` / ``find_objects_near`` result. Manipulation
    reads from here to act on the named target."""

    def find_objects(self, prompt: str) -> str:
        """Find objects matching ``prompt``. Returns the most recent
        confident match with timestamp.

        Decorated ``@skill`` in the implementation. Composes
        ``.search()`` + ``.filter(similarity >= min_similarity)`` +
        ``.order_by("ts", desc=True)`` + ``.first()`` over
        ``color_image_embedded``, then runs VLM detection on the
        matching frame and projects to 3D. Publishes ``list[Object]``
        on ``latest_detections``.

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
        as ``find_objects`` but with an extra ``.near((x, y, z), radius)``
        filter that uses memory2's R*Tree spatial index. Otherwise
        identical: take most recent confident match → VLM → 3D project
        → publish on ``latest_detections``.

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
