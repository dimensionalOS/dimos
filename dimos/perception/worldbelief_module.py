# Copyright 2025-2026 Dimensional Inc.
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

"""Query plane of the WorldBelief stack: on-demand ``scan``/``recall`` MCP skills over
the recording written by :class:`WorldBeliefRecorder`. Owns the warm belief fold and the
scan/CLIP models; the recorder stays a dumb writer."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any, ClassVar

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.memory2.module import MemoryModuleConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.detection.world_belief import WorldBelief, WorldBeliefConfig
from dimos.perception.worldbelief_recorder import WorldBeliefRecorder
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.scene_scan import SceneScanner

_STATE_DIR = STATE_DIR / "worldbelief"
_HISTORY_PATH = _STATE_DIR / "worldbelief_history.db"

# Default prompts so a bare `dimos mcp call scan` detects common tabletop objects instead
# of returning [] — an open-vocab detector needs a prompt.
DEFAULT_SCAN_PROMPTS = ["mug", "coke can", "box"]

_SCAN_LOCK = threading.RLock()

logger = setup_logger()


def worldbelief_history_path() -> str:
    return str(_HISTORY_PATH)


class WorldBeliefModuleConfig(MemoryModuleConfig):
    # Recording db to READ — a config fallback; the live path comes from the recorder over
    # RPC when one is wired in (see WorldBeliefModule._recording_path).
    db_path: str | Path = _STATE_DIR / "recordings" / "worldbelief.db"
    history_path: str | Path = _HISTORY_PATH
    scan_prompts: list[str] = Field(default_factory=lambda: list(DEFAULT_SCAN_PROMPTS))
    # Identity-core tuning of the owned WorldBelief; a blueprint can override its constants.
    belief: WorldBeliefConfig = Field(default_factory=WorldBeliefConfig)


class WorldBeliefModule(Module):
    """Query plane of the WorldBelief stack: ``scan``/``recall`` on demand over the
    recording db. Owns the warm belief + scan/CLIP models; the recorder is a dumb writer.
    """

    config: WorldBeliefModuleConfig
    dedicated_worker: ClassVar[bool] = True

    detections_3d: Out[Detection3DArray]
    pointcloud: Out[PointCloud2]
    objects: Out[list[Object]]

    # Recorder writing the live recording; the coordinator injects an RPC proxy when a
    # WorldBeliefRecorder is in the blueprint. None → read config.db_path.
    _recorder: WorldBeliefRecorder | None = None
    _belief: WorldBelief | None = None

    @rpc
    def start(self) -> None:
        super().start()
        # Pre-load YOLOe/CLIP/DINO off the main path so the first scan/recall is fast.
        threading.Thread(target=self._warmup, name="worldbelief-scan-warmup", daemon=True).start()

    @rpc
    def stop(self) -> None:
        belief = getattr(self, "_belief", None)
        if belief is not None:
            try:
                belief.close()  # hand this session's galleries to a later model on the same path
            except Exception as e:
                logger.warning("belief close failed: %s", e)
        super().stop()

    def _recording_path(self) -> str:
        if self._recorder is not None:
            try:
                return str(self._recorder.recording_path())
            except Exception as e:
                logger.warning("recorder path RPC failed (using configured db_path): %s", e)
        return str(self.config.db_path)

    def _scanner(self) -> SceneScanner:
        """The one warm scan engine plus the WorldBelief it folds into — belief state
        persists across calls; galleries persist cross-session via history_path.

        Callers hold ``_SCAN_LOCK`` (warmup / scan / recall all wrap this), so lazy
        init needs no internal locking of its own."""
        scanner: SceneScanner | None = getattr(self, "_live_scanner", None)
        if scanner is None:
            from dimos.perception.scene_scan import SceneScanner as _SceneScanner

            bcfg = self.config.belief
            if bcfg.history_path is None:
                bcfg = bcfg.model_copy(update={"history_path": str(self.config.history_path)})
            self._belief = WorldBelief(bcfg)
            scanner = _SceneScanner(
                target_frame="world",
                text_prompts=list(self.config.scan_prompts),
            )
            self._live_scanner = scanner
        return scanner

    def _get_recall_clip(self) -> Any:
        """The one lazily-built CLIP model shared by memory indexing and recall search."""
        clip = getattr(self, "_recall_clip", None)
        if clip is None:
            from dimos.models.embedding.clip import CLIPModel

            clip = CLIPModel()
            clip.start()
            self._recall_clip = clip
        return clip

    def _index_recall_frames(self, read_store: Any, rec_path: str) -> None:
        """Top up the shared cross-session recall index: whole-frame CLIP vectors +
        thumbnails tagged with this recording. Incremental (watermark) and best-effort —
        memory indexing must never fail a scan."""
        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.recall import build_frame_clip_index

        try:
            since = getattr(self, "_recall_indexed_ts", None)
            with SqliteStore(path=str(self.config.history_path)) as hist_store:
                build_frame_clip_index(
                    read_store,
                    model=self._get_recall_clip(),
                    start=since,
                    index_store=hist_store,
                    source_tag=rec_path,
                )
            self._recall_indexed_ts = float(read_store.stream("color_image", Image).last().ts)
        except Exception as e:
            logger.warning("recall-index top-up skipped: %s", e)

    def _warmup(self) -> None:
        try:
            with _SCAN_LOCK:  # hold through the model load so a first scan waits, then runs warm
                self._scanner().warmup()
            logger.info("WorldBelief scan models warmed up")
        except Exception as e:
            logger.warning("scan warmup failed (first scan will be slower): %s", e)

    @skill
    def scan(self, prompt: list[str] | None = None, window: float = 60.0) -> list[dict[str, Any]]:
        """Advance the world fold to now and report the believed-present objects.

        Callable live::

            dimos mcp call scan -a prompt='["cup","bottle","box"]'

        Folds everything recorded since the previous scan; ``window`` bounds only the
        FIRST scan's lookback (0 = recording start). Publishes ``detections_3d``/
        ``pointcloud``/``objects``; the summary carries per-object trust (``confirmed`` =
        multi-viewpoint) and identity basis (``tracked``/``reacquired``/``twin-anchor``/
        ``restored``).
        """
        import copy as _copy

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.detection.type.detection3d.object import (
            aggregate_pointclouds,
            to_detection3d_array,
        )

        rec_path = self._recording_path()
        with _SCAN_LOCK:  # serialize concurrent scan/recall (shared warm belief)
            scanner = self._scanner()
            belief = self._belief
            if belief is None:  # unreachable — _scanner() builds it; narrows for mypy
                raise RuntimeError("belief not initialized")
            with SqliteStore(path=rec_path, must_exist=True) as read_store:
                # One call answers with the FULLY-CURRENT world, however long the backlog:
                # the caller asked "what is the world now?" — a partial answer needing a
                # re-ask is worse than a wait. The CLI waits accordingly
                # (`dimos mcp call --timeout`, default 600 s).
                present = scanner.scan_recent(
                    read_store,
                    belief,
                    window=window,
                    prompt=prompt or list(self.config.scan_prompts),
                )
                # every scan also tops up the cross-session recall memory, so "when did
                # I see X" is answerable for any session that ever scanned
                self._index_recall_frames(read_store, rec_path)
            # Publish COPIES: present() returns the live internal entities, which later
            # scans mutate in place — an in-process consumer holding the payload would see
            # geometry change under it (torn reads).
            present = [_copy.copy(o) for o in present]
            ts = present[0].ts if present else None
            self.detections_3d.publish(to_detection3d_array(present, frame_id="world", ts=ts))
            # detected-object clouds, colored per object id (downsampled to bound the payload)
            self.pointcloud.publish(aggregate_pointclouds(present).voxel_downsample(0.005))
            # → PickAndPlace obstacle monitor (autoconnect-bound). Publishes tentative+confirmed:
            # conservative-safe for collision; a grasp consumer must gate on trust (present_for_grasp).
            self.objects.publish(present)
            return [
                {
                    "name": o.name,
                    "id": o.object_id[:8],
                    "trust": belief.trust_of(o.object_id),
                    "basis": belief.basis_of(o.object_id),
                    "xyz": [
                        round(float(o.center.x), 3),
                        round(float(o.center.y), 3),
                        round(float(o.center.z), 3),
                    ],
                }
                for o in present
            ]

    @skill
    def recall(self, text: str, k: int = 20) -> dict[str, Any] | None:
        """Recall "when/where did I last see <text>" over every session ever recorded.

        Callable live::

            dimos mcp call recall -a text="coffee cup"

        The CLIP index lives in the shared history store, so one search spans all
        sessions; this session's new frames are indexed incrementally first. CLIP only
        proposes moments (whole-frame similarity is nearly flat) — the detector confirms
        them best-first against each moment's source recording. Returns ``{when_ts,
        where_camera, where_object, recording, clip_similarity}`` for the confirmed
        moment; ``where_object`` is None when nothing confirms (fields then describe the
        best unverified hit) or the footage is gone. ``None`` when nothing matches.
        """
        from contextlib import nullcontext

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.recall import recall as _recall

        rec_path = self._recording_path()
        with _SCAN_LOCK:  # serialize concurrent scan/recall (shared CLIP model + index watermark)
            clip = self._get_recall_clip()
            with (
                SqliteStore(path=rec_path, must_exist=True) as read_store,
                SqliteStore(path=str(self.config.history_path)) as hist_store,
            ):
                # top up memory with any frames since the last scan/recall, then search
                # the SHARED index — one query spans every session ever indexed. Object
                # localization always uses a fresh throwaway belief per moment — a past
                # window must never fold into the live one.
                self._index_recall_frames(read_store, rec_path)
                hit, obj_center = _recall(
                    hist_store,
                    text,
                    model=clip,
                    k=k,
                    detector=self._scanner()._detector,
                    open_recording=lambda rec: (
                        nullcontext(read_store)
                        if rec == rec_path
                        else SqliteStore(path=rec, must_exist=True)
                        if Path(rec).exists()
                        else None
                    ),
                )
        if hit is None:
            return None
        rec_tag = hit.tags.get("rec")
        pose = hit.pose_stamped
        return {
            "text": text,
            "when_ts": round(float(hit.ts), 3),
            "where_camera": None
            if pose is None
            else [round(pose.x, 3), round(pose.y, 3), round(pose.z, 3)],
            "where_object": None
            if obj_center is None
            else [
                round(float(obj_center.x), 3),
                round(float(obj_center.y), 3),
                round(float(obj_center.z), 3),
            ],
            "recording": Path(rec_tag).name if rec_tag else None,
            # whole-frame CLIP text-image cosine — uncalibrated; only the RANKING is meaningful
            "clip_similarity": round(float(hit.similarity or 0.0), 3),
        }
