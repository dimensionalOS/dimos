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

"""Memory2 recorder for the xArm6 WorldBelief hardware blueprint."""

from __future__ import annotations

from datetime import datetime
import inspect
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

from dimos.agents.annotation import skill
from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.object import Object
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.scene_scan import SceneScanner

_STATE_DIR = STATE_DIR / "worldbelief" / "xarm6"
_HISTORY_PATH = _STATE_DIR / "worldbelief_history.db"
_RECORDING_BASE_PATH = _STATE_DIR / "recordings" / "xarm6_worldbelief.db"

# Default open-vocab prompts so a bare `dimos mcp call scan` still detects common tabletop
# objects instead of silently returning [] (an open-vocab detector needs a prompt).
DEFAULT_SCAN_PROMPTS = ["mug", "coke can", "box"]

_SCAN_LOCK = threading.RLock()

logger = setup_logger()


def xarm6_worldbelief_history_path() -> str:
    return str(_HISTORY_PATH)


def _timestamped_recording_path(base: str | Path) -> Path:
    base_path = Path(base)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    candidate = base_path.with_name(f"{base_path.stem}_{timestamp}{base_path.suffix}")
    suffix = 1
    while candidate.exists():
        candidate = base_path.with_name(f"{base_path.stem}_{timestamp}_{suffix}{base_path.suffix}")
        suffix += 1
    return candidate


class XArm6WorldBeliefRecorderConfig(RecorderConfig):
    db_path: str | Path = _RECORDING_BASE_PATH
    default_frame_id: str = "world"


class XArm6WorldBeliefRecorder(Recorder):
    """Record the raw xArm6 perception log: sensor inputs + tf + proprioception.

    Records only streams that are reproducible-from-source (raw camera + tf + joint
    state); detections, embeddings, and pointclouds are derived on demand from this log,
    not stored. Recordings are inspected with the existing ``dimos mem rerun`` tooling.
    """

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    coordinator_joint_state: In[JointState]
    detections_3d: Out[Detection3DArray]
    pointcloud: Out[PointCloud2]
    objects: Out[list[Object]]
    config: XArm6WorldBeliefRecorderConfig

    @rpc
    def start(self) -> None:
        if not self.config.g.replay:
            db_path = _timestamped_recording_path(self.config.db_path)
            db_path.parent.mkdir(parents=True, exist_ok=True)
            self.config.db_path = db_path
            logger.info("xArm6 WorldBelief recording DB: %s", db_path)
        super().start()
        # Bug-4: pre-load YOLOe/CLIP/DINO off the main path so the first scan/recall is fast.
        threading.Thread(target=self._warmup, name="worldbelief-scan-warmup", daemon=True).start()

    def _scanner(self) -> SceneScanner:
        """The one warm scanner for live scans (belief + vec0 galleries persist across calls)."""
        scanner: SceneScanner | None = getattr(self, "_live_scanner", None)
        if scanner is not None:
            return scanner
        with _SCAN_LOCK:  # double-checked so warmup + a first scan can't build it twice
            scanner = getattr(self, "_live_scanner", None)
            if scanner is None:
                from dimos.perception.scene_scan import SceneScanner as _SceneScanner

                scanner = _SceneScanner(
                    target_frame="world",
                    history_path=xarm6_worldbelief_history_path(),
                    text_prompts=DEFAULT_SCAN_PROMPTS,
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

    def _index_recall_frames(self, read_store: Any) -> None:
        """Top up the shared cross-session recall index: whole-frame CLIP vectors +
        thumbnails tagged with this recording. Incremental (watermark) and best-effort —
        memory indexing must never fail a scan."""
        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.recall import build_frame_clip_index

        try:
            since = getattr(self, "_recall_indexed_ts", None)
            with SqliteStore(path=xarm6_worldbelief_history_path()) as hist_store:
                build_frame_clip_index(
                    read_store, model=self._get_recall_clip(), start=since,
                    index_store=hist_store, source_tag=str(self.config.db_path),
                )
            self._recall_indexed_ts = float(read_store.stream("color_image", Image).last().ts)
        except Exception as e:
            logger.warning("recall-index top-up skipped: %s", e)

    def _warmup(self) -> None:
        try:
            with _SCAN_LOCK:  # hold through the model load so a first scan waits, then runs warm
                self._scanner().warmup()
            logger.info("xArm6 WorldBelief scan models warmed up")
        except Exception as e:
            logger.warning("scan warmup failed (first scan will be slower): %s", e)

    @skill
    def scan(self, prompt: list[str] | None = None, window: float = 60.0) -> list[dict[str, Any]]:
        """Live query-on-demand perception: advance the world fold to now and report it.

        Callable on the running blueprint::

            dimos mcp call scan -a prompt='["cup","bottle","box"]'

        Folds the complete recorded interval since the previous scan (adaptive density —
        see :mod:`dimos.perception.scene_scan`); ``window`` only bounds the FIRST scan's
        lookback (0 = recording start; may exceed an MCP timeout past a few minutes).
        Publishes the believed-present objects to ``detections_3d``/``pointcloud``/
        ``objects`` and returns a summary with per-object trust (``confirmed`` =
        multi-viewpoint) and identity basis (``tracked``/``reacquired``/``twin-anchor``/
        ``restored``; ``twin-anchor`` = look-alike anchored by position).
        """
        import copy as _copy

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.detection.type.detection3d.object import (
            aggregate_pointclouds,
            to_detection3d_array,
        )

        with _SCAN_LOCK:  # serialize concurrent scan/recall (shared warm belief)
            scanner = self._scanner()
            with SqliteStore(path=str(self.config.db_path), must_exist=True) as read_store:
                # One call answers with the FULLY-CURRENT world, however long the backlog
                # (the caller asked "what is the world now?" — a partial answer that
                # needs a re-ask is worse than a wait). The CLI waits accordingly
                # (`dimos mcp call --timeout`, default 600 s). Live cost ≈ 200-400 ms per
                # folded frame, ~1 frame per backlog second on a static scene.
                present = scanner.scan_recent(
                    read_store, window=window, prompt=prompt or DEFAULT_SCAN_PROMPTS,
                )
                # every scan also tops up the cross-session recall memory, so "when did
                # I see X" is answerable for any session that ever scanned
                self._index_recall_frames(read_store)
            # Publish COPIES: present() returns the live internal entities, which later
            # scans mutate in place — an in-process consumer holding the payload would see
            # geometry change under it (probe-confirmed torn reads).
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
                    "trust": scanner._belief.trust_of(o.object_id),
                    "basis": scanner._belief.basis_of(o.object_id),
                    "xyz": [round(float(o.center.x), 3), round(float(o.center.y), 3), round(float(o.center.z), 3)],
                }
                for o in present
            ]

    @skill
    def recall(self, text: str, k: int = 20) -> dict[str, Any] | None:
        """Recall "when/where did I last see <text>" over what's been recorded so far.

        Callable live::

            dimos mcp call recall -a text="coffee cup"

        CROSS-SESSION memory: the CLIP frame index lives in the shared history store
        (small thumbnails + vectors, each tagged with its source recording), so one
        search spans every session ever indexed — while the raw frames stay in their
        per-session recordings. Each call incrementally indexes this session's new
        frames first (cheap after the first call), then searches all sessions. CLIP
        only PROPOSES moments (whole-frame similarity is nearly flat across one scene);
        the detector picks: the top hits are verified best-first with a small scan of
        each moment's SOURCE recording, and the first moment where the detector finds
        ``text`` is the answer. Returns
        ``{when_ts, where_camera, where_object, recording, clip_similarity}`` for that
        verified moment — ``where_object`` is None when no proposed moment confirms
        (then the fields describe the best unverified hit) or the source recordings
        have been deleted (memory of the sighting outlives the raw footage; only
        verification needs it). ``None`` when nothing matches at all.
        """
        from contextlib import nullcontext

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.recall import recall as _recall

        with _SCAN_LOCK:  # serialize concurrent scan/recall (shared CLIP model + index watermark)
            clip = self._get_recall_clip()
            rec_path = str(self.config.db_path)
            with SqliteStore(path=rec_path, must_exist=True) as read_store, \
                    SqliteStore(path=xarm6_worldbelief_history_path()) as hist_store:
                # top up memory with any frames since the last scan/recall, then search
                # the SHARED index — one query spans every session ever indexed. Object
                # localization always uses a fresh throwaway belief per moment — a past
                # window must never fold into the live one.
                self._index_recall_frames(read_store)
                hit, obj_center = _recall(
                    hist_store, text, model=clip, k=k,
                    detector=self._scanner()._detector,
                    open_recording=lambda rec: (
                        nullcontext(read_store) if rec == rec_path
                        else SqliteStore(path=rec, must_exist=True) if Path(rec).exists()
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
            "where_camera": None if pose is None else [round(pose.x, 3), round(pose.y, 3), round(pose.z, 3)],
            "where_object": None if obj_center is None else [
                round(float(obj_center.x), 3), round(float(obj_center.y), 3), round(float(obj_center.z), 3)
            ],
            "recording": Path(rec_tag).name if rec_tag else None,
            # whole-frame CLIP text-image cosine — uncalibrated, ~0.2-0.3 is a normal
            # strong match; only the RANKING is meaningful
            "clip_similarity": round(float(hit.similarity or 0.0), 3),
        }

    @staticmethod
    def _stream_kwargs(name: str) -> dict[str, Any]:
        return {"codec": "lcm"} if name == "depth_image" else {}

    @pose_setter_for("coordinator_joint_state")
    async def _proprio_pose(self, msg: Any) -> Any:
        """Joint state is proprioception, not a spatially-anchored sensor reading, and its
        ``coordinator`` frame is not in the tf tree. Anchor it to identity so the recorder
        never attempts a (failing) tf lookup — which would otherwise log a "No pose"
        warning at the ~100 Hz tick rate for the whole recording."""
        return Transform.identity().to_pose()

    async def _resolve_pose(self, name: str, msg: Any, ts: float) -> Any:
        frame_id = getattr(msg, "frame_id", None) or self.config.default_frame_id
        if frame_id == "world":
            return Transform.identity().to_pose()
        pose = super()._resolve_pose(name, msg, ts)
        return await pose if inspect.isawaitable(pose) else pose
