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

"""On-demand WorldBelief scan and recall over recorded perception."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import re
import threading
from typing import TYPE_CHECKING, Any, ClassVar

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.memory2.module import MemoryModuleConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.detection.world_belief import (
    DEFAULT_DINO_MODEL,
    WorldBelief,
    WorldBeliefConfig,
)
from dimos.perception.recall import DEFAULT_CLIP_MODEL
from dimos.perception.worldbelief_recorder import WorldBeliefRecorderSpec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.scene_scan import SceneScanner

_STATE_DIR = STATE_DIR / "worldbelief"
_HISTORY_PATH = _STATE_DIR / "worldbelief_history.db"

logger = setup_logger()


class WorldBeliefModuleConfig(MemoryModuleConfig):
    # Replay reads this path directly. Live mode gets the active path from Recorder RPC
    # and uses this value only as the base naming convention for sibling-session recall.
    db_path: str | Path = _STATE_DIR / "recordings" / "worldbelief.db"
    history_path: str | Path = _HISTORY_PATH
    scan_prompts: list[str] = Field(default_factory=list)
    depth_tolerance_s: float = Field(default=0.1, gt=0.0)
    stationary_hz: float = Field(default=1.0, gt=0.0)
    yoloe_model_name: str = Field(default="yoloe-11s-seg.pt", min_length=1)
    dino_model_name: str = Field(default=DEFAULT_DINO_MODEL, min_length=1)
    clip_model_name: str = Field(default=DEFAULT_CLIP_MODEL, min_length=1)
    belief: WorldBeliefConfig = Field(default_factory=WorldBeliefConfig)


class WorldBeliefModule(Module):
    """Own the shared scan/recall models and mutable WorldBelief fold."""

    config: WorldBeliefModuleConfig
    dedicated_worker: ClassVar[bool] = True

    detections_3d: Out[Detection3DArray]
    pointcloud: Out[PointCloud2]
    objects: Out[list[Object]]

    recorder: WorldBeliefRecorderSpec | None = None
    _belief: WorldBelief | None = None
    _engine_lock: threading.Lock

    @rpc
    def start(self) -> None:
        # Locks are process-local runtime state and must be created after deployment.
        self._engine_lock = threading.Lock()
        if not self.config.g.replay:
            self._recording_path()
        super().start()
        with self._engine() as (scanner, _belief):
            scanner.warmup()
        logger.info("WorldBelief scan models warmed up")

    @rpc
    def stop(self) -> None:
        lock = getattr(self, "_engine_lock", None)
        if lock is not None:
            with lock:
                belief = getattr(self, "_belief", None)
                if belief is not None:
                    try:
                        belief.close()
                    except Exception as e:
                        logger.warning("belief close failed: %s", e)
                    self._belief = None
        super().stop()

    def _recording_path(self) -> str:
        if self.config.g.replay:
            return str(self.config.db_path)
        if self.recorder is None:
            raise RuntimeError("live WorldBeliefModule requires WorldBeliefRecorder")
        path = self.recorder.recording_path()
        if not path:
            raise RuntimeError("WorldBeliefRecorder returned an empty recording path")
        return str(path)

    def _recording_paths(self, current: str) -> list[str]:
        """Immutable sessions available to recall under the configured recording base."""
        if self.config.g.replay:
            return [current]
        base = Path(self.config.db_path)
        timestamped = re.compile(
            rf"{re.escape(base.stem)}_\d{{8}}_\d{{6}}_\d{{6}}(?:_\d+)?"
            rf"{re.escape(base.suffix)}"
        )
        siblings = (
            path
            for path in base.parent.glob(f"{base.stem}_*{base.suffix}")
            if timestamped.fullmatch(path.name)
        )
        candidates = [base, *sorted(siblings)]
        by_path = {path.resolve(): str(path) for path in candidates if path.is_file()}
        by_path[Path(current).resolve()] = current
        return [by_path[path] for path in sorted(by_path, key=str)]

    def _scanner(self) -> SceneScanner:
        """Return the lazy scanner paired with the persistent belief and gallery.
        Callers hold ``_engine_lock`` while initializing or using it."""
        scanner: SceneScanner | None = getattr(self, "_live_scanner", None)
        if scanner is None:
            from dimos.models.embedding.dino import DINOModel
            from dimos.perception.detection.detectors.yoloe import (
                Yoloe2DDetector,
                YoloePromptMode,
            )
            from dimos.perception.scene_scan import SceneScanner as _SceneScanner

            bcfg = self._belief_config()
            self._belief = WorldBelief(bcfg)
            scanner = _SceneScanner(
                detector=Yoloe2DDetector(
                    model_name=self.config.yoloe_model_name,
                    prompt_mode=YoloePromptMode.PROMPT,
                ),
                target_frame=bcfg.frame_id,
                text_prompts=list(self.config.scan_prompts),
                visual_embedder=DINOModel(model_name=self.config.dino_model_name),
                depth_tolerance_s=self.config.depth_tolerance_s,
                stationary_hz=self.config.stationary_hz,
            )
            self._live_scanner = scanner
        return scanner

    def _belief_config(self) -> WorldBeliefConfig:
        updates: dict[str, Any] = {
            "appearance_model_id": self.config.dino_model_name,
            "history_path": str(self.config.history_path),
        }
        return self.config.belief.model_copy(update=updates)

    @contextmanager
    def _engine(self) -> Iterator[tuple[SceneScanner, WorldBelief]]:
        """Serialize access to the shared models and mutable belief fold."""
        lock = getattr(self, "_engine_lock", None)
        if lock is None:
            raise RuntimeError("WorldBeliefModule has not started")
        with lock:
            scanner = self._scanner()
            belief = self._belief
            if belief is None:  # _scanner() initializes both; narrows for type checkers
                raise RuntimeError("WorldBelief was not initialized")
            yield scanner, belief

    def _get_recall_clip(self) -> Any:
        clip = getattr(self, "_recall_clip", None)
        if clip is None:
            from dimos.models.embedding.clip import CLIPModel

            clip = CLIPModel(model_name=self.config.clip_model_name)
            clip.start()
            self._recall_clip = clip
        return clip

    def _index_recall_frames(self, read_store: Any, hist_store: Any, rec_path: str) -> None:
        """Incrementally index one recording for recall; failures are non-fatal."""
        from dimos.perception.recall import build_frame_clip_index, index_cursor_stream_name

        try:
            source_end = float(read_store.stream("color_image", Image).last().ts)
            model_id = self.config.clip_model_name
            cursor = hist_store.stream(index_cursor_stream_name(model_id), float)
            try:
                since = float(cursor.tags(rec=rec_path).last().data)
            except LookupError:
                since = None
            if since is not None and source_end <= since:
                return
            build_frame_clip_index(
                read_store,
                model=self._get_recall_clip(),
                model_id=model_id,
                start=since,
                end=source_end,
                index_store=hist_store,
                source_tag=rec_path,
            )
            cursor.append(source_end, ts=source_end, tags={"rec": rec_path})
        except Exception as e:
            logger.warning("recall-index top-up skipped: %s", e)

    @skill
    def scan(self, prompt: list[str] | None = None, window: float = 60.0) -> SkillResult:
        """Fold recorded frames and publish present objects.

        ``prompt`` overrides configured prompts; ``window`` limits the initial scan.
        """
        import copy as _copy

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.detection.type.detection3d.object import (
            aggregate_pointclouds,
            to_detection3d_array,
        )
        from dimos.perception.scene_scan import ScanIncompleteError

        requested = self.config.scan_prompts if prompt is None else prompt
        if not isinstance(requested, list) or any(
            not isinstance(value, str) or not value.strip() for value in requested
        ):
            return SkillResult.fail("INVALID_INPUT", "prompt must contain only non-empty strings")
        prompts = sorted({value.strip() for value in requested})
        if not prompts:
            return SkillResult.fail(
                "INVALID_INPUT", "scan requires a prompt or configured scan_prompts"
            )

        try:
            rec_path = self._recording_path()
        except (TimeoutError, OSError, RuntimeError) as exc:
            return SkillResult.fail("INVALID_STATE", f"Recorder unavailable: {exc}")

        try:
            with SqliteStore(path=rec_path, must_exist=True) as read_store:
                with self._engine() as (scanner, belief):
                    vocabulary = tuple(prompts)
                    active = getattr(self, "_active_scan_prompts", None)
                    prompt_changed = active is not None and active != vocabulary
                    result = scanner.scan_recent(
                        read_store,
                        belief,
                        window=window,
                        prompt=prompts,
                    )
                    if prompt_changed and result.folded_frames == 0:
                        raise ScanIncompleteError(
                            "no new frame was available for the changed prompt vocabulary"
                        )
                    self._active_scan_prompts = vocabulary
                    # Out.publish may invoke in-process consumers with the same object.
                    # Deep snapshots keep those consumers out of mutable belief state.
                    observations = _copy.deepcopy(result.objects)
                    present = _copy.deepcopy(belief.present())
            summaries = [
                {
                    "name": obj.name,
                    "id": obj.object_id,
                    "trust": obj.identity_status,
                    "basis": obj.identity_basis,
                    "frame_id": self.config.belief.frame_id,
                    "last_seen_ts": float(obj.last_seen_ts or obj.ts),
                    "geometry_ts": float(obj.ts),
                    "geometry_frozen": bool(
                        obj.observation_partial
                        or (obj.last_seen_ts is not None and obj.last_seen_ts > obj.ts)
                    ),
                    "observation_partial": bool(obj.observation_partial),
                    "xyz": [
                        float(obj.center.x),
                        float(obj.center.y),
                        float(obj.center.z),
                    ],
                }
                for obj in observations
            ]
        except FileNotFoundError as exc:
            return SkillResult.fail("INVALID_STATE", str(exc))
        except ScanIncompleteError as exc:
            return SkillResult.fail("EXECUTION_FAILED", f"Scan incomplete: {exc}")

        frame_id = self.config.belief.frame_id
        self.detections_3d.publish(
            to_detection3d_array(present, frame_id=frame_id, ts=result.as_of_ts)
        )
        cloud = aggregate_pointclouds(present).voxel_downsample(0.005)
        cloud.frame_id = frame_id
        cloud.ts = result.as_of_ts
        self.pointcloud.publish(cloud)
        self.objects.publish(present)
        return SkillResult.ok(
            f"Scan complete: {len(present)} object(s) present",
            objects=summaries,
            frame_id=frame_id,
            source_end_ts=result.source_end_ts,
            as_of_ts=result.as_of_ts,
            selected_frames=result.selected_frames,
            folded_frames=result.folded_frames,
            skipped_frames=result.skipped_frames,
            prompts=list(vocabulary),
            prompt_changed=prompt_changed,
        )

    @skill
    def recall(self, text: str, k: int = 20) -> dict[str, Any] | None:
        """Search all recorded sessions for ``text``.

        Detector-confirmed hits include ``where_object``.
        """
        from contextlib import nullcontext

        from dimos.memory2.store.sqlite import SqliteStore
        from dimos.perception.recall import recall as _recall

        text = text.strip()
        if not text:
            raise ValueError("recall text must be non-empty")
        if k <= 0:
            raise ValueError("recall k must be positive")

        rec_path = self._recording_path()
        with (
            SqliteStore(path=rec_path, must_exist=True) as read_store,
            SqliteStore(path=str(self.config.history_path)) as hist_store,
        ):
            with self._engine() as (scanner, _belief):
                clip = self._get_recall_clip()
                # Index each immutable session incrementally. Historical localization uses
                # a throwaway belief so it cannot mutate the live fold.
                current_path = Path(rec_path).resolve()
                for recording in self._recording_paths(rec_path):
                    if Path(recording).resolve() == current_path:
                        self._index_recall_frames(read_store, hist_store, rec_path)
                        continue
                    try:
                        with SqliteStore(path=recording, must_exist=True) as old_store:
                            self._index_recall_frames(old_store, hist_store, recording)
                    except (FileNotFoundError, OSError) as exc:
                        logger.warning("recall skipped recording %s: %s", recording, exc)
                hit, obj_center = _recall(
                    hist_store,
                    text,
                    model=clip,
                    model_id=self.config.clip_model_name,
                    k=k,
                    detector=scanner.detector,
                    visual_embedder=scanner.visual_embedder,
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
            # Uncalibrated CLIP cosine; use for ranking, not as an absolute score.
            "clip_similarity": round(float(hit.similarity or 0.0), 3),
        }
