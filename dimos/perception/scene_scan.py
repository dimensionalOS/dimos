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

"""On-demand scene scanning: adaptive fold over recorded frames."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection3d.object import Object
from dimos.perception.detection.world_belief import WorldBelief
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.perception.detection.detectors.base import Detector
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

logger = setup_logger()

_STATIONARY_HZ = 1.0  # coarse keyframe rate while the camera is still
_MOVING_HZ = 7.5  # dense fold rate while the camera moves (sweep parallax tracking)
_TRANS_SPEED = 0.02  # m/s camera translation ⇒ "moving"
_ROT_SPEED = 0.05  # rad/s camera rotation ⇒ "moving"
_NMS_IOU = 0.5  # class-agnostic NMS across prompts (one region can fire several prompts)
_NMS_CONTAINMENT = 0.7  # a bbox ≥70% inside a stronger det's bbox is a sub-region candidate
_SUBREGION_COLOCATED_M = 0.13  # same surface iff also 3D-co-located within the noise envelope
_DEPTH_TOLERANCE_S = 0.1  # max color↔depth frame skew for a lift
_NEG_INF = -float("inf")  # sentinel: "no strict lower bound" for a fresh (non-catch-up) scan


class ScanIncompleteError(RuntimeError):
    """The requested interval lacks the evidence needed for a current fold."""


@dataclass(frozen=True, slots=True)
class SceneScan:
    objects: list[Object]
    source_end_ts: float
    as_of_ts: float
    selected_frames: int
    folded_frames: int
    skipped_frames: int


def _bbox_area(b: tuple[float, float, float, float]) -> float:
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def _bbox_inter(
    b1: tuple[float, float, float, float], b2: tuple[float, float, float, float]
) -> float:
    return max(0.0, min(b1[2], b2[2]) - max(b1[0], b2[0])) * max(
        0.0, min(b1[3], b2[3]) - max(b1[1], b2[1])
    )


def _bbox_iou(
    b1: tuple[float, float, float, float], b2: tuple[float, float, float, float]
) -> float:
    inter = _bbox_inter(b1, b2)
    return inter / (_bbox_area(b1) + _bbox_area(b2) - inter + 1e-9)


def _bbox_containment(
    inner: tuple[float, float, float, float], outer: tuple[float, float, float, float]
) -> float:
    """Fraction of ``inner``'s area lying inside ``outer``."""
    return _bbox_inter(inner, outer) / (_bbox_area(inner) + 1e-9)


class SceneScanner:
    """Stateless scan engine: detect + 2D→3D lift + embed recorded frames."""

    def __init__(
        self,
        detector: Detector | None = None,
        *,
        target_frame: str = "world",
        text_prompts: list[str] | None = None,
        embed: bool = True,
        visual_embedder: Any = None,
        detector_conf: float = 0.6,
        depth_tolerance_s: float = _DEPTH_TOLERANCE_S,
        stationary_hz: float = _STATIONARY_HZ,
    ) -> None:
        if depth_tolerance_s < 0:
            raise ValueError("depth_tolerance_s must be non-negative")
        if stationary_hz <= 0:
            raise ValueError("stationary_hz must be positive")
        self._detector = detector
        self._target_frame = target_frame
        self._text_prompts = list(text_prompts) if text_prompts else None
        self._embed = embed
        self._visual_embedder = visual_embedder
        self._detector_conf = detector_conf
        self._depth_tolerance_s = depth_tolerance_s
        self._stationary_hz = stationary_hz

    def _get_detector(self) -> Detector:
        """Lazily build the default YOLOe detector; injected detectors are used as-is."""
        if self._detector is None:
            from dimos.perception.detection.detectors.yoloe import (
                Yoloe2DDetector,
                YoloePromptMode,
            )

            self._detector = Yoloe2DDetector(
                prompt_mode=YoloePromptMode.PROMPT,
                conf=self._detector_conf,
            )
        return self._detector

    @property
    def detector(self) -> Detector:
        """The lazily initialized detector used by scans and recall confirmation."""
        return self._get_detector()

    @property
    def visual_embedder(self) -> Any:
        """The lazily initialized visual embedder used by scans and recall confirmation."""
        return self._get_embedders()

    def _get_embedders(self) -> Any:
        """The visual (DINO) identity embedder, or None when embeddings are disabled."""
        if not self._embed:
            return None
        if self._visual_embedder is None:
            from dimos.models.embedding.dino import DINOModel

            self._visual_embedder = DINOModel(model_name="facebook/dinov2-base")
            self._visual_embedder.start()
        return self._visual_embedder

    def warmup(self) -> None:
        """Load models and run one inference so successful startup means scan-ready."""
        detector = self._get_detector()
        set_prompts = getattr(detector, "set_prompts", None)
        if self._text_prompts and callable(set_prompts):
            set_prompts(text=list(self._text_prompts))
        dummy = Image(
            data=np.zeros((64, 64, 3), dtype=np.uint8),
            format=ImageFormat.RGB,
        )
        detector_warmup = getattr(detector, "warmup", None)
        if callable(detector_warmup):
            detector_warmup(dummy)
        visual = self._get_embedders()
        if visual is not None:
            visual.embed(dummy)

    @staticmethod
    def _resolve_window(
        belief: WorldBelief,
        color: Any,
        start: float | None,
        end: float | None,
        initial_lookback_s: float | None,
    ) -> tuple[float, float, float] | None:
        """Return ``(start, end, after)``; only catch-up uses an exclusive watermark."""
        try:
            latest = float(color.last().ts)
        except LookupError:
            return None
        end = latest if end is None else min(float(end), latest)
        if start is not None:
            return start, end, _NEG_INF  # caller pinned start → honor it inclusively
        watermark = belief.last_fold_ts
        if watermark > 0.0:
            return watermark, end, watermark  # catch-up: fold STRICTLY after the last folded frame
        if initial_lookback_s is not None and initial_lookback_s > 0.0:
            return end - initial_lookback_s, end, _NEG_INF  # first scan, bounded lookback
        return 0.0, end, _NEG_INF  # first scan: from the beginning of the recording

    def scan(
        self,
        store: Store,
        belief: WorldBelief,
        *,
        prompt: list[str] | None = None,
        start: float | None = None,
        end: float | None = None,
        initial_lookback_s: float | None = None,
    ) -> SceneScan:
        """Advance the belief over the requested recording interval."""
        detector = self._get_detector()
        prompts = self._text_prompts if prompt is None else prompt
        set_prompts = getattr(detector, "set_prompts", None)
        if prompts and callable(set_prompts):
            set_prompts(text=list(prompts))
        elif callable(set_prompts):
            raise ValueError("scan requires at least one text prompt")

        color = store.stream("color_image", Image)
        depth = store.stream("depth_image", Image)
        window = self._resolve_window(belief, color, start, end, initial_lookback_s)
        if window is None:
            raise ScanIncompleteError("recording has no color frames")
        start, end, after = window
        source_end_ts = end
        previous_as_of = belief.last_fold_ts

        info = store.stream("camera_info", CameraInfo)
        try:
            camera_info = info.last().data
        except LookupError as exc:
            raise ScanIncompleteError("recording has no camera_info") from exc

        # Skip pose-less frames unless they already use the target frame.
        interval = color.time_range(start, end)
        if after > _NEG_INF:
            interval = interval.after(after)  # catch-up: strict ts > watermark, pushed to SQL
        skipped = 0
        frames: list[Any] = []
        last_source_ts: float | None = None
        poseless_in_target: bool | None = None
        for obs in interval:
            obs_ts = float(obs.ts)
            last_source_ts = obs_ts if last_source_ts is None else max(last_source_ts, obs_ts)
            if obs.pose is None:
                if poseless_in_target is None:
                    frame_id = obs.data.frame_id or ""  # one decode, first pose-less frame only
                    poseless_in_target = frame_id in ("", self._target_frame)
                if not poseless_in_target:
                    skipped += 1
                    continue
            frames.append(obs)
        if last_source_ts is not None:
            source_end_ts = last_source_ts
        frames.sort(key=lambda obs: float(obs.ts))
        if skipped:
            logger.info("scan: skipped %d frame(s) without a camera pose", skipped)
        if not frames:
            if source_end_ts <= previous_as_of:
                return SceneScan(
                    objects=belief.observations(),
                    source_end_ts=source_end_ts,
                    as_of_ts=previous_as_of,
                    selected_frames=0,
                    folded_frames=0,
                    skipped_frames=skipped,
                )
            raise ScanIncompleteError("recording has no eligible posed color frames")

        selected = self._select_frames(frames)
        folded = 0
        for obs in selected:
            got = self._detect_frame(obs, camera_info, depth, self._depth_tolerance_s, detector)
            if got is None:
                skipped += 1
                continue
            objects, depth_arr, camera_transform = got
            before = belief.last_fold_ts
            belief.observe(
                objects,
                frame_ts=obs.ts,
                camera_transform=camera_transform,
                camera_info=camera_info,
                depth_m=depth_arr,
            )
            if belief.last_fold_ts > before:
                folded += 1
            else:
                skipped += 1
        if not folded and source_end_ts > previous_as_of:
            raise ScanIncompleteError("recording has no foldable RGB-depth/TF frames")
        if belief.last_fold_ts < source_end_ts:
            raise ScanIncompleteError(
                f"fold stopped at {belief.last_fold_ts}, before source end {source_end_ts}"
            )
        return SceneScan(
            objects=belief.observations(),
            source_end_ts=source_end_ts,
            as_of_ts=belief.last_fold_ts,
            selected_frames=len(selected),
            folded_frames=folded,
            skipped_frames=skipped,
        )

    def scan_recent(
        self,
        store: Store,
        belief: WorldBelief,
        *,
        window: float,
        prompt: list[str] | None = None,
    ) -> SceneScan:
        """Catch ``belief`` up to the newest frame; ``window`` caps only the first scan's
        lookback (<=0 = recording start), later scans fold everything since the last."""
        return self.scan(
            store,
            belief,
            prompt=prompt,
            initial_lookback_s=window if window > 0 else None,
        )

    def _select_frames(self, frames: list[Any]) -> list[Any]:
        """Coarse keyframes while still, dense while moving — judged from pose deltas alone."""
        selected = [frames[0]]
        last_kept = frames[0]
        for prev, cur in itertools.pairwise(frames):
            dt = cur.ts - prev.ts
            if dt <= 0:
                continue
            p0, p1 = prev.pose, cur.pose
            moving = (
                p0 is not None
                and p1 is not None
                and (
                    p0.position.distance(p1.position) / dt > _TRANS_SPEED
                    or p0.orientation.angle_to(p1.orientation) / dt > _ROT_SPEED
                )
            )
            period = 1.0 / (_MOVING_HZ if moving else self._stationary_hz)
            if cur.ts - last_kept.ts >= period:
                selected.append(cur)
                last_kept = cur
        if selected[-1].ts != frames[-1].ts:
            selected.append(frames[-1])
        return selected

    def _detect_frame(
        self,
        obs: Any,
        camera_info: CameraInfo,
        depth_stream: Any,
        depth_tolerance: float,
        detector: Detector,
    ) -> tuple[list[Object], Any, Transform | None] | None:
        """Detect + NMS + 2D→3D lift + embed one frame; None if no depth within tolerance."""
        color_img: Image = obs.data
        depth_img = self._nearest_depth(depth_stream, obs.ts, depth_tolerance)
        if depth_img is None:
            return None
        process = getattr(detector, "predict_image", detector.process_image)
        detections: ImageDetections2D[Any] = process(color_img)
        frame_id = color_img.frame_id or ""
        in_target = frame_id in ("", self._target_frame)
        camera_transform = None if in_target else Transform.from_pose(frame_id, obs.pose)
        if camera_transform is not None:
            camera_transform.frame_id = self._target_frame
            camera_transform.ts = float(obs.ts)
        objects = Object.from_2d_to_list(
            detections_2d=detections,
            color_image=color_img,
            depth_image=depth_img,
            camera_info=camera_info,
            camera_transform=camera_transform,
        )
        for obj in objects:
            obj.frame_id = obj.pose.frame_id = obj.pointcloud.frame_id = self._target_frame
        objects = [o for o in objects if float(o.confidence) >= self._detector_conf]
        # Remove overlapping prompts and same-depth contained regions; a contained object
        # at a different depth survives.
        objects.sort(key=lambda o: -float(o.confidence))
        kept: list[Object] = []
        for o in objects:
            if not any(
                _bbox_iou(o.bbox, k.bbox) > _NMS_IOU
                or (
                    _bbox_containment(o.bbox, k.bbox) >= _NMS_CONTAINMENT
                    and o.center.distance(k.center) <= _SUBREGION_COLOCATED_M
                )
                for k in kept
            ):
                kept.append(o)
        if kept:
            for o in kept:
                # Bbox on the frame edge = partial view: trust identity, freeze geometry.
                x1, y1, x2, y2 = o.bbox
                o.observation_partial = (
                    x1 <= 3 or y1 <= 3 or x2 >= color_img.width - 3 or y2 >= color_img.height - 3
                )
            self._attach_embeddings(kept, color_img)
        depth_arr = np.asarray(depth_img.to_opencv(), dtype=np.float32)
        return kept, depth_arr, camera_transform

    def _attach_embeddings(self, objects: list[Object], color_img: Image) -> None:
        """Attach DINO identity embeddings per object crop for re-ID; failures skip, not crash."""
        visual = self._get_embedders()
        if visual is None:
            return
        crops: list[Image] = []
        idxs: list[int] = []
        for i, obj in enumerate(objects):
            crop = obj.cropped_image(padding=0).to_rgb()
            if crop.width < 2 or crop.height < 2:
                continue
            crop.frame_id = color_img.frame_id
            crop.ts = color_img.ts
            crops.append(crop)
            idxs.append(i)
        if not crops:
            return
        try:
            embeddings = visual.embed(*crops)
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for i, embedding in zip(idxs, embeddings, strict=False):
                objects[i].visual_embedding = embedding.to_numpy().reshape(-1).astype(np.float32)
        except Exception as exc:
            logger.warning("visual embedding step failed (skipping this frame): %s", exc)

    @staticmethod
    def _nearest_depth(depth_stream: Any, ts: float, tolerance: float) -> Image | None:
        """Return nearest depth; ``at().first()`` can be stale while moving."""
        candidates = list(depth_stream.at(ts, tolerance))
        if not candidates:
            return None
        depth_obs = min(candidates, key=lambda o: abs(float(o.ts) - ts))
        raw: Image = depth_obs.data
        cv = raw.to_opencv()
        if raw.format == ImageFormat.DEPTH16:
            cv = cv.astype("float32") / 1000.0
        elif cv.dtype != "float32":
            cv = cv.astype("float32")
        return Image(data=cv, format=ImageFormat.DEPTH, frame_id=raw.frame_id, ts=raw.ts)
