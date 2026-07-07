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

"""On-demand scene scanning — the complete adaptive fold over recorded frames.

A ``scan`` advances the world model over the ENTIRE recorded interval since the last
fold — never a sampled window, so there are never observation holes between scans.
Folding everything is affordable because density adapts to the recorded camera pose:
coarse 1 s keyframes while static, 7.5 Hz while moving (> 0.02 m/s or > 0.05 rad/s),
plus bisection (depth ≤ 4) wherever the detection set changes between consecutive
selected frames — so scene events get dense coverage even under a static camera.
Benchmark cost: 11-18 % of recorded frames folded, every scripted event covered.

Pass ``history_path`` for cross-session identity (memory2 vec0 galleries).
"""

from __future__ import annotations

import bisect
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

_STATIC_STRIDE_S = 1.0  # coarse keyframe stride while the camera is still
_MOVING_HZ = 7.5  # dense fold rate while the camera moves (sweep parallax tracking)
_TRANS_SPEED = 0.02  # m/s camera translation ⇒ "moving"
_ROT_SPEED = 0.05  # rad/s camera rotation ⇒ "moving"
_BISECT_DEPTH = 4  # max recursive refinement depth between two selected frames
_NMS_IOU = 0.5  # class-agnostic NMS across prompts (one region can fire several prompts)
_NMS_CONTAINMENT = 0.7  # a bbox ≥70% inside a stronger det's bbox is a sub-region candidate
_SUBREGION_COLOCATED_M = 0.13  # same surface iff also 3D-co-located within the noise envelope
_DEPTH_TOLERANCE_S = 0.1  # max color↔depth frame skew for a lift


def _bbox_area(b: tuple[float, float, float, float]) -> float:
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def _bbox_inter(b1: tuple[float, float, float, float], b2: tuple[float, float, float, float]) -> float:
    return max(0.0, min(b1[2], b2[2]) - max(b1[0], b2[0])) * max(
        0.0, min(b1[3], b2[3]) - max(b1[1], b2[1]))


def _bbox_iou(b1: tuple[float, float, float, float], b2: tuple[float, float, float, float]) -> float:
    inter = _bbox_inter(b1, b2)
    return inter / (_bbox_area(b1) + _bbox_area(b2) - inter + 1e-9)


def _bbox_containment(inner: tuple[float, float, float, float], outer: tuple[float, float, float, float]) -> float:
    """Fraction of ``inner``'s area lying inside ``outer``."""
    return _bbox_inter(inner, outer) / (_bbox_area(inner) + 1e-9)


class SceneScanner:
    """Runs detection + 2D→3D lift + embeddings on demand and folds into a WorldBelief.

    The belief (and, with ``history_path``, its vec0 galleries) persists across scans;
    the scanner owns frame selection, all identity/trust/absence logic lives in the
    world model. Call :meth:`present` for the current object set.
    """

    def __init__(
        self,
        detector: Detector | None = None,
        *,
        target_frame: str = "world",
        min_frames: int = 3,
        min_span_s: float = 1.5,
        text_prompts: list[str] | None = None,
        embed: bool = True,
        visual_embedder: Any = None,  # test-only injection point (FakeEmbedder)
        history_path: str | None = None,
        detector_conf: float = 0.6,
    ) -> None:
        self._detector = detector
        self._target_frame = target_frame
        self._text_prompts = list(text_prompts) if text_prompts else None
        self._embed = embed
        self._visual_embedder = visual_embedder
        # 0.6: lower (0.4) proved noisy on the arm — sub-0.5 junk kept entering the
        # lifecycle; every benchmark prompt (incl. "green can" at 0.89-0.97) clears 0.6.
        self._detector_conf = detector_conf
        self._belief = WorldBelief(
            min_frames=min_frames,
            min_span_s=min_span_s,
            history_path=history_path,
        )
        if history_path is None and embed:
            # Without a history store, identity does NOT survive a restart — say so.
            logger.info("SceneScanner: no history_path — identity will not persist across sessions")

    # ── model plumbing ─────────────────────────────────────────────────────────────

    def _resolve_device(self) -> str:
        """GPU when torch sees one, else CPU. (The stack's default auto-detect uses
        ``pycuda``, often absent even when torch's CUDA works — resolve via torch.)"""
        try:
            import torch

            return "cuda" if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"

    def _get_detector(self) -> Detector:
        """Lazily build the default YOLOe detector; injected detectors are used as-is."""
        if self._detector is None:
            from dimos.perception.detection.detectors.yoloe import (
                Yoloe2DDetector,
                YoloePromptMode,
            )

            self._detector = Yoloe2DDetector(
                prompt_mode=YoloePromptMode.PROMPT,
                device=self._resolve_device(),
                conf=self._detector_conf,
            )
        return self._detector

    def _get_embedders(self) -> Any:
        """The visual (DINO) identity embedder, or None when embeddings are disabled.
        (Identity is DINO-only: per-crop CLIP was measured as dead weight; whole-frame
        CLIP semantic recall belongs to the recall index.)"""
        if not self._embed:
            return None
        if self._visual_embedder is None:
            from dimos.models.embedding.dino import DINOModel

            self._visual_embedder = DINOModel(
                model_name="facebook/dinov2-base", device=self._resolve_device()
            )
            self._visual_embedder.start()
        return self._visual_embedder

    def warmup(self) -> None:
        """Pre-build the detector + embedders so the first :meth:`scan` doesn't pay the
        one-time model-load stall on the caller's thread."""
        self._get_detector()
        self._get_embedders()

    # ── the scan ───────────────────────────────────────────────────────────────────

    def scan(
        self,
        store: Store,
        *,
        prompt: list[str] | None = None,
        start: float | None = None,
        end: float | None = None,
        initial_lookback_s: float | None = None,
    ) -> list[Object]:
        """Advance the fold over ``[start, end]`` and return the present object set.

        No ``start`` = CATCH UP from the last folded frame to ``end`` (newest frame) —
        repeated scans cover the recording completely, no holes. First-ever scan:
        ``initial_lookback_s`` bounds the lookback (None = recording start). Explicit
        ``start`` overrides; the model skips frames at/before its watermark (time only
        moves forward)."""
        detector = self._get_detector()
        prompts = prompt or self._text_prompts
        set_prompts = getattr(detector, "set_prompts", None)
        if prompts and callable(set_prompts):
            set_prompts(text=list(prompts))
        elif not prompts:
            # An open-vocab detector (YOLOe) with no prompt defaults to class "nothing"
            # and silently returns []. Make that visible rather than a mystery.
            logger.warning("scan: no prompt/text_prompts set — an open-vocab detector will detect nothing")

        color = store.stream("color_image", Image)
        depth = store.stream("depth_image", Image)
        info = store.stream("camera_info", CameraInfo)
        try:
            camera_info = info.last().data
        except LookupError:
            logger.warning("scan: no camera_info in store; cannot lift")
            return self.present()
        if end is None:
            try:
                end = float(color.last().ts)
            except LookupError:
                return self.present()  # nothing recorded yet
        after = -float("inf")  # exclusive lower bound (catch-up must not refetch the boundary)
        if start is None:
            watermark = self._belief.last_fold_ts
            if watermark > 0.0:
                # Catch-up: fold everything STRICTLY AFTER the last folded frame — the
                # boundary frame itself was already folded (re-detecting it wasted one
                # full GPU frame per scan call).
                start = watermark
                after = watermark
            elif initial_lookback_s is not None and initial_lookback_s > 0.0:
                start = end - initial_lookback_s
            else:
                start = 0.0  # first scan: from the beginning of the recording

        # Materialize observation HANDLES only — reading obs.data here would JPEG-decode
        # and pin every frame in the interval; only the selected ~13-19 % may decode.
        # Pose-less frames (tf not yet up in the first ~1 s of a real recording) can't be
        # placed in the target frame and are skipped — unless the recording itself is in
        # the target frame (probed ONCE, from a single frame).
        skipped = 0
        frames = []
        poseless_in_target: bool | None = None
        for obs in color.time_range(start, end):
            if obs.ts <= after:
                continue
            if obs.pose is None:
                if poseless_in_target is None:
                    frame_id = obs.data.frame_id or ""  # one decode, first pose-less frame only
                    poseless_in_target = frame_id in ("", self._target_frame)
                if not poseless_in_target:
                    skipped += 1
                    continue
            frames.append(obs)
        if skipped:
            logger.info("scan: skipped %d frame(s) without a camera pose", skipped)
        if not frames:
            return self.present()

        # per-scan detection cache: ts → (objects, depth_m, camera_transform) | None
        cache: dict[float, tuple[list[Object], Any, Transform | None] | None] = {}

        def detect(obs: Any) -> tuple[list[Object], Any, Transform | None] | None:
            if obs.ts not in cache:
                cache[obs.ts] = self._detect_frame(obs, camera_info, depth, _DEPTH_TOLERANCE_S)
            return cache[obs.ts]

        selected = self._select_frames(frames)
        selected = self._refine(selected, frames, detect)
        for obs in selected:
            got = detect(obs)
            if got is None:
                continue  # no matching depth frame — cannot lift or vote absence
            objects, depth_arr, camera_transform = got
            self._belief.observe(
                objects,
                frame_ts=obs.ts,
                camera_transform=camera_transform,
                camera_info=camera_info,
                depth_m=depth_arr,
            )
        return self.present()

    def scan_recent(
        self,
        store: Store,
        *,
        window: float,
        prompt: list[str] | None = None,
    ) -> list[Object]:
        """Catch-up scan to the newest recorded frame. ``window`` bounds only the FIRST
        scan's lookback (``<= 0`` = recording start); later calls fold the complete
        interval since the previous scan."""
        return self.scan(
            store,
            prompt=prompt,
            initial_lookback_s=window if window > 0 else None,
        )

    # ── frame selection: adaptive density + bisection ─────────────────────────────

    @staticmethod
    def _select_frames(frames: list[Any]) -> list[Any]:
        """Coarse keyframes while the camera is still, dense while it moves — judged from
        the recorded pose deltas (free and exact; no models involved)."""
        selected = [frames[0]]
        last_kept = frames[0]
        for prev, cur in itertools.pairwise(frames):
            dt = cur.ts - prev.ts
            if dt <= 0:
                continue
            p0, p1 = prev.pose, cur.pose
            moving = p0 is not None and p1 is not None and (
                p0.position.distance(p1.position) / dt > _TRANS_SPEED
                or p0.orientation.angle_to(p1.orientation) / dt > _ROT_SPEED
            )
            period = 1.0 / _MOVING_HZ if moving else _STATIC_STRIDE_S
            if cur.ts - last_kept.ts >= period:
                selected.append(cur)
                last_kept = cur
        if selected[-1].ts != frames[-1].ts:
            selected.append(frames[-1])
        return selected

    @staticmethod
    def _refine(selected: list[Any], frames: list[Any], detect: Any) -> list[Any]:
        """Bisect-insert midpoint frames wherever the detection label-set changes between
        consecutive selected frames — scene events get dense coverage even when the
        camera is static (a hand rearranging objects fires no pose trigger)."""

        by_ts = {f.ts: f for f in frames}
        ts_sorted = sorted(by_ts)

        def signature(obs: Any) -> tuple[str, ...] | None:
            got = detect(obs)
            return None if got is None else tuple(sorted(o.name for o in got[0]))

        def midframe(a: Any, b: Any) -> Any | None:
            lo = bisect.bisect_right(ts_sorted, a.ts)
            hi = bisect.bisect_left(ts_sorted, b.ts)
            if hi <= lo:
                return None
            return by_ts[ts_sorted[(lo + hi) // 2]]

        inserted: list[Any] = []
        work = [(a, b, 0) for a, b in itertools.pairwise(selected)]
        while work:
            a, b, depth = work.pop()
            if depth >= _BISECT_DEPTH or signature(a) == signature(b):
                continue
            mid = midframe(a, b)
            if mid is None or mid.ts in (a.ts, b.ts):
                continue
            inserted.append(mid)
            work.append((a, mid, depth + 1))
            work.append((mid, b, depth + 1))
        merged = {obs.ts: obs for obs in [*selected, *inserted]}
        return [merged[ts] for ts in sorted(merged)]

    # ── per-frame detect + lift + embed (cached per scan) ─────────────────────────

    def _detect_frame(
        self, obs: Any, camera_info: CameraInfo, depth_stream: Any, depth_tolerance: float
    ) -> tuple[list[Object], Any, Transform | None] | None:
        """Detector + NMS + 2D→3D lift + embeddings for one recorded frame, or None when
        no depth frame lands within ``depth_tolerance``."""
        color_img: Image = obs.data
        depth_img = self._nearest_depth(depth_stream, obs.ts, depth_tolerance)
        if depth_img is None:
            return None
        detections: ImageDetections2D[Any] = self._detector.process_image(color_img)
        frame_id = color_img.frame_id or ""
        in_target = frame_id in ("", self._target_frame)
        camera_transform = None if in_target else Transform.from_pose(frame_id, obs.pose)
        objects = Object.from_2d_to_list(
            detections_2d=detections,
            color_image=color_img,
            depth_image=depth_img,
            camera_info=camera_info,
            camera_transform=camera_transform,
        )
        objects = [o for o in objects if float(o.confidence) >= self._detector_conf]
        # Class-agnostic NMS across prompts (one region may fire several prompts) — keep
        # the highest-confidence reading per region. Two rules, both physical:
        #  - IoU > 0.5: the same region under two prompts.
        #  - CONTAINMENT + 3D co-location (0.13 m sensor-noise envelope): a det mostly
        #    inside a stronger det's bbox at the same lifted depth is a SUB-REGION
        #    reading of the same surface (e.g. a cup's green print firing 'green can' —
        #    label and appearance lie together; only per-frame geometry can call it).
        #    Two rigid objects cannot co-locate in 3D; a real small object IN FRONT of a
        #    big one keeps a different center depth and survives.
        objects.sort(key=lambda o: -float(o.confidence))
        kept: list[Object] = []
        for o in objects:
            if not any(
                _bbox_iou(o.bbox, k.bbox) > _NMS_IOU
                or (_bbox_containment(o.bbox, k.bbox) >= _NMS_CONTAINMENT
                    and o.center.distance(k.center) <= _SUBREGION_COLOCATED_M)
                for k in kept
            ):
                kept.append(o)
        if kept:
            for o in kept:
                # Border-touching bbox = PARTIAL view: lifted geometry is untrustworthy;
                # the world model still credits identity/support but freezes geometry.
                x1, y1, x2, y2 = o.bbox
                o.observation_partial = (
                    x1 <= 3 or y1 <= 3 or x2 >= color_img.width - 3 or y2 >= color_img.height - 3
                )
            self._attach_embeddings(kept, color_img)
        depth_arr = np.asarray(depth_img.to_opencv(), dtype=np.float32)
        return kept, depth_arr, camera_transform

    def _attach_embeddings(self, objects: list[Object], color_img: Image) -> None:
        """Attach DINO identity embeddings to each object's crop (feeds the belief's
        appearance galleries for re-ID); failures skip the frame, not crash it."""
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
        """Nearest depth frame to ``ts``, converted to float32 meters (DEPTH format).

        Genuinely nearest: ``at(...).first()`` returns the EARLIEST frame in the band
        (insertion order), which is systematically ~tolerance stale exactly when the
        camera moves. The band holds only a handful of handles; blobs stay lazy."""
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

    # ── queries ────────────────────────────────────────────────────────────────────

    def present(self) -> list[Object]:
        """Objects believed present now — see :meth:`WorldBelief.present` for semantics."""
        return self._belief.present()

    def close(self) -> None:
        """Close the belief's history store (see :meth:`WorldBelief.close`)."""
        self._belief.close()
