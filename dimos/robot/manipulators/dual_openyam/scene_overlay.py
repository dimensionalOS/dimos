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

"""Overlay a saved reference image on the overhead camera stream to rebuild a benchmark scene.

SceneReplica (arXiv 2306.15620) reproduces tabletop scenes without markers: a
half-transparent reference image is drawn over the live camera view and the
objects are moved until they line up. This only places objects repeatably
while the camera stays fixed relative to the robot. With no scene selected the
live image passes through, so the module costs nothing until a benchmark run.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import threading
import time
from typing import Any, Literal

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SCENE_OVERLAY_DIR = STATE_DIR / "scene_overlay"
OverlayMode = Literal["blend", "edges"]
_SCENE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
_EDGE_COLOR = (0, 255, 0)
_CHANGE_THRESHOLD = 35
_MIN_OBJECT_AREA = 300


class SceneOverlayConfig(ModuleConfig):
    scenes_dir: Path = SCENE_OVERLAY_DIR
    # Empty selects no scene: the live image passes through until set_scene is called.
    scene: str = ""
    opacity: float = Field(default=0.35, ge=0.0, le=1.0)
    mode: OverlayMode = "blend"
    # 0 publishes every frame.
    max_fps: float = Field(default=10.0, ge=0.0)
    label: bool = True
    # Snapshot of the empty table; with it each object's ghost gets an alignment score.
    background: str = "background"
    alignment: bool = True


def scene_paths(scenes_dir: Path, scene: str) -> tuple[Path, Path]:
    """Return the image and metadata paths of a scene; names stay inside scenes_dir."""
    if not _SCENE_NAME.match(scene):
        raise ValueError(f"Invalid scene name {scene!r}")
    return scenes_dir / f"{scene}.png", scenes_dir / f"{scene}.json"


def fit_reference(reference: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Resize a reference to the live frame's (height, width) when they differ."""
    if reference.shape[:2] == tuple(shape):
        return reference
    import cv2

    return cv2.resize(reference, (shape[1], shape[0]), interpolation=cv2.INTER_AREA)


def blend(live: np.ndarray, reference: np.ndarray, opacity: float) -> np.ndarray:
    import cv2

    return cv2.addWeighted(live, 1.0 - opacity, reference, opacity, 0.0)


def edge_overlay(live: np.ndarray, reference: np.ndarray) -> np.ndarray:
    import cv2

    edges = cv2.Canny(cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY), 80, 160)
    out = live.copy()
    out[edges > 0] = _EDGE_COLOR
    return out


def foreground_mask(image: np.ndarray, background: np.ndarray) -> np.ndarray:
    """Pixels whose colour differs from the empty table, cleaned of speckle."""
    import cv2

    diff = cv2.absdiff(
        cv2.GaussianBlur(image, (5, 5), 0), cv2.GaussianBlur(background, (5, 5), 0)
    ).max(axis=2)
    changed = (diff > _CHANGE_THRESHOLD).astype(np.uint8)
    opened = cv2.morphologyEx(changed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return np.asarray(closed, dtype=bool)


def segment_objects(mask: np.ndarray) -> list[np.ndarray]:
    """Connected foreground blobs large enough to be objects, left to right."""
    import cv2

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    blobs = [
        (int(stats[index, cv2.CC_STAT_LEFT]), labels == index)
        for index in range(1, count)
        if stats[index, cv2.CC_STAT_AREA] >= _MIN_OBJECT_AREA
    ]
    return [blob for _, blob in sorted(blobs, key=lambda item: item[0])]


def alignment_scores(live_mask: np.ndarray, object_masks: list[np.ndarray]) -> list[float]:
    """Per object: overlap between the live foreground near its ghost and the ghost itself."""
    scores: list[float] = []
    height, width = live_mask.shape
    every_ghost = np.zeros_like(live_mask)
    for ghost in object_masks:
        every_ghost |= ghost
    for ghost in object_masks:
        ys, xs = np.nonzero(ghost)
        pad_y = int(0.3 * (ys.max() - ys.min() + 1))
        pad_x = int(0.3 * (xs.max() - xs.min() + 1))
        roi = np.zeros_like(ghost)
        roi[
            max(0, ys.min() - pad_y) : min(height, ys.max() + pad_y + 1),
            max(0, xs.min() - pad_x) : min(width, xs.max() + pad_x + 1),
        ] = True
        # A neighbour sitting on its own ghost must not count against this one.
        live_near = live_mask & roi & ~(every_ghost & ~ghost)
        union = int((live_near | ghost).sum())
        scores.append(float((live_near & ghost).sum()) / union if union else 0.0)
    return scores


def draw_alignment(frame: np.ndarray, object_masks: list[np.ndarray], scores: list[float]) -> None:
    """Outline each ghost red to green by score and print the percentage beside it."""
    import cv2

    for ghost, score in zip(object_masks, scores, strict=True):
        color = (0, int(255 * score), int(255 * (1.0 - score)))
        contours, _ = cv2.findContours(
            ghost.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(frame, contours, -1, color, 2)
        ys, xs = np.nonzero(ghost)
        origin = (int(xs.min()), max(14, int(ys.min()) - 6))
        text = f"{score:.0%}"
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


class SceneOverlayModule(Module):
    """Publish the live camera image with the current scene's reference drawn over it."""

    config: SceneOverlayConfig

    color_image: In[Image]
    overlay_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._scene = self.config.scene
        self._opacity = self.config.opacity
        self._mode: OverlayMode = self.config.mode
        self._reference: np.ndarray | None = None
        self._background: np.ndarray | None = None
        self._object_masks: list[np.ndarray] = []
        self._scores: list[float] = []
        self._latest: Image | None = None
        self._last_publish = 0.0
        self._frames = 0
        self._resize_warned = False
        self._load_background()
        self._load_reference(self._scene)

    @rpc
    def start(self) -> None:
        super().start()
        # One line per run so the run log says which scene it belongs to.
        logger.info(
            "Scene overlay scenes_dir=%s scene=%s has_reference=%s",
            self.config.scenes_dir,
            self._scene or "none",
            self._reference is not None,
        )
        unsubscribe = self.color_image.subscribe(self._on_frame)
        self.register_disposable(Disposable(unsubscribe) if callable(unsubscribe) else unsubscribe)

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def capture_reference(self, scene: str | None = None) -> str:
        """Save the latest live frame as the reference of a scene and switch to it."""
        name = scene or self._scene
        if not name:
            raise ValueError("No scene selected; pass a scene name")
        image_path, meta_path = scene_paths(self.config.scenes_dir, name)
        with self._lock:
            latest = self._latest
        if latest is None:
            raise RuntimeError("No camera frame received yet")
        import cv2

        frame = latest.to_opencv()
        image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), frame):
            raise RuntimeError(f"Could not write {image_path}")
        meta_path.write_text(
            json.dumps(
                {
                    "scene": name,
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                    "frame_id": latest.frame_id,
                    "frame_ts": latest.ts,
                    "captured_at": time.time(),
                },
                indent=2,
            )
            + "\n"
        )
        with self._lock:
            self._scene = name
            self._reference = frame
            self._resize_warned = False
            self._update_object_masks()
        logger.info("Saved scene reference %s", image_path)
        return str(image_path)

    @rpc
    def capture_background(self) -> str:
        """Save the latest live frame as the empty table every scene is compared against."""
        image_path, _ = scene_paths(self.config.scenes_dir, self.config.background)
        with self._lock:
            latest = self._latest
        if latest is None:
            raise RuntimeError("No camera frame received yet")
        import cv2

        frame = latest.to_opencv()
        image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), frame):
            raise RuntimeError(f"Could not write {image_path}")
        with self._lock:
            self._background = frame
            self._update_object_masks()
        logger.info("Saved scene background %s", image_path)
        return str(image_path)

    @rpc
    def get_alignment(self) -> dict[str, Any]:
        """Per-object alignment of the live scene to the current reference, 0.0 to 1.0."""
        with self._lock:
            scores = list(self._scores)
            return {
                "scene": self._scene,
                "objects": len(self._object_masks),
                "scores": scores,
                "mean": sum(scores) / len(scores) if scores else None,
                "has_background": self._background is not None,
            }

    @rpc
    def set_scene(self, scene: str) -> bool:
        """Switch to a scene; returns False when it has no reference yet."""
        loaded = self._load_reference(scene)
        logger.info("Scene overlay scene=%s has_reference=%s", scene or "none", loaded)
        return loaded

    @rpc
    def set_opacity(self, opacity: float) -> float:
        if not 0.0 <= opacity <= 1.0:
            raise ValueError("opacity must be between 0.0 and 1.0")
        with self._lock:
            self._opacity = float(opacity)
        return self._opacity

    @rpc
    def set_mode(self, mode: OverlayMode) -> str:
        if mode not in ("blend", "edges"):
            raise ValueError("mode must be 'blend' or 'edges'")
        with self._lock:
            self._mode = mode
        return self._mode

    @rpc
    def list_scenes(self) -> list[str]:
        scenes_dir = self.config.scenes_dir
        if not scenes_dir.is_dir():
            return []
        return sorted(path.stem for path in scenes_dir.glob("*.png"))

    @rpc
    def get_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "scene": self._scene,
                "has_reference": self._reference is not None,
                "opacity": self._opacity,
                "mode": self._mode,
                "scenes_dir": str(self.config.scenes_dir),
                "frames": self._frames,
                "last_frame_ts": self._latest.ts if self._latest is not None else None,
                "has_background": self._background is not None,
                "objects": len(self._object_masks),
            }

    def _load_reference(self, scene: str) -> bool:
        reference: np.ndarray | None = None
        if not scene:
            with self._lock:
                self._scene = ""
                self._reference = None
                self._update_object_masks()
            return False
        image_path, _ = scene_paths(self.config.scenes_dir, scene)
        if image_path.is_file():
            import cv2

            reference = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        with self._lock:
            self._scene = scene
            self._reference = reference
            self._resize_warned = False
            self._update_object_masks()
        if reference is None:
            logger.warning(
                "Scene %s has no reference at %s; publishing the live image until one is captured",
                scene,
                image_path,
            )
            return False
        return True

    def _load_background(self) -> None:
        image_path, _ = scene_paths(self.config.scenes_dir, self.config.background)
        if not image_path.is_file():
            return
        import cv2

        with self._lock:
            self._background = cv2.imread(str(image_path), cv2.IMREAD_COLOR)

    def _update_object_masks(self) -> None:
        """Ghost footprints of the current reference; caller holds the lock."""
        self._scores = []
        if self._reference is None or self._background is None or not self.config.alignment:
            self._object_masks = []
            return
        background = fit_reference(self._background, self._reference.shape[:2])
        self._object_masks = segment_objects(foreground_mask(self._reference, background))

    def _on_frame(self, image: Image) -> None:
        with self._lock:
            self._latest = image
            self._frames += 1
            max_fps = self.config.max_fps
            if max_fps > 0:
                now = time.monotonic()
                if now - self._last_publish < 1.0 / max_fps:
                    return
                self._last_publish = now
        out = self._compose(image)
        if out is not None:
            self.overlay_image.publish(out)

    def _compose(self, image: Image) -> Image | None:
        """Live frame with the reference over it; None on any error so the subscription survives."""
        try:
            live = image.to_opencv()
            with self._lock:
                reference = self._reference
                background = self._background
                object_masks = self._object_masks
                opacity = self._opacity
                mode = self._mode
                scene = self._scene
            scores: list[float] = []
            if object_masks and background is not None and reference is not None:
                if reference.shape[:2] == live.shape[:2]:
                    live_mask = foreground_mask(live, fit_reference(background, live.shape[:2]))
                    scores = alignment_scores(live_mask, object_masks)
            if reference is not None:
                if reference.shape[:2] != live.shape[:2] and not self._resize_warned:
                    logger.warning(
                        "Reference %s is %sx%s but the camera is %sx%s; resizing the reference",
                        scene,
                        reference.shape[1],
                        reference.shape[0],
                        live.shape[1],
                        live.shape[0],
                    )
                    self._resize_warned = True
                reference = fit_reference(reference, live.shape[:2])
                live = (
                    blend(live, reference, opacity)
                    if mode == "blend"
                    else edge_overlay(live, reference)
                )
            else:
                live = live.copy()
            if scores:
                draw_alignment(live, object_masks, scores)
                with self._lock:
                    self._scores = scores
            if self.config.label:
                self._draw_label(live, scene, reference is not None, opacity, mode, scores)
            return Image.from_opencv(live, frame_id=image.frame_id, ts=image.ts)
        except Exception:
            logger.exception("Scene overlay failed for one frame")
            return None

    @staticmethod
    def _draw_label(
        frame: np.ndarray,
        scene: str,
        has_reference: bool,
        opacity: float,
        mode: OverlayMode,
        scores: list[float],
    ) -> None:
        import cv2

        if has_reference:
            text = f"{scene}  {mode}" + (f" {opacity:.0%}" if mode == "blend" else "")
            if scores:
                text += f"  aligned {sum(scores) / len(scores):.0%}"
        elif scene:
            text = f"{scene}  no reference, call capture_reference"
        else:
            text = "no scene selected, call set_scene"
        cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(
            frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
        )
