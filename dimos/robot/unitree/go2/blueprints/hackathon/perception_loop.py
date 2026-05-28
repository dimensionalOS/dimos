"""PerceptionLoopModule — always-on L2 shared perception layer.

Runs YOLO at ~15fps. Maintains a rolling SceneBuffer (2s TTL) with MobileCLIP
embeddings. Exposes a process-singleton so smart_follow / find / dog_mode all
read from *one* YOLO pass instead of each running their own detector.

Cache hierarchy role:
  HDD  Claude via MCP          — novel situations only
  SSD  dimOS SQLite map        — spatial memory
  RAM  BehaviorFSMs            — smart_follow, dog_mode recovery
  L2   This module             — always-on YOLO+MobileCLIP, SceneBuffer <50ms
  L1   VisualServoing2D        — servo math <1ms

Module-level API (used by smart_follow / find / dog_mode):
  get_shared_detections() → (list[Detection], timestamp_float)
  get_shared_buffer()     → dict[track_id, TrackedObject]
  set_active_mode(str|None) — active mode suppresses idle frame-writing
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from threading import Event, RLock, Thread
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.models.embedding.mobileclip import MobileCLIPModel
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.utils.logging_config import setup_logger
from dimos.robot.unitree.go2.blueprints.hackathon.frame_writer import write_annotated_frame, write_state

logger = setup_logger()

_BUFFER_TTL     = 2.0   # seconds before a track expires
_CLIP_INTERVAL  = 5     # MobileCLIP every N frames (~3fps on CPU)
_DET_FRESHNESS  = 0.12  # shared detection max age in seconds
_LOOP_HZ        = 15.0
_CLIP_THRESHOLD = 0.20

# ── Process-singleton shared state ───────────────────────────────────────────
_shared_lock       = RLock()
_shared_detections: list = []
_shared_det_ts: float   = 0.0
_shared_buffer: dict    = {}   # track_id → TrackedObject
_active_mode: str | None = None  # "follow" | "find" | "dog" | None


def get_shared_detections() -> tuple[list, float]:
    """Return (detections, monotonic_ts) from the shared YOLO pass."""
    with _shared_lock:
        return list(_shared_detections), _shared_det_ts


def get_shared_buffer() -> dict:
    """Return a snapshot of the current SceneBuffer."""
    with _shared_lock:
        return dict(_shared_buffer)


def set_active_mode(mode: str | None) -> None:
    """Signal that a behaviour module is active (suppresses idle frame-writing)."""
    global _active_mode
    with _shared_lock:
        _active_mode = mode


def shared_detections_fresh() -> bool:
    """True if the shared YOLO pass ran within the freshness window."""
    with _shared_lock:
        return (time.monotonic() - _shared_det_ts) <= _DET_FRESHNESS


# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class TrackedObject:
    track_id:   int
    name:       str
    bbox:       tuple
    confidence: float
    last_seen:  float
    clip_emb:   Any | None = field(default=None, repr=False)  # Embedding | None


class Config(ModuleConfig):
    camera_info: CameraInfo


class PerceptionLoopModule(Module):
    """Always-on L2 perception: YOLO+MobileCLIP, shared SceneBuffer, query_scene skill."""

    config: Config
    color_image: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._detector    = Yolo2DDetector()
        self._clip        = MobileCLIPModel()
        self._lock        = RLock()
        self._latest_image: Image | None = None
        self._last_fast_write: float = 0.0  # gate for _on_image dashboard writes
        self._should_stop = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        self._should_stop.clear()
        self._thread = Thread(target=self._perception_loop, daemon=True, name="PerceptionLoop")
        self._thread.start()
        logger.info("PerceptionLoopModule started.")

    @rpc
    def stop(self) -> None:
        self._should_stop.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _on_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    # ── Shared-CLIP RPCs (so SmartFollow/Find don't each load their own) ──

    @rpc
    def embed_text(self, text: str) -> list[float]:
        """Embed a text query using the SHARED MobileCLIP (no duplicate model loads).
        Returns the embedding as a plain list[float] (RPC-serializable)."""
        emb = self._clip.embed_text(text)
        return emb.vector.detach().cpu().float().tolist()

    @rpc
    def match_text(self, text: str, class_filter: str | None = None) -> list[dict]:
        """Score tracked objects against `text` using the shared CLIP.
        Returns [{track_id, bbox, name, score}] sorted by score desc.
        Uses cached track embeddings — zero extra inference per frame.
        Optional class_filter narrows to a YOLO class (e.g. 'person')."""
        text_emb = self._clip.embed_text(text)
        now = time.monotonic()
        out = []
        with _shared_lock:
            for t in _shared_buffer.values():
                if now - t.last_seen > _BUFFER_TTL:
                    continue
                if class_filter and t.name.lower() != class_filter.lower():
                    continue
                if t.clip_emb is None:
                    continue
                try:
                    score = float(text_emb @ t.clip_emb)
                except Exception:
                    continue
                out.append({
                    "track_id": int(t.track_id),
                    "bbox": [float(v) for v in t.bbox],
                    "name": t.name,
                    "score": score,
                })
        out.sort(key=lambda m: m["score"], reverse=True)
        return out

    @rpc
    def current_detections(self, class_filter: str | None = None) -> list[dict]:
        """Snapshot of the live YOLO detections (fresh-window only).
        Returns [{track_id, bbox, name, confidence}]."""
        with _shared_lock:
            dets = list(_shared_detections)
            ts = _shared_det_ts
        if (time.monotonic() - ts) > _DET_FRESHNESS:
            return []
        out = []
        for d in dets:
            name = getattr(d, "name", "")
            if class_filter and name.lower() != class_filter.lower():
                continue
            bbox = getattr(d, "bbox", None)
            if bbox is None:
                continue
            out.append({
                "track_id": int(getattr(d, "track_id", -1) or -1),
                "bbox": [float(v) for v in bbox],
                "name": name,
                "confidence": float(getattr(d, "confidence", 0.0) or 0.0),
            })
        return out

    # ── Skill ─────────────────────────────────────────────────────────────────

    @skill
    def query_scene(self, description: str = "") -> str:
        """Ask what the robot's camera currently sees — no new inference triggered.

        Reads directly from the live SceneBuffer (updated at ~15fps). Use this
        before making follow/find decisions — it's cheap and instant.

        Args:
            description: Optional text to filter by similarity (e.g. "person with
                         red shirt", "dog", "chair"). Leave empty to list everything.
        """
        now = time.monotonic()
        with _shared_lock:
            tracks = [t for t in _shared_buffer.values() if now - t.last_seen <= _BUFFER_TTL]

        if not tracks:
            return "Nothing detected in current frame."

        if description:
            text_emb = self._clip.embed_text(description)
            scored = []
            for t in tracks:
                if t.clip_emb is not None:
                    score = float(text_emb @ t.clip_emb)
                    scored.append((score, t))
            scored.sort(key=lambda x: x[0], reverse=True)
            matches = [(s, t) for s, t in scored if s >= _CLIP_THRESHOLD]
            if not matches:
                names = ", ".join(sorted({t.name for t in tracks}))
                return f"No match for '{description}'. Visible: {names}."
            lines = [f"Matches for '{description}':"]
            for score, t in matches[:5]:
                x1, _, x2, _ = t.bbox
                cx = (x1 + x2) / 2
                side = "left" if cx < 0.4 else "right" if cx > 0.6 else "center"
                lines.append(f"  #{t.track_id} {t.name} ({score:.0%}) — {side} of frame")
            return "\n".join(lines)

        by_class: dict[str, list] = {}
        for t in tracks:
            by_class.setdefault(t.name, []).append(t)
        lines = [f"{len(tracks)} object(s) visible:"]
        for name, objs in sorted(by_class.items()):
            ids = ", ".join(f"#{o.track_id}" for o in objs)
            lines.append(f"  {name} × {len(objs)}  [{ids}]")
        return "\n".join(lines)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _perception_loop(self) -> None:
        global _shared_detections, _shared_det_ts, _shared_buffer

        frame_count = 0
        period = 1.0 / _LOOP_HZ
        next_t = time.monotonic()

        while not self._should_stop.is_set():
            next_t += period
            frame_count += 1
            now = time.monotonic()

            with self._lock:
                image = self._latest_image

            if image is None:
                time.sleep(period)
                continue

            # YOLO every frame
            result = self._detector.process_image(image)
            valid  = [d for d in result.detections if d.is_valid()]

            # MobileCLIP every N frames
            embs: list = [None] * len(valid)
            if valid and frame_count % _CLIP_INTERVAL == 0:
                try:
                    crops = [d.cropped_image() for d in valid]
                    raw = self._clip.embed(*crops) if len(crops) > 1 else [self._clip.embed(crops[0])]
                    embs = raw
                except Exception:
                    pass

            # Write into shared singleton
            with _shared_lock:
                _shared_detections = valid
                _shared_det_ts     = now
                for det, emb in zip(valid, embs):
                    tid      = det.track_id
                    existing = _shared_buffer.get(tid)
                    _shared_buffer[tid] = TrackedObject(
                        track_id   = tid,
                        name       = getattr(det, "name", "object"),
                        bbox       = det.bbox,
                        confidence = float(getattr(det, "confidence", 0.0)),
                        last_seen  = now,
                        clip_emb   = emb if emb is not None else (existing.clip_emb if existing else None),
                    )
                _shared_buffer = {k: v for k, v in _shared_buffer.items()
                                  if now - v.last_seen <= _BUFFER_TTL}

            # Frame annotation — every loop iteration (was throttled to every
            # 3rd, which capped the dashboard camera at ~5fps; YOLO is the real
            # gate so writing every frame just keeps up with the loop).
            if True:
                with _shared_lock:
                    mode = _active_mode
                if mode is None:
                    try:
                        snap = [t for t in _shared_buffer.values() if now - t.last_seen <= _BUFFER_TTL]
                        det_list = [{"bbox": t.bbox, "track_id": t.track_id, "label": t.name} for t in snap]
                        write_annotated_frame(image.to_opencv(), det_list, overlay_text="PERCEPTION\nACTIVE")
                        write_state({
                            "mode":    "perception",
                            "state":   "ACTIVE",
                            "objects": len(det_list),
                            "classes": list({t.name for t in snap}),
                        })
                    except Exception:
                        pass

            sleep_dur = next_t - time.monotonic()
            if sleep_dur > 0:
                time.sleep(sleep_dur)
