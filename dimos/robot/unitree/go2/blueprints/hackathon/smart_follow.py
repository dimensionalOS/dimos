"""SmartFollowSkillContainer — multi-target YOLO+MobileCLIP person/object follower.

L2: reads shared YOLO detections from PerceptionLoopModule (zero duplicate inference).
    Falls back to its own Yolo2DDetector if the shared pass is stale.
RAM: active_track_ids set, SpinSearch recovery, nearest-target servo.
L1: VisualServoing2D control loop.

Design:
- YOLO servo uses shared detections (already running in PerceptionLoopModule)
- CLIP re-scoring reads embeddings from SceneBuffer — no redundant CLIP calls
- Multi-target: servo toward nearest active match (largest bbox = closest)
- Class-only fast path: if query is a YOLO class name, skip CLIP entirely
"""

from __future__ import annotations

import time
from threading import Event, RLock, Thread
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.unitree.go2.blueprints.hackathon.perception_loop import PerceptionLoopModule
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D
from dimos.utils.logging_config import setup_logger
from dimos.robot.unitree.go2.blueprints.hackathon.frame_writer import write_annotated_frame, write_state
from dimos.robot.unitree.go2.blueprints.hackathon.perception_loop import get_shared_buffer, set_active_mode

logger = setup_logger()

_YOLO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "cat", "dog", "horse", "sheep", "cow", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "sports ball", "bottle",
    "cup", "chair", "couch", "potted plant", "bed", "tv", "laptop", "phone",
    "keyboard", "mouse", "remote", "microwave", "oven", "sink", "refrigerator", "book",
}

_MATCH_THRESHOLD    = 0.22
_REACQUIRE_THRESHOLD = 0.20


class Config(ModuleConfig):
    camera_info: CameraInfo


class _Det:
    """Lightweight wrapper around dict-returned detections from PerceptionLoop.
    Exposes the .track_id / .bbox / .name / .bbox_2d_volume() API the rest of
    this module already expects, so we can swap detector source without
    rewriting the follow loop."""
    __slots__ = ("track_id", "bbox", "name", "confidence")
    def __init__(self, m: dict) -> None:
        self.track_id = m.get("track_id", -1)
        self.bbox = tuple(m["bbox"])
        self.name = m.get("name", "")
        self.confidence = m.get("score", m.get("confidence", 0.0))
    def bbox_2d_volume(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)
    def is_valid(self) -> bool:
        return True


class SmartFollowSkillContainer(Module):
    """Multi-target person/object follower using shared L2 perception."""

    config: Config
    color_image: In[Image]
    cmd_vel: Out[Twist]
    # Shared inference: ONE MobileCLIP + ONE YOLO lives in PerceptionLoop.
    # We don't load our own; all detection + text matching is RPC'd.
    _perception: PerceptionLoopModule

    _frequency: float       = 15.0
    _clip_interval: int     = 5      # re-score from SceneBuffer every N frames
    _max_lost_frames: int   = 20
    _spin_speed: float      = 0.45
    _spin_timeout_s: float  = 12.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # No more own MobileCLIP / Yolo2DDetector — both come from PerceptionLoop
        # via RPC. Saves ~1.8 GB of duplicated model weights per skill module.
        self._visual_servo = VisualServoing2D(self.config.camera_info)

        self._latest_image: Image | None = None
        self._active_ids: set[int]       = set()
        self._query: str                 = ""
        self._text_emb                   = None
        self._class_mode: bool           = False
        self._class_name: str            = ""

        self._thread: Thread | None = None
        self._should_stop = Event()
        self._lock = RLock()
        self._tool_name: str = "smart_follow_person"

    @rpc
    def start(self) -> None:
        super().start()
        # CLIP loads lazily on first embed (see _clip cached model) — eager
        # start() here caused concurrent multi-process model loads to stall.
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))

    @rpc
    def stop(self) -> None:
        self._should_stop.set()
        set_active_mode(None)
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _on_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _get_detections(self, image: Image, class_filter: str | None = None) -> list:
        """Pull detections from the shared PerceptionLoop (RPC). image param
        retained for signature compatibility; not used now (PerceptionLoop has
        its own subscription to the same color_image stream)."""
        try:
            raw = self._perception.current_detections(class_filter=class_filter)
        except Exception:
            return []
        return [_Det(m) for m in raw]

    # ── Skills ────────────────────────────────────────────────────────────────

    @skill
    def smart_follow_person(self, query: str) -> str:
        """Follow one or more people matching a description.

        YOLO+MobileCLIP instead of QwenVL — faster lock-on, auto SpinSearch recovery.
        If multiple people match, follows the nearest. Switches target automatically.

        Args:
            query: e.g. "gray pants", "black shirt", "woman with backpack"
        """
        return self._start_follow(query, mode="person")

    @skill
    def smart_follow_object(self, class_name: str) -> str:
        """Follow any YOLO-detectable object by class name (no CLIP — pure fast path).

        Args:
            class_name: YOLO class: "dog", "cat", "chair", "bottle", "person", etc.
        """
        return self._start_follow(class_name, mode="object")

    @skill
    def smart_approach(self, query: str, stop_distance_m: float = 0.8) -> str:
        """Move toward a person or object and stop when close enough.

        Args:
            query: Description or YOLO class name.
            stop_distance_m: Stop distance in meters (default 0.8m).
        """
        self._should_stop.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._should_stop.clear()

        with self._lock:
            image = self._latest_image
        if image is None:
            return "No camera image yet."

        if query.lower() in _YOLO_CLASSES:
            self._class_mode  = True
            self._class_name  = query.lower()
            self._text_emb    = None
            self._query       = query
            matches = self._get_detections(image, class_filter=self._class_name)
        else:
            self._class_mode  = False
            self._query       = query
            self._text_emb    = None  # PerceptionLoop holds the embedding
            ranked = self._perception.match_text(query, class_filter="person")
            matches = [_Det(m) for m in ranked if m["score"] >= _MATCH_THRESHOLD]
            if not matches:
                return "No persons matching that description in current scene."

        if not matches:
            return f"Cannot see '{query}' — use smart_find first."

        self._active_ids = {d.track_id for d in matches}
        set_active_mode("follow")
        self.start_tool("smart_approach")
        self._thread = Thread(target=self._approach_loop, args=(stop_distance_m,), daemon=True, name="SmartApproach")
        self._thread.start()
        return f"Approaching '{query}', stopping at {stop_distance_m}m."

    @skill
    def stop_following(self) -> str:
        """Stop following or approaching."""
        self._should_stop.set()
        set_active_mode(None)
        self.cmd_vel.publish(Twist.zero())
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        return "Stopped."

    # ── Loops ─────────────────────────────────────────────────────────────────

    def _approach_loop(self, stop_distance_m: float) -> None:
        lost_count = 0
        period     = 1.0 / self._frequency
        next_time  = time.monotonic()

        while not self._should_stop.is_set():
            next_time += period
            with self._lock:
                image = self._latest_image
            if image is None:
                time.sleep(period)
                continue

            if self._class_mode:
                candidates = self._get_detections(image, class_filter=self._class_name)
                self._active_ids = {d.track_id for d in candidates}
            else:
                candidates = self._get_detections(image, class_filter="person")

            active = [d for d in candidates if d.track_id in self._active_ids]

            if active:
                lost_count = 0
                target = max(active, key=lambda d: d.bbox_2d_volume())
                dist   = self._visual_servo._estimate_distance(target.bbox)
                if dist is not None and dist <= stop_distance_m:
                    self.cmd_vel.publish(Twist.zero())
                    self.tool_update("smart_approach", f"Arrived. Distance: {dist:.2f}m.")
                    self.stop_tool("smart_approach")
                    set_active_mode(None)
                    return
                self.cmd_vel.publish(self._visual_servo.compute_twist(target.bbox, image.width))
            else:
                lost_count += 1
                self.cmd_vel.publish(Twist.zero())
                if lost_count > self._max_lost_frames * 2:
                    self.tool_update("smart_approach", "Lost target during approach.")
                    self.stop_tool("smart_approach")
                    set_active_mode(None)
                    return

            sleep_dur = next_time - time.monotonic()
            if sleep_dur > 0:
                time.sleep(sleep_dur)

        self.cmd_vel.publish(Twist.zero())
        set_active_mode(None)
        self.stop_tool("smart_approach")

    def _start_follow(self, query: str, mode: str) -> str:
        self._should_stop.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._should_stop.clear()

        with self._lock:
            image = self._latest_image
        if image is None:
            return "No camera image yet — is DimOS running?"

        if mode == "object" or query.lower() in _YOLO_CLASSES:
            self._class_mode  = True
            self._class_name  = query.lower()
            self._text_emb    = None
            self._query       = query
            # Use the SceneBuffer (2s TTL) instead of current_detections() (120ms TTL)
            # so MCP round-trip + thread-join latency don't produce a false "not visible"
            # result for objects that were clearly in frame when the user clicked.
            _now = time.monotonic()
            _buf = get_shared_buffer()
            _cls = query.lower()
            buf_matches = [t for t in _buf.values()
                           if t.name.lower() == _cls and (_now - t.last_seen) <= 2.0]
            self._active_ids = {t.track_id for t in buf_matches}
            self._tool_name = "smart_follow_object"
            set_active_mode("follow")
            self.start_tool(self._tool_name)
            self._thread = Thread(target=self._follow_loop, daemon=True, name="SmartFollow")
            self._thread.start()
            if not buf_matches:
                return f"No '{query}' visible — will search when spinning."
            return f"Tracking {len(buf_matches)} '{query}' (ids={list(self._active_ids)}). Following nearest."

        # Person mode: use shared CLIP via PerceptionLoop.match_text — scores
        # cached track embeddings (zero extra inference) and returns ranked
        # candidates with bbox/score. Same effective semantics as the old
        # SceneBuffer fast path, just across-process instead of broken local.
        self._class_mode = False
        self._query      = query
        self._text_emb   = None

        ranked = self._perception.match_text(query, class_filter="person")
        matches = [_Det(m) for m in ranked if m["score"] >= _MATCH_THRESHOLD]
        self._active_ids = {d.track_id for d in matches}

        self._tool_name = "smart_follow_person"
        set_active_mode("follow")
        self.start_tool(self._tool_name)
        self._thread = Thread(target=self._follow_loop, daemon=True, name="SmartFollow")
        self._thread.start()

        if not matches:
            if ranked:
                return (f"No match for '{query}' (best={ranked[0]['score']:.2f}, "
                        f"need {_MATCH_THRESHOLD}). Spinning to search.")
            return "No persons in frame — spinning to search."
        return f"Locked {len(matches)} match(es) for '{query}' (ids={list(self._active_ids)})."

    def _follow_loop(self) -> None:
        lost_count  = 0
        spin_start: float | None = None
        frame_count = 0
        period      = 1.0 / self._frequency
        next_time   = time.monotonic()

        while not self._should_stop.is_set():
            next_time  += period
            frame_count += 1

            with self._lock:
                image = self._latest_image
            if image is None:
                time.sleep(period)
                continue

            # Get detections — shared YOLO pass when fresh
            if self._class_mode:
                candidates = self._get_detections(image, class_filter=self._class_name)
                self._active_ids = {d.track_id for d in candidates}
            else:
                candidates = self._get_detections(image, class_filter="person")
                # Re-score conditions:
                #   1) IMMEDIATE if we have candidates but none of them match our
                #      current active_ids. This is what stops the 360 spin the
                #      moment the target comes into view (was: only every 5
                #      frames, so the bot would happily rotate past the target).
                #   2) Periodic during steady tracking to keep active_ids
                #      refreshed against shape/lighting changes.
                _none_active_visible = candidates and not any(
                    d.track_id in self._active_ids for d in candidates
                )
                _should_rescore = candidates and self._query and (
                    _none_active_visible or frame_count % self._clip_interval == 0
                )
                if _should_rescore:
                    try:
                        ranked = self._perception.match_text(self._query, class_filter="person")
                    except Exception:
                        ranked = []
                    if ranked:
                        new_active = {m["track_id"] for m in ranked if m["score"] >= _MATCH_THRESHOLD}
                        ranked_ids = {m["track_id"] for m in ranked}
                        # Only swap if PerceptionLoop has scored any of our candidates.
                        if any(d.track_id in ranked_ids for d in candidates):
                            self._active_ids = new_active

            active = [d for d in candidates if d.track_id in self._active_ids]
            # If we just (re)acquired the target while spinning, kill the spin
            # immediately — don't finish the rotation before reacting.
            if active and spin_start is not None:
                spin_start = None
                lost_count = 0
                self.cmd_vel.publish(Twist.zero())

            if active:
                target = max(active, key=lambda d: d.bbox_2d_volume())
                self.cmd_vel.publish(self._visual_servo.compute_twist(target.bbox, image.width))
                lost_count = 0
                spin_start = None
            else:
                lost_count += 1
                self.cmd_vel.publish(Twist.zero())

                if lost_count > self._max_lost_frames:
                    if spin_start is None:
                        spin_start = time.monotonic()
                        self.tool_update(self._tool_name, f"Lost '{self._query}' — spinning to search.")

                    if time.monotonic() - spin_start > self._spin_timeout_s:
                        self.tool_update(self._tool_name,
                                         f"Full 360° search failed. Could not re-acquire '{self._query}'.")
                        self.stop_tool(self._tool_name)
                        set_active_mode(None)
                        self.cmd_vel.publish(Twist.zero())
                        return

                    self.cmd_vel.publish(
                        Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, self._spin_speed))
                    )

            # Dashboard frame every 3 ticks
            if frame_count % 3 == 0:
                try:
                    state_label = "LOST" if lost_count > self._max_lost_frames else "FOLLOWING"
                    det_list = [{"bbox": d.bbox, "track_id": d.track_id,
                                 "label": "TARGET" if d.track_id in self._active_ids else None}
                                for d in candidates]
                    write_annotated_frame(image.to_opencv(), det_list,
                                          overlay_text=f"FOLLOW\n{state_label}\n{self._query}")
                    write_state({"mode": "follow", "state": state_label,
                                 "query": self._query, "targets": len(active)})
                except Exception:
                    pass

            sleep_dur = next_time - time.monotonic()
            if sleep_dur > 0:
                time.sleep(sleep_dur)

        self.cmd_vel.publish(Twist.zero())
        set_active_mode(None)
        self.tool_update(self._tool_name, "Following stopped.")
        self.stop_tool(self._tool_name)
