"""FindSkillContainer — explore + find anything using YOLO+CLIP in the background.

Architecture:
  - Declares explore_cmd / stop_explore_cmd Out streams → auto-wires to WavefrontFrontierExplorer
  - Declares odom In stream → knows where it found the target
  - YOLO+CLIP runs in background while the robot is moving
  - Found → stops exploration, reports position, optionally hands off to SmartFollow
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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool
from dimos.robot.unitree.go2.blueprints.hackathon.frame_writer import write_annotated_frame, write_state
from dimos.robot.unitree.go2.blueprints.hackathon.perception_loop import PerceptionLoopModule, set_active_mode

logger = setup_logger()

_MATCH_THRESHOLD = 0.22

# Full YOLO11 COCO class list — these skip CLIP, pure class filter
_YOLO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
}


class Config(ModuleConfig):
    pass


class _Det:
    """Wrap dict-returned detections from PerceptionLoop with the small attr
    surface the rest of this module needs (.track_id / .bbox / .name)."""
    __slots__ = ("track_id", "bbox", "name", "confidence")
    def __init__(self, m: dict) -> None:
        self.track_id = m.get("track_id", -1)
        self.bbox = tuple(m["bbox"])
        self.name = m.get("name", "")
        self.confidence = m.get("score", m.get("confidence", 0.0))
    def bbox_2d_volume(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


class FindSkillContainer(Module):
    """Find + explore skill.

    Starts autonomous frontier exploration while watching the camera for a target.
    When found, stops exploration and reports position to Claude via tool_update.
    Shares CLIP + YOLO with PerceptionLoop via RPC (no local model loads).
    """

    color_image: In[Image]
    odom: In[PoseStamped]
    # Shared inference layer — ONE MobileCLIP + ONE YOLO across all skills.
    _perception: PerceptionLoopModule

    # These wire directly to WavefrontFrontierExplorer via name+type matching
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # No own MobileCLIP / YOLO — both come from PerceptionLoop via RPC.
        # Saves ~1.85 GB of duplicated model weights per skill module.
        self._latest_image: Image | None = None
        self._latest_odom: PoseStamped | None = None
        self._thread: Thread | None = None
        self._should_stop = Event()
        self._lock = RLock()

    @rpc
    def start(self) -> None:
        super().start()
        # CLIP loads lazily on first embed — eager start() caused concurrent
        # multi-process model loads to stall startup.
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

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

    def _on_odom(self, odom: PoseStamped) -> None:
        with self._lock:
            self._latest_odom = odom

    @skill
    def smart_find(self, query: str, explore: bool = True, then_approach: bool = False, timeout_s: float = 0.0) -> str:
        """Explore the environment while searching for a specific person or object.

        Starts autonomous frontier exploration and watches the camera for the target.
        When found, stops exploration and reports back. If then_approach=True, the robot
        will also move close to the object after finding it.

        Works for people ("person with red shirt"), YOLO objects ("chair", "dog", "bottle"),
        or any visual description.

        Args:
            query: What to look for, e.g. "red chair", "dog", "person with backpack"
            explore: If True (default), start autonomous exploration while searching.
            then_approach: If True, automatically approach and get close after finding.
            timeout_s: Give up after this many seconds (default 120s).

        Returns:
            Immediate confirmation. Will send tool_update when found or timed out.
        """
        self._should_stop.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        self._should_stop.clear()

        with self._lock:
            image = self._latest_image

        if image is None:
            return "No camera image yet — is DimOS running?"

        if explore:
            cmd = Bool()
            cmd.data = True
            self.explore_cmd.publish(cmd)

        set_active_mode("find")
        self.start_tool("smart_find")
        self._thread = Thread(
            target=self._find_loop,
            args=(query, explore, then_approach, timeout_s),
            daemon=True,
            name="SmartFind",
        )
        self._thread.start()

        return (
            f"Searching for '{query}'. "
            + ("Exploration started. " if explore else "Watching without moving. ")
            + f"Timeout: {int(timeout_s)}s. Will update you when found."
        )

    @skill
    def stop_find(self) -> str:
        """Stop an active find/explore mission and halt exploration."""
        self._should_stop.set()
        set_active_mode(None)
        self._send_stop_explore()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        return "Find mission stopped."

    def _send_stop_explore(self) -> None:
        cmd = Bool()
        cmd.data = True
        self.stop_explore_cmd.publish(cmd)

    def _get_detections(self, image: Image, class_filter: str | None = None) -> list:
        """Pull detections from the shared PerceptionLoop (RPC)."""
        try:
            raw = self._perception.current_detections(class_filter=class_filter)
        except Exception:
            return []
        return [_Det(m) for m in raw]

    def _find_loop(self, query: str, exploring: bool, then_approach: bool, timeout_s: float) -> None:
        class_mode = query.lower() in _YOLO_CLASSES

        start_time  = time.monotonic()
        frame_count = 0
        clip_interval = 5

        unbounded = timeout_s <= 0
        while not self._should_stop.is_set():
            elapsed = time.monotonic() - start_time
            if not unbounded and elapsed > timeout_s:
                if exploring:
                    self._send_stop_explore()
                self.tool_update("smart_find", f"Search timed out after {int(elapsed)}s. '{query}' not found.")
                set_active_mode(None)
                self.stop_tool("smart_find")
                return

            with self._lock:
                image = self._latest_image
                odom  = self._latest_odom

            if image is None:
                time.sleep(0.1)
                continue

            frame_count += 1

            if class_mode:
                matches = self._get_detections(image, class_filter=query.lower())
                if matches:
                    best = max(matches, key=lambda d: d.bbox_2d_volume())
                    try:
                        all_dets = self._get_detections(image)
                        det_list = [{"bbox": d.bbox, "track_id": d.track_id,
                                     "label": "FOUND" if d.track_id == best.track_id else None}
                                    for d in all_dets]
                        write_annotated_frame(image.to_opencv(), det_list, overlay_text=f"FIND\nFOUND\n{query}")
                        write_state({"mode": "find", "state": "FOUND", "query": query})
                    except Exception:
                        pass
                    set_active_mode(None)
                    self._report_found(query, best, odom, exploring, then_approach)
                    return

            elif frame_count % clip_interval == 0:
                # Shared-CLIP scoring via PerceptionLoop — no local inference,
                # no duplicate MobileCLIP load.
                try:
                    ranked = self._perception.match_text(query)
                except Exception:
                    ranked = []
                if ranked and ranked[0]["score"] >= _MATCH_THRESHOLD:
                    best = _Det(ranked[0])
                    try:
                        all_dets = self._get_detections(image)
                        det_list = [{"bbox": d.bbox, "track_id": d.track_id,
                                     "label": "FOUND" if d.track_id == best.track_id else None}
                                    for d in all_dets]
                        write_annotated_frame(image.to_opencv(), det_list, overlay_text=f"FIND\nFOUND\n{query}")
                        write_state({"mode": "find", "state": "FOUND", "query": query})
                    except Exception:
                        pass
                    set_active_mode(None)
                    self._report_found(query, best, odom, exploring, then_approach)
                    return

            # Annotate searching frames every 3 ticks
            if frame_count % 3 == 0:
                try:
                    cur_dets = self._get_detections(image)
                    det_list = [{"bbox": d.bbox, "track_id": d.track_id} for d in cur_dets]
                    write_annotated_frame(image.to_opencv(), det_list, overlay_text=f"FIND\nSEARCHING\n{query}")
                    write_state({"mode": "find", "state": "SEARCHING", "query": query})
                except Exception:
                    pass

            time.sleep(1.0 / 15.0)

        if exploring:
            self._send_stop_explore()
        set_active_mode(None)
        self.tool_update("smart_find", f"Find mission for '{query}' was stopped.")
        self.stop_tool("smart_find")

    def _report_found(self, query: str, detection: Any, odom: PoseStamped | None, exploring: bool, then_approach: bool) -> None:
        if exploring:
            self._send_stop_explore()

        pos_str = (
            f"x={odom.x:.2f}, y={odom.y:.2f}" if odom is not None else "position unknown"
        )
        x1, y1, x2, y2 = detection.bbox
        center_x = (x1 + x2) / 2
        img_w = detection.image.width if hasattr(detection, "image") else 1280
        side = "left" if center_x < img_w * 0.4 else "right" if center_x > img_w * 0.6 else "center"

        approach_hint = f" Now call smart_approach('{query}') to move close." if then_approach else ""
        self.tool_update(
            "smart_find",
            f"Found '{query}'! Robot at {pos_str}, target is {side} of frame.{approach_hint}",
        )
        self.stop_tool("smart_find")
