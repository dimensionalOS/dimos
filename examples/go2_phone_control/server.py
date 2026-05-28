#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Go2 phone-controller server (Scheme B: phone = remote, Mac = brain).

The Mac holds a single persistent WebRTC connection to the dog. A mobile-friendly
web page (served at /) sends button taps to this server over HTTP; the server
translates each tap into a robot command and returns the result. The phone needs
nothing but a browser on the same Wi-Fi.

v1 actions:
    1. Stand up
    2. Sit down
    3. March in place (N reps)
    4. D-pad: forward / back / left / right — one tap = one step
    5. Wave hello
    6. Observe people nearby (YOLO on one camera frame)

Run (on the Mac, while on the dog's Wi-Fi):
    .venv/bin/python examples/go2_phone_control/server.py --ip 192.168.12.1
    # then open http://<MAC_LAN_IP>:8800/ on your phone

Options:
    --port 8800
    --step-distance 0.3   # meters per D-pad tap
    --step-speed 0.3      # m/s for D-pad / march pulses
    --close-ratio 0.5     # person bbox-height/frame-height to count as "near"
"""

from __future__ import annotations

import argparse
import base64
import math
from pathlib import Path
import random
import threading
import time
from typing import Any

import uvicorn
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

from unitree_webrtc_connect.constants import WebRTCConnectionMethod

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import RTC_TOPIC, SPORT_CMD, UnitreeWebRTCConnection
from dimos.utils.data import get_data

# Real people tracker imports are lazy (inside __init__) — they pull heavy deps
# (torch, ultralytics, torchreid). MockPeopleTracker stays import-free so --mock
# starts instantly.

SPORT_TOPIC = RTC_TOPIC["SPORT_MOD"]
HERE = Path(__file__).parent


# --- Three-level identity model (MOCK for now) -----------------------------
# track_id      : short-term, from tracker; changes when a person re-appears.
# long_term_id  : stable unique person ID (real version = YOLO-E + OSNet Re-ID).
# name          : human-readable label (Harry Potter chars) mapped to long_term_id.
#
# MockPeopleTracker fakes the perception+reid pipeline so the two lists and the
# UI can be built/validated WITHOUT the camera, models, or dog. To go real,
# replace _loop() with a stream of detections (YOLO-E) fed through
# EmbeddingIDSystem; keep get_state()'s output shape identical.
HP_NAMES = [
    "Harry", "Hermione", "Ron", "Dumbledore", "Hagrid", "Snape", "Draco",
    "Luna", "Neville", "McGonagall", "Sirius", "Dobby", "Ginny", "Fred",
]


class MockPeopleTracker:
    def __init__(self, tick: float = 1.5, recent_window: float = 7.0) -> None:
        self._lock = threading.Lock()
        self.tick = tick
        self.recent_window = recent_window
        self._people: dict[int, dict[str, Any]] = {}  # long_term_id -> record
        self._next_long_id = 1
        self._next_track_id = 100
        self._name_pool = list(HP_NAMES)
        random.shuffle(self._name_pool)
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()
        print("[people] MOCK people tracker running (fake re-id + HP names)")

    @staticmethod
    def _mock_thumbnail(name: str, lid: int) -> str:
        """Generate a placeholder 'photo' the dog supposedly captured.

        Real version: replace with base64 JPEG of the YOLO bbox crop from the
        latest frame. Same `data:` URL form, so the UI doesn't change.
        """
        hue = (lid * 47) % 360
        initial = (name[:1] or "?").upper()
        svg = (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">'
            f'<rect width="48" height="48" fill="hsl({hue},45%,38%)"/>'
            f'<rect x="2" y="2" width="44" height="44" fill="none" '
            f'stroke="#0f380f" stroke-width="2" opacity="0.45"/>'
            f'<text x="24" y="33" font-family="ui-monospace,monospace" font-size="24" '
            f'font-weight="800" fill="#f5f5f0" text-anchor="middle">{initial}</text>'
            f'</svg>'
        )
        return "data:image/svg+xml;base64," + base64.b64encode(svg.encode()).decode()

    # World: 6m x 6m centered on dog. Dog faces +x. Front cone: +x with |angle|<45°.
    _WORLD_HALF = 3.0
    _CONE_HALF_ANGLE = math.radians(45)
    _SIGHT_RANGE = 4.0

    def _new_person(self) -> int:
        lid = self._next_long_id
        self._next_long_id += 1
        name = self._name_pool.pop(0) if self._name_pool else f"Wizard{lid}"
        # Spawn at a random spot in the world (some in cone, some not).
        x = random.uniform(-self._WORLD_HALF, self._WORLD_HALF)
        y = random.uniform(-self._WORLD_HALF, self._WORLD_HALF)
        self._people[lid] = {
            "long_term_id": lid,
            "name": name,
            "count": 0,
            "first_seen": time.time(),
            "last_seen": 0.0,
            "visible": False,
            "track_id": None,
            "thumbnail": self._mock_thumbnail(name, lid),
            "pos_x": x,
            "pos_y": y,
        }
        return lid

    def _in_cone(self, x: float, y: float) -> bool:
        """In the dog's front sight cone (forward = +x)."""
        if x <= 0:
            return False
        if math.hypot(x, y) > self._SIGHT_RANGE:
            return False
        return abs(math.atan2(y, x)) <= self._CONE_HALF_ANGLE

    def _enter(self, lid: int) -> None:
        """A person becomes visible: new short-term track_id, appearance count++."""
        p = self._people[lid]
        p["visible"] = True
        p["track_id"] = self._next_track_id
        self._next_track_id += 1
        p["count"] += 1
        p["last_seen"] = time.time()

    def _loop(self) -> None:
        # Seed a few wizards; their visibility falls out of position vs. cone.
        for _ in range(3):
            self._new_person()
        while not self._stop.is_set():
            time.sleep(self.tick)
            with self._lock:
                now = time.time()
                # Random walk: each person drifts a bit each tick.
                for p in self._people.values():
                    p["pos_x"] = max(-self._WORLD_HALF, min(self._WORLD_HALF,
                                     p["pos_x"] + random.uniform(-0.35, 0.35)))
                    p["pos_y"] = max(-self._WORLD_HALF, min(self._WORLD_HALF,
                                     p["pos_y"] + random.uniform(-0.35, 0.35)))
                # Visibility is purely geometric: in front cone & in range.
                for p in self._people.values():
                    in_cone = self._in_cone(p["pos_x"], p["pos_y"])
                    was_visible = p["visible"]
                    p["visible"] = in_cone
                    if in_cone:
                        # Re-entry into the cone bumps the count.
                        if not was_visible:
                            p["count"] += 1
                            p["track_id"] = self._next_track_id
                            self._next_track_id += 1
                        p["last_seen"] = now
                # Occasionally spawn a brand-new wizard somewhere.
                if random.random() < 0.18 and len(self._people) < 8:
                    self._new_person()

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            now = time.time()
            all_people = sorted(
                (
                    {
                        "long_term_id": p["long_term_id"],
                        "name": p["name"],
                        "count": p["count"],
                        "thumbnail": p["thumbnail"],
                        "pos_x": round(p["pos_x"], 2),
                        "pos_y": round(p["pos_y"], 2),
                        "visible": p["visible"],
                        "last_seen_ago": round(now - p["last_seen"], 1) if p["last_seen"] else None,
                    }
                    for p in self._people.values()
                ),
                key=lambda x: (-x["count"], x["long_term_id"]),
            )
            in_view = []
            for p in self._people.values():
                if p["last_seen"] > 0 and (now - p["last_seen"]) <= self.recent_window:
                    in_view.append(
                        {
                            "track_id": p["track_id"],
                            "long_term_id": p["long_term_id"],
                            "name": p["name"],
                            "secs_ago": round(now - p["last_seen"], 1),
                            "visible": p["visible"],
                            "thumbnail": p["thumbnail"],
                            "pos_x": round(p["pos_x"], 2),
                            "pos_y": round(p["pos_y"], 2),
                        }
                    )
            in_view.sort(key=lambda x: x["secs_ago"])
            return {"all": all_people, "in_view": in_view, "total": len(self._people)}


class RealPeopleTracker:
    """Real pipeline: YOLO person tracking + OSNet Re-ID for stable long-term IDs.

    Subscribes to the dog's WebRTC video stream, runs detection at ~3 Hz on a
    dedicated worker thread (rx thread only stashes the latest frame; older
    frames are dropped to avoid blocking the camera stream), assigns long-term
    unique IDs via dimos's EmbeddingIDSystem (OSNet body embeddings), maps each
    long-term ID to a Harry Potter name, and stores a JPEG thumbnail of the
    last bbox crop.

    Output of `get_state()` is identical in shape to MockPeopleTracker so the
    UI doesn't change.
    """

    def __init__(
        self,
        conn: UnitreeWebRTCConnection,
        osnet_variant: str = "osnet_x0_5",
        recent_window: float = 7.0,
        process_hz: float = 3.0,
    ) -> None:
        # Heavy imports kept local so mock mode stays fast.
        import cv2  # noqa: F401  -- used for JPEG encode
        from dimos.models.embedding.treid import TorchReIDModel, TorchReIDModelConfig
        from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
        from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem
        from dimos.robot.unitree.go2.connection import _camera_info_static

        self._cv2 = cv2
        # Camera intrinsics for pinhole pos estimation (shoulder-width assumption).
        ci = _camera_info_static()
        self._fx = float(ci.K[0])
        self._cx = float(ci.K[2])
        self._person_real_width = 0.45  # meters (shoulder), matches VisualServoing2D
        self._lock = threading.Lock()
        self.recent_window = recent_window
        self._tick = 1.0 / max(0.5, process_hz)

        print("[people] loading YOLO person detector ...")
        self._detector = YoloPersonDetector()
        print(f"[people] loading OSNet ({osnet_variant}) ...")
        self._embed = TorchReIDModel(TorchReIDModelConfig(model_name=osnet_variant))
        self._idsys = EmbeddingIDSystem(model=lambda: self._embed)

        self._people: dict[int, dict[str, Any]] = {}
        self._name_pool = list(HP_NAMES)
        random.shuffle(self._name_pool)

        # Latest-frame buffer (older frames dropped if worker is busy).
        self._latest: Any = None
        self._latest_ts: float = 0.0
        self._stop = threading.Event()

        # Subscribe to camera; rx thread only stashes (cheap).
        conn.video_stream().subscribe(self._on_frame)
        threading.Thread(target=self._worker, daemon=True).start()
        print(f"[people] REAL tracker running (target {process_hz:.1f} Hz)")

    def _on_frame(self, image: Any) -> None:
        # Replace prior frame; producer is faster than consumer by design.
        with self._lock:
            self._latest = image
            self._latest_ts = time.time()

    def _assign_name(self, lid: int) -> str:
        if self._name_pool:
            return self._name_pool.pop(0)
        return f"Wizard{lid}"

    def _make_thumb(self, detection: Any) -> str:
        """JPEG-encode the bbox crop → base64 data URL."""
        try:
            img = detection.cropped_image(padding=8)
            bgr = img.to_opencv()
            # Cap size to keep payload small in /api/people.
            h, w = bgr.shape[:2]
            if max(h, w) > 64:
                scale = 64.0 / max(h, w)
                bgr = self._cv2.resize(bgr, (int(w * scale), int(h * scale)))
            ok, buf = self._cv2.imencode(
                ".jpg", bgr, [int(self._cv2.IMWRITE_JPEG_QUALITY), 65]
            )
            if not ok:
                return ""
            return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()
        except Exception:
            return ""

    def _worker(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                image = self._latest
                self._latest = None
            if image is None:
                time.sleep(0.05)
                continue
            try:
                self._process(image)
            except Exception as e:  # noqa: BLE001
                print(f"[people] frame skipped: {e}")
            time.sleep(self._tick)

    def _process(self, image: Any) -> None:
        detections = self._detector.process_image(image)
        seen_lids: set[int] = set()
        now = time.time()

        for det in getattr(detections, "detections", []) or []:
            # YoloPersonDetector returns persons only; track_id present after
            # first BoT-SORT update (may be None on the very first detection).
            track_id = getattr(det, "track_id", None)
            if track_id is None:
                continue

            lid = self._idsys.register_detection(det)
            if lid is None or lid < 0:
                # Still warming up embeddings for this track (need ≥10).
                continue

            with self._lock:
                rec = self._people.get(lid)
                if rec is None:
                    rec = {
                        "long_term_id": lid,
                        "name": self._assign_name(lid),
                        "count": 0,
                        "first_seen": now,
                        "last_seen": 0.0,
                        "visible": False,
                        "track_id": None,
                        "thumbnail": "",
                        "bbox": None,            # latest (x1,y1,x2,y2) for follow
                        "image_width": 0,        # latest frame width
                        "pos_x": 0.0,            # dog-frame meters, +x forward
                        "pos_y": 0.0,            # dog-frame meters, +y left
                    }
                    self._people[lid] = rec
                # Re-entry: visible=False → True bumps the appearance count.
                if not rec["visible"]:
                    rec["count"] += 1
                rec["visible"] = True
                rec["track_id"] = int(track_id)
                rec["last_seen"] = now
                # Store the latest bbox so FollowController can servo on it.
                try:
                    x1, y1, x2, y2 = det.bbox
                    rec["bbox"] = (float(x1), float(y1), float(x2), float(y2))
                    rec["image_width"] = int(image.width)
                    # Pinhole pose estimate (dog frame: +x forward, +y left).
                    cx_pix = (x1 + x2) / 2.0
                    bbox_w = max(1.0, x2 - x1)
                    distance = (self._person_real_width * self._fx) / bbox_w
                    x_norm = (cx_pix - self._cx) / self._fx
                    angle = math.atan(x_norm)  # right of optical center = positive
                    rec["pos_x"] = round(distance * math.cos(angle), 2)
                    rec["pos_y"] = round(-distance * math.sin(angle), 2)
                except Exception:
                    pass
                # Refresh thumbnail (latest crop is usually the cleanest).
                thumb = self._make_thumb(det)
                if thumb:
                    rec["thumbnail"] = thumb
                seen_lids.add(lid)

        # Mark people not in this frame as no longer visible.
        with self._lock:
            for lid, rec in self._people.items():
                if lid not in seen_lids and rec["visible"]:
                    rec["visible"] = False

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            now = time.time()
            all_people = sorted(
                (
                    {
                        "long_term_id": p["long_term_id"],
                        "name": p["name"],
                        "count": p["count"],
                        "thumbnail": p["thumbnail"],
                        "pos_x": p["pos_x"],
                        "pos_y": p["pos_y"],
                        "visible": p["visible"],
                        "last_seen_ago": round(now - p["last_seen"], 1) if p["last_seen"] else None,
                    }
                    for p in self._people.values()
                ),
                key=lambda x: (-x["count"], x["long_term_id"]),
            )
            in_view = []
            for p in self._people.values():
                if p["last_seen"] > 0 and (now - p["last_seen"]) <= self.recent_window:
                    in_view.append(
                        {
                            "track_id": p["track_id"],
                            "long_term_id": p["long_term_id"],
                            "name": p["name"],
                            "secs_ago": round(now - p["last_seen"], 1),
                            "visible": p["visible"],
                            "thumbnail": p["thumbnail"],
                            "pos_x": p["pos_x"],
                            "pos_y": p["pos_y"],
                        }
                    )
            in_view.sort(key=lambda x: x["secs_ago"])
            return {"all": all_people, "in_view": in_view, "total": len(self._people)}

    def get_bbox(self, lid: int) -> tuple[tuple[float, float, float, float], int] | None:
        """Latest bbox for a long_term_id IF that person was visible recently.

        Returns (bbox xyxy, image_width) or None.
        """
        with self._lock:
            p = self._people.get(lid)
            if p is None or not p["visible"] or p["bbox"] is None:
                return None
            return (p["bbox"], p["image_width"])


# MockPeopleTracker doesn't have real bboxes; FollowController will see
# get_bbox()=None in mock and just no-op.
def _mock_get_bbox(self, lid: int) -> None:  # type: ignore[no-untyped-def]
    return None
MockPeopleTracker.get_bbox = _mock_get_bbox  # type: ignore[assignment]


people: Any = None  # set in main() based on --mock and --no-people


# --- "Follow this person" selection ----------------------------------------
# The UI lets the user pick a long_term_id from the people list as the follow
# target. For now this state is read-only on the dog side; a future control
# loop will read get_target() and steer the dog toward that person's bbox.
_tracked_lid: int | None = None
_following: bool = False
_tracked_lock = threading.Lock()


def select_target(lid: int | None) -> None:
    global _tracked_lid, _following
    with _tracked_lock:
        _tracked_lid = lid
        if lid is None:
            _following = False  # clearing the target also stops follow


def get_target() -> int | None:
    with _tracked_lock:
        return _tracked_lid


def set_following(b: bool) -> None:
    global _following
    with _tracked_lock:
        _following = bool(b)


def get_following() -> bool:
    with _tracked_lock:
        return _following


class FollowController:
    """Visual-servo follow loop. Reads selected long_term_id + tracker's latest
    bbox, drives the dog at ~10 Hz via VisualServoing2D. Safe by default: only
    sends Twist commands when `following=True` AND target has a fresh bbox.
    """

    def __init__(self, brain: "RobotBrain", tracker: Any) -> None:
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3
        from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D
        from dimos.robot.unitree.go2.connection import _camera_info_static

        self._brain = brain
        self._tracker = tracker
        self._Twist = Twist
        self._Vector3 = Vector3
        # Tuned defaults inside VisualServoing2D: max 0.5 m/s, 0.8 rad/s,
        # target distance 1.5m, min distance 0.8m. See visual_servoing_2d.py.
        self._vs = VisualServoing2D(_camera_info_static())
        self._lost_max = 20         # ~2s at 10Hz before sending stop
        self._period = 0.1
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()
        print("[follow] controller running (10 Hz, idle until FOLLOW)")

    def shutdown(self) -> None:
        self._stop.set()

    def _zero(self) -> Any:
        return self._Twist(
            linear=self._Vector3(0.0, 0.0, 0.0),
            angular=self._Vector3(0.0, 0.0, 0.0),
        )

    def _loop(self) -> None:
        lost = 0
        sent_stop = False  # avoid spamming zero twists every tick
        while not self._stop.is_set():
            try:
                if get_following() and get_target() is not None:
                    bbox_info = self._tracker.get_bbox(get_target())
                    if bbox_info is None:
                        lost += 1
                        if lost >= self._lost_max and not sent_stop:
                            print("[follow] target lost — sending zero twist")
                            self._brain.conn.move(self._zero())
                            sent_stop = True
                    else:
                        bbox, w = bbox_info
                        twist = self._vs.compute_twist(bbox, w)
                        self._brain.conn.move(twist)
                        lost = 0
                        sent_stop = False
                else:
                    # Not following: ensure dog is stopped once after each
                    # follow→idle transition, then go quiet.
                    if not sent_stop:
                        try:
                            self._brain.conn.move(self._zero())
                        except Exception:
                            pass
                        sent_stop = True
                    lost = 0
            except Exception as e:  # noqa: BLE001
                print(f"[follow] tick error: {e}")
            time.sleep(self._period)


follow_controller: FollowController | None = None  # set in main() if real dog


# --- E-STOP: panic stop the dog regardless of state ------------------------
def panic_stop(brain: Any) -> dict[str, Any]:
    """Hard stop:
      1. Cancel any follow / target selection.
      2. Spam zero-twist (3x) — joystick auto-timeout would also do this but
         we want to be loud.
      3. Send liedown so the dog actively lowers itself.
    Safe to call from any thread, any time.
    """
    select_target(None)
    set_following(False)
    out = {"following": False, "tracked_lid": None, "moves_sent": 0, "liedown": False}
    if brain is None or not hasattr(brain, "conn"):
        out["mock"] = True
        return out
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    zero = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
    for _ in range(3):
        try:
            brain.conn.move(zero)
            out["moves_sent"] += 1
        except Exception as e:  # noqa: BLE001
            print(f"[halt] zero-twist failed: {e}")
        time.sleep(0.05)
    try:
        brain.conn.liedown()
        out["liedown"] = True
    except Exception as e:  # noqa: BLE001
        print(f"[halt] liedown failed: {e}")
    print(f"[halt] PANIC STOP executed: {out}")
    return out


class MockBrain:
    """Stand-in for RobotBrain — no dog, no WebRTC. For previewing the UI.

    Same method surface as RobotBrain so the API routes work unchanged.
    """

    def __init__(self, step_distance: float = 0.3, **_: Any):
        self.step_distance = step_distance
        print("[brain] MOCK mode — no robot connection")

    def stand_up(self) -> str:
        time.sleep(0.3)
        return "standing (mock)"

    def sit_down(self) -> str:
        time.sleep(0.3)
        return "sitting (mock)"

    def lie_down(self) -> str:
        time.sleep(0.3)
        return "lying down (mock)"

    def wave(self) -> str:
        time.sleep(0.3)
        return "waved (mock)"

    def step(self, key: str, fast: bool = False) -> str:
        valid = {"W", "S", "Q", "E", "A", "D",
                 "forward", "back", "left", "right"}
        k = key.upper() if len(key) == 1 else key
        if k not in valid:
            raise ValueError(f"unknown step key {key!r}")
        time.sleep(0.15)
        return f"stepped {k}{' (fast)' if fast else ''} (mock)"

    def stop_move(self) -> str:
        return "stopped (mock)"

    def march(self, reps: int) -> str:
        time.sleep(0.5)
        return f"marched {reps} reps in place (mock)"

    def observe(self) -> dict[str, Any]:
        time.sleep(0.4)
        # Fake two detections: one near, one far.
        return {
            "count": 2,
            "near_count": 1,
            "people": [
                {"h_ratio": 0.71, "conf": 0.92, "near": True},
                {"h_ratio": 0.19, "conf": 0.63, "near": False},
            ],
        }


class RobotBrain:
    """Owns the single WebRTC connection and serializes commands to the dog."""

    def __init__(
        self,
        ip: str,
        step_distance: float,
        step_speed: float,
        close_ratio: float,
        mode: str = "normal",
        ap: bool = False,
        angular_speed: float = 0.5,  # rad/s for A/D turning (~28°/tap at 1s)
    ):
        self.ip = ip
        self.step_distance = step_distance
        self.step_speed = step_speed
        self.angular_speed = angular_speed
        self.close_ratio = close_ratio
        self._lock = threading.Lock()  # one command to the dog at a time
        self._yolo: Any = None
        method = WebRTCConnectionMethod.LocalAP if ap else WebRTCConnectionMethod.LocalSTA
        print(f"[brain] connecting to {ip} (motion mode: {mode}, link: {method.name}) ...")
        # mode="normal" enables the classic sport commands (StandUp/BalanceStand/
        # joystick move). The library default "ai" mode ignores them.
        # ap=True (LocalAP) is for when you're on the dog's own hotspot (192.168.12.1).
        self.conn = UnitreeWebRTCConnection(ip=ip, mode=mode, connection_method=method)
        time.sleep(2)  # let the MOTION_SWITCHER take effect before commanding
        # Bring the dog to a known, command-ready state.
        print("[brain] standup ...")
        self.conn.standup()
        time.sleep(3)
        print("[brain] balance_stand ...")
        self.conn.balance_stand()
        time.sleep(1)
        # FreeWalk = locomotion gait; without it, joystick velocity often only
        # makes the dog lean in place instead of actually walking.
        print("[brain] free_walk (enable locomotion) ...")
        self.conn.free_walk()
        time.sleep(1)
        print("[brain] connected and ready (standing, walk-enabled)")

    # --- posture ---------------------------------------------------------
    def stand_up(self) -> str:
        with self._lock:
            self.conn.standup()
            time.sleep(2)
            self.conn.balance_stand()
        print("[cmd] stand_up")
        return "standing"

    def sit_down(self) -> str:
        with self._lock:
            self.conn.publish_request(SPORT_TOPIC, {"api_id": SPORT_CMD["Sit"]})
        return "sitting"

    def lie_down(self) -> str:
        with self._lock:
            self.conn.liedown()  # StandDown — dog lowers all the way to the ground (prone)
        print("[cmd] lie_down")
        return "lying down"

    def wave(self) -> str:
        with self._lock:
            self.conn.publish_request(SPORT_TOPIC, {"api_id": SPORT_CMD["Hello"]})
            time.sleep(2)
            self.conn.balance_stand()
        print("[cmd] wave")
        return "waved"

    def _ensure_walk(self) -> None:
        """Re-assert FreeWalk locomotion gait so joystick velocity actually walks."""
        self.conn.free_walk()
        time.sleep(0.15)

    # --- locomotion ------------------------------------------------------
    def _pulse(self, x: float, y: float, wz: float, duration: float) -> None:
        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, wz))
        self.conn.move(twist, duration=duration)
        self.conn.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

    # Official mapping (mirrors dimos/robot/unitree/keyboard_teleop.py):
    #   W/S = forward/back   (linear.x)
    #   Q/E = strafe L/R     (linear.y)
    #   A/D = turn  L/R      (angular.z)
    # Old direction names kept as aliases for backward compatibility.
    _KEY_TO_VEC: dict[str, tuple[float, float, float]] = {}  # filled in __init__

    def _build_key_map(self) -> None:
        s = self.step_speed
        w = self.angular_speed
        self._KEY_TO_VEC = {
            "W": (s, 0.0, 0.0),
            "S": (-s, 0.0, 0.0),
            "Q": (0.0, s, 0.0),
            "E": (0.0, -s, 0.0),
            "A": (0.0, 0.0, w),
            "D": (0.0, 0.0, -w),
            # legacy aliases (older UI):
            "forward": (s, 0.0, 0.0),
            "back": (-s, 0.0, 0.0),
            "left": (0.0, s, 0.0),
            "right": (0.0, -s, 0.0),
        }

    def step(self, key: str, fast: bool = False) -> str:
        if not self._KEY_TO_VEC:
            self._build_key_map()
        k = key.upper() if len(key) == 1 else key
        vec = self._KEY_TO_VEC.get(k)
        if vec is None:
            raise ValueError(f"unknown step key {key!r}; expected W/S/Q/E/A/D")
        vx, vy, wz = vec
        mult = 2.0 if fast else 1.0
        vx *= mult; vy *= mult; wz *= mult
        dur = self.step_distance / max(self.step_speed, 0.05)
        with self._lock:
            self._ensure_walk()
            self._pulse(vx, vy, wz, dur)
        tag = "⚡" if fast else ""
        print(f"[cmd] step {tag}{k} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
        return f"stepped {k}{' (fast)' if fast else ''}"

    def stop_move(self) -> str:
        """Single zero-twist — dog stops moving but stays standing (no liedown).
        This is the equivalent of releasing the WASD keys / the official STOP."""
        with self._lock:
            self._pulse(0.0, 0.0, 0.0, 0.0)
        print("[cmd] stop_move (zero twist)")
        return "stopped"

    def march(self, reps: int) -> str:
        """Approximate marching in place: short forward+back pulse pairs (net ~0)."""
        pulse_dur = 0.18
        with self._lock:
            self._ensure_walk()
            for _ in range(reps):
                self._pulse(self.step_speed, 0.0, 0.0, pulse_dur)
                self._pulse(-self.step_speed, 0.0, 0.0, pulse_dur)
            self.conn.balance_stand()
        print(f"[cmd] march {reps} reps")
        return f"marched {reps} reps in place"

    # --- perception ------------------------------------------------------
    def _ensure_yolo(self) -> Any:
        if self._yolo is None:
            from ultralytics import YOLO  # type: ignore[attr-defined]

            print("[brain] loading YOLO11n-pose ...")
            self._yolo = YOLO(get_data("models_yolo") / "yolo11n-pose.pt")
        return self._yolo

    def _grab_frame(self, timeout: float = 10.0):  # type: ignore[no-untyped-def]
        holder: list = []
        got = threading.Event()

        def on_next(frame) -> None:  # type: ignore[no-untyped-def]
            if not holder:
                holder.append(frame)
                got.set()

        sub = self.conn.video_stream().subscribe(on_next=on_next)
        try:
            if not got.wait(timeout=timeout):
                raise TimeoutError(f"no camera frame within {timeout:.1f}s")
        finally:
            sub.dispose()
        return holder[0]

    def observe(self) -> dict[str, Any]:
        model = self._ensure_yolo()
        with self._lock:
            frame = self._grab_frame()
        img = frame.to_opencv()
        frame_h = img.shape[0]
        results = model.predict(img, conf=0.5, verbose=False)
        people = []
        if results and results[0].boxes is not None:
            r = results[0]
            for box, cls_id, c in zip(r.boxes.xyxy, r.boxes.cls, r.boxes.conf, strict=False):
                if int(cls_id.item()) != 0:  # COCO class 0 = person
                    continue
                x1, y1, x2, y2 = (float(v) for v in box.tolist())
                h_ratio = (y2 - y1) / frame_h
                people.append(
                    {
                        "h_ratio": round(h_ratio, 3),
                        "conf": round(float(c.item()), 3),
                        "near": h_ratio >= self.close_ratio,
                    }
                )
        near = sum(1 for p in people if p["near"])
        return {"count": len(people), "near_count": near, "people": people}


# ---------------------------------------------------------------------------
brain: RobotBrain | None = None
app = FastAPI(title="Go2 Phone Control")


class MarchReq(BaseModel):
    reps: int = 10


class StepReq(BaseModel):
    key: str | None = None        # new: W/S/Q/E/A/D
    direction: str | None = None  # legacy: forward/back/left/right
    fast: bool = False


@app.get("/")
def index() -> FileResponse:
    return FileResponse(HERE / "index.html")


@app.get("/people")
def people_page() -> FileResponse:
    return FileResponse(HERE / "people.html")


@app.get("/api/people")
def api_people() -> JSONResponse:
    if people is None:
        return JSONResponse(
            {
                "all": [], "in_view": [], "total": 0, "disabled": True,
                "tracked_lid": None, "following": False,
            }
        )
    state = people.get_state()
    state["tracked_lid"] = get_target()
    state["following"] = get_following()
    return JSONResponse(state)


class TrackSelectReq(BaseModel):
    long_term_id: int


@app.post("/api/track/select")
def api_track_select(req: TrackSelectReq) -> JSONResponse:
    select_target(req.long_term_id)
    return JSONResponse({"ok": True, "tracked_lid": req.long_term_id, "following": False})


@app.post("/api/track/clear")
def api_track_clear() -> JSONResponse:
    select_target(None)
    return JSONResponse({"ok": True, "tracked_lid": None, "following": False})


@app.post("/api/follow/start")
def api_follow_start() -> JSONResponse:
    if get_target() is None:
        return JSONResponse(
            {"ok": False, "error": "no target selected"}, status_code=400
        )
    set_following(True)
    return JSONResponse({"ok": True, "following": True, "tracked_lid": get_target()})


@app.post("/api/follow/stop")
def api_follow_stop() -> JSONResponse:
    set_following(False)
    return JSONResponse({"ok": True, "following": False, "tracked_lid": get_target()})


@app.post("/api/halt")
def api_halt() -> JSONResponse:
    """E-STOP: cancel target, stop following, zero-twist x3, liedown.
    Reachable any time (no auth, no preconditions)."""
    result = panic_stop(brain)
    return JSONResponse({"ok": True, **result})


def _ok(result: Any) -> JSONResponse:
    return JSONResponse({"ok": True, "result": result})


def _err(msg: str) -> JSONResponse:
    return JSONResponse({"ok": False, "error": msg}, status_code=500)


@app.post("/api/stand_up")
def api_stand_up() -> JSONResponse:
    try:
        return _ok(brain.stand_up())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/sit_down")
def api_sit_down() -> JSONResponse:
    try:
        return _ok(brain.sit_down())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/lie_down")
def api_lie_down() -> JSONResponse:
    try:
        return _ok(brain.lie_down())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/wave")
def api_wave() -> JSONResponse:
    try:
        return _ok(brain.wave())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/march")
def api_march(req: MarchReq) -> JSONResponse:
    try:
        return _ok(brain.march(req.reps))  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/step")
def api_step(req: StepReq) -> JSONResponse:
    k = req.key or req.direction
    if not k:
        return _err("missing 'key' (W/S/Q/E/A/D) or 'direction'")
    try:
        return _ok(brain.step(k, fast=req.fast))  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/stop_move")
def api_stop_move() -> JSONResponse:
    """Soft stop — single zero twist. Dog stops moving but stays standing.
    (Use /api/halt for hard panic: clear follow + zero x3 + liedown.)"""
    try:
        return _ok(brain.stop_move())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


@app.post("/api/observe")
def api_observe() -> JSONResponse:
    try:
        return _ok(brain.observe())  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return _err(str(e))


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 phone-control server")
    parser.add_argument("--ip", help="Robot IP, e.g. 192.168.12.1 (omit with --mock)")
    parser.add_argument("--port", type=int, default=8800, help="HTTP port (default 8800)")
    parser.add_argument("--step-distance", type=float, default=0.3, dest="step_distance")
    parser.add_argument("--step-speed", type=float, default=0.3, dest="step_speed")
    parser.add_argument("--close-ratio", type=float, default=0.5, dest="close_ratio")
    parser.add_argument(
        "--mode",
        default="normal",
        help="Go2 motion mode: 'normal' (classic sport cmds) or 'ai' (default normal)",
    )
    parser.add_argument(
        "--ap",
        action="store_true",
        help="Use LocalAP link (you're on the dog's own hotspot at 192.168.12.1)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="UI preview mode: no robot connection, fake responses",
    )
    parser.add_argument(
        "--no-people",
        action="store_true",
        dest="no_people",
        help="Disable the people tracker (saves CPU; /api/people returns empty)",
    )
    parser.add_argument(
        "--mock-people",
        action="store_true",
        dest="mock_people",
        help="Force the MOCK people tracker even when connected to the real dog",
    )
    parser.add_argument(
        "--osnet",
        default="osnet_x0_5",
        help="OSNet variant for Re-ID: x0_25/x0_5/x0_75/x1_0 (default x0_5)",
    )
    parser.add_argument(
        "--no-follow",
        action="store_true",
        dest="no_follow",
        help="Disable the follow controller (no visual-servo loop runs)",
    )
    args = parser.parse_args()

    global brain, people, follow_controller
    if args.mock:
        brain = MockBrain(step_distance=args.step_distance)  # type: ignore[assignment]
    else:
        if not args.ip:
            parser.error("--ip is required unless --mock is set")
        brain = RobotBrain(
            ip=args.ip,
            step_distance=args.step_distance,
            step_speed=args.step_speed,
            close_ratio=args.close_ratio,
            mode=args.mode,
            ap=args.ap,
        )

    # People tracker: mock by default in --mock; real (YOLO+OSNet) when on a dog
    # unless --mock-people overrides. --no-people disables the feature.
    if args.no_people:
        people = None
        print("[people] disabled (--no-people)")
    elif args.mock or args.mock_people:
        people = MockPeopleTracker()  # type: ignore[assignment]
    else:
        try:
            people = RealPeopleTracker(brain.conn, osnet_variant=args.osnet)  # type: ignore[assignment]
        except Exception as e:  # noqa: BLE001
            print(f"[people] real tracker failed to start: {e}\n[people] falling back to MOCK")
            people = MockPeopleTracker()  # type: ignore[assignment]

    # Follow controller (only meaningful on a real dog). Always available so the
    # API exists for testing; in mock mode it just no-ops since get_bbox returns
    # None and brain has no `.conn`.
    if args.no_follow or args.mock:
        follow_controller = None
        if args.no_follow:
            print("[follow] disabled (--no-follow)")
    else:
        try:
            follow_controller = FollowController(brain, people)
        except Exception as e:  # noqa: BLE001
            print(f"[follow] failed to start: {e}")
            follow_controller = None

    # 0.0.0.0 so the phone on the same Wi-Fi can reach it.
    print(f"\n[server] open http://<this-mac-LAN-IP>:{args.port}/ on your phone\n")
    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="info")


if __name__ == "__main__":
    main()
