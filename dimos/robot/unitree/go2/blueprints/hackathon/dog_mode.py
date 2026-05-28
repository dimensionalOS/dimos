"""DogModeModule — autonomous dog behavior for Unitree Go2.

State machine:
  WANDERING  — explore environment, periodic sniff animation
  ALERT      — person detected, face them, bark randomly based on threat level
  CLOSE      — person within growl range, track and growl, stop moving

Threat levels (assigned per track_id on first detection, random):
  0.0–0.4  curious:    occasional soft bark, no growl
  0.4–0.7  cautious:   bark after short delay, growl at 2.0m
  0.7–1.0  aggressive: bark immediately, growl at 2.5m
"""

from __future__ import annotations

import random
import subprocess
import time
from pathlib import Path
from threading import Event, RLock, Thread
from typing import Any

from reactivex.disposable import Disposable
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool
from dimos.robot.unitree.go2.blueprints.hackathon.frame_writer import write_annotated_frame, write_state
from dimos.robot.unitree.go2.blueprints.hackathon.perception_loop import get_shared_detections, set_active_mode, shared_detections_fresh

logger = setup_logger()

SOUNDS_DIR = Path(__file__).parent / "sounds"
SOUND_FILES = {
    "bark":    SOUNDS_DIR / "minecraft-dog-bark.mp3",
    "deepbark": SOUNDS_DIR / "deepbark.mp3",
    "growl":   SOUNDS_DIR / "dog-growling-and-barking.mp3",
    "perry":   SOUNDS_DIR / "perry-the-platypuss-growl.mp3",
}

# Threat thresholds
_AGGRESSIVE_THRESHOLD = 0.7
_CAUTIOUS_THRESHOLD   = 0.4

# Distance thresholds (meters, estimated from bbox width)
_CLOSE_AGGRESSIVE = 2.5   # growl distance for aggressive
_CLOSE_CAUTIOUS   = 2.0   # growl distance for cautious
_FACE_RANGE       = 5.0   # start facing at this distance

# Timing
_SNIFF_INTERVAL_MIN = 20.0   # minimum seconds between sniffs
_SNIFF_INTERVAL_MAX = 45.0
_BARK_COOLDOWN      = 4.0    # minimum seconds between barks
_LOOP_HZ            = 10.0   # main loop frequency


def _play_sound(name: str) -> None:
    path = SOUND_FILES.get(name)
    if path and path.exists():
        try:
            import sounddevice as sd
            import soundfile as sf
            data, sr = sf.read(str(path))
            sd.play(data.astype("float32"), sr)
            sd.wait()
        except Exception as e:
            logger.warning(f"Sound playback failed: {e}")


class Config(ModuleConfig):
    camera_info: CameraInfo


class DogModeModule(Module):
    """Autonomous dog behavior module."""

    config: Config

    color_image: In[Image]
    cmd_vel: Out[Twist]
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]

    _connection: GO2ConnectionSpec  # injected by dimOS — used for sport commands

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._detector = YoloPersonDetector()  # fallback only
        self._servo = VisualServoing2D(self.config.camera_info)

        self._latest_image: Image | None = None
        self._thread: Thread | None = None
        self._should_stop = Event()
        self._lock = RLock()

        # Threat memory: track_id → threat float [0,1]
        self._threats: dict[int, float] = {}
        self._state = "IDLE"

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))

    @rpc
    def stop(self) -> None:
        self._should_stop.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        self._detector.stop()
        super().stop()

    def _on_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    @skill
    def start_dog_mode(self) -> str:
        """Start autonomous dog mode.

        The robot will:
        - Roam and explore the environment
        - Periodically sniff the ground (StandDown animation)
        - Detect people and assess threat level (random, per person)
        - Bark at people based on threat level
        - Growl and track anyone who gets close
        - Face people when they move toward it
        """
        if self._thread and self._thread.is_alive():
            return "Dog mode is already running."

        self._should_stop.clear()
        self._threats.clear()
        self._state = "WANDERING"
        set_active_mode("dog")

        self._thread = Thread(target=self._dog_loop, daemon=True, name="DogMode")
        self._thread.start()

        return "Woof. Dog mode activated. I'll explore, sniff around, and keep an eye on people."

    @skill
    def stop_dog_mode(self) -> str:
        """Stop dog mode and return the robot to idle."""
        self._should_stop.set()
        set_active_mode(None)
        self._send_stop_explore()
        self.cmd_vel.publish(Twist.zero())
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        self._state = "IDLE"
        return "Dog mode stopped."

    # ──────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────

    def _sport(self, name: str) -> None:
        try:
            cmd_id = SPORT_CMD[name]
            self._connection.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": cmd_id})
        except Exception as e:
            logger.warning(f"Sport command '{name}' failed: {e}")

    def _send_explore(self) -> None:
        msg = Bool(); msg.data = True
        self.explore_cmd.publish(msg)

    def _send_stop_explore(self) -> None:
        msg = Bool(); msg.data = True
        self.stop_explore_cmd.publish(msg)

    def _threat_for(self, track_id: int) -> float:
        if track_id not in self._threats:
            self._threats[track_id] = random.random()
            logger.info(f"DogMode: new person track_id={track_id} threat={self._threats[track_id]:.2f}")
        return self._threats[track_id]

    def _growl_distance(self, threat: float) -> float:
        if threat >= _AGGRESSIVE_THRESHOLD:
            return _CLOSE_AGGRESSIVE
        if threat >= _CAUTIOUS_THRESHOLD:
            return _CLOSE_CAUTIOUS
        return 0.0  # curious — no growl

    def _face_person(self, bbox: tuple, image_width: int) -> None:
        """Publish angular-only twist to face person (no linear movement)."""
        x1, _, x2, _ = bbox
        cx = (x1 + x2) / 2.0
        fx = self.config.camera_info.K[0]
        optical_cx = self.config.camera_info.K[2]
        x_norm = (cx - optical_cx) / fx
        angular_z = float(-x_norm * 1.2)
        angular_z = max(-0.6, min(0.6, angular_z))
        if abs(angular_z) > 0.05:
            self.cmd_vel.publish(
                Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, angular_z))
            )
        else:
            self.cmd_vel.publish(Twist.zero())

    def _do_sniff(self) -> None:
        """StandDown → pause → RecoveryStand in a background thread."""
        def _sniff() -> None:
            self.cmd_vel.publish(Twist.zero())
            self._sport("StandDown")
            time.sleep(2.0)
            self._sport("RecoveryStand")
            time.sleep(1.0)
        Thread(target=_sniff, daemon=True).start()

    # ──────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────

    def _dog_loop(self) -> None:
        last_sniff = time.monotonic() - random.uniform(0, _SNIFF_INTERVAL_MIN)
        last_bark  = 0.0
        next_sniff_in = random.uniform(_SNIFF_INTERVAL_MIN, _SNIFF_INTERVAL_MAX)
        sniffing   = False
        frame_count = 0

        # Start exploring
        self._send_explore()

        period = 1.0 / _LOOP_HZ
        next_t = time.monotonic()

        while not self._should_stop.is_set():
            next_t += period
            now = time.monotonic()

            with self._lock:
                image = self._latest_image

            if image is None:
                time.sleep(period)
                continue

            frame_count += 1

            # ── Detect people — use shared YOLO pass when fresh ──
            if shared_detections_fresh():
                all_dets, _ = get_shared_detections()
                persons = [d for d in all_dets if getattr(d, "name", "") == "person"]
            else:
                dets = self._detector.process_image(image)
                persons = [d for d in dets.detections if d.is_valid()]

            if not persons:
                # Nobody around — wander
                if self._state != "WANDERING":
                    self._state = "WANDERING"
                    self._send_explore()

                # Sniff periodically
                if not sniffing and (now - last_sniff) >= next_sniff_in:
                    sniffing = True
                    last_sniff = now
                    next_sniff_in = random.uniform(_SNIFF_INTERVAL_MIN, _SNIFF_INTERVAL_MAX)
                    self._do_sniff()
                    time.sleep(3.2)  # wait for animation to complete
                    sniffing = False

            else:
                # ── Person(s) detected ──
                # Pick the most prominent (nearest)
                target = max(persons, key=lambda d: d.bbox_2d_volume())
                tid    = target.track_id
                threat = self._threat_for(tid)
                dist   = self._servo._estimate_distance(target.bbox)
                growl_dist = self._growl_distance(threat)

                # Stop exploration while attending to person
                if self._state == "WANDERING":
                    self._send_stop_explore()
                    self._state = "ALERT"

                is_close = dist is not None and growl_dist > 0 and dist <= growl_dist

                if is_close:
                    # ── CLOSE: growl and face, stop moving ──
                    if self._state != "CLOSE":
                        self._state = "CLOSE"
                        Thread(target=_play_sound, args=("growl",), daemon=True).start()
                    self._face_person(target.bbox, image.width)

                else:
                    # ── ALERT: face person, decide whether to bark ──
                    self._state = "ALERT"

                    # Face if within range
                    if dist is None or dist <= _FACE_RANGE:
                        self._face_person(target.bbox, image.width)
                    else:
                        self.cmd_vel.publish(Twist.zero())

                    # Bark logic by threat level
                    if now - last_bark >= _BARK_COOLDOWN:
                        if threat >= _AGGRESSIVE_THRESHOLD:
                            # Aggressive: always bark on sight
                            last_bark = now
                            sound = "deepbark" if random.random() < 0.5 else "bark"
                            Thread(target=_play_sound, args=(sound,), daemon=True).start()
                        elif threat >= _CAUTIOUS_THRESHOLD:
                            # Cautious: 40% chance each cycle (at 10Hz → ~0.5Hz effective)
                            if random.random() < 0.04:
                                last_bark = now
                                Thread(target=_play_sound, args=("bark",), daemon=True).start()
                        else:
                            # Curious: occasional soft bark (10% per cycle)
                            if random.random() < 0.01:
                                last_bark = now
                                Thread(target=_play_sound, args=("bark",), daemon=True).start()

            # ── Annotate frame every 3 ticks for dashboard (~3fps) ──
            if frame_count % 3 == 0:
                try:
                    bgr = image.to_opencv()
                    det_list = []
                    for p in persons:
                        tid = p.track_id
                        det_list.append({
                            "bbox": p.bbox,
                            "track_id": tid,
                            "threat": self._threats.get(tid),
                        })
                    write_annotated_frame(bgr, det_list, overlay_text=f"DOG MODE\n{self._state}")
                    write_state({"mode": "dog", "state": self._state, "persons": len(persons),
                                 "threats": {str(k): round(v, 2) for k, v in self._threats.items()}})
                except Exception:
                    pass

            # Pace loop
            sleep_dur = next_t - time.monotonic()
            if sleep_dur > 0:
                time.sleep(sleep_dur)

        self.cmd_vel.publish(Twist.zero())
        self._send_stop_explore()
