#!/usr/bin/env python3
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

"""Go2 visual-event notifier demo.

This example turns stable red, yellow, or green visual states into phone alerts.
It can read from a local webcam for rehearsal or from a real Unitree Go2 camera
through DimOS's direct WebRTC connection.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Literal

import cv2
import numpy as np
import requests

from dimos.agents.skills.browser_notification_skill import BrowserNotificationSkill
from dimos.core.global_config import global_config
from dimos.perception.visual_events.traffic_light import (
    StableStateDebouncer,
    TrafficLightColorDetector,
    TrafficLightDetection,
    TrafficLightState,
)

SourceName = Literal["go2", "webcam"]


EVENT_MESSAGES: dict[TrafficLightState, dict[str, str]] = {
    "red": {
        "title": "Stop",
        "message": "Red visual signal detected.",
        "browser_urgency": "high",
        "ntfy_priority": "urgent",
        "ntfy_tags": "rotating_light,warning",
    },
    "yellow": {
        "title": "Caution",
        "message": "Yellow visual signal detected.",
        "browser_urgency": "high",
        "ntfy_priority": "high",
        "ntfy_tags": "warning",
    },
    "green": {
        "title": "Go",
        "message": "Green visual signal detected.",
        "browser_urgency": "normal",
        "ntfy_priority": "default",
        "ntfy_tags": "green_circle,heavy_check_mark",
    },
    "unknown": {
        "title": "Unknown",
        "message": "No stable visual signal detected.",
        "browser_urgency": "normal",
        "ntfy_priority": "default",
        "ntfy_tags": "question",
    },
}


def parse_roi(value: str | None) -> tuple[float, float, float, float] | None:
    """Parse a normalized ROI string in x1,y1,x2,y2 form."""

    if not value:
        return None

    parts = [float(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("--roi must be x1,y1,x2,y2")

    x1, y1, x2, y2 = parts
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise argparse.ArgumentTypeError("--roi values must satisfy 0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1")
    return x1, y1, x2, y2


class BrowserAlertPublisher:
    """Manage the local DimOS browser notification skill for this example."""

    def __init__(self, enabled: bool, listen_host: str, port: int) -> None:
        self.enabled = enabled
        self.listen_host = listen_host
        self.port = port
        self._skill: BrowserNotificationSkill | None = None

    @property
    def url(self) -> str:
        return f"https://{self.listen_host}:{self.port}/notify"

    def start(self) -> None:
        if not self.enabled:
            return

        global_config.listen_host = self.listen_host
        self._skill = BrowserNotificationSkill(server_port=self.port)
        self._skill.start()
        print(f"[BROWSER] Open {self.url} on your phone and tap Enable alerts.", flush=True)

    def stop(self) -> None:
        if self._skill is not None:
            self._skill.stop()
            self._skill = None

    def publish(self, title: str, message: str, urgency: str) -> str:
        if self._skill is None:
            return "browser alerts disabled"
        return self._skill.notify_user(
            title=title,
            message=message,
            urgency=urgency,
            sound=True,
            vibrate=True,
        )


def publish_ntfy(
    *,
    server: str,
    topic: str,
    title: str,
    message: str,
    priority: str,
    tags: str,
    timeout_s: float,
) -> str:
    """Publish a notification through ntfy."""

    if not topic:
        return "ntfy skipped: no topic configured"

    response = requests.post(
        f"{server.rstrip('/')}/{topic}",
        data=message.encode("utf-8"),
        headers={
            "Title": title,
            "Priority": priority,
            "Tags": tags,
            "Content-Type": "text/plain; charset=utf-8",
        },
        timeout=timeout_s,
    )
    response.raise_for_status()
    return f"ntfy status={response.status_code}"


def append_jsonl(path: str | None, record: dict[str, Any]) -> None:
    """Append one JSON record to a log file if logging is enabled."""

    if not path:
        return

    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as log_file:
        log_file.write(json.dumps(record, ensure_ascii=True) + "\n")


def handle_event(
    args: argparse.Namespace,
    detection: TrafficLightDetection,
    browser_alerts: BrowserAlertPublisher,
) -> None:
    """Send all configured outputs for one stable visual event."""

    event = EVENT_MESSAGES[detection.state]
    record: dict[str, Any] = {
        "ts": time.time(),
        "event": detection.state,
        "confidence": detection.confidence,
        "area_ratio": detection.area_ratio,
        "red_score": detection.red_score,
        "yellow_score": detection.yellow_score,
        "green_score": detection.green_score,
        "notification": event,
    }

    print(
        "\n[EVENT] "
        f"{detection.state.upper()} confidence={detection.confidence:.2f} "
        f"area={detection.area_ratio:.3f}",
        flush=True,
    )

    browser_result = browser_alerts.publish(
        title=event["title"],
        message=event["message"],
        urgency=event["browser_urgency"],
    )
    record["browser_result"] = browser_result
    print(f"[BROWSER] {browser_result}", flush=True)

    if args.ntfy_topic:
        try:
            ntfy_result = publish_ntfy(
                server=args.ntfy_server,
                topic=args.ntfy_topic,
                title=f"Go2 {event['title']}",
                message=event["message"],
                priority=event["ntfy_priority"],
                tags=event["ntfy_tags"],
                timeout_s=args.ntfy_timeout_s,
            )
            record["ntfy_result"] = ntfy_result
            print(f"[NTFY] {ntfy_result}", flush=True)
        except requests.RequestException as exc:
            record["ntfy_error"] = repr(exc)
            print(f"[NTFY_ERROR] {exc!r}", flush=True)

    append_jsonl(args.log_file, record)


def process_frame(
    args: argparse.Namespace,
    detector: TrafficLightColorDetector,
    debouncer: StableStateDebouncer,
    browser_alerts: BrowserAlertPublisher,
    frame_rgb: np.ndarray,
) -> bool:
    """Process one RGB frame. Returns False when the display window requests stop."""

    detection = detector.classify(frame_rgb, color_space="RGB", roi=args.roi)
    print(
        f"[FRAME] state={detection.state:<7} conf={detection.confidence:.2f} "
        f"R={detection.red_score:.3f} Y={detection.yellow_score:.3f} G={detection.green_score:.3f}",
        end="\r",
        flush=True,
    )

    event = debouncer.update(detection.state)
    if event is not None:
        handle_event(args, detection, browser_alerts)

    if args.display:
        debug = detector.draw_debug_overlay(frame_rgb, detection, color_space="RGB")
        cv2.imshow("Go2 Visual Event Notifier", debug)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return False

    return True


def run_webcam(args: argparse.Namespace, browser_alerts: BrowserAlertPublisher) -> None:
    """Run the detector against a local webcam."""

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam index {args.camera_index}")

    detector = TrafficLightColorDetector(min_area_ratio=args.min_area_ratio)
    debouncer = StableStateDebouncer(stable_frames=args.stable_frames, cooldown_s=args.cooldown_s)
    start = time.time()

    print("[INFO] Running webcam source. Press Ctrl-C or q in the display window to stop.", flush=True)
    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            if not process_frame(args, detector, debouncer, browser_alerts, frame_rgb):
                break

            if args.run_seconds and time.time() - start > args.run_seconds:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_go2(args: argparse.Namespace, browser_alerts: BrowserAlertPublisher) -> None:
    """Run the detector against a real Go2 WebRTC camera stream."""

    from dimos.robot.unitree.connection import UnitreeWebRTCConnection

    latest_frame: dict[str, np.ndarray | None] = {"frame": None}
    lock = threading.Lock()

    print(f"[INFO] Connecting to Go2 at {args.robot_ip}", flush=True)
    connection = UnitreeWebRTCConnection(args.robot_ip)

    def on_image(image: Any) -> None:
        try:
            frame_rgb = image.to_rgb().as_numpy()
        except Exception:
            frame_rgb = image.as_numpy()

        with lock:
            latest_frame["frame"] = np.array(frame_rgb, copy=True)

    subscription = connection.video_stream().subscribe(on_image)
    detector = TrafficLightColorDetector(min_area_ratio=args.min_area_ratio)
    debouncer = StableStateDebouncer(stable_frames=args.stable_frames, cooldown_s=args.cooldown_s)
    start = time.time()

    print("[INFO] Go2 video source running. Press Ctrl-C to stop.", flush=True)
    try:
        while True:
            with lock:
                frame = None if latest_frame["frame"] is None else np.array(latest_frame["frame"], copy=True)

            if frame is None:
                print("[INFO] Waiting for Go2 camera frame...", end="\r", flush=True)
                time.sleep(0.1)
                continue

            if not process_frame(args, detector, debouncer, browser_alerts, frame):
                break

            if args.run_seconds and time.time() - start > args.run_seconds:
                break

            time.sleep(1.0 / max(args.fps, 1.0))
    finally:
        print("\n[INFO] Shutting down Go2 connection.", flush=True)
        subscription.dispose()
        connection.stop()
        cv2.destroyAllWindows()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Go2 visual event notifier demo")
    parser.add_argument("--source", choices=["go2", "webcam"], default="go2")
    parser.add_argument("--robot-ip", default=os.environ.get("ROBOT_IP", "192.168.12.1"))
    parser.add_argument("--camera-index", type=int, default=0)

    parser.add_argument("--browser-alerts", action="store_true")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--browser-port", type=int, default=8450)

    parser.add_argument("--ntfy-topic", default=os.environ.get("NTFY_TOPIC", ""))
    parser.add_argument("--ntfy-server", default=os.environ.get("NTFY_SERVER", "https://ntfy.sh"))
    parser.add_argument("--ntfy-timeout-s", type=float, default=5.0)

    parser.add_argument("--min-area-ratio", type=float, default=0.015)
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--run-seconds", type=float, default=0.0)
    parser.add_argument("--roi", type=parse_roi, default=None, help="Optional normalized crop x1,y1,x2,y2")
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--log-file", default="go2_visual_event_log.jsonl")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    browser_alerts = BrowserAlertPublisher(
        enabled=args.browser_alerts,
        listen_host=args.listen_host,
        port=args.browser_port,
    )

    print("[INFO] Config:")
    print(json.dumps(vars(args), indent=2, default=str), flush=True)

    browser_alerts.start()
    try:
        if args.source == "webcam":
            run_webcam(args, browser_alerts)
        else:
            run_go2(args, browser_alerts)
    finally:
        browser_alerts.stop()


if __name__ == "__main__":
    main()

