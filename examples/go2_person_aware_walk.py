#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Go2: stand up, repeatedly check for a person nearby, march in place while
present, then walk forward once the path is clear.

Pipeline:
    1. WebRTC connect
    2. Stand up + balance_stand
    3. LOOP every --check-interval seconds:
         - grab one camera frame
         - run YOLO11n-pose to detect persons
         - if any person bbox occupies > --close-ratio of frame height:
             => robot waves hello on first detection, then stays in balance_stand
                ("march in place"). Continue the loop.
         - else:
             => break out of loop
       Loop exits early after --max-wait seconds as a safety bound.
    4. Walk forward --distance meters at --speed m/s
    5. Lie down

⚠️  bbox-height / frame-height is a coarse heuristic — its threshold depends on
    camera FOV, mounting height, and person height. Use --close-ratio to tune
    in your venue. Default 0.5 is a safe starting point on the Go2's front cam.

Run:
    .venv/bin/python examples/go2_person_aware_walk.py --ip 192.168.12.1
    # Tunables:
    #   --close-ratio 0.5    # bbox-height / frame-height threshold for "close"
    #   --conf 0.5           # YOLO confidence
    #   --check-interval 1.5 # seconds between detection ticks
    #   --max-wait 60        # seconds to keep marching before giving up
    #   --distance 2.0       # meters to walk forward when path is clear
    #   --speed 0.3          # forward speed m/s
"""

from __future__ import annotations

import argparse
import threading
import time

from ultralytics import YOLO  # type: ignore[attr-defined]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.utils.data import get_data


# SPORT_MOD api_id for the Hello (paw wave) action — see
# dimos/robot/unitree/unitree_skill_container.py UNITREE_WEBRTC_CONTROLS.
SPORT_TOPIC = "rt/api/sport/request"
HELLO_API_ID = 1016


def grab_first_frame(conn: UnitreeWebRTCConnection, timeout: float = 10.0):  # type: ignore[no-untyped-def]
    """Subscribe to the WebRTC video stream and return the first frame."""
    holder: list = []
    got = threading.Event()

    def on_next(frame) -> None:  # type: ignore[no-untyped-def]
        if not holder:
            holder.append(frame)
            got.set()

    subscription = conn.video_stream().subscribe(on_next=on_next)
    try:
        if not got.wait(timeout=timeout):
            raise TimeoutError(f"No camera frame within {timeout:.1f}s")
    finally:
        subscription.dispose()
    return holder[0]


def person_close(
    frame, model: YOLO, conf: float, close_ratio: float
) -> tuple[bool, list[tuple[float, float]]]:
    """Return (someone_close, list of (height_ratio, confidence)) per person detected.

    "Close" = any person bbox height / frame height >= close_ratio.
    """
    img = frame.to_opencv()  # BGR ndarray
    frame_h = img.shape[0]
    results = model.predict(img, conf=conf, verbose=False)
    if not results:
        return False, []

    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        return False, []

    # YOLO class 0 in COCO is 'person'
    detections: list[tuple[float, float]] = []
    for box, cls_id, c in zip(r.boxes.xyxy, r.boxes.cls, r.boxes.conf, strict=False):
        if int(cls_id.item()) != 0:
            continue
        x1, y1, x2, y2 = (float(v) for v in box.tolist())
        h_ratio = (y2 - y1) / frame_h
        detections.append((h_ratio, float(c.item())))

    if not detections:
        return False, []
    close = any(h >= close_ratio for h, _ in detections)
    return close, detections


def wave_hello(conn: UnitreeWebRTCConnection) -> None:
    """Trigger the Go2 Hello (paw wave) action. Animation runs ~1.5s on the dog."""
    conn.publish_request(SPORT_TOPIC, {"api_id": HELLO_API_ID})
    time.sleep(2.0)
    # Return to balance_stand so cmd_vel works again afterward.
    conn.balance_stand()


def wait_for_clear_path(
    conn: UnitreeWebRTCConnection,
    model: YOLO,
    conf: float,
    close_ratio: float,
    check_interval: float,
    max_wait: float,
) -> bool:
    """Loop until no person is within close_ratio (or max_wait elapsed).

    Returns True if path became clear, False if max_wait reached while a
    person is still close (caller should then choose what to do).
    """
    deadline = time.time() + max_wait
    waved = False
    tick = 0
    while True:
        tick += 1
        frame = grab_first_frame(conn)
        close, detections = person_close(frame, model, conf, close_ratio)

        if not detections:
            print(f"  [tick {tick}] no persons detected")
        else:
            for h_ratio, c in detections:
                tag = "CLOSE" if h_ratio >= close_ratio else "far"
                print(f"  [tick {tick}] person h/H={h_ratio:.2f} conf={c:.2f} [{tag}]")

        if not close:
            print("→ path is clear")
            return True

        if not waved:
            print("→ person within range — waving hello and marching in place")
            wave_hello(conn)
            waved = True
        else:
            # Already waved; just keep standing in place while we wait.
            conn.balance_stand()

        if time.time() >= deadline:
            print(f"→ max-wait ({max_wait:.0f}s) elapsed while person still near — aborting walk")
            return False
        time.sleep(check_interval)


def walk_forward(conn: UnitreeWebRTCConnection, distance: float, speed: float) -> None:
    duration = distance / speed
    print(f"→ path clear — walking forward {distance:.2f}m at {speed:.2f} m/s ({duration:.1f}s)")
    twist = Twist(linear=Vector3(speed, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    conn.move(twist, duration=duration)
    conn.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
    time.sleep(0.5)


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2: person-aware walk forward")
    parser.add_argument("--ip", required=True, help="Robot IP, e.g. 192.168.12.1")
    parser.add_argument("--distance", type=float, default=2.0, help="meters to walk if clear")
    parser.add_argument("--speed", type=float, default=0.3, help="forward speed m/s")
    parser.add_argument(
        "--close-ratio",
        type=float,
        default=0.5,
        dest="close_ratio",
        help="bbox-height / frame-height threshold for 'within ~1m' (default 0.5)",
    )
    parser.add_argument("--conf", type=float, default=0.5, help="YOLO confidence threshold")
    parser.add_argument(
        "--check-interval",
        type=float,
        default=1.5,
        dest="check_interval",
        help="seconds between detection ticks while marching (default 1.5)",
    )
    parser.add_argument(
        "--max-wait",
        type=float,
        default=60.0,
        dest="max_wait",
        help="give up and lie down after this many seconds of person presence (default 60)",
    )
    args = parser.parse_args()

    if args.distance <= 0 or args.speed <= 0:
        parser.error("--distance and --speed must be positive")
    if args.check_interval <= 0 or args.max_wait <= 0:
        parser.error("--check-interval and --max-wait must be positive")

    print(f"Loading YOLO11n-pose...")
    model_path = get_data("models_yolo") / "yolo11n-pose.pt"
    model = YOLO(model_path)

    print(f"Connecting to {args.ip}...")
    conn = UnitreeWebRTCConnection(ip=args.ip)

    try:
        print("Standing up...")
        conn.standup()
        time.sleep(3)

        print("Entering balance stand mode...")
        conn.balance_stand()
        time.sleep(1)

        print(
            f"Watching for persons (check every {args.check_interval}s, "
            f"give up after {args.max_wait:.0f}s)..."
        )
        path_clear = wait_for_clear_path(
            conn,
            model,
            conf=args.conf,
            close_ratio=args.close_ratio,
            check_interval=args.check_interval,
            max_wait=args.max_wait,
        )

        if path_clear:
            walk_forward(conn, distance=args.distance, speed=args.speed)
        else:
            print("Skipping forward walk — staying put.")

        print("Lying down...")
        conn.liedown()
        time.sleep(2)
        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted — stopping robot.")
        conn.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
    finally:
        conn.stop()


if __name__ == "__main__":
    main()
