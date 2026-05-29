#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Standalone webcam test for the YOLO empty-seat logic (no robot needed).

Mirrors SeatFinderSkill's detection: YOLO detects chairs/couches and people,
then a seat is "occupied" if a person box overlaps it past a threshold.

Usage:
    .venv/bin/python demo_seat_check_webcam.py [--camera 0]

Overlay:
    green box  = empty seat
    red box    = occupied seat
    blue box   = person
    gray box   = other detected object
Keys: q / ESC to quit.
"""

from __future__ import annotations

import argparse

import cv2

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.yolo import Yolo2DDetector

SEAT_CLASSES = ("chair", "couch", "bench")
OCCUPANCY_OVERLAP = 0.2


def is_occupied(seat, persons) -> bool:
    sx1, sy1, sx2, sy2 = seat.bbox
    seat_area = max(1.0, (sx2 - sx1) * (sy2 - sy1))
    for p in persons:
        px1, py1, px2, py2 = p.bbox
        iw = max(0.0, min(sx2, px2) - max(sx1, px1))
        ih = max(0.0, min(sy2, py2) - max(sy1, py1))
        if (iw * ih) / seat_area > OCCUPANCY_OVERLAP:
            return True
    return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    args = parser.parse_args()

    print("Loading YOLO...")
    detector = Yolo2DDetector()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera index {args.camera}")
    print("Running. green=empty seat, red=occupied, blue=person. q/ESC to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        detections = detector.process_image(Image.from_opencv(frame)).detections
        persons = [d for d in detections if d.name == "person"]
        seats = [d for d in detections if d.name in SEAT_CLASSES]

        empty = 0
        for d in detections:
            x1, y1, x2, y2 = (int(v) for v in d.bbox)
            if d.name in SEAT_CLASSES:
                occupied = is_occupied(d, persons)
                color = (0, 0, 255) if occupied else (0, 255, 0)
                label = f"{d.name} {'OCCUPIED' if occupied else 'EMPTY'} {d.confidence:.2f}"
                empty += 0 if occupied else 1
            elif d.name == "person":
                color = (255, 0, 0)
                label = f"person {d.confidence:.2f}"
            else:
                color = (150, 150, 150)
                label = f"{d.name} {d.confidence:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, label, (x1, max(15, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
            )

        cv2.putText(
            frame, f"seats={len(seats)} empty={empty} persons={len(persons)}",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2,
        )
        cv2.imshow("YOLO empty-seat check", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
