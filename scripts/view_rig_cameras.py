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

"""Standalone viewer for the four openarm rig camera streams.

Run alongside `learning-collect-quest-openarm-multicam` to confirm each
labeled camera is actually streaming before or during a collection session.
Independent of the recorder; only subscribes, never writes to the DB.

    uv run python scripts/view_rig_cameras.py
"""

from __future__ import annotations

import threading

import cv2
import numpy as np

from dimos.core.transport_factory import make_transport
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

TOPICS = ("chest_image", "left_hand_image", "right_hand_image", "waist_image")
TILE_SIZE = (320, 240)


def _to_bgr(image: Image) -> np.ndarray:
    frame = image.data
    if image.format == ImageFormat.RGB:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return cv2.resize(frame, TILE_SIZE)


def main() -> None:
    latest: dict[str, np.ndarray] = {}
    lock = threading.Lock()

    def make_handler(topic: str):
        def handler(msg: Image) -> None:
            with lock:
                latest[topic] = _to_bgr(msg)

        return handler

    for topic in TOPICS:
        transport = make_transport(topic, Image)
        transport.subscribe(make_handler(topic))

    print("Waiting for rig camera frames... (Ctrl-C to stop)")
    blank = np.zeros((TILE_SIZE[1], TILE_SIZE[0], 3), dtype=np.uint8)
    try:
        while True:
            with lock:
                tiles = [latest.get(topic, blank).copy() for topic in TOPICS]
            for tile, topic in zip(tiles, TOPICS, strict=True):
                label = topic.removesuffix("_image")
                cv2.putText(
                    tile, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
            top = np.hstack(tiles[:2])
            bottom = np.hstack(tiles[2:])
            grid = np.vstack([top, bottom])
            cv2.imshow("Rig cameras: chest | left_hand / right_hand | waist", grid)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
