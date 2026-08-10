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

"""Generate raw semantic detections for go2_bigoffice (run once; frozen data).

Runs a YOLO detector over every FRAME_STRIDE-th color frame and records, per
detection: class_name, confidence, ts (seconds from first frame), and the
robot odom pose at that ts — world x/y/z plus yaw (radians, from the odom
quaternion). Object world position = robot position at detection time
(odom-grounding convention: ~1-2 m error, benchmark bands account for it).

Output: detections.json next to this script, plus a class histogram on stdout.
"""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import numpy as np

from dimos.memory2.cli.dataset import open_dataset

DATASET = "go2_bigoffice"
MODEL = "yolov8m.pt"
CONF = 0.4
FRAME_STRIDE = 5
WEIGHTS_DIR = Path.home() / ".cache" / "ultralytics"  # never the repo


def load_model():
    from ultralytics import YOLO
    from ultralytics.utils.downloads import attempt_download_asset

    WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    weights = WEIGHTS_DIR / MODEL
    if not weights.exists():
        attempt_download_asset(str(weights))
    return YOLO(str(weights))


def main() -> None:
    model = load_model()
    store = open_dataset(DATASET)
    detections: list[dict[str, object]] = []
    try:
        odom = store.streams.odom.to_list()
        odom_ts = np.array([o.ts for o in odom])
        frames = store.streams.color_image.to_list()[::FRAME_STRIDE]
        t0 = frames[0].ts
        for k, obs in enumerate(frames):
            result = model.predict(obs.data.to_opencv(), conf=CONF, verbose=False)[0]
            if len(result.boxes) == 0:
                continue
            od = odom[int(np.abs(odom_ts - obs.ts).argmin())].data
            yaw = float(od.orientation.euler[2])
            for box in result.boxes:
                detections.append(
                    {
                        "class_name": result.names[int(box.cls)],
                        "confidence": round(float(box.conf), 3),
                        "ts": round(obs.ts - t0, 2),
                        "x": round(float(od.position.x), 3),
                        "y": round(float(od.position.y), 3),
                        "z": round(float(od.position.z), 3),
                        "yaw": round(yaw, 3),
                    }
                )
            if k % 100 == 0:
                print(f"frame {k}/{len(frames)}: {len(detections)} detections so far")
    finally:
        store.stop()

    out = Path(__file__).parent / "detections.json"
    out.write_text(json.dumps(detections, indent=1) + "\n")
    print(f"\nwrote {len(detections)} detections from {len(frames)} frames -> {out}")
    for name, count in Counter(d["class_name"] for d in detections).most_common():
        print(f"  {name:20s} {count}")


if __name__ == "__main__":
    main()
