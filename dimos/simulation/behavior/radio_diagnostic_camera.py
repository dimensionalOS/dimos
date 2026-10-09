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

"""Opt-in evaluator viewer recording; never added to robot observations."""

import json
from pathlib import Path
import time
from typing import Any

import numpy as np


class RadioDiagnosticCamera:
    def __init__(self, sensor: Any, directory: str) -> None:
        import cv2

        self.cv2 = cv2
        self.sensor = sensor
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.writer: Any = None
        self.last = 0.0
        self.frames = 0
        self.metadata = (self.directory / "evaluator-sidecam-frames.jsonl").open("w")

    def capture(self, ts: float) -> None:
        now = time.monotonic()
        if now - self.last < 0.25:
            return
        self.last = now
        try:
            observation, _ = self.sensor.get_obs()
            rgb = observation["rgb"]
            if hasattr(rgb, "detach"):
                rgb = rgb.detach().cpu().numpy()
            rgb_image = np.asarray(rgb, dtype=np.uint8)[..., :3]
            if rgb_image.ndim != 3 or rgb_image.shape[2] != 3 or not rgb_image.size:
                raise ValueError("Evaluator viewer RGB unavailable")
            image = self.cv2.resize(rgb_image[..., ::-1], (640, 480))
            stage_file = self.directory / "active-stage.json"
            stage = json.loads(stage_file.read_text()) if stage_file.exists() else "initializing"
            self.cv2.putText(
                image,
                f"EVALUATOR ONLY | oracle development | {stage}",
                (8, 20),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
            )
            if self.writer is None:
                self.writer = self.cv2.VideoWriter(
                    str(self.directory / "evaluator-sidecam.avi"),
                    self.cv2.VideoWriter.fourcc(*"MJPG"),
                    4.0,
                    (640, 480),
                )
                if not self.writer.isOpened():
                    raise RuntimeError("Evaluator video writer unavailable")
            self.writer.write(image)
            self.metadata.write(
                json.dumps(
                    {
                        "frame": self.frames,
                        "timestamp": ts,
                        "monotonic": now,
                        "stage": stage,
                        "evaluator_only": True,
                    }
                )
                + "\n"
            )
            self.metadata.flush()
            self.frames += 1
        except Exception as error:
            # Recording failures remain explicit and never affect physical actuation.
            self.metadata.write(json.dumps({"timestamp": ts, "error": repr(error)}) + "\n")
            self.metadata.flush()

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
        self.metadata.close()
