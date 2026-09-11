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

"""One shared DimOS detector for native MuJoCo head-camera observations."""

import base64
import json
import logging
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any

import cv2
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.web.codecs import web_encoder
from microduck_world.robot_io import ROBOT_IDS, RobotVision
from microduck_world.scene import PROJECT_ROOT
from reactivex.disposable import Disposable

BALL_CAMERA_ENCODING = "ball-camera.json.v1"
BALL_CAMERA_CHANNELS = tuple(f"{robot}_ball_camera" for robot in ROBOT_IDS)
logger = logging.getLogger(__name__)


@web_encoder(BALL_CAMERA_ENCODING)
def encode_ball_camera(message: str) -> bytes:
    return message.encode("utf-8")


@dataclass(frozen=True)
class BallBox:
    xyxy: tuple[float, float, float, float]
    confidence: float


@dataclass(frozen=True)
class FootballObservation:
    """Pixels and detections share a timestamp and participant generation."""

    robot: str
    generation: str
    timestamp: float
    boxes: tuple[BallBox, ...]


class BallPerceptionConfig(ModuleConfig):
    confidence: float = 0.25
    max_frame_age: float = 1.5
    model_name: str = "yolo11n.pt"
    model_path: str = str(PROJECT_ROOT / "cache/models")
    device: str | None = None


class BallPerception(Module):
    config: BallPerceptionConfig
    duck1_vision: In[RobotVision]
    duck2_vision: In[RobotVision]
    duck3_vision: In[RobotVision]
    duck4_vision: In[RobotVision]
    duck5_vision: In[RobotVision]
    duck6_vision: In[RobotVision]
    duck1_ball_camera: Out[str]
    duck2_ball_camera: Out[str]
    duck3_ball_camera: Out[str]
    duck4_ball_camera: Out[str]
    duck5_ball_camera: Out[str]
    duck6_ball_camera: Out[str]
    ball_detections: Out[FootballObservation]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._condition = threading.Condition()
        self._pending: OrderedDict[str, RobotVision] = OrderedDict()
        self._generations: dict[str, str] = {}
        self._stopped = False
        self._thread: threading.Thread | None = None
        self._detector: Any = None
        self._load_error = False
        self._last_inference_error = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        for robot in ROBOT_IDS:
            self.register_disposable(
                Disposable(
                    getattr(self, f"{robot}_vision").subscribe(
                        lambda vision, robot=robot: self.receive(robot, vision)
                    )
                )
            )
        self._thread = threading.Thread(target=self._run, daemon=True, name="football-perception")
        self._thread.start()

    def receive(self, robot: str, vision: RobotVision) -> None:
        if robot not in ROBOT_IDS or not vision.generation:
            return
        with self._condition:
            if self._stopped:
                return
            # Replacement retains the duck's queue position: a faster camera
            # cannot starve another duck, and only six RGB-D frames are retained.
            self._generations[robot] = vision.generation
            self._pending[robot] = vision
            self._condition.notify()

    def _run(self) -> None:
        try:
            from dimos.perception.detection.detectors.yolo import Yolo2DDetector

            self._detector = Yolo2DDetector(
                model_path=self.config.model_path,
                model_name=self.config.model_name,
                device=self.config.device,
            )
        except Exception:
            self._load_error = True
            logger.exception("Football detector could not load")
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._stopped or bool(self._pending))
                    if self._stopped:
                        break
                    robot, vision = self._pending.popitem(last=False)
                if time.time() - vision.image.ts > self.config.max_frame_age:
                    continue
                try:
                    payload, observation = self.process(robot, vision)
                except Exception:
                    if time.monotonic() - self._last_inference_error >= 30:
                        logger.exception("Football detector inference failed")
                        self._last_inference_error = time.monotonic()
                    payload, observation = (
                        self.camera_payload(robot, vision, (), "unavailable"),
                        None,
                    )
                with self._condition:
                    if self._stopped or self._generations.get(robot) != vision.generation:
                        continue
                if time.time() - vision.image.ts > self.config.max_frame_age:
                    continue
                getattr(self, f"{robot}_ball_camera").publish(json.dumps(payload))
                if observation is not None:
                    self.ball_detections.publish(observation)
        finally:
            if self._detector is not None:
                self._detector.stop()

    def process(
        self, robot: str, vision: RobotVision
    ) -> tuple[dict[str, Any], FootballObservation | None]:
        if self._load_error or self._detector is None:
            return self.camera_payload(robot, vision, (), "unavailable"), None
        # Stateless prediction is essential when a single model serves six
        # cameras. The detector's persistent single-camera tracker is not used.
        results = self._detector.model.predict(
            source=vision.image.to_opencv(),
            device=self._detector.device,
            classes=[32],
            conf=self.config.confidence,
            iou=0.6,
            imgsz=640,
            verbose=False,
        )
        boxes: list[BallBox] = []
        for result in results:
            if result.boxes is None:
                continue
            for xyxy, confidence, cls in zip(
                result.boxes.xyxy.cpu().tolist(),
                result.boxes.conf.cpu().tolist(),
                result.boxes.cls.cpu().tolist(),
                strict=True,
            ):
                if int(cls) == 32:
                    boxes.append(
                        BallBox(
                            (float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])),
                            float(confidence),
                        )
                    )
        observation = FootballObservation(robot, vision.generation, vision.image.ts, tuple(boxes))
        return self.camera_payload(robot, vision, observation.boxes, "ready"), observation

    def camera_payload(
        self, robot: str, vision: RobotVision, boxes: tuple[BallBox, ...], status: str
    ) -> dict[str, Any]:
        pixels = vision.image.to_opencv()
        ok, jpeg = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 65])
        if not ok:
            raise ValueError("Could not encode camera image")
        return {
            "robot": robot,
            "generation": vision.generation,
            "ts": vision.image.ts,
            "width": pixels.shape[1],
            "height": pixels.shape[0],
            "status": status,
            "image": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii"),
            "boxes": [{"xyxy": b.xyxy, "confidence": b.confidence} for b in boxes],
            "model": self.config.model_name,
            "class": "sports ball",
        }

    @rpc
    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._pending.clear()
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=15)
        super().stop()
