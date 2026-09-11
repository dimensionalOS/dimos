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

import base64
import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from microduck_world.ball_detection import BallPerception


def vision(generation="first"):
    pixels = np.zeros((36, 64, 3), dtype=np.uint8)
    pixels[:, :, 0] = 220
    return SimpleNamespace(
        generation=generation, image=Image(data=pixels, format=ImageFormat.RGB, ts=time.time())
    )


def test_shared_queue_is_bounded_latest_per_duck_and_fair(module_factory):
    module = module_factory(BallPerception)
    for i in range(1, 7):
        module.receive(f"duck{i}", vision())
    latest = vision("second")
    module.receive("duck1", latest)
    assert len(module._pending) == 6
    assert list(module._pending)[0] == "duck1"
    assert module._pending["duck1"] is latest
    assert module._generations["duck1"] == "second"
    module.receive("unknown", vision())
    assert len(module._pending) == 6


def test_frame_and_boxes_share_camera_identity_and_pixels(module_factory):
    module = module_factory(BallPerception)
    module._detector = Mock(device="cpu")
    boxes = Mock()
    boxes.xyxy.cpu().tolist.return_value = [[2, 3, 22, 23], [1, 1, 8, 8]]
    boxes.conf.cpu().tolist.return_value = [0.9, 0.8]
    boxes.cls.cpu().tolist.return_value = [32, 0]
    module._detector.model.predict.return_value = [SimpleNamespace(boxes=boxes)]
    camera = vision()
    payload, observation = module.process("duck3", camera)
    assert payload["robot"] == "duck3"
    assert payload["generation"] == "first"
    assert payload["ts"] == camera.image.ts
    assert payload["boxes"] == [{"xyxy": (2.0, 3.0, 22.0, 23.0), "confidence": 0.9}]
    assert observation.robot == "duck3"
    assert observation.timestamp == camera.image.ts
    module._detector.model.track.assert_not_called()
    assert module._detector.model.predict.call_args.kwargs["classes"] == [32]
    pixels = cv2.imdecode(
        np.frombuffer(base64.b64decode(payload["image"].split(",")[1]), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    assert pixels.shape == (36, 64, 3)
    assert pixels[0, 0, 2] > 200 and pixels[0, 0, 0] < 10


def test_detector_failure_is_distinct_from_a_frame_with_no_ball(module_factory):
    module = module_factory(BallPerception)
    payload, observation = module.process("duck1", vision())
    assert payload["status"] == "unavailable"
    assert payload["image"].startswith("data:image/jpeg;base64,")
    assert observation is None
