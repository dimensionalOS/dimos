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

from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.geometry_msgs.msg import Twist
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
import numpy as np
import pytest

from dimos.agents.skills import person_follow
from dimos.core.global_config import GlobalConfig
from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


@pytest.mark.parametrize("lost", [False, True])
def test_generated_person_follow_loop_stops_cleanly(monkeypatch, lost):
    monkeypatch.setattr(person_follow, "create", lambda _: MagicMock())
    camera = CameraInfo(width=640, height=480, k=[500, 0, 320, 0, 500, 240, 0, 0, 1])
    module = person_follow.PersonFollowSkillContainer(
        camera_info=camera, g=GlobalConfig(simulation="")
    )
    image = image_from_array(np.zeros((480, 640, 3), dtype=np.uint8), encoding="rgb8")
    image = Image.decode(image.encode())
    detection = Detection2DBBox(
        bbox=(270, 100, 370, 300),
        track_id=1,
        class_id=0,
        confidence=1,
        name="person",
        ts=0,
        image=image,
    )
    tracker = SimpleNamespace(
        init_track=lambda **kwargs: ImageDetections2D(image, [detection]),
        stop=lambda: None,
        process_image=lambda frame: ImageDetections2D(frame, [] if lost else [detection]),
    )
    outputs = []

    def receive(value):
        outputs.append(Twist.decode(value.encode()))
        if not lost:
            module._should_stop.set()

    unsubscribe = module.cmd_vel.subscribe(receive)
    module._max_lost_frames = 0
    module._frequency = 1000
    try:
        module._on_color_image(image)
        module._tracker = tracker
        status = module._follow_person("person", (270, 100, 370, 300))
        assert "Starting to follow" in status
        thread = module._thread
        assert thread is not None
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert outputs
        assert outputs[-1] == Twist()
        if lost:
            assert all(value == Twist() for value in outputs)
        else:
            assert outputs[0].linear.x == 0.5
            assert outputs[0].angular.z == 0
    finally:
        unsubscribe()
        module.stop()
