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

"""Run the person-follow control thread with deterministic tracker output."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from dimos_generated.geometry_msgs.msg import Twist
from dimos_generated.sensor_msgs.msg import CameraInfo
import numpy as np

from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.core.global_config import GlobalConfig
from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def main() -> None:
    camera = CameraInfo(width=640, height=480, k=[500, 0, 320, 0, 500, 240, 0, 0, 1])
    with patch("dimos.agents.skills.person_follow.create", return_value=MagicMock()):
        module = PersonFollowSkillContainer(camera_info=camera, g=GlobalConfig(simulation=""))
    image = image_from_array(np.zeros((480, 640, 3), dtype=np.uint8), encoding="rgb8")
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
        process_image=lambda frame: ImageDetections2D(frame, [detection]),
        stop=lambda: None,
    )
    commands: list[Twist] = []

    def receive(value: Twist) -> None:
        command = Twist.decode(value.encode())
        commands.append(command)
        print(
            f"CDR command: forward={command.linear.x:.2f} m/s, turn={command.angular.z:.2f} rad/s"
        )
        module._should_stop.set()

    unsubscribe = module.cmd_vel.subscribe(receive)
    try:
        module._tracker = tracker
        module._on_color_image(image)
        print(module._follow_person("synthetic person", (270, 100, 370, 300)))
        thread = module._thread
        assert thread is not None
        thread.join(timeout=2)
        assert not thread.is_alive()
        assert commands[0].linear.x == 0.5 and commands[-1] == Twist()
        print("PASS: real control thread publishes motion, stops, and publishes zero")
        print("Tracker/model output is substituted; no inference or robot actuation.")
    finally:
        unsubscribe()
        module.stop()


if __name__ == "__main__":
    main()
