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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Twist, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.core.global_config import GlobalConfig
from dimos.msgs.image import image_from_array
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def main() -> None:
    camera = CameraInfo(
        width=640,
        height=480,
        k=np.array([500, 0, 320, 0, 500, 240, 0, 0, 1], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        distortion_model="",
        d=np.array([], dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
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
        command = cdr_decode(cdr_encode(value), Twist)
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
        assert commands[0].linear.x == 0.5 and commands[-1] == Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        print("PASS: real control thread publishes motion, stops, and publishes zero")
        print("Tracker/model output is substituted; no inference or robot actuation.")
    finally:
        unsubscribe()
        module.stop()


if __name__ == "__main__":
    main()
