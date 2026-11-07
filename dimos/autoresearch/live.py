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

"""The backend a program drives on a running stack: ``run_program`` names
``dimos.autoresearch.live:backend`` and the program subprocess calls it."""

from __future__ import annotations

import atexit
from pathlib import Path
import threading

from dimos.autoresearch.program import DimosBackend, GripperRange, RobotBackend
from dimos.core.stream import RemoteOut
from dimos.manipulation.sdk import Arm
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.experimental.object_scene_registration_spec import (
    ObjectSceneRegistrationSpec,
)
from dimos.porcelain.dimos import Dimos

# The Metal's jaw joint (joint7) in metres: 0 closed, -0.05 fully open (METAL_GRIPPER_RANGE).
METAL_GRIPPER = GripperRange(closed=0.0, open=-0.05)
# How long the first camera frame may take to arrive, seconds.
FRAME_TIMEOUT_S = 10.0


class StreamObserver:
    """Keeps the newest frame of an image stream and writes it as a PNG on ``capture``."""

    def __init__(self, stream: RemoteOut[Image]) -> None:
        self._frame: Image | None = None
        self._arrived = threading.Event()
        self._unsubscribe = stream.subscribe(self._on_frame)

    def _on_frame(self, image: Image) -> None:
        self._frame = image
        self._arrived.set()

    def capture(self, path: Path) -> None:
        if not self._arrived.wait(FRAME_TIMEOUT_S):
            raise RuntimeError(f"no camera frame arrived within {FRAME_TIMEOUT_S} s")
        frame = self._frame
        assert frame is not None
        import cv2

        if not cv2.imwrite(str(path), frame.to_opencv()):
            raise RuntimeError(f"could not write {path}")

    def close(self) -> None:
        self._unsubscribe()


def backend(trial_dir: Path) -> RobotBackend:
    """Attach to the stack on this bus and return its ``DimosBackend``: the one
    arm, the scene module and the camera's ``color_image`` stream for keyframes.
    The connection closes when the process exits."""
    app = Dimos.connect()
    atexit.register(app.stop)
    cameras = [
        info
        for info in app.list_streams()
        if info.name == "color_image" and info.direction == "out"
    ]
    if len(cameras) != 1 or cameras[0].module_name is None:
        raise RuntimeError(
            f"expected one color_image stream, found {[c.module_name for c in cameras]}"
        )
    observer = StreamObserver(app.get_module(cameras[0].module_name).get_stream("color_image"))
    atexit.register(observer.close)
    arm = Arm.from_app(app)
    scene = app.find_module_by_spec(ObjectSceneRegistrationSpec)
    return DimosBackend(arm, scene, observer, METAL_GRIPPER)
