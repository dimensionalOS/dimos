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

"""One camera entity whichever encoding GO2DDS serves: jpeg `image` is redirected onto `video`."""

from typing import Any

import numpy as np
import rerun as rr

from dimos.message_utils import camera_info, time as message_time
from dimos.messages.sensor_msgs.msg import CompressedImage
from dimos.robot.unitree.go2.dds.blueprints import CAMERA_ENTITY, go2_nav_viewer


def _rerun_kwargs(blueprint: Any) -> dict[str, Any]:
    (atom,) = (a for a in blueprint.active_blueprints if a.name == "rerunbridgemodule")
    return atom.kwargs


def test_both_encodings_land_on_the_pane() -> None:
    kwargs = _rerun_kwargs(go2_nav_viewer)
    overrides = kwargs["visual_override"]
    assert kwargs["blueprint"]().root_container is not None
    info = camera_info.from_intrinsics(
        width=640,
        height=480,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        header=message_time.header_now("camera_optical"),
    )
    [(pinhole_path, _)] = overrides["world/camera_info"](info)
    jpeg = CompressedImage(
        header=message_time.header_now("camera_optical"),
        data=np.frombuffer(b"\xff\xd8", dtype=np.uint8).copy(),
        format="jpeg",
    )
    [(image_path, archetype)] = overrides["world/image"](jpeg)
    assert pinhole_path == image_path == CAMERA_ENTITY
    assert isinstance(archetype, rr.EncodedImage)


def test_viewer_subscribes_both_encodings() -> None:
    topics = _rerun_kwargs(go2_nav_viewer)["topics"]
    assert {"video", "image", "camera_info"} <= set(topics)
