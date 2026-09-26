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

"""Standalone RTSP camera viewer: the H.265 stream decoded in rerun, JPEGs beside it.

``dimos run rtsp-camera-vis --rtspcamera.url=rtsp://<camera>:8554/<path>``
``dimos run rtsp-camera-vis --rtspcamera.url=clip.mp4``     # replay a capture
``dimos run rtsp-camera-vis --rtspcamera.url=synthetic``    # no camera

Mirrors the RealSense standalone viewer. The rerun bridge never caps ``world/video``:
dropping H.265 access units breaks in-viewer decode, so the rate is set at the source.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.visualization.vis_module import vis_module


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/video", name="H.265"),
            rrb.Spatial2DView(origin="world/color_jpeg", name="Link JPEG"),
            column_shares=[2, 1],
        ),
    )


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "max_hz": {"world/color_jpeg": 2.0},
    # Raw decoded frames stay on this machine; the viewer decodes the H.265 itself.
    "visual_override": {"world/color_image": None},
}

rtsp_camera_vis = autoconnect(
    vis_module(global_config.viewer, rerun_config=_rerun_config),
    RtspCamera.blueprint(),
)
