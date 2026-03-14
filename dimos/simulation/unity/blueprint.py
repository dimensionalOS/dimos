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

"""Standalone Unity sim blueprint — interactive test of the Unity bridge.

Launches the Unity simulator, displays lidar + camera in Rerun, and accepts
keyboard teleop via TUI. No navigation stack — just raw sim data.

Usage:
    dimos run unity-sim
"""

from __future__ import annotations

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.rerun.bridge import _resolve_viewer_mode, rerun_bridge


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="3D"),
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            row_shares=[2, 1],
        ),
    )


rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "visual_override": {
        "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
    },
    "static": {
        "world/color_image": UnityBridgeModule.rerun_static_pinhole,
    },
}


unity_sim = autoconnect(
    UnityBridgeModule.blueprint(),
    rerun_bridge(viewer_mode=_resolve_viewer_mode(), **rerun_config),
)
