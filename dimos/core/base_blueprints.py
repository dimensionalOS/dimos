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

import platform

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import Blueprint, autoconnect
from dimos.core.transport import (
    pSHMTransport,
)
from dimos.msgs.sensor_msgs import Image
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.visualization.rerun.bridge import rerun_bridge


def base_blueprint() -> Blueprint:
    from dimos.core.global_config import GlobalConfig

    _config = GlobalConfig()

    # Mac has some issue with high bandwidth UDP, so we use pSHMTransport for color_image
    # actually we can use pSHMTransport for all platforms, and for all streams
    # TODO need a global transport toggle on blueprints/global config
    mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
        ("color_image", Image): pSHMTransport(
            "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        ),
    }

    base = (
        autoconnect() if platform.system() == "Linux" else autoconnect().transports(mac_transports)
    )

    with_vis = None

    if _config.viewer_backend == "foxglove":
        with_vis = autoconnect(
            base,
            foxglove_bridge(shm_channels=["/color_image#sensor_msgs.Image"]),
        )
    elif _config.viewer_backend == "rerun":
        with_vis = autoconnect(base, rerun_bridge())
    elif _config.viewer_backend == "rerun-web":
        with_vis = autoconnect(base, rerun_bridge(viewer_mode="web"))
    else:
        with_vis = base

    return with_vis
