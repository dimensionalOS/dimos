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

from __future__ import annotations

import math
import time

import numpy as np
from reactivex import interval

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.visualization_msgs.EntityMarkers import EntityMarkers, Marker
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DemoRerunTopicSelectorPublisher(Module):
    """PC-only synthetic LCM publisher for the Rerun topic selector demo.

    Publishes a spread of topics that exercises the Visual Console catalog:
    a renderable point and markers, a heavy renderable camera image, an
    unsupported command topic, and a text/status topic.
    """

    selector_demo_point: Out[PointStamped]
    selector_demo_markers: Out[EntityMarkers]
    selector_demo_camera: Out[Image]
    selector_demo_cmd_vel: Out[Twist]
    selector_demo_status: Out[str]

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("Starting Rerun topic selector demo publisher")
        self.register_disposable(interval(0.25).subscribe(self._publish_sample_safe))

    def _publish_sample_safe(self, tick: int) -> None:
        try:
            self._publish_sample(tick)
        except Exception:
            logger.exception("Rerun topic selector demo publisher failed")

    def _publish_sample(self, tick: int) -> None:
        now = time.time()
        phase = tick * 0.1
        x = math.cos(phase)
        y = math.sin(phase)
        z = 0.5 + 0.25 * math.sin(phase * 0.5)

        self.selector_demo_point.transport.publish(
            PointStamped(x=x, y=y, z=z, ts=now, frame_id="world")
        )
        self.selector_demo_markers.transport.publish(
            EntityMarkers(
                markers=[
                    Marker(
                        entity_id="demo-orbit",
                        label="Orbiting demo point",
                        entity_type="object",
                        x=x,
                        y=y,
                        z=z,
                    ),
                    Marker(
                        entity_id="demo-origin",
                        label="World origin",
                        entity_type="location",
                        x=0.0,
                        y=0.0,
                        z=0.0,
                    ),
                ],
                ts=now,
            )
        )
        self.selector_demo_camera.transport.publish(self._sample_image(tick, now))
        self.selector_demo_cmd_vel.transport.publish(
            Twist((0.4 * x, 0.4 * y, 0.0), (0.0, 0.0, 0.2 * math.sin(phase)))
        )
        self.selector_demo_status.transport.publish(f"selector demo tick={tick}")

    @staticmethod
    def _sample_image(tick: int, now: float) -> Image:
        """Build a small moving-gradient frame; heavy enough for the heavy badge."""

        size = 96
        ramp = (np.arange(size, dtype=np.uint16) * 255 // (size - 1)).astype(np.uint8)
        data = np.zeros((size, size, 3), dtype=np.uint8)
        data[:, :, 0] = ramp[np.newaxis, :]
        data[:, :, 1] = ramp[:, np.newaxis]
        data[:, :, 2] = (tick * 8) % 256
        return Image(data=data, format=ImageFormat.RGB, ts=now, frame_id="world")


demo_rerun_topic_selector = autoconnect(
    DemoRerunTopicSelectorPublisher.blueprint(),
)
