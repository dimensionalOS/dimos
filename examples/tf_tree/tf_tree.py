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

"""Live TF-tree demo.

Three modules wired with autoconnect:

  - FakeTfPublisher: publishes an animated humanoid-on-a-rover transform tree
                     on the standard `/tf` topic via the module TF subsystem.
  - DimosWebsocket:  streams + hosts web pages over one websocket/HTTP server.
  - TfWebUi:         a @web_module whose page (served by the bridge at /TfWebUi)
                     subscribes to `/tf` via @dimos/client and draws it live.

Run it:  python examples/tf_tree/tf_tree.py
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
import math
import os
from pathlib import Path
import time
import webbrowser

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, web_init, web_module
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger
from dimos.web.ts_bridge.ts_bridge_module import DimosWebsocket

logger = setup_logger()

PUBLISH_HZ = 30.0
INDEX_HTML = Path(__file__).parent / "index.html"


def _yaw(angle: float) -> Quaternion:
    return Quaternion.from_euler(Vector3(0.0, 0.0, angle))


class FakeTfPublisher(Module):
    """Publishes an animated transform tree so the demo has something to show."""

    async def main(self) -> AsyncIterator[None]:
        # Accessing self.tf brings up the LCM TF subsystem (publishes on /tf).
        assert self.tf is not None
        task = asyncio.create_task(self._publish_loop())
        yield
        task.cancel()

    async def _publish_loop(self) -> None:
        start = time.time()
        try:
            while True:
                await asyncio.sleep(1.0 / PUBLISH_HZ)
                self.tf.publish(*self._tree(time.time() - start))
        except asyncio.CancelledError:
            pass

    def _tree(self, t: float) -> list[Transform]:
        """A small humanoid-on-a-rover tree that wiggles over time."""
        swing = math.sin(t * 1.5)
        now = time.time()

        def link(parent: str, child: str, xyz: Vector3, rotation: Quaternion) -> Transform:
            return Transform(
                translation=xyz, rotation=rotation, frame_id=parent, child_frame_id=child, ts=now
            )

        return [
            # rover driving a slow circle around the origin
            link(
                "world",
                "base_link",
                Vector3(2.0 * math.cos(t * 0.4), 2.0 * math.sin(t * 0.4), 0.0),
                _yaw(t * 0.4 + math.pi / 2),
            ),
            link("base_link", "lidar", Vector3(0.0, 0.0, 0.35), _yaw(t * 3.0)),  # spinning lidar
            link("base_link", "torso", Vector3(0.0, 0.0, 0.5), _yaw(0.0)),
            link("torso", "head", Vector3(0.0, 0.0, 0.45), _yaw(0.6 * math.sin(t * 2.0))),
            # left arm
            link("torso", "left_shoulder", Vector3(0.0, 0.25, 0.3), _yaw(0.0)),
            link("left_shoulder", "left_arm", Vector3(0.0, 0.0, -0.3), _yaw(0.8 * swing)),
            link("left_arm", "left_hand", Vector3(0.0, 0.0, -0.3), _yaw(0.0)),
            # right arm (mirrored phase)
            link("torso", "right_shoulder", Vector3(0.0, -0.25, 0.3), _yaw(0.0)),
            link("right_shoulder", "right_arm", Vector3(0.0, 0.0, -0.3), _yaw(-0.8 * swing)),
            link("right_arm", "right_hand", Vector3(0.0, 0.0, -0.3), _yaw(0.0)),
        ]


@web_module
class TfWebUi(Module):
    """The live TF-tree page. Served by the bridge at /TfWebUi — no server here."""

    @web_init
    def _web_init(self) -> dict[str, str | Path]:
        # "icon" can be any format — here a resolution-independent SVG.
        return {"index.html": INDEX_HTML, "icon": INDEX_HTML.parent / "logo.svg"}

    @rpc
    def start(self) -> None:
        super().start()
        url = f"http://localhost:{self.config.g.ts_api_port}/{type(self).__name__}"
        logger.info(f"TF tree UI: {url}")
        if os.environ.get("DIMOS_NO_BROWSER") != "1":
            try:
                webbrowser.open_new_tab(url)
            except Exception:
                pass


tf_tree = autoconnect(
    FakeTfPublisher.blueprint(),
    DimosWebsocket.blueprint(),
    TfWebUi.blueprint(),
)


if __name__ == "__main__":
    ModuleCoordinator.build(tf_tree).loop()
