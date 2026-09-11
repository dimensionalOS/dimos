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

"""A depth-camera recording published on ports at wall-clock pace.

Stands in for the xArm's sensors: ``color_image``, ``depth_image``,
``camera_info`` and ``tf`` come out exactly as the recorder took them in, on
the same topics a live rig would publish.

One coordinator per bus is allowed, so beside a running blueprint the feed
runs as a plain process, with its port transports built the way the
coordinator builds them:

    uv run python -m dimos.perception.memory.demo_blueprints.xarm_feed [--seek 427 --duration 76]

The ``xarm-feed`` blueprint is the same module for composing into one
blueprint with its consumers.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.replay import ReplayStream
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

XARM_DATASET = (
    "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
    "xarm6_worldbelief_20260729_203624_161992.db"
)


class DepthCameraFeedConfig(ModuleConfig):
    dataset: str = XARM_DATASET
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None


class DepthCameraFeed(Module):
    """Replay a recording's colour, depth, camera_info and tf as the sensors published them."""

    config: DepthCameraFeedConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        given = Path(self.config.dataset)
        path = given if given.exists() else get_data(self.config.dataset)
        store = self.register_disposable(SqliteStore(path=str(path), must_exist=True))
        store.start()
        replay = store.replay(
            speed=self.config.speed, seek=self.config.seek, duration=self.config.duration
        )
        ports: dict[str, Out[Any]] = {
            "color_image": self.color_image,
            "depth_image": self.depth_image,
            "camera_info": self.camera_info,
            "tf": self.tf,
        }
        for name, port in ports.items():
            stream: ReplayStream[Any] = replay.stream(name)
            logger.info(
                f"feed: {path.name}:{name} ({stream.count()} frames at {self.config.speed}x)"
            )
            self.register_disposable(stream.observable().subscribe(port.publish))


xarm_feed = DepthCameraFeed.blueprint()


def main() -> int:
    from dimos.core.transport_factory import make_transport

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=XARM_DATASET)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--seek", type=float, default=None, help="start offset into the recording (s)"
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to play (s)")
    args = parser.parse_args()

    feed = DepthCameraFeed(
        dataset=args.dataset, speed=args.speed, seek=args.seek, duration=args.duration
    )
    for name, port in feed.outputs.items():
        feed.set_transport(name, make_transport(f"/{name}", port.type))
    feed.start()
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    feed.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
