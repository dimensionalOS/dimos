# Copyright 2025 Dimensional Inc.
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

import asyncio
import logging
import threading

# this is missing, I'm just trying to import lcm_foxglove_bridge.py from dimos_lcm
from dimos_lcm.foxglove_bridge import (  # type: ignore[import-untyped]
    FoxgloveBridge as LCMFoxgloveBridge,
)

from dimos.core import DimosCluster, Module, rpc

logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)


class FoxgloveBridge(Module):
    _thread: threading.Thread
    _loop: asyncio.AbstractEventLoop

    def __init__(self, *args, shm_channels=None, jpeg_shm_channels=None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.shm_channels = shm_channels or []
        self.jpeg_shm_channels = jpeg_shm_channels or []

    @rpc
    def start(self) -> None:
        super().start()

        def run_bridge() -> None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            try:
                for logger in ["lcm_foxglove_bridge", "FoxgloveServer"]:
                    logger = logging.getLogger(logger)  # type: ignore[assignment]
                    logger.setLevel(logging.ERROR)  # type: ignore[attr-defined]
                    for handler in logger.handlers:  # type: ignore[attr-defined]
                        handler.setLevel(logging.ERROR)

                bridge = LCMFoxgloveBridge(
                    host="0.0.0.0",
                    port=8765,
                    debug=False,
                    num_threads=4,
                    shm_channels=self.shm_channels,
                    jpeg_shm_channels=self.jpeg_shm_channels,
                )
                self._loop.run_until_complete(bridge.run())
            except Exception as e:
                print(f"Foxglove bridge error: {e}")

        self._thread = threading.Thread(target=run_bridge, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(timeout=2)

        super().stop()


def deploy(
    dimos: DimosCluster,
    shm_channels: list[str] | None = None,
) -> FoxgloveBridge:
    if shm_channels is None:
        shm_channels = [
            "/image#sensor_msgs.Image",
            "/lidar#sensor_msgs.PointCloud2",
            "/map#sensor_msgs.PointCloud2",
        ]
    foxglove_bridge = dimos.deploy(  # type: ignore[attr-defined]
        FoxgloveBridge,
        shm_channels=shm_channels,
    )
    foxglove_bridge.start()
    return foxglove_bridge  # type: ignore[no-any-return]


foxglove_bridge = FoxgloveBridge.blueprint


__all__ = ["FoxgloveBridge", "deploy", "foxglove_bridge"]
