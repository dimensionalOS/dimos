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

"""Python publishes a tf chain, a Rust #[tf] module reads it back.

Run with:
    python examples/native-modules/rust_tf.py
"""

from __future__ import annotations

import asyncio
import math
from pathlib import Path
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_RUST_DIR = Path(__file__).parent / "rust"
_EXAMPLES = _RUST_DIR / "target" / "release"
_BUILD = "cargo build --release"


class TfProducer(Module):
    """Publishes a time-varying a -> b -> c transform chain onto /tf."""

    _running: bool = False

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True
        self.spawn(self._publish_loop())

    async def _publish_loop(self) -> None:
        start = time.time()
        while self._running:
            t = time.time() - start
            now = time.time()
            self.tf.publish(
                Transform(
                    translation=Vector3(0.0, math.cos(t), math.sin(t)),
                    frame_id="a",
                    child_frame_id="b",
                    ts=now,
                ),
                Transform(
                    translation=Vector3(1.0, 0.0, 0.0),
                    frame_id="b",
                    child_frame_id="c",
                    ts=now,
                ),
            )
            await asyncio.sleep(0.1)

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()


class TfListenerConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "tf_listener")
    build_command: str = _BUILD
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class TfListenerModule(NativeModule):
    """Rust module that looks up a -> c and logs it.

    Expect to see (1.0, cos(t), sin(t))
    """

    config: TfListenerConfig


if __name__ == "__main__":
    bp = autoconnect(TfProducer.blueprint(), TfListenerModule.blueprint()).global_config(
        viewer="none"
    )
    ModuleCoordinator.build(bp).loop()
