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

"""Python and Rust each publish part of a tf chain, a Rust #[tf] module reads it back.

Python publishes a -> b -> c, a Rust broadcaster publishes c -> d, and a Rust
listener composes the full a -> d transform.

Run with:
    python examples/native-modules/rust_tf.py
"""

from __future__ import annotations

import asyncio
import math
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO
from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage

_BUILD = "nix build -L path:.#dimos-native-module-examples"


class TfProducer(Module):
    """Publishes a time-varying a -> b -> c transform chain onto /tf."""

    tf: IO[TFMessage]

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
            self.tfbuffer.publish(
                TransformStamped(
                    Header(message_time.time_from_seconds(now), "a"),
                    "b",
                    Transform(
                        Vector3(0.0, math.cos(t), math.sin(t)), Quaternion(0.0, 0.0, 0.0, 1.0)
                    ),
                ),
                TransformStamped(
                    Header(message_time.time_from_seconds(now), "b"),
                    "c",
                    Transform(Vector3(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                ),
            )
            await asyncio.sleep(0.1)

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()


class TfListenerConfig(NativeModuleConfig):
    executable: str = "result/bin/tf_listener"
    build_command: str = _BUILD
    source_dir: str = "examples/native-modules/rust"
    stdin_config: bool = True


class TfListenerModule(NativeModule):
    """Rust module that looks up a -> d and logs it.

    Expect to see (1.5, cos(t), sin(t))
    """

    config: TfListenerConfig
    tf: IO[TFMessage]


class TfBroadcasterConfig(NativeModuleConfig):
    executable: str = "result/bin/tf_broadcaster"
    build_command: str = _BUILD
    source_dir: str = "examples/native-modules/rust"
    stdin_config: bool = True


class TfBroadcasterModule(NativeModule):
    """Rust module that publishes the c -> d transform."""

    config: TfBroadcasterConfig
    tf: IO[TFMessage]


if __name__ == "__main__":
    bp = autoconnect(
        TfProducer.blueprint(), TfBroadcasterModule.blueprint(), TfListenerModule.blueprint()
    ).global_config(viewer="none")
    ModuleCoordinator.build(bp).loop()
