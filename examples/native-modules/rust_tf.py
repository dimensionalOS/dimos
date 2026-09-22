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
from pathlib import Path
import time

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO
from dimos.msgs.time import time_from_nanoseconds
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, frames_to_edge_transforms
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RUST_DIR = Path(__file__).parent / "rust"
# The crate is a workspace member, so cargo builds into the repo-root target dir.
_EXAMPLES = DIMOS_PROJECT_ROOT / "target" / "release"
_BUILD = "cargo build --release"


class TfProducer(Module):
    """Publishes the time-varying a -> b edge onto /tf."""

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
            stamp = time_from_nanoseconds(time.time_ns())
            self.tf.publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=Header(frame_id="a", stamp=stamp),
                            child_frame_id="b",
                            transform=Transform(
                                translation=Vector3(y=math.cos(t), z=math.sin(t)),
                                rotation=Quaternion(w=1),
                            ),
                        ),
                    ]
                )
            )
            composed = self.tfbuffer.get("a", "d", warn=False)
            if composed is not None:
                position = composed.transform.translation
                logger.info(
                    "Python TF lookup",
                    x=position.x,
                    y=position.y,
                    z=position.z,
                )
            await asyncio.sleep(0.1)

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()


class StaticMount(StaticTfPublisher):
    """Exercise the production periodic publisher for the fixed b -> c mount."""

    def transforms(self) -> list[TransformStamped]:
        return frames_to_edge_transforms([("c", "b", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))])


class TfListenerConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "tf_listener")
    build_command: str = _BUILD
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class TfListenerModule(NativeModule):
    """Rust module that looks up a -> d and logs it.

    Expect to see (1.5, cos(t), sin(t))
    """

    config: TfListenerConfig
    tf: IO[TFMessage]


class TfBroadcasterConfig(NativeModuleConfig):
    executable: str = str(_EXAMPLES / "tf_broadcaster")
    build_command: str = _BUILD
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True


class TfBroadcasterModule(NativeModule):
    """Rust module that publishes the c -> d transform."""

    config: TfBroadcasterConfig
    tf: IO[TFMessage]


if __name__ == "__main__":
    bp = autoconnect(
        TfProducer.blueprint(),
        StaticMount.blueprint(),
        TfBroadcasterModule.blueprint(),
        TfListenerModule.blueprint(),
    ).global_config(viewer="none")
    ModuleCoordinator.build(bp).loop()
