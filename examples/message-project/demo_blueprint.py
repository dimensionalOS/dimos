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

"""Run the documented custom message through actual Python/C++/Rust workers."""

import argparse
from contextlib import ExitStack
from functools import partial
from pathlib import Path
import socket
import threading
from unittest.mock import patch
import uuid

from demo_modules import ReadingProcessor
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import encode as cdr_encode
from story_messages.story_msgs.msg import DeviceReading

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule
from dimos.core.stream import In, Out
from dimos.protocol.service.system_configurator.base import configure_system
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


class Exchange(Module):
    raw: Out[DeviceReading]
    checked: In[DeviceReading]

    @rpc
    def start(self) -> None:
        self.received: DeviceReading | None = None
        self.done = threading.Event()
        super().start()

    async def handle_checked(self, message: DeviceReading) -> None:
        self.received = message
        self.done.set()

    @rpc
    def exchange(self) -> str:
        message = DeviceReading(
            header=Header(stamp=Time(sec=17, nanosec=123456789), frame_id="sensor"),
            sequence=42,
            value=20.5,
            label="sensor",
        )
        for _ in range(60):
            self.raw.publish(message)
            if self.done.wait(0.25):
                break
        if self.received is None:
            raise TimeoutError("No reply from the three-language module chain")
        expected = DeviceReading(header=message.header, sequence=42, value=23.5, label="sensor")
        if cdr_encode(self.received) != cdr_encode(expected):
            raise ValueError("Received fields differ from the expected three module edits")
        return "PASS: 20.5 -> Python 21.5 -> C++ 22.5 -> Rust 23.5; Header and sequence preserved"


class CppProcessor(NativeModule):
    reading: In[DeviceReading]
    processed: Out[DeviceReading]


class RustProcessor(NativeModule):
    processed: In[DeviceReading]
    checked: Out[DeviceReading]


def blueprint(cpp: Path, rust: Path) -> Blueprint:
    return autoconnect(
        Exchange.blueprint(),
        ReadingProcessor.blueprint(),
        CppProcessor.blueprint(executable=str(cpp.resolve()), stdin_config=True),
        RustProcessor.blueprint(executable=str(rust.resolve()), stdin_config=True),
    ).namespace("reading" + uuid.uuid4().hex[:8])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["zenoh", "lcm"], default="zenoh")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    cpp, rust = root / "build/cpp/processor", root / "rust/target/debug/reading-processor"
    if not cpp.is_file() or not rust.is_file():
        parser.error("Build both native processors before running this example")
    with ExitStack() as stack:
        # This finite offline example can report tuning needs but must never apply them.
        stack.enter_context(
            patch(
                "dimos.protocol.service.system_configurator.base.configure_system",
                partial(configure_system, check_only=True),
            )
        )
        configured = blueprint(cpp, rust).global_config(viewer="none", transport=args.transport)
        if args.transport == "zenoh":
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
            pool = ZenohSessionPool()
            stack.callback(pool.close_all)
            pool.acquire(
                ZenohConfig(
                    mode="router", listen=[endpoint], connect=[], multicast=False, gossip=False
                )
            )
            configured = configured.global_config(
                zenoh_mode="client", zenoh_connect=endpoint, zenoh_multicast=False
            )
        coordinator = ModuleCoordinator.build(configured)
        stack.callback(coordinator.stop)
        print(coordinator.get_instance(Exchange).exchange(), flush=True)


if __name__ == "__main__":
    main()
