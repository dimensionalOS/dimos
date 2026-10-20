# Copyright 2025-2026 Dimensional Inc.
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

"""Actual Python -> C++ -> Rust -> Python coordinator example over local Zenoh."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time

from acme_telemetry.telemetry_msgs.msg import DeviceReading
from reactivex.disposable import Disposable
import zenoh

from dimos.core.cdr_transport import CdrTransport
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule
from dimos.core.stream import In, Out
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.service.zenohservice import ZenohConfig


class Probe(Module):
    raw: Out[DeviceReading]
    done: In[DeviceReading]

    @rpc
    def start(self) -> None:
        super().start()
        self._result: DeviceReading | None = None
        self._received = threading.Event()
        self.register_disposable(Disposable(self.done.subscribe(self._receive)))

    def _receive(self, value: DeviceReading) -> None:
        self._result = value
        self._received.set()

    @rpc
    def send(self) -> None:
        self.raw.publish(DeviceReading(Header(Time(17, 23), "map"), 7, 2.5, "sensor"))

    @rpc
    def exchange(self) -> DeviceReading:
        deadline = time.monotonic() + 20
        # Retry an idempotent request while native subscribers become ready.
        while time.monotonic() < deadline:
            self.send()
            if self._received.wait(0.2):
                assert self._result is not None
                return self._result
        raise TimeoutError("No complete CDR module roundtrip before the deadline")


class CppRelay(NativeModule):
    raw: In[DeviceReading]
    cpp: Out[DeviceReading]


class RustRelay(NativeModule):
    cpp: In[DeviceReading]
    done: Out[DeviceReading]


def run(cpp: Path, rust: Path) -> None:
    # The existing coordinator may offer optional host setup. Its pytest path
    # skips those checks; manual runs must keep its confirmation interactive.
    if not os.environ.get("PYTEST_VERSION") and not sys.stdin.isatty():
        raise RuntimeError(
            "Run this manual demo in an interactive terminal and decline optional host setup"
        )
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
    config = zenoh.Config()
    for key, value in {
        "mode": "router",
        "listen/endpoints": [endpoint],
        "scouting/multicast/enabled": False,
        "scouting/gossip/enabled": False,
    }.items():
        config.insert_json5(key, json.dumps(value))
    with ExitStack() as resources:
        resources.enter_context(zenoh.open(config))
        session = ZenohConfig(
            mode="client",
            connect=[endpoint],
            listen=[],
            scouting=False,
            multicast=False,
            gossip=False,
            connect_timeout=10,
        )
        blueprint = (
            autoconnect(
                Probe.blueprint(),
                CppRelay.blueprint(
                    executable=str(cpp.resolve()), stdin_config=True, session=session
                ),
                RustRelay.blueprint(
                    executable=str(rust.resolve()), stdin_config=True, session=session
                ),
            )
            .transports(
                {
                    (name, DeviceReading): CdrTransport.spec(
                        "example/" + name,
                        DeviceReading,
                        backend="zenoh",
                        **session.model_dump(),
                    )
                    for name in ("raw", "cpp", "done")
                }
            )
            .global_config(
                viewer="none",
                n_workers=3,
                transport="zenoh",
                robot_ip=None,
                robot_ips="",
                zenoh_connect=endpoint,
                zenoh_mode="client",
                zenoh_multicast=False,
                zenoh_gossip=False,
                zenoh_scouting=False,
                zenoh_connect_timeout=10,
            )
        )
        coordinator = ModuleCoordinator.build(blueprint)
        resources.callback(coordinator.stop)
        result = coordinator.get_instance(Probe).exchange()
        assert result == DeviceReading(Header(Time(17, 23), "map"), 7, 4.5, "sensor/cpp/rust")
        assert type(result.header) is Header
        print("PASS: coordinator Python -> C++ -> Rust -> Python over loopback Zenoh CDR")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpp", type=Path, default=Path("build/cpp/telemetry_module"))
    parser.add_argument("--rust", type=Path, default=Path("modules/target/debug/telemetry_module"))
    args = parser.parse_args()
    run(args.cpp, args.rust)
