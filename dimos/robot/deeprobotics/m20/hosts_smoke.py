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

"""Smoke test of the M20's AOS and NOS Hosts: Ping on AOS -> Echo on NOS -> back to this root.

Needs no `dimos run` and no local Host: this process is the blueprint root, a zenoh client of
one board's Host router, and reads the run's streams itself. Run on GOS from a checkout of the
boards' revision:

    python -c "from dimos.robot.deeprobotics.m20.hosts_smoke import main; main()"
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import suppress
import os
import socket
import threading
import uuid

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hosted.client import root_rpc
from dimos.hosted.deploy import deployed, wait_for_hosts
from dimos.msgs.std_msgs.String import String


class PingConfig(ModuleConfig):
    period_seconds: float = 0.5


class Ping(Module):
    config: PingConfig
    pings: Out[String]

    async def main(self) -> AsyncIterator[None]:
        task = asyncio.create_task(self._publish())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task

    async def _publish(self) -> None:
        count = 0
        while True:
            count += 1
            self.pings.publish(
                String(f"ping {count} from {socket.gethostname()} pid {os.getpid()}")
            )
            await asyncio.sleep(self.config.period_seconds)


class Echo(Module):
    pings: In[String]
    echoes: Out[String]

    async def handle_pings(self, message: String) -> None:
        self.echoes.publish(String(f"echo by pid {os.getpid()}: {message.data}"))


def main(connect: str = "tcp/10.21.31.103:7447", wanted: int = 3, timeout: float = 120.0) -> None:
    app = (
        autoconnect(
            Ping.blueprint().hosted(tags={"board": "aos"}),
            Echo.blueprint().hosted(tags={"board": "nos"}),
        )
        .solidify()
        .global_config(n_workers=1)
    )
    run_id = f"m20-hosts-smoke-{uuid.uuid4().hex[:6]}"
    received: list[str] = []
    done = threading.Event()

    def on_sample(sample: object) -> None:
        key = str(sample.key_expr)  # type: ignore[attr-defined]
        if "echoes" not in key:
            return
        payload = sample.payload.to_bytes()  # type: ignore[attr-defined]
        try:
            text = String.lcm_decode(payload).data
        except Exception:
            text = repr(payload[-80:])
        received.append(f"{key}: {text}")
        print(f"root got {key}: {text}", flush=True)
        if len(received) >= wanted:
            done.set()

    with root_rpc([connect]) as rpc:
        descriptors = wait_for_hosts(rpc, set(), tag_sets=[{"board": "aos"}, {"board": "nos"}])
        for d in descriptors:
            print(f"host {d.name} {d.revision[:18]} {d.tags}", flush=True)
        subscriber = rpc.session.declare_subscriber(f"dimos-domains/{run_id}/**", on_sample)
        try:
            with deployed(
                app,
                rpc,
                run_id=run_id,
                local_host_id=None,
                descriptors=descriptors,
                on_log=lambda host, line: print(f"{host:>6} | {line}", flush=True),
            ) as placement:
                print(f"placement {placement}", flush=True)
                if not done.wait(timeout):
                    raise TimeoutError(f"only {len(received)} echoes in {timeout}s")
        finally:
            subscriber.undeclare()
    print(f"OK: {len(received)} echoes came back; run {run_id} stopped", flush=True)
