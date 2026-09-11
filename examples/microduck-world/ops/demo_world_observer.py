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

"""Full-state read-only protocol observer for manual browser checkpoints."""

import asyncio
import json
import threading

import requests
from dimos.web.relay_bridge.protocol import Sub, Watch
from dimos.web.relay_bridge.wt_client import RelayClient
from microduck_world.relay import relay_settings


class WorldObserver:
    def __init__(self):
        self.latest = None
        self.phase = "starting"
        self.error = None
        self.ready = threading.Event()
        self.loop = None
        self.task = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self):
        self.thread.start()
        if not self.ready.wait(15) or self.error is not None:
            self.__exit__(None, None, None)
            raise RuntimeError(f"World observer failed: {self.phase}, {self.error}")
        return self

    def __exit__(self, *args):
        if self.loop is not None and self.task is not None and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self.task.cancel)
        self.thread.join(timeout=5)

    def _run(self):
        try:
            asyncio.run(self._observe())
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.error = type(exc).__name__
            self.ready.set()

    async def _observe(self):
        self.loop = asyncio.get_running_loop()
        self.task = asyncio.current_task()
        with requests.Session() as http:
            http.trust_env = False
            base = relay_settings().upstream_url
            p = http.post(base + "/api/lobby", json={"action": "create"}, timeout=3).json()
            info = http.get(base + "/sessions/" + p["token"] + "/api/info", timeout=3).json()
        async with await RelayClient.connect(info["wtUrl"], "viewer") as client:
            self.phase = "hello"
            await client.hello()
            self.phase = "frames"

            # Python viewers use datagram replies, too small for this cockpit's
            # full manifest. State streaming itself uses reliable data carriers.
            # Heal unordered watch/sub datagrams until the first frame arrives.
            async def subscribe():
                while self.latest is None:
                    client.send_control(Watch(robotId="world"))
                    await client.ping()
                    client.send_control(Sub(ch="world_state"))
                    await asyncio.sleep(0.5)

            subscribing = asyncio.create_task(subscribe())
            try:
                async for frame in client.frames():
                    if frame.header.ch == "world_state":
                        self.latest = json.loads(frame.payload)
                        self.ready.set()
            finally:
                subscribing.cancel()
