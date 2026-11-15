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

"""Real relay fixture for the Node adapter tests, controlled by JSON lines."""

import asyncio
import base64
import json
import sys

from dimos.web.relay_bridge.protocol import Manifest, RobotInfo
from dimos.web.relay_bridge.relay_process import RelayProcess
from dimos.web.relay_bridge.wt_client import RelayClient


async def main() -> None:
    relay = RelayProcess()
    info = relay.start()
    robot: RelayClient | None = None
    try:
        print(json.dumps({"url": info.open_url, "robot": "adapter-test"}), flush=True)
        while line := await asyncio.to_thread(sys.stdin.readline):
            command = json.loads(line)
            if command["kind"] == "manifest":
                manifest = {
                    "version": 1,
                    "channels": command["channels"],
                    "panels": [],
                    "layout": None,
                    "pages": [],
                }
                if robot is None:
                    robot = await RelayClient.connect(info.wt_url, "robot", insecure=True)
                    await robot.hello(
                        robot=RobotInfo(id="adapter-test", name="Adapter test", model="test"),
                        manifest=manifest,
                    )
                else:
                    robot.send_control_frame(Manifest(robotId="adapter-test", manifest=manifest))
            elif command["kind"] == "frame":
                assert robot is not None
                robot.send_frame(
                    command["channel"],
                    base64.b64decode(command["bytes"]),
                    delivery="reliable",
                    meta=command.get("meta"),
                )
            elif command["kind"] == "restart":
                if robot is not None:
                    await robot.close()
                    robot = None
                relay.stop()
                relay = RelayProcess(port=info.http_port)
                info = relay.start()
            elif command["kind"] == "disconnect":
                assert robot is not None
                await robot.close()
                robot = None
            print(json.dumps({"done": True}), flush=True)
    finally:
        if robot is not None:
            await robot.close()
        relay.stop()


asyncio.run(main())
