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

"""Deliver a generated camera frame over LCM and save the MCP observe image."""

import asyncio
import base64
from pathlib import Path
from threading import Event, Thread
from uuid import uuid4

from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.agents.mcp.mcp_server import handle_request
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.transport import LCMTransport
from dimos.msgs.image import image_from_array


def main() -> None:
    module = ObserveSkill()
    module.color_image.transport = LCMTransport(f"/observe/{uuid4().hex[:8]}", Image)
    pixels = np.zeros((120, 160, 3), dtype=np.uint8)
    pixels[:, :80, 0] = 255
    pixels[:, 80:, 1] = 255
    frame = image_from_array(pixels, encoding="rgb8")
    stop = Event()

    def publish() -> None:
        while not stop.is_set():
            module.color_image.transport.publish(frame)
            stop.wait(0.05)

    thread = Thread(target=publish, daemon=True)
    try:
        thread.start()
        response = asyncio.run(
            handle_request(
                {"method": "tools/call", "id": 1, "params": {"name": "observe"}},
                [],
                {"observe": module.observe},
            )
        )
        assert response is not None
        image = response["result"]["content"][0]
        assert image["type"] == "image" and image["mimeType"] == "image/jpeg"
        destination = Path("build/message-codegen/demo/evidence/mcp-observe.jpg")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(base64.b64decode(image["data"]))
        print(f"LCM CDR camera → observe skill → MCP JPEG image: {destination}")
        print("Expected image: red left half, green right half, 160 x 120 pixels")
        print("Real LCM delivery; MCP handler invoked directly, no model inference.")
    finally:
        stop.set()
        thread.join(timeout=3)
        module.stop()


if __name__ == "__main__":
    main()
