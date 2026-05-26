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

"""Capture the robot's current camera frame and upload it to the robomoo app.

Exposes a `take_picture` skill the agent calls (e.g. "take a picture"); the
latest `color_image` frame is JPEG-encoded and POSTed to robomoo's
`/api/robot/frame` (shared-secret bearer token). Configure via env:

    ROBOMOO_URL=https://gateway-...up.railway.app
    ROBOT_INGEST_TOKEN=<secret matching the server>
"""

import os

import cv2
import httpx

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TakePictureSkillConfig(ModuleConfig):
    robomoo_url: str = os.getenv("ROBOMOO_URL", "")
    ingest_token: str = os.getenv("ROBOT_INGEST_TOKEN", "")


class TakePictureSkill(Module):
    config: TakePictureSkillConfig
    color_image: In[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self._latest: Image | None = None
        self.color_image.subscribe(self._on_image)

    def _on_image(self, image: Image) -> None:
        self._latest = image

    @skill
    def take_picture(self, note: str = "") -> SkillResult:
        """Capture a photo from the robot's camera and upload it.

        Use whenever the user asks the robot to take or capture a picture or
        photo of what it currently sees. `note` is an optional short caption
        to tag the image with (e.g. "kitchen", "plant").
        """
        frame = getattr(self, "_latest", None)
        if frame is None:
            return SkillResult.fail("NO_FRAME", "No camera frame received yet")

        url = self.config.robomoo_url
        token = self.config.ingest_token
        if not url or not token:
            return SkillResult.fail(
                "NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set"
            )

        ok, buf = cv2.imencode(".jpg", frame.data)
        if not ok:
            return SkillResult.fail("EXECUTION_FAILED", "JPEG encode failed")

        try:
            resp = httpx.post(
                f"{url.rstrip('/')}/api/robot/frame",
                headers={"Authorization": f"Bearer {token}"},
                files={"file": ("frame.jpg", buf.tobytes(), "image/jpeg")},
                data={"note": note},
                timeout=30.0,
            )
            resp.raise_for_status()
        except Exception as e:  # noqa: BLE001 — report any upload failure to the agent
            logger.exception("take_picture upload failed")
            return SkillResult.fail("EXECUTION_FAILED", f"upload failed: {e}")

        key = resp.json().get("key", "")
        logger.info("take_picture uploaded frame key=%s", key)
        return SkillResult.ok(f"Picture taken and uploaded ({key})", key=key)
