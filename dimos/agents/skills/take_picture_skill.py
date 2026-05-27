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

"""Capture robot camera frames and upload them to the robomoo app.

Skills:
  - take_picture: one-shot capture of the current frame.
  - explore_and_capture: start autonomous exploration and take a photo every few
    seconds as the robot moves (deterministic — drives the cadence itself rather
    than relying on the agent to keep calling take_picture).

Each frame is JPEG-encoded and POSTed (with the robot's odom pose) to robomoo's
`/api/robot/frame` (shared-secret bearer token). Configure via env:

    ROBOMOO_URL=https://gateway-...up.railway.app
    ROBOT_INGEST_TOKEN=<secret matching the server>
"""

import os
import threading
import time

import cv2
import httpx
from dimos_lcm.std_msgs import Bool

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.frontier_exploration.frontier_explorer_spec import (
    FrontierExplorerSpec,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TakePictureSkillConfig(ModuleConfig):
    robomoo_url: str = os.getenv("ROBOMOO_URL", "")
    ingest_token: str = os.getenv("ROBOT_INGEST_TOKEN", "")


class TakePictureSkill(Module):
    config: TakePictureSkillConfig
    color_image: In[Image]
    odom: In[PoseStamped]
    # Drive frontier exploration (consumed by WavefrontFrontierExplorer).
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    # Auto-wired (structurally) to WavefrontFrontierExplorer — lets us gate the
    # capture loop on whether exploration is still running.
    _explorer: FrontierExplorerSpec

    @rpc
    def start(self) -> None:
        super().start()
        self._latest: Image | None = None
        self._pose: PoseStamped | None = None
        self._capture_thread: threading.Thread | None = None
        self._capture_stop = threading.Event()
        self.color_image.subscribe(self._on_image)
        self.odom.subscribe(self._on_odom)

    @rpc
    def stop(self) -> None:
        self._capture_stop.set()
        thread = getattr(self, "_capture_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        super().stop()

    def _on_image(self, image: Image) -> None:
        self._latest = image

    def _on_odom(self, pose: PoseStamped) -> None:
        self._pose = pose

    def _configured(self) -> bool:
        return bool(self.config.robomoo_url and self.config.ingest_token)

    # Encode the latest frame and POST it (with pose) to robomoo. Returns the
    # stored key, or None if there's no frame / encode failed. Raises on HTTP error.
    def _upload_current(self, note: str = "", label: str = "") -> str | None:
        frame = getattr(self, "_latest", None)
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame.data)
        if not ok:
            return None

        data: dict[str, str] = {}
        if note:
            data["note"] = note
        if label:
            data["label"] = label
        pose = getattr(self, "_pose", None)
        if pose is not None:
            data["poseX"] = str(pose.position.x)
            data["poseY"] = str(pose.position.y)

        resp = httpx.post(
            f"{self.config.robomoo_url.rstrip('/')}/api/robot/frame",
            headers={"Authorization": f"Bearer {self.config.ingest_token}"},
            files={"file": ("frame.jpg", buf.tobytes(), "image/jpeg")},
            data=data,
            timeout=30.0,
        )
        resp.raise_for_status()
        return resp.json().get("key", "")

    @skill
    def take_picture(self, note: str = "") -> SkillResult:
        """Capture a photo from the robot's camera and upload it.

        Use whenever the user asks the robot to take or capture a single picture
        or photo of what it currently sees. `note` is an optional short caption
        to tag the image with (e.g. "kitchen", "plant").
        """
        if getattr(self, "_latest", None) is None:
            return SkillResult.fail("NO_FRAME", "No camera frame received yet")
        if not self._configured():
            return SkillResult.fail(
                "NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set"
            )
        try:
            key = self._upload_current(note=note, label=note)
        except Exception as e:  # noqa: BLE001 — surface upload failure to the agent
            logger.exception("take_picture upload failed")
            return SkillResult.fail("EXECUTION_FAILED", f"upload failed: {e}")
        if key is None:
            return SkillResult.fail("EXECUTION_FAILED", "JPEG encode failed")
        logger.info("take_picture uploaded frame key=%s", key)
        return SkillResult.ok(f"Picture taken and uploaded ({key})", key=key)

    @skill
    def explore_and_capture(
        self,
        interval_s: float = 4.0,
        max_duration_s: float = 600.0,
        note: str = "exploring",
    ) -> SkillResult:
        """Explore the room and keep taking photos until exploration is complete.

        Use when the user asks the robot to explore / wander / look around AND
        take pictures (or capture/photograph) as it goes. Starts autonomous
        frontier exploration, then captures and uploads a frame every
        `interval_s` seconds for as long as the robot is still exploring — it
        stops on its own once the room is fully explored. `max_duration_s` is
        only a safety cap. Returns immediately; capturing runs in the background.
        """
        if not self._configured():
            return SkillResult.fail(
                "NOT_CONFIGURED", "ROBOMOO_URL / ROBOT_INGEST_TOKEN not set"
            )

        # Cancel any in-flight run before starting a new one.
        self._capture_stop.set()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=5.0)
        self._capture_stop = threading.Event()

        self.explore_cmd.publish(Bool(data=True))
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(interval_s, max_duration_s, note),
            daemon=True,
            name="explore-and-capture",
        )
        self._capture_thread.start()

        return SkillResult.ok(
            f"Exploring and capturing a photo every {interval_s:.0f}s until the "
            "room is fully explored."
        )

    def _exploring(self) -> bool:
        try:
            return bool(self._explorer.is_exploration_active())
        except Exception as e:  # noqa: BLE001 — if the ref errors, keep capturing
            logger.warning("is_exploration_active() failed: %s", e)
            return True

    def _capture_loop(self, interval_s: float, max_duration_s: float, note: str) -> None:
        start = time.monotonic()
        grace_s = 6.0  # let exploration spin up before trusting the active flag
        count = 0
        try:
            while not self._capture_stop.is_set():
                elapsed = time.monotonic() - start
                if elapsed > max_duration_s:
                    logger.info("explore_and_capture hit max_duration_s cap")
                    break
                if elapsed > grace_s and not self._exploring():
                    logger.info("exploration finished — stopping capture loop")
                    break
                try:
                    if self._upload_current(note=note, label=note):
                        count += 1
                except Exception as e:  # noqa: BLE001 — keep going on transient errors
                    logger.warning("explore_and_capture upload failed: %s", e)
                self._capture_stop.wait(interval_s)
        finally:
            try:
                self.stop_explore_cmd.publish(Bool(data=True))
            except Exception:  # noqa: BLE001 — best effort on shutdown
                pass
            logger.info("explore_and_capture finished: uploaded %d photos", count)
