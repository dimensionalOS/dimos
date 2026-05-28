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

"""PACK MIND — fast, GPU-free "red object" detector for the live demo.

The demo's target is defined by its COLOUR, so the right detector is a colour
filter on the live camera, not a 4GB VLM. moondream on a CPU ground station is
slow (tens of seconds, async) and fragile; this is a deterministic per-frame red
test that runs in milliseconds and is honest — the dog literally sees red.

On a hit, it reports the finding to the pack coordinator (zone left blank, so the
coordinator fills in the dog's currently-claimed zone). Exposed as the @skill
``look_for_red`` so the operator (or agent) triggers it with one call.
"""

from __future__ import annotations

import os
from typing import Any

import numpy as np
import requests
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.experimental.pack_mind.pack_coordinator_server import DEFAULT_COORDINATOR_PORT
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_RED_THRESHOLD = 0.02  # ≥2% of the frame being red ⇒ a red object is in view
_REQUEST_TIMEOUT = 5.0


def red_fraction(data: np.ndarray) -> float:
    """Fraction of pixels that are convincingly red (RGB, lighting-tolerant ratio test).

    Red = a high red channel that clearly dominates green and blue. Using channel
    *ratios* rather than absolute thresholds keeps it robust across lighting.
    Assumes RGB channel order (DimOS camera frames); if it flags blue, the frame is
    BGR — swap channels 0 and 2.
    """
    if data.ndim != 3 or data.shape[2] < 3:
        return 0.0
    r = data[..., 0].astype(np.int16)
    g = data[..., 1].astype(np.int16)
    b = data[..., 2].astype(np.int16)
    mask = (r > 110) & ((r - g) > 50) & ((r - b) > 50)
    return float(mask.mean())


class RedObjectDetector(Module):
    """Watches the camera; ``look_for_red`` checks the latest frame and, if a red
    object is in view, tells the pack coordinator."""

    color_image: In[Image]

    def __init__(
        self,
        dog_name: str | None = None,
        coordinator_url: str | None = None,
        threshold: float = DEFAULT_RED_THRESHOLD,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._latest: Image | None = None
        self._threshold = threshold
        self._dog = dog_name or os.environ.get("PACK_DOG_NAME") or "alpha"
        self._url = (
            coordinator_url
            or os.environ.get("PACK_COORDINATOR_URL")
            or f"http://127.0.0.1:{DEFAULT_COORDINATOR_PORT}"
        ).rstrip("/")

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))

    def _on_image(self, image: Image) -> None:
        self._latest = image

    @skill
    def look_for_red(self) -> str:
        """Look for a red object in the robot's current camera view. If one is
        visible, report it to the pack's shared memory and say so.
        """
        img = self._latest
        if img is None:
            return "No camera frame yet — is the robot streaming video?"
        frac = red_fraction(np.asarray(img.data))
        if frac < self._threshold:
            return f"No red object in view (red {frac:.1%})."
        try:
            requests.post(
                f"{self._url}/report_finding",
                json={"dog": self._dog, "object": "red object", "zone": ""},
                timeout=_REQUEST_TIMEOUT,
            )
        except requests.RequestException as exc:
            logger.warning("red object seen but coordinator unreachable", error=str(exc))
            return f"I see a red object (red {frac:.0%}) but could not reach the pack memory."
        logger.info("red object detected", dog=self._dog, fraction=round(frac, 3))
        return f"I see a red object (red {frac:.0%}) — reported to the pack."


red_object_detector = RedObjectDetector.blueprint
