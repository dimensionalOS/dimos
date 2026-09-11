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

"""Live health reporting for the PICO WebXR body-tracking demo."""

from time import monotonic
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

REPORT_INTERVAL_S = 5.0


def body_tracking_summary(
    snapshot: BodyTrackingSnapshot,
    *,
    snapshot_rate_hz: float,
    resolved_joint_ever_seen: bool,
) -> dict[str, Any]:
    """Build one compact body-tracking health summary."""
    joints = snapshot.joints
    state = "unavailable" if joints is None else "empty" if not joints else "tracking"
    positions: dict[str, tuple[float, float, float]] = {}
    if joints:
        positions = {
            name: (
                round(pose.position[0], 3),
                round(pose.position[1], 3),
                round(pose.position[2], 3),
            )
            for name, pose in joints.items()
        }

    return {
        "snapshot_rate_hz": round(snapshot_rate_hz, 1),
        "state": state,
        "reference_space": snapshot.frame_id,
        "resolved_joint_count": 0 if joints is None else len(joints),
        "resolved_joint_ever_seen": resolved_joint_ever_seen,
        "joint_positions": positions,
    }


class BodyTrackingMonitor(Module):
    """Report live PICO body-tracking availability, rate, and joint poses."""

    body_tracking: In[BodyTrackingSnapshot]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._report_started_at = monotonic()
        self._snapshots_since_report = 0
        self._resolved_joint_ever_seen = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.body_tracking.subscribe(self._on_body_tracking)))

    def _on_body_tracking(self, snapshot: BodyTrackingSnapshot) -> None:
        self._snapshots_since_report += 1
        if snapshot.joints and not self._resolved_joint_ever_seen:
            self._resolved_joint_ever_seen = True
            logger.info(
                "WebXR body tracking acquired",
                reference_space=snapshot.frame_id,
                resolved_joint_count=len(snapshot.joints),
            )

        now = monotonic()
        elapsed = now - self._report_started_at
        if elapsed < REPORT_INTERVAL_S:
            return

        summary = body_tracking_summary(
            snapshot,
            snapshot_rate_hz=self._snapshots_since_report / elapsed,
            resolved_joint_ever_seen=self._resolved_joint_ever_seen,
        )
        if snapshot.joints:
            logger.info("WebXR body tracking health", **summary)
        else:
            logger.warning("WebXR body tracking has no resolved joints", **summary)
        self._report_started_at = now
        self._snapshots_since_report = 0
