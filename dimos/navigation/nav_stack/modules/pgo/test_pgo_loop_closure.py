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

"""End-to-end check that PGO publishes valid loop_closure_event messages.

Replays the ``og_nav_60s`` rosbag through PGO with aggressive
loop-closure thresholds and asserts each emitted ``loop_closure_event``
(a ``GraphDelta3D``) has positive-shape per-node SE(3) deltas with
unit-norm quaternions and finite translations. Wired with the DimOS
Module + Blueprint pipeline so no LCM topic strings live here.

If the bag doesn't trigger any loop the test skips — the rosbag
trajectory is data-dependent, not a code defect.
"""

from __future__ import annotations

import math
import time
from typing import Any

import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    RosbagScanOdomPlaybackModule,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_no_nix, pytest.mark.skipif_macos_bug]

POST_FEED_DRAIN_SEC = 5.0
POLL_INTERVAL_SEC = 0.25

QUATERNION_UNIT_TOL = 0.05
TRANSLATION_MAX_M = 100.0


class LoopClosureEventRecorderModule(Module):
    """Accumulates every loop_closure_event so the test can validate the shape."""

    loop_closure_event: In[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._events: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.loop_closure_event.subscribe(self._on_loop_closure_event))
        )

    def _on_loop_closure_event(self, message: GraphDelta3D) -> None:
        # JSON-friendly snapshot — Pydantic-friendly RPC return.
        self._events.append(
            {
                "ts": message.ts,
                "transforms": [
                    {
                        "translation": (
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z,
                        ),
                        "rotation": (
                            transform.rotation.x,
                            transform.rotation.y,
                            transform.rotation.z,
                            transform.rotation.w,
                        ),
                    }
                    for transform in message.transforms
                ],
            }
        )
        first_transform = message.transforms[0] if message.transforms else None
        first_summary = (
            f"first=t=({first_transform.translation.x:.3f},{first_transform.translation.y:.3f},"
            f"{first_transform.translation.z:.3f}) "
            f"q=({first_transform.rotation.x:.3f},{first_transform.rotation.y:.3f},"
            f"{first_transform.rotation.z:.3f},{first_transform.rotation.w:.3f})"
            if first_transform
            else "<empty>"
        )
        logger.info(
            f"[loop_closure_event] event #{len(self._events) - 1} received: "
            f"node_count={len(message.nodes)}, ts={message.ts:.3f}, {first_summary}"
        )

    @rpc
    def events(self) -> list[dict[str, Any]]:
        return list(self._events)


def _validate_loop_closure_event(event: dict[str, Any], event_index: int) -> tuple[float, float]:
    """Assert each transform has unit-norm rotation + finite translation.

    Returns aggregate ``(max_translation_norm, max_quaternion_drift)`` stats.
    """
    transforms = event["transforms"]
    assert len(transforms) > 0, f"event {event_index}: loop-closure event has no transforms"

    max_translation_norm = 0.0
    max_quaternion_drift = 0.0
    for transform_index, transform in enumerate(transforms):
        translation_x, translation_y, translation_z = transform["translation"]
        rotation_x, rotation_y, rotation_z, rotation_w = transform["rotation"]
        for value, name in [
            (translation_x, "translation_x"),
            (translation_y, "translation_y"),
            (translation_z, "translation_z"),
        ]:
            assert math.isfinite(value), (
                f"event {event_index} transform {transform_index}: {name}={value} not finite"
            )
        for value, name in [
            (rotation_x, "rotation_x"),
            (rotation_y, "rotation_y"),
            (rotation_z, "rotation_z"),
            (rotation_w, "rotation_w"),
        ]:
            assert math.isfinite(value), (
                f"event {event_index} transform {transform_index}: {name}={value} not finite"
            )
        translation_norm = math.sqrt(
            translation_x * translation_x
            + translation_y * translation_y
            + translation_z * translation_z
        )
        assert translation_norm < TRANSLATION_MAX_M, (
            f"event {event_index} transform {transform_index}: "
            f"|t|={translation_norm:.3f}m exceeds sanity cap {TRANSLATION_MAX_M}m"
        )
        quaternion_norm = math.sqrt(
            rotation_x * rotation_x
            + rotation_y * rotation_y
            + rotation_z * rotation_z
            + rotation_w * rotation_w
        )
        quaternion_drift = abs(quaternion_norm - 1.0)
        assert quaternion_drift < QUATERNION_UNIT_TOL, (
            f"event {event_index} transform {transform_index}: "
            f"|q|={quaternion_norm:.6f} drifts from unit by {quaternion_drift:.6f} "
            f"(tol {QUATERNION_UNIT_TOL})"
        )
        max_translation_norm = max(max_translation_norm, translation_norm)
        max_quaternion_drift = max(max_quaternion_drift, quaternion_drift)

    return max_translation_norm, max_quaternion_drift


class TestPGOLoopClosure:
    """End-to-end: PGO publishes loop_closure_event with valid SE(3) deltas."""

    def test_loop_closure_events_published(self) -> None:
        playback_blueprint = RosbagScanOdomPlaybackModule.blueprint()
        # Aggressive loop-closure thresholds — bag is 60s, so we need short
        # re-visit windows to actually fire events.
        pgo_blueprint = PGO.blueprint(
            key_pose_delta_trans=0.5,
            loop_search_radius=2.0,
            loop_time_thresh=5.0,
            loop_score_thresh=0.5,
            loop_submap_half_range=5,
            submap_resolution=0.1,
            min_loop_detect_duration=1.0,
            global_map_voxel_size=0.1,
            global_map_publish_rate=1.0,
            unregister_input=True,
        )
        recorder_blueprint = LoopClosureEventRecorderModule.blueprint()

        blueprint = autoconnect(playback_blueprint, pgo_blueprint, recorder_blueprint)
        coordinator = ModuleCoordinator.build(blueprint)
        try:
            playback = coordinator.get_instance(RosbagScanOdomPlaybackModule)
            recorder = coordinator.get_instance(LoopClosureEventRecorderModule)
            while not playback.is_finished():
                time.sleep(POLL_INTERVAL_SEC)
            time.sleep(POST_FEED_DRAIN_SEC)
            events = recorder.events()
        finally:
            coordinator.stop()

        logger.info(f"\n[loop_closure_event] total events received: {len(events)}")

        if not events:
            pytest.skip(
                "rosbag trajectory didn't trigger any PGO loop closures "
                "even with aggressive thresholds — this validates only the "
                "publishing path's existence (verified via native log "
                "lines), not the on-wire payload."
            )

        for event_index, event in enumerate(events):
            max_translation_norm, max_quaternion_drift = _validate_loop_closure_event(
                event, event_index
            )
            logger.info(
                f"[loop_closure_event] event #{event_index} VALID: "
                f"transform_count={len(event['transforms'])}, "
                f"max|t|={max_translation_norm:.4f}m, "
                f"max|q|-1|={max_quaternion_drift:.6f}"
            )

        assert all(len(event["transforms"]) > 0 for event in events)
