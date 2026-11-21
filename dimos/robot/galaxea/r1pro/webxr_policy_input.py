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

"""Pair WebXR controller poses and grip authorization in one typed packet."""

import math
import time

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.std_msgs.Header import Header
from dimos.teleop.webxr.controller_types import Hand
from dimos.teleop.webxr.extensions import ArmTeleopModule


class R1ProPolicyWebXR(ArmTeleopModule):
    controller_pair: Out[PoseArray]

    def _publish_button_state(self, left, right):
        now = time.monotonic()
        poses = [self._current_poses.get(hand) for hand in (Hand.LEFT, Hand.RIGHT)]
        stamps = [
            source[hand]
            for source in (self._last_pose_update, self._last_controller_update)
            for hand in (Hand.LEFT, Hand.RIGHT)
        ]
        fresh = [stamp for stamp in stamps if stamp is not None]
        frame, pair = "webxr_idle", []
        if (
            left is not None
            and right is not None
            and len(fresh) == 4
            and all(now - stamp <= self.config.input_timeout_s for stamp in fresh)
            and all(math.isfinite(c.grip) and math.isfinite(c.trigger) for c in (left, right))
        ):
            if left.grip > 0.5 and right.grip > 0.5 and all(p is not None for p in poses):
                frame, pair = "webxr", poses
            elif left.grip <= 0.5 or right.grip <= 0.5:
                frame = "webxr_released"
            super()._publish_button_state(left, right)
        else:
            super()._publish_button_state(None, None)
        stamp = time.time() - (now - min(fresh)) if frame != "webxr_idle" else time.time()
        self.controller_pair.publish(PoseArray(Header(stamp, frame), pair))
