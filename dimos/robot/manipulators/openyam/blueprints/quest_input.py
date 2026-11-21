"""Task-local deadman interlock; synthetic neutral is never a physical release."""

import time

from dimos.teleop.webxr.controller_types import Hand
from dimos.teleop.webxr.extensions import ArmTeleopModule

# Reserved bit, ignored by normal WebXR button edge/gripper routing.
RIGHT_GRIP_OBSERVED = 1 << 15


class QuestArmTeleopModule(ArmTeleopModule):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._release_required = {hand: True for hand in Hand}
        self._right_grip_observed = False

    def _reset_controller_state(self):
        self._release_required = {hand: True for hand in Hand}
        super()._reset_controller_state()

    def _expire_stale_state(self, now):
        for hand in Hand:
            stamps = (self._last_pose_update[hand], self._last_controller_update[hand])
            if any(stamp is None or now - stamp > self.config.input_timeout_s for stamp in stamps):
                self._release_required[hand] = True
        super()._expire_stale_state(now)

    def _publish_button_state(self, left, right):
        now = time.monotonic()
        safe = []
        for hand, controller in ((Hand.LEFT, left), (Hand.RIGHT, right)):
            stamps = (self._last_pose_update[hand], self._last_controller_update[hand])
            fresh = all(
                stamp is not None and now - stamp <= self.config.input_timeout_s for stamp in stamps
            )
            # Only an actual fresh controller packet with released grip unlocks.
            # None comes from expiry/disconnect and cannot authorize re-engage.
            if fresh and controller is not None and controller.grip <= 0.5:
                self._release_required[hand] = False
            safe.append(controller if fresh and not self._release_required[hand] else None)
        self._right_grip_observed = safe[1] is not None
        super()._publish_button_state(*safe)

    def _publish_buttons(self, buttons):
        if self._right_grip_observed:
            buttons.data |= RIGHT_GRIP_OBSERVED
        super()._publish_buttons(buttons)
