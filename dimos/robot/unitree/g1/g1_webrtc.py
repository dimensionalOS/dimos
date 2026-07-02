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

"""G1 WebRTC connection — G1's own move/posture commands (kept G1-local; FSMs may diverge)."""

import asyncio
import time

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.unitree_webrtc import UnitreeWebRTCConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1WebRTCConnection(UnitreeWebRTCConnection):
    """G1 high-level WebRTC connection — G1's own move + posture commands."""

    def __init__(self, ip: str, mode: str = "ai", aes_128_key: str | None = None) -> None:
        self._stop_handle: asyncio.TimerHandle | None = None
        self.cmd_vel_timeout = 0.2
        super().__init__(ip, mode=mode, aes_128_key=aes_128_key)

    def _publish_joystick(self, x: float, y: float, yaw: float) -> None:
        """Wireless-controller joystick send (yaw is a joystick axis, not rad/s)."""
        self.publish(RTC_TOPIC["WIRELESS_CONTROLLER"], {"lx": -y, "ly": x, "rx": -yaw, "ry": 0})

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a twist as joystick axes. duration=0 sends once; >0 resends until elapsed."""
        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z
        self._arm_auto_stop()
        try:
            if duration > 0:
                start_time = time.time()
                while time.time() - start_time < duration:
                    self._publish_joystick(x, y, yaw)
                    time.sleep(0.01)
                self._publish_joystick(0.0, 0.0, 0.0)
            else:
                self._publish_joystick(x, y, yaw)
            return True
        except Exception as e:
            logger.warning("Failed to send movement command: %s", e)
            return False

    def _arm_auto_stop(self) -> None:
        """(Re)arm the command-timeout auto-stop on the event loop (no per-call threads)."""

        def _rearm() -> None:  # runs on the loop thread
            if self._stop_handle is not None:
                self._stop_handle.cancel()
            self._stop_handle = self.loop.call_later(self.cmd_vel_timeout, self._auto_stop)

        self.loop.call_soon_threadsafe(_rearm)

    def _auto_stop(self) -> None:
        """Zero the robot when commands stop arriving (runs on the loop thread)."""
        self._stop_handle = None
        try:
            self._publish_joystick(0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning("Auto-stop send failed: %s", e)

    def stop(self) -> None:
        # Safety: zero the robot before the base tears down the loop. Loop teardown
        # also drops any pending auto-stop, so nothing can fire afterwards.
        try:
            self._publish_joystick(0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning("Failed to send stop on disconnect: %s", e)
        super().stop()

    def standup(self) -> bool:
        return bool(self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]}))

    def liedown(self) -> bool:
        return bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})
        )
