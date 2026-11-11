"""Robot-independent mapping of WebXR Joy sticks to base and lift velocities."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from pydantic import Field
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32


class JoystickTeleopConfig(ModuleConfig):
    base_linear_speed: float = Field(default=0.3, ge=0, allow_inf_nan=False)
    base_angular_speed: float = Field(default=0.5, ge=0, allow_inf_nan=False)
    lift_speed: float = Field(default=0.03, ge=0, allow_inf_nan=False)
    deadzone: float = Field(default=0.1, ge=0, lt=1, allow_inf_nan=False)
    input_timeout_s: float = Field(default=0.2, gt=0, allow_inf_nan=False)
    enable_button: int | None = Field(default=3, ge=0)


class JoystickTeleopMapper(Module):
    """Left -Y/-X drive X/Y; right -X/-Y drive yaw/lift.

    Joy axes are x, y, trigger, grip. The right thumbstick button is a
    hold-to-enable clutch (None disables it). Invalid input clears that hand.
    Source age uses wall time; hand expiry uses only monotonic receive time.
    """

    config: JoystickTeleopConfig
    left_joy: In[Joy]
    right_joy: In[Joy]
    base_twist: Out[Twist]
    lift_rate: Out[Float32]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._running = False
        self._hands: dict[str, tuple[float, float, float, bool]] = {}

    @rpc
    def start(self) -> None:
        super().start()
        with self._lock:
            if self._running:
                return
            self._hands.clear()
            self._running = True
        self.register_disposable(Disposable(self.left_joy.subscribe(self._on_left_joy)))
        self.register_disposable(Disposable(self.right_joy.subscribe(self._on_right_joy)))
        self.register_disposable(
            rx.interval(min(0.05, self.config.input_timeout_s / 2)).subscribe(
                lambda _: self._watchdog()
            )
        )

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._running = False
            self._hands.clear()
            self.base_twist.publish(Twist())
            self.lift_rate.publish(Float32(data=0.0))
        super().stop()

    def _on_left_joy(self, joy: Joy) -> None:
        self._receive("left", joy)

    def _on_right_joy(self, joy: Joy) -> None:
        self._receive("right", joy)

    def _receive(self, hand: str, joy: Joy) -> None:
        with self._lock:
            if not self._running:
                return
            self._hands.pop(hand, None)
            try:
                valid = (
                    isinstance(joy, Joy)
                    and len(joy.axes) >= 2
                    and all(math.isfinite(a) and -1 <= a <= 1 for a in joy.axes)
                    and math.isfinite(joy.ts)
                    and -0.1 <= time.time() - joy.ts <= self.config.input_timeout_s
                )
                button = self.config.enable_button
                enabled = button is None or (len(joy.buttons) > button and joy.buttons[button] == 1)
                if valid:
                    x, y = (a if abs(a) > self.config.deadzone else 0.0 for a in joy.axes[:2])
                    self._hands[hand] = (x, y, time.monotonic(), enabled)
            except (AttributeError, TypeError, ValueError, OverflowError):
                pass
            self._publish()

    def _watchdog(self) -> None:
        with self._lock:
            if not self._running:
                return
            now = time.monotonic()
            expired = [
                hand
                for hand, state in self._hands.items()
                if now - state[2] > self.config.input_timeout_s
            ]
            for hand in expired:
                del self._hands[hand]
            # A Twist cannot zero one axis without reissuing the others. On
            # expiry, stop the shared base output; only new Joy may resume it.
            if expired:
                self.base_twist.publish(Twist())
            if "right" in expired:
                self.lift_rate.publish(Float32(data=0.0))

    def _publish(self) -> None:
        # Serialize state and publication: an older callback must never publish
        # motion after a concurrent clutch release or stop has published zero.
        with self._lock:
            if not self._running:
                return
            now = time.monotonic()
            fresh = {
                hand: state
                for hand, state in self._hands.items()
                if now - state[2] <= self.config.input_timeout_s
            }
            left = fresh.get("left", (0.0, 0.0, 0.0, False))
            right = fresh.get("right", (0.0, 0.0, 0.0, False))
            enabled = self.config.enable_button is None or right[3]
            vx = -left[1] * self.config.base_linear_speed if enabled else 0.0
            vy = -left[0] * self.config.base_linear_speed if enabled else 0.0
            yaw = -right[0] * self.config.base_angular_speed if enabled else 0.0
            lift = -right[1] * self.config.lift_speed if enabled else 0.0
            self.base_twist.publish(Twist(linear=(vx, vy, 0.0), angular=(0.0, 0.0, yaw)))
            self.lift_rate.publish(Float32(data=lift))
