from __future__ import annotations

from dataclasses import dataclass
import time
from threading import Condition, Event, Lock, Thread
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

from dimos.control.safety.guardrail_policy import GuardrailDecision, GuardrailState

_DEFAULT_FALLBACK_PERIOD_S = 0.1
_THREAD_JOIN_TIMEOUT_S = 2.0

logger = setup_logger()


class RGBCollisionGuardrailConfig(ModuleConfig):
    # TODO: Step 2
    guarded_output_publish_hz: float = 10.0
    risk_evaluation_hz: float = 10.0
    command_timeout_s: float = 0.25
    image_timeout_s: float = 0.25
    risk_timeout_s: float = 0.25
    fail_closed_on_missing_image: bool = True
    publish_zero_on_stop: bool = True


@dataclass
class _GuardrailRuntimeState:
    # TODO: Step 2
    latest_image: Image | None = None
    previous_image: Image | None = None
    latest_image_time: float | None = None
    previous_image_time: float | None = None
    latest_cmd_vel: Twist | None = None
    latest_cmd_time: float | None = None
    last_decision: GuardrailDecision | None = None
    last_risk_time: float | None = None
    state: GuardrailState = GuardrailState.INIT


class RGBCollisionGuardrail(Module[RGBCollisionGuardrailConfig]):
    """RGB-only motion guardrail for direct Twist control."""

    default_config = RGBCollisionGuardrailConfig

    color_image: In[Image]
    incoming_cmd_vel: In[Twist]
    safe_cmd_vel: Out[Twist]

    _condition: Condition
    _runtime_lock: Lock
    _runtime_state: _GuardrailRuntimeState
    _stop_event: Event
    _thread: Thread | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime_lock = Lock()
        self._condition = Condition(self._runtime_lock)
        self._runtime_state = _GuardrailRuntimeState()
        self._stop_event = Event()
        self._thread = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._disposables.add(Disposable(self.color_image.subscribe(self._on_color_image)))
        self._disposables.add(Disposable(self.incoming_cmd_vel.subscribe(self._on_incoming_cmd_vel)))

        self._thread = Thread(
            target=self._decision_loop,
            name=f"{self.__class__.__name__}-thread",
            daemon=True,
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        if self.config.publish_zero_on_stop:
            self.safe_cmd_vel.publish(Twist.zero())

        if self._thread is not None:
            self._thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
            self._thread = None

        super().stop()

    def _on_color_image(self, image: Image) -> None:
        now = time.monotonic()
        with self._condition:
            self._runtime_state.previous_image = self._runtime_state.latest_image
            self._runtime_state.previous_image_time = self._runtime_state.latest_image_time
            self._runtime_state.latest_image = image
            self._runtime_state.latest_image_time = now
            self._condition.notify()

    def _on_incoming_cmd_vel(self, cmd_vel: Twist) -> None:
        now = time.monotonic()
        with self._condition:
            self._runtime_state.latest_cmd_vel = cmd_vel
            self._runtime_state.latest_cmd_time = now
            self._condition.notify()

    def _decision_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._condition:
                timeout_s = self._next_wakeup_timeout_locked()
                self._condition.wait(timeout=timeout_s)

                if self._stop_event.is_set():
                    return

                # Step 2 will add the timed wakeup and per-command evaluation.
                continue

    def _guarded_output_publish_period_s(self) -> float:
        if self.config.guarded_output_publish_hz <= 0:
            return _DEFAULT_FALLBACK_PERIOD_S
        return 1.0 / self.config.guarded_output_publish_hz

    def _risk_evaluation_period_s(self) -> float:
        if self.config.risk_evaluation_hz <= 0:
            return _DEFAULT_FALLBACK_PERIOD_S
        return 1.0 / self.config.risk_evaluation_hz

    def _next_wakeup_timeout_locked(self) -> float:
        return min(
            self._guarded_output_publish_period_s(),
            self._risk_evaluation_period_s(),
        )


rgb_collision_guardrail = RGBCollisionGuardrail.blueprint
