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

from __future__ import annotations

from collections.abc import Callable
import queue
import threading
import time
from typing import Any, TypeVar

import numpy as np
import pytest

from dimos.control.safety.guardrail_policy import (
    GuardrailDecision,
    GuardrailHealth,
    GuardrailState,
)
from dimos.control.safety.rgb_collision_guardrail import RGBCollisionGuardrail
from dimos.core.stream import Out, Transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

T = TypeVar("T")


class FakeTransport(Transport[T]):
    def __init__(self) -> None:
        self._subscribers: list[Callable[[T], Any]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Out[T] | None, value: T) -> None:
        for callback in list(self._subscribers):
            callback(value)

    def subscribe(
        self,
        callback: Callable[[T], Any],
        selfstream=None,
    ) -> Callable[[], None]:
        self._subscribers.append(callback)

        def unsubscribe() -> None:
            self._subscribers.remove(callback)

        return unsubscribe


class SequencePolicy:
    def __init__(self, decisions: list[GuardrailDecision]) -> None:
        self._decisions = decisions
        self._index = 0

    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        if self._index < len(self._decisions):
            decision = self._decisions[self._index]
            self._index += 1
            return decision
        return self._decisions[-1]


def _textured_gray_image(*, width: int = 160, height: int = 120, shift_x: int = 0) -> Image:
    yy, xx = np.indices((height, width))
    pattern = ((xx * 5 + yy * 9 + shift_x * 17) % 256).astype(np.uint8)
    return Image.from_numpy(pattern, format=ImageFormat.GRAY)


def _black_gray_image(*, width: int = 160, height: int = 120) -> Image:
    return Image.from_numpy(
        np.zeros((height, width), dtype=np.uint8),
        format=ImageFormat.GRAY,
    )


def _cmd(
    x: float = 0.35,
    *,
    linear_y: float = 0.0,
    angular_z: float = 0.25,
) -> Twist:
    return Twist(
        linear=[x, linear_y, 0.0],
        angular=[0.0, 0.0, angular_z],
    )


def _decision(
    state: GuardrailState,
    cmd_vel: Twist,
    *,
    reason: str = "test",
    publish_immediately: bool = False,
) -> GuardrailDecision:
    return GuardrailDecision(
        state=state,
        cmd_vel=cmd_vel,
        reason=reason,
        publish_immediately=publish_immediately,
    )


def _start_guardrail(
    **config_overrides: float,
) -> tuple[
    RGBCollisionGuardrail,
    FakeTransport[Image],
    FakeTransport[Twist],
    queue.Queue[tuple[float, Twist]],
]:
    config: dict[str, float] = {
        "guarded_output_publish_hz": 20.0,
        "risk_evaluation_hz": 20.0,
        "command_timeout_s": 0.3,
        "image_timeout_s": 0.3,
        "risk_timeout_s": 0.3,
    }
    config.update(config_overrides)

    guardrail = RGBCollisionGuardrail(**config)
    image_transport: FakeTransport[Image] = FakeTransport()
    cmd_transport: FakeTransport[Twist] = FakeTransport()
    outputs: queue.Queue[tuple[float, Twist]] = queue.Queue()

    guardrail.color_image.transport = image_transport
    guardrail.incoming_cmd_vel.transport = cmd_transport
    guardrail.safe_cmd_vel.subscribe(lambda msg: outputs.put((time.monotonic(), msg)))
    guardrail.start()

    return guardrail, image_transport, cmd_transport, outputs


@pytest.fixture
def started_guardrail() -> tuple[
    RGBCollisionGuardrail,
    FakeTransport[Image],
    FakeTransport[Twist],
    queue.Queue[tuple[float, Twist]],
]:
    guardrail, image_transport, cmd_transport, outputs = _start_guardrail()

    yield guardrail, image_transport, cmd_transport, outputs

    guardrail.stop()


def _wait_for_output(
    outputs: queue.Queue[tuple[float, Twist]],
    predicate: Callable[[Twist], bool],
    *,
    timeout_s: float = 1.0,
) -> tuple[float, Twist]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            ts, msg = outputs.get(timeout=max(remaining, 0.01))
        except queue.Empty:
            continue
        if predicate(msg):
            return ts, msg
    raise AssertionError("Timed out waiting for matching guardrail output")


@pytest.mark.slow
def test_stream_wiring_end_to_end_passes_upstream_twist(
    started_guardrail: tuple[
        RGBCollisionGuardrail,
        FakeTransport[Image],
        FakeTransport[Twist],
        queue.Queue[tuple[float, Twist]],
    ],
) -> None:
    guardrail, image_transport, cmd_transport, outputs = started_guardrail

    upstream = _cmd(0.03, angular_z=0.15)

    cmd_transport.publish(upstream)
    image_transport.publish(_textured_gray_image())
    image_transport.publish(_textured_gray_image(shift_x=2))

    _, observed = _wait_for_output(outputs, lambda msg: msg == upstream)
    assert observed == upstream


@pytest.mark.slow
def test_non_pass_output_is_republished_while_upstream_is_quiet() -> None:
    guardrail, image_transport, cmd_transport, outputs = _start_guardrail(
        guarded_output_publish_hz=20.0,
        risk_evaluation_hz=20.0,
        command_timeout_s=0.5,
        image_timeout_s=0.5,
        risk_timeout_s=0.5,
    )
    guarded = Twist(linear=[0.1, 0.0, 0.0], angular=[0.0, 0.0, 0.2])

    try:
        guardrail._policy = SequencePolicy(
            [_decision(GuardrailState.CLAMP, guarded, reason="forced_clamp")]
        )

        cmd_transport.publish(_cmd(0.4, angular_z=0.2))
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        _wait_for_output(outputs, lambda msg: msg == guarded, timeout_s=1.0)
        _wait_for_output(outputs, lambda msg: msg == guarded, timeout_s=1.0)

        t1, msg1 = _wait_for_output(outputs, lambda msg: msg == guarded, timeout_s=1.0)
        t2, msg2 = _wait_for_output(outputs, lambda msg: msg == guarded, timeout_s=1.0)

        assert msg1 == guarded
        assert msg2 == guarded

        interval = t2 - t1
        expected_period = 1.0 / guardrail.config.guarded_output_publish_hz

        assert expected_period * 0.5 <= interval <= expected_period * 2.0
    finally:
        guardrail.stop()


@pytest.mark.slow
def test_forced_stop_never_leaks_positive_linear_x_under_concurrent_updates() -> None:
    guardrail, image_transport, cmd_transport, outputs = _start_guardrail(
        guarded_output_publish_hz=30.0,
        risk_evaluation_hz=30.0,
        command_timeout_s=0.5,
        image_timeout_s=0.5,
        risk_timeout_s=0.5,
    )
    stop_cmd = Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.2])

    stop_event = threading.Event()
    errors: list[Exception] = []

    def publish_images() -> None:
        try:
            shift = 0
            image_transport.publish(_textured_gray_image(shift_x=shift))
            while not stop_event.is_set():
                shift += 1
                image_transport.publish(_textured_gray_image(shift_x=shift))
                time.sleep(0.01)
        except Exception as exc:
            errors.append(exc)

    def publish_commands() -> None:
        try:
            speeds = [0.4, 0.25, 0.5, 0.15, 0.35]
            idx = 0
            while not stop_event.is_set():
                cmd_transport.publish(_cmd(speeds[idx % len(speeds)], angular_z=0.2))
                idx += 1
                time.sleep(0.01)
        except Exception as exc:
            errors.append(exc)

    try:
        guardrail._policy = SequencePolicy(
            [
                _decision(
                    GuardrailState.STOP_LATCHED,
                    stop_cmd,
                    reason="forced_stop",
                    publish_immediately=True,
                )
            ]
        )

        image_thread = threading.Thread(target=publish_images, daemon=True)
        cmd_thread = threading.Thread(target=publish_commands, daemon=True)

        image_thread.start()
        cmd_thread.start()

        observed_outputs: list[Twist] = []
        while len(observed_outputs) < 8:
            _, observed = _wait_for_output(
                outputs, lambda msg: isinstance(msg, Twist), timeout_s=1.0
            )
            observed_outputs.append(observed)

        assert observed_outputs
        assert all(twist.linear.x == pytest.approx(0.0) for twist in observed_outputs)
        assert all(twist == stop_cmd or twist == Twist.zero() for twist in observed_outputs)
    finally:
        stop_event.set()
        if "image_thread" in locals():
            image_thread.join(timeout=1.0)
        if "cmd_thread" in locals():
            cmd_thread.join(timeout=1.0)
        guardrail.stop()

    assert not errors
    assert not image_thread.is_alive()
    assert not cmd_thread.is_alive()


@pytest.mark.slow
def test_black_frame_end_to_end_fail_closes_to_zero(
    started_guardrail: tuple[
        RGBCollisionGuardrail,
        FakeTransport[Image],
        FakeTransport[Twist],
        queue.Queue[tuple[float, Twist]],
    ],
) -> None:
    guardrail, image_transport, cmd_transport, outputs = started_guardrail

    cmd_transport.publish(_cmd(0.3, angular_z=0.1))
    image_transport.publish(_textured_gray_image())
    image_transport.publish(_black_gray_image())

    _, observed = _wait_for_output(outputs, lambda msg: msg == Twist.zero(), timeout_s=1.0)
    assert observed == Twist.zero()


@pytest.mark.slow
def test_stale_image_end_to_end_fail_closes_without_new_command() -> None:
    guardrail, image_transport, cmd_transport, outputs = _start_guardrail(
        guarded_output_publish_hz=25.0,
        risk_evaluation_hz=25.0,
        command_timeout_s=0.5,
        image_timeout_s=0.08,
        risk_timeout_s=0.5,
    )
    # Keep the initial command below deadband so this test is about
    # autonomous stale-image fail-close, not optical-flow threshold tuning.
    upstream = _cmd(0.03, angular_z=0.1)

    try:
        cmd_transport.publish(upstream)
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        _, pass_output = _wait_for_output(outputs, lambda msg: msg == upstream, timeout_s=1.0)
        assert pass_output == upstream

        _, degraded_output = _wait_for_output(
            outputs,
            lambda msg: msg == Twist.zero(),
            timeout_s=0.4,
        )
        assert degraded_output == Twist.zero()
    finally:
        guardrail.stop()


@pytest.mark.slow
def test_stale_command_end_to_end_fail_closes_to_zero(
    started_guardrail: tuple[
        RGBCollisionGuardrail,
        FakeTransport[Image],
        FakeTransport[Twist],
        queue.Queue[tuple[float, Twist]],
    ],
) -> None:
    guardrail, image_transport, cmd_transport, outputs = started_guardrail

    # Keep the initial command below deadband so this test is about
    # stale-command fail-close, not optical-flow clamp behavior.
    upstream = _cmd(0.03, angular_z=0.1)

    cmd_transport.publish(upstream)
    image_transport.publish(_textured_gray_image())
    image_transport.publish(_textured_gray_image(shift_x=2))

    _wait_for_output(outputs, lambda msg: msg == upstream, timeout_s=1.0)
    _, observed = _wait_for_output(outputs, lambda msg: msg == Twist.zero(), timeout_s=1.0)

    assert observed == Twist.zero()
