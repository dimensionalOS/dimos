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

from __future__ import annotations

from collections.abc import Callable, Iterator
import queue
import threading
import time
from typing import Any, TypeVar

import pytest

from dimos.control.safety.guardrail_policy import (
    GuardrailDecision,
    GuardrailHealth,
    GuardrailState,
)
from dimos.control.safety.rgb_collision_guardrail import RGBCollisionGuardrail
from dimos.control.safety.test_utils import (
    FakeTransport,
    SequencePolicy,
    _cmd,
    _decision,
    _textured_gray_image,
)
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image

T = TypeVar("T")


class RaisingPolicy:
    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        raise RuntimeError("synthetic policy failure")


class CountingPassPolicy:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._call_count = 0

    @property
    def call_count(self) -> int:
        with self._lock:
            return self._call_count

    def evaluate(
        self,
        previous_image: Image,
        current_image: Image,
        incoming_cmd_vel: Twist,
        health: GuardrailHealth,
    ) -> GuardrailDecision:
        with self._lock:
            self._call_count += 1

        return GuardrailDecision(
            state=GuardrailState.PASS,
            cmd_vel=Twist(
                linear=incoming_cmd_vel.linear,
                angular=incoming_cmd_vel.angular,
            ),
            reason="counting_pass",
        )


@pytest.fixture
def module() -> Iterator[RGBCollisionGuardrail]:
    guardrail = RGBCollisionGuardrail(
        guarded_output_publish_hz=50.0,
        risk_evaluation_hz=50.0,
        command_timeout_s=0.05,
        image_timeout_s=0.05,
        risk_timeout_s=0.05,
    )
    yield guardrail
    guardrail._close_module()


def _wait_for_output(
    outputs: queue.Queue[Twist],
    predicate: Callable[[Twist], bool],
    *,
    timeout_s: float = 0.5,
) -> Twist:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            candidate = outputs.get(timeout=max(remaining, 0.01))
        except queue.Empty:
            continue
        if predicate(candidate):
            return candidate
    raise AssertionError("Timed out waiting for matching guardrail output")


def _wait_for_decision(
    guardrail: RGBCollisionGuardrail,
    predicate: Callable[[GuardrailDecision], bool],
    *,
    timeout_s: float = 0.5,
) -> GuardrailDecision:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with guardrail._condition:
            decision = guardrail._runtime_state.last_decision
            if decision is not None and predicate(decision):
                return decision
        time.sleep(0.01)

    raise AssertionError("Timed out waiting for matching guardrail decision")


def _start_threaded_guardrail(
    policy: Any,
    **config_overrides: float,
) -> tuple[RGBCollisionGuardrail, FakeTransport[Image], FakeTransport[Twist], queue.Queue[Twist]]:
    config: dict[str, float] = {
        "guarded_output_publish_hz": 50.0,
        "risk_evaluation_hz": 50.0,
        "command_timeout_s": 0.3,
        "image_timeout_s": 0.3,
        "risk_timeout_s": 0.3,
    }
    config.update(config_overrides)

    guardrail = RGBCollisionGuardrail(**config)
    image_transport: FakeTransport[Image] = FakeTransport()
    cmd_transport: FakeTransport[Twist] = FakeTransport()
    outputs: queue.Queue[Twist] = queue.Queue()

    guardrail.color_image.transport = image_transport
    guardrail.incoming_cmd_vel.transport = cmd_transport
    guardrail.safe_cmd_vel.subscribe(outputs.put)
    guardrail._policy = policy
    guardrail.start()

    return guardrail, image_transport, cmd_transport, outputs


def test_no_command_returns_init_zero(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()

    with module._condition:
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.INIT
    assert decision.reason == "no_command_received"
    assert decision.cmd_vel == Twist.zero()


def test_waiting_for_first_image_returns_init_zero(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = now
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.INIT
    assert decision.reason == "waiting_for_first_image"
    assert decision.cmd_vel == Twist.zero()


def test_no_frame_pair_returns_init_zero(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = now
        module._runtime_state.latest_image = _textured_gray_image()
        module._runtime_state.latest_image_time = now
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.INIT
    assert decision.reason == "waiting_for_frame_pair"
    assert decision.cmd_vel == Twist.zero()


def test_waiting_for_first_risk_evaluation_returns_init_zero(
    module: RGBCollisionGuardrail,
) -> None:
    now = time.monotonic()

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = now
        module._runtime_state.previous_image = _textured_gray_image()
        module._runtime_state.previous_image_time = now
        module._runtime_state.latest_image = _textured_gray_image(shift_x=2)
        module._runtime_state.latest_image_time = now
        module._runtime_state.last_risk_time = None
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.INIT
    assert decision.reason == "waiting_for_first_risk_evaluation"
    assert decision.cmd_vel == Twist.zero()


def test_stale_image_returns_sensor_degraded_zero(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()
    stale_time = now - 0.2

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = now
        module._runtime_state.previous_image = _textured_gray_image()
        module._runtime_state.latest_image = _textured_gray_image(shift_x=2)
        module._runtime_state.previous_image_time = stale_time
        module._runtime_state.latest_image_time = stale_time
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.SENSOR_DEGRADED
    assert decision.reason == "image_stale"
    assert decision.cmd_vel == Twist.zero()


def test_stale_risk_returns_sensor_degraded_zero(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()
    stale_risk_time = now - 0.2

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = now
        module._runtime_state.previous_image = _textured_gray_image()
        module._runtime_state.latest_image = _textured_gray_image(shift_x=2)
        module._runtime_state.previous_image_time = now
        module._runtime_state.latest_image_time = now
        module._runtime_state.last_risk_time = stale_risk_time
        decision = module._select_fallback_decision_locked(now)

    assert decision is not None
    assert decision.state == GuardrailState.SENSOR_DEGRADED
    assert decision.reason == "risk_state_stale"
    assert decision.cmd_vel == Twist.zero()


def test_stale_command_publishes_zero_output(module: RGBCollisionGuardrail) -> None:
    now = time.monotonic()
    stale_cmd_time = now - 0.2

    with module._condition:
        module._runtime_state.latest_cmd_vel = _cmd()
        module._runtime_state.latest_cmd_time = stale_cmd_time
        module._runtime_state.last_decision = _decision(GuardrailState.PASS, _cmd())
        module._runtime_state.pending_cmd_update = True
        cmd_to_publish = module._consume_publish_cmd_locked(now)

    assert cmd_to_publish == Twist.zero()


def test_policy_exception_fail_closes_to_zero() -> None:
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(
        RaisingPolicy(),
    )

    try:
        cmd_transport.publish(_cmd(0.4, angular_z=0.3))
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        observed = _wait_for_output(outputs, lambda twist: twist == Twist.zero())
        assert observed == Twist.zero()

        decision = _wait_for_decision(
            guardrail,
            lambda d: d.state == GuardrailState.SENSOR_DEGRADED
            and d.reason == "policy_evaluation_failed",
            timeout_s=0.5,
        )

        assert decision.state == GuardrailState.SENSOR_DEGRADED
        assert decision.reason == "policy_evaluation_failed"

        with guardrail._condition:
            assert guardrail._runtime_state.state == GuardrailState.SENSOR_DEGRADED
    finally:
        guardrail.stop()


def test_pass_publishes_latest_upstream_command() -> None:
    upstream_first = _cmd(0.3, angular_z=0.1)
    upstream_second = _cmd(0.45, angular_z=0.35)
    misleading_policy_cmd = _cmd(0.02, angular_z=-0.2)
    policy = SequencePolicy([_decision(GuardrailState.PASS, misleading_policy_cmd, reason="pass")])
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(policy)

    try:
        cmd_transport.publish(upstream_first)
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        first_output = _wait_for_output(outputs, lambda twist: twist == upstream_first)
        assert first_output == upstream_first

        cmd_transport.publish(upstream_second)
        second_output = _wait_for_output(outputs, lambda twist: twist == upstream_second)
        assert second_output == upstream_second
    finally:
        guardrail.stop()


@pytest.mark.parametrize(
    ("state", "guarded_cmd"),
    [
        (
            GuardrailState.CLAMP,
            Twist(linear=[0.1, 0.0, 0.0], angular=[0.0, 0.0, 0.4]),
        ),
        (
            GuardrailState.STOP_LATCHED,
            Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.4]),
        ),
    ],
)
def test_non_pass_states_publish_guarded_output(state: GuardrailState, guarded_cmd: Twist) -> None:
    upstream_cmd = _cmd(0.35, angular_z=0.4)
    policy = SequencePolicy([_decision(state, guarded_cmd)])
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(policy)

    try:
        cmd_transport.publish(upstream_cmd)
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        published = _wait_for_output(outputs, lambda twist: twist == guarded_cmd)
        assert published == guarded_cmd
    finally:
        guardrail.stop()


def test_non_pass_heartbeat_republishes_guarded_output() -> None:
    guarded_cmd = Twist(linear=[0.1, 0.0, 0.0], angular=[0.0, 0.0, 0.5])
    policy = SequencePolicy([_decision(GuardrailState.CLAMP, guarded_cmd)])
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(policy)

    try:
        cmd_transport.publish(_cmd(0.4, angular_z=0.5))
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        first = _wait_for_output(outputs, lambda twist: twist == guarded_cmd)
        second = _wait_for_output(outputs, lambda twist: twist == guarded_cmd)

        assert first == guarded_cmd
        assert second == guarded_cmd
    finally:
        guardrail.stop()


def test_non_pass_decision_can_publish_without_new_command() -> None:
    upstream_cmd = _cmd(0.4, angular_z=0.3)
    stop_cmd = Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.3])
    policy = SequencePolicy(
        [
            _decision(GuardrailState.PASS, upstream_cmd, reason="initial_pass"),
            _decision(
                GuardrailState.STOP_LATCHED,
                stop_cmd,
                reason="forced_stop",
                publish_immediately=True,
            ),
        ]
    )
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(policy)

    try:
        cmd_transport.publish(upstream_cmd)
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        first_output = _wait_for_output(outputs, lambda twist: twist == upstream_cmd)
        assert first_output == upstream_cmd

        image_transport.publish(_textured_gray_image(shift_x=4))

        autonomous_stop = _wait_for_output(outputs, lambda twist: twist == stop_cmd)
        assert autonomous_stop == stop_cmd
    finally:
        guardrail.stop()


def test_fast_upstream_commands_reuse_last_risk_decision() -> None:
    policy = CountingPassPolicy()
    guardrail, image_transport, cmd_transport, outputs = _start_threaded_guardrail(
        policy,
        guarded_output_publish_hz=100.0,
        risk_evaluation_hz=2.0,
        command_timeout_s=1.0,
        image_timeout_s=1.0,
        risk_timeout_s=1.0,
    )

    first_cmd = _cmd(0.20, angular_z=0.10)
    second_cmd = _cmd(0.32, angular_z=0.20)
    third_cmd = _cmd(0.44, angular_z=0.30)

    try:
        cmd_transport.publish(first_cmd)
        image_transport.publish(_textured_gray_image())
        image_transport.publish(_textured_gray_image(shift_x=2))

        first_output = _wait_for_output(outputs, lambda twist: twist == first_cmd, timeout_s=0.6)
        assert first_output == first_cmd
        assert policy.call_count == 1

        cmd_transport.publish(second_cmd)
        second_output = _wait_for_output(
            outputs,
            lambda twist: twist == second_cmd,
            timeout_s=0.2,
        )
        assert second_output == second_cmd

        cmd_transport.publish(third_cmd)
        third_output = _wait_for_output(
            outputs,
            lambda twist: twist == third_cmd,
            timeout_s=0.2,
        )
        assert third_output == third_cmd

        assert policy.call_count == 1
        assert policy.call_count < 3
    finally:
        guardrail.stop()
