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

"""HealthMonitor — contract messages, states, and drop accounting (fake clock)."""

from __future__ import annotations

from typing import Any

import pytest

import dimos.memory2.health
from dimos.memory2.health import (
    DEGRADED,
    OK,
    STALLED,
    Health,
    HealthConfig,
    HealthMonitor,
    ModuleContracts,
)


class LogSpy:
    """Records formatted log lines — independent of logging configuration."""

    def __init__(self) -> None:
        self.lines: list[tuple[str, str]] = []

    def _record(self, level: str, msg: str, *args: object) -> None:
        self.lines.append((level, msg % args if args else msg))

    def info(self, msg: str, *args: object) -> None:
        self._record("INFO", msg, *args)

    def warning(self, msg: str, *args: object) -> None:
        self._record("WARNING", msg, *args)

    def exception(self, msg: str, *args: object) -> None:
        self._record("ERROR", msg, *args)

    def at(self, level: str) -> list[str]:
        return [line for lvl, line in self.lines if lvl == level]


@pytest.fixture
def logspy(monkeypatch: pytest.MonkeyPatch) -> LogSpy:
    spy = LogSpy()
    monkeypatch.setattr(dimos.memory2.health, "logger", spy)
    return spy


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


_CONTRACT_KEYS = {"min_output_hz", "max_drop_ratio", "max_tick_latency_s", "max_missing_ratio"}
_HEALTH_KEYS = {
    "interval_s",
    "warmup_s",
    "unhealthy_log_every_s",
    "stall_after_s",
    "ratio_min_samples",
    "stream",
}


def make(clock: Clock, **kwargs: Any) -> tuple[HealthMonitor, list[Health]]:
    """Build a monitor from flat kwargs, routed into the structured sub-configs."""
    snaps: list[Health] = []
    health_kwargs = {k: v for k, v in kwargs.items() if k in _HEALTH_KEYS}
    health_kwargs.setdefault("interval_s", 1.0)
    health_kwargs.setdefault("warmup_s", 0.0)  # most tests skip warmup gating
    rest = {k: v for k, v in kwargs.items() if k not in _CONTRACT_KEYS | _HEALTH_KEYS}
    m = HealthMonitor(
        "mod",
        clock=clock,
        sink=snaps.append,
        contracts=ModuleContracts(**{k: v for k, v in kwargs.items() if k in _CONTRACT_KEYS}),
        health=HealthConfig(**health_kwargs),
        **rest,
    )
    return m, snaps


def report(m: HealthMonitor, clock: Clock) -> Health:
    clock.advance(1.0)
    h = m.maybe_report()
    assert h is not None
    return h


def test_reports_only_after_interval() -> None:
    clock = Clock()
    m, snaps = make(clock)
    assert m.maybe_report() is None  # interval not elapsed
    clock.advance(0.5)
    assert m.maybe_report() is None
    clock.advance(0.6)
    assert m.maybe_report() is not None
    assert len(snaps) == 1


def test_healthy_steady_state_with_expected_drops() -> None:
    """KeepLast dropping most ticks is OK as long as contracts hold."""
    clock = Clock()
    m, _ = make(clock, min_output_hz=5.0)
    for _ in range(30):  # 30 ticks queued...
        m.on_resolved()
        m.on_queued()
    for _ in range(10):  # ...10 stepped (KeepLast ate 20)
        m.on_step(0.05, {}, emitted=True)

    h = report(m, clock)
    assert h.state == OK  # drops alone are not a violation
    assert h.metrics["drops_backpressure_hz"] == 20.0
    assert h.metrics["ticks_stepped_hz"] == 10.0


def test_output_contract_violation_and_recovery() -> None:
    clock = Clock()
    m, _ = make(clock, min_output_hz=10.0)

    for _ in range(3):
        m.on_step(0.01, {}, emitted=True)
    h = report(m, clock)
    assert h.state == DEGRADED
    assert any("output 3.0 Hz < contract 10 Hz" in v for v in h.violations)

    for _ in range(12):
        m.on_step(0.01, {}, emitted=True)
    h = report(m, clock)
    assert h.state == OK
    assert h.violations == ()


def test_input_rate_violation_mentions_rates() -> None:
    clock = Clock()
    m, _ = make(clock, expected_hz={"pose": 50.0})
    for _ in range(12):
        m.on_input("pose")
    h = report(m, clock)
    assert h.state == DEGRADED
    assert any("'pose' at 12.0 Hz, expected 50 Hz" in v for v in h.violations)


def test_missing_input_majority_is_a_violation() -> None:
    clock = Clock()
    m, _ = make(clock)
    for _ in range(10):
        m.on_resolved()
    for _ in range(8):
        m.on_missing(["imu"])
    h = report(m, clock)
    assert h.state == DEGRADED
    assert any("'imu' missing on 8/10 ticks" in v for v in h.violations)


def test_stall_requires_persistence() -> None:
    clock = Clock()
    m, _ = make(clock, stall_after_s=3.0)

    # Ticks queue but nothing steps. One window: not yet a stall.
    m.on_resolved()
    m.on_queued()
    assert report(m, clock).state == OK

    # Buffer still holding, two more zero-step windows -> STALLED.
    m.attach_gauges(buffer_len=lambda: 1, pending_len=lambda: 0)
    assert report(m, clock).state == OK
    h = report(m, clock)
    assert h.state == STALLED
    assert any("none stepped" in v for v in h.violations)

    # A step recovers it.
    m.on_step(0.01, {}, emitted=True)
    m.attach_gauges(buffer_len=lambda: 0, pending_len=lambda: 0)
    assert report(m, clock).state == OK


def test_alignment_stall_when_inputs_flow_but_nothing_resolves() -> None:
    clock = Clock()
    m, _ = make(clock, stall_after_s=2.0)
    m.attach_gauges(buffer_len=lambda: 0, pending_len=lambda: 3)
    for _ in range(2):
        for _ in range(5):
            m.on_input("image")
        clock.advance(1.0)
        h = m.maybe_report()
    assert h is not None
    assert h.state == STALLED
    assert any("no ticks resolving" in v for v in h.violations)


def test_warmup_gates_violations() -> None:
    clock = Clock()
    m, _ = make(clock, warmup_s=3.0, expected_hz={"image": 30.0})
    h = report(m, clock)  # t=1.0 — warming up, no violations yet
    assert h.state == OK and h.violations == ()
    report(m, clock)
    h = report(m, clock)  # t=3.0 — warmup done, contract active
    assert h.state == DEGRADED


def test_staleness_ages_reported() -> None:
    clock = Clock()
    m, _ = make(clock)
    m.on_step(0.01, {"imu": 0.02}, emitted=True)
    m.on_step(0.01, {"imu": 0.09}, emitted=True)
    h = report(m, clock)
    assert h.metrics["age_imu_p99_s"] == 0.09
    assert h.metrics["step_p50_ms"] == 10.0


def test_blocked_drops_metric() -> None:
    clock = Clock()
    m, _ = make(clock)
    m.on_blocked(7)
    h = report(m, clock)
    assert h.metrics["drops_blocked_hz"] == 7.0


# -- ratio & latency contracts ---------------------------------------------------


def test_drop_ratio_contract() -> None:
    clock = Clock()
    m, _ = make(clock, max_drop_ratio=0.5)
    for _ in range(20):
        m.on_resolved()
        m.on_queued()
    for _ in range(5):  # 15 of 20 skipped -> 75% > 50% contract
        m.on_step(0.01, {}, emitted=True)
    h = report(m, clock)
    assert h.state == DEGRADED
    assert any("drop ratio 75% > contract 50%" in v for v in h.violations)

    for _ in range(20):  # keeping up again
        m.on_resolved()
        m.on_queued()
        m.on_step(0.01, {}, emitted=True)
    assert report(m, clock).state == OK


def test_ratio_contracts_need_min_samples() -> None:
    """Tiny windows and zero traffic must not trip ratio contracts."""
    clock = Clock()
    m, _ = make(clock, max_drop_ratio=0.1, max_tick_latency_s=0.1)

    assert report(m, clock).state == OK  # zero traffic: vacuous, stays OK

    for _ in range(4):  # 3 of 4 dropped = 75%, but below ratio_min_samples=10
        m.on_queued()
    m.on_step(0.01, {}, emitted=True, latency_s=5.0)  # 1 latency sample, ditto
    assert report(m, clock).state == OK


def test_tick_latency_contract_and_metric() -> None:
    clock = Clock()
    m, _ = make(clock, max_tick_latency_s=0.5)
    for _ in range(12):
        m.on_step(0.01, {}, emitted=True, latency_s=1.2)
    h = report(m, clock)
    assert h.state == DEGRADED
    assert any("tick latency p99 1200 ms > contract 500 ms" in v for v in h.violations)
    assert h.metrics["tick_latency_p99_ms"] == 1200.0

    for _ in range(12):
        m.on_step(0.01, {}, emitted=True, latency_s=0.02)
    assert report(m, clock).state == OK


def test_missing_ratio_is_configurable() -> None:
    clock = Clock()
    m, _ = make(clock, max_missing_ratio=0.9)
    for _ in range(10):
        m.on_resolved()
    for _ in range(8):  # 80% missing, under the 90% contract
        m.on_missing(["imu"])
    assert report(m, clock).state == OK


def test_per_output_contract_only_flags_contracted_port() -> None:
    clock = Clock()
    m, _ = make(clock, out_min_hz={"cmd": 10.0})
    for _ in range(3):
        m.on_output("cmd")  # 3 Hz < 10 Hz contract
    # 'alerts' emits nothing at all — sparse by design, uncontracted, no violation
    h = report(m, clock)
    assert h.state == DEGRADED
    assert h.violations == ("output 'cmd' at 3.0 Hz < contract 10 Hz",)

    for _ in range(12):
        m.on_output("cmd")
    assert report(m, clock).state == OK


def test_per_input_missing_ratio_overrides_global() -> None:
    clock = Clock()
    m, _ = make(clock, max_missing_ratio=0.5, missing_ratio_by_input={"gps": 0.9})
    for _ in range(10):
        m.on_resolved()
    for _ in range(8):
        m.on_missing(["gps"])  # 80% — over global 0.5, under gps's declared 0.9
    assert report(m, clock).state == OK


# -- contract messages (log lines) ---------------------------------------------------


def test_degraded_logged_once_then_throttled(logspy: LogSpy) -> None:
    clock = Clock()
    m, _ = make(clock, min_output_hz=10.0, unhealthy_log_every_s=10.0)

    report(m, clock)  # violation -> DEGRADED, logged
    warns = logspy.at("WARNING")
    assert len(warns) == 1
    assert "mod DEGRADED: output 0.0 Hz < contract 10 Hz" in warns[0]

    report(m, clock)  # still degraded, inside throttle window -> no new line
    assert len(logspy.at("WARNING")) == 1

    clock.advance(10.0)  # throttle elapsed -> one reminder with duration
    m.maybe_report()
    warns = logspy.at("WARNING")
    assert len(warns) == 2
    assert "still DEGRADED" in warns[1]


def test_recovery_logged_with_duration(logspy: LogSpy) -> None:
    clock = Clock()
    m, _ = make(clock, min_output_hz=10.0)
    report(m, clock)  # DEGRADED
    for _ in range(12):
        m.on_step(0.01, {}, emitted=True)
    report(m, clock)  # back to OK
    infos = logspy.at("INFO")
    assert any("mod OK: recovered after" in line for line in infos)


def test_warmup_line_reports_observed_vs_expected(logspy: LogSpy) -> None:
    clock = Clock()
    m, _ = make(clock, warmup_s=2.0, expected_hz={"image": 30.0, "pose": 50.0})
    for _ in range(60):
        m.on_input("image")  # 30 Hz over the 2s warmup
    for _ in range(10):
        m.on_input("pose")  # 5 Hz — far below 50
    report(m, clock)
    report(m, clock)  # t=2.0 -> warmup line emitted exactly once
    report(m, clock)
    warmups = [line for line in logspy.at("INFO") if "warmup" in line]
    assert len(warmups) == 1
    assert "image 30.0 Hz (expected 30)" in warmups[0]
    assert "pose 5.0 Hz (expected 50 — LOW)" in warmups[0]


def test_stall_message_names_the_suspect(logspy: LogSpy) -> None:
    clock = Clock()
    m, _ = make(clock, stall_after_s=1.0)
    m.attach_gauges(buffer_len=lambda: 1, pending_len=lambda: 0)
    m.on_resolved()
    m.on_queued()
    h = report(m, clock)
    assert h.state == STALLED
    assert any("STALLED" in line and "step stuck" in line for line in logspy.at("WARNING"))
