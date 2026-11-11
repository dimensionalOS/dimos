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

"""Latest-target consumption and single-owner SDK I/O scheduling.

The fake SDK boundary is blocked with Event barriers so every assertion
about what was written, and when, is deterministic: the command callback
must return while a write is held, a burst must collapse to its newest
target, stale targets must die before the write, and a stop or fault while
the SDK is held must invalidate pending work instead of replaying it.
"""

from __future__ import annotations

import threading
import time

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao import connection as common
from dimos.utils.testing.waiting import wait_until


def _block_write(module, monkeypatch):
    """Hold the fake SDK write at a barrier; returns (entered, release)."""
    entered, release = threading.Event(), threading.Event()
    original = module._arm.command

    def blocked(targets):
        entered.set()
        assert release.wait(2)
        return original(targets)

    monkeypatch.setattr(module._arm, "command", blocked)
    return entered, release


def _written(module):
    return module._arm._arms["left_arm"].commands


def test_callback_returns_while_write_held_and_burst_writes_latest_only(make_rig, monkeypatch):
    module, commands, _ = make_rig()
    entered, release = _block_write(module, monkeypatch)
    commands.publish(JointState(name=["arm/joint1"], position=[0.1]))
    assert entered.wait(2), "owner did not reach the held write"

    # The callback only merges and wakes: five publishes return while the
    # write is still held, and the pending target stays key-bounded.
    started = time.monotonic()
    for step in range(1, 6):
        commands.publish(JointState(name=["arm/joint1"], position=[0.1 + 0.01 * step]))
    assert time.monotonic() - started < 1.0
    assert module._pending_max <= len(module.joint_names)

    release.set()
    wait_until(lambda: len(_written(module)) >= 2, timeout=2)
    time.sleep(0.05)
    rows = [row[0, 2] for row in _written(module)]
    assert rows == [pytest.approx(0.1), pytest.approx(0.15)]
    for superseded in (0.11, 0.12, 0.13, 0.14):
        assert superseded not in [round(row, 4) for row in rows]


def test_partial_updates_merge_by_key(make_rig, monkeypatch):
    module, commands, _ = make_rig()
    entered, release = _block_write(module, monkeypatch)
    commands.publish(JointState(name=["arm/joint1"], position=[0.5]))
    assert entered.wait(2)
    commands.publish(JointState(name=["arm/joint1"], position=[0.6]))
    commands.publish(JointState(name=["arm/joint2"], position=[-0.4]))
    release.set()
    wait_until(lambda: len(_written(module)) >= 2, timeout=2)
    # The merged flush carries the newest value of each key: joint1 from the
    # second update, joint2 from the third, neither lost nor reordered.
    assert _written(module)[-1][:, 2] == pytest.approx([0.6, -0.4])


def test_write_wakes_owner_without_waiting_for_the_sampling_tick(make_rig):
    module, commands, _ = make_rig(state_rate_hz=1.0)
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    # The read period is a full second; a wake-driven write lands long before it.
    wait_until(lambda: _written(module), timeout=0.5)
    assert _written(module)[-1][0, 2] == pytest.approx(0.7)


def test_stale_source_timestamp_is_dropped_before_write(make_rig):
    module, commands, _ = make_rig(command_max_age_s=0.05)
    commands.publish(JointState(ts=time.time() - 1.0, name=["arm/joint1"], position=[0.7]))
    wait_until(lambda: "expired" in (module.status().last_rejection or ""), timeout=2)
    time.sleep(0.05)
    assert _written(module) == []
    # An expired target never executes, and the deadman stays where the last
    # genuinely accepted command left it.
    assert not module.status().deadman_fired


def test_target_held_past_its_age_budget_is_dropped_at_the_write(make_rig, monkeypatch):
    module, commands, _ = make_rig(command_max_age_s=0.05)
    entered, release = _block_write(module, monkeypatch)
    commands.publish(JointState(name=["arm/joint1"], position=[0.1]))
    assert entered.wait(2)
    # Fresh when accepted, older than command_max_age_s when the flush runs.
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    assert module._pending
    time.sleep(0.1)
    release.set()
    wait_until(lambda: "expired" in (module.status().last_rejection or ""), timeout=2)
    time.sleep(0.05)
    rows = [row[0, 2] for row in _written(module)]
    assert rows == [pytest.approx(0.1)]
    # The target queued behind the blocked write expired without reaching the
    # hardware, so it never authorised the deadman either.
    assert module._command_seq == 1


def test_stop_while_write_held_clears_pending_and_never_replays(make_rig, boundary, monkeypatch):
    module, commands, _ = make_rig()
    monkeypatch.setattr(common, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.2)
    entered, release = _block_write(module, monkeypatch)
    commands.publish(JointState(name=["arm/joint1"], position=[0.1]))
    assert entered.wait(2)
    for step in range(1, 4):
        commands.publish(JointState(name=["arm/joint1"], position=[0.1 + 0.01 * step]))

    stopped = threading.Event()
    stopper = threading.Thread(target=lambda: (module.stop(), stopped.set()), daemon=True)
    stopper.start()
    try:
        # Pending work is invalidated before the join, while the SDK is held.
        wait_until(lambda: not module._pending, timeout=2)
    finally:
        release.set()
        stopper.join(timeout=5)
    assert stopped.is_set()
    time.sleep(0.05)
    rows = [row[0, 2] for row in boundary[0].groups["left_arm"].commands]
    # Only the held write (0.1) and the shutdown safety hold (measured 0.2)
    # may appear; the invalidated burst never replays.
    for superseded in (0.11, 0.12, 0.13):
        assert round(superseded, 4) not in [round(row, 4) for row in rows]
    assert rows[0] == pytest.approx(0.1)
    assert not module._reader.is_alive()
    assert not module.status().connected


def test_fault_while_write_held_invalidates_pending(make_rig, monkeypatch):
    module, commands, _ = make_rig()
    entered, release = _block_write(module, monkeypatch)
    disabled: list[int] = []
    original_disable = module._arm.disable

    def tracked_disable():
        disabled.append(threading.get_ident())
        return original_disable()

    monkeypatch.setattr(module._arm, "disable", tracked_disable)
    commands.publish(JointState(name=["arm/joint1"], position=[0.1]))
    assert entered.wait(2)
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    module._feedback_failed(RuntimeError("lost feedback"))
    assert module._pending == {}
    release.set()
    wait_until(lambda: bool(disabled), timeout=2)
    time.sleep(0.05)
    rows = [row[0, 2] for row in _written(module)]
    # The held write is already admitted, so it may finish: an SDK call is
    # not preemptible. The target queued behind it must never run, and the
    # disable itself must execute on the single SDK owner.
    assert rows == [pytest.approx(0.1)]
    assert disabled and all(owner == module._reader.ident for owner in disabled)
    assert "feedback failed" in (module._fault or "")


def test_reader_continues_under_command_flood_and_sdk_calls_never_overlap(make_rig, monkeypatch):
    module, commands, readings = make_rig()
    active = 0
    violations = []
    guard = threading.Lock()
    original_command = module._arm.command
    original_read = module._arm.read

    def tracked(call):
        def wrapped(*args):
            nonlocal active
            with guard:
                active += 1
                if active != 1:
                    violations.append(active)
            try:
                return call(*args)
            finally:
                with guard:
                    active -= 1

        return wrapped

    monkeypatch.setattr(module._arm, "command", tracked(original_command))
    monkeypatch.setattr(module._arm, "read", tracked(original_read))

    flooding = threading.Event()
    flooding.set()

    def flood():
        while flooding.is_set():
            commands.publish(JointState(name=["arm/joint1"], position=[0.5]))

    flooder = threading.Thread(target=flood, daemon=True)
    reads_before = len(readings.sent)
    flooder.start()
    try:
        # Reads are never permanently skipped, even under a sustained flood.
        wait_until(lambda: len(readings.sent) > reads_before + 5, timeout=2)
    finally:
        flooding.clear()
        flooder.join(timeout=2)
    wait_until(lambda: _written(module), timeout=2)
    assert violations == []
    assert module._pending_max <= len(module.joint_names)


def test_repeated_stale_source_traffic_never_renews_the_deadman(make_rig):
    module, commands, _ = make_rig(command_max_age_s=0.05)
    module.config.deadman_timeout_s = 0.08
    commands.publish(JointState(name=["arm/joint1"], position=[0.1]))
    wait_until(lambda: _written(module), timeout=2)
    authorized = module._command_seq
    for _ in range(12):
        commands.publish(JointState(ts=time.time() - 10, name=["arm/joint1"], position=[0.7]))
        time.sleep(0.01)
    # Already-stale traffic is rejected at admission: it neither authorises
    # the deadman nor increments the sequence the timer compares against.
    assert module._command_seq == authorized
    assert "expired" in (module.status().last_rejection or "")
    # With no eligible write, the deadman is not suppressed by that traffic.
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    written = [round(row[0, 2], 4) for row in _written(module)]
    assert written[0] == 0.1
    assert 0.7 not in written


def test_deadman_hold_and_teardown_disable_run_on_the_owner(make_rig, monkeypatch):
    module, commands, _ = make_rig()
    module.config.deadman_timeout_s = 0.05
    holds: list[int] = []
    disables: list[int] = []
    original_hold = module._arm.hold
    original_disable = module._arm.disable

    def tracked_hold():
        holds.append(threading.get_ident())
        return original_hold()

    def tracked_disable():
        disables.append(threading.get_ident())
        return original_disable()

    monkeypatch.setattr(module._arm, "hold", tracked_hold)
    monkeypatch.setattr(module._arm, "disable", tracked_disable)
    commands.publish(JointState(name=["arm/joint1"], position=[0.2]))
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    assert holds and all(owner == module._reader.ident for owner in holds)
    module.stop()
    assert disables and all(owner == module._reader.ident for owner in disables)
    assert module._arm is None


def test_stop_is_bounded_when_the_hardware_lock_is_held(make_rig, monkeypatch):
    module, _, _ = make_rig()
    monkeypatch.setattr(common, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.05)
    entered, release = threading.Event(), threading.Event()
    original_read = module._read_state

    def blocked_read():
        entered.set()
        assert release.wait(2)
        return original_read()

    monkeypatch.setattr(module, "_read_state", blocked_read)
    assert entered.wait(2)
    done = threading.Event()
    stopper = threading.Thread(target=lambda: (module.stop(), done.set()), daemon=True)
    stopper.start()
    try:
        # The read holds the hardware lock: shutdown returns at its bound
        # instead of waiting for that lock without a limit.
        assert done.wait(1.0)
    finally:
        release.set()
        stopper.join(timeout=2)
    # The owner closes the hardware itself once its SDK call returns.
    wait_until(lambda: not module._reader.is_alive(), timeout=2)
    assert module._arm is None
    assert not module.status().connected


def test_commands_after_stop_are_dropped_not_queued(make_rig, boundary):
    module, commands, _ = make_rig()
    written = len(boundary[0].groups["left_arm"].commands)
    module.stop()
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    time.sleep(0.05)
    assert module._pending == {}
    assert len(boundary[0].groups["left_arm"].commands) == written
