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

"""The arithmetic a vendor hook needs and should not be writing again.

Everything here is pure, is called from a vendor's own hooks, and is unknown to
``ConnectedHardware``. The reason these live together is that more than one vendor
needs them, and the vendors that need each one are not the same set.
SE(2) integration is shared by a chassis and a mock, PD emulation by a simulator alone.

"""

from __future__ import annotations

from collections.abc import Callable
import math
import time


def wrap_to_pi(angle: float) -> float:
    """``angle`` folded into ``(-pi, pi]``.

    The half-open end matters: a heading of exactly -pi and one of +pi are the
    same heading, and picking one of them keeps a wrapped yaw from flipping
    sign every time it passes behind the robot.
    """
    wrapped = math.remainder(angle, math.tau)
    return math.pi if wrapped == -math.pi else wrapped


def integrate_planar_twist(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    vy: float,
    wz: float,
    dt: float,
    *,
    wrap: bool = True,
) -> tuple[float, float, float]:
    """One step of a body-frame twist integrated into a world pose.

    ``wrap`` is the disagreement between the existing copies made explicit.
    A wrapped yaw is what a consumer comparing headings wants, an unwrapped one
    is what a consumer counting turns wants. Whichever a base does, it says so
    in its description's ``yaw_convention``.

    A ``dt`` of zero or less returns the pose unchanged rather than integrating
    backwards. Timestamps do go backwards -- a replayed bag, a resynchronized
    clock -- and the pose should stand still when they do.

    Args:
        x: World x of the pose to advance, in metres.
        y: World y, in metres.
        yaw: World heading, in radians.
        vx: Body-frame forward velocity, in m/s.
        vy: Body-frame left velocity, in m/s.
        wz: Yaw rate, in rad/s.
        dt: Step, in seconds. Zero or less is a no-op.
        wrap: Whether to fold the new yaw into ``(-pi, pi]``.

    Returns:
        The advanced ``(x, y, yaw)``.
    """
    if dt <= 0.0:
        return (x, y, yaw)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    advanced_yaw = yaw + wz * dt
    return (
        x + (cos_yaw * vx - sin_yaw * vy) * dt,
        y + (sin_yaw * vx + cos_yaw * vy) * dt,
        wrap_to_pi(advanced_yaw) if wrap else advanced_yaw,
    )


def se2_body_twist(
    prev: tuple[float, float, float],
    curr: tuple[float, float, float],
    dt: float,
    *,
    wrap: bool = True,
) -> tuple[float, float, float]:
    """Work out how fast a robot was moving, from two positions and a time.

    The reverse of ``integrate_planar_twist``. Give it where the robot was,
    where it is now, and how long that took, and it reports the speed from the
    robot's own point of view: how fast it was driving forwards, sliding
    sideways, and turning.

    That last part is the whole job. Knowing the robot moved two metres
    north-east does not say whether it drove forwards or slid sideways -- that
    depends on which way it was pointing while it moved.

    Use it when a robot reports its position but not its speed.

    Args:
        prev: Where the robot was, as ``(x, y, yaw)`` in metres and radians.
        curr: Where it is now, in the same form.
        dt: Seconds between the two readings. Must be positive.
        wrap: Which convention ``yaw`` uses in ``prev`` and ``curr``. ``True``
            if the heading stays within one turn, jumping from ``+pi`` to
            ``-pi`` as it passes. ``False`` if it keeps counting up, so two
            full turns read as ``12.6``. A wrong value gives badly wrong
            answers: with a counting heading, wrapping silently drops whole
            turns, and a robot turning four radians in a second is reported as
            turning 2.28 rad/s the other way.

    Returns:
        ``(vx, vy, wz)``: forwards speed in m/s, leftwards speed in m/s, and
        turn rate in rad/s.

    Raises:
        ValueError: If ``dt`` is zero or negative. There is no speed to report
            over no time at all.

    The robot is assumed to have curved smoothly between the two positions,
    which holds when they are close together in time. The more it turned
    between them, the rougher the forwards and sideways figures get; the turn
    rate stays exact either way.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive to differentiate a pose, got {dt}")
    prev_x, prev_y, prev_yaw = prev
    curr_x, curr_y, curr_yaw = curr

    d_yaw = curr_yaw - prev_yaw
    if wrap:
        d_yaw = wrap_to_pi(d_yaw)
    # Measure against the heading halfway through the step rather than the one
    # at either end: over a step where the robot also turned, that is the
    # direction it spent most of the step pointing nearest to.
    mid_yaw = prev_yaw + d_yaw / 2.0
    cos_mid, sin_mid = math.cos(mid_yaw), math.sin(mid_yaw)
    dx, dy = curr_x - prev_x, curr_y - prev_y
    return (
        (cos_mid * dx + sin_mid * dy) / dt,
        (-sin_mid * dx + cos_mid * dy) / dt,
        d_yaw / dt,
    )


def pd_torque(
    q_target: float,
    dq_target: float,
    q: float,
    dq: float,
    kp: float,
    kd: float,
    tau_ff: float,
) -> float:
    """The torque a PD joint with feedforward should be producing.

    ``kp (q_target - q) + kd (dq_target - dq) + tau_ff``.

    Args:
        q_target: Commanded position.
        dq_target: Commanded velocity.
        q: Measured position.
        dq: Measured velocity.
        kp: Proportional gain.
        kd: Derivative gain.
        tau_ff: Feedforward torque.

    Returns:
        The torque, in the units kp, kd and tau_ff were given in.
    """
    return kp * (q_target - q) + kd * (dq_target - dq) + tau_ff


def await_state(
    predicate: Callable[[], bool],
    timeout_s: float,
    *,
    poll_s: float = 0.01,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> bool:
    """Wait for something to become true, and give up if it takes too long.

    Calls ``predicate`` over and over until it returns True or the time runs
    out. It is checked once before any waiting, so a condition that already
    holds returns immediately and costs nothing.

    Use it for waits that must not hang: an arm moving into position, a motor
    controller finishing its start-up. A wait with no timeout will eventually
    block the robot's whole driver.

    Args:
        predicate: Called repeatedly to test whether the thing has happened.
            Should return quickly and must not block.
        timeout_s: How long to keep trying, in seconds. Zero or negative still
            gets one attempt.
        poll_s: How long to wait between attempts, in seconds. The last wait is
            shortened so the total never runs past ``timeout_s``.
        clock: Returns the current time in seconds. Only replaced in tests, so
            a timeout can be exercised without waiting out the timeout.
        sleep: Waits for the given number of seconds. Replaced alongside
            ``clock``.

    Returns:
        True if the condition came true in time, False if the time ran out.

    Raises:
        ValueError: If ``poll_s`` is zero or negative, which would spin the CPU
            at full speed instead of waiting.
    """
    if poll_s <= 0.0:
        raise ValueError(f"poll_s must be positive, got {poll_s}")
    deadline = clock() + timeout_s
    while True:
        if predicate():
            return True
        remaining = deadline - clock()
        if remaining <= 0.0:
            return False
        sleep(min(poll_s, remaining))
