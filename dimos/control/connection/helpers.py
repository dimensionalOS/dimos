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

"""Shared maths for robot drivers.

Four small things that several drivers each need:

  - work out where a robot has got to, from how fast it has been going
  - work out how fast it was going, from where it has got to
  - how hard to push a joint to get it where it should be
  - wait for something to happen, but give up if it takes too long

None of it holds state or talks to hardware, so any driver can use any of it.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import time


def wrap_to_pi(angle: float) -> float:
    """Fold an angle into the range -pi to +pi.

    Angles a whole turn apart point the same way, so 0, 2*pi and -2*pi all
    come back as 0.

    Exactly backwards comes back as +pi, never -pi. Both mean the same
    direction, but always giving the same one stops a heading appearing to
    flip sign every time the robot turns past it.

    Args:
        angle: Any angle, in radians.

    Returns:
        The same direction, in radians, above -pi and up to and including +pi.
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
    """Work out where a robot gets to, from where it is and how fast it is going.

    The opposite of ``se2_body_twist``.

    The speed is given in the robot's own terms -- forwards, sideways, turning
    -- while the position is in the world frame.

    Args:
        x: Where the robot is now, in metres.
        y: Where the robot is now, in metres, at right angles to x.
        yaw: Which way it is facing, in radians. 0 faces along x, and the
            angle grows as it turns left.
        vx: How fast it is driving forwards, in m/s. Negative is backwards.
        vy: How fast it is sliding to its left, in m/s. Always 0 for a robot
            that cannot move sideways, such as a car.
        wz: How fast it is turning, in rad/s. Positive is to the left.
        dt: How long it moves for, in seconds. Zero or less gives the position
            back unchanged, so a repeated or out-of-order timestamp cannot
            make the robot appear to jump or run backwards.
        wrap: Keep the heading it returns between -pi and +pi. Pass False to
            let it keep counting, so a robot that has turned twice reads 12.6
            rather than 0. Use whichever the robot itself uses.

    Returns:
        Where it gets to, as ``(x, y, yaw)`` in metres and radians.
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
    """Work out how hard to push a joint to get it where it should be.

    The joint is pushed harder the further it is from where it should be, and
    braked the further its speed is from the speed it should be going at::

        kp * (how far off it is) + kd * (how far off its speed is) + tau_ff

    The speed is measured against ``dq_target``, not against zero. Braking
    towards zero instead would fight every commanded movement, treating any
    motion at all as something to damp out.

    Args:
        q_target: Where the joint should be, in radians.
        dq_target: How fast it should be moving, in rad/s. Pass 0 to hold it
            still.
        q: Where the joint actually is, in radians.
        dq: How fast it actually is moving, in rad/s.
        kp: Stiffness, in Nm per radian. Higher pulls harder towards
            ``q_target``.
        kd: Damping, in Nm per rad/s. Higher resists moving at the wrong
            speed.
        tau_ff: Extra torque added on regardless, in Nm. Used to cancel out a
            known force such as gravity. Pass 0.0 if there is none.

    Returns:
        How hard to push, in Nm.
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
