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
``ConnectedHardware``. It is not a base class and not a mixin: the reason these
live together is that more than one vendor needs them, and the vendors that
need each one are not the same set. SE(2) integration is shared by a chassis
and a mock, PD emulation by a simulator alone.

The audit found the existing copies of the first two already disagreeing about
yaw wrapping and the dt guard, which is the whole argument for one copy with
the disagreement turned into an argument.

Nothing here reads a clock except ``await_state``, and that only through
callables the caller can replace, so a test never has to sleep.
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

    Euler integration about the heading at the start of the step, which is what
    every copy of this in the tree already does and what a base publishing at
    50 Hz has the resolution for.

    ``wrap`` is the disagreement between the existing copies made explicit. The
    mock base wraps and R1Pro does not, and both are right for what they are:
    a wrapped yaw is what a consumer comparing headings wants, an unwrapped one
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
    """The body-frame twist that carried a base from ``prev`` to ``curr``.

    The world displacement is rotated into the body frame using the heading
    halfway through the step, not the heading at either end. Over a step where
    the base also turned, the midpoint is the heading it actually spent the
    step pointing near, so this is the twist that best explains the arc.

    That makes it the midpoint rule rather than the exact inverse of
    ``integrate_planar_twist``, which is Euler. Feeding one into the other
    round-trips exactly when ``wz`` is zero; when it is not, the recovered
    twist is the original rotated by ``-wz * dt / 2``, which is the correction
    the midpoint rule is making. Over one step of a base running at its state
    rate that is well under a degree.

    ``wrap`` must match the convention the two poses are in -- the same flag
    ``integrate_planar_twist`` took, and the ``yaw_convention`` the base
    declares in its description.

    On a wrapped yaw it has to be True and the shortest rotation is the best
    anyone can do: the samples are only known modulo 2 pi, and subtracting them
    raw would report a base that merely crossed +/-pi as spinning at
    ``2 pi / dt``.

    On a yaw that counts turns it has to be False. There the difference is
    already unambiguous, and wrapping would throw away whole turns: four
    radians over a second comes back as -2.28 rad/s, having quietly lost the
    2 pi. That only bites when more than half a turn happens between two
    samples -- impossible for a chassis at its state rate, ordinary after a
    dropped-sample gap or on a sparsely replayed bag. The heading is taken from
    the same difference, so a wrong flag tilts ``vx`` and ``vy`` too, not just
    ``wz``.

    Args:
        prev: The earlier ``(x, y, yaw)``.
        curr: The later ``(x, y, yaw)``.
        dt: Seconds between them.
        wrap: Whether these poses carry a wrapped yaw.

    Returns:
        The body-frame ``(vx, vy, wz)``.

    Raises:
        ValueError: If ``dt`` is zero or less. Unlike integration, there is no
            sensible twist to report over no time at all.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive to differentiate a pose, got {dt}")
    prev_x, prev_y, prev_yaw = prev
    curr_x, curr_y, curr_yaw = curr

    d_yaw = curr_yaw - prev_yaw
    if wrap:
        d_yaw = wrap_to_pi(d_yaw)
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

    The ``dq_target`` term is the one today's MuJoCo emulation leaves out: it
    computes ``kd (0 - dq)``, which damps against the world instead of tracking
    a commanded velocity, so a sim joint asked to move at a steady rate fights
    itself the whole way. Real PD firmware -- Unitree's among them -- includes
    it, so the sim was not emulating the thing it stood in for.

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
    """Poll ``predicate`` until it holds or ``timeout_s`` passes.

    For the bounded waits in lifecycle hooks: an arm converging on its park
    pose, an SDK finishing a bring-up. The point is the bound -- a hook that
    waits forever is a hook that hangs the whole connection, which is what a
    bare ``.result()`` does today.

    ``predicate`` is checked before the first sleep, so something already true
    costs nothing and a non-positive timeout still gets one look.

    ``clock`` and ``sleep`` are arguments because this is the only helper here
    that touches time. Injecting them is what lets the tests cover a timeout
    without taking as long as the timeout.

    Args:
        predicate: The condition to wait for. Called repeatedly, so it should
            be cheap and must not block.
        timeout_s: How long to keep trying, in seconds.
        poll_s: Gap between attempts, in seconds. The last gap is shortened so
            the wait does not overrun the deadline.
        clock: Monotonic clock, in seconds.
        sleep: How to wait, in seconds.

    Returns:
        True if the predicate held, False if the timeout passed first.

    Raises:
        ValueError: If ``poll_s`` is zero or less, which would spin.
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
