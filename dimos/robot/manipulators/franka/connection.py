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

"""A Franka Research 3 as a connection module, over Franka's pylibfranka (FCI)."""

from __future__ import annotations

from collections.abc import Callable
import ipaddress
import json
import queue
import subprocess
import sys
import threading
import time
from typing import Any

from pydantic import field_validator, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig,
    ConnectionModule,
    Values,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.franka.config import (
    FR3_DEFAULT_SPEED_FRACTION,
    FR3_FINGER_JOINT,
    FR3_VELOCITY_MAX,
    fr3_joint_names,
    fr3_model,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_READ_HZ = 100.0
# Joint targets are refused once the latest joint reading is older than this,
# in seconds: ten missed readings.
_READING_MAX_AGE_S = 10 / _READ_HZ
# The Hand is read on every 10th reading of the joints.
_GRIPPER_READ_EVERY = 10

# Joint collision thresholds, as in Franka's examples (example_common.setDefaultBehaviour).
_TORQUE_THRESHOLDS = ([20.0] * 7, [40.0] * 7)  # Nm, lower then upper

# How close (rad) the arm must come to a target for libfranka to call it reached.
_GOAL_TOLERANCE_RAD = 0.01

# Drift guard: while a session runs, each joint must stay between where it was
# when its latest target was sent and that target, give or take this (rad).
# Further out, the robot is stopped.
_DRIFT_TOLERANCE_RAD = 0.02

# An async session is a point-to-point motion: once the arm reaches its target
# and settles, the robot ends the session by itself. A target within this (rad)
# of where every joint is needs no new session.
_AT_TARGET_RAD = 0.01
# Not-following guard: with a session running, a joint this far (rad) from its
# target while the arm stands still for _STALL_S seconds faults the connection.
_STALL_RAD = 0.02
_STALL_SPEED = 0.005  # rad/s, "standing still"
_STALL_S = 0.3

_GRIPPER_SPEED = 0.05  # m/s
# Finger targets closer than this (m) to the last one sent are not sent again.
_GRIPPER_DEADBAND = 0.0005
# A target this much (m, total opening) narrower than the Hand is a close, sent as a
# force grasp; anything else, even fractionally narrower, is a move. A homed Hand
# sits at ~80.1 mm, so a hold at 80.0 mm must not become a grasp: a grasp with
# nothing between the fingers was seen to close them fully.
_GRASP_MIN_CLOSE = 0.005
# Sessions the robot ended short of their target (in no stop) that may be restarted
# within this many seconds; one more is a fault.
_RESTARTS_MAX = 3
_RESTART_WINDOW_S = 5.0


class FrankaError(RuntimeError):
    """The arm, or libfranka, refused; the message says what and why."""


class FrankaConfig(ConnectionConfig):
    """Where the FR3 is, what is on its flange and how fast it may go.

    Attributes:
        ip: The address of Control's LAN port (FCI), e.g. "172.16.0.2". FCI
            must be activated in Desk, with the arm in Execution mode.
        hand: Whether the Franka Hand is on the flange. Its joint is
            ``fr3_finger_joint1``: 0.0 closed to 0.04 m open, per finger.
        source: The robot's name, in front of each joint's name, e.g. "arm"
            gives "arm/fr3_joint1".
        speed_fraction: The share of each joint's top speed the robot's own
            planner may use, in (0, 1]. The default model's velocity limits are
            scaled by the same share, so plans keep pace with the arm.
        grasp_force: The force (N) the Hand holds an object with on closing.
        jump_limit_s: How far a joint target may be from the one before it
            (from where the joint is, when no motion is running), in seconds at
            the joint's (scaled) top speed. A target further away is refused.
        lag_limit_s: How far a joint target may be from where the joint is,
            in the same units. The robot's own planner accelerates more gently
            than a streamed trajectory, so the arm legitimately trails the
            targets; this bound only catches targets far from the arm.
        contact_force: The tool's collision thresholds, lower then upper, in N
            (and Nm for the torques). Franka's examples use 10 / 20; with the
            wrist camera and its cable, the FR3's own estimate read up to 12.6 N
            with nothing touching while moving through a folded pose near the
            base, and a 20 N upper threshold stopped a pick there.
        enforce_realtime: Make libfranka refuse to run without a real-time
            kernel. Off by default: the async position control used here plans
            on the robot, and Franka's own example for it runs without one.
        hand_process: Drive the Hand from a program of its own
            (``franka.hand_process``). pylibfranka's
            Gripper calls hold Python's GIL for as long as the fingers move
            (about a second), which stops the arm's reading thread for that
            long: joint targets are then refused as stale and a running control
            session can end. Off only for tests on a fake pylibfranka.
        hold_on_short_end: When the robot ends a motion short of its target
            with no reflex or stop ("Unexpected reply to a Move command"), hold
            where the arm is instead of faulting: the next target within one jump
            starts a fresh session, and the caller re-plans from where the arm
            is. Three within 5 s still fault. Off by default (such an ending is a
            fault); one FR3 needed it after it began ending most motions
            1-10 deg short.
    """

    ip: str
    hand: bool = True
    source: str = "arm"
    speed_fraction: float = FR3_DEFAULT_SPEED_FRACTION
    grasp_force: float = 20.0
    jump_limit_s: float = 0.1
    lag_limit_s: float = 1.0
    contact_force: tuple[float, float] = (20.0, 40.0)
    enforce_realtime: bool = False
    hand_process: bool = True
    hold_on_short_end: bool = False

    @field_validator("ip")
    @classmethod
    def _ip_is_an_address(cls, ip: str) -> str:
        ipaddress.ip_address(ip)
        return ip

    @field_validator("speed_fraction")
    @classmethod
    def _speed_fraction_in_range(cls, fraction: float) -> float:
        if not 0.0 < fraction <= 1.0:
            raise ValueError(f"speed_fraction must be in (0, 1], got {fraction}")
        return fraction

    @model_validator(mode="after")
    def _model_from_hand(self) -> Self:
        if self.model is None:
            self.model = fr3_model(
                hand=self.hand, robot=self.source, speed_fraction=self.speed_fraction
            )
        if not self.joint_commands:
            names = fr3_joint_names(self.source)
            if self.hand:
                names.append(f"{self.source}{SEPARATOR}{FR3_FINGER_JOINT}")
            self.joint_commands = {name: (Interface.POSITION,) for name in names}
        for name, interfaces in self.joint_commands.items():
            if interfaces != (Interface.POSITION,):
                raise ValueError(
                    f"FrankaConnection takes positions only, not {interfaces} on {name}"
                )
        return self


class FrankaConnection(ConnectionModule):
    """A Franka Research 3 driven over FCI at ``config.ip``.

    Joint targets come in on ``position_command`` in rad, and the Hand's
    finger in m. Targets go to libfranka's AsyncPositionControlHandler: the
    robot plans toward each one itself, so they can arrive at the
    coordinator's rate rather than libfranka's usual 1 kHz. Franka marks that
    API experimental. Closing the Hand grasps with ``grasp_force``; opening
    moves it. ``joint_state`` goes out about 100 times a second, the Hand in
    a message of its own. Once the arm stops on a reflex (a collision, a limit)
    or is stopped from outside DimOS, it takes no command until
    ``clear_errors()``. Joint targets are refused while the latest joint
    reading is over 0.1 s old.
    """

    config: FrankaConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._joints = fr3_joint_names(self.config.source)
        self._finger = (
            f"{self.config.source}{SEPARATOR}{FR3_FINGER_JOINT}" if self.config.hand else None
        )
        self._libfranka: Any = None
        self._robot: Any = None
        self._handler: Any = None
        # Held around every call into the robot: the reading thread and the
        # command thread both use it.
        self._robot_lock = threading.Lock()
        # Set when the arm stops on its own or a command fails at the arm;
        # only clear_errors() clears it.
        self._fault: str | None = None
        self._state: Any = None
        # The joints' latest measured positions (rad), replaced whole by the
        # reading thread.
        self._measured: list[float] = []
        # time.monotonic() when _measured was read.
        self._measured_at = float("-inf")
        self._target: list[float] | None = None
        # Each joint's (low, high) bound for the drift guard while a session runs.
        self._band: list[tuple[float, float]] | None = None
        # time.monotonic() since when the arm has stood still away from its target.
        self._stalled_since: float | None = None
        # time.monotonic() of each recent session restarted after ending short.
        self._restarts: list[float] = []
        # How far each joint's target may be from where it is, in rad.
        self._reach = [
            self.config.jump_limit_s * top * self.config.speed_fraction for top in FR3_VELOCITY_MAX
        ]
        # How far each joint's target may be from where the joint is, in rad.
        self._lag = [
            self.config.lag_limit_s * top * self.config.speed_fraction for top in FR3_VELOCITY_MAX
        ]
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        # The Hand's move() and grasp() wait for the fingers, so a worker of its
        # own (a process on hardware, see FrankaConfig.hand_process) owns the
        # Gripper: widths go to it on one queue, its readings come back on another.
        self._hand: _HandWorker | None = None
        self._finger_sent: float | None = None
        self._width: float | None = None
        self._hand_listener: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="FrankaConnection-read"
        )
        self._reader.start()
        if self._hand is not None:
            self._hand_listener = threading.Thread(
                target=self._hand_listen, daemon=True, name="FrankaConnection-hand"
            )
            self._hand_listener.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._hand is not None:
            self._hand.stop()
        for thread in (self._reader, self._hand_listener):
            if thread is not None:
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        self._end_control()
        self._robot = None
        self._hand = None

    @rpc
    def clear_errors(self) -> None:
        """Recover the arm from a reflex or a stop and let it take commands again.

        Does what Desk's recovery does (libfranka's automatic error recovery).
        Position control starts again with the next joint target. Call it once
        whatever stopped the arm, such as a collision or a joint at its limit,
        is dealt with.

        Raises:
            FrankaError: If the arm refuses.
        """
        with self._hardware_lock:
            if self._robot is None:
                raise FrankaError("not connected")
            self._end_control()
            try:
                with self._robot_lock:
                    self._robot.automatic_error_recovery()
            except Exception as error:
                raise FrankaError(f"error recovery failed: {error}") from error
            self._fault = None

    def connect(self) -> None:
        ip = self.config.ip
        self._libfranka = _import_libfranka()
        realtime = (
            self._libfranka.RealtimeConfig.kEnforce
            if self.config.enforce_realtime
            else self._libfranka.RealtimeConfig.kIgnore
        )
        try:
            robot = self._libfranka.Robot(ip, realtime)
            state = robot.read_once()
        except Exception as error:
            raise ConnectionError(
                f"no FR3 answers over FCI at {ip} (is FCI activated in Desk?): {error}"
            ) from error
        self._robot = robot
        self._take_reading(state)
        try:
            lower, upper = self.config.contact_force
            robot.set_collision_behavior(
                _TORQUE_THRESHOLDS[0], _TORQUE_THRESHOLDS[1], [lower] * 6, [upper] * 6
            )
            if self.config.hand:
                self._hand = _HandWorker(
                    ip, self.config.grasp_force, self._libfranka, self.config.hand_process
                )
                self._width = self._hand.first_width()
        except Exception:
            if self._hand is not None:
                self._hand.stop()
            self._robot = self._hand = None
            raise
        # No control session yet: the first joint target starts one.
        if state.robot_mode.name in ("Reflex", "UserStopped"):
            self._set_fault(f"the arm is in {state.robot_mode.name} at start")

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise FrankaError(self._fault)
        targets = [values.get(f"{joint}/{Interface.POSITION}") for joint in self._joints]
        # The Hand first: a refused arm target must not also drop a Hand command (with
        # the arm held short of an unreachable target, the Hand would never open).
        self._write_finger(values)
        if any(target is not None for target in targets):
            age = time.monotonic() - self._measured_at
            if age > _READING_MAX_AGE_S:
                raise FrankaError(
                    f"the joints were last read {age:.2f} s ago; joint targets need a "
                    f"reading at most {_READING_MAX_AGE_S} s old"
                )
            # A joint left out stays where it is.
            measured = self._measured
            angles = [m if t is None else t for t, m in zip(targets, measured, strict=True)]
            if self._handler is not None and self._target:
                self._check_jump(angles, self._target, "the previous target", self._reach)
            else:
                self._check_jump(angles, measured, "where it is (no motion running)", self._reach)
            self._check_jump(angles, measured, "where it is", self._lag)
            if self._handler is None:
                if all(abs(a - m) <= _AT_TARGET_RAD for a, m in zip(angles, measured, strict=True)):
                    # Already there (e.g. holding after a finished motion): starting a
                    # session for it would only have the robot end it again.
                    self._target = angles
                    return
                self._begin_control()
            self._send_target(angles)

    def _write_finger(self, values: Values) -> None:
        if self._finger is not None:
            finger = values.get(f"{self._finger}/{Interface.POSITION}")
            if finger is not None and (
                self._finger_sent is None or abs(finger - self._finger_sent) > _GRIPPER_DEADBAND
            ):
                logger.info(f"FrankaConnection: finger target {finger * 1000:.1f} mm received")
                if self._hand is not None:
                    self._hand.goals.put(2.0 * finger)  # both fingers move: width is twice one
                self._finger_sent = finger

    def halt(self) -> None:
        # The robot keeps heading for its last target; make that where it is now.
        if self._handler is not None and self._fault is None and self._target is not None:
            if self._measured:
                self._send_target(list(self._measured))

    def _begin_control(self) -> None:
        """Start a session of async joint position control, holding the arm where it is.

        While a session starts, libfranka sends motion commands whose joint
        positions are all zero, and an async session takes them as its target:
        left alone, the arm heads for the all-zero pose. So the measured pose
        is sent as the target straight away, and the drift guard watches.
        """
        lib = self._libfranka
        config = lib.AsyncPositionControlHandler.Configuration(
            maximum_joint_velocities=[top * self.config.speed_fraction for top in FR3_VELOCITY_MAX],
            goal_tolerance=_GOAL_TOLERANCE_RAD,
        )
        with self._robot_lock:
            result = lib.AsyncPositionControlHandler.configure(self._robot, config)
            if result.error_message is not None:
                raise FrankaError(f"position control did not start: {result.error_message}")
            hold = [float(q) for q in self._robot.read_once().q]
            sent = result.handler.set_joint_position_target(
                lib.AsyncPositionControlHandler.JointPositionTarget(joint_positions=hold)
            )
            self._handler = result.handler
        if not sent.was_successful:
            self._set_fault(f"the arm refused its hold target: {sent.error_message}")
            self._end_control()
            raise FrankaError(self._fault)
        self._target = hold
        self._band = [(q - _DRIFT_TOLERANCE_RAD, q + _DRIFT_TOLERANCE_RAD) for q in hold]

    def _end_control(self) -> None:
        """End the session by cancelling it: libfranka sends the robot a stop.

        Not ``stop_control()``: that finishes the motion with all-zero joint
        positions, which an async session can take as a target.
        """
        with self._robot_lock:
            handler, self._handler = self._handler, None
            self._band = None
            # Dropping the last reference destroys the session, which cancels the motion.
            del handler

    def _send_target(self, angles: list[float]) -> None:
        if self._handler is None:
            raise FrankaError("position control is not running; call clear_errors()")
        target = self._libfranka.AsyncPositionControlHandler.JointPositionTarget(
            joint_positions=angles
        )
        with self._robot_lock:
            result = self._handler.set_joint_position_target(target)
        if not result.was_successful:
            self._set_fault(f"the arm refused a target: {result.error_message}")
            self._end_control()
            raise FrankaError(self._fault)
        measured = self._measured
        self._band = [
            (min(m, t) - _DRIFT_TOLERANCE_RAD, max(m, t) + _DRIFT_TOLERANCE_RAD)
            for m, t in zip(measured, angles, strict=True)
        ]
        self._target = angles

    def _check_drift(self) -> None:
        """Stop the robot if a joint leaves its band while a session runs."""
        band = self._band
        if band is None or self._fault is not None:
            return
        for joint, q, (low, high) in zip(self._joints, self._measured, band, strict=True):
            if not low <= q <= high:
                self._set_fault(
                    f"drift guard: {joint} at {q:.3f} rad, outside {low:.3f} to {high:.3f}"
                )
                self._end_control()
                return

    def _check_jump(
        self, angles: list[float], reference: list[float], what: str, reach: list[float]
    ) -> None:
        """Refuse joint targets (rad) further than ``reach`` from ``reference`` (rad)."""
        for joint, target, ref, limit in zip(self._joints, angles, reference, reach, strict=True):
            if abs(target - ref) > limit:
                raise FrankaError(
                    f"{joint} told {target:.3f} rad, {abs(target - ref):.3f} rad from {what}; "
                    f"at most {limit:.3f} rad away is taken"
                )

    def _set_fault(self, message: str) -> None:
        if self._fault is None:
            self._fault = f"{message}; call clear_errors() once it is safe to move"
            self._report(self._fault)

    def _take_reading(self, state: Any) -> None:
        self._state = state
        self._measured = [float(q) for q in state.q]
        self._measured_at = time.monotonic()

    def _watch_session(self) -> None:
        """Ask the session how it is doing, through the session, as Franka's example does.

        Reading through the session is what surfaces the robot ending it. Once
        the arm reaches its target the robot ends the motion, and libfranka
        reports the robot's reply to it as an error; that is a finished
        session, not a fault. Ended anywhere else, it is a fault.
        """
        handler = self._handler
        if handler is None:
            return
        with self._robot_lock:
            feedback = handler.get_target_feedback()
        if feedback.status.name != "kAborted" and feedback.error_message is None:
            return
        target = self._target
        at_target = target is not None and all(
            abs(t - m) <= _AT_TARGET_RAD * 2 for t, m in zip(target, self._measured, strict=True)
        )
        if at_target:
            logger.debug(
                f"FrankaConnection: session ended at its target ({feedback.error_message})"
            )
            self._end_control()
            return
        # The robot can also end a motion short of its target with no reflex and
        # nothing in Desk (one FR3 did so on most moves, 1-10 deg short, while trailing
        # a streamed trajectory). With hold_on_short_end and the arm in no stop, end the
        # session without a fault: the arm holds where it is. A next target within one
        # jump starts a fresh session that finishes the move; one further away is
        # refused by the jump check as ever, and a new plan from where the arm is
        # finishes it. Too many in a row is a fault as before.
        mode = self._state.robot_mode.name if self._state is not None else ""
        now = time.monotonic()
        self._restarts = [t for t in self._restarts if now - t < _RESTART_WINDOW_S]
        if (
            self.config.hold_on_short_end
            and mode not in ("Reflex", "UserStopped")
            and "Unexpected reply" in str(feedback.error_message)
            and target is not None
            and len(self._restarts) < _RESTARTS_MAX
        ):
            self._restarts.append(now)
            away = max(abs(t - m) for t, m in zip(target, self._measured, strict=True))
            logger.warning(
                f"FrankaConnection: the robot ended the motion {away:.3f} rad short of its "
                "target, in no stop; holding there until a target starts a new session"
            )
            self._end_control()
            return
        self._set_fault(f"the control session ended away from its target: {feedback.error_message}")
        self._end_control()

    def _check_following(self, state: Any) -> None:
        """Fault if, with a session running, the arm stands still short of its target."""
        target = self._target
        if self._handler is None or target is None or self._fault is not None:
            self._stalled_since = None
            return
        away = max(abs(t - m) for t, m in zip(target, self._measured, strict=True))
        still = max(abs(float(v)) for v in state.dq) < _STALL_SPEED
        if away <= _STALL_RAD or not still:
            self._stalled_since = None
            return
        now = time.monotonic()
        if self._stalled_since is None:
            self._stalled_since = now
        elif now - self._stalled_since > _STALL_S:
            self._set_fault(
                f"the arm is not following: {away:.3f} rad from its target and not moving"
            )
            self._end_control()

    def _watch_for_stop(self, state: Any) -> None:
        """Fault on a reflex or an outside stop; libfranka's own error list says why."""
        mode = state.robot_mode.name
        if mode not in ("Reflex", "UserStopped") or self._fault is not None:
            return
        errors = state.current_errors if state.current_errors else state.last_motion_errors
        self._set_fault(f"the arm stopped ({mode}){f': {errors}' if errors else ''}")
        # The robot has already ended the motion; clear_errors() starts a new one.
        self._end_control()

    def _read_loop(self) -> None:
        period = 1.0 / _READ_HZ
        due = time.monotonic()
        count = 0
        while not self._stop_reading.wait(max(0.0, due - time.monotonic())):
            due = max(due + period, time.monotonic())
            try:
                robot = self._robot
                if robot is None:
                    continue
                with self._robot_lock:
                    state = robot.read_once()
                self._take_reading(state)
                self._watch_for_stop(state)
                self._watch_session()
                self._check_drift()
                self._check_following(state)
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        name=list(self._joints),
                        position=list(self._measured),
                        velocity=[float(v) for v in state.dq],
                        effort=[float(t) for t in state.tau_J],
                    )
                )
                width = self._width
                if (
                    self._finger is not None
                    and width is not None
                    and count % _GRIPPER_READ_EVERY == 0
                ):
                    # The Hand reports no speed or effort, so it has a message of its own.
                    self.joint_state.publish(
                        JointState(ts=time.time(), name=[self._finger], position=[width / 2.0])
                    )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
            count += 1

    def _hand_listen(self) -> None:
        hand = self._hand
        while hand is not None and not self._stop_reading.is_set():
            try:
                kind, value = hand.events.get(timeout=0.2)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                return
            if kind == "width":
                self._width = float(value)
            elif kind == "info":
                logger.info(f"FrankaConnection: {value}")
            else:
                self._report(str(value))


class _HandWorker:
    """The Franka Hand, worked from a queue of widths by a process or a thread.

    pylibfranka's Gripper calls hold the GIL while the fingers move, so on
    hardware the Gripper lives in a process of its own; tests on a fake
    pylibfranka use a thread. Readings and messages come back on ``events``.
    """

    def __init__(self, ip: str, grasp_force: float, libfranka: Any, in_process: bool) -> None:
        if in_process:
            self._proc: subprocess.Popen[str] | None = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "dimos.robot.manipulators.franka.hand_process",
                    ip,
                    str(grasp_force),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self.goals: Any = _Pipe(self._proc)
            self.events: queue.Queue[Any] = queue.Queue()
            self._stop = threading.Event()
            self._runner = threading.Thread(
                target=_relay_lines,
                args=(self._proc, self.events),
                daemon=True,
                name="FrankaConnection-hand",
            )
        else:
            self._proc = None
            self.goals = queue.Queue()
            self.events = queue.Queue()
            self._stop = threading.Event()
            first = libfranka.Gripper(ip)  # opened here, so a failure fails connect()

            def open_gripper(made: list[Any] = [first]) -> Any:  # noqa: B006
                return made.pop() if made else libfranka.Gripper(ip)

            self._runner = threading.Thread(
                target=_hand_loop,
                args=(open_gripper, grasp_force, self.goals, self.events, self._stop),
                daemon=True,
                name="FrankaConnection-hand",
            )
        self._runner.start()

    def first_width(self, timeout: float = 15.0) -> float:
        """The Hand's opening (m) once it answers; raises if it does not."""
        try:
            kind, value = self.events.get(timeout=timeout)
        except queue.Empty as error:
            raise ConnectionError("the Franka Hand did not answer") from error
        if kind != "width":
            raise ConnectionError(f"the Franka Hand did not answer: {value}")
        return float(value)

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            try:
                assert self._proc.stdin is not None
                self._proc.stdin.close()  # the Hand's program stops at the end of its input
                self._proc.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            except (OSError, subprocess.TimeoutExpired):
                self._proc.kill()
        self._runner.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


class _Pipe:
    """``put`` sends one width to the Hand's program."""

    def __init__(self, proc: subprocess.Popen[str]) -> None:
        self._proc = proc

    def put(self, width: float) -> None:
        try:
            assert self._proc.stdin is not None
            self._proc.stdin.write(json.dumps(width) + "\n")
            self._proc.stdin.flush()
        except (OSError, ValueError):
            pass  # it has stopped; the listener reports why


def _relay_lines(proc: subprocess.Popen[str], events: queue.Queue[Any]) -> None:
    """Pass the Hand program's readings on, and say so if it stops."""
    assert proc.stdout is not None
    for line in proc.stdout:
        try:
            kind, value = json.loads(line)
        except ValueError:
            continue
        events.put((kind, value))
    if proc.poll() not in (None, 0):
        events.put(("error", f"the Franka Hand's process stopped (exit {proc.returncode})"))


def _hand_loop(
    open_gripper: Callable[[], Any], grasp_force: float, goals: Any, events: Any, stop: Any
) -> None:
    """Take widths off ``goals`` and move the Hand there, the latest first; report its
    opening on ``events`` every 0.1 s, and anything that goes wrong."""
    try:
        gripper = open_gripper()
        events.put(("width", float(gripper.read_once().width)))
    except Exception as error:
        events.put(("error", f"Franka Hand failed: {type(error).__name__}: {error}"))
        return
    wanted: float | None = None  # the latest width asked for, kept to resend after a reconnect
    sent: float | None = None
    while not stop.is_set():
        try:
            wanted = goals.get(timeout=1.0 / _READ_HZ * _GRIPPER_READ_EVERY)
            while True:  # only the newest goal matters
                wanted = goals.get_nowait()
        except queue.Empty:
            pass
        except (EOFError, OSError):
            return
        goal = wanted
        try:
            if goal is not None and goal != sent:
                sent = goal
                hand = gripper.read_once()
                width = max(0.0, min(float(hand.max_width), goal))
                closing = width < float(hand.width) - _GRASP_MIN_CLOSE
                events.put(
                    (
                        "info",
                        f"Hand to {width * 1000:.1f} mm ({'grasp' if closing else 'move'}), "
                        f"now {float(hand.width) * 1000:.1f} mm",
                    )
                )
                if closing:
                    # grasp() holds what is between the fingers with a force, where
                    # move() would fail once they are blocked. Epsilons of the full
                    # stroke accept any width it stops at.
                    gripper.grasp(
                        width,
                        _GRIPPER_SPEED,
                        grasp_force,
                        epsilon_inner=float(hand.max_width),
                        epsilon_outer=float(hand.max_width),
                    )
                else:
                    gripper.move(width, _GRIPPER_SPEED)
            events.put(("width", float(gripper.read_once().width)))
        except Exception as error:
            events.put(("error", f"Franka Hand failed: {type(error).__name__}: {error}"))
            # A dropped link (another client connecting to the Hand, a cable blip)
            # leaves this Gripper dead: open a fresh one and resend.
            if "Net" in str(error) or "Network" in type(error).__name__:
                try:
                    gripper = open_gripper()
                    sent = None
                    events.put(("info", "reconnected to the Franka Hand"))
                except Exception as again:
                    events.put(("error", f"Franka Hand reconnect failed: {again}"))
                    stop.wait(1.0)


def _import_libfranka() -> Any:
    """pylibfranka, imported here so blueprints load without it installed."""
    try:
        import pylibfranka
    except ImportError as error:
        raise ImportError(
            "FrankaConnection needs Franka's pylibfranka >= 0.18 (robot system 5.9 or newer): "
            "pip install pylibfranka"
        ) from error
    return pylibfranka
