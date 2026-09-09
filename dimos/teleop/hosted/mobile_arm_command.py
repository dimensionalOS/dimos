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

"""Hosted command plane for a mobile bimanual robot.

Adds thumbstick base driving and a thumbstick-jogged torso height to
``ArmCommandModule``, so one operator session covers arms, chassis, and torso.
"""

from __future__ import annotations

import time
from typing import Any

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.hosted.arm_command import ArmCommandConfig, ArmCommandModule
from dimos.teleop.quest.quest_types import Buttons, Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A Joy gap longer than this is a link stall, not a slow frame; clamping keeps
# one late message from stepping the torso through a large jump.
_MAX_JOG_STEP_S = 0.2


class MobileArmCommandConfig(ArmCommandConfig):
    base_linear_speed: float = 0.3
    base_angular_speed: float = 0.4
    # Holding B (right secondary) scales both base speeds by this while
    # driving. It sits under the thumb that is not on the translation stick.
    # The sticks keep their full resolution at the normal speed, so the fast
    # gear is an explicit choice instead of a wider deadzone.
    boost_multiplier: float = 2.0
    base_deadzone: float = 0.15
    torso_deadzone: float = 0.25
    # Require the right stick to be clicked in before its Y axis jogs the
    # torso, so the same axis cannot be nudged by accident while driving.
    torso_requires_stick_click: bool = True
    torso_speed: float = 0.25
    # A joint-space height axis solved offline for the robot: drops ascending
    # in metres, and each joint's angle at those drops. Empty disables the jog.
    torso_fold_drops: list[float] = []
    torso_fold_joints: dict[str, list[float]] = {}
    # Engage on the side grip rather than the X/A face button. Holding a face
    # button parks the thumb away from the stick, which the operator needs for
    # driving; the grip is held by the hand that is already gripping.
    engage_on_grip: bool = True
    grip_engage_threshold: float = 0.5
    # Hold the same grips to drive. One deadman for the whole robot: let go and
    # the arms stop tracking and the base stops moving together, rather than
    # leaving a live chassis under a disengaged operator.
    require_deadman_to_drive: bool = True
    # Joint name -> radians. While the recover button is held, this is
    # re-issued as a joint goal so the arms walk back to it. Left empty the
    # feature is off, which keeps this module free of any one robot's pose.
    recover_pose: dict[str, float] = {}
    # Re-issue rate. Each goal is a one-point trajectory planned from the
    # measured position, so repeating it servos toward the pose under the
    # trajectory task's velocity limits rather than stepping.
    recover_hz: float = 5.0


class MobileArmCommandModule(ArmCommandModule):
    """Arm command plane plus holonomic base drive and torso height.

    Left stick translates the base and right stick X yaws it; holding B
    (right secondary) multiplies both base speeds by ``boost_multiplier``.
    Clicking the right stick in arms its Y axis as a straight vertical axis
    for the torso, commanded in joint space on its own stream so the arm
    solver cannot move it. Driving and the torso are both gated
    on the same grips that engage the arms, so releasing them stops the
    whole robot rather than leaving a live chassis behind.
    """

    config: MobileArmCommandConfig

    twist_command: Out[Twist]
    joint_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._torso_drop = 0.0
        self._torso_settling = False
        self._last_jog_t: float | None = None
        self._deadman_held = False
        self._recovering = False
        self._last_recover_t = 0.0
        self._boosting = False

    def _publish_safe_command(self) -> None:
        self.twist_command.publish(Twist.zero())

    @staticmethod
    def _deadzone(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    def _on_joy_bytes(self, data: bytes) -> bool:
        try:
            valid = super()._on_joy_bytes(data)
        except ValueError:
            self._publish_safe_command()
            raise
        if not valid:
            self._publish_safe_command()
            return False
        return True

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Drive base and torso from the control loop, not from Joy arrival.

        The operator page sends Joy in bursts rather than every frame, so
        publishing only on arrival starved the chassis velocity task (which
        timed out every 0.2s), left the teleoperation task's head target stale,
        and let the deadman lapse about a second after engaging. The base class
        calls this every control-loop tick with the latest controller state,
        which is the cadence these commands actually need.
        """
        self._publish_engage_buttons(left, right)
        self._publish_gripper_commands(left, right)
        self._track_deadman(left, right)
        self._publish_base_twist(left, right)
        self._publish_torso_target(right)
        self._publish_recovery(right)

    def _publish_recovery(self, right: QuestControllerState | None) -> None:
        """Walk the arms back to the configured pose while the button is held.

        The goal goes to the coordinator's joint-trajectory task, which sits
        above teleoperation in the priority order, so holding the button
        preempts the operator's hands rather than fighting them. Releasing it
        stops re-issuing and the last short move runs out.
        """
        pose = self.config.recover_pose
        if not pose:
            return
        wants = right is not None and right.primary and not self._estopped
        if not wants:
            if self._recovering:
                logger.info("recovery released")
            self._recovering = False
            return
        now = time.monotonic()
        if self._recovering and now - self._last_recover_t < 1.0 / max(self.config.recover_hz, 0.1):
            return
        if not self._recovering:
            logger.info("recovering to the configured pose while held")
        self._recovering = True
        self._last_recover_t = now
        names = list(pose)
        self.joint_command.publish(JointState(name=names, position=[pose[name] for name in names]))

    def _engaging(self, controller: QuestControllerState | None) -> bool:
        """Is this hand asking to be engaged?"""
        if controller is None:
            return False
        if not self.config.engage_on_grip:
            return controller.primary
        return controller.grip >= self.config.grip_engage_threshold

    def _publish_engage_buttons(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish Buttons with the engage control mirrored onto the primary bits.

        TeleopIKTask's deadman reads left_primary/right_primary, so switching
        the operator's engage control means presenting the chosen button there.
        The real face-button and grip bits are still reported untouched, so
        anything else reading them sees the physical truth.
        """
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        if self.config.engage_on_grip:
            buttons.left_primary = self._engaging(left)
            buttons.right_primary = self._engaging(right)
        self.teleop_buttons.publish(buttons)

    def _handle_engage(self) -> None:
        """Press-and-hold on the configured engage control.

        Mirrors the base implementation; only the button it reads differs. The
        E-STOP latch is still honoured, as in ArmCommandModule.
        """
        if not self.config.engage_on_grip:
            super()._handle_engage()
            return
        if self._estopped:
            for hand in Hand:
                if self._is_engaged[hand]:
                    self._disengage(hand)
            return
        for hand in Hand:
            wants = self._engaging(self._controllers.get(hand))
            if wants and not self._is_engaged[hand]:
                self._engage(hand)
            elif not wants and self._is_engaged[hand]:
                self._disengage(hand)

    def _track_deadman(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Track whether both engage controls are held.

        The torso height is absolute now, not an offset from an engagement, so
        it deliberately survives a release: letting go should stop the torso
        where it is, not drop the robot back to full extension.
        """
        self._deadman_held = self._engaging(left) and self._engaging(right) and not self._estopped

    def _publish_base_twist(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        if self._estopped or (self.config.require_deadman_to_drive and not self._deadman_held):
            self._publish_safe_command()
            return
        twist = Twist()
        twist.linear = Vector3(0.0, 0.0, 0.0)
        twist.angular = Vector3(0.0, 0.0, 0.0)
        deadzone = self.config.base_deadzone
        boost = self._boost_scale(right)
        linear_speed = self.config.base_linear_speed * boost
        angular_speed = self.config.base_angular_speed * boost
        # Left stick translates, right stick turns: the layout every shooter
        # has trained into the operator's thumbs.
        if left is not None:
            twist.linear.x = -self._deadzone(left.thumbstick.y, deadzone) * linear_speed
            twist.linear.y = -self._deadzone(left.thumbstick.x, deadzone) * linear_speed
        # Clicking the right stick claims it for torso height; yawing at the
        # same time makes the height impossible to place.
        if right is not None and not self._torso_armed(right):
            twist.angular.z = -self._deadzone(right.thumbstick.x, deadzone) * angular_speed
        self.twist_command.publish(twist)

    def _boost_scale(self, right: QuestControllerState | None) -> float:
        """Speed multiplier for the base: the boost while B is held, else 1."""
        multiplier = self.config.boost_multiplier
        boosting = right is not None and right.secondary and multiplier > 0.0
        if boosting != self._boosting:
            logger.info("base boost %s", "on" if boosting else "off")
            self._boosting = boosting
        return multiplier if boosting else 1.0

    def _torso_armed(self, controller: QuestControllerState) -> bool:
        """Is the operator asking for torso height on this stick?"""
        if not self.config.torso_requires_stick_click:
            return True
        return controller.thumbstick_press

    def _torso_joints_at(self, drop: float) -> dict[str, float]:
        """Interpolate the height table at ``drop`` metres below full extension."""
        drops = self.config.torso_fold_drops
        upper = next((i for i, d in enumerate(drops) if d >= drop), len(drops) - 1)
        lower = max(upper - 1, 0)
        span = drops[upper] - drops[lower]
        blend = 0.0 if span <= 0.0 else (drop - drops[lower]) / span
        return {
            name: values[lower] + blend * (values[upper] - values[lower])
            for name, values in self.config.torso_fold_joints.items()
        }

    def _publish_torso_target(self, controller: QuestControllerState | None) -> None:
        """Jog head height by sending torso joint positions.

        Positions, not velocities: the R1 Pro whole-body bridge accepts only
        POSITION commands. They go to the coordinator's joint-trajectory task,
        which arbitrates per joint, so commanding the four torso joints leaves
        the arms with the teleoperation task untouched. That is what keeps a
        late or jumpy hand target from ever moving the torso.

        Only published while the jog is actually moving, plus once when it
        stops. Streaming it every tick would restart the trajectory endlessly
        and fight the hold-to-recover goal on the same stream.
        """
        drops = self.config.torso_fold_drops
        if not drops or not self.config.torso_fold_joints:
            return
        now = time.monotonic()
        elapsed = 0.0 if self._last_jog_t is None else min(now - self._last_jog_t, _MAX_JOG_STEP_S)
        self._last_jog_t = now

        axis = 0.0
        if controller is not None and not self._estopped and self._torso_armed(controller):
            # Stick forward is negative y and should raise the head, which
            # means reducing the drop.
            axis = self._deadzone(controller.thumbstick.y, self.config.torso_deadzone)

        previous = self._torso_drop
        self._torso_drop = min(
            drops[-1],
            max(drops[0], previous + axis * self.config.torso_speed * elapsed),
        )
        moving = self._torso_drop != previous
        if not moving and not self._torso_settling:
            return
        self._torso_settling = moving

        pose = self._torso_joints_at(self._torso_drop)
        names = list(pose)
        self.joint_command.publish(JointState(name=names, position=[pose[name] for name in names]))

    def _handle_estop(self, nonce: Any) -> None:
        super()._handle_estop(nonce)
        self._publish_safe_command()

    def _on_operator_lost(self) -> None:
        super()._on_operator_lost()
        self._publish_safe_command()
