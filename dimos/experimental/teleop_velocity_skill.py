#!/usr/bin/env python3
# Drop-in Guide teleop velocity skill — Out[Twist] stream variant.
# Declares a tele_cmd_vel output stream that the dimOS blueprint composer
# auto-wires to MovementManager's tele_cmd_vel input. MovementManager then
# muxes it with nav_cmd_vel and forwards to /cmd_vel which GO2Connection
# subscribes to. No raw lcm.LCM() — uses the same publish path that
# MovementManager and other in-process modules use.

from __future__ import annotations

import time

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PUB_HZ = 10.0

# Safety caps.
MAX_VX = 0.6   # m/s forward/back
MAX_VY = 0.4   # m/s strafe
MAX_WZ = 1.5   # rad/s yaw


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class TeleopVelocitySkill(Module):
    """Direct teleop velocity publisher via dimOS Out stream.

    Declares `tele_cmd_vel: Out[Twist]` so the blueprint composer auto-wires
    it to MovementManager's matching input. This is the same publish path
    dimOS's own teleop modules use — bypasses the nav planner but still
    goes through MovementManager → GO2Connection → motors.
    """

    tele_cmd_vel: Out[Twist]

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("TeleopVelocitySkill: ready (Out[Twist] stream)")

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def teleop_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
        duration_s: float = 1.5,
    ) -> str:
        """Drive the robot with raw cmd_vel for a fixed duration, then zero.

        Bypasses the nav planner — uses the tele_cmd_vel input stream that
        MovementManager subscribes to. Use this for WASD-style teleop where
        relative_move (goal-based) fails because the planner refuses
        pure-rotation, pure-backward, or costmap-blocked goals.

        Args:
            vx: forward velocity m/s (positive=forward, negative=backward).
                Capped at +/- 0.6 m/s.
            vy: strafe velocity m/s (positive=left). Capped at +/- 0.4 m/s.
            wz: yaw rate rad/s (positive=turn left/counter-clockwise).
                Capped at +/- 1.5 rad/s.
            duration_s: how long to hold the velocity in seconds. After
                this, zero velocity is published 3 times to ensure stop.
                Max 5.0s per call.

        Combine vx + wz for smooth curving motion (walk + turn at once).
        """
        vx = _clamp(float(vx), -MAX_VX, MAX_VX)
        vy = _clamp(float(vy), -MAX_VY, MAX_VY)
        wz = _clamp(float(wz), -MAX_WZ, MAX_WZ)
        dur = _clamp(float(duration_s), 0.0, 5.0)

        msg = Twist(Vector3(x=vx, y=vy, z=0.0), Vector3(x=0.0, y=0.0, z=wz))
        period = 1.0 / PUB_HZ

        deadline = time.time() + dur
        pubs = 0
        while time.time() < deadline:
            self.tele_cmd_vel.publish(msg)
            pubs += 1
            time.sleep(period)

        # Stop — publish zero velocity a few times to be sure.
        zero = Twist(Vector3(), Vector3())
        for _ in range(3):
            self.tele_cmd_vel.publish(zero)
            time.sleep(period)

        return (
            f"Teleop: vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} held {dur:.1f}s "
            f"({pubs} pubs at {PUB_HZ}Hz), then zero."
        )

    @skill
    def teleop_stop(self) -> str:
        """Immediately publish zero velocity. Stops the robot.

        Use this anytime you want to interrupt motion without waiting for
        the current teleop_velocity duration to elapse.
        """
        zero = Twist(Vector3(), Vector3())
        for _ in range(5):
            self.tele_cmd_vel.publish(zero)
            time.sleep(0.05)
        return "Teleop stop: zero velocity published."
