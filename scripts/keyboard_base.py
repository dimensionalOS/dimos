"""Keyboard teleop for FlowBase — publishes Twist to /cmd_vel over LCM.

Run alongside coordinator-flowbase-stereo-nav in a separate terminal.

Controls:
    W/S: forward/backward
    Q/E: strafe left/right
    A/D: turn left/right
    Space: stop
    Ctrl+C: quit
"""

import sys
import termios
import tty

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic

SPEED = 0.2
TURN = 0.35

# Must match the type-suffixed channel dimos's own transport publishes/subscribes on
# (visible in `dimos run` logs as "topic=/cmd_vel#geometry_msgs.Twist") — a bare
# "/cmd_vel" string is a different LCM channel and ControlCoordinator never sees it.
CMD_VEL_TOPIC = Topic(topic="/cmd_vel", lcm_type=Twist)

KEY_MAP = {
    "w": (SPEED,  0.0,   0.0),
    "s": (-SPEED, 0.0,   0.0),
    "q": (0.0,    SPEED, 0.0),
    "e": (0.0,   -SPEED, 0.0),
    "a": (0.0,    0.0,   TURN),
    "d": (0.0,    0.0,  -TURN),
    " ": (0.0,    0.0,   0.0),
}


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    lcm = LCM()
    print("WASD to move (A/D turn), Q/E to strafe, Space to stop, Ctrl+C to quit")

    while True:
        key = _getch()
        if key == "\x03":
            break

        vx, vy, wz = KEY_MAP.get(key.lower(), (None, None, None))
        if vx is None:
            continue

        twist = Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz),
        )
        lcm.publish(CMD_VEL_TOPIC, twist)
        print(f"vx={vx:.1f}  vy={vy:.1f}  wz={wz:.1f}")

    lcm.publish(CMD_VEL_TOPIC, Twist(linear=Vector3(), angular=Vector3()))


if __name__ == "__main__":
    main()
