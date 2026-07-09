"""Keyboard teleop for FlowBase — publishes Twist to /cmd_vel over LCM.

Run alongside coordinator-flowbase-stereo-nav in a separate terminal.

Controls:
    W/S: forward/backward
    A/D: strafe left/right
    Q/E: rotate left/right
    Space: stop
    Ctrl+C: quit
"""

import sys
import termios
import tty

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCM

SPEED = 0.3
TURN = 0.5

KEY_MAP = {
    "w": (SPEED,  0.0,   0.0),
    "s": (-SPEED, 0.0,   0.0),
    "a": (0.0,    SPEED, 0.0),
    "d": (0.0,   -SPEED, 0.0),
    "q": (0.0,    0.0,   TURN),
    "e": (0.0,    0.0,  -TURN),
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
    print("WASD to move, Q/E to rotate, Space to stop, Ctrl+C to quit")

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
        lcm.publish("/cmd_vel", twist)
        print(f"vx={vx:.1f}  vy={vy:.1f}  wz={wz:.1f}")

    lcm.publish("/cmd_vel", Twist(linear=Vector3(), angular=Vector3()))


if __name__ == "__main__":
    main()
