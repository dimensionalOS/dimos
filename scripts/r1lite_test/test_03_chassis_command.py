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

"""
Test 3: chassis command. Drives ~5 cm forward with all gates held open, then
streams an active zero stop. PASS requires measured speed to rise during the
move and settle near zero after.

The chassis node requires, simultaneously and streamed every tick: a
subscriber on /motion_control/chassis_speed (this script's own monitor),
brake_mode false, and a nonzero chassis_acc_limit. Commands are dead-man
streams.

The RC must be ON with all switches in position 1 (mode 5). RC OFF fails safe
to a mode that vetoes software commands and mimics a latched VCU; verify on
the host with the vendor workspace sourced: ros2 topic echo /controller --once
The robot must have been powered on with the e-stop released. Keep ~0.5 m of
clear floor ahead and a hand on the e-stop.

Run:
    export ROS_DOMAIN_ID=2
    python3 scripts/r1lite_test/test_03_chassis_command.py
"""

from pathlib import Path
import sys
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import BRAKE_MODE, CHASSIS_ACC_LIMIT, CHASSIS_SPEED_FB, CMD_CHASSIS_SPEED

VELOCITY = 0.05  # m/s forward, 5 cm over MOVE_DURATION
MOVE_DURATION = 1.0  # s
STOP_DURATION = 1.5  # s of active zero-velocity streaming after the move
ACC_LIMIT_XY = 0.5  # m/s^2, gentle ramp (Gate 3 opener)
ACC_LIMIT_YAW = 0.5  # rad/s^2
PUBLISH_HZ = 20
DISCOVERY_WAIT = 3.0
MOVE_THRESHOLD = 0.02  # m/s measured speed that counts as "it moved"
STOP_THRESHOLD = 0.01  # m/s measured speed that counts as "stopped"


def main(skip_prompt=False) -> bool:
    node = rclpy.create_node("dimos_chassis_test")

    speed_pub = node.create_publisher(TwistStamped, CMD_CHASSIS_SPEED, 10)
    acc_pub = node.create_publisher(TwistStamped, CHASSIS_ACC_LIMIT, 10)
    brake_pub = node.create_publisher(Bool, BRAKE_MODE, 10)

    # Gate 1: our own subscription on the measured-speed topic. Also gives
    # us the PASS/FAIL evidence.
    peak_speed = [0.0]
    last_speed = [0.0]

    def speed_cb(msg):
        vx = abs(msg.twist.linear.x)
        vy = abs(msg.twist.linear.y)
        v = max(vx, vy)
        last_speed[0] = v
        if v > peak_speed[0]:
            peak_speed[0] = v

    node.create_subscription(TwistStamped, CHASSIS_SPEED_FB, speed_cb, 10)

    if not skip_prompt:
        print("!!! SAFETY CHECK !!!")
        print("- ~0.5 m clear floor AHEAD of the robot?")
        print("- Ethernet cable slack tended? Charger unplugged?")
        print("- Hand on e-stop?")
        print("- RC ON with ALL SWITCHES IN POSITION 1 (= mode 5)?")
        print("    RC OFF fails safe to mode 3 and VETOES software -> 0.3mm/s creep.")
        print("    Verify on the HOST (hdas_msg lives in the vendor workspace, not")
        print("    this container):  source ~/galaxea/install/setup.bash &&")
        print("                      ros2 topic echo /controller --once")
        print("- Robot was powered on with e-stop RELEASED?")
        if input("\nType 'yes' to proceed: ").strip().lower() != "yes":
            print("Aborted.")
            node.destroy_node()
            return False

    print("Waiting for DDS discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    subs = node.count_subscribers(CMD_CHASSIS_SPEED)
    print(f"{CMD_CHASSIS_SPEED}: {subs} subscriber(s)")
    if subs == 0:
        print("FAIL: chassis controller not subscribed, is the robot stack running?")
        node.destroy_node()
        return False

    def stream(vx: float, label: str, duration: float) -> None:
        """One tick = acc_limit + brake-off + speed target (gates 2+3 + command)."""
        print(f"{label}: vx={vx} m/s for {duration}s")
        period = 1.0 / PUBLISH_HZ
        end = time.time() + duration
        while time.time() < end:
            stamp = node.get_clock().now().to_msg()

            acc = TwistStamped()
            acc.header.stamp = stamp
            acc.twist.linear.x = ACC_LIMIT_XY
            acc.twist.linear.y = ACC_LIMIT_XY
            acc.twist.angular.z = ACC_LIMIT_YAW
            acc_pub.publish(acc)

            brake_pub.publish(Bool(data=False))

            cmd = TwistStamped()
            cmd.header.stamp = stamp
            cmd.twist.linear.x = vx
            speed_pub.publish(cmd)

            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

    moved_peak = 0.0
    try:
        stream(VELOCITY, "MOVE", MOVE_DURATION)
        moved_peak = peak_speed[0]
    finally:
        # Always end on a zero-velocity stream, whatever happened above.
        try:
            stream(0.0, "STOP", STOP_DURATION)
        except Exception:
            for _ in range(5):
                z = TwistStamped()
                z.header.stamp = node.get_clock().now().to_msg()
                speed_pub.publish(z)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)

    # Let feedback settle, then read final speed.
    settle_end = time.time() + 0.5
    while time.time() < settle_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"\nPeak measured speed during move: {moved_peak:.4f} m/s (threshold {MOVE_THRESHOLD})")
    print(f"Final measured speed after stop:  {last_speed[0]:.4f} m/s (threshold {STOP_THRESHOLD})")

    moved = moved_peak >= MOVE_THRESHOLD
    stopped = last_speed[0] <= STOP_THRESHOLD

    if moved and stopped:
        print(
            "\nPASS: chassis moved (~5 cm) and stopped cleanly. "
            "Confirm visually that it rolled forward."
        )
    elif not moved:
        print(
            "\nFAIL: chassis did not reach measurable speed. Suspects, in order:\n"
            "  1. e-stop was latched at power-on (whole-session inhibit)\n"
            "  2. a gate not open (brake/acc/subscriber, this script streams all three)\n"
            "  3. VCU fault, check /hdas/feedback_status_chassis on the robot"
        )
    else:
        print(
            "\nFAIL: chassis moved but measured speed did not settle to zero, "
            "verify visually it is stationary; e-stop if not."
        )

    node.destroy_node()
    return moved and stopped


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
