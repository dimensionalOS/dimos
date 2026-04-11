"""
Test 3: Chassis Command via Gatekeeper

Publishes Twist to /cmd_vel (RELIABLE QoS) which the chassis_gatekeeper
on the robot forwards to chassis_control_node after unlocking all 3 gates.

Prerequisites:
  - chassis_gatekeeper.py running on the robot
  - Launch file remap applied: ('/controller', '/controller_unused')
  - Remote control ON, DDUU position

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_03_chassis_command.py

Pass condition: Robot moves forward briefly (~5-10cm) and stops cleanly.
"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
from geometry_msgs.msg import Twist

VELOCITY = -0.2   # m/s forward — small and safe
DURATION = 2.0   # seconds of movement
PUBLISH_HZ = 20  # match gatekeeper tick rate
DISCOVERY_WAIT = 3.0

# RELIABLE QoS — matches gatekeeper's /cmd_vel subscriber
QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


def main() -> bool:
    """Run chassis command test. Returns True if command was sent successfully."""

    node = rclpy.create_node("dimos_chassis_test")
    pub = node.create_publisher(Twist, "/cmd_vel", QOS_RELIABLE)

    # Wait for DDS discovery
    print("Waiting for DDS discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Check gatekeeper is listening
    sub_count = node.count_subscribers("/cmd_vel")
    if sub_count == 0:
        print("WARNING: No subscriber on /cmd_vel — is chassis_gatekeeper running on robot?")
    else:
        print(f"  /cmd_vel has {sub_count} subscriber(s) — gatekeeper connected.")

    # --- Move ---
    print(f"Sending vx={VELOCITY} m/s for {DURATION}s...")
    move_msg = Twist()
    move_msg.linear.x = VELOCITY
    deadline = time.time() + DURATION
    while time.time() < deadline:
        pub.publish(move_msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_HZ)

    # --- Stop ---
    print("Stopping...")
    stop_msg = Twist()
    for _ in range(20):
        pub.publish(stop_msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_HZ)

    print("\nPASS: Commands sent. Did the robot move forward and stop?")
    node.destroy_node()
    return True


if __name__ == "__main__":
    print("!!! SAFETY CHECK !!!")
    print("- Robot on flat ground with clear space ahead?")
    print("- Hand on e-stop?")
    print("- Remote control ON, DDUU position?")
    print("- chassis_gatekeeper.py running on robot?")
    response = input("\nType 'yes' to proceed: ").strip().lower()
    if response != "yes":
        print("Aborted.")
        exit(0)

    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
