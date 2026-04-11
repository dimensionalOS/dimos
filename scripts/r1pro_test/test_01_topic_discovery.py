"""
Test 1: Topic Discovery
Verify that the DiMOS laptop can see the R1 Pro's ROS2 topics over ethernet.

Run standalone:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_01_topic_discovery.py

Or via run_all_tests.py (preferred — single DDS session).

Pass condition: All expected R1 Pro topics found.
"""
import rclpy
import time

DISCOVERY_WAIT = 10.0


def main() -> bool:
    """Run topic discovery. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_probe")

    # Give DDS time to discover peers across the network
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    topics = node.get_topic_names_and_types()
    print(f"\nFound {len(topics)} topics:\n")
    for name, types in sorted(topics):
        print(f"  {name}  [{', '.join(types)}]")

    expected = [
        "/hdas/feedback_arm_left",
        "/hdas/feedback_arm_right",
        "/hdas/feedback_chassis",
        "/hdas/feedback_torso",
        "/motion_target/target_speed_chassis",
        "/motion_target/target_joint_state_arm_left",
        "/motion_target/target_joint_state_arm_right",
    ]
    topic_names = {name for name, _ in topics}
    print("\n--- Expected topic check ---")
    all_found = True
    for t in expected:
        found = t in topic_names
        status = "OK" if found else "MISSING"
        print(f"  [{status}] {t}")
        if not found:
            all_found = False

    result = "PASS" if all_found else "FAIL"
    detail = "All expected topics found" if all_found else "Some topics missing"
    print(f"\n{result}: {detail}")

    node.destroy_node()
    return all_found


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
