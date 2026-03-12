"""
Test 1: Topic Discovery
Verify that the DiMOS laptop can see the R1 Pro's ROS2 topics over ethernet.

Run with:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/test_01_topic_discovery.py

Pass condition: Prints /hdas/feedback_arm_left, /hdas/feedback_chassis, etc.
"""
import rclpy

rclpy.init()
node = rclpy.create_node("dimos_probe")

# Give DDS time to discover peers across the network (cross-version needs more time)
import time
time.sleep(10.0)

topics = node.get_topic_names_and_types()
print(f"\nFound {len(topics)} topics:\n")
for name, types in sorted(topics):
    print(f"  {name}  [{', '.join(types)}]")

# Check for expected R1 Pro topics
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

print(f"\n{'PASS' if all_found else 'FAIL'}: {'All expected topics found' if all_found else 'Some topics missing — check robot is on and mobiman is running'}")

node.destroy_node()
rclpy.shutdown()
