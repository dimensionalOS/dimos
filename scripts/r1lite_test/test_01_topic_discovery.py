"""
Test 1: Topic Discovery
Verify that the DiMOS laptop can see the R1 Lite's ROS2 topics over ethernet.

Ported from scripts/r1pro_test/test_01_topic_discovery.py — expected topic
names come from r1lite_config.py (TODO(recon) until verified on hardware).

Run standalone:
    export ROS_DOMAIN_ID=<see r1lite_config>
    python3 scripts/r1lite_test/test_01_topic_discovery.py

Or via run_all_tests.py (preferred — single DDS session).

Pass condition: All expected R1 Lite topics found.
"""
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1lite_config import EXPECTED_TOPICS

DISCOVERY_WAIT = 10.0


def main() -> bool:
    """Run topic discovery. Assumes rclpy.init() already called."""
    node = rclpy.create_node("dimos_probe")

    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    topics = node.get_topic_names_and_types()
    print(f"\nFound {len(topics)} topics:\n")
    for name, types in sorted(topics):
        print(f"  {name}  [{', '.join(types)}]")

    topic_names = {name for name, _ in topics}
    print("\n--- Expected topic check ---")
    all_found = True
    for t in EXPECTED_TOPICS:
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
