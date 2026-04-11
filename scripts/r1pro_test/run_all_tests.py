#!/usr/bin/env python3
"""
R1 Pro Test Runner — Single DDS Session

Runs all tests with ONE rclpy.init()/shutdown() cycle to avoid
FastDDS 2.x/3.x participant corruption that kills robot processes.

Usage:
    export ROS_DOMAIN_ID=41
    python3 scripts/r1pro_test/run_all_tests.py
    python3 scripts/r1pro_test/run_all_tests.py --skip-chassis
    python3 scripts/r1pro_test/run_all_tests.py --skip-arm
"""
import sys
import argparse

# Ensure scripts package is importable
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from scripts.r1pro_test import test_01_topic_discovery as t01
from scripts.r1pro_test import test_02_read_arm_feedback as t02
from scripts.r1pro_test import test_03_chassis_command as t03
from scripts.r1pro_test import test_04_arm_joint_command as t04


def main():
    parser = argparse.ArgumentParser(description="R1 Pro integration tests")
    parser.add_argument("--skip-chassis", action="store_true", help="Skip chassis test")
    parser.add_argument("--skip-arm", action="store_true", help="Skip arm movement test")
    args = parser.parse_args()

    print("=" * 60)
    print("R1 Pro Integration Tests (single DDS session)")
    print("=" * 60)

    # Single init — prevents DDS participant cycling
    rclpy.init()
    results = {}

    def confirm(msg):
        """Ask user to confirm before proceeding. Returns True to continue."""
        resp = input(f"\n{msg} Press Enter to continue, 'q' to quit: ").strip().lower()
        return resp != "q"

    # Test 01: Topic Discovery
    print("\n" + "=" * 60)
    print("TEST 01: Topic Discovery")
    print("=" * 60)
    results["01_topic_discovery"] = t01.main()

    # Test 02: Arm Feedback
    if not confirm("Ready for Test 02 (Arm Feedback — read only)?"):
        print("Stopping early.")
        rclpy.shutdown()
        return True
    print("=" * 60)
    print("TEST 02: Arm Feedback")
    print("=" * 60)
    results["02_arm_feedback"] = t02.main()

    # Test 03: Chassis Command
    if args.skip_chassis:
        print("\n[SKIPPED] Test 03: Chassis Command")
        results["03_chassis_command"] = None
    else:
        if not confirm("Ready for Test 03 (Chassis — robot will move, gatekeeper must be running)?"):
            print("Stopping early.")
            rclpy.shutdown()
            return True
        print("=" * 60)
        print("TEST 03: Chassis Command (requires gatekeeper on robot)")
        print("=" * 60)
        results["03_chassis_command"] = t03.main()

    # Test 04: Arm Movement
    if args.skip_arm:
        print("\n[SKIPPED] Test 04: Arm Movement")
        results["04_arm_movement"] = None
    else:
        if not confirm("Ready for Test 04 (Arm Movement — arm will physically move)?"):
            print("Stopping early.")
            rclpy.shutdown()
            return True
        print("=" * 60)
        print("TEST 04: Arm Movement (arm will move!)")
        print("=" * 60)
        results["04_arm_movement"] = t04.main(skip_prompt=True)

    # Single shutdown
    rclpy.shutdown()

    # Summary
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    for name, result in results.items():
        if result is None:
            status = "SKIP"
        elif result:
            status = "PASS"
        else:
            status = "FAIL"
        print(f"  [{status}] {name}")

    failed = sum(1 for r in results.values() if r is False)
    if failed:
        print(f"\n{failed} test(s) FAILED")
    else:
        print("\nAll tests passed!")
    return failed == 0


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
