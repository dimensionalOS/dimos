#!/usr/bin/env python3
"""
R1 Lite Test Runner — Single DDS Session

Runs all tests with ONE rclpy.init()/shutdown() cycle to avoid the FastDDS
2.x/3.x participant corruption that killed robot processes on the R1 Pro.

Ported from scripts/r1pro_test/run_all_tests.py.

Prereqs:
  1. Robot ROS 2 stack running (see README).
  2. Set ROS_DOMAIN_ID to match the robot (find it with domain_scan.py).
  3. Run test_00_recon.py first and reconcile r1lite_config.py.

Usage:
    export ROS_DOMAIN_ID=<from domain_scan.py>
    python3 scripts/r1lite_test/run_all_tests.py
    python3 scripts/r1lite_test/run_all_tests.py --skip-chassis
    python3 scripts/r1lite_test/run_all_tests.py --skip-arm
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy

from scripts.r1lite_test import test_00_recon as t00
from scripts.r1lite_test import test_01_topic_discovery as t01
from scripts.r1lite_test import test_02_read_arm_feedback as t02
from scripts.r1lite_test import test_03_chassis_command as t03
from scripts.r1lite_test import test_04_arm_joint_command as t04


def main():
    parser = argparse.ArgumentParser(description="R1 Lite integration tests")
    parser.add_argument("--skip-chassis", action="store_true", help="Skip chassis test")
    parser.add_argument("--skip-arm", action="store_true", help="Skip arm movement test")
    parser.add_argument("--recon", action="store_true", help="Run recon dump then exit")
    args = parser.parse_args()

    print("=" * 60)
    print("R1 Lite Integration Tests (single DDS session)")
    print("=" * 60)

    rclpy.init()

    if args.recon:
        t00.main()
        rclpy.shutdown()
        return True

    results = {}

    def confirm(msg):
        resp = input(f"\n{msg} Press Enter to continue, 'q' to quit: ").strip().lower()
        return resp != "q"

    print("\n" + "=" * 60)
    print("TEST 01: Topic Discovery")
    print("=" * 60)
    results["01_topic_discovery"] = t01.main()

    if not confirm("Ready for Test 02 (Arm Feedback — read only)?"):
        print("Stopping early.")
        rclpy.shutdown()
        return True
    print("=" * 60)
    print("TEST 02: Arm Feedback")
    print("=" * 60)
    results["02_arm_feedback"] = t02.main()

    if args.skip_chassis:
        print("\n[SKIPPED] Test 03: Chassis Command")
        results["03_chassis_command"] = None
    else:
        if not confirm("Ready for Test 03 (Chassis — robot will move)?"):
            print("Stopping early.")
            rclpy.shutdown()
            return True
        print("=" * 60)
        print("TEST 03: Chassis Command")
        print("=" * 60)
        results["03_chassis_command"] = t03.main()

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

    rclpy.shutdown()

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    for name, result in results.items():
        status = "SKIP" if result is None else ("PASS" if result else "FAIL")
        print(f"  [{status}] {name}")

    failed = sum(1 for r in results.values() if r is False)
    print(f"\n{failed} test(s) FAILED" if failed else "\nAll tests passed!")
    return failed == 0


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
