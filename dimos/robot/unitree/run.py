#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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
Centralized runner for modular Unitree robot deployment scripts.

Usage:
    python run.py g1agent --ip 192.168.1.100
    python run.py g1/g1zed --ip $ROBOT_IP
    python run.py go2/go2.py --ip $ROBOT_IP
    python run.py connection/g1.py --ip $ROBOT_IP
"""

import argparse
import importlib
import os
import sys

from dotenv import load_dotenv

from dimos.core import start, wait_exit


def main() -> None:
    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree Robot Modular Deployment Runner")
    parser.add_argument(
        "module",
        help="Module name/path to run (e.g., g1agent, g1/g1zed, go2/go2.py)",
    )
    parser.add_argument(
        "--ip",
        default=os.getenv("ROBOT_IP"),
        help="Robot IP address (default: ROBOT_IP from .env)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Number of worker threads for DimosCluster (default: 8)",
    )

    args = parser.parse_args()

    # Validate IP address
    if not args.ip:
        print("ERROR: Robot IP address not provided")
        print("Please provide --ip or set ROBOT_IP in .env")
        sys.exit(1)

    # Parse the module path
    module_path = args.module

    # Remove .py extension if present
    if module_path.endswith(".py"):
        module_path = module_path[:-3]

    # Convert path separators to dots for import
    module_path = module_path.replace("/", ".")

    # Import the module
    try:
        # Build the full import path
        full_module_path = f"dimos.robot.unitree.{module_path}"
        print(f"Importing module: {full_module_path}")
        module = importlib.import_module(full_module_path)
    except ImportError:
        # Try as a relative import from the unitree package
        try:
            module = importlib.import_module(f".{module_path}", package="dimos.robot.unitree")
        except ImportError as e2:
            import traceback

            traceback.print_exc()

            print(f"\nERROR: Could not import module '{args.module}'")
            print("Tried importing as:")
            print(f"  1. {full_module_path}")
            print("  2. Relative import from dimos.robot.unitree")
            print("Make sure the module exists in dimos/robot/unitree/")
            print(f"Import error: {e2}")

            sys.exit(1)

    # Verify deploy function exists
    if not hasattr(module, "deploy"):
        print(f"ERROR: Module '{args.module}' does not have a 'deploy' function")
        sys.exit(1)

    print(f"Running {args.module}.deploy() with IP {args.ip}")

    # Run the standard deployment pattern
    dimos = start(args.workers)
    try:
        module.deploy(dimos, args.ip)
        wait_exit()
    finally:
        dimos.close_all()  # type: ignore[attr-defined]


if __name__ == "__main__":
    main()
