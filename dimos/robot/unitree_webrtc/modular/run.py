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
Centralized runner for modular G1 deployment scripts.

Usage:
    python run.py g1agent --ip 192.168.1.100
    python run.py g1zed
    python run.py g1detector --ip $ROBOT_IP
"""

import argparse
import importlib
import os
import sys

from dotenv import load_dotenv

from dimos.core import start, wait_exit


def main():
    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Modular Deployment Runner")
    parser.add_argument(
        "module",
        help="Module name to run (e.g., g1agent, g1zed, g1detector)",
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

    # Import the module
    try:
        # Try importing from current package first
        module = importlib.import_module(
            f".{args.module}", package="dimos.robot.unitree_webrtc.modular"
        )
    except ImportError as e:
        print(f"ERROR: Could not import module '{args.module}'")
        print(f"Make sure the module exists in dimos/robot/unitree_webrtc/modular/")
        print(f"Import error: {e}")
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
        dimos.close_all()


if __name__ == "__main__":
    main()
