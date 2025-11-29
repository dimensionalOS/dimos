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
Wrapper script to properly handle ROS2 launch file shutdown.
This script ensures clean shutdown of all ROS nodes when receiving SIGINT.
"""

import os
import signal
import subprocess
import sys
import time


class ROSLaunchWrapper:
    def __init__(self):
        self.ros_process = None
        self.dimos_process = None
        self.shutdown_in_progress = False

    def signal_handler(self, _signum, _frame):
        """Handle shutdown signals gracefully"""
        if self.shutdown_in_progress:
            return

        self.shutdown_in_progress = True
        print("\n\nShutdown signal received. Stopping services gracefully...")

        # Stop DimOS first
        if self.dimos_process and self.dimos_process.poll() is None:
            print("Stopping DimOS...")
            self.dimos_process.terminate()
            try:
                self.dimos_process.wait(timeout=5)
                print("DimOS stopped cleanly.")
            except subprocess.TimeoutExpired:
                print("Force stopping DimOS...")
                self.dimos_process.kill()
                self.dimos_process.wait()

        # Stop ROS - send SIGINT first for graceful shutdown
        if self.ros_process and self.ros_process.poll() is None:
            print("Stopping ROS nodes (this may take a moment)...")

            # Send SIGINT to trigger graceful ROS shutdown
            self.ros_process.send_signal(signal.SIGINT)

            # Wait for graceful shutdown with timeout
            try:
                self.ros_process.wait(timeout=15)
                print("ROS stopped cleanly.")
            except subprocess.TimeoutExpired:
                print("ROS is taking too long to stop. Sending SIGTERM...")
                self.ros_process.terminate()
                try:
                    self.ros_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print("Force stopping ROS...")
                    self.ros_process.kill()
                    self.ros_process.wait()

        # Clean up any remaining processes
        print("Cleaning up any remaining processes...")
        cleanup_commands = [
            "pkill -f 'ros2' || true",
            "pkill -f 'localPlanner' || true",
            "pkill -f 'pathFollower' || true",
            "pkill -f 'terrainAnalysis' || true",
            "pkill -f 'sensorScanGeneration' || true",
            "pkill -f 'vehicleSimulator' || true",
            "pkill -f 'visualizationTools' || true",
            "pkill -f 'far_planner' || true",
            "pkill -f 'graph_decoder' || true",
        ]

        for cmd in cleanup_commands:
            subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print("All services stopped.")
        sys.exit(0)

    def run(self):
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        print("Starting ROS route planner and DimOS...")

        # Change to the ROS workspace directory
        os.chdir("/ros2_ws/src/ros-navigation-autonomy-stack")

        # Start ROS route planner
        print("Starting ROS route planner...")
        self.ros_process = subprocess.Popen(
            ["bash", "./system_simulation_with_route_planner.sh"],
            preexec_fn=os.setsid,  # Create new process group
        )

        print("Waiting for ROS to initialize...")
        time.sleep(5)

        print("Starting DimOS navigation bot...")

        nav_bot_path = "/workspace/dimos/dimos/navigation/demo_ros_navigation.py"
        venv_python = "/opt/dimos-venv/bin/python"

        if not os.path.exists(nav_bot_path):
            print(f"ERROR: demo_ros_navigation.py not found at {nav_bot_path}")
            nav_dir = "/workspace/dimos/dimos/navigation/"
            if os.path.exists(nav_dir):
                print(f"Contents of {nav_dir}:")
                for item in os.listdir(nav_dir):
                    print(f"  - {item}")
            else:
                print(f"Directory not found: {nav_dir}")
            return

        if not os.path.exists(venv_python):
            print(f"ERROR: venv Python not found at {venv_python}, using system Python")
            return

        print(f"Using Python: {venv_python}")
        print(f"Starting script: {nav_bot_path}")

        # Use the venv Python explicitly
        try:
            self.dimos_process = subprocess.Popen(
                [venv_python, nav_bot_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True,
            )

            # Give it a moment to start and check if it's still running
            time.sleep(2)
            poll_result = self.dimos_process.poll()
            if poll_result is not None:
                # Process exited immediately
                stdout, stderr = self.dimos_process.communicate(timeout=1)
                print(f"ERROR: DimOS failed to start (exit code: {poll_result})")
                if stdout:
                    print(f"STDOUT: {stdout}")
                if stderr:
                    print(f"STDERR: {stderr}")
                self.dimos_process = None
            else:
                print(f"DimOS started successfully (PID: {self.dimos_process.pid})")

        except Exception as e:
            print(f"ERROR: Failed to start DimOS: {e}")
            self.dimos_process = None

        if self.dimos_process:
            print("Both systems are running. Press Ctrl+C to stop.")
        else:
            print("ROS is running (DimOS failed to start). Press Ctrl+C to stop.")
        print("")

        # Wait for processes
        try:
            # Monitor both processes
            while True:
                # Check if either process has died
                if self.ros_process.poll() is not None:
                    print("ROS process has stopped unexpectedly.")
                    self.signal_handler(signal.SIGTERM, None)
                    break
                if self.dimos_process and self.dimos_process.poll() is not None:
                    print("DimOS process has stopped.")
                    # DimOS stopping is less critical, but we should still clean up ROS
                    self.signal_handler(signal.SIGTERM, None)
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            pass  # Signal handler will take care of cleanup


if __name__ == "__main__":
    wrapper = ROSLaunchWrapper()
    wrapper.run()
