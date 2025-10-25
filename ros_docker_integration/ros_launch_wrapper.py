#!/usr/bin/env python3
"""
Wrapper script to properly handle ROS2 launch file shutdown.
This script ensures clean shutdown of all ROS nodes when receiving SIGINT.
"""

import os
import sys
import signal
import subprocess
import time
import threading

class ROSLaunchWrapper:
    def __init__(self):
        self.ros_process = None
        self.dimos_process = None
        self.shutdown_in_progress = False

    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        if self.shutdown_in_progress:
            return

        self.shutdown_in_progress = True
        print("\n\nShutdown signal received. Stopping services gracefully...")

        # Stop DimOS first (it typically shuts down cleanly)
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
        """Main execution function"""
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        print("Starting ROS route planner and DimOS...")

        # Change to the ROS workspace directory
        os.chdir("/ros2_ws/src/autonomy_stack_mecanum_wheel_platform")

        # Start ROS route planner
        print("Starting ROS route planner...")
        self.ros_process = subprocess.Popen(
            ["bash", "./system_simulation_with_route_planner.sh"],
            preexec_fn=os.setsid  # Create new process group
        )

        # Wait for ROS to initialize
        print("Waiting for ROS to initialize...")
        time.sleep(5)

        # Start DimOS
        print("Starting DimOS Unitree G1 controller...")
        self.dimos_process = subprocess.Popen(
            [sys.executable, "/home/p/pro/dimensional/dimos/dimos/robot/unitree_webrtc/unitree_g1.py"]
        )

        print("Both systems are running. Press Ctrl+C to stop.")
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
                if self.dimos_process.poll() is not None:
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