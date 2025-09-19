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
Force-Torque Sensor Driver Module for Dimos

Reads from serial port, applies moving average and calibration,
and publishes calibrated force-torque data via LCM.
"""

import serial
import json
import time
import numpy as np
import argparse
import threading
from collections import deque
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, field

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.std_msgs import Header


@dataclass
class RawSensorData:
    """Data structure for raw sensor values with moving averages."""

    sensor_values: list = field(default_factory=list)
    timestamp: float = 0.0


class FTDriverModule(Module):
    """Force-Torque sensor driver module with calibration."""

    # Output ports - separate force and torque as Vector3
    force: Out[Vector3] = None  # Force vector in Newtons
    torque: Out[Vector3] = None  # Torque vector in Newton-meters
    raw_sensor_data: Out[RawSensorData] = None  # Raw sensor values (optional)

    def __init__(
        self,
        serial_port: str = "/dev/tty.usbserial-0001",
        baud_rate: int = 115200,
        window_size: int = 3,
        calibration_file: Optional[str] = None,
        verbose: bool = False,
        frame_id: str = "ft_sensor",
    ):
        """
        Initialize the FT driver module.

        Args:
            serial_port: Serial port device path
            baud_rate: Serial baud rate
            window_size: Moving average window size
            calibration_file: Path to calibration JSON/NPZ file
            verbose: Enable verbose output
            frame_id: Frame ID for published messages
        """
        super().__init__()

        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.window_size = window_size
        self.calibration_file = calibration_file
        self.verbose = verbose
        self.frame_id = frame_id

        # Serial connection
        self.ser = None

        # Moving average buffers for each sensor
        self.buffers = [deque(maxlen=window_size) for _ in range(16)]

        # Calibration matrix and bias
        self.calibration_matrix = None  # 6x16 matrix
        self.bias_vector = None  # 6x1 vector

        # Statistics
        self.message_count = 0
        self.error_count = 0
        self.calibrated_count = 0

        # Running flag and thread
        self.running = False
        self._thread = None

        # Store latest values for stats
        self.latest_force_mag = 0.0
        self.latest_torque_mag = 0.0

    def load_calibration(self):
        """Load calibration matrix and bias from file."""
        if not self.calibration_file:
            print("No calibration file specified, outputting raw sensor values only")
            return

        filepath = Path(self.calibration_file)
        if not filepath.exists():
            print(f"Warning: Calibration file {filepath} not found")
            return

        try:
            if filepath.suffix == ".npz":
                data = np.load(filepath)
                self.calibration_matrix = np.array(data["calibration_matrix"])
                self.bias_vector = (
                    np.array(data["bias_vector"]) if data["bias_vector"] is not None else None
                )
            else:
                with open(filepath, "r") as f:
                    data = json.load(f)
                self.calibration_matrix = np.array(data["calibration_matrix"])
                self.bias_vector = (
                    np.array(data["bias_vector"]) if data["bias_vector"] is not None else None
                )

            print(f"Calibration loaded from: {filepath}")
            print(f"  Calibration matrix shape: {self.calibration_matrix.shape}")
            print(f"  Has bias: {self.bias_vector is not None}")
        except Exception as e:
            print(f"Error loading calibration: {e}")

    def apply_calibration(self, sensor_data: np.ndarray) -> Optional[np.ndarray]:
        """
        Apply calibration to sensor data.

        Args:
            sensor_data: 16x1 array of sensor readings

        Returns:
            6x1 array of calibrated forces/torques or None if no calibration
        """
        if self.calibration_matrix is None:
            return None

        # Apply calibration: F = S @ C^T + b
        force_torque = sensor_data @ self.calibration_matrix.T

        if self.bias_vector is not None:
            force_torque += self.bias_vector

        return force_torque

    def connect_serial(self):
        """Connect to serial port."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            print(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False

    def read_and_process(self):
        """Read from serial, apply moving average, and optionally calibrate."""
        if not self.ser:
            return

        try:
            # Read line from serial
            line = self.ser.readline().decode("utf-8").strip()
            if not line:
                return

            # Parse comma-separated values (remove trailing comma)
            if line.endswith(","):
                line = line[:-1]
            values = [float(x) for x in line.split(",")]

            if len(values) != 16:
                if self.verbose:
                    print(f"Warning: Expected 16 values, got {len(values)}")
                self.error_count += 1
                return

            # Update moving average buffers
            moving_averages = []
            for i, value in enumerate(values):
                self.buffers[i].append(value)
                moving_averages.append(np.mean(self.buffers[i]))

            timestamp = time.time()

            # Optionally publish raw sensor data with moving averages
            if self.raw_sensor_data is not None:
                raw_data = RawSensorData(sensor_values=moving_averages, timestamp=timestamp)
                self.raw_sensor_data.publish(raw_data)

            # Apply calibration if available
            if self.calibration_matrix is not None:
                sensor_array = np.array(moving_averages)
                force_torque = self.apply_calibration(sensor_array)

                if force_torque is not None:
                    # Extract force and torque components
                    force_vec = Vector3(force_torque[0], force_torque[1], force_torque[2])
                    torque_vec = Vector3(force_torque[3], force_torque[4], force_torque[5])

                    # Calculate magnitudes for display
                    self.latest_force_mag = np.linalg.norm(force_torque[:3])
                    self.latest_torque_mag = np.linalg.norm(force_torque[3:])

                    # Publish force and torque as separate Vector3 messages
                    self.force.publish(force_vec)
                    self.torque.publish(torque_vec)
                    self.calibrated_count += 1

                    if self.verbose:
                        print(
                            f"\r{time.strftime('%H:%M:%S')} "
                            f"F:({force_torque[0]:7.2f},{force_torque[1]:7.2f},{force_torque[2]:7.2f}) "
                            f"T:({force_torque[3]:7.4f},{force_torque[4]:7.4f},{force_torque[5]:7.4f}) "
                            f"|F|:{self.latest_force_mag:7.2f} |T|:{self.latest_torque_mag:7.4f}",
                            end="",
                            flush=True,
                        )

            self.message_count += 1

        except ValueError as e:
            if self.verbose:
                print(f"Parse error: {e}")
            self.error_count += 1
        except Exception as e:
            if self.verbose:
                print(f"Error: {e}")
            self.error_count += 1

    def _run_loop(self):
        """Main loop that reads from serial port - runs in background thread."""
        if self.verbose:
            print("\nSensor readings:")
            print("-" * 80)

        try:
            while self.running:
                self.read_and_process()
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"Error in driver loop: {e}")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()

    @rpc
    def start(self):
        """Start the sensor driver."""
        if self.running:
            print("FT driver already running")
            return

        print(f"Starting FT driver module...")
        print(f"  Serial port: {self.serial_port}")
        print(f"  Baud rate: {self.baud_rate}")
        print(f"  Moving average window: {self.window_size}")
        print(f"  Calibration file: {self.calibration_file or 'None'}")

        # Load calibration if available
        self.load_calibration()

        # Connect to serial
        if not self.connect_serial():
            print("Failed to connect to serial port")
            return

        # Set running flag
        self.running = True

        # Start background thread for serial reading
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        print("FT driver started successfully")

    @rpc
    def stop(self):
        """Stop the sensor driver."""
        if not self.running:
            return

        print("Stopping FT driver...")
        self.running = False

        # Wait for thread to finish
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        # Close serial if still open
        if self.ser and self.ser.is_open:
            self.ser.close()

        print(f"FT driver stopped. Messages: {self.message_count}, Errors: {self.error_count}, Calibrated: {self.calibrated_count}")

    @rpc
    def get_stats(self) -> Dict[str, Any]:
        """Get driver statistics."""
        return {
            "message_count": self.message_count,
            "error_count": self.error_count,
            "calibrated_count": self.calibrated_count,
            "calibration_loaded": self.calibration_matrix is not None,
            "serial_connected": self.ser is not None and self.ser.is_open,
            "latest_force_magnitude": self.latest_force_mag,
            "latest_torque_magnitude": self.latest_torque_mag,
        }


if __name__ == "__main__":
    # For testing standalone
    parser = argparse.ArgumentParser(description="FT Driver Module")
    parser.add_argument("--port", default="/dev/tty.usbserial-0001", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--window", type=int, default=3, help="Moving average window size")
    parser.add_argument("--calibration", type=str, help="Calibration file path")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()

    from dimos.core import start

    dimos = start(1)
    driver = dimos.deploy(
        FTDriverModule,
        serial_port=args.port,
        baud_rate=args.baud,
        window_size=args.window,
        calibration_file=args.calibration,
        verbose=args.verbose,
    )

    driver.start()