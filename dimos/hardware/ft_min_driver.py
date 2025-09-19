#!/usr/bin/env python3
"""
Minimal Force-Torque Sensor Driver

Reads 16 comma-separated sensor values from serial port,
applies moving average, and publishes via ZMQ.
"""

import serial
import json
import time
import zmq
import numpy as np
import argparse
from collections import deque


def main():
    parser = argparse.ArgumentParser(description="Minimal FT sensor driver")
    parser.add_argument('--port', default='/dev/tty.usbserial-0001', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--zmq-port', type=int, default=5555, help='ZMQ publish port')
    parser.add_argument('--window', type=int, default=3, help='Moving average window size')
    parser.add_argument('--verbose', action='store_true', help='Print sensor values')
    args = parser.parse_args()

    # Initialize moving average buffers for each sensor
    buffers = [deque(maxlen=args.window) for _ in range(16)]

    # Setup ZMQ publisher
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://*:{args.zmq_port}")

    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Connected to {args.port} at {args.baud} baud")
        print(f"Publishing on ZMQ port {args.zmq_port}")
        print(f"Moving average window: {args.window}")
        if args.verbose:
            print("\nSensor readings:")
            print("-" * 60)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        return 1

    try:
        while True:
            try:
                # Read line from serial
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                # Parse comma-separated values (remove trailing comma)
                if line.endswith(','):
                    line = line[:-1]
                values = [float(x) for x in line.split(',')]

                if len(values) != 16:
                    if args.verbose:
                        print(f"Warning: Expected 16 values, got {len(values)}")
                    continue

                # Update moving average buffers
                moving_averages = []
                for i, value in enumerate(values):
                    buffers[i].append(value)
                    moving_averages.append(np.mean(buffers[i]))

                # Publish to ZMQ
                data = {
                    'sensor_moving_averages': moving_averages,
                    'timestamp': time.time()
                }
                publisher.send_string(json.dumps(data))

                # Optional verbose output
                if args.verbose:
                    print(f"\r{time.strftime('%H:%M:%S')} ", end="")
                    for i, avg in enumerate(moving_averages):
                        print(f"{avg:7.2f}", end=" ")
                    print("", end="", flush=True)

            except ValueError as e:
                if args.verbose:
                    print(f"Parse error: {e}")
            except Exception as e:
                if args.verbose:
                    print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        ser.close()
        publisher.close()
        context.term()
        print("Cleanup complete")

    return 0


if __name__ == '__main__':
    exit(main())