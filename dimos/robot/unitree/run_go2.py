#!/usr/bin/env python3

from dimos.robot.unitree.unitree_go2 import UnitreeGo2, WebRTCConnectionMethod
import os
import time


def get_env_var(var_name, default=None, required=False):
    """Get environment variable with validation."""
    value = os.getenv(var_name, default)
    if required and not value:
        raise ValueError(f"{var_name} environment variable is required")
    return value


if __name__ == "__main__":
    # Get configuration from environment variables
    robot_ip = get_env_var("ROBOT_IP", "192.168.9.140")
    connection_method = get_env_var("CONNECTION_METHOD", "LocalSTA")
    serial_number = get_env_var("SERIAL_NUMBER", None)
    output_dir = get_env_var("OUTPUT_DIR", os.path.join(os.getcwd(), "output"))

    # Convert connection method string to enum
    connection_method = getattr(WebRTCConnectionMethod, connection_method)

    print("Initializing UnitreeGo2...")
    print(f"Configuration:")
    print(f"  IP: {robot_ip}")
    print(f"  Connection Method: {connection_method}")
    print(
        f"  Serial Number: {serial_number if serial_number else 'Not provided'}"
    )
    print(f"  Output Directory: {output_dir}")

    robot = UnitreeGo2(ip=robot_ip,
                       connection_method=connection_method,
                       serial_number=serial_number,
                       output_dir=output_dir)

    try:
        # Start perception
        print("\nStarting perception system...")
        robot.start_perception()

        print("\nMonitoring agent outputs (Press Ctrl+C to stop)...")
        # Monitor agent outputs every 5 seconds
        while True:
            time.sleep(5)
            robot.read_agent_outputs()

    except KeyboardInterrupt:
        print("\nStopping perception...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        del robot
        print("Cleanup complete.")
