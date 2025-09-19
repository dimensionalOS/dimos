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
Force-Torque Module Test/Deployment Script

Deploys and connects the FT driver and visualizer modules using Dimos.
Uses LCM transport for Vector3 force and torque messages.
"""

import time
import argparse
from pathlib import Path

from dimos.core import start, LCMTransport, pLCMTransport
from dimos.utils.logging_config import setup_logger
from dimos.msgs.geometry_msgs import Vector3
from dimos.hardware.ft_driver_module import FTDriverModule, RawSensorData
from dimos.hardware.ft_visualizer_module import FTVisualizerModule

logger = setup_logger(__name__)


def main():
    """Main deployment function for FT sensor modules."""
    parser = argparse.ArgumentParser(
        description="Deploy Force-Torque sensor driver and visualizer modules",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default settings
  python ft_module_test.py

  # Run with calibration file
  python ft_module_test.py --calibration ft_calibration.json

  # Run with custom serial port and verbose output
  python ft_module_test.py --port /dev/ttyUSB0 --calibration ft_cal.json --verbose

  # Run with custom dashboard port
  python ft_module_test.py --dash-port 8080 --calibration ft_calibration.npz
        """,
    )

    # Driver arguments
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port for sensor (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--window", type=int, default=3, help="Moving average window size (default: 3)"
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default="dimos/hardware/ft_calibration.json",
        help="Path to calibration file (default: dimos/hardware/ft_calibration.json)"
    )

    # Visualizer arguments
    parser.add_argument(
        "--dash-port", type=int, default=8052, help="Port for Dash web server (default: 8052)"
    )
    parser.add_argument(
        "--dash-host", default="0.0.0.0", help="Host for Dash web server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--history", type=int, default=500, help="Max history points to keep (default: 500)"
    )
    parser.add_argument(
        "--update-interval",
        type=int,
        default=100,
        help="Dashboard update interval in ms (default: 100)",
    )

    # LCM transport arguments
    parser.add_argument(
        "--lcm-force-channel",
        default="/ft/force",
        help="LCM channel for force Vector3 data (default: /ft/force)",
    )
    parser.add_argument(
        "--lcm-torque-channel",
        default="/ft/torque",
        help="LCM channel for torque Vector3 data (default: /ft/torque)",
    )
    parser.add_argument(
        "--lcm-raw-channel",
        default="/ft/raw_sensors",
        help="LCM channel for raw sensor data (default: /ft/raw_sensors)",
    )

    # General arguments
    parser.add_argument(
        "--processes", type=int, default=3, help="Number of Dimos processes (default: 3)"
    )
    parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument(
        "--no-visualizer", action="store_true", help="Run driver only, without visualizer"
    )
    parser.add_argument(
        "--no-raw", action="store_true", help="Don't publish raw sensor data"
    )

    args = parser.parse_args()

    # Check if calibration file exists if specified
    if args.calibration:
        cal_path = Path(args.calibration)
        if not cal_path.exists():
            logger.warning(f"Calibration file {cal_path} not found")
            logger.warning("Will run without calibration (raw sensor values only)")
            args.calibration = None

    # Start Dimos
    logger.info("=" * 60)
    logger.info("Force-Torque Sensor Module Deployment")
    logger.info("=" * 60)
    logger.info(f"Starting Dimos with {args.processes} processes...")
    dimos = start(args.processes)

    # Deploy FT driver module
    logger.info("Deploying FT driver module...")
    logger.info(f"  Serial port: {args.port}")
    logger.info(f"  Baud rate: {args.baud}")
    logger.info(f"  Moving average window: {args.window}")
    logger.info(f"  Calibration file: {args.calibration or 'None (raw data only)'}")

    driver = dimos.deploy(
        FTDriverModule,
        serial_port=args.port,
        baud_rate=args.baud,
        window_size=args.window,
        calibration_file=args.calibration,
        verbose=args.verbose,
    )
    logger.info("Driver deployment complete")

    # Set up LCM transport for driver outputs
    logger.info("Setting up LCM transports...")

    # Force and torque use proper LCMTransport with Vector3 type
    driver.force.transport = LCMTransport(args.lcm_force_channel, Vector3)
    logger.info(f"  Force Vector3 channel: {args.lcm_force_channel}")

    driver.torque.transport = LCMTransport(args.lcm_torque_channel, Vector3)
    logger.info(f"  Torque Vector3 channel: {args.lcm_torque_channel}")

    # Raw sensor data (optional) uses pLCMTransport since it's a custom dataclass
    if not args.no_raw:
        driver.raw_sensor_data.transport = pLCMTransport(args.lcm_raw_channel)
        logger.info(f"  Raw sensor data channel: {args.lcm_raw_channel}")

    # Deploy visualizer if requested
    visualizer = None
    if not args.no_visualizer and args.calibration:
        logger.info("Deploying FT visualizer module...")
        logger.info(f"  Dashboard port: {args.dash_port}")
        logger.info(f"  Dashboard host: {args.dash_host}")
        logger.info(f"  History points: {args.history}")
        logger.info(f"  Update interval: {args.update_interval}ms")
        logger.warning("Note: Visualizer may have issues in multiprocess environment")
        logger.warning("  Consider using --no-visualizer flag to disable if not needed")

        visualizer = dimos.deploy(
            FTVisualizerModule,
            max_history=args.history,
            update_interval_ms=args.update_interval,
            dash_port=args.dash_port,
            dash_host=args.dash_host,
            verbose=args.verbose,
        )

        # Connect visualizer inputs to driver outputs
        visualizer.force.connect(driver.force)
        visualizer.torque.connect(driver.torque)
        logger.info(f"  Connected to force and torque streams")

    elif not args.no_visualizer and not args.calibration:
        logger.warning("Visualizer requires calibration file to run")
        logger.warning("  Please provide a calibration file with --calibration flag")

    # Start modules
    logger.info("=" * 60)
    logger.info("Starting modules...")
    logger.info("=" * 60)

    # Start driver
    driver.start()

    # Start visualizer
    if visualizer:
        visualizer.start()
        logger.info(
            f"Dashboard running at http://{'127.0.0.1' if args.dash_host == '0.0.0.0' else args.dash_host}:{args.dash_port}"
        )

    logger.info("All modules started successfully!")
    logger.info("Press Ctrl+C to stop...")

    # Main loop - print statistics periodically
    try:
        last_print_time = time.time()
        while True:
            time.sleep(1)

            # Print stats every 10 seconds
            if time.time() - last_print_time > 10:
                driver_stats = driver.get_stats()
                logger.info(
                    f"Driver Stats: Messages={driver_stats['message_count']}, "
                    f"Errors={driver_stats['error_count']}, "
                    f"Calibrated={driver_stats['calibrated_count']}, "
                    f"Calibration={'Yes' if driver_stats['calibration_loaded'] else 'No'}"
                )

                if driver_stats['calibration_loaded']:
                    logger.info(
                        f"  Latest |F|={driver_stats['latest_force_magnitude']:.2f} N, "
                        f"|T|={driver_stats['latest_torque_magnitude']:.4f} N⋅m"
                    )

                if visualizer:
                    viz_stats = visualizer.get_stats()
                    logger.info(
                        f"Visualizer Stats: Force msgs={viz_stats['force_count']}, "
                        f"Torque msgs={viz_stats['torque_count']}, "
                        f"Data points={viz_stats['data_points']}"
                    )

                last_print_time = time.time()

    except KeyboardInterrupt:
        logger.info("=" * 60)
        logger.info("Shutting down...")
        logger.info("=" * 60)

        # Stop modules
        driver.stop()
        if visualizer:
            visualizer.stop()

        # Shutdown Dimos
        time.sleep(0.5)  # Give modules time to clean up
        dimos.shutdown()

        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()