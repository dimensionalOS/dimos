#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Interactive calibration script for XLeRobot.

Run this BEFORE starting DimOS to create a calibration file that the
XLeRobotDriver can load headlessly.

Usage:
    python -m dimos.robot.xlerobot.calibrate
    python -m dimos.robot.xlerobot.calibrate --port1 /dev/ttyACM0 --port2 /dev/ttyACM1
    python -m dimos.robot.xlerobot.calibrate --output /path/to/calibration.json

The calibration procedure:
  1. Connects to both servo buses
  2. Disables torque on arm + head motors
  3. Prompts you to move joints to center, then through full range
  4. Records homing offsets and range of motion
  5. Saves calibration JSON to ~/.cache/xlerobot/calibration.json (or --output)
"""

import argparse
import json
import logging
import platform
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

try:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
except ImportError:
    logger.error("lerobot[feetech] is required. Install with: pip install lerobot[feetech]")
    sys.exit(1)


def _default_port1() -> str:
    return "/dev/cu.usbmodem1101" if platform.system() == "Darwin" else "/dev/ttyACM0"


def _default_port2() -> str:
    return "/dev/cu.usbmodem1201" if platform.system() == "Darwin" else "/dev/ttyACM1"


def _default_output() -> Path:
    return Path.home() / ".cache" / "xlerobot" / "calibration.json"


def build_bus1(port: str) -> tuple[FeetechMotorsBus, list[str], list[str]]:
    norm = MotorNormMode.RANGE_M100_100
    bus = FeetechMotorsBus(
        port=port,
        motors={
            "left_arm_shoulder_pan": Motor(1, "sts3215", norm),
            "left_arm_shoulder_lift": Motor(2, "sts3215", norm),
            "left_arm_elbow_flex": Motor(3, "sts3215", norm),
            "left_arm_wrist_flex": Motor(4, "sts3215", norm),
            "left_arm_wrist_roll": Motor(5, "sts3215", norm),
            "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            "head_motor_1": Motor(7, "sts3215", norm),
            "head_motor_2": Motor(8, "sts3215", norm),
        },
    )
    left_arm = [m for m in bus.motors if m.startswith("left_arm")]
    head = [m for m in bus.motors if m.startswith("head")]
    return bus, left_arm, head


def build_bus2(port: str) -> tuple[FeetechMotorsBus, list[str], list[str]]:
    norm = MotorNormMode.RANGE_M100_100
    bus = FeetechMotorsBus(
        port=port,
        motors={
            "right_arm_shoulder_pan": Motor(1, "sts3215", norm),
            "right_arm_shoulder_lift": Motor(2, "sts3215", norm),
            "right_arm_elbow_flex": Motor(3, "sts3215", norm),
            "right_arm_wrist_flex": Motor(4, "sts3215", norm),
            "right_arm_wrist_roll": Motor(5, "sts3215", norm),
            "right_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
        },
    )
    right_arm = [m for m in bus.motors if m.startswith("right_arm")]
    base = [m for m in bus.motors if m.startswith("base")]
    return bus, right_arm, base


def calibrate_bus1(bus: FeetechMotorsBus, left_arm: list[str], head: list[str]) -> dict[str, MotorCalibration]:
    motors_to_calibrate = left_arm + head
    bus.disable_torque()

    for name in motors_to_calibrate:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

    input(
        "\n>>> Move LEFT ARM and HEAD motors to the MIDDLE of their range of motion.\n"
        "    Press ENTER when ready..."
    )

    homing_offsets = bus.set_half_turn_homings(motors_to_calibrate)

    print(
        "\n>>> Now move all LEFT ARM and HEAD joints sequentially through their\n"
        "    ENTIRE range of motion. Recording positions...\n"
        "    Press ENTER to stop recording."
    )
    range_mins, range_maxes = bus.record_ranges_of_motion(motors_to_calibrate)

    calibration = {}
    for name, motor in bus.motors.items():
        calibration[name] = MotorCalibration(
            id=motor.id,
            drive_mode=0,
            homing_offset=homing_offsets.get(name, 0),
            range_min=range_mins.get(name, 0),
            range_max=range_maxes.get(name, 4095),
        )

    bus.write_calibration(calibration)
    return calibration


def calibrate_bus2(bus: FeetechMotorsBus, right_arm: list[str], base: list[str]) -> dict[str, MotorCalibration]:
    bus.disable_torque(right_arm)

    for name in right_arm:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

    input(
        "\n>>> Move RIGHT ARM motors to the MIDDLE of their range of motion.\n"
        "    Press ENTER when ready..."
    )

    homing_offsets = bus.set_half_turn_homings(right_arm)
    homing_offsets.update(dict.fromkeys(base, 0))

    print(
        "\n>>> Now move all RIGHT ARM joints sequentially through their\n"
        "    ENTIRE range of motion (wheels are skipped).\n"
        "    Press ENTER to stop recording."
    )
    range_mins, range_maxes = bus.record_ranges_of_motion(right_arm)

    for name in base:
        range_mins[name] = 0
        range_maxes[name] = 4095

    calibration = {}
    for name, motor in bus.motors.items():
        calibration[name] = MotorCalibration(
            id=motor.id,
            drive_mode=0,
            homing_offset=homing_offsets.get(name, 0),
            range_min=range_mins.get(name, 0),
            range_max=range_maxes.get(name, 4095),
        )

    bus.write_calibration(calibration)
    return calibration


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive XLeRobot calibration")
    parser.add_argument("--port1", default=_default_port1(), help="Bus 1 serial port (left arm + head)")
    parser.add_argument("--port2", default=_default_port2(), help="Bus 2 serial port (right arm + wheels)")
    parser.add_argument("--output", default=str(_default_output()), help="Output calibration JSON path")
    args = parser.parse_args()

    output_path = Path(args.output)

    print("=" * 60)
    print("  XLeRobot Calibration")
    print("=" * 60)
    print(f"  Bus 1: {args.port1}")
    print(f"  Bus 2: {args.port2}")
    print(f"  Output: {output_path}")
    print("=" * 60)

    bus1, left_arm, head = build_bus1(args.port1)
    bus2, right_arm, base = build_bus2(args.port2)

    print("\nConnecting bus 1...")
    bus1.connect()
    print("Connecting bus 2...")
    bus2.connect()

    print("\n--- Calibrating Bus 1 (left arm + head) ---")
    cal1 = calibrate_bus1(bus1, left_arm, head)

    print("\n--- Calibrating Bus 2 (right arm + wheels) ---")
    cal2 = calibrate_bus2(bus2, right_arm, base)

    combined = {**cal1, **cal2}

    output_path.parent.mkdir(parents=True, exist_ok=True)
    serializable = {}
    for name, mc in combined.items():
        serializable[name] = {
            "id": mc.id,
            "drive_mode": mc.drive_mode,
            "homing_offset": mc.homing_offset,
            "range_min": mc.range_min,
            "range_max": mc.range_max,
        }

    with open(output_path, "w") as f:
        json.dump(serializable, f, indent=2)

    print(f"\nCalibration saved to {output_path}")
    print("You can now start DimOS with: dimos run xlerobot-basic")

    bus1.disconnect(True)
    bus2.disconnect(True)
    print("Done.")


if __name__ == "__main__":
    main()
