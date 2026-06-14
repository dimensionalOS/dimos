"""Low-level XLeRobot hardware driver.

Wraps the Feetech servo buses for full robot control:
  - Bus 1: left arm (6 DOF) + head (2 DOF) — position mode
  - Bus 2: right arm (6 DOF) + base wheels (3) — position + velocity mode
  - USB camera via OpenCV

Mirrors the hardware API from XLeRobot/software/src/robots/xlerobot/xlerobot.py
but adapted for headless DimOS operation (no interactive calibration).

Requires: pip install lerobot[feetech]
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from dimos.robot.xlerobot.config import XLeRobotConfig

logger = logging.getLogger(__name__)

try:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    _HAS_LEROBOT = True
except ImportError:
    _HAS_LEROBOT = False


def _degps_to_raw(degps: float) -> int:
    """Convert degrees/sec to Feetech raw velocity ticks."""
    steps_per_deg = 4096.0 / 360.0
    raw = int(round(degps * steps_per_deg))
    return max(-0x8000, min(0x7FFF, raw))


def _raw_to_degps(raw: int) -> float:
    return raw / (4096.0 / 360.0)


LEFT_ARM_MOTOR_NAMES = [
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
]

HEAD_MOTOR_NAMES = ["head_motor_1", "head_motor_2"]

RIGHT_ARM_MOTOR_NAMES = [
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
]

BASE_MOTOR_NAMES = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]

ALL_ARM_JOINT_NAMES = LEFT_ARM_MOTOR_NAMES + RIGHT_ARM_MOTOR_NAMES + HEAD_MOTOR_NAMES


class XLeRobotDriver:
    """Full driver for XLeRobot: dual arms, head, omni base, and camera.

    Handles:
      - Bus 1: left arm (6 position-mode servos) + head (2 position-mode servos)
      - Bus 2: right arm (6 position-mode servos) + wheels (3 velocity-mode servos)
      - USB camera via OpenCV
      - Omni-wheel inverse/forward kinematics
      - Calibration loading from file
      - Safety clamping on arm position targets
    """

    def __init__(self, config: XLeRobotConfig) -> None:
        if not _HAS_LEROBOT:
            raise ImportError(
                "lerobot[feetech] is required for XLeRobot hardware control. "
                "Install with: pip install lerobot[feetech]"
            )

        self._config = config
        self._connected = False
        self._camera: cv2.VideoCapture | None = None
        self._latest_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        # Kinematics matrices (omni base)
        angles = np.radians(np.array([240, 0, 120]) - 90)
        self._kin_matrix = np.array(
            [[np.cos(a), np.sin(a), config.base_radius] for a in angles]
        )
        self._kin_matrix_inv = np.linalg.inv(self._kin_matrix)

        # Motor norm mode
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # Bus 1: left arm + head
        self._bus1: FeetechMotorsBus | None = None
        if config.enable_arms or config.enable_head:
            bus1_motors: dict[str, Motor] = {}
            if config.enable_arms:
                bus1_motors.update(
                    {
                        "left_arm_shoulder_pan": Motor(config.left_arm_shoulder_pan_id, "sts3215", norm_mode_body),
                        "left_arm_shoulder_lift": Motor(config.left_arm_shoulder_lift_id, "sts3215", norm_mode_body),
                        "left_arm_elbow_flex": Motor(config.left_arm_elbow_flex_id, "sts3215", norm_mode_body),
                        "left_arm_wrist_flex": Motor(config.left_arm_wrist_flex_id, "sts3215", norm_mode_body),
                        "left_arm_wrist_roll": Motor(config.left_arm_wrist_roll_id, "sts3215", norm_mode_body),
                        "left_arm_gripper": Motor(config.left_arm_gripper_id, "sts3215", MotorNormMode.RANGE_0_100),
                    }
                )
            if config.enable_head:
                bus1_motors.update(
                    {
                        "head_motor_1": Motor(config.head_motor_1_id, "sts3215", norm_mode_body),
                        "head_motor_2": Motor(config.head_motor_2_id, "sts3215", norm_mode_body),
                    }
                )
            self._bus1 = FeetechMotorsBus(port=config.port_bus1, motors=bus1_motors)

        # Bus 2: right arm + wheels (always created — wheels are required)
        bus2_motors: dict[str, Motor] = {
            "base_left_wheel": Motor(config.wheel_left_id, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_back_wheel": Motor(config.wheel_back_id, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(config.wheel_right_id, "sts3215", MotorNormMode.RANGE_M100_100),
        }
        if config.enable_arms:
            bus2_motors.update(
                {
                    "right_arm_shoulder_pan": Motor(config.right_arm_shoulder_pan_id, "sts3215", norm_mode_body),
                    "right_arm_shoulder_lift": Motor(config.right_arm_shoulder_lift_id, "sts3215", norm_mode_body),
                    "right_arm_elbow_flex": Motor(config.right_arm_elbow_flex_id, "sts3215", norm_mode_body),
                    "right_arm_wrist_flex": Motor(config.right_arm_wrist_flex_id, "sts3215", norm_mode_body),
                    "right_arm_wrist_roll": Motor(config.right_arm_wrist_roll_id, "sts3215", norm_mode_body),
                    "right_arm_gripper": Motor(config.right_arm_gripper_id, "sts3215", MotorNormMode.RANGE_0_100),
                }
            )
        self._bus2 = FeetechMotorsBus(port=config.port_bus2, motors=bus2_motors)

        # Classify motor names for this instance
        self._left_arm_motors = [m for m in (self._bus1.motors if self._bus1 else {}) if m.startswith("left_arm")]
        self._head_motors = [m for m in (self._bus1.motors if self._bus1 else {}) if m.startswith("head")]
        self._right_arm_motors = [m for m in self._bus2.motors if m.startswith("right_arm")]
        self._base_motors = [m for m in self._bus2.motors if m.startswith("base")]
        self._wheel_names = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def arms_enabled(self) -> bool:
        return self._config.enable_arms

    @property
    def head_enabled(self) -> bool:
        return self._config.enable_head

    def connect(self) -> None:
        """Connect to servo buses, load calibration, configure motors, and open camera."""
        logger.info("Connecting XLeRobot...")
        handshake = self._config.strict_handshake

        if self._bus1 is not None:
            logger.info(f"Connecting bus 1 on {self._config.port_bus1}")
            self._bus1.connect(handshake=handshake)

        logger.info(f"Connecting bus 2 on {self._config.port_bus2}")
        self._bus2.connect(handshake=handshake)

        if not handshake:
            self._prune_missing_motors()

        self._load_calibration()
        self._configure_motors()
        self._open_camera()

        self._connected = True
        logger.info("XLeRobot fully connected")

    def _prune_missing_motors(self) -> None:
        """Ping each motor; remove any that don't respond and update name lists."""
        for label, bus in [("bus1", self._bus1), ("bus2", self._bus2)]:
            if bus is None:
                continue
            missing = []
            for name in list(bus.motors):
                if bus.ping(name, num_retry=2) is None:
                    missing.append(name)
            if missing:
                logger.warning(f"{label}: motors not found, disabling: {missing}")
                for name in missing:
                    del bus.motors[name]

        self._left_arm_motors = [m for m in (self._bus1.motors if self._bus1 else {}) if m.startswith("left_arm")]
        self._head_motors = [m for m in (self._bus1.motors if self._bus1 else {}) if m.startswith("head")]
        self._right_arm_motors = [m for m in self._bus2.motors if m.startswith("right_arm")]
        self._base_motors = [m for m in self._bus2.motors if m.startswith("base")]
        self._wheel_names = [n for n in ["base_left_wheel", "base_back_wheel", "base_right_wheel"] if n in self._bus2.motors]

    def _load_calibration(self) -> None:
        """Load calibration from file if available."""
        cal_path = self._resolve_calibration_path()
        if cal_path is None or not cal_path.is_file():
            if self._config.enable_arms or self._config.enable_head:
                logger.warning(
                    "No calibration file found. Arm/head positions may be inaccurate. "
                    "Run `python -m dimos.robot.xlerobot.calibrate` to create one."
                )
            return

        logger.info(f"Loading calibration from {cal_path}")
        try:
            with open(cal_path) as f:
                cal_data = json.load(f)

            if self._bus1 is not None:
                bus1_cal = {
                    k: MotorCalibration(**v) for k, v in cal_data.items() if k in self._bus1.motors
                }
                if bus1_cal:
                    self._bus1.calibration = bus1_cal
                    self._bus1.write_calibration(bus1_cal)

            bus2_cal = {
                k: MotorCalibration(**v) for k, v in cal_data.items() if k in self._bus2.motors
            }
            if bus2_cal:
                self._bus2.calibration = bus2_cal
                self._bus2.write_calibration(bus2_cal)

            logger.info("Calibration loaded successfully")
        except Exception as e:
            logger.warning(f"Failed to load calibration: {e}")

    def _resolve_calibration_path(self) -> Path | None:
        if self._config.calibration_file:
            return Path(self._config.calibration_file)
        default = Path.home() / ".cache" / "xlerobot" / "calibration.json"
        if default.is_file():
            return default
        return None

    def _configure_motors(self) -> None:
        """Set operating modes and PID for all motors."""
        cfg = self._config

        if self._bus1 is not None:
            self._bus1.disable_torque()
            self._bus1.configure_motors()
            for name in self._left_arm_motors + self._head_motors:
                self._bus1.write("Operating_Mode", name, OperatingMode.POSITION.value)
                self._bus1.write("P_Coefficient", name, cfg.arm_p_coefficient)
                self._bus1.write("I_Coefficient", name, cfg.arm_i_coefficient)
                self._bus1.write("D_Coefficient", name, cfg.arm_d_coefficient)
            self._bus1.enable_torque()

        self._bus2.disable_torque()
        self._bus2.configure_motors()
        for name in self._right_arm_motors:
            self._bus2.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self._bus2.write("P_Coefficient", name, cfg.arm_p_coefficient)
            self._bus2.write("I_Coefficient", name, cfg.arm_i_coefficient)
            self._bus2.write("D_Coefficient", name, cfg.arm_d_coefficient)
        for name in self._base_motors:
            self._bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self._bus2.enable_torque()

    def _open_camera(self) -> None:
        self._camera = cv2.VideoCapture(self._config.camera_index)
        if self._camera.isOpened():
            self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.camera_width)
            self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.camera_height)
            self._camera.set(cv2.CAP_PROP_FPS, self._config.camera_fps)
            logger.info(f"Camera {self._config.camera_index} opened")
        else:
            logger.warning(f"Camera {self._config.camera_index} failed to open — vision disabled")
            self._camera = None

    # ---- Base movement ----

    def _body_to_wheel_raw(self, x: float, y: float, theta_deg: float) -> dict[str, int]:
        """Convert body-frame velocity to per-wheel raw commands."""
        theta_rad = np.radians(theta_deg)
        wheel_linear = self._kin_matrix.dot([x, y, theta_rad])
        wheel_degps = np.degrees(wheel_linear / self._config.wheel_radius)

        raw_abs = np.abs(wheel_degps) * (4096.0 / 360.0)
        peak = raw_abs.max()
        if peak > self._config.max_wheel_raw:
            wheel_degps *= self._config.max_wheel_raw / peak

        return {
            name: _degps_to_raw(deg)
            for name, deg in zip(self._wheel_names, wheel_degps)
        }

    def _wheel_raw_to_body(self, raw: dict[str, int]) -> dict[str, float]:
        """Convert wheel feedback back to body-frame velocities."""
        wheel_degps = np.array([_raw_to_degps(raw[n]) for n in self._wheel_names])
        wheel_linear = np.radians(wheel_degps) * self._config.wheel_radius
        body = self._kin_matrix_inv.dot(wheel_linear)
        return {"x_vel": body[0], "y_vel": body[1], "theta_vel": np.degrees(body[2])}

    def move(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Send base velocity command.

        Args:
            x: forward m/s
            y: lateral m/s (positive = left)
            theta: rotation deg/s (positive = ccw)
        """
        if not self._wheel_names:
            logger.warning("No wheel motors available — ignoring move command")
            return
        cmds = self._body_to_wheel_raw(x, y, theta)
        cmds = {k: v for k, v in cmds.items() if k in self._wheel_names}
        if cmds:
            self._bus2.sync_write("Goal_Velocity", cmds)

    def stop_base(self) -> None:
        if not self._wheel_names:
            return
        self._bus2.sync_write(
            "Goal_Velocity", dict.fromkeys(self._wheel_names, 0), num_retry=5
        )

    def read_wheel_velocities(self) -> dict[str, float]:
        """Read current body-frame velocities from wheel encoders."""
        if not self._base_motors:
            return {"x_vel": 0.0, "y_vel": 0.0, "theta_vel": 0.0}
        raw = self._bus2.sync_read("Present_Velocity", self._base_motors)
        return self._wheel_raw_to_body(raw)

    # ---- Arm control ----

    def move_arm(self, arm: str, positions: dict[str, float]) -> None:
        """Set arm joint position targets.

        Args:
            arm: "left" or "right"
            positions: dict of joint_name -> target position value
        """
        if arm == "left":
            if self._bus1 is None:
                raise RuntimeError("Bus 1 not initialized — arms disabled")
            goal = {k: v for k, v in positions.items() if k in self._left_arm_motors}
            if self._config.max_relative_target is not None:
                goal = self._clamp_arm_positions(self._bus1, goal, self._left_arm_motors)
            if goal:
                self._bus1.sync_write("Goal_Position", goal)
        elif arm == "right":
            goal = {k: v for k, v in positions.items() if k in self._right_arm_motors}
            if self._config.max_relative_target is not None:
                goal = self._clamp_arm_positions(self._bus2, goal, self._right_arm_motors)
            if goal:
                self._bus2.sync_write("Goal_Position", goal)
        else:
            raise ValueError(f"Unknown arm '{arm}', expected 'left' or 'right'")

    def _clamp_arm_positions(
        self, bus: FeetechMotorsBus, goal: dict[str, float], motor_names: list[str]
    ) -> dict[str, float]:
        """Clamp goal positions to be within max_relative_target of current."""
        present = bus.sync_read("Present_Position", [m for m in motor_names if m in goal])
        max_delta = self._config.max_relative_target
        assert max_delta is not None
        clamped = {}
        for name, target in goal.items():
            if name in present:
                current = present[name]
                delta = target - current
                if abs(delta) > max_delta:
                    delta = max_delta if delta > 0 else -max_delta
                clamped[name] = current + delta
            else:
                clamped[name] = target
        return clamped

    def move_head(self, pan: float, tilt: float) -> None:
        """Set head joint position targets.

        Args:
            pan: head_motor_1 target position
            tilt: head_motor_2 target position
        """
        if self._bus1 is None:
            raise RuntimeError("Bus 1 not initialized — head disabled")
        goal = {"head_motor_1": pan, "head_motor_2": tilt}
        if self._config.max_relative_target is not None:
            goal = self._clamp_arm_positions(self._bus1, goal, self._head_motors)
        self._bus1.sync_write("Goal_Position", goal)

    def open_gripper(self, arm: str) -> None:
        """Open the gripper on the specified arm (100 = fully open)."""
        self._set_gripper(arm, 100.0)

    def close_gripper(self, arm: str) -> None:
        """Close the gripper on the specified arm (0 = fully closed)."""
        self._set_gripper(arm, 0.0)

    def _set_gripper(self, arm: str, value: float) -> None:
        if arm == "left":
            if self._bus1 is None:
                raise RuntimeError("Bus 1 not initialized — arms disabled")
            self._bus1.sync_write("Goal_Position", {"left_arm_gripper": value})
        elif arm == "right":
            self._bus2.sync_write("Goal_Position", {"right_arm_gripper": value})
        else:
            raise ValueError(f"Unknown arm '{arm}', expected 'left' or 'right'")

    def read_joint_positions(self) -> dict[str, float]:
        """Read current positions of all arm and head joints.

        Returns:
            Dict mapping joint name -> position value (14 entries max).
        """
        positions: dict[str, float] = {}
        if self._bus1 is not None and self._left_arm_motors:
            positions.update(self._bus1.sync_read("Present_Position", self._left_arm_motors))
        if self._bus1 is not None and self._head_motors:
            positions.update(self._bus1.sync_read("Present_Position", self._head_motors))
        if self._right_arm_motors:
            positions.update(self._bus2.sync_read("Present_Position", self._right_arm_motors))
        return positions

    def home_arms(self) -> None:
        """Move all arm and head joints to zero position."""
        if self._bus1 is not None:
            zeros_bus1 = dict.fromkeys(self._left_arm_motors + self._head_motors, 0.0)
            self._bus1.sync_write("Goal_Position", zeros_bus1)
        if self._right_arm_motors:
            zeros_bus2 = dict.fromkeys(self._right_arm_motors, 0.0)
            self._bus2.sync_write("Goal_Position", zeros_bus2)

    # ---- Full observation (mirrors XLerobot.get_observation) ----

    def get_observation(self) -> dict[str, Any]:
        """Read all 17 state features + camera frame.

        Returns dict with keys like 'left_arm_shoulder_pan.pos', 'x.vel', etc.
        """
        obs: dict[str, Any] = {}

        if self._bus1 is not None and self._left_arm_motors:
            left_pos = self._bus1.sync_read("Present_Position", self._left_arm_motors)
            obs.update({f"{k}.pos": v for k, v in left_pos.items()})

        if self._bus1 is not None and self._head_motors:
            head_pos = self._bus1.sync_read("Present_Position", self._head_motors)
            obs.update({f"{k}.pos": v for k, v in head_pos.items()})

        if self._right_arm_motors:
            right_pos = self._bus2.sync_read("Present_Position", self._right_arm_motors)
            obs.update({f"{k}.pos": v for k, v in right_pos.items()})

        wheel_vel = self._bus2.sync_read("Present_Velocity", self._base_motors)
        body_vel = self._wheel_raw_to_body(wheel_vel)
        obs["x.vel"] = body_vel["x_vel"]
        obs["y.vel"] = body_vel["y_vel"]
        obs["theta.vel"] = body_vel["theta_vel"]

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send a full action dict (mirrors XLerobot.send_action).

        Arm/head keys ending in '.pos' set Goal_Position.
        Velocity keys ('x.vel', 'y.vel', 'theta.vel') set wheel Goal_Velocity.
        """
        left_arm = {k.replace(".pos", ""): v for k, v in action.items()
                     if k.startswith("left_arm_") and k.endswith(".pos")}
        right_arm = {k.replace(".pos", ""): v for k, v in action.items()
                      if k.startswith("right_arm_") and k.endswith(".pos")}
        head = {k.replace(".pos", ""): v for k, v in action.items()
                 if k.startswith("head_") and k.endswith(".pos")}
        base_vel = {k: v for k, v in action.items() if k.endswith(".vel")}

        if self._config.max_relative_target is not None:
            if left_arm and self._bus1 is not None:
                left_arm = self._clamp_arm_positions(self._bus1, left_arm, self._left_arm_motors)
            if right_arm:
                right_arm = self._clamp_arm_positions(self._bus2, right_arm, self._right_arm_motors)
            if head and self._bus1 is not None:
                head = self._clamp_arm_positions(self._bus1, head, self._head_motors)

        if left_arm and self._bus1 is not None:
            self._bus1.sync_write("Goal_Position", left_arm)
        if head and self._bus1 is not None:
            self._bus1.sync_write("Goal_Position", head)
        if right_arm:
            self._bus2.sync_write("Goal_Position", right_arm)

        if base_vel:
            wheel_cmds = self._body_to_wheel_raw(
                base_vel.get("x.vel", 0.0),
                base_vel.get("y.vel", 0.0),
                base_vel.get("theta.vel", 0.0),
            )
            self._bus2.sync_write("Goal_Velocity", wheel_cmds)

        return action

    # ---- Camera ----

    def read_camera(self) -> np.ndarray | None:
        """Capture a single camera frame (BGR numpy array)."""
        if self._camera is None:
            return None
        ret, frame = self._camera.read()
        if ret:
            with self._frame_lock:
                self._latest_frame = frame
            return frame
        return None

    @property
    def latest_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self._latest_frame

    # ---- Lifecycle ----

    def disconnect(self) -> None:
        if not self._connected:
            return

        self.stop_base()

        if self._config.enable_arms or self._config.enable_head:
            if self._bus1 is not None:
                self.home_arms()
                time.sleep(0.5)
                self._bus1.disconnect(self._config.disable_torque_on_disconnect)

        self._bus2.disconnect(self._config.disable_torque_on_disconnect)

        if self._camera is not None:
            self._camera.release()

        self._connected = False
        logger.info("XLeRobot disconnected")
