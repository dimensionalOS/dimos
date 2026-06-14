"""XLeRobot configuration for DimOS."""

import platform

from pydantic import Field

from dimos.core.module import ModuleConfig


def _default_bus1_port() -> str:
    if platform.system() == "Darwin":
        return "/dev/cu.usbmodem1101"
    return "/dev/ttyACM0"


def _default_bus2_port() -> str:
    if platform.system() == "Darwin":
        return "/dev/cu.usbmodem1201"
    return "/dev/ttyACM1"


class XLeRobotConfig(ModuleConfig):
    """Configuration for XLeRobot connection module.

    The XLeRobot has two serial buses:
      - Bus 1: left arm (6 DOF) + head (2 DOF)
      - Bus 2: right arm (6 DOF) + base wheels (3)

    On macOS, serial ports appear as /dev/cu.usbmodem* instead of /dev/ttyACM*.
    The defaults auto-detect the platform.
    """

    # Serial ports
    port_bus1: str = Field(
        default_factory=_default_bus1_port,
        description="Serial port for bus 1 (left arm + head)",
    )
    port_bus2: str = Field(
        default_factory=_default_bus2_port,
        description="Serial port for bus 2 (right arm + omni base wheels)",
    )

    # Feature flags
    enable_arms: bool = Field(default=True, description="Enable arm motor control")
    enable_head: bool = Field(default=True, description="Enable head motor control")

    # Camera
    camera_index: int = Field(
        default=0,
        description="OpenCV camera device index or /dev/videoN",
    )
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30

    # Omni base kinematics
    wheel_radius: float = Field(default=0.05, description="Omni wheel radius in meters")
    base_radius: float = Field(default=0.125, description="Distance from center to wheel in meters")
    max_wheel_raw: int = Field(default=3000, description="Max raw velocity ticks per wheel")

    # Bus 1 servo IDs (left arm + head)
    left_arm_shoulder_pan_id: int = 1
    left_arm_shoulder_lift_id: int = 2
    left_arm_elbow_flex_id: int = 3
    left_arm_wrist_flex_id: int = 4
    left_arm_wrist_roll_id: int = 5
    left_arm_gripper_id: int = 6
    head_motor_1_id: int = 7
    head_motor_2_id: int = 8

    # Bus 2 servo IDs (right arm + wheels)
    right_arm_shoulder_pan_id: int = 1
    right_arm_shoulder_lift_id: int = 2
    right_arm_elbow_flex_id: int = 3
    right_arm_wrist_flex_id: int = 4
    right_arm_wrist_roll_id: int = 5
    right_arm_gripper_id: int = 6
    wheel_left_id: int = 7
    wheel_back_id: int = 8
    wheel_right_id: int = 9

    # Motor tuning
    disable_torque_on_disconnect: bool = Field(
        default=True,
        description="Disable torque on all motors when disconnecting",
    )
    max_relative_target: int | None = Field(
        default=None,
        description="Max position delta per step for arm safety clamping (None = no limit)",
    )
    use_degrees: bool = Field(
        default=False,
        description="Use degree-based motor normalization instead of range [-100, 100]",
    )

    # PID tuning (applied to arm + head position-mode motors)
    arm_p_coefficient: int = 16
    arm_i_coefficient: int = 0
    arm_d_coefficient: int = 43

    # Calibration
    calibration_file: str | None = Field(
        default=None,
        description="Path to saved calibration JSON. If None, looks for default location.",
    )

    # Connection
    strict_handshake: bool = Field(
        default=False,
        description="If True, fail on connect when any expected motor is missing. "
        "If False, log a warning and continue with the motors that are found.",
    )
