from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from .arm_robot import ArmRobot

def get_a1z_robot(
    can_channel: str = ...,
    gravity_comp_factor: float = ...,
    zero_gravity_mode: bool = ...,
    control_freq_hz: int = ...,
    urdf_path: str | Path | None = ...,
    default_kp: npt.NDArray[np.floating[Any]] | None = ...,
    default_kd: npt.NDArray[np.floating[Any]] | None = ...,
    with_gripper: bool = ...,
    gripper_max_torque: float = ...,
) -> ArmRobot: ...
