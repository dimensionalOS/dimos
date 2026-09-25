# Copyright 2026 Dimensional Inc.
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

"""Public data types. Units are SI; frames follow REP-103 (X forward, Y left, Z up).

Joint order everywhere: [slab_1_hinge, slab_1_slide, ..., slab_4_hinge, slab_4_slide],
slabs numbered 1..4 from the robot's left.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

N_SLABS = 4
N_JOINTS = 2 * N_SLABS
JOINT_NAMES: tuple[str, ...] = tuple(
    f"slab_{i}_{kind}" for i in range(1, N_SLABS + 1) for kind in ("hinge", "slide")
)


@dataclass
class Measurement:
    """Raw sensor snapshot, what a real robot would expose."""

    time: float
    joint_q: NDArray[np.float64]  # rad / m
    joint_dq: NDArray[np.float64]  # rad/s / m/s
    joint_tau: NDArray[np.float64]  # Nm / N (applied)
    imu_quat: NDArray[np.float64]  # wxyz, hub orientation in world
    imu_gyro: NDArray[np.float64]  # rad/s, hub frame
    imu_acc: NDArray[np.float64]  # m/s^2, hub frame
    foot_force: NDArray[np.float64]  # N, per slab foot (touch sensors)


@dataclass
class Odometry:
    """Hub pose in the odom frame + body-frame velocity."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    quat: NDArray[np.float64] = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))


@dataclass
class JointTargets:
    """Per-joint PD + feedforward command: tau = kp*(q - q_meas) + kd*(dq - dq_meas) + tau_ff."""

    q: NDArray[np.float64] = field(default_factory=lambda: np.zeros(N_JOINTS))
    dq: NDArray[np.float64] = field(default_factory=lambda: np.zeros(N_JOINTS))
    kp: NDArray[np.float64] = field(default_factory=lambda: np.zeros(N_JOINTS))
    kd: NDArray[np.float64] = field(default_factory=lambda: np.zeros(N_JOINTS))
    tau: NDArray[np.float64] = field(default_factory=lambda: np.zeros(N_JOINTS))


@dataclass
class CameraFrame:
    rgb: NDArray[np.uint8]  # HxWx3
    depth: NDArray[np.float32] | None  # HxW, meters
    fovy_deg: float
    time: float


@dataclass
class TarsState:
    time: float
    mode: str  # gait phase: sit, rise, ready, swing, shift, twist, lower
    command: tuple[float, float]  # (vx, wz) currently applied
    odom: Odometry  # estimated (leg kinematics + IMU)
    odom_gt: Odometry | None  # simulator ground truth, None on real hardware
    measurement: Measurement
