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

"""Unitree G1 GR00T whole-body-control blueprint.

One blueprint, ``--simulation`` flag picks the backend:

Real hardware (default):
    G1WholeBodyConnection (DDS rt/lowstate <-> rt/lowcmd) + transport_lcm
    whole-body adapter. 500 Hz tick. Safety profile: unarmed + dry-run on
    start; activate explicitly through ControlCoordinator RPC after
    verifying commands. The policy ramps from the current pose to its
    bent-knee default over 10 s before taking torque control. The 14 arm
    joints are held at the relaxed GR00T-trained default via a lower-priority
    servo task.

Sim (``--simulation``):
    MujocoSimModule (in-process MuJoCo + SHM) + sim_mujoco_g1 adapter.
    50 Hz tick (matches the rate the policy was trained at). No arming
    ramp, no dry-run, no servo_arms -- sim physics doesn't gravity-collapse
    the arms between trajectories. Optional passive viewer runs in the
    engine subprocess; flip on via
    ``-o mujocosimmodule.engine_mode=subprocess -o mujocosimmodule.headless=false``
    so the viewer lives on that subprocess's main thread.

Usage:
    dimos run unitree-g1-groot-wbc                 # real hardware
    dimos --simulation run unitree-g1-groot-wbc    # sim

Overrides (replace the old env-var dance):
    dimos run unitree-g1-groot-wbc \\
        -o g1wholebodyconnection.network_interface=enp2s0
    dimos --simulation run unitree-g1-groot-wbc \\
        -o mujocosimmodule.engine_mode=subprocess -o mujocosimmodule.headless=false
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.data import LfsPath

# Lazy data handles. LfsPath only triggers the LFS pull on first
# str()/open(); using ``get_data(...)`` at import time would block the
# whole CLI on a multi-GB download every time the module is imported.
_GROOT_MODEL_DIR = LfsPath("groot")
_MJCF_PATH = LfsPath("mujoco_sim/g1_gear_wbc.xml")

_adapter_address: str | Path

if global_config.simulation:
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    # Sim backend: MuJoCo engine via SHM.
    _backend = MujocoSimModule.blueprint(
        address=_MJCF_PATH,
        headless=True,
        dof=29,
        enable_color=False,
        enable_depth=False,
        enable_pointcloud=False,
        inject_legacy_assets=True,
    )
    # MujocoSimModule's ``odom`` Out is the sole producer of ``/odom``
    # now — the coordinator no longer polls the whole-body adapter for
    # base pose (read_odom was dropped from the Protocol). autoconnect
    # maps ``(odom, PoseStamped)`` to ``/odom`` by default; no override.
    _adapter_type = "sim_mujoco_g1"
    _adapter_address = _MJCF_PATH
    _tick_rate = 50.0
    _auto_arm = True
    _auto_dry_run = False
    _default_ramp_seconds = 0.0
    _decimation: int | None = 1
    # Sim physics holds the arms between trajectories on its own -- no
    # servo task needed.
    _arm_holder: TaskConfig | None = None
else:
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

    # Real-hw backend: DDS connection module + transport_lcm adapter.
    _backend = G1WholeBodyConnection.blueprint(release_sport_mode=True)
    _adapter_type = "transport_lcm"
    _adapter_address = ""
    _tick_rate = 500.0
    # Real hardware: come up unarmed + dry-run; operator must click
    # Activate (10 s ramp) after verifying commands.
    _auto_arm = False
    _auto_dry_run = True
    _default_ramp_seconds = 10.0
    _decimation = None  # task default (10) pairs with 500 Hz tick.
    # Real hardware needs the arms held -- kd damping alone would let
    # them sag toward singular configurations between trajectories.
    _arm_holder = TaskConfig(
        name="servo_arms",
        type="servo",
        joint_names=g1_arms,
        priority=10,
        default_positions=ARM_DEFAULT_POSE,
        auto_start=True,
    )


_coordinator = ControlCoordinator.blueprint(
    tick_rate=_tick_rate,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type=_adapter_type,
            address=_adapter_address,
            auto_enable=True,
            wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="g1_groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            model_path=_GROOT_MODEL_DIR,
            hardware_id="g1",
            auto_start=True,
            auto_arm=_auto_arm,
            auto_dry_run=_auto_dry_run,
            default_ramp_seconds=_default_ramp_seconds,
            decimation=_decimation,
        ),
        *([_arm_holder] if _arm_holder is not None else []),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("twist_command", Twist): LCMTransport("/g1/cmd_vel", Twist),
        # Real-hw only: the transport_lcm adapter speaks to
        # G1WholeBodyConnection over these topics. autoconnect already
        # matches by (name, type) so sim doesn't need them -- they're
        # harmless when the sim engine doesn't expose those ports.
        ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
        ("imu", Imu): LCMTransport("/g1/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport("/g1/motor_command", MotorCommandArray),
    }
)

unitree_g1_groot_wbc = autoconnect(_backend, _coordinator)

__all__ = ["unitree_g1_groot_wbc"]
