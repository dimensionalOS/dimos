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

Runs the ControlCoordinator at 500 Hz with two tasks:

  - ``groot_wbc``  (priority 50) claims legs + waist (15 DOF) and runs
    the GR00T balance / walk ONNX policies at 50 Hz.
  - ``servo_arms`` (priority 10) claims the 14 arm joints and holds
    them at a configured relaxed pose.  No timeout — the task holds
    until an external caller sends new arm targets.

Velocity commands come in over LCM on ``/g1/cmd_vel`` as a Twist.  The
coordinator's ``twist_command`` dispatcher routes them into the task.

Architecture:
    Twist/cmd_vel ──▶ coordinator twist_command ──▶ GrootWBCTask
                                                   │
    ControlCoordinator ──joint_state──▶ LCM /coordinator/joint_state
                       ◀─joint_command── LCM /g1/joint_command
                              │
                    WholeBodyAdapter:
                      --simulation    → SimMujocoG1WholeBodyAdapter
                                        (MujocoConnection subprocess,
                                         low-level passthrough)
                      real hardware   → UnitreeG1LowLevelAdapter (DDS)

Usage:
    dimos --simulation run unitree-g1-groot-wbc          # MuJoCo subprocess, viewer pops
    ROBOT_INTERFACE=enp86s0 dimos run unitree-g1-groot-wbc   # real robot

Environment:
    ROBOT_INTERFACE   DDS network interface for real robot (default "enp86s0").
                      Ignored under --simulation.
    DIMOS_DDS_DOMAIN  DDS domain id for real robot (default 0). Ignored
                      under --simulation.
    GROOT_MODEL_DIR   Directory containing balance.onnx + walk.onnx
                      (default "data/groot").
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_humanoid_joints,
)
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.std_msgs.Bool import Bool as DimosBool
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

_g1_joints = make_humanoid_joints("g1")
_g1_legs_waist = _g1_joints[:15]  # indices 0..14 — legs (12) + waist (3)
_g1_arms = _g1_joints[15:]  # indices 15..28 — left arm (7) + right arm (7)

# Per-joint PD gains, 29 entries in DDS motor order.  Lifted verbatim
# from g1-control-api/configs/g1_groot_wbc.yaml, which itself copies
# GR00T-WBC's g1_29dof_gear_wbc.yaml reference config.  These gains
# were the ones the balance / walk ONNX policies were trained against
# — diverging from them on real hardware risks instability.
_G1_GROOT_KP = [
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # left leg
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # right leg
    250.0,
    250.0,
    250.0,  # waist
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # left arm
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # right arm
]
_G1_GROOT_KD = [
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # left leg
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # right leg
    5.0,
    5.0,
    5.0,  # waist
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # left arm
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # right arm
]

# Relaxed arms-down pose.  Values taken from
# g1_control/backends/groot_wbc_backend.py:DEFAULT_29[15:] (all zeros),
# which is the zero-offset pose the policy was trained against.
# Operators can override at runtime by publishing joint targets on the
# arms via the joint_command transport.
_ARM_DEFAULT_POSE = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # left arm (7 DOF)
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # right arm (7 DOF)
]

_adapter_type = "sim_mujoco_g1" if global_config.simulation else "unitree_g1"
_address = None if global_config.simulation else os.getenv("ROBOT_INTERFACE", "enp86s0")

# Arming defaults: sim auto-arms (the MuJoCo subprocess holds the MJCF
# pose until first command, no ramp needed); real hardware comes up
# unarmed + dry-run so the operator can see computed commands before
# committing motor torques, then hit Activate in the dashboard for a
# 10 s ramp to the bent-knee default (mirrors g1-control-api).
_AUTO_ARM = global_config.simulation
_AUTO_DRY_RUN = not global_config.simulation
_DEFAULT_RAMP_SECONDS = 0.0 if global_config.simulation else 10.0

_g1_coordinator = (
    control_coordinator(
        tick_rate=500.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="g1",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_g1_joints,
                adapter_type=_adapter_type,
                address=_address,
                domain_id=int(os.getenv("DIMOS_DDS_DOMAIN", "0")),
                auto_enable=True,
                kp=_G1_GROOT_KP,
                kd=_G1_GROOT_KD,
            ),
        ],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="groot_wbc",
                joint_names=_g1_legs_waist,
                priority=50,
                model_path=os.getenv("GROOT_MODEL_DIR", "data/groot"),
                hardware_id="g1",
                auto_start=True,
                auto_arm=_AUTO_ARM,
                auto_dry_run=_AUTO_DRY_RUN,
                default_ramp_seconds=_DEFAULT_RAMP_SECONDS,
            ),
            TaskConfig(
                name="servo_arms",
                type="servo",
                joint_names=_g1_arms,
                priority=10,
                default_positions=_ARM_DEFAULT_POSE,
                auto_start=True,
            ),
        ],
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("twist_command", Twist): LCMTransport("/g1/cmd_vel", Twist),
            ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
            ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
        }
    )
    .global_config(
        # Picked up by MujocoConnection → mujoco_process.py when the blueprint
        # is run with --simulation.  robot_model selects which MJCF the sim
        # child loads; mujoco_room wraps it in a flat floor (vs the default
        # "office1" room used by the perceptive G1 sim blueprint).
        robot_model="unitree_g1",
        mujoco_room="empty",
    )
)


# WASD teleop via the web dashboard (http://localhost:7779/) served by
# WebsocketVisModule.  The bundled React command-center at
# ``data/command_center.html`` includes a KeyboardControlPanel that
# captures W/S/A/D on keydown/keyup and emits ``move_command`` events
# which the module re-publishes on its ``cmd_vel`` port.  We route that
# over LCM to the coordinator's ``twist_command`` port on /g1/cmd_vel.
#
# This replaces the pygame-based ``keyboard_teleop`` module because
# pygame's pygame.display.set_mode() calls NSWindow on macOS, and Cocoa
# rejects NSWindow creation from non-main threads — which is where
# dimos runs module code.  A browser tab has no such constraint.
_g1_ws_vis = websocket_vis().transports(
    {
        ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
        ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
        ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
    },
)

unitree_g1_groot_wbc = autoconnect(_g1_coordinator, _g1_ws_vis)

__all__ = ["_ARM_DEFAULT_POSE", "unitree_g1_groot_wbc"]
