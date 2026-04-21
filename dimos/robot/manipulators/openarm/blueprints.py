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

"""OpenArm coordinator blueprints (single-arm + bimanual).

Usage:
    dimos run coordinator-openarm-mock         # Bimanual, mock adapters
    dimos run coordinator-openarm-left         # Single arm, real CAN
    dimos run coordinator-openarm-right        # Single arm, real CAN
    dimos run coordinator-openarm-bimanual     # Both arms, real CAN

The CAN interface each arm uses is set by the ``LEFT_CAN``/``RIGHT_CAN``
constants below. Linux enumerates gs_usb adapters in USB-discovery order
which isn't guaranteed stable, so if your arms come up "swapped" just flip
these two values and rerun — no other code changes needed.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.openarm import (
    OPENARM_V10_FK_MODEL,
    openarm_arm as _openarm,
    openarm_single as _openarm_single,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# ── Mock bimanual: no hardware, great for verifying wiring ─────────────
_mock_left = _openarm(side="left", name="left_arm")
_mock_right = _openarm(side="right", name="right_arm")

coordinator_openarm_mock = ControlCoordinator.blueprint(
    hardware=[_mock_left.to_hardware_component(), _mock_right.to_hardware_component()],
    tasks=[
        _mock_left.to_task_config(),
        _mock_right.to_task_config(),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# ── Single-arm hardware blueprints (first real bring-up targets) ───────
# CAN interface each physical arm is on. Linux assigns can0/can1 in USB
# enumeration order which isn't guaranteed stable — if the arms come up
# swapped, flip these two values.
LEFT_CAN = "can1"
RIGHT_CAN = "can0"

# Flip to False to skip the CTRL_MODE=MIT write at connect-time — useful for
# verifying the setting persists across power cycles. Leave True for normal
# operation (idempotent; ensures motors work even if they were reflashed /
# replaced / factory-reset).
AUTO_SET_MIT_MODE = True

_HW_KW = dict(adapter_type="openarm",
              adapter_kwargs={"auto_set_mit_mode": AUTO_SET_MIT_MODE})
_left_hw = _openarm(side="left", name="left_arm", address=LEFT_CAN, **_HW_KW)
_right_hw = _openarm(side="right", name="right_arm", address=RIGHT_CAN, **_HW_KW)

coordinator_openarm_left = ControlCoordinator.blueprint(
    hardware=[_left_hw.to_hardware_component()],
    tasks=[_left_hw.to_task_config()],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

coordinator_openarm_right = ControlCoordinator.blueprint(
    hardware=[_right_hw.to_hardware_component()],
    tasks=[_right_hw.to_task_config()],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# ── Bimanual hardware blueprint ────────────────────────────────────────
coordinator_openarm_bimanual = ControlCoordinator.blueprint(
    hardware=[_left_hw.to_hardware_component(), _right_hw.to_hardware_component()],
    tasks=[
        _left_hw.to_task_config(),
        _right_hw.to_task_config(),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# ── Planner + coordinator (mock): Drake plans, mock adapters execute ────
# Great for visualizing motions in Meshcat with no hardware.
openarm_mock_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_mock_left.to_robot_model_config(), _mock_right.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_mock_left.to_hardware_component(), _mock_right.to_hardware_component()],
        tasks=[
            _mock_left.to_task_config(),
            _mock_right.to_task_config(),
        ],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# ── Planner + coordinator (real hw): plan & execute on both arms ────────
openarm_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_left_hw.to_robot_model_config(), _right_hw.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_left_hw.to_hardware_component(), _right_hw.to_hardware_component()],
        tasks=[
            _left_hw.to_task_config(),
            _right_hw.to_task_config(),
        ],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# ── Keyboard teleop (single arm, mock) ──────────────────────────────────
# pygame keyboard UI → Cartesian IK (Drake) → mock coordinator execution,
# with Drake/Meshcat visualization. Good for testing the single-arm URDF
# and IK without touching hardware.
_teleop_cfg = _openarm_single(name="arm")

keyboard_teleop_openarm_mock = autoconnect(
    KeyboardTeleopModule.blueprint(model_path=str(OPENARM_V10_FK_MODEL), ee_joint_id=_teleop_cfg.dof),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_teleop_cfg.to_hardware_component()],
        tasks=[
            _teleop_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_arm",
                model_path=OPENARM_V10_FK_MODEL,
                ee_joint_id=_teleop_cfg.dof,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_teleop_cfg.to_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# ── Keyboard teleop (single arm, real hw on can0) ───────────────────────
_teleop_hw_cfg = _openarm_single(name="arm", adapter_type="openarm", address=LEFT_CAN)

keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(model_path=str(OPENARM_V10_FK_MODEL), ee_joint_id=_teleop_hw_cfg.dof),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_teleop_hw_cfg.to_hardware_component()],
        tasks=[
            _teleop_hw_cfg.to_task_config(
                task_type="cartesian_ik",
                task_name="cartesian_ik_arm",
                model_path=OPENARM_V10_FK_MODEL,
                ee_joint_id=_teleop_hw_cfg.dof,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_teleop_hw_cfg.to_robot_model_config()],
        enable_viz=True,
    ),
).transports(
    {
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


__all__ = [
    "coordinator_openarm_bimanual",
    "coordinator_openarm_left",
    "coordinator_openarm_mock",
    "coordinator_openarm_right",
    "keyboard_teleop_openarm",
    "keyboard_teleop_openarm_mock",
    "openarm_mock_planner_coordinator",
    "openarm_planner_coordinator",
]
