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

"""
Blueprints for VR teleoperation.

This module provides declarative blueprints for VR teleoperation with any robot.

Architecture (with connectors):
    VRTeleopModule (VR calibration, delta computation)
        ↓ Calibrate button → calibrate VR
        ↓ Computes: delta = current_controller - initial_controller
        ↓ Routes through connector.transform_delta()
        ↓ Publishes: robot commands (PoseStamped for arms, Twist for quadrupeds)
        ↓
    Robot Driver (any manipulator/quadruped driver)

Usage with connectors:
    from dimos.teleop.connectors import ArmConnector, ArmConnectorConfig
    from dimos.teleop.devices.vr_teleop_module import vr_teleop_module
    from dimos.core.blueprints import autoconnect

    # Create connectors for each input
    left_arm = ArmConnector(ArmConnectorConfig(dummy_driver=True))
    right_arm = ArmConnector(ArmConnectorConfig(dummy_driver=True))

    my_system = autoconnect(
        vr_teleop_module(
            num_inputs=2,
            enable_inputs=[True, True],
            input_labels=["left_vr", "right_vr"],
            connectors=[left_arm, right_arm],
        ),
    )
"""

from dimos_lcm.geometry_msgs import Transform as LCMTransform

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.std_msgs import Bool, Float32
from dimos.teleop.connectors import ArmConnector, ArmConnectorConfig, QuadrupedConnector, QuadrupedConnectorConfig
from dimos.teleop.devices.vr_teleop_module import vr_teleop_module

# =============================================================================
# Quest3 Teleoperation Blueprint (with connectors)
# =============================================================================

# Create connectors: left controller → arm (index 0), right controller → quadruped (index 2)
# Index 0: PoseStamped output (controller_delta_0)
# Index 1: disabled
# Index 2: Twist output (controller_delta_2)
_left_arm_connector = ArmConnector(
    ArmConnectorConfig(
        driver_module_name="LeftArmDriver",
        dummy_driver=True,
    )
)
_right_quadruped_connector = QuadrupedConnector(
    QuadrupedConnectorConfig(
        driver_module_name="Go2Driver",
        dummy_driver=True,
    )
)

quest3_teleop = autoconnect(
    vr_teleop_module(
        num_inputs=3,
        enable_inputs=[True, False, True],
        input_labels=["left_vr", "unused", "right_vr"],
        connectors=[_left_arm_connector, None, _right_quadruped_connector],
        visualize_in_rerun=True,
        safety_limits=True,
    ),
).transports(
    {
        # VRTeleopModule inputs (from external LCM - Deno bridge)
        ("vr_left_transform", LCMTransform): LCMTransport("/vr_left_transform", LCMTransform),
        ("vr_right_transform", LCMTransform): LCMTransport("/vr_right_transform", LCMTransform),
        ("vr_trigger_0", Float32): LCMTransport("/vr_trigger_0", Float32),
        ("vr_trigger_1", Float32): LCMTransport("/vr_trigger_1", Float32),
        ("teleop_enable", Bool): LCMTransport("/vr_teleop_enable", Bool),
        # VRTeleopModule outputs - add transports here when connecting to robot drivers
        # ("controller_delta_0", PoseStamped): LCMTransport("/left_arm_command", PoseStamped),
        # ("controller_delta_1", PoseStamped): LCMTransport("/right_arm_command", PoseStamped),
        # ("trigger_value_0", Bool): LCMTransport("/left_gripper_command", Bool),
        # ("trigger_value_1", Bool): LCMTransport("/right_gripper_command", Bool),
    }
)

__all__ = [
    "quest3_teleop",
]
