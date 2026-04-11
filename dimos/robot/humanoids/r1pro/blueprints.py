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

"""R1 Pro dual-arm blueprints (single URDF, two 7-DOF arms).

Usage:
    dimos run r1pro-dual-mock             # Mock coordinator only
    dimos run r1pro-planner-coordinator   # Planner + coordinator (plan & execute, mock positions)
    dimos run r1pro-full                  # Real hardware: arms + chassis + all sensors
    dimos run r1pro-planner-full          # Planner + real hardware (plans from actual positions)
"""

from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.galaxea import (
    r1pro_arm,
    r1pro_chassis,
    r1pro_torso,
    r1pro_upper_body,
    r1pro_whole_body,
)

_left = r1pro_arm(side="left")
_right = r1pro_arm(side="right")

# Mock dual-arm coordinator (no planner, no visualization)
r1pro_dual_mock = ControlCoordinator.blueprint(
    hardware=[_left.to_hardware_component(), _right.to_hardware_component()],
    tasks=[_left.to_task_config(), _right.to_task_config()],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Planner + coordinator (plan, preview in Meshcat, execute via mock adapters)
r1pro_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            _left.to_robot_model_config(),
            _right.to_robot_model_config(),
        ],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_left.to_hardware_component(), _right.to_hardware_component()],
        tasks=[_left.to_task_config(), _right.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Real hardware: both arms + chassis with all sensor streams.
# Sensor data (cameras, LiDAR, IMUs) is published by the adapters directly
# to independent LCM transports — no coordinator changes needed:
#   /r1pro/left_arm/wrist_color, /r1pro/left_arm/wrist_depth
#   /r1pro/right_arm/wrist_color, /r1pro/right_arm/wrist_depth
#   /r1pro/chassis/head, /r1pro/chassis/chassis_*, /r1pro/chassis/lidar
#   /r1pro/chassis/imu_chassis, /r1pro/chassis/imu_torso
_left_real = r1pro_arm(side="left", adapter_type="r1pro_arm", add_gripper=True)
_right_real = r1pro_arm(side="right", adapter_type="r1pro_arm", add_gripper=True)
_chassis = r1pro_chassis()

r1pro_full = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        _left_real.to_hardware_component(),
        _right_real.to_hardware_component(),
        _chassis,
    ],
    tasks=[
        _left_real.to_task_config(),
        _right_real.to_task_config(),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=make_twist_base_joints("chassis"),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Planner + real hardware — plans from actual joint positions.
# Requires ROS 2 + hardware connected.
r1pro_planner_full = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            _left_real.to_robot_model_config(),
            _right_real.to_robot_model_config(),
        ],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            _left_real.to_hardware_component(),
            _right_real.to_hardware_component(),
            _chassis,
        ],
        tasks=[
            _left_real.to_task_config(),
            _right_real.to_task_config(),
            TaskConfig(
                name="vel_chassis",
                type="velocity",
                joint_names=make_twist_base_joints("chassis"),
                priority=10,
            ),
        ],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


_torso_real = r1pro_torso()

# Real hardware: arms + torso + chassis, each as a separate adapter.
# Good for testing torso independently or for planners that want to
# command each subsystem separately.
r1pro_full_with_torso = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        _left_real.to_hardware_component(),
        _right_real.to_hardware_component(),
        _chassis,
        _torso_real,
    ],
    tasks=[
        _left_real.to_task_config(),
        _right_real.to_task_config(),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=make_twist_base_joints("chassis"),
            priority=10,
        ),
        TaskConfig(
            name="pos_torso",
            type="trajectory",
            joint_names=["torso/joint1", "torso/joint2", "torso/joint3", "torso/joint4"],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
    }
)

# Upper-body composite (18-DOF): mock adapters — no hardware required.
# Useful for unit tests and offline planner development.
_upper_body_mock = r1pro_upper_body(adapter_type="mock")
_upper_body_mock_joints = _upper_body_mock.joints

r1pro_upper_body_mock = ControlCoordinator.blueprint(
    hardware=[_upper_body_mock],
    tasks=[
        TaskConfig(
            name="pos_upper_body",
            type="trajectory",
            joint_names=_upper_body_mock_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
    }
)

# Upper-body composite (18-DOF): real hardware + chassis.
# Single adapter owns torso + left arm + right arm; chassis remains
# a separate TwistBaseAdapter.  Use this for whole-body policies and
# 18-DOF joint commands.
_upper_body_real = r1pro_upper_body()
_upper_body_joints = _upper_body_real.joints

r1pro_upper_body_full = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_upper_body_real, _chassis],
    tasks=[
        TaskConfig(
            name="pos_upper_body",
            type="trajectory",
            joint_names=_upper_body_joints,
            priority=10,
        ),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=make_twist_base_joints("chassis"),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
    }
)


# Whole-body controller (WholeBodyAdapter, 18-DOF) + chassis.
# The WholeBodyAdapter gives the coordinator per-motor MotorCommand control
# (q, dq, kp, kd, tau) over the full upper body.  The chassis stays as a
# separate TwistBaseAdapter (velocity-controlled).
_r1pro_whole_body = r1pro_whole_body()
_r1pro_wb_joints = _r1pro_whole_body.joints

r1pro_whole_body_full = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_r1pro_whole_body, _chassis],
    tasks=[
        TaskConfig(
            name="servo_r1pro",
            type="servo",
            joint_names=_r1pro_wb_joints,
            priority=10,
        ),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=make_twist_base_joints("chassis"),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/r1pro/joint_command", JointState),
    }
)


def r1pro_rerun_blueprint() -> "Any":
    """Rerun viewer layout for r1pro_full: wrist cameras + head + 3D world.

    Pass this as the ``blueprint`` kwarg when constructing a RerunBridgeModule:

        from dimos.visualization.rerun.bridge import RerunBridgeModule
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
        bridge = RerunBridgeModule(blueprint=r1pro_rerun_blueprint, pubsubs=[LCM()])

    Or run the standalone bridge (no code changes needed):

        dimos rerun-bridge

    Sensor streams appear automatically under:
        world/r1pro/left_arm/wrist_color, world/r1pro/left_arm/wrist_depth
        world/r1pro/right_arm/wrist_color, world/r1pro/right_arm/wrist_depth
        world/r1pro/chassis/head, world/r1pro/chassis/chassis_*, world/r1pro/chassis/lidar
    """
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/r1pro/left_arm/wrist_color", name="Left wrist"),
                rrb.Spatial2DView(origin="world/r1pro/right_arm/wrist_color", name="Right wrist"),
                rrb.Spatial2DView(origin="world/r1pro/chassis/head", name="Head"),
            ),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        )
    )


__all__ = [
    "r1pro_dual_mock",
    "r1pro_full",
    "r1pro_full_with_torso",
    "r1pro_planner_coordinator",
    "r1pro_planner_full",
    "r1pro_rerun_blueprint",
    "r1pro_upper_body_full",
    "r1pro_upper_body_mock",
    "r1pro_whole_body_full",
]
