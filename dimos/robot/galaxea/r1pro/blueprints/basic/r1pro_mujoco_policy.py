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

"""WebXR dual-wrist R1 Pro policy in a hardware-free MuJoCo assembly."""

import os
from pathlib import Path

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.connection import R1PRO_GRIPPER_JOINTS
from dimos.robot.galaxea.r1pro.mujoco_connection import R1ProMujocoConfig, R1ProMujocoConnection
from dimos.robot.galaxea.r1pro.mujoco_plant import CHECKPOINT, DT
from dimos.robot.galaxea.r1pro.policy_task import TASK_TYPE, R1ProPolicyTask
from dimos.robot.galaxea.r1pro.webxr_policy_input import R1ProPolicyWebXR


class R1ProPolicyCoordinator(ControlCoordinator):
    r1pro_position_command: Out[JointState]
    r1pro_velocity_command: Out[JointState]
    r1pro_joint_state: In[JointState]
    r1pro_imu: In[Imu]
    wrist_poses: In[PoseArray]
    wrist_targets: In[PoseArray]
    odometry: In[Odometry]
    controller_pair: In[PoseArray]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]

    def _create_task_from_config(self, cfg):
        if cfg.type != TASK_TYPE:
            return super()._create_task_from_config(cfg)
        control_task_registry.register_bindings(
            TASK_TYPE,
            consumes={
                "r1pro_joint_state": ("on_joints", "direct"),
                "odometry": ("on_odometry", "direct"),
                "wrist_poses": ("on_wrists", "direct"),
                "wrist_targets": ("on_world_target", "direct"),
                "controller_pair": ("on_controllers", "direct"),
            },
        )
        if self.config.tick_rate != 1 / DT:
            raise ValueError("R1 Pro policy requires a 50 Hz coordinator")
        return R1ProPolicyTask(cfg.name, cfg.params["model_path"], cfg.priority)


def _simulation():
    bundle = os.environ.get("R1PRO_SIM_BUNDLE", "")
    checkpoint = os.environ.get("R1PRO_POLICY_ONNX", str(Path(bundle) / "models" / CHECKPOINT))
    declaration = R1ProMujocoConfig(bundle_dir=bundle, viser_port=8080)
    return autoconnect(
        R1ProMujocoConnection.blueprint(**declaration.model_dump()),
        R1ProPolicyCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tick_rate=1 / DT,
            connections=[
                ConnectionConfig(
                    **declaration.model_dump(include=set(ConnectionConfig.model_fields))
                )
            ],
            tasks=[
                TaskConfig(
                    name="r1pro_policy",
                    type=TASK_TYPE,
                    priority=40,
                    params={"model_path": checkpoint},
                ),
                *(
                    TaskConfig(
                        name=f"{hand}_gripper",
                        type="gripper",
                        joint_names=[jaw],
                        stream_bind={"gripper_command": f"{hand}_gripper_command"},
                    )
                    for hand, jaw in zip(("left", "right"), R1PRO_GRIPPER_JOINTS, strict=True)
                ),
            ],
        ),
        R1ProPolicyWebXR.blueprint(input_timeout_s=0.12),
    ).remappings(
        [
            (R1ProMujocoConnection, "position_command", "r1pro_position_command"),
            (R1ProMujocoConnection, "velocity_command", "r1pro_velocity_command"),
            (R1ProPolicyCoordinator, "r1pro_joint_state", "joint_state"),
            (R1ProPolicyCoordinator, "r1pro_imu", "imu_chassis"),
        ]
    )


r1pro_mujoco_policy = _simulation().global_config(
    viewer="none",
    build_native=False,
    transport="zenoh",
    serve_coordinator_rpc=False,
    zenoh_mode="peer",
    zenoh_connect="",
    zenoh_interface="lo",
    zenoh_multicast=True,
    zenoh_domain="r1pro-layer2",
    zenoh_scout_addr="239.77.38.96:17696",
    robot_ip=None,
    robot_ips=None,
    local_relay=False,
    relay_url=None,
)
