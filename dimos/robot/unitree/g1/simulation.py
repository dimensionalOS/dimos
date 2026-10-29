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

"""The simulated G1 for sim2: MJCF body, actuator bindings, control IMU and ports.

The joint names are the coordinator's (``make_humanoid_joints("g1")``), so the
world, the GR00T task and the transport adapter agree on order. ``G1World``
publishes the same messages ``G1WholeBodyConnection`` does on hardware, so the
coordinator drives the simulated G1 through the same adapter. Sensors are not
declared here: each hardware driver in a blueprint (Mid360, cameras) declares
its own simulated device, attached at its ``frame_id``, which the MJCF carries
as a named site or camera (``mid360_link``, ``front_camera``).
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from dimos.control.components import make_humanoid_joints
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.spec import Joint, RobotConfig, RobotInstance
from dimos.sim2.world.scene import scene_robot
from dimos.utils.data import LfsPath

G1_SIM = RobotConfig(
    model=Path(__file__).parent / "assets" / "g1_29dof.xml",
    meshdir=LfsPath("g1_urdf/meshes"),
    root_body="pelvis",
    floating=True,
    spawn_height=0.793,
    joints=tuple(
        Joint(
            name=name,
            model_name=name.split("/", 1)[1] + "_joint",
            actuator=name.split("/", 1)[1] + "_joint",
        )
        for name in make_humanoid_joints("g1")
    ),
    # The control IMU travels with motor feedback, as it does over DDS on hardware.
    sensors=(Imu("imu", site="control_imu"),),
)


def g1_groot_instance(
    scene: Path, *, spawn: str = "default", frame_id: str = "g1_pelvis"
) -> RobotInstance:
    """Place the G1 in a scene, held in the GR00T home pose until the policy commands it.

    Args:
        scene: The scene MJCF; its optional sidecar may author named spawns.
        spawn: Which authored spawn to stand on; the scene origin when none exists.
        frame_id: Frame stamped on joint and IMU feedback and the TF child of ``world``.
    """
    instance = scene_robot(scene, G1_SIM, spawn, default=(0.0, 0.0, 0.0))
    names = [joint.name for joint in G1_SIM.joints]
    return replace(
        instance,
        initial_positions=dict(zip(names, G1_GROOT_HOME, strict=True)),
        hold_gains={
            name: (kp, kd) for name, kp, kd in zip(names, G1_GROOT_KP, G1_GROOT_KD, strict=True)
        },
        control_imu="imu",
        wait_for_first_command=True,
        frame_id=frame_id,
    )


class G1World(SimulationModule):
    """A world holding the robot ``g1``, wired like ``G1WholeBodyConnection``.

    ``g1_motor_command`` takes the coordinator's PD targets; ``g1_motor_states``
    and ``g1_imu`` are what the GR00T task observes; ``g1_odom`` and ``g1_tf``
    are ground truth the hardware connection cannot publish.
    """

    g1_motor_command: In[MotorCommandArray]
    g1_motor_states: Out[JointState]
    g1_imu: Out[ImuMessage]
    g1_odom: Out[PoseStamped]
    g1_tf: Out[TFMessage]
