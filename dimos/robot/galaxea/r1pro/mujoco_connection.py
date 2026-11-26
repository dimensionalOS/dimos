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

"""R1 Pro connection contract backed by the native MuJoCo run45 plant."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.control.connection.connection_module import ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.mujoco_plant import DT, R1ProPlant, pose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1ProMujocoConfig(R1ProConnectionConfig):
    bundle_dir: str = ""
    viser_host: str = "127.0.0.1"
    viser_port: int | None = None
    publish_rate_hz: float = 50.0
    boot_vendor_stack: bool = False
    enable_wrist_color: bool = False


class R1ProMujocoConnection(ConnectionModule):
    """Free-root physics and the standard R1 Pro control/feedback ports.

    The training plant welds fingers. Jaw opening readback is an ideal simulated
    position joint, independent of the 126 kg plant (no jaw contact dynamics).
    Odometry is simulation ground truth, not integrated wheel measurements.
    """

    config: R1ProMujocoConfig
    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]
    odom: Out[PoseStamped]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    wrist_poses: Out[PoseArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._plant_lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._viewer = None

    def connect(self) -> None:
        if self.config.publish_rate_hz != 1 / DT:
            raise ValueError("The run45 plant publishes at exactly 50 Hz")
        self.plant = R1ProPlant(self.config.bundle_dir)
        self.jaws = dict.fromkeys(R1PRO_GRIPPER_JOINTS, 0.0)
        if self.config.viser_port is not None:
            from dimos.robot.galaxea.r1pro.mujoco_viser import R1ProMujocoViewer

            self._viewer = R1ProMujocoViewer(
                self.plant, self.config.viser_host, self.config.viser_port
            )

    @rpc
    def start(self) -> None:
        super().start()
        for stream, interface in (
            (self.position_command, Interface.POSITION),
            (self.velocity_command, Interface.VELOCITY),
        ):
            self.register_disposable(
                Disposable(
                    stream.subscribe(
                        lambda msg, interface=interface: self.command(
                            joint_state_to_values(msg, (interface,))
                        )
                    )
                )
            )
        self._worker = threading.Thread(
            target=self._simulation_loop, name="r1pro-mujoco", daemon=True
        )
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._worker is not None:
            self._worker.join()
        try:
            super().stop()
        finally:
            if self._viewer is not None:
                self._viewer.stop()
                self._viewer = None

    def write(self, values: Values) -> None:
        with self._plant_lock:
            for i, joint in enumerate(R1PRO_UPPER_BODY_JOINTS):
                key = f"{joint}/position"
                if key in values:
                    self.plant.data.ctrl[i] = values[key]
            for i, joint in enumerate(R1PRO_BASE_JOINTS, 18):
                key = f"{joint}/velocity"
                if key in values:
                    self.plant.data.ctrl[i] = values[key]
            for joint in self.jaws:
                key = f"{joint}/position"
                if key in values:
                    self.jaws[joint] = values[key]

    def halt(self) -> None:
        with self._plant_lock:
            self.plant.data.ctrl[:18] = self.plant.data.qpos[7:25]
            self.plant.data.ctrl[18:] = 0.0

    def _simulation_loop(self) -> None:
        deadline = time.perf_counter()
        try:
            while not self._stop_event.is_set():
                with self._plant_lock:
                    feedback = self.feedback(time.time())
                    if self._viewer is not None:
                        self._viewer.update(self.plant)
                for stream, msg in feedback:
                    stream.publish(msg)
                deadline = max(deadline + DT, time.perf_counter())
                if self._stop_event.wait(max(0.0, deadline - time.perf_counter())):
                    break
                with self._plant_lock:
                    self.plant.step()
        except Exception:
            logger.exception("R1 Pro MuJoCo stopped after a simulation fault")
            self.halt()

    def feedback(self, ts: float) -> list[tuple[Out, Any]]:
        """Capture all streams at one physics sample (caller owns the plant lock)."""
        d = self.plant.data
        frame, world = self.config.frame_id, self.config.odom_frame_id
        root = d.body("base_link")
        rotation = root.xmat.reshape(3, 3)
        root_pose = pose(root.xpos, rotation)
        linear = rotation.T @ d.qvel[:3]
        angular = d.qvel[3:6]
        root_stamped = PoseStamped(
            ts=ts, frame_id=world, position=root_pose.position, orientation=root_pose.orientation
        )
        messages = [
            (
                self.joint_state,
                JointState(
                    ts=ts,
                    frame_id=frame,
                    name=R1PRO_UPPER_BODY_JOINTS,
                    position=d.qpos[7:25].tolist(),
                    velocity=d.qvel[6:24].tolist(),
                    effort=d.actuator_force[:18].tolist(),
                ),
            ),
            (
                self.joint_state,
                JointState(
                    ts=ts,
                    frame_id=frame,
                    name=R1PRO_BASE_JOINTS,
                    velocity=[*linear[:2], angular[2]],
                ),
            ),
            (
                self.joint_state,
                JointState(
                    ts=ts, frame_id=frame, name=list(self.jaws), position=list(self.jaws.values())
                ),
            ),
            (
                self.odometry,
                Odometry(
                    ts=ts,
                    frame_id=world,
                    child_frame_id=frame,
                    pose=root_pose,
                    twist=Twist(Vector3(*linear), Vector3(*angular)),
                ),
            ),
            (self.wrist_poses, PoseArray(Header(ts, world), self.plant.wrists())),
        ]
        for stream, name in (
            (self.imu_chassis, "imu_chassis_link"),
            (self.imu_torso, "imu_torso_link"),
        ):
            site = d.site(name)
            messages.append(
                (
                    stream,
                    Imu(
                        ts=ts,
                        frame_id=name,
                        orientation=pose(
                            site.xpos, np.asarray(site.xmat).reshape(3, 3)
                        ).orientation,
                        angular_velocity=Vector3(*d.sensor(f"{name}_gyro").data),
                        linear_acceleration=Vector3(*d.sensor(f"{name}_accel").data),
                    ),
                )
            )
        if self.config.publish_odom_tf:
            messages.extend(
                [
                    (self.odom, root_stamped),
                    (self.tf, TFMessage(Transform.from_pose(frame, root_stamped))),
                ]
            )
        return messages
