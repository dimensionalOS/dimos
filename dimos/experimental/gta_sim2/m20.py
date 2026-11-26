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

"""M20 on the current USD/HardwareModule Sim2 stack, through measured sensors.

The actor consumes synchronous IMU/encoder acquisitions. Joint targets use the
same clock, IDs, atomic five-interface action and expiry contract as the G1.
"""

import math
from pathlib import Path
import threading
import time
from typing import Any

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand, JointCommand
from dimos_lcm.std_msgs import Header, Time
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.hardware_module import Mounts
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.experimental.gta_sim2.m20_actor import JOINTS, KD, KP, M20Actor, Profile, home_pose
from dimos.experimental.gta_sim2.prepare_m20 import BASE
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.sim2.description import SceneConfig, joint_paths
from dimos.sim2.hardware import SimHardwareModule
from dimos.sim2.module import SimModule
from dimos.sim2.physics.drives import usd_drives
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule


class M20Hardware(SimHardwareModule):
    mounts = Mounts(BASE + "/imu_mount", BASE + "/camera_mount")


class M20PolicyConfig(SceneConfig):
    model_path: Path
    profile: Profile = "sdk57"
    period_ns: int = 20_000_000
    command_ttl_ns: int = 100_000_000


class M20PolicyModule(Module):
    config: M20PolicyConfig
    _simulation: SimModule
    imu: In[Imu]
    joint_state: In[JointState]
    cmd_vel: In[Twist]
    actions: Out[ActionCommand]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._actor: M20Actor | None = None
        self._velocity = np.zeros(3)
        self._until = 0.0
        self._identity = -1
        self._joints: dict[int, JointState] = {}
        self._imus: dict[int, Imu] = {}
        self._last_action: ActionCommand | None = None
        self._clock_id: str | None = None
        self._inferences = self._missed = 0
        self._standing = True
        self._park_wheels: NDArray[np.float64] | None = None

    @rpc
    def build(self) -> None:
        super().build()
        if (
            self.config.period_ns != 20_000_000
            or self.config.command_ttl_ns < self.config.period_ns
        ):
            raise ValueError("M20 policy requires 50 Hz and a TTL covering one period")
        scene = self.config.scene
        if not isinstance(scene, UsdModel):
            raise TypeError("M20 requires its composed USD scene")
        self._clock_id = self._simulation.simulation_config()["sim_clock_id"]
        paths = joint_paths(scene)
        self._ids = tuple(paths[n] for n in JOINTS)
        joints = {j.name: j for j in scene.joints}
        self._lower = np.array([joints[n].lower for n in JOINTS[:12]], dtype=float)
        self._upper = np.array([joints[n].upper for n in JOINTS[:12]], dtype=float)
        self._actor = M20Actor(self.config.model_path, self.config.profile)

    @rpc
    def move(self, twist: Twist, duration: float = 0.35) -> None:
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not np.isfinite((*values, duration)).all() or duration <= 0:
            raise ValueError("Finite velocity and positive duration required")
        with self._lock:
            self._velocity = np.clip(values, [-1.5, -0.5, -0.7], [1.5, 0.5, 0.7])
            self._until = time.monotonic() + duration

    async def handle_cmd_vel(self, sample: Twist) -> None:
        self.move(sample)

    async def handle_imu(self, sample: Imu) -> None:
        self._receive(sample)

    async def handle_joint_state(self, sample: JointState) -> None:
        self._receive(sample)

    def _boundary(self, stamp: float) -> int | None:
        if not math.isfinite(stamp) or stamp < 0:
            return None
        boundary = round(stamp * 1e9 / self.config.period_ns) * self.config.period_ns
        return boundary if abs(stamp * 1e9 - boundary) <= 2 + math.ulp(stamp) * 1e9 else None

    def _receive(self, sample: JointState | Imu) -> None:
        identity = self._boundary(sample.ts)
        if identity is None:
            return
        with self._lock:
            if identity <= self._identity:
                return
            if isinstance(sample, JointState):
                self._joints[identity] = sample
            else:
                self._imus[identity] = sample
            if identity in self._joints and identity in self._imus:
                self.actions.publish(
                    self.compute(self._joints.pop(identity), self._imus.pop(identity))
                )
            self._joints = {k: v for k, v in self._joints.items() if k >= identity}
            self._imus = {k: v for k, v in self._imus.items() if k >= identity}

    def compute(self, joints: JointState, imu: Imu) -> ActionCommand:
        with self._lock:
            stamp = self._boundary(joints.ts)
            if stamp is None or stamp != self._boundary(imu.ts) or stamp < self._identity:
                raise ValueError("M20 requires matching, monotonic IMU/encoder acquisitions")
            if self._actor is None or self._clock_id is None:
                raise ValueError("Build the policy before control")
            if stamp == self._identity:
                assert self._last_action is not None
                return self._last_action
            if imu.orientation_covariance[0] < 0:
                raise ValueError("M20 requires an attitude estimate or explicit ideal orientation")
            order = [joints.name.index(n) for n in self._ids]
            orientation, gyro = imu.orientation, imu.angular_velocity
            gravity = (
                Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
                .inv()
                .apply([0, 0, -1])
            )
            velocity = self._velocity if time.monotonic() < self._until else np.zeros(3)
            standing = bool(np.max(np.abs(velocity)) < 1e-6)
            if standing:
                if not self._standing:
                    self._actor.reset()
                if self._park_wheels is None or not self._standing:
                    self._park_wheels = np.asarray(joints.position)[order][12:].copy()
                q, dq = home_pose(self.config.profile), np.zeros(16)
                q[12:] = self._park_wheels
            else:
                q, dq = self._actor.act(
                    np.asarray(joints.position)[order],
                    np.asarray(joints.velocity)[order],
                    np.array([gyro.x, gyro.y, gyro.z]),
                    gravity,
                    velocity,
                )
                self._inferences += 1
            self._standing = standing
            kp = np.r_[KP[:12], np.full(4, 20.0)] if standing else KP
            kd = np.r_[KD[:12], np.full(4, 2.0)] if standing else KD
            bounded_q = q.copy()
            bounded_q[:12] = np.clip(q[:12], self._lower, self._upper)
            # Preserve the policy's PD torque when Sim2 requires an in-range
            # position target. Actuator force limits still bound the total torque.
            feedforward = kp * (q - bounded_q)
            packet = ActionCommand(
                header=SampleHeader(
                    clock_id=self._clock_id,
                    epoch=0,
                    time_ns=stamp,
                    source_id=self.config.instance_name or "m20/sdk",
                    sequence=stamp // self.config.period_ns,
                ),
                based_on_time_ns=stamp,
                apply_at_ns=stamp,
                valid_until_ns=stamp + self.config.command_ttl_ns,
                joint_commands_length=5,
                joint_commands=[
                    JointCommand(
                        header=Header(stamp=Time(sec=stamp // 10**9, nsec=stamp % 10**9)),
                        joint_names_length=16,
                        joint_names=list(self._ids),
                        interface_name=interface,
                        values_length=16,
                        values=list(values),
                    )
                    for interface, values in (
                        ("position", bounded_q),
                        ("velocity", dq),
                        ("kp", kp),
                        ("kd", kd),
                        ("effort", feedforward),
                    )
                ],
            )
            if self._identity >= 0:
                self._missed += max(0, (stamp - self._identity) // self.config.period_ns - 1)
            self._identity, self._last_action = stamp, packet
            return packet

    @rpc
    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "inferences": self._inferences,
                "missed_samples": self._missed,
                "active": self._actor is not None,
                "contract": self.config.profile,
                "mode": "stand" if self._standing else "policy",
                "stairs_validated": False,
            }

    @rpc
    def stop(self) -> None:
        super().stop()
        with self._lock:
            self._actor = None
            self._velocity.fill(0)


def m20_blueprint(scene_path: Path, policy_path: Path, profile: Profile = "sdk57") -> MetaBlueprint:
    scene = UsdModel.from_file(scene_path)
    paths = joint_paths(scene)
    ids = tuple(paths[n] for n in JOINTS)
    drives = usd_drives(scene.usd, scene_path.resolve().parent)
    for name, kp, kd, home in zip(ids, KP, KD, home_pose(profile), strict=True):
        if name not in drives or not np.allclose(drives[name], (kp, kd, home), atol=1e-7):
            raise ValueError(f"Scene drive {name} does not match M20 profile {profile}")
    camera = SimCameraModule.attach(
        M20Hardware.mounts.camera_mount, instance_name="camera", sensor=CameraConfig("camera")
    )
    return autoconnect(
        M20Hardware.blueprint(instance_name="scene", usd=scene),
        SimImuModule.attach(
            M20Hardware.mounts.imu_mount,
            instance_name="imu",
            sensor=ImuConfig("imu", hz=50, orientation="ideal"),
        ).remappings([(SimImuModule, "imu", "imu")]),
        camera,
        CameraRenderModule.blueprint(
            renderer=MujocoRenderer, cameras=tuple(atom.name for atom in camera.active_blueprints)
        ),
        SimModule.blueprint(backend=MujocoBackend, observation_hz=200, joint_ids=ids),
        SimJointEncoderModule.blueprint(sensor=EncoderConfig(ids, hz=50)),
        M20PolicyModule.blueprint(scene=scene, model_path=policy_path, profile=profile),
    ).global_config(viewer="none", transport="zenoh", n_workers=6)
