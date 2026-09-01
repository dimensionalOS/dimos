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

"""Microduck MuJoCo simulation module.

``MicroduckSimModule`` extends ``MujocoSimModule`` with an in-process RL
locomotion layer: a ``cmd_vel`` Twist input is fed to the pretrained
Microduck walking policy (ONNX, 50 Hz), whose position targets drive the
MJCF's servo actuators through the engine's normal command path. No
ControlCoordinator or SHM adapter is involved - the whole robot fits in
one module:

    cmd_vel (Twist) --> MicroduckGaitPolicy --> position targets --> MuJoCo

Everything else (odom, tf, IMU, head camera, raycast lidar pointcloud)
is inherited from ``MujocoSimModule``.
"""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.microduck import assets_fetch
from dimos.robot.microduck.gait import CONTROL_DT, MicroduckGaitPolicy
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import (
    MujocoSimModule,
    MujocoSimModuleConfig,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# One physics step is 5 ms (forced below); the policy runs every 4th step.
PHYSICS_TIMESTEP = 0.005
POLICY_DECIMATION = 4

# Trunk-mounted raycast-lidar cameras, mirroring the G1 sim convention
# (front/left/right, wide fovy). Positions are in the trunk_base frame of a
# ~25 cm tall robot whose trunk origin sits ~12 cm above ground.
LIDAR_CAMERA_SPECS: tuple[tuple[str, float], ...] = (
    ("lidar_front_camera", 0.0),
    ("lidar_left_camera", math.radians(120.0)),
    ("lidar_right_camera", math.radians(-120.0)),
)
_LIDAR_CAM_POS = (0.0, 0.0, 0.08)
_LIDAR_CAM_FOVY = 140.0


def _camera_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """MuJoCo camera quat looking horizontally along `yaw` (0 = body +x).

    A MuJoCo camera looks along its -z axis with +y up. Columns of the
    rotation are the camera axes in the parent frame.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    # forward (view direction) in body frame
    fx, fy = cy, sy
    x_cam = (fy, -fx, 0.0)  # right = forward x up(0,0,1) ... kept right-handed
    y_cam = (0.0, 0.0, 1.0)
    z_cam = (-fx, -fy, 0.0)  # camera looks along -z_cam == (fx, fy, 0)
    rot = np.array([x_cam, y_cam, z_cam]).T  # columns are camera axes
    from scipy.spatial.transform import Rotation as R

    x, y, z, w = R.from_matrix(rot).as_quat()
    return (float(w), float(x), float(y), float(z))


class MicroduckSimModuleConfig(MujocoSimModuleConfig):
    # 14 servo joints and no gripper; a smaller dof would make the parent
    # misread joint 14 onward as a gripper.
    dof: int = 14
    camera_name: str = "head_camera"
    base_frame_id: str = "trunk_base"
    imu_gyro_sensor_names: list[str] = ["imu_ang_vel"]
    imu_accel_sensor_names: list[str] = ["imu_accel"]
    walking_policy_path: str | Path | None = None
    # Gains mapping the requested twist to the policy's command, compensating
    # the policy's velocity-tracking undershoot (measured ~2.5x in sim).
    cmd_gain_linear: float = 2.4
    cmd_gain_angular: float = 2.6
    # Minimum |yaw-rate| command actually sent to the policy when turning.
    min_effective_wz: float = 1.0
    # Zero the command when nothing published cmd_vel for this long.
    cmd_timeout: float = 1.0
    # Stand the robot back up in place when it has been on the ground this
    # long (no recovery policy ships with the public Microduck repos).
    auto_stand: bool = True
    auto_stand_after: float = 2.0


class MicroduckSimModule(MujocoSimModule):
    """MuJoCo sim of the Microduck biped with its walking policy in the loop."""

    config: MicroduckSimModuleConfig
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gait: MicroduckGaitPolicy | None = None
        self._engine_target_perm: np.ndarray | None = None
        self._phys_step = 0
        self._pose_initialized = False
        self._latest_twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_twist_ts = 0.0
        self._fallen_since: float | None = None

    @rpc
    def start(self) -> None:
        assets_fetch.ensure_assets()
        if not self.config.robot_mjcf:
            self.config.robot_mjcf = assets_fetch.robot_mjcf_path()
        policy_path = self.config.walking_policy_path or assets_fetch.walking_policy_path()

        super().start()

        engine = self._engine
        assert engine is not None
        self._gait = MicroduckGaitPolicy(policy_path, engine.model)

        # write_joint_command() consumes targets in engine joint-mapping order
        # (actuator order for composed models); the policy emits policy order.
        engine_names = engine.joint_names
        policy_index = {name: i for i, name in enumerate(self._gait.joint_names)}
        try:
            self._engine_target_perm = np.array(
                [policy_index[name] for name in engine_names], dtype=np.int64
            )
        except KeyError as exc:
            raise RuntimeError(
                f"engine joint {exc} not in Microduck policy joints {self._gait.joint_names}"
            ) from exc

        self._phys_step = 0
        self._pose_initialized = False

        # Splice the gait into the engine's step hooks, keeping the parent's
        # post-step publishing (odom/imu/SHM state).
        engine.set_step_hooks(before=self._gait_pre_step, after=self._publish_shm_and_lcm)

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        logger.info(
            "MicroduckSimModule started",
            policy=str(policy_path),
            joints=len(engine_names),
        )

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._latest_twist = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        self._latest_twist_ts = time.time()

    def _gait_pre_step(self, engine: MujocoEngine) -> None:
        """Engine sim-thread hook, before each physics step."""
        gait = self._gait
        if gait is None:
            return

        if not self._pose_initialized:
            gait.initial_qpos(engine.data)
            mujoco.mj_forward(engine.model, engine.data)
            engine.write_joint_command(
                JointState(position=gait.default_pose[self._engine_target_perm].tolist())
            )
            self._pose_initialized = True

        # Fall handling: only the plain walking policy ships, with no fall
        # recovery, and the walk-optimized model has no trunk collisions (it
        # sinks into the floor when down). If the trunk stays tilted past
        # ~55 degrees, stand the duck back up in place - the sim equivalent
        # of a human picking it up.
        if self.config.auto_stand and self._phys_step % POLICY_DECIMATION == 0:
            gravity_z = float(gait.projected_gravity(engine.data)[2])
            now = time.time()
            if gravity_z > -0.55:
                if self._fallen_since is None:
                    self._fallen_since = now
                elif now - self._fallen_since > self.config.auto_stand_after:
                    logger.warning("Microduck fell over; standing it back up")
                    # Falls usually happen tripping over an obstacle; standing
                    # back up exactly in place can wedge the robot inside it.
                    # Nudge toward the room origin, which is open floor in the
                    # bundled scene.
                    adr = gait.root_qpos_adr
                    x = float(engine.data.qpos[adr])
                    y = float(engine.data.qpos[adr + 1])
                    dist = math.hypot(x, y)
                    if dist > 1e-3:
                        shift = min(0.25, dist)
                        engine.data.qpos[adr] = x - shift * x / dist
                        engine.data.qpos[adr + 1] = y - shift * y / dist
                    gait.initial_qpos(engine.data)
                    gait.reset()
                    mujoco.mj_forward(engine.model, engine.data)
                    engine.write_joint_command(
                        JointState(position=gait.default_pose[self._engine_target_perm].tolist())
                    )
                    self._fallen_since = None
                    self._phys_step += 1
                    return
            else:
                self._fallen_since = None

        if self._phys_step % POLICY_DECIMATION == 0:
            vx, vy, wz = self._latest_twist
            if time.time() - self._latest_twist_ts > self.config.cmd_timeout:
                vx, vy, wz = 0.0, 0.0, 0.0
            vx *= self.config.cmd_gain_linear
            vy *= self.config.cmd_gain_linear
            wz *= self.config.cmd_gain_angular
            # The walking policy has a yaw deadband: pure-turn commands below
            # ~1.0 rad/s barely rotate the robot (measured ~1 deg/s at 0.73
            # vs 22 deg/s at 1.0). Any real turn request is bumped to the
            # effective minimum so rotate-in-place actually rotates.
            if 0.05 < abs(wz) < self.config.min_effective_wz:
                wz = math.copysign(self.config.min_effective_wz, wz)
            gait.set_twist(vx, vy, wz)
            targets = gait.step(engine.data)
            engine.write_joint_command(
                JointState(position=targets[self._engine_target_perm].tolist())
            )
        self._phys_step += 1

        # Keep the SHM bridge fed (harmless no-op without a coordinator).
        if self._sim_hooks is not None:
            self._sim_hooks.pre_step(engine)

    def _compose_model(self) -> mujoco.MjModel:
        """Compose scene + robot, adding lidar cameras to the trunk.

        Simplified from the parent: no scene-package entities, and the
        physics timestep is pinned to the 5 ms the walking policy expects.
        """
        if self.config.robot_mjcf is None:
            raise RuntimeError("MicroduckSimModule: robot_mjcf is required")

        if self.config.scene_xml is not None:
            spec_scene = mujoco.MjSpec.from_file(str(self.config.scene_xml))
        else:
            spec_scene = mujoco.MjSpec()

        spec_robot = mujoco.MjSpec.from_file(str(self.config.robot_mjcf))
        if self.config.robot_meshdir is not None:
            spec_robot.meshdir = str(self.config.robot_meshdir)

        trunk = spec_robot.body("trunk_base")
        if trunk is None:
            raise RuntimeError("Microduck robot MJCF has no 'trunk_base' body")
        for name, yaw in LIDAR_CAMERA_SPECS:
            trunk.add_camera(
                name=name,
                pos=list(_LIDAR_CAM_POS),
                quat=list(_camera_quat_wxyz(yaw)),
                fovy=_LIDAR_CAM_FOVY,
            )

        spec_scene.option.timestep = PHYSICS_TIMESTEP
        spec_robot.option.timestep = PHYSICS_TIMESTEP

        spawn_xy = self.config.spawn_xy or (0.0, 0.0)
        spawn_z = self.config.spawn_z if self.config.spawn_z is not None else 0.0
        frame_kwargs: dict[str, Any] = {
            "pos": [float(spawn_xy[0]), float(spawn_xy[1]), float(spawn_z)],
        }
        if self.config.spawn_yaw is not None:
            yaw = float(self.config.spawn_yaw)
            frame_kwargs["quat"] = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
        frame = spec_scene.worldbody.add_frame(**frame_kwargs)
        spec_scene.attach(spec_robot, prefix="", frame=frame)
        return spec_scene.compile()


# The policy holds a stand on a zero command, so the module needs no
# explicit idle handling; CONTROL_DT is re-exported for tests.
__all__ = ["CONTROL_DT", "MicroduckSimModule", "MicroduckSimModuleConfig"]
