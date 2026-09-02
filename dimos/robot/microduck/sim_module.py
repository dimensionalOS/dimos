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
locomotion layer: every pretrained Microduck policy (ONNX, 50 Hz) is loaded
into a ``PolicyBank`` and a ``PolicyScheduler`` decides which one runs and
with which command. The position targets drive the MJCF's servo actuators
through the engine's normal command path. No ControlCoordinator or SHM
adapter is involved - the whole robot fits in one module:

    cmd_vel (Twist) ------------> PolicyScheduler --> PolicyBank --> targets --> MuJoCo
    policy_request (JSON str) ----^      |
                                         v
                                policy_state (JSON str)

Everything else (odom, tf, IMU, head camera, raycast lidar pointcloud) is
inherited from ``MujocoSimModule``. On top of that this module renders a
third-person chase camera (``chase_image``) and drops the kickable ball the
kick policies aim at into the scene.

JSON contracts (the state's fields are documented in ``policies.py``)::

    policy_request  {"action": "start" | "stop" | "toggle", "policy": "<name>", "t": <float>}
                    ("policy" may be absent for a bare "stop": abort whatever runs)
    policy_state    PolicyScheduler.snapshot(), published whenever it changes
                    and at least every 1 / state_hz seconds
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.microduck import assets_fetch
from dimos.robot.microduck.gait import CONTROL_DT
from dimos.robot.microduck.places import BALL_BODY, BALL_RADIUS, add_ball_body
from dimos.robot.microduck.policies import (
    DEFAULT_VARIANT,
    FALL_GRAVITY_Z,
    PolicyBank,
    PolicyScheduler,
)
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

# Third-person camera following the trunk; published on ``chase_image``.
CHASE_CAMERA_NAME = "chase_camera"
_CHASE_OPTICAL_FRAME = f"{CHASE_CAMERA_NAME}_optical_frame"


def _camera_quat_wxyz(yaw: float, pitch: float = 0.0) -> tuple[float, float, float, float]:
    """MuJoCo camera quat looking along `yaw` (0 = body +x), tilted by `pitch`.

    ``pitch`` is in radians, negative = looking down; ``|pitch|`` must stay
    below 90 degrees. A MuJoCo camera looks along its -z axis with +y up.
    Columns of the rotation are the camera axes in the parent frame.
    """
    cp = math.cos(pitch)
    fwd = np.array([cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch)])
    right = np.cross(fwd, (0.0, 0.0, 1.0))  # right = forward x up ... kept right-handed
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    rot = np.stack([right, up, -fwd], axis=1)  # columns: camera x, y, z (looks along -z)
    from scipy.spatial.transform import Rotation as R

    x, y, z, w = R.from_matrix(rot).as_quat()
    return (float(w), float(x), float(y), float(z))


def _ball_spawn_xy(
    trunk_x: float, trunk_y: float, trunk_yaw: float, dx: float, dy: float
) -> tuple[float, float]:
    """World x/y of a point ``(dx, dy)`` in the trunk's yaw frame (x forward, y left)."""
    cy, sy = math.cos(trunk_yaw), math.sin(trunk_yaw)
    return (trunk_x + cy * dx - sy * dy, trunk_y + sy * dx + cy * dy)


def _state_key(snapshot: dict[str, Any]) -> dict[str, Any]:
    """A ``policy_state`` snapshot without its timestamp, for change detection."""
    return {key: value for key, value in snapshot.items() if key != "t"}


def _shape_twist(
    config: MicroduckSimModuleConfig, vx: float, vy: float, wz: float
) -> tuple[float, float, float]:
    """Map a requested twist onto the walking policy's command.

    Gains compensate the policy's velocity-tracking undershoot. The policy
    also has a yaw deadband: pure-turn commands below the top of its range
    barely rotate the robot (measured in sim: ~3-9 deg/s at 1.0 rad/s vs
    25-31 deg/s at 1.5; while stepping forward it mostly vanishes, 23-27
    deg/s at 1.0 and 31-43 deg/s at 1.5, no falls). The planner's
    rotate-in-place twists (0.2-0.3 rad/s) land around 0.7 after the gain,
    so any real turn request is bumped to ``min_effective_wz``.
    """
    vx *= config.cmd_gain_linear
    vy *= config.cmd_gain_linear
    wz *= config.cmd_gain_angular
    if 0.05 < abs(wz) < config.min_effective_wz:
        wz = math.copysign(config.min_effective_wz, wz)
    return vx, vy, wz


class MicroduckSimModuleConfig(MujocoSimModuleConfig):
    # 14 servo joints and no gripper; a smaller dof would make the parent
    # misread joint 14 onward as a gripper.
    dof: int = 14
    camera_name: str = "head_camera"
    base_frame_id: str = "trunk_base"
    imu_gyro_sensor_names: list[str] = ["imu_ang_vel"]
    imu_accel_sensor_names: list[str] = ["imu_accel"]
    # Robot model and policy set: "default" (legged) or "rollers" (wheeled
    # feet); see policies.py. Selects the MJCF unless robot_mjcf is given.
    variant: str = DEFAULT_VARIANT
    # Directory holding the policy ONNX files; None = the asset cache.
    policy_dir: str | Path | None = None
    # Gains mapping the requested twist to the policy's command, compensating
    # the policy's velocity-tracking undershoot (measured ~2.5x in sim).
    cmd_gain_linear: float = 2.4
    cmd_gain_angular: float = 2.6
    # Minimum |yaw-rate| command actually sent to the policy when turning
    # (the top of the walk policy's WZ_RANGE; see the deadband note below).
    min_effective_wz: float = 1.5
    # Zero the command when nothing published cmd_vel for this long.
    cmd_timeout: float = 1.0
    # Stand the robot back up in place when it has been on the ground this
    # long (no fall-recovery policy ships with the public Microduck repos).
    auto_stand: bool = True
    auto_stand_after: float = 2.0
    # Third-person chase camera: a TRACK-mode camera on trunk_base, offset
    # (x, y, z) m from it in the world frame, looking along +x tilted by
    # pitch. Rendered in the sim thread at chase_cam_fps.
    chase_cam: bool = True
    chase_cam_size: tuple[int, int] = (640, 360)
    chase_cam_fps: float = 12.0
    chase_cam_offset: tuple[float, float, float] = (-0.8, 0.0, 0.45)
    chase_cam_pitch_deg: float = -20.0
    chase_cam_fovy: float = 60.0
    # Shadow-casting lights re-draw the robot's 215k-vertex meshes once per
    # light per render; off, a 640x360 chase frame costs ~11 ms instead of
    # ~22 ms and the chase camera no longer halves the sim rate.
    cast_shadows: bool = False
    # Body name of the kickable ball added to the scene; "" leaves it out
    # (the kick policies then run without a ball).
    ball_body: str = BALL_BODY
    # policy_state is published on every change and at least this often.
    state_hz: float = 5.0


class MicroduckSimModule(MujocoSimModule):
    """MuJoCo sim of the Microduck biped with its RL policies in the loop."""

    config: MicroduckSimModuleConfig
    cmd_vel: In[Twist]
    policy_request: In[str]
    policy_state: Out[str]
    chase_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._bank: PolicyBank | None = None
        self._scheduler: PolicyScheduler | None = None
        self._engine_target_perm: np.ndarray | None = None
        self._phys_step = 0
        self._pose_initialized = False
        self._latest_twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_twist_ts = 0.0
        self._fallen_since: float | None = None
        self._ball_qpos_adr: int | None = None
        self._ball_qvel_adr: int | None = None
        self._last_state_key: dict[str, Any] | None = None
        self._last_state_ts = 0.0
        self._last_chase_ts = 0.0

    @rpc
    def start(self) -> None:
        variant = self.config.variant
        assets = assets_fetch.ensure_assets(variant)
        if not self.config.robot_mjcf:
            self.config.robot_mjcf = assets.robot_mjcf(variant)
        policy_dir = self.config.policy_dir or assets.policy_dir
        if self.config.chase_cam:
            width, height = self.config.chase_cam_size
            self.config.extra_cameras = {
                **self.config.extra_cameras,
                CHASE_CAMERA_NAME: (int(width), int(height), float(self.config.chase_cam_fps)),
            }

        super().start()

        engine = self._engine
        assert engine is not None
        model = engine.model
        bank = PolicyBank(policy_dir, model, variant=variant, missing=assets.missing)
        scheduler = PolicyScheduler(bank.availability, variant, spawn_ball=self._spawn_ball)

        # The parent takes joint 0 as the robot root; pin odom/IMU to the
        # trunk's free joint by name instead (the ball has a free joint too).
        self._root_base_qpos_adr = bank.root_qpos_adr
        self._imu_base_qpos_slice = slice(bank.root_qpos_adr + 3, bank.root_qpos_adr + 7)

        # write_joint_command() consumes targets in engine joint-mapping order
        # (actuator order for composed models); the policies emit policy order.
        engine_names = engine.joint_names
        policy_index = {name: i for i, name in enumerate(bank.joint_names)}
        try:
            self._engine_target_perm = np.array(
                [policy_index[name] for name in engine_names], dtype=np.int64
            )
        except KeyError as exc:
            raise RuntimeError(
                f"engine joint {exc} not in Microduck policy joints {bank.joint_names}"
            ) from exc

        self._ball_qpos_adr = self._ball_qvel_adr = None
        if self.config.ball_body:
            ball_joint = f"{self.config.ball_body}_freejoint"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ball_joint)
            if jid >= 0 and int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE):
                self._ball_qpos_adr = int(model.jnt_qposadr[jid])
                self._ball_qvel_adr = int(model.jnt_dofadr[jid])
            else:
                logger.warning(
                    "Microduck ball free joint not in the model; kicks run without a ball",
                    joint=ball_joint,
                )

        self._phys_step = 0
        self._pose_initialized = False
        self._fallen_since = None
        self._last_state_key = None
        self._last_state_ts = 0.0
        self._last_chase_ts = 0.0
        self._bank = bank
        self._scheduler = scheduler

        # Splice the policies into the engine's step hooks, keeping the
        # parent's post-step publishing (odom/imu/SHM state).
        engine.set_step_hooks(before=self._gait_pre_step, after=self._after_step)

        self._subscribe(self.cmd_vel, self._on_cmd_vel)
        self._subscribe(self.policy_request, self._on_policy_request)
        logger.info(
            "MicroduckSimModule started",
            variant=variant,
            robot_mjcf=str(self.config.robot_mjcf),
            policies=list(bank.names),
            missing=[str(name) for name in bank.missing],
            joints=len(engine_names),
            chase_cam=self.config.chase_cam,
            ball=self.config.ball_body or None,
        )

    def _subscribe(self, stream: In[Any], callback: Any) -> None:
        if stream.transport is None:
            logger.warning(
                "MicroduckSimModule input has no transport; not subscribing", stream=stream.name
            )
            return
        self.register_disposable(Disposable(stream.subscribe(callback)))

    # ------------------------------------------------------------------ inputs

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._latest_twist = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        self._latest_twist_ts = time.time()

    def _on_policy_request(self, raw: str) -> None:
        """``policy_request`` subscriber (transport thread): parse and queue only.

        Never touches MuJoCo - the scheduler applies the request on the sim
        thread at its next tick; a rejection is logged and lands in
        ``policy_state.last_error``.
        """
        scheduler = self._scheduler
        if scheduler is None:
            return
        try:
            request = json.loads(raw)
        except (TypeError, ValueError) as exc:
            logger.warning("Ignoring malformed policy_request", error=str(exc))
            return
        if not isinstance(request, dict):
            logger.warning("Ignoring policy_request that is not a JSON object", request=raw)
            return
        action = request.get("action")
        policy = request.get("policy")
        if not isinstance(action, str) or not (policy is None or isinstance(policy, str)):
            logger.warning("Ignoring policy_request with bad fields", request=request)
            return
        accepted, reason = scheduler.request(policy, action)
        if accepted:
            logger.info("Microduck policy request", policy=policy, action=action)
        else:
            logger.warning(
                "Microduck policy request rejected", policy=policy, action=action, reason=reason
            )

    # -------------------------------------------------------------- sim thread

    def _gait_pre_step(self, engine: MujocoEngine) -> None:
        """Engine sim-thread hook, before each physics step."""
        bank = self._bank
        scheduler = self._scheduler
        if bank is None or scheduler is None:
            return

        if not self._pose_initialized:
            self._stand_in_place(engine)
            self._pose_initialized = True

        if self._phys_step % POLICY_DECIMATION == 0:
            now = time.time()
            if self._check_fall(engine, now):
                # Just teleported upright: let the physics settle one tick.
                self._phys_step += 1
                if self._sim_hooks is not None:
                    self._sim_hooks.pre_step(engine)
                return

            vx, vy, wz = self._latest_twist
            if now - self._latest_twist_ts > self.config.cmd_timeout:
                vx, vy, wz = 0.0, 0.0, 0.0
            scheduler.set_twist(*_shape_twist(self.config, vx, vy, wz))

            name, command = scheduler.tick(CONTROL_DT)
            targets = bank.step(name, command, engine.data)
            engine.write_joint_command(
                JointState(position=targets[self._engine_target_perm].tolist())
            )
            self._publish_policy_state(scheduler.snapshot(), now)
        self._phys_step += 1

        # Keep the SHM bridge fed (harmless no-op without a coordinator).
        if self._sim_hooks is not None:
            self._sim_hooks.pre_step(engine)

    def _check_fall(self, engine: MujocoEngine, now: float) -> bool:
        """Debounced fall detector; True when the duck was just stood back up.

        A trunk tilted past ~55 degrees for ``auto_stand_after`` seconds is
        a fall: the scheduler is told (it aborts tricks and locks) and, with
        ``auto_stand``, the duck is teleported upright - the sim equivalent
        of a human picking it up. Skipped while the scheduler runs the
        roulade, which is upside down on purpose.
        """
        bank = self._bank
        scheduler = self._scheduler
        assert bank is not None and scheduler is not None
        if scheduler.suspend_fall_detector:
            self._fallen_since = None
            return False

        gravity_z = float(bank.projected_gravity(engine.data)[2])
        if gravity_z <= FALL_GRAVITY_Z:
            self._fallen_since = None
            scheduler.notify_fall(False)
            return False
        if self._fallen_since is None:
            self._fallen_since = now
        if now - self._fallen_since <= self.config.auto_stand_after:
            return False

        scheduler.notify_fall(True)
        if not self.config.auto_stand:
            return False
        logger.warning("Microduck fell over; standing it back up")
        # Falls usually happen tripping over an obstacle; standing back up
        # exactly in place can wedge the robot inside it. Nudge toward the
        # room origin, which is open floor in the bundled scenes.
        data = engine.data
        adr = bank.root_qpos_adr
        x = float(data.qpos[adr])
        y = float(data.qpos[adr + 1])
        dist = math.hypot(x, y)
        if dist > 1e-3:
            shift = min(0.25, dist)
            data.qpos[adr] = x - shift * x / dist
            data.qpos[adr + 1] = y - shift * y / dist
        self._stand_in_place(engine)
        self._fallen_since = None
        return True

    def _stand_in_place(self, engine: MujocoEngine) -> None:
        """Sim thread: pose the duck at its standing home pose where it is."""
        bank = self._bank
        assert bank is not None
        bank.initial_qpos(engine.data)
        bank.reset()
        mujoco.mj_forward(engine.model, engine.data)
        engine.write_joint_command(
            JointState(position=bank.default_pose[self._engine_target_perm].tolist())
        )

    def _publish_policy_state(self, snapshot: dict[str, Any], now: float) -> bool:
        """Publish ``snapshot`` if it changed (ignoring ``t``) or the periodic slot is due."""
        key = _state_key(snapshot)
        due = now - self._last_state_ts >= 1.0 / self.config.state_hz
        if key == self._last_state_key and not due:
            return False
        self._last_state_key = key
        self._last_state_ts = now
        self.policy_state.publish(json.dumps(snapshot, separators=(",", ":")))
        return True

    def _spawn_ball(self, dx: float, dy: float) -> None:
        """Drop the ball at rest ``(dx, dy)`` m from the trunk, in its yaw frame.

        Called by the scheduler from inside ``tick`` when a kick starts, so
        it runs on the sim thread (the only place MjData may be written).
        """
        engine = self._engine
        bank = self._bank
        if engine is None or bank is None:
            return
        if self._ball_qpos_adr is None or self._ball_qvel_adr is None:
            logger.warning("Microduck ball not in the scene; kicking without a ball")
            return
        data = engine.data
        adr = bank.root_qpos_adr
        x, y = _ball_spawn_xy(
            float(data.qpos[adr]), float(data.qpos[adr + 1]), bank.root_yaw(data), dx, dy
        )
        data.qpos[self._ball_qpos_adr : self._ball_qpos_adr + 7] = (
            x,
            y,
            BALL_RADIUS,
            1.0,
            0.0,
            0.0,
            0.0,
        )
        data.qvel[self._ball_qvel_adr : self._ball_qvel_adr + 6] = 0.0
        mujoco.mj_forward(engine.model, data)

    def _after_step(self, engine: MujocoEngine) -> None:
        """Engine sim-thread hook after each physics step."""
        self._publish_shm_and_lcm(engine)
        self._publish_chase(engine)

    def _publish_chase(self, engine: MujocoEngine) -> None:
        """Publish the chase camera's latest frame once, when a new one was rendered."""
        if not self.config.chase_cam:
            return
        frame = engine.read_camera(CHASE_CAMERA_NAME)
        if frame is None or frame.timestamp <= self._last_chase_ts:
            return
        self._last_chase_ts = frame.timestamp
        self.chase_image.publish(
            Image(
                data=frame.rgb,
                format=ImageFormat.RGB,
                frame_id=_CHASE_OPTICAL_FRAME,
                ts=frame.timestamp,
            )
        )

    # ------------------------------------------------------------------- model

    def _compose_model(self) -> mujoco.MjModel:
        """Compose scene + robot, adding the lidar and chase cameras and the ball.

        Simplified from the parent: no scene-package entities, and the
        physics timestep is pinned to the 5 ms the policies expect. The ball
        is added after the robot so the trunk's free joint stays joint 0
        (the engine's notion of the robot root).
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
        if self.config.chase_cam:
            # TRACK: the offset and the orientation stay fixed in the world
            # frame while the camera follows the trunk's position.
            trunk.add_camera(
                name=CHASE_CAMERA_NAME,
                mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACK,
                pos=[float(v) for v in self.config.chase_cam_offset],
                quat=list(_camera_quat_wxyz(0.0, math.radians(self.config.chase_cam_pitch_deg))),
                fovy=float(self.config.chase_cam_fovy),
            )
            # mujoco.Renderer needs the offscreen buffer at least as large as
            # the biggest camera it renders.
            width, height = self.config.chase_cam_size
            visual = spec_scene.visual.global_
            visual.offwidth = max(int(visual.offwidth), int(width))
            visual.offheight = max(int(visual.offheight), int(height))

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
        if self.config.ball_body:
            add_ball_body(spec_scene, name=self.config.ball_body)
        if not self.config.cast_shadows:
            for light in spec_scene.lights:
                light.castshadow = False
        return spec_scene.compile()


# The base policies hold a stand on a zero command, so the module needs no
# explicit idle handling; CONTROL_DT is re-exported for tests.
__all__ = ["CHASE_CAMERA_NAME", "CONTROL_DT", "MicroduckSimModule", "MicroduckSimModuleConfig"]
