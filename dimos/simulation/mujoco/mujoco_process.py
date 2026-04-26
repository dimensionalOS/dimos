#!/usr/bin/env python3

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

import base64
import json
import pickle
import signal
import sys
import time
from typing import Any

import mujoco
from mujoco import viewer
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]

from dimos.core.global_config import GlobalConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.mujoco.constants import (
    DEPTH_CAMERA_FOV,
    LIDAR_FPS,
    LIDAR_RESOLUTION,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.model import get_assets, load_model, load_scene_xml
from dimos.simulation.mujoco.person_on_track import PersonPositionController
from dimos.simulation.mujoco.shared_memory import ShmReader
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MockController:
    """Controller that reads commands from shared memory."""

    def __init__(self, shm_interface: ShmReader) -> None:
        self.shm = shm_interface
        self._command = np.zeros(3, dtype=np.float32)

    def get_command(self) -> NDArray[Any]:
        """Get the current movement command."""
        cmd_data = self.shm.read_command()
        if cmd_data is not None:
            linear, angular = cmd_data
            # MuJoCo expects [forward, lateral, rotational]
            self._command[0] = linear[0]  # forward/backward
            self._command[1] = linear[1]  # left/right
            self._command[2] = angular[2]  # rotation
        result: NDArray[Any] = self._command.copy()
        return result

    def stop(self) -> None:
        """Stop method to satisfy InputController protocol."""
        pass


def _find_sensor_slice(model: mujoco.MjModel, *names: str, dim: int = 3) -> slice | None:
    """Return the first matching sensor slice across ``names``, or None."""
    for n in names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
        if sid >= 0:
            adr = int(model.sensor_adr[sid])
            return slice(adr, adr + dim)
    return None


def _load_g1_gear_wbc_lowlevel() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load GR00T's ``g1_gear_wbc.xml`` for low-level passthrough mode.

    This MJCF is the one the GR00T balance/walk ONNX policies were
    trained against.  Critically, it uses ``<motor>`` (torque) actuators
    — NOT ``<position>`` — so the subprocess does the PD itself with the
    per-joint kp/kd coming in over shm, matching the gains in
    ``g1_gear_wbc.yaml`` that shaped the policy during training.

    Position-actuator alternatives (dimos's bundled ``unitree_g1.xml``
    at kp=75 or menagerie's ``unitree_g1/scene.xml`` at kp=500) don't
    match the trained gains (hips=150, knees=200, ankles=40, waist=250)
    and produce violent instability when driven by the policy.

    The XML references meshes by bare filename (``meshdir`` stripped
    when bundled); ``get_assets()`` already injects menagerie's G1 mesh
    bytes under those names.
    """
    xml_path = get_data("mujoco_sim") / "g1_gear_wbc.xml"
    with open(xml_path) as f:
        xml_str = f.read()
    model = mujoco.MjModel.from_xml_string(xml_str, assets=get_assets())
    data = mujoco.MjData(model)
    return model, data


def _run_simulation(config: GlobalConfig, shm: ShmReader, control_mode: str = "high_level") -> None:
    robot_name = config.robot_model or "unitree_go1"
    if robot_name == "unitree_go2":
        robot_name = "unitree_go1"

    controller = MockController(shm)
    skip_controller = control_mode == "low_level"
    if skip_controller and robot_name == "unitree_g1":
        # Low-level G1: use GR00T's training MJCF (torque actuators) and
        # run PD in this subprocess.  The dimos-bundled and menagerie
        # MJCFs are position-actuator variants whose baked kp does NOT
        # match the policy's trained gains.
        model, data = _load_g1_gear_wbc_lowlevel()
    else:
        model, data = load_model(
            controller,
            robot=robot_name,
            scene_xml=load_scene_xml(config),
            skip_controller=skip_controller,
        )

    if model is None or data is None:
        raise ValueError("Failed to load MuJoCo model: model or data is None")

    match robot_name:
        case "unitree_go1":
            z = 0.3
        case "unitree_g1":
            # Match g1_gear_wbc.xml's pelvis pos.  Was 0.8 — overrode the
            # MJCF and dropped the robot 7 mm at the first mj_step.
            z = 0.793
        case _:
            z = 0

    pos = config.mujoco_start_pos_float

    data.qpos[0:3] = [pos[0], pos[1], z]

    mujoco.mj_forward(model, data)

    # Camera / person machinery only exists in the high-level scenes
    # (scene_office1 etc.).  Low-level mode uses a minimal robot scene
    # (menagerie), so skip those lookups entirely.
    camera_id = lidar_camera_id = lidar_left_camera_id = lidar_right_camera_id = -1
    person_position_controller: PersonPositionController | None = None
    if not skip_controller:
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        lidar_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_front_camera")
        lidar_left_camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_left_camera"
        )
        lidar_right_camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_right_camera"
        )
        person_position_controller = PersonPositionController(model)

    # Low-level passthrough precomputes: actuator→qpos/qvel maps and IMU
    # sensor slices so the per-tick hot path is just array copies.
    imu_gyro_slice = imu_accel_slice = None
    act_qposadr = act_dofadr = None
    num_motors = 0
    if skip_controller:
        # Menagerie uses "imu-pelvis-*" with hyphens; bundled MJX variant
        # uses "gyro_pelvis"/"accelerometer_pelvis" with underscores.
        # Try both so the low-level path works against either MJCF.
        imu_gyro_slice = _find_sensor_slice(
            model, "imu-pelvis-angular-velocity", "gyro_pelvis", dim=3
        )
        imu_accel_slice = _find_sensor_slice(
            model, "imu-pelvis-linear-acceleration", "accelerometer_pelvis", dim=3
        )
        num_motors = int(model.nu)
        act_qposadr = np.array(
            [int(model.jnt_qposadr[int(model.actuator_trnid[i, 0])]) for i in range(num_motors)],
            dtype=np.intp,
        )
        act_dofadr = np.array(
            [int(model.jnt_dofadr[int(model.actuator_trnid[i, 0])]) for i in range(num_motors)],
            dtype=np.intp,
        )

    shm.signal_ready()

    with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as m_viewer:
        camera_size = (VIDEO_WIDTH, VIDEO_HEIGHT)

        # Create renderers
        rgb_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_renderer.enable_depth_rendering()

        depth_left_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_left_renderer.enable_depth_rendering()

        depth_right_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_right_renderer.enable_depth_rendering()

        scene_option = mujoco.MjvOption()

        # Timing control
        last_video_time = 0.0
        last_lidar_time = 0.0
        video_interval = 1.0 / VIDEO_FPS
        lidar_interval = 1.0 / LIDAR_FPS

        m_viewer.cam.lookat = config.mujoco_camera_position_float[0:3]
        m_viewer.cam.distance = config.mujoco_camera_position_float[3]
        m_viewer.cam.azimuth = config.mujoco_camera_position_float[4]
        m_viewer.cam.elevation = config.mujoco_camera_position_float[5]

        # Low-level startup: the subprocess comes up ~2 s before the
        # coordinator starts ticking.  Without this flag, the robot would
        # free-fall into a sprawl during those 2 s and the first PD tick
        # would yank it at kp=150-200 from the fallen heap back toward
        # the default bent-knee pose — a startup seizure.
        controller_ready = False

        while m_viewer.is_running() and not shm.should_stop():
            step_start = time.time()

            # Low-level passthrough: read per-joint (q_target, kp, kd) from
            # shm, compute PD torque, write to data.ctrl.  The MJCF has
            # torque-mode <motor> actuators, so this subprocess plays the
            # role that onboard motor drivers play on real hardware.
            # Using shm-sourced kp/kd (not MJCF-baked gains) is the whole
            # point: the GR00T policy was trained against a specific
            # per-joint PD, and any deviation destabilises it.
            if skip_controller:
                assert act_qposadr is not None and act_dofadr is not None
                cmd = shm.read_joint_cmd(num_motors)
                if cmd is not None:
                    controller_ready = True
                    q = data.qpos[act_qposadr].astype(np.float32)
                    dq = data.qvel[act_dofadr].astype(np.float32)
                    q_tgt = cmd[:, 0]
                    kp = cmd[:, 1]
                    kd = cmd[:, 2]
                    data.ctrl[:num_motors] = kp * (q_tgt - q) - kd * dq

            # Step simulation.  In low-level mode we step once per outer
            # iteration so sim-time advances in lock-step with wall-time
            # — the coordinator is writing new PD targets at ~500 Hz and
            # we need fresh (q, dq) → PD each step, not a stale PD held
            # across 7 substeps (which made physics run 7× real-time and
            # PD react to 14 ms-old state, the seizure we just debugged).
            # High-level mode keeps the substeps-per-frame speedup because
            # its ONNX controller lives inside mj_step via mjcb_control.
            if skip_controller and not controller_ready:
                # mj_forward runs kinematics but not dynamics — robot
                # stays at MJCF initial pose until the coordinator's
                # first command arrives.
                mujoco.mj_forward(model, data)
            else:
                steps = 1 if skip_controller else config.mujoco_steps_per_frame
                for _ in range(steps):
                    mujoco.mj_step(model, data)

            if person_position_controller is not None:
                person_position_controller.tick(data)

            m_viewer.sync()

            # Always update odometry
            pos = data.qpos[0:3].copy()
            quat = data.qpos[3:7].copy()  # (w, x, y, z)
            shm.write_odom(pos, quat, time.time())

            # Low-level passthrough: export per-joint state + IMU to shm.
            if skip_controller:
                assert act_qposadr is not None and act_dofadr is not None
                q_out = data.qpos[act_qposadr].astype(np.float32)
                dq_out = data.qvel[act_dofadr].astype(np.float32)
                tau_out = data.actuator_force[:num_motors].astype(np.float32)
                shm.write_joint_state(q_out, dq_out, tau_out)
                # Base orientation from the free joint (qpos[3:7] is
                # w,x,y,z per MuJoCo convention) — no framequat sensor
                # needed, which menagerie's G1 doesn't ship with.
                quat = data.qpos[3:7].astype(np.float32)
                gyro = (
                    data.sensordata[imu_gyro_slice].astype(np.float32)
                    if imu_gyro_slice is not None
                    else np.zeros(3, dtype=np.float32)
                )
                accel = (
                    data.sensordata[imu_accel_slice].astype(np.float32)
                    if imu_accel_slice is not None
                    else np.zeros(3, dtype=np.float32)
                )
                shm.write_imu(quat, gyro, accel)

            current_time = time.time()

            # In low-level mode the robot scene has no head / lidar cameras,
            # so the video + lidar streams are skipped entirely.  Odom +
            # joint_state + imu above are all the rerun layer needs.
            if skip_controller:
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                continue

            # Video rendering
            if current_time - last_video_time >= video_interval:
                rgb_renderer.update_scene(data, camera=camera_id, scene_option=scene_option)
                pixels = rgb_renderer.render()
                shm.write_video(pixels)
                last_video_time = current_time

            # Lidar/depth rendering
            if current_time - last_lidar_time >= lidar_interval:
                # Render all depth cameras
                depth_renderer.update_scene(data, camera=lidar_camera_id, scene_option=scene_option)
                depth_front = depth_renderer.render()

                depth_left_renderer.update_scene(
                    data, camera=lidar_left_camera_id, scene_option=scene_option
                )
                depth_left = depth_left_renderer.render()

                depth_right_renderer.update_scene(
                    data, camera=lidar_right_camera_id, scene_option=scene_option
                )
                depth_right = depth_right_renderer.render()

                shm.write_depth(depth_front, depth_left, depth_right)

                # Process depth images into lidar message
                all_points = []
                cameras_data = [
                    (
                        depth_front,
                        data.cam_xpos[lidar_camera_id],
                        data.cam_xmat[lidar_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_left,
                        data.cam_xpos[lidar_left_camera_id],
                        data.cam_xmat[lidar_left_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_right,
                        data.cam_xpos[lidar_right_camera_id],
                        data.cam_xmat[lidar_right_camera_id].reshape(3, 3),
                    ),
                ]

                for depth_image, camera_pos, camera_mat in cameras_data:
                    points = depth_image_to_point_cloud(
                        depth_image, camera_pos, camera_mat, fov_degrees=DEPTH_CAMERA_FOV
                    )
                    if points.size > 0:
                        all_points.append(points)

                if all_points:
                    combined_points = np.vstack(all_points)
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(combined_points)
                    pcd = pcd.voxel_down_sample(voxel_size=LIDAR_RESOLUTION)

                    lidar_msg = PointCloud2(
                        pointcloud=pcd,
                        ts=time.time(),
                        frame_id="world",
                    )
                    shm.write_lidar(lidar_msg)

                last_lidar_time = current_time

            # Control simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        if person_position_controller is not None:
            person_position_controller.stop()


if __name__ == "__main__":
    global_config = pickle.loads(base64.b64decode(sys.argv[1]))
    shm_names = json.loads(sys.argv[2])
    control_mode = sys.argv[3] if len(sys.argv) > 3 else "high_level"

    shm = ShmReader(shm_names)

    def signal_handler(_signum: int, _frame: Any) -> None:
        # Signal the main loop to exit gracefully so the viewer context
        # manager can close the window and clean up resources.
        shm.signal_stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        _run_simulation(global_config, shm, control_mode=control_mode)
    finally:
        shm.cleanup()
