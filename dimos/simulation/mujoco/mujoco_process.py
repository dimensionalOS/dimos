#!/usr/bin/env python3

# pyright: reportMissingImports=false
# pyright: reportMissingModuleSource=false

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
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.simulation.mujoco.constants import (
    LIDAR_FPS,
    LIDAR_RESOLUTION,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.model import (
    load_bundle_json,
    load_model,
    load_model_sdk2,
    load_scene_xml,
)
from dimos.simulation.mujoco.person_on_track import PersonPositionController
from dimos.simulation.mujoco.shared_memory import ShmReader
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_GLFW_KEY_UP = 265
_GLFW_KEY_DOWN = 264
# NOTE: Avoid keys that MuJoCo viewer uses for built-in toggles (can blank the view).
# FALCON/unitree_mujoco uses '9' for elastic band toggle; keep that convention.
_GLFW_KEY_BAND_TOGGLE = 57  # '9'


class ElasticBand:
    """Simple spring-damper 'gantry' assistance (ported from FALCON/unitree_mujoco style).

    Controls (in the MuJoCo viewer window):
    - G: toggle elastic band on/off
    - Up/Down arrows: shorten/lengthen the band (stronger/weaker lift)
    """

    def __init__(self) -> None:
        self.stiffness = 200.0
        self.damping = 100.0
        self.point = np.array([0.0, 0.0, 3.0], dtype=np.float64)  # world anchor
        # Rest length (m). Set to match the real harness starting length.
        self.length = 0.7
        # Enable by default (matches typical real harness setup).
        self.enable = True
        # Safety clamp to avoid insane impulses if the band parameters are mis-set.
        self.max_force = 500.0  # N per axis

    def reset_to_pose(self, attach_pos: np.ndarray) -> None:
        # Anchor above current position; rest length set so initial force is ~0.
        self.point = np.array(
            [float(attach_pos[0]), float(attach_pos[1]), float(attach_pos[2]) + 2.0],
            dtype=np.float64,
        )
        # Do not override self.length here; we want deterministic starting behavior across runs.

    def advance(self, x: np.ndarray, dx: np.ndarray) -> np.ndarray:
        delta = self.point - x
        dist = float(np.linalg.norm(delta))
        if dist < 1e-6:
            return np.zeros(3, dtype=np.float64)
        direction = delta / dist
        v = float(np.dot(dx, direction))
        f = (self.stiffness * (dist - self.length) - self.damping * v) * direction
        f = np.clip(f, -self.max_force, self.max_force)
        return f

    def key_callback(self, key: int) -> None:
        if key == _GLFW_KEY_BAND_TOGGLE:
            self.enable = not self.enable
            logger.info("Elastic band toggled", enabled=self.enable, length=self.length)
        elif key == _GLFW_KEY_UP:
            self.length = max(0.0, self.length - 0.05)
            logger.info("Elastic band up (shorten)", length=self.length)
        elif key == _GLFW_KEY_DOWN:
            self.length = self.length + 0.05
            logger.info("Elastic band down (lengthen)", length=self.length)


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
        return self._command.copy()

    def stop(self) -> None:
        """Stop method to satisfy InputController protocol."""
        pass


def _step_or_mirror(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: GlobalConfig,
    sdk2_bridge: Any | None,
    sdk2_mirror: Any | None,
    elastic_band: ElasticBand,
    gantry_body_id: int,
    last_sim_time: float,
) -> float:
    """Advance simulation one rendered frame (or mirror the real robot)."""
    if sdk2_mirror is not None:
        sdk2_mirror.apply_to_mujoco()
        mujoco.mj_forward(model, data)
        return float(data.time)

    for _ in range(config.mujoco_steps_per_frame):
        if gantry_body_id >= 0:
            if elastic_band.enable:
                force = elastic_band.advance(data.xpos[gantry_body_id], data.qvel[0:3])
                data.xfrc_applied[gantry_body_id, 0:3] = force
            else:
                data.xfrc_applied[gantry_body_id, 0:3] = 0.0

        mujoco.mj_step(model, data)
        if sdk2_bridge is not None:
            sdk2_bridge.publish_state()

    # Reset detection (viewer backspace): time jumps backwards.
    if sdk2_bridge is not None:
        sim_time = float(data.time)
        if sim_time + 1e-9 < last_sim_time:
            sdk2_bridge.on_mujoco_reset()
            if gantry_body_id >= 0:
                elastic_band.reset_to_pose(data.xpos[gantry_body_id].copy())
        return sim_time

    return float(data.time)


def _render_rgb_frame(
    *,
    rgb_renderer: mujoco.Renderer,
    scene_option: mujoco.MjvOption,
    data: mujoco.MjData,
    camera_id: int,
) -> np.ndarray:
    rgb_renderer.update_scene(data, camera=camera_id, scene_option=scene_option)
    return rgb_renderer.render()


def _render_depth_triplet(
    *,
    depth_front_renderer: mujoco.Renderer,
    depth_left_renderer: mujoco.Renderer,
    depth_right_renderer: mujoco.Renderer,
    scene_option: mujoco.MjvOption,
    data: mujoco.MjData,
    cam_front_id: int,
    cam_left_id: int,
    cam_right_id: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    depth_front_renderer.update_scene(data, camera=cam_front_id, scene_option=scene_option)
    depth_front = depth_front_renderer.render()

    depth_left_renderer.update_scene(data, camera=cam_left_id, scene_option=scene_option)
    depth_left = depth_left_renderer.render()

    depth_right_renderer.update_scene(data, camera=cam_right_id, scene_option=scene_option)
    depth_right = depth_right_renderer.render()

    return depth_front, depth_left, depth_right


def _run_simulation(config: GlobalConfig, shm: ShmReader) -> None:
    robot_name = config.robot_model or "unitree_go1"
    if robot_name == "unitree_go2":
        robot_name = "unitree_go1"

    # Only use a MuJoCo profile bundle when explicitly requested.
    # Otherwise fall back to the legacy behavior (menagerie assets + <robot>.xml include).
    profile = config.mujoco_profile
    scene_xml = load_scene_xml(config)

    # SDK2 controllers:
    # - sdk2: bridge subscribes lowcmd and publishes lowstate (simulated robot)
    # - mirror: subscribes real robot lowstate and mirrors it into MuJoCo for rendering (no DDS publishing)
    sdk2_bridge = None
    sdk2_mirror = None

    if config.mujoco_control_mode in ("sdk2", "mirror"):
        # SDK2 mode: load model without ONNX policy, use DDS bridge for control
        # SDK2BridgeController is created later, after mj_forward updates sensors
        model, data = load_model_sdk2(
            robot=robot_name,
            scene_xml=scene_xml,
            profile=profile,
        )
        # Debug: verify keyframe was applied in load_model_sdk2
        logger.info(
            "SDK2 model loaded",
            nkey=model.nkey,
            qpos_joints_0_5=data.qpos[7:13].tolist(),
            expected_joints=[-0.312, 0, 0, 0.669, -0.363, 0],
        )
    else:
        # ONNX policy mode (default): use shared memory controller
        controller = MockController(shm)
        model, data = load_model(
            controller,
            robot=robot_name,
            scene_xml=scene_xml,
            profile=profile,
        )

    if model is None or data is None:
        raise ValueError("Failed to load MuJoCo model: model or data is None")

    match robot_name:
        case "unitree_go1":
            z = 0.3
        case "unitree_g1":
            z = 0.8
        case _:
            z = 0

    pos = config.mujoco_start_pos_float

    data.qpos[0:3] = [pos[0], pos[1], z]

    mujoco.mj_forward(model, data)

    # Create SDK2 controller AFTER mj_forward so sensors are updated from keyframe
    if config.mujoco_control_mode == "sdk2":
        # Debug: verify qpos and sensordata after mj_forward
        logger.info(
            "After mj_forward",
            qpos_joints_0_5=data.qpos[7:13].tolist(),
            sensordata_0_5=data.sensordata[0:6].tolist(),
        )
        from dimos.simulation.mujoco.sdk2_bridge import (
            SDK2BridgeConfig,
            SDK2BridgeController,
        )
        sdk2_bridge = SDK2BridgeController(
            model,
            data,
            SDK2BridgeConfig(
                domain_id=config.sdk2_domain_id,
                interface=config.sdk2_interface,
                robot_type=robot_name.replace("unitree_", ""),  # "g1" -> "g1"
            ),
        )
        logger.info(
            "SDK2 bridge mode enabled",
            domain_id=config.sdk2_domain_id,
            interface=config.sdk2_interface,
        )
    elif config.mujoco_control_mode == "mirror":
        from dimos.simulation.mujoco.sdk2_bridge import SDK2BridgeConfig, SDK2MirrorController

        sdk2_mirror = SDK2MirrorController(
            model,
            data,
            SDK2BridgeConfig(
                domain_id=config.sdk2_domain_id,
                interface=config.sdk2_interface,
                robot_type=robot_name.replace("unitree_", ""),  # "g1" -> "g1"
            ),
        )
        logger.info(
            "SDK2 mirror mode enabled (subscriber-only)",
            domain_id=config.sdk2_domain_id,
            interface=config.sdk2_interface,
        )

    # Camera naming can differ per profile bundle. If bundle.json exists, use it.
    bundle_cfg = load_bundle_json(profile) if profile else None
    rgb_cam_name = (
        str(bundle_cfg.get("rgb_camera"))  # type: ignore[union-attr]
        if bundle_cfg and "rgb_camera" in bundle_cfg
        else "head_camera"
    )
    lidar_front_name = (
        str(bundle_cfg.get("lidar_front_camera"))  # type: ignore[union-attr]
        if bundle_cfg and "lidar_front_camera" in bundle_cfg
        else "lidar_front_camera"
    )
    lidar_left_name = (
        str(bundle_cfg.get("lidar_left_camera"))  # type: ignore[union-attr]
        if bundle_cfg and "lidar_left_camera" in bundle_cfg
        else "lidar_left_camera"
    )
    lidar_right_name = (
        str(bundle_cfg.get("lidar_right_camera"))  # type: ignore[union-attr]
        if bundle_cfg and "lidar_right_camera" in bundle_cfg
        else "lidar_right_camera"
    )

    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, rgb_cam_name)
    lidar_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, lidar_front_name)
    lidar_left_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, lidar_left_name)
    lidar_right_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, lidar_right_name)

    person_position_controller = PersonPositionController(model)

    shm.signal_ready()

    # Lightweight profiler: log rolling averages of time spent in the MuJoCo subprocess.
    # Includes physics stepping, rendering, pointcloud conversion, SHM writes, and policy inference.
    profiler_enabled = bool(getattr(config, "mujoco_profiler", False))
    profiler_interval_s = float(getattr(config, "mujoco_profiler_interval_s", 2.0))
    if profiler_enabled:
        from dimos.simulation.mujoco import policy as mujoco_policy

        mujoco_policy.set_mujoco_profiler_enabled(True)
        logger.info(
            "MuJoCo profiler enabled",
            interval_s=profiler_interval_s,
            video_fps=VIDEO_FPS,
            lidar_fps=LIDAR_FPS,
            steps_per_frame=config.mujoco_steps_per_frame,
        )

    # Elastic band / gantry assist (sim-only). Only meaningful when physics is stepping.
    elastic_band = ElasticBand()
    if config.mujoco_control_mode == "mirror":
        elastic_band.enable = False
    try:
        gantry_body_id = model.body("torso_link").id
    except Exception:
        gantry_body_id = -1

    if gantry_body_id >= 0:
        elastic_band.reset_to_pose(data.xpos[gantry_body_id].copy())
        logger.info(
            "Elastic band initialized",
            enabled=elastic_band.enable,
            toggle_key="9",
            adjust_keys="Up/Down",
            length=elastic_band.length,
        )

    with viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
        key_callback=elastic_band.key_callback,
    ) as m_viewer:
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

        # Profiler accumulators (seconds).
        acc_frames = 0
        acc_step_s = 0.0
        acc_sync_s = 0.0
        acc_odom_s = 0.0
        acc_rgb_render_s = 0.0
        acc_rgb_shm_s = 0.0
        acc_depth_render_s = 0.0
        acc_depth_shm_s = 0.0
        acc_pcd_s = 0.0
        acc_lidar_shm_s = 0.0
        next_report_t = time.perf_counter() + profiler_interval_s

        last_sim_time = float(data.time)

        while m_viewer.is_running() and not shm.should_stop():
            step_start = time.time()
            time.perf_counter()

            # Step / update simulation
            t0 = time.perf_counter()
            last_sim_time = _step_or_mirror(
                model=model,
                data=data,
                config=config,
                sdk2_bridge=sdk2_bridge,
                sdk2_mirror=sdk2_mirror,
                elastic_band=elastic_band,
                gantry_body_id=gantry_body_id,
                last_sim_time=last_sim_time,
            )
            acc_step_s += time.perf_counter() - t0

            t0 = time.perf_counter()
            person_position_controller.tick(data)

            m_viewer.sync()
            acc_sync_s += time.perf_counter() - t0

            # Always update odometry
            t0 = time.perf_counter()
            pos = data.qpos[0:3].copy()
            quat = data.qpos[3:7].copy()  # (w, x, y, z)
            shm.write_odom(pos, quat, time.time())
            acc_odom_s += time.perf_counter() - t0

            current_time = time.time()

            # Video rendering
            if current_time - last_video_time >= video_interval:
                t0 = time.perf_counter()
                pixels = _render_rgb_frame(
                    rgb_renderer=rgb_renderer,
                    scene_option=scene_option,
                    data=data,
                    camera_id=camera_id,
                )
                acc_rgb_render_s += time.perf_counter() - t0
                t0 = time.perf_counter()
                shm.write_video(pixels)
                acc_rgb_shm_s += time.perf_counter() - t0
                last_video_time = current_time

            # Lidar/depth rendering
            if current_time - last_lidar_time >= lidar_interval:
                # Render all depth cameras
                t0 = time.perf_counter()
                depth_front, depth_left, depth_right = _render_depth_triplet(
                    depth_front_renderer=depth_renderer,
                    depth_left_renderer=depth_left_renderer,
                    depth_right_renderer=depth_right_renderer,
                    scene_option=scene_option,
                    data=data,
                    cam_front_id=lidar_camera_id,
                    cam_left_id=lidar_left_camera_id,
                    cam_right_id=lidar_right_camera_id,
                )
                acc_depth_render_s += time.perf_counter() - t0

                t0 = time.perf_counter()
                shm.write_depth(depth_front, depth_left, depth_right)
                acc_depth_shm_s += time.perf_counter() - t0

                # Process depth images into lidar message
                t0 = time.perf_counter()
                all_points = []
                cameras_data = [
                    (
                        depth_front,
                        lidar_camera_id,
                        data.cam_xpos[lidar_camera_id],
                        data.cam_xmat[lidar_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_left,
                        lidar_left_camera_id,
                        data.cam_xpos[lidar_left_camera_id],
                        data.cam_xmat[lidar_left_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_right,
                        lidar_right_camera_id,
                        data.cam_xpos[lidar_right_camera_id],
                        data.cam_xmat[lidar_right_camera_id].reshape(3, 3),
                    ),
                ]

                for depth_image, cam_id, camera_pos, camera_mat in cameras_data:
                    points = depth_image_to_point_cloud(
                        depth_image,
                        camera_pos,
                        camera_mat,
                        fov_degrees=float(model.cam_fovy[cam_id]),
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
                    acc_pcd_s += time.perf_counter() - t0
                    t0 = time.perf_counter()
                    shm.write_lidar(lidar_msg)
                    acc_lidar_shm_s += time.perf_counter() - t0
                else:
                    acc_pcd_s += time.perf_counter() - t0

                last_lidar_time = current_time

            # Control simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            if profiler_enabled:
                acc_frames += 1
                now = time.perf_counter()
                if now >= next_report_t:
                    from dimos.simulation.mujoco import policy as mujoco_policy

                    pol = mujoco_policy.get_mujoco_profiler_and_reset()
                    calls = int(pol.get("control_calls", 0))
                    ctrl_ms = (float(pol.get("control_total_s", 0.0)) * 1000.0) / max(calls, 1)
                    obs_ms = (float(pol.get("obs_total_s", 0.0)) * 1000.0) / max(calls, 1)
                    onnx_ms = (float(pol.get("onnx_total_s", 0.0)) * 1000.0) / max(calls, 1)

                    def per_frame_ms(total_s: float) -> float:
                        return (total_s * 1000.0) / max(acc_frames, 1)

                    logger.info(
                        "MuJoCo perf (avg ms/frame)",
                        frames=acc_frames,
                        physics_ms=per_frame_ms(acc_step_s),
                        viewer_sync_ms=per_frame_ms(acc_sync_s),
                        odom_ms=per_frame_ms(acc_odom_s),
                        rgb_render_ms=per_frame_ms(acc_rgb_render_s),
                        rgb_shm_ms=per_frame_ms(acc_rgb_shm_s),
                        depth_render_ms=per_frame_ms(acc_depth_render_s),
                        depth_shm_ms=per_frame_ms(acc_depth_shm_s),
                        pcd_ms=per_frame_ms(acc_pcd_s),
                        lidar_shm_ms=per_frame_ms(acc_lidar_shm_s),
                        ctrl_calls=calls,
                        ctrl_total_ms_per_call=ctrl_ms,
                        ctrl_obs_ms_per_call=obs_ms,
                        ctrl_onnx_ms_per_call=onnx_ms,
                    )

                    acc_frames = 0
                    acc_step_s = 0.0
                    acc_sync_s = 0.0
                    acc_odom_s = 0.0
                    acc_rgb_render_s = 0.0
                    acc_rgb_shm_s = 0.0
                    acc_depth_render_s = 0.0
                    acc_depth_shm_s = 0.0
                    acc_pcd_s = 0.0
                    acc_lidar_shm_s = 0.0
                    next_report_t = now + profiler_interval_s

        person_position_controller.stop()


if __name__ == "__main__":

    def signal_handler(_signum: int, _frame: Any) -> None:
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    global_config = pickle.loads(base64.b64decode(sys.argv[1]))
    shm_names = json.loads(sys.argv[2])

    shm = ShmReader(shm_names)
    try:
        _run_simulation(global_config, shm)
    finally:
        shm.cleanup()
