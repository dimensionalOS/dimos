#!/usr/bin/env python3
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

"""M20 native navigation blueprint — no Docker, no ROS.

All modules run on the NOS host via nix:
- DrddsLidarBridge: reads drdds SHM (lidar + IMU) and publishes to LCM
- SLAM: either AriseSLAM or FastLio2 (C++ NativeModule, env-selectable)
- SmartNav: full planning stack (terrain analysis, local planner, path
  follower, simple planner, PGO, CmdVelMux, ClickToGoal)
- NavCmdPub: raw FastDDS publisher for /NAV_CMD (no rclpy needed)
- M20Connection: camera (RTSP), robot state (UDP heartbeat, gait, mode)

Data flow:
    drdds_recv (host) → POSIX SHM
    → DrddsLidarBridge → raw_points + imu (LCM)
    → SLAM backend → registered_scan + odometry
    → SmartNav (TerrainAnalysis → LocalPlanner → PathFollower)
    → CmdVelMux → cmd_vel → NavCmdPub → rt/NAV_CMD (FastDDS) → AOS motors

SLAM backend is selected by the `M20_SLAM_BACKEND` env var:
    arise (default) — AriseSLAM single-pass laser-mapping (current state;
        indoor yaw tracking broken pending Option C 4-node port, bead di-ony5x)
    fastlio2        — FAST-LIO2 (native, LCM-input, tight IMU coupling) —
        the click-to-goal unblocker tracked by bead di-857dn
"""

import os

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import AriseSLAM
from dimos.navigation.smart_nav.modules.fastlio2.fastlio2 import FastLio2
from dimos.robot.deeprobotics.m20.blueprints.nav.m20_rerun import (
    M20RerunLCM,
    camera_info_override,
    m20_odometry_tf_override,
    m20_rerun_blueprint,
    raw_points_override,
    registered_scan_override,
    static_robot,
)
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.robot.deeprobotics.m20.drdds_bridge.module import (
    AiryImuBridge,
    DrddsLidarBridge,
    NavCmdPub,
)
from dimos.visualization.vis_module import vis_module

# M20 physical dimensions
M20_HEIGHT_CLEARANCE = 0.57  # 570mm standing height
M20_LIDAR_HEIGHT = 0.47  # lidar at 47cm in agile stance

# CPU affinity for the latency-critical native sensor/SLAM path.
# NOS boots with `isolcpus=4,5,6,7` so cores 4-7 are reserved (no default
# scheduler placement). Pinning the native binaries there gives them isolation
# while Python smartnav workers run on 0-3. Without this pinning, 15+ R/D
# threads crammed onto 4 scheduler-default cores produced 3-4 SECOND IMU
# delivery gaps during rotation, which caused FAST-LIO2 to extrapolate forward
# on stale IMU and lock ICP to wrong local minima. See FASTLIO2_LOG Finding
# #35. M20_SLAM_CORES is the default for the native path; per-process overrides
# let viewer-test runs reserve one isolated core for Rerun listener workers.
def _cpu_affinity_from_env(
    name: str,
    default_raw: str | None,
) -> frozenset[int] | None:
    raw = os.environ.get(name)
    if raw is None:
        raw = default_raw
    if raw is None:
        return None
    raw = raw.strip()
    if not raw:
        return None
    try:
        return frozenset(int(c.strip()) for c in raw.split(",") if c.strip())
    except ValueError as exc:
        raise ValueError(f"{name} must be a comma-separated CPU list") from exc


_SLAM_CORES_RAW = os.environ.get("M20_SLAM_CORES", "4,5,6,7")
_SLAM_CPU_AFFINITY = _cpu_affinity_from_env("M20_SLAM_CORES", "4,5,6,7")
_FASTLIO_CPU_AFFINITY = _cpu_affinity_from_env("M20_FASTLIO_CORES", _SLAM_CORES_RAW)
_DRDDS_LIDAR_CPU_AFFINITY = _cpu_affinity_from_env(
    "M20_DRDDS_LIDAR_CORES",
    _SLAM_CORES_RAW,
)
_AIRY_IMU_CPU_AFFINITY = _cpu_affinity_from_env(
    "M20_AIRY_IMU_CORES",
    _SLAM_CORES_RAW,
)

# Robot-body AABB in base_link meters for lidar self-filtering. Computed
# from M20_high_res.urdf collision geometries at zero-joint pose, padded
# for leg swings during gait (see FASTLIO2_LOG Finding #34):
#   - body_link chassis      : X ±0.38 Y ±0.12 Z ±0.07
#   - hips/knees/wheels      : X ±0.40 Y ±0.25 Z −0.59..+0.05
#   - + leg-swing padding    : 0.05 X, 0.10 Y, 0.15 Z
# Points inside this box are dropped at the drdds_lidar_bridge before
# publishing to LCM, preventing the ikd-Tree from registering self-features
# as persistent world obstacles (which causes rotation-SLAM drift).
# Override with M20_BODY_CROP env var if needed.
_M20_BODY_CROP_DEFAULT = "-0.454,0.454,-0.354,0.354,-0.740,0.220"
_M20_BODY_CROP = os.environ.get("M20_BODY_CROP", _M20_BODY_CROP_DEFAULT).strip()

_SLAM_BACKEND = os.environ.get("M20_SLAM_BACKEND", "arise").strip().lower()
if _SLAM_BACKEND not in {"arise", "fastlio2"}:
    raise ValueError(
        f"M20_SLAM_BACKEND must be 'arise' or 'fastlio2' (got {_SLAM_BACKEND!r})"
    )

# Which IMU feeds FAST-LIO2:
#   yesense (legacy): DeepRobotics body IMU on /IMU via DrddsLidarBridge SHM.
#       Unknown IMU↔lidar extrinsic + cross-clock-domain timestamps with
#       rsdriver caused the 20–30 m/15 s stationary drift documented in
#       FASTLIO2_LOG Findings #7–#17.
#   airy: Front RoboSense Airy integrated IMU, tapped via AiryImuBridge
#       multicast (224.10.10.201:6681). Shares a PTP-locked hardware clock
#       with the Airy lidar optics; rotated into base_link at the bridge so
#       the FAST-LIO2 extrinsic is identity. This is the codex-recommended
#       clean test replacing yesense.
_FASTLIO2_IMU = os.environ.get("M20_FASTLIO2_IMU", "airy").strip().lower()
if _FASTLIO2_IMU not in {"yesense", "airy"}:
    raise ValueError(
        f"M20_FASTLIO2_IMU must be 'yesense' or 'airy' (got {_FASTLIO2_IMU!r})"
    )

if _SLAM_BACKEND == "fastlio2":
    # FAST-LIO2 via the Velodyne PointCloud2 path (aphexcx/FAST-LIO-NON-ROS
    # branch dimos-integration-velodyne). `velodyne.yaml` has the Airy IMU
    # extrinsic baked as identity — valid ONLY when the IMU input is the
    # AiryImuBridge output (which rotates into base_link). With yesense,
    # the extrinsic is wrong and drift will be significant.
    _slam_module = FastLio2.blueprint(
        build_command=None,
        config="velodyne.yaml",
        native_clock=(_FASTLIO2_IMU == "airy"),
        cpu_affinity=_FASTLIO_CPU_AFFINITY,
    )
else:
    _slam_module = AriseSLAM.blueprint(
        build_command=None,
        scan_voxel_size=0.1,
        max_range=50.0,
    )

_extra_imu_modules: tuple = ()
if _SLAM_BACKEND == "fastlio2" and _FASTLIO2_IMU == "airy":
    # rsdriver in send_separately:true mode publishes FRONT Airy alone on
    # /LIDAR/POINTS (in base_link frame, its extrinsic applied). Pair with
    # the front Airy IMU rotated into base_link — both streams in the same
    # frame, so velodyne.yaml's identity extrinsic is honest.
    #
    # M20_AIRY_LEVER_ARM_BASE_M="x,y,z" overrides the C++ default lever
    # arm for centripetal + tangential fictitious-accel subtraction during
    # rotation. Default (unset) uses the geometric Section-1.10 values
    # (0.320, 0, -0.013) for front / (-0.320, 0, -0.013) for rear. Empirical
    # tuning (Finding #29) suggests r_x ≈ 0.22 matches the effective rotation
    # center during wheel-legged yaw better than 0.320. Set to "0,0,0" to
    # disable the correction entirely.
    _lever_arm_env = os.environ.get("M20_AIRY_LEVER_ARM_BASE_M", "").strip()
    _extra_imu_modules = (
        AiryImuBridge.blueprint(
            build_command=None,
            which="front",
            frame="base_link",
            lever_arm_base_m=_lever_arm_env or None,
            cpu_affinity=_AIRY_IMU_CPU_AFFINITY,
        ),
    )

#   M20_NAV_ENABLED=0 drops the entire smart_nav() planner stack
# (simple_planner / local_planner / path_follower / terrain_analysis).
# Useful for pure SLAM + teleop testing where an autonomous planner
# could inject conflicting cmd_vel and make it impossible to isolate
# SLAM issues from planner issues.
_NAV_ENABLED = os.environ.get("M20_NAV_ENABLED", "1").strip() != "0"
_DIRECT_CLICK_WAYPOINT = os.environ.get("M20_DIRECT_CLICK_WAYPOINT", "0").strip() == "1"


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return float(value)


# Conservative live-bringup limits. Override with env vars once click-to-goal
# behavior is proven in the current physical test area.
_NAV_MAX_SPEED = _env_float("M20_NAV_MAX_SPEED", 0.20)
_NAV_AUTONOMY_SPEED = _env_float("M20_NAV_AUTONOMY_SPEED", _NAV_MAX_SPEED)
_NAV_MAX_ACCEL = _env_float("M20_NAV_MAX_ACCEL", 0.30)
_NAV_MAX_YAW_RATE = _env_float("M20_NAV_MAX_YAW_RATE", 40.0)
_NAV_MAX_COMMAND_DURATION = _env_float("M20_NAV_MAX_COMMAND_DURATION", 30.0)
_NAV_OMNI_DIR_GOAL_THRESHOLD = _env_float("M20_NAV_OMNI_DIR_GOAL_THRESHOLD", 2.0)
_NAV_OMNI_DIR_DIFF_THRESHOLD = _env_float("M20_NAV_OMNI_DIR_DIFF_THRESHOLD", 3.2)
_NAV_GOAL_REACHED_THRESHOLD = _env_float("M20_NAV_GOAL_REACHED_THRESHOLD", 0.30)
_NAV_GOAL_BEHIND_RANGE = _env_float("M20_NAV_GOAL_BEHIND_RANGE", 0.30)
_NAV_FREEZE_ANG = _env_float("M20_NAV_FREEZE_ANG", 180.0)

_nav_modules: tuple = ()
if _NAV_ENABLED:
    _nav_modules = (
        smart_nav(
            use_simple_planner=True,
            direct_click_waypoint=_DIRECT_CLICK_WAYPOINT,
            vehicle_height=M20_HEIGHT_CLEARANCE,
            terrain_analysis={
                "build_command": None,
                "obstacle_height_threshold": 0.01,
                "ground_height_threshold": 0.01,
            },
            local_planner={
                "build_command": None,
                "max_speed": _NAV_MAX_SPEED,
                "autonomy_speed": _NAV_AUTONOMY_SPEED,
                "goal_reached_threshold": _NAV_GOAL_REACHED_THRESHOLD,
                "goal_behind_range": _NAV_GOAL_BEHIND_RANGE,
                "freeze_ang": _NAV_FREEZE_ANG,
                "two_way_drive": False,
            },
            path_follower={
                "build_command": None,
                "max_speed": _NAV_MAX_SPEED,
                "autonomy_speed": _NAV_AUTONOMY_SPEED,
                "max_acceleration": _NAV_MAX_ACCEL,
                "max_yaw_rate": _NAV_MAX_YAW_RATE,
                "omni_dir_goal_threshold": _NAV_OMNI_DIR_GOAL_THRESHOLD,
                "omni_dir_diff_threshold": _NAV_OMNI_DIR_DIFF_THRESHOLD,
                "two_way_drive": False,
            },
            simple_planner={
                "cell_size": 0.3,
                "obstacle_height_threshold": 0.20,
                "inflation_radius": 0.4,
                "ground_offset_below_robot": M20_HEIGHT_CLEARANCE,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
            cmd_vel_mux={
                "max_nav_command_duration_sec": _NAV_MAX_COMMAND_DURATION,
            },
        ),
    )

m20_smartnav_native = (
    autoconnect(
        m20_connection(
            ip="10.21.31.103",
            enable_camera=False,  # TODO: re-enable once startup is fast
        ),
        NavCmdPub.blueprint(),
        DrddsLidarBridge.blueprint(
            build_command=None,
            body_crop=_M20_BODY_CROP or None,
            cpu_affinity=_DRDDS_LIDAR_CPU_AFFINITY,
        ),
        *_extra_imu_modules,
        _slam_module,
        *_nav_modules,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=smart_nav_rerun_config(
                {
                    "blueprint": m20_rerun_blueprint,
                    "visual_override": {
                        "world/camera_info": camera_info_override,
                        "world/odometry": m20_odometry_tf_override,
                        "world/raw_points": raw_points_override,
                        "world/registered_scan": registered_scan_override,
                    },
                    "pubsubs": [M20RerunLCM()],
                    "max_hz": {
                        "world/registered_scan": 1.0,
                    },
                    "static": {
                        "world/tf/base_link": static_robot,
                    },
                }
            ),
        ),
    )
    .remappings(
        [
            # DrddsLidarBridge outputs "lidar"; SLAM backends expect "raw_points"
            (DrddsLidarBridge, "lidar", "raw_points"),
            # When the Airy IMU path is active, two IMU publishers coexist —
            # rename both to distinct topics and pin FAST-LIO2 to the Airy one
            # so autoconnect isn't ambiguous. AriseSLAM (the other SLAM option)
            # stays on the default `imu` topic since it's never co-scheduled
            # with AiryImuBridge.
            *(
                [
                    (DrddsLidarBridge, "imu", "yesense_imu"),
                    (AiryImuBridge, "imu", "airy_imu_front"),
                    (FastLio2, "imu", "airy_imu_front"),
                ]
                if _SLAM_BACKEND == "fastlio2" and _FASTLIO2_IMU == "airy"
                else []
            ),
        ]
    )
    .global_config(
        n_workers=4,
        robot_model="deeprobotics_m20",
        robot_ip="10.21.31.103",
        robot_width=0.45,
        robot_rotation_diameter=0.6,
    )
)


def main() -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    coordinator = ModuleCoordinator.build(m20_smartnav_native)
    coordinator.loop()


__all__ = ["m20_smartnav_native"]

if __name__ == "__main__":
    main()
