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

"""Mobile manipulation coordinator blueprints.

Usage:
    dimos run coordinator-mock-twist-base                # Mock holonomic base
    dimos run coordinator-mobile-manip-mock              # Mock arm + base
    dimos run coordinator-flowbase                       # FlowBase holonomic base (Portal RPC)
    dimos run coordinator-flowbase-keyboard-teleop       # FlowBase + WASD pygame teleop
    dimos run coordinator-flowbase-nav                   # FlowBase + FastLio2 + nav stack (click-to-drive)
    dimos run coordinator-sim-fopdt                      # FOPDT sim plant on /go2/cmd_vel|odom (Go2-shaped)
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.drive_trains.flowbase.driver import FlowBaseDriver
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.navigation.nav_stack.modules.nav_record.nav_record import NavRecord
from dimos.navigation.odometry_to_pose_stamped import OdometryToPoseStamped
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.catalog.ufactory import xarm7 as _catalog_xarm7
from dimos.robot.sim.fopdt_plant_connection import FopdtPlantConnection
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.benchmarking.benchmark import Benchmarker
from dimos.utils.benchmarking.characterization_recorder import CharacterizationRecorder
from dimos.utils.benchmarking.plant import (
    FLOWBASE_CMD_MAX_ACC,
    FLOWBASE_CMD_MAX_VEL,
    FLOWBASE_PLANT_FITTED,
)
from dimos.utils.path_utils import get_project_root
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

_base_joints = make_twist_base_joints("base")

# FlowBase pure-pursuit lookahead (m). The follower defaults to 0.5 m, which on
# a 1 m-radius circle leaves a ~10 cm geometric chord offset (error ~L²/2R).
# A benchmark lookahead sweep (2026-06-11) found 0.25 m cuts circle CTE ~4.5×
# (10.2→2.3 cm) and corner ~2× with no regressions and no wobble up to 0.6 m/s
# (0.15 m was tighter on the circle but regressed the square at speed). This is
# FlowBase-specific — applied per-task below, leaving the global 0.5 m default
# intact for the Go2 and other robots.
_FLOWBASE_LOOKAHEAD = 0.25


def _mock_twist_base(hw_id: str = "base") -> HardwareComponent:
    """Mock holonomic twist base (3-DOF: vx, vy, wz)."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="mock_twist_base",
    )


def _flowbase_twist_base(
    hw_id: str = "base",
    address: str | None = None,
) -> HardwareComponent:
    """FlowBase holonomic platform via Portal RPC (3-DOF: vx, vy, wz).

    Address defaults to ``FlowBaseAdapter.DEFAULT_ADDRESS`` when ``None``.
    """
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="flowbase",
        address=address,
    )


# Mock holonomic twist base (3-DOF: vx, vy, wz)
coordinator_mock_twist_base = ControlCoordinator.blueprint(
    hardware=[_mock_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
            params={"zero_on_timeout": False},
        ),
        # Closed-loop path follower used by the benchmark tool.
        # Inactive until the tool RPCs configure(...) + start_path(...).
        TaskConfig(
            name="path_follower",
            type="path_follower",
            joint_names=_base_joints,
            priority=20,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase holonomic twist base (3-DOF: vx, vy, wz) over Portal RPC
coordinator_flowbase = ControlCoordinator.blueprint(
    hardware=[_flowbase_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
            params={"zero_on_timeout": False},
        ),
        # Closed-loop path follower used by the benchmark tool.
        # Inactive until the tool RPCs configure(...) + start_path(...).
        TaskConfig(
            name="path_follower",
            type="path_follower",
            joint_names=_base_joints,
            priority=20,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase + WASD pygame keyboard teleop in a single blueprint
coordinator_flowbase_keyboard_teleop = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_flowbase_twist_base()],
        tasks=[
            # Pure teleop: zero_on_timeout=True so a stalled/lost command stream
            # actively BRAKES instead of coasting (Mode-A runaway fix; see
            # flowbase-teleop-runaway investigation). Safe here because
            # path_follower is dormant (only the benchmark tool RPCs it).
            # NOTE: BENCHMARKING through this coordinator needs this flipped back
            # to False — vel_base (pri 20) would otherwise preempt path_follower
            # (pri 10). Does NOT cover Mode B (stuck key) or Mode C (PC death).
            TaskConfig(
                name="vel_base",
                type="velocity",
                joint_names=_base_joints,
                priority=20,
                params={"zero_on_timeout": True},
            ),
            # Closed-loop path follower used by the benchmark tool. Inactive
            # until the tool RPCs configure(...) + start_path(...).
            TaskConfig(
                name="path_follower",
                type="path_follower",
                joint_names=_base_joints,
                priority=10,
                params={"lookahead_dist": _FLOWBASE_LOOKAHEAD},
            ),
        ],
    ),
    KeyboardTeleop.blueprint(publish_only_when_active=True),
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

# FlowBase + Livox MID-360 + FastLio2 SLAM + nav stack with click-to-drive in Rerun. The velocity
# sink is ControlCoordinator + FlowBaseAdapter

_flowbase_mid360_mount = Pose(0.20, -0.20, 0.10, *Quaternion.from_euler(Vector3(0, 0, 0)))

# Global-planner lookahead for the FlowBase nav stack. The proven base nav uses
# SimplePlanner lookahead_distance=2.0 — it aims ~2 m ahead and cuts curves (the
# same pure-pursuit chord effect the follower benchmark found). The benchmark
# lookahead sweep showed a shorter lookahead tracks tight curves far better, so
# the `-nav-tuned` blueprint drops it while keeping the SAME working stack.
# Sweep per run via -o <simpleplanner>.lookahead_distance=<x>.
_FLOWBASE_NAV_LOOKAHEAD = 1.0


def _make_flowbase_nav(lookahead_distance: float, *, record_db: str = "nav_recording.db"):
    """FlowBase MID-360 + FastLio2 SLAM + terrain-aware simple nav stack with
    click-to-drive in Rerun. ``lookahead_distance`` tunes the SimplePlanner's
    curve-cut (proven base value = 2.0). Records planned path + odometry so runs
    can be CTE-scored against the Phase-0 baseline (20260603-164011)."""
    return (
        autoconnect(
            FastLio2.blueprint(
                host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
                lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
                mount=_flowbase_mid360_mount,
                map_freq=1.0,
                config="default.yaml",
            ),
            create_nav_stack(
                planner="simple",
                record=True,
                nav_record={
                    "db_path": os.path.join(os.environ.get("DIMOS_RUN_LOG_DIR", "."), record_db)
                },
                vehicle_height=0.5,  # FlowBase platform clearance — tune if needed
                max_speed=0.8,  # conservative starting point
                terrain_analysis={
                    # MID-360 is mounted ~10cm above base (close to floor); G1 has it at ~1.2m.
                    # Looser thresholds avoid classifying floor noise as obstacles.
                    "obstacle_height_threshold": 0.15,
                    "ground_height_threshold": 0.10,
                    "sensor_range": 20,
                },
                local_planner={
                    # Reusing G1's precomputed paths until FlowBase-specific ones exist.
                    "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
                    "publish_free_paths": False,
                },
                simple_planner={
                    "cell_size": 0.2,
                    "obstacle_height_threshold": 0.15,
                    "inflation_radius": 0.3,  # FlowBase footprint smaller than G1's 0.5
                    "lookahead_distance": lookahead_distance,
                    "replan_rate": 5.0,
                    "replan_cooldown": 2.0,
                },
            ),
            # MovementManager: subscribes clicked_point + nav_cmd_vel + tele_cmd_vel,
            # publishes muxed cmd_vel + goal (+ way_point, disconnected below).
            MovementManager.blueprint(),
            # FlowBase driver: ControlCoordinator with the existing JointVelocityTask
            # passthrough; receives Twist from MovementManager on LCM /cmd_vel.
            ControlCoordinator.blueprint(
                hardware=[_flowbase_twist_base()],
                tasks=[
                    TaskConfig(
                        name="vel_base",
                        type="velocity",
                        joint_names=_base_joints,
                        priority=10,
                    ),
                ],
            ),
            RerunBridgeModule.blueprint(
                **nav_stack_rerun_config({"memory_limit": "1GB"}, vis_throttle=0.5),
                rerun_open="native",
            ),
            RerunWebSocketServer.blueprint(),
        )
        .remappings(
            [
                (FastLio2, "lidar", "registered_scan"),
                (FastLio2, "global_map", "global_map_fastlio"),
                # SimplePlanner / FarPlanner owns way_point — disconnect MovementManager's
                # redundant pass-through copy (matches unitree-g1-nav-onboard).
                (MovementManager, "way_point", "_mgr_way_point_unused"),
            ]
        )
        .transports(
            {
                # MovementManager.cmd_vel publishes to LCM /cmd_vel by default; the
                # coordinator's twist_command listens on the same topic.
                ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
                ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            }
        )
        .global_config(n_workers=8)
    )


# Proven click-to-go nav (produced the Phase-0 baseline 20260603-164011).
coordinator_flowbase_nav = _make_flowbase_nav(2.0)
# SAME stack, shorter global-planner lookahead → tighter curve tracking, for an
# apples-to-apples click-to-go comparison vs the baseline.
coordinator_flowbase_nav_tuned = (
    _make_flowbase_nav(_FLOWBASE_NAV_LOOKAHEAD)
    .remappings(
        [
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
            # SimplePlanner / FarPlanner owns way_point — disconnect MovementManager's
            # redundant pass-through copy (matches unitree-g1-nav-onboard).
            (MovementManager, "way_point", "_mgr_way_point_unused"),
            # MovementManager.cmd_vel publishes to LCM /cmd_vel by default; the
            # coordinator's twist_command listens on the same name.
            (ControlCoordinator, "twist_command", "cmd_vel"),
        ]
    )
    .global_config(n_workers=8)
)


# Mock arm (7-DOF) + mock holonomic base (3-DOF)
_mock_arm_cfg = _catalog_xarm7(name="arm")

coordinator_mobile_manip_mock = ControlCoordinator.blueprint(
    hardware=[_mock_arm_cfg.to_hardware_component(), _mock_twist_base()],
    tasks=[
        _mock_arm_cfg.to_task_config(task_name="traj_arm"),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
            params={"zero_on_timeout": False},
        ),
        # Closed-loop path follower used by the benchmark tool.
        # Inactive until the tool RPCs configure(...) + start_path(...).
        TaskConfig(
            name="path_follower",
            type="path_follower",
            joint_names=_base_joints,
            priority=20,
        ),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])


# FOPDT in-process sim plant + a ControlCoordinator on top, so the
# tuning tools see exactly the same /cmd_vel + /coordinator/joint_state
# contract sim and hw. FopdtPlantConnection exposes /sim/cmd_vel (In)
# and /sim/odom (Out); the coord drives /sim/cmd_vel via its
# transport_lcm adapter (hardware_id="sim"), reads pose back via the
# same adapter's /sim/odom subscription, and publishes JointState +
# hosts the path_follower task. Drop-in stand-in for a real robot.
_sim_joints = make_twist_base_joints("sim")

coordinator_sim_fopdt = (
    autoconnect(
        FopdtPlantConnection.blueprint(),
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="sim",
                    hardware_type=HardwareType.BASE,
                    joints=_sim_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="vel_sim",
                    type="velocity",
                    joint_names=_sim_joints,
                    priority=10,
                    params={"zero_on_timeout": False},
                ),
                TaskConfig(
                    name="path_follower",
                    type="path_follower",
                    joint_names=_sim_joints,
                    priority=20,
                ),
            ],
        ),
    )
    .remappings(
        [
            (FopdtPlantConnection, "cmd_vel", "sim_cmd_vel"),
            (FopdtPlantConnection, "odom", "sim_odom"),
        ]
    )
    .transports(
        {
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("sim_cmd_vel", Twist): LCMTransport("/sim/cmd_vel", Twist),
            ("sim_odom", PoseStamped): LCMTransport("/sim/odom", PoseStamped),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
)


# FlowBase-shaped FOPDT sim: identical to coordinator_sim_fopdt but publishes
# joint_state under the `base/*` prefix (instead of `sim/*`) so that
# `characterization --robot flowbase` can validate the full LCM/gate/topic
# plumbing against a simulated plant with NO real robot connected.
#
# The plant is the vendored 2026-06-09 FlowBase fit with the firmware-style
# command limiter, ticking at the coordinator's 100 Hz, so controller
# behavior in this sim is representative of the real base.
coordinator_sim_fopdt_flowbase = (
    autoconnect(
        FopdtPlantConnection.blueprint(
            plant_params=FLOWBASE_PLANT_FITTED,
            tick_rate_hz=100.0,
            cmd_max_vel=FLOWBASE_CMD_MAX_VEL,
            cmd_max_acc=FLOWBASE_CMD_MAX_ACC,
        ),
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="sim",
                    hardware_type=HardwareType.BASE,
                    joints=_base_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="vel_sim",
                    type="velocity",
                    joint_names=_base_joints,
                    priority=10,
                    params={"zero_on_timeout": False},
                ),
                TaskConfig(
                    name="path_follower",
                    type="path_follower",
                    joint_names=_base_joints,
                    priority=20,
                ),
                # FF + per-axis P trajectory tracker (trajtrack arm).
                # Inactive until the Benchmarker RPCs configure + start_path.
                TaskConfig(
                    name="trajectory_tracker",
                    type="trajectory_tracking",
                    joint_names=_base_joints,
                    priority=20,
                ),
            ],
        ),
    )
    .remappings(
        [
            (FopdtPlantConnection, "cmd_vel", "sim_cmd_vel"),
            (FopdtPlantConnection, "odom", "sim_odom"),
        ]
    )
    .transports(
        {
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("sim_cmd_vel", Twist): LCMTransport("/sim/cmd_vel", Twist),
            ("sim_odom", PoseStamped): LCMTransport("/sim/odom", PoseStamped),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
)


# FlowBase precision-nav — mirrors the Go2 precision-nav (voxel/A* planner →
# precision controller), made frame-consistent for FlowBase.
#
# Click a goal in the rerun viewer; FastLio2 (Mid-360 SLAM) feeds
# VoxelGridMapper → CostMapper → ReplanningAStarPlanner, which emits `path`.
# The ControlCoordinator's `precision_follower` (PrecisionPathFollowerTask)
# follows it; KeyboardTeleop 0-9 keys tune the corridor half-width live.
#
# Frame consistency (the key difference vs Go2): FastLio2 emits `Odometry`, but
# the planner (`odom: In[PoseStamped]`) and the coordinator's transport_lcm
# adapter both need `PoseStamped`. OdometryToPoseStamped converts it and the
# SINGLE SLAM pose is published on `/flowbase/odom`, feeding BOTH — so the
# path/map and the follower share one frame. The coordinator writes `cmd_vel`
# on `/flowbase/cmd_vel`; FlowBaseDriver forwards it to the FlowBase via Portal
# RPC (applying the Y/yaw negation). The planner's `nav_cmd_vel` is left unwired.
#
# NOTE: reuses `_flowbase_mid360_mount` (the un-calibrated guess); the measured
# mount Pose(0.22,-0.185,0.381) would improve map quality — see tuning log open
# items. Tracking uses FastLio2 odom, so the mount mainly affects obstacle map.
_flowbase_precision_joints = make_twist_base_joints("flowbase")
_FLOWBASE_ARTIFACT = (
    get_project_root()
    / "data"
    / "characterization"
    / "flowbase"
    / "flowbase_config_hw_concrete_2026-06-09_704a591f5.json"
)

coordinator_flowbase_precision_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
            mount=_flowbase_mid360_mount,
            map_freq=1.0,
            config="default.yaml",
        ),
        OdometryToPoseStamped.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        ControlCoordinator.blueprint(
            publish_joint_state=True,
            hardware=[
                HardwareComponent(
                    hardware_id="flowbase",
                    hardware_type=HardwareType.BASE,
                    joints=_flowbase_precision_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="precision_follower",
                    type="precision_path_follower",
                    joint_names=_flowbase_precision_joints,
                    priority=10,
                    params={
                        "artifact_path": str(_FLOWBASE_ARTIFACT),
                        "speed": 0.5,  # under the measured 0.63 m/s ceiling
                        "v_max_override": 0.5,
                        "lookahead_dist": _FLOWBASE_LOOKAHEAD,
                    },
                ),
            ],
        ),
        FlowBaseDriver.blueprint(),
        KeyboardTeleop.blueprint(
            publish_only_when_active=True,
            disable_movement=True,  # 0-9 e_max slider only; no WASD Twist
        ),
        vis_module(viewer_backend=global_config.viewer),
        # Records planned `path` + actual `odometry` (+ cmd_vel) to SQLite so a
        # click-to-goal run can be CTE-scored against the Phase-0 nav baseline
        # (flowbase_baselines/20260603-164011). Mirrors coordinator_flowbase_nav's
        # record=True. Override path with -o nav-record.db_path=<file>.
        NavRecord.blueprint(
            db_path=os.path.join(
                os.environ.get("DIMOS_RUN_LOG_DIR", "."), "precision_nav_recording.db"
            ),
        ),
    )
    .remappings(
        [
            # FastLio2's accumulated map → distinct name so it doesn't clobber
            # VoxelGridMapper's `global_map` (which CostMapper consumes).
            (FastLio2, "global_map", "global_map_fastlio"),
            # OdometryToPoseStamped publishes the SLAM pose as `odom` so it
            # autoconnects to ReplanningAStarPlanner.odom and shares /flowbase/odom.
            (OdometryToPoseStamped, "pose", "odom"),
            # NavRecord: skip recording the heavy voxel global_map (remap to an
            # unbound name) — only path + odometry + cmd_vel are needed for CTE.
            (NavRecord, "global_map", "global_map_pgo"),
            # The planner emits goal_reached as dimos_lcm.Bool but NavRecord's
            # port is dimos.msgs.Bool — remap to an unbound name to avoid the
            # type conflict (not needed for scoring).
            (NavRecord, "goal_reached", "goal_reached_unused"),
        ]
    )
    .transports(
        {
            # ONE SLAM pose feeds the planner AND the coordinator's transport
            # adapter (which reads /flowbase/odom by hardware_id) → frame-consistent.
            ("odom", PoseStamped): LCMTransport("/flowbase/odom", PoseStamped),
            # coordinator's transport adapter writes cmd_vel here; FlowBaseDriver reads it.
            ("cmd_vel", Twist): LCMTransport("/flowbase/cmd_vel", Twist),
            ("e_max", Float32): LCMTransport("/e_max", Float32),
            ("path", Path): LCMTransport("/precision_nav/path", Path),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(n_workers=12, robot_model="flowbase")
)


# FlowBase operating-point benchmark — one-terminal HW flow, mirrors the Go2
# benchmark (unitree_go2_benchmark + _rg). Bundles the FlowBase coordinator +
# pygame KeyboardTeleop + the Benchmarker module + a telemetry recorder so the
# operator runs a single command and steps through line/corner/square/circle ×
# speeds, gated in the pygame window (ENTER run, K skip, Backspace quit). The
# Benchmarker scores cross-track error from `joint_state` positions ([x,y,yaw],
# populated by the flowbase adapter's read_odometry over Portal RPC) — NOT from
# a raw /odom topic — so no SLAM/odom transport is needed here.
#
# The coordinator mirrors `coordinator_flowbase` (direct Portal-RPC adapter,
# hw_id="base" so joint_state lands under the "base/*" prefix the flowbase plant
# profile reads) and adds the precision_follower (rg arm). Priorities mirror the
# Go2 benchmark coord: vel_base is HIGHEST (pri 20) so the operator keeps
# keyboard override authority over the autonomous follower (highest priority
# wins); the followers (pri 10) drive only while teleop is idle. zero_on_timeout
# is False so vel_base goes DORMANT when no key is pressed (does NOT brake) —
# otherwise it would continuously preempt the follower (Mode-A flag is for pure
# teleop, not benchmarking). e-stop is mandatory: firmware has no command-deadman.
coordinator_flowbase_benchmark = ControlCoordinator.blueprint(
    publish_joint_state=True,
    hardware=[_flowbase_twist_base()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=20,
            params={"zero_on_timeout": False},
        ),
        # Baseline arm. Inactive until the Benchmarker RPCs configure(...) +
        # start_path(...).
        TaskConfig(
            name="path_follower",
            type="path_follower",
            joint_names=_base_joints,
            priority=10,
            params={"lookahead_dist": _FLOWBASE_LOOKAHEAD},
        ),
        # RG arm — same control law as path_follower but owns its own
        # solve_profile() recompute reacting to KeyboardTeleop's e_max stream
        # (number keys 0-9 set the corridor half-width live). artifact_path is
        # the dense-fit tuning JSON loaded on start_path() for the plant model +
        # velocity-profile constants.
        TaskConfig(
            name="precision_follower",
            type="precision_path_follower",
            joint_names=_base_joints,
            priority=10,
            params={
                "artifact_path": str(_FLOWBASE_ARTIFACT),
                "speed": 0.5,  # under the measured 0.63 m/s ceiling
                "v_max_override": 0.5,
                "lookahead_dist": _FLOWBASE_LOOKAHEAD,
            },
        ),
        # Trajtrack arm — time-parameterized FF + per-axis P tracker with
        # plant-gain inversion. Gains/limits trace to
        # dimos/control/tasks/trajectory_tracking_task/constants.py
        # (computed from the vendored 2026-06-09 fit).
        TaskConfig(
            name="trajectory_tracker",
            type="trajectory_tracking",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


def _make_flowbase_benchmark(rg: bool, tag: str, trajtrack: bool = False):
    """Compose the FlowBase benchmark bundle. ``rg=False`` is the bare baseline
    arm (routes runs through ``path_follower``); ``rg=True`` routes through
    ``precision_follower``; ``trajtrack=True`` routes through
    ``trajectory_tracker``. Recordings land at
    ``<repo>/data/benchmark/flowbase/`` (tag differentiates the arms)."""
    return autoconnect(
        coordinator_flowbase_benchmark,
        KeyboardTeleop.blueprint(publish_only_when_active=True),
        Benchmarker.blueprint(
            robot="flowbase", mode="hw", gate_source="stream", rg=rg, trajtrack=trajtrack
        ),
        CharacterizationRecorder.blueprint(
            robot_id="flowbase",
            tag=tag,
            out_dir=str(get_project_root() / "data" / "benchmark" / "flowbase"),
        ),
    ).transports(
        {
            ("gate", Int8): LCMTransport("/benchmark/gate", Int8),
            ("e_max", Float32): LCMTransport("/e_max", Float32),
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )


flowbase_benchmark = _make_flowbase_benchmark(rg=False, tag="benchmark")
flowbase_benchmark_rg = _make_flowbase_benchmark(rg=True, tag="benchmark_rg")
flowbase_benchmark_trajtrack = _make_flowbase_benchmark(
    rg=False, tag="benchmark_trajtrack", trajtrack=True
)


__all__ = [
    "coordinator_flowbase",
    "coordinator_flowbase_benchmark",
    "coordinator_flowbase_keyboard_teleop",
    "coordinator_flowbase_nav",
    "coordinator_flowbase_nav_tuned",
    "coordinator_flowbase_precision_nav",
    "coordinator_mobile_manip_mock",
    "coordinator_mock_twist_base",
    "coordinator_sim_fopdt",
    "coordinator_sim_fopdt_flowbase",
    "flowbase_benchmark",
    "flowbase_benchmark_rg",
    "flowbase_benchmark_trajtrack",
]
