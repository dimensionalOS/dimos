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

"""Alfred: lidar click-and-go navigation + pillar lift + both OpenArms planned from viser.

    dimos run alfred-nav

The navigation half is the house pattern every Mid-360 robot uses (see the Go2
``go2-zenoh-htc`` and ``unitree-go2-nav-3d`` blueprints), nothing Alfred-specific:

* ``PointLio`` on the Mid-360: lidar odometry ``odom -> mid360_link`` (also on tf) plus the
  registered cloud. No cameras, no CUDA.
* ``AlfredLidarMountTf``: the URDF mount tree re-rooted at ``mid360_link`` so
  ``odom <- base_link`` composes (Point-LIO owns the lidar's parent edge).
* ``RayTracingVoxelMap`` → ``MLSPlannerNative`` → ``StartRelay`` → ``DanLocalPlanner`` →
  ``DanHolonomicTC`` (holonomic, Alfred can strafe) → ``MovementManager`` (a click in rerun
  is the goal; ``tele_cmd_vel`` beats ``nav_cmd_vel``).
* ``AlfredHighLevel``: the ONLY writer to the FlowBase (Portal RPC + wheel odometry).

The manipulation half is unchanged from the sim:

* ``PillarConnection`` + ``ControlCoordinator`` with the pillar (LCM transport adapter) and the
  OpenArms (Damiao CAN when both ports are given, mock otherwise): one joint trajectory task
  for lift + arms. No base hardware in the coordinator on purpose.
* ``ManipulationModule`` on the whole-robot ``alfred_v1`` model (viser on :8095): plan the
  ``lift``, ``left_manipulator`` and ``right_manipulator`` groups, execute via the coordinator.
  It publishes no tf, so nothing plants a second ``world`` root beside the ``odom`` tree.
* ``KeyboardTeleop`` (pygame WASD/QE) publishes ``tele_cmd_vel``.

Hardware pins: ``DIMOS_POINTLIO_HOST_IP`` (this computer's address on the lidar's subnet) is
read by Point-LIO; the lidar address comes from ``ALFRED.mid360_ip``; the OpenArm CAN ports
from ``OPENARM_LEFT_CAN`` / ``OPENARM_RIGHT_CAN`` (both or neither); the pillar serial device
from ``PillarConnection`` config (``/dev/ttyUSB0``).
"""

from __future__ import annotations

from functools import partial
import os
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.robot.diy.alfred.alfred_model import alfred_arm_joints, alfred_model_config
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    PILLAR_MOTOR_TRANSPORTS,
)
from dimos.robot.diy.alfred.config import ALFRED, ALFRED_URDF
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredLidarMountTf
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import openarm_hardware
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory
from dimos.visualization.vis_module import vis_module

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 1.0

ODOM_FRAME = "odom"
LIDAR_FRAME = "mid360_link"
VOXEL_SIZE_M = 0.08
# The Mid-360 reaches far past this, but the voxel map raytraces every point from the
# sensor origin, so the far returns cost the most and carve the least reliable space.
MAP_MAX_RANGE_M = 15.0
# Wheeled base: a kerb-sized step is an obstacle, not a foothold (Go2 uses 0.16).
STEP_THRESHOLD_M = 0.06
ALFRED_RERUN_ROOT = "world/alfred"


def _openarm_hardware_from_env() -> HardwareComponent:
    """Real Damiao arms only when both CAN ports are set; the mock adapter otherwise."""
    return openarm_hardware(
        left_can_port=os.environ.get(OPENARM_LEFT_CAN_ENV) or None,
        right_can_port=os.environ.get(OPENARM_RIGHT_CAN_ENV) or None,
    )


def alfred_manipulation_tasks() -> list[TaskConfig]:
    """The coordinator's single trajectory task: planner executions for arms + lift, and any
    streamed ``joint_command`` (velocity-bounded; the lift at the pillar's safe 0.1 m/s)."""
    return [
        joint_trajectory_task(
            [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
            # The task wants a limit for every joint once any is given: arms keep the
            # task's own 1 rad/s default, the lift gets the pillar's safe speed.
            velocity_limits={
                **dict.fromkeys(alfred_arm_joints(), ARM_VELOCITY_LIMIT_RAD_S),
                PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
            },
        ),
    ]


def _path_colored(msg: Any, color: tuple[int, int, int]) -> Any:
    return msg.to_rerun(color=color)


def _empty_path_dropped(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route; keep the last drawn one.
    return None if len(msg.poses) == 0 else msg.to_rerun(color=(170, 60, 220))


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    factory = UrdfRobotStaticRerunFactory(urdf_path=ALFRED_URDF, root_path=ALFRED_RERUN_ROOT)
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


def _rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="Alfred nav",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
        collapse_panels=True,
    )


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "tf_axes": 0.35,
    "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
    # Held to a viewer-friendly rate: the viewer usually rides a wifi link.
    "max_hz": {
        "world/tf": 2.0,
        "world/lidar": 1.0,
        "world/global_map": 0.5,
        "world/local_map": 1.0,
        "world/surface_map": 1.0,
    },
    "visual_override": {
        "world/planner_path": _empty_path_dropped,
        "world/path": partial(_path_colored, color=(60, 220, 120)),
    },
}


alfred_nav = (
    autoconnect(
        # --- navigation (Go2 pattern on lidar odometry) -------------------------------
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        AlfredHighLevel.blueprint(),
        AlfredLidarMountTf.blueprint(),
        PointLio.blueprint(
            lidar_ip=ALFRED.mid360_ip,
            frame_id=ODOM_FRAME,
            sensor_frame_id=LIDAR_FRAME,
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=MAP_MAX_RANGE_M,
            emit_every=1,
            global_emit_every=50,
            min_health=-1,
            max_health=5,
            support_min=4,
            world_frame=ODOM_FRAME,
        ),
        MLSPlannerNative.blueprint(
            world_frame=ODOM_FRAME,
            base_frame="base_link",
            voxel_size=VOXEL_SIZE_M,
            # The span that must be free above a cell; the mast is taller but nothing up
            # there is a collision risk at door height (Jeff's ALFRED.body_height).
            robot_height=ALFRED.body_height,
            start_z_offset_m=0.0,  # base_link is on the floor
            wall_clearance_m=0.2,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=STEP_THRESHOLD_M,
            step_penalty_weight=4.0,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
            ]
        ),
        # Solely the tf-driven start_pose source for the dannav odom remaps below.
        StartRelay.blueprint(world_frame=ODOM_FRAME, base_frame="base_link"),
        DanLocalPlanner.blueprint(resample_spacing_m=0.1).remappings(
            [(DanLocalPlanner, "odom", "start_pose")]
        ),
        DanHolonomicTC.blueprint().remappings([(DanHolonomicTC, "odom", "start_pose")]),
        MovementManager.blueprint(),
        KeyboardTeleop.blueprint(),
        # --- manipulation (as alfred-sim, real pillar + arms) --------------------------
        PillarConnection.blueprint(),
        planner(
            model=alfred_model_config(),
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[pillar_hardware(), _openarm_hardware_from_env()],
            tasks=alfred_manipulation_tasks(),
        ),
    )
    .transports(dict(PILLAR_MOTOR_TRANSPORTS))
    .remappings(
        [
            # Operator twist goes through MovementManager's teleop/nav mux, not to the base.
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    )
    .global_config(n_workers=12, robot_model="alfred")
)
