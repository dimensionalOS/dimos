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

"""3D navigation on Go2 with the RPP follower as the trajectory controller.

The exact ``unitree_go2_mls_htc`` perception/planning stack (voxel-grid
mapping, MLS planning, ``DanLocalPlanner`` gating/smoothing) with the tracking
stage swapped: ``DanHolonomicTC`` + ``MovementManager``'s velocity mux are
replaced by a ``ControlCoordinator`` running the calibrated regulated-pure-
pursuit follower — the same controller ``unitree-go2-rpp-controller`` ships.
Head-to-head A/B against ``unitree-go2-mls-htc`` behind the same planner.

Wiring differences vs the htc blueprint:

- ``DanLocalPlanner.path`` feeds ``ControlCoordinator.path`` (name match); the
  coordinator broadcasts each committed path to the ``rpp_follower`` task. An
  empty planner path (nothing traversable) cancels the active follow.
- The coordinator's transport-LCM base adapter drives the robot on
  ``/go2/cmd_vel`` and reads leg odom on ``/go2/odom``, so ``GO2Connection``'s
  ``cmd_vel`` is remapped onto that topic and ``odom`` is pinned to it.
- ``MovementManager`` stays for the click->goal relay only; its velocity mux is
  idle (the coordinator arbitrates instead) and its ``cmd_vel`` is remapped off
  the bus so nothing races the adapter.
- ``KeyboardTeleop`` (pygame window) publishes ``/cmd_vel`` ->
  ``twist_command`` -> the priority-20 ``vel_go2`` task, preempting the
  priority-10 follower while a WASD key is held — the manual override for
  positioning between runs.

Run::

    dimos --robot-ip <ip> run unitree-go2-mls-rpp
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_mls_htc import (
    _nav_rerun_config,
    go2_lidar_height,
    voxel_size,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

_go2_joints = make_twist_base_joints("go2")

unitree_go2_mls_rpp = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
        GO2Connection.blueprint(motion_mode="mcf"),
        VoxelGridMapper.blueprint(
            voxel_size=voxel_size,
            frame_id="world",
            emit_every=1,
        ),
        MLSPlannerNative.blueprint(
            world_frame="world",
            voxel_size=voxel_size,
            robot_height=go2_lidar_height,
            wall_clearance_m=0.2,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=0.16,
            step_penalty_weight=1.0,
            viz_publish_hz=0.0,
        ).remappings(
            [
                (MLSPlannerNative, "path", "planner_path"),
                # The planner's start pose is the robot's odom pose
                (MLSPlannerNative, "start_pose", "odom"),
            ]
        ),
        GoalRelay.blueprint(),
        DanLocalPlanner.blueprint(resample_spacing_m=0.1),
        # Click->goal relay only: the coordinator owns velocity arbitration, so
        # the mux inputs (nav_cmd_vel/tele_cmd_vel) are left unconnected.
        MovementManager.blueprint(),
        ControlCoordinator.blueprint(
            publish_joint_state=True,
            hardware=[
                HardwareComponent(
                    hardware_id="go2",
                    hardware_type=HardwareType.BASE,
                    joints=_go2_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                # Idle/teleop velocity task. Priority 20 so a held WASD key
                # preempts the priority-10 follower for repositioning.
                TaskConfig(
                    name="vel_go2",
                    type="velocity",
                    joint_names=_go2_joints,
                    priority=20,
                    params={"zero_on_timeout": False},
                ),
                # Same calibrated pursuit setup as unitree-go2-rpp-controller.
                TaskConfig(
                    name="rpp_follower",
                    type="rpp_path_follower",
                    joint_names=_go2_joints,
                    priority=10,
                    params={
                        "speed": 0.7,
                        "goal_tolerance": 0.20,
                        "orientation_tolerance": 0.35,
                        "k_angular": 1.5,
                        "lookahead_min": 0.5,
                        "lookahead_max": 0.7,
                        "lookahead_speed_scale": 2.0,
                        "forward_only": True,
                    },
                ),
            ],
        ),
        KeyboardTeleop.blueprint(publish_only_when_active=True),
    )
    .remappings(
        [
            # The coordinator's base adapter owns /go2/cmd_vel; keep the teleop
            # bus (/cmd_vel) and the robot input separate.
            (GO2Connection, "cmd_vel", "go2_cmd_vel"),
            # MovementManager's mux never fires here; keep its output off the
            # teleop bus regardless.
            (MovementManager, "cmd_vel", "mm_cmd_vel"),
        ]
    )
    .transports(
        {
            # Teleop -> coordinator twist_command (vel_go2 task).
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            # Base adapter <-> GO2Connection link (drives the robot, reads odom).
            # The adapter derives these topics from hardware_id="go2".
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            # Committed nav path -> coordinator (also injectable externally).
            ("path", Path): LCMTransport("/path", Path),
            # Optional runtime cruise-speed override for the follower.
            ("speed", Float32): LCMTransport("/speed", Float32),
            # Teleop gate events; unused here but the stream needs a home.
            ("gate", Int8): LCMTransport("/benchmark/gate", Int8),
            # Aggregated joint state for observability (positions = [x,y,yaw]).
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("coordinator_joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
        }
    )
    .global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)
)
