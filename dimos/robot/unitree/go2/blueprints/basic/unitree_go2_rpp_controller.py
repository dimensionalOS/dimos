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

"""Unitree Go2 RPP controller — a standalone, path-following control process.

GO2Connection + a ControlCoordinator running the calibrated regulated-pure-
pursuit (RPP) follower, plus a KeyboardTeleop for manual positioning. There is
NO planner and NO map here: this blueprint is a controller you feed paths to
from any source over the transport.

Interface (pure LCM pub/sub — the ONLY coupling to whatever drives it):

    IN   path   (nav_msgs/Path)        -> coordinator.path  -> rpp_follower.set_path
    IN   speed  (std_msgs/Float32, m/s)-> coordinator.speed -> rpp_follower.set_speed
    OUT  odom   (geometry_msgs/PoseStamped, /go2/odom)  -- the Go2 leg odom
    OUT  cmd_vel(geometry_msgs/Twist,        /cmd_vel)  -- aggregated command echo

Two tasks ship: ``vel_go2`` (priority 20, idle/teleop) and ``rpp_follower``
(priority 10). The follower is the only task the coordinator's ``set_path``
broadcast arms, so a published path drives pursuit and nothing else. Holding a
WASD key publishes a Twist on ``/cmd_vel`` (the coordinator's ``twist_command``
input), driving the priority-20 ``vel_go2`` task and preempting the follower —
that is how the operator repositions/aims the robot between runs. Preemption
aborts the active path, so publish a new path to resume autonomous following.

KeyboardTeleop also emits a per-run **gate** (``ENTER``/``K``/``Backspace`` ->
advance/skip/quit) on ``/benchmark/gate``, which the benchmark process consumes
to pace its runs. The follower self-calibrates from a vendored tuning artifact
on the first path (feedforward so commanded == achieved, curvature speed
regulation, measured yaw-rate clamp) and runs forward-only (the Go2 lidar faces
forward; no strafe/reverse off the path).

Run (one of two processes; the benchmark is the other)::

    dimos run unitree-go2-rpp-controller
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_go2_joints = make_twist_base_joints("go2")


unitree_go2_rpp_controller = (
    autoconnect(
        GO2Connection.blueprint(),
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
                # Sole set_path/set_speed responder. Self-calibrates from the
                # vendored artifact on the first path; pursuit knobs are the
                # CTE-tuned defaults; forward_only for the forward-facing lidar.
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
        # Manual override for positioning the robot between runs + the per-run
        # gate the benchmark consumes. publish_only_when_active=True => emits a
        # Twist only while a key is held, so it coexists with the follower.
        KeyboardTeleop.blueprint(publish_only_when_active=True),
    )
    .remappings(
        [
            (GO2Connection, "cmd_vel", "go2_cmd_vel"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    )
    .transports(
        {
            # External command interface: teleop publishes on /cmd_vel; the
            # coordinator's twist_command reads it and echoes cmd_vel back.
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            # Base adapter <-> GO2Connection link (drives the robot, reads odom).
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            # Controller IN: planned path + target speed (from any source).
            ("path", Path): LCMTransport("/path", Path),
            ("speed", Float32): LCMTransport("/speed", Float32),
            # Operator gate (teleop -> benchmark) to pace runs.
            ("gate", Int8): LCMTransport("/benchmark/gate", Int8),
            # Aggregated joint state for observability (positions = [x,y,yaw]).
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("coordinator_joint_state", JointState): LCMTransport(
                "/coordinator/joint_state", JointState
            ),
        }
    )
    .global_config(obstacle_avoidance=False)
)
