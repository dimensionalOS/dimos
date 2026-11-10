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

"""Unitree Go2 holonomic full-pose controller — a standalone control process.

The strafe-capable counterpart to ``unitree-go2-rpp-controller``: the same
GO2Connection + ControlCoordinator + KeyboardTeleop shell, but the follower is
the progress-indexed HOLONOMIC full-pose tracker. It consumes a poses-only Path
whose per-waypoint orientation is the planner's COMMANDED heading — possibly
decoupled from the travel direction (face along a tunnel while strafing
through it) — and tracks position and yaw simultaneously. RPP cannot do this
(it faces the travel direction); conversely, for position-only paths where
tangent heading is wanted, use the RPP controller instead. Never run both
followers in one coordinator — they would fight over the base joints.

Interface (pure LCM pub/sub, identical to the RPP controller):

    IN   path   (nav_msgs/Path)        -> coordinator.path  -> holonomic_follower.on_path
    IN   speed  (std_msgs/Float32, m/s)-> coordinator.speed -> holonomic_follower.on_speed
    OUT  odom   (geometry_msgs/PoseStamped, /go2/odom)  -- the Go2 leg odom
    IN   cmd_vel(geometry_msgs/Twist,        /cmd_vel)  -- operator body-frame intent

The priority-20 ``vel_go2`` teleop task preempts the priority-10 follower while
a WASD key is held (repositioning between runs); KeyboardTeleop also emits the
per-run gate on ``/benchmark/gate``. The follower self-calibrates from the
vendored pose-domain artifact on the first path (per-axis P gains from the
plant fit, feedforward gain inversion, measured envelope caps).

Run the controller standalone to drive paths from any source on ``/path``::

    dimos run unitree-go2-holonomic-controller

To pace runs with the built-in battery instead, run
``unitree-go2-holonomic-benchmark``, which composes this controller with the
Benchmarker in one process.
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig as JointConnectionConfig
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.go2.connection import GO2_BASE_JOINTS, ConnectionConfig, GO2Connection
from dimos.robot.unitree.go2.coordinator import Go2Coordinator
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_go2_joints = list(GO2_BASE_JOINTS)


class _Go2HolonomicCoordinator(Go2Coordinator):
    """Holonomic deployment with canonical joint I/O."""


unitree_go2_holonomic_controller = (
    autoconnect(
        # velocity_api=True routes cmd_vel through the SPORT_MOD ``Move`` API
        # (real m/s and rad/s) instead of the default WIRELESS_CONTROLLER
        # joystick emulation (normalized stick deflections). The follower's
        # calibration artifact was characterized against the velocity API, so
        # under the default the commanded and achieved speeds disagree — the
        # robot runs hot and glides past the goal.
        GO2Connection.blueprint(obstacle_avoidance=False, velocity_api=True),
        _Go2HolonomicCoordinator.blueprint(
            instance_name="ControlCoordinator",
            publish_joint_state=True,
            publish_robot_joint_states=True,
            connections=[
                JointConnectionConfig(
                    **ConnectionConfig().model_dump(include=set(JointConnectionConfig.model_fields))
                )
            ],
            tasks=[
                # Idle/teleop velocity task. Priority 20 so a held WASD key
                # preempts the priority-10 follower for repositioning.
                TaskConfig(
                    name="vel_go2",
                    type="twist_velocity",
                    joint_names=_go2_joints,
                    priority=20,
                    params={
                        "axes": ["linear.x", "linear.y", "angular.z"],
                        "yaw_joint": GO2_BASE_JOINTS[2],
                        "zero_on_timeout": False,
                    },
                ),
                # Sole path/speed consumer: the progress-indexed
                # holonomic full-pose tracker. Self-calibrates from the
                # vendored pose-domain artifact on the first path.
                TaskConfig(
                    name="holonomic_follower",
                    type="holonomic_pose_follower",
                    joint_names=_go2_joints,
                    priority=10,
                    params={
                        "speed": 0.5,
                        "yaw_joint": GO2_BASE_JOINTS[2],
                        "lookahead": 0.25,
                        "goal_tolerance": 0.20,
                        "orientation_tolerance": 0.25,
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
            (GO2Connection, "velocity_command", "go2_velocity_command"),
            (_Go2HolonomicCoordinator, "go2_joint_state", "joint_state"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    )
    .transports(
        {
            # External command interface: teleop publishes on /cmd_vel; the
            # coordinator's twist_command reads it as operator intent.
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            # Preserve the public body-velocity RPC/stream and navigation odometry.
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            # Controller IN: planned full-pose path + target speed (any source).
            ("path", Path): LCMTransport("/path", Path),
            ("speed", Float32): LCMTransport("/speed", Float32),
            # Operator gate (teleop -> benchmark) to pace runs.
            ("operator_command", Int8): LCMTransport("/benchmark/gate", Int8),
            # This robot's joint state for observability (positions = [x,y,yaw]).
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
)
