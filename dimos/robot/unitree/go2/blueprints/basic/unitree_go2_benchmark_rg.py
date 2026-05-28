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

"""Unitree Go2 benchmark — RG arm variant.

Same composition as ``unitree-go2-benchmark`` but with ``rg=True`` baked
into the Benchmarker config. Runs are routed through the operator coord's
``precision_follower`` task (a ``PathFollowerTask`` subclass that owns its
own ``solve_profile()`` recompute reacting to ``KeyboardTeleop``'s
``e_max`` stream — number keys 0-9 set the corridor half-width live).

The precision_follower needs its own copy of the artifact path so it can
load the plant + velocity-profile constants:

    dimos run unitree-go2-benchmark-rg \\
        --module.benchmarker.config <artifact> \\
        -o coordinator.tasks[1].params.artifact_path=<artifact>
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
    unitree_go2_coordinator,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.benchmarking.benchmark import Benchmarker
from dimos.utils.benchmarking.characterization_recorder import CharacterizationRecorder
from dimos.utils.path_utils import get_project_root

unitree_go2_benchmark_rg = autoconnect(
    unitree_go2_coordinator,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
    Benchmarker.blueprint(robot="go2", mode="hw", gate_source="stream", rg=True),
    CharacterizationRecorder.blueprint(
        robot_id="go2",
        tag="benchmark",
        out_dir=str(get_project_root() / "data" / "benchmark" / "go2"),
    ),
).transports(
    {
        ("gate", Int8): LCMTransport("/benchmark/gate", Int8),
        # KeyboardTeleop's number-key (0-9) corridor stream -> coord's
        # e_max input -> precision_follower.set_e_max(). Live RG corridor.
        ("e_max", Float32): LCMTransport("/e_max", Float32),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
    }
)

__all__ = ["unitree_go2_benchmark_rg"]
