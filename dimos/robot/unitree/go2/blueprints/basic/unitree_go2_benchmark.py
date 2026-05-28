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

"""Unitree Go2 operating-point benchmark — one-terminal HW flow.

Bundles GO2Connection + ControlCoordinator + pygame keyboard teleop +
the :class:`Benchmarker` module + a per-session telemetry recorder so
the operator runs a single command:

    dimos run unitree-go2-benchmark --module.benchmarker.config <artifact>

instead of the two-terminal CLI flow. Operator UX matches B1
(``unitree-go2-characterization``): WASD/QE in the pygame window to
reposition/aim the robot between runs; ENTER to advance, K to skip,
Backspace to quit.

Comparison arms are config flags (all default OFF — bare baseline):

    --module.benchmarker.ff=true       # apply derived feedforward
    --module.benchmarker.profile=true  # apply derived static velocity profile
    --module.benchmarker.rg=true \\
    --module.benchmarker.e_max=0.05    # apply RG-derived per-waypoint profile

The RG arm uses ``solve_profile()`` imported directly from
``reference_governor.py`` — there is no RG ``Module`` in this blueprint
because the per-cell math is one-shot (no live ``e_max`` stream to react
to), and a Module wrapper would force a cross-process per-tick RPC on
the controller's hot path. The Benchmarker computes the per-waypoint
speeds once per path and ships them to the follower as a plain
``list[float]`` via the new ``BaselinePathFollowerTask.start_path(
velocity_profile=...)`` kwarg.

Recordings land at
``<repo>/data/benchmark/<robot_id>/<robot_id>_benchmark_<date>_<sha>.db``
(tag="benchmark" so they don't collide with characterization recordings).

For ``--mode sim`` (FOPDT plant pre-check, no robot), keep using the
CLI entrypoint against ``coordinator-sim-fopdt``.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
    unitree_go2_coordinator,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.benchmarking.benchmark import Benchmarker
from dimos.utils.benchmarking.characterization_recorder import CharacterizationRecorder
from dimos.utils.path_utils import get_project_root

unitree_go2_benchmark = autoconnect(
    unitree_go2_coordinator,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
    Benchmarker.blueprint(robot="go2", mode="hw", gate_source="stream"),
    CharacterizationRecorder.blueprint(
        robot_id="go2",
        tag="benchmark",
        out_dir=str(get_project_root() / "data" / "benchmark" / "go2"),
    ),
).transports(
    {
        # Operator gate events from the pygame window -> Benchmarker.
        # Distinct topic from B1 (`/characterizer/gate`) so the two
        # blueprints don't cross-talk if both happen to run on the same
        # LCM bus.
        ("gate", Int8): LCMTransport("/benchmark/gate", Int8),
        # Recorder taps the same LCM topics the rest of the stack
        # already uses; no new wires, just additional subscribers.
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
    }
)

__all__ = ["unitree_go2_benchmark"]
