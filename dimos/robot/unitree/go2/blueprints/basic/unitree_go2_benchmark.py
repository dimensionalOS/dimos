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

instead of the two-terminal CLI flow. Defaults are the bare baseline
arm (ff/profile/rg all OFF) — use a sibling blueprint
(``unitree-go2-benchmark-rg``) to bake the RG arm in, rather than
overriding ``--module.benchmarker.rg`` at every invocation. Operator UX matches B1
(``unitree-go2-characterization``): WASD/QE in the pygame window to
reposition/aim the robot between runs; ENTER to advance, K to skip,
Backspace to quit.

Comparison arms are config flags (all default OFF — bare baseline):

    --module.benchmarker.ff=true       # apply derived feedforward
    --module.benchmarker.profile=true  # apply derived static velocity profile
    --module.benchmarker.rg=true       # route runs through precision_follower

The RG arm runs against the operator coord's ``precision_follower`` task
(a ``PathFollowerTask`` subclass that owns its own ``solve_profile()``
recompute and reacts to ``KeyboardTeleop``'s ``e_max`` stream — number
keys 0-9 set the corridor half-width live). No RG math inside Benchmarker;
it just picks the task name based on the ``rg`` flag.

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
from dimos.msgs.std_msgs.Float32 import Float32
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
        # KeyboardTeleop's number-key (0-9) corridor stream -> any task in
        # the coord with set_e_max(). No-ops for the bare arm (path_follower
        # ignores it), live for the RG arm (precision_follower recomputes).
        ("e_max", Float32): LCMTransport("/e_max", Float32),
        # Recorder taps the same LCM topics the rest of the stack
        # already uses; no new wires, just additional subscribers.
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
    }
)

__all__ = ["unitree_go2_benchmark"]
