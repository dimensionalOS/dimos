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

"""Unitree Go2 holonomic full-pose benchmark — controller + benchmark in ONE
launchable.

The full-pose counterpart to ``unitree-go2-rpp-benchmark``: the same decoupled
pub/sub shape (the Benchmarker imports nothing from the controller and talks
only over LCM), composed with the holonomic controller and the ``fullpose``
battery — paths whose commanded yaw is decoupled from the travel direction
(translate-while-rotating, strafe, fixed tangent offset). Offline scoring
reports ``heading_err_rms`` against the COMMANDED yaw alongside cte, so pose
tracking (not tangent alignment) is what gets measured.

Run (one terminal). Focus the pygame window: WASD to position the robot, then
ENTER to start each run (K=skip, Backspace=quit). Completion is detected from
odom automatically::

    dimos run unitree-go2-holonomic-benchmark
    # afterwards, score offline (the benchmark logs the recordings dir on start):
    python -m dimos.control.benchmarking.score <recordings-dir>
"""

from __future__ import annotations

from typing import Any

from dimos.control.benchmarking.benchmark import Benchmarker
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.stream import Transport
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_holonomic_controller import (
    unitree_go2_holonomic_controller,
)

# The Benchmarker's ``odom`` In must read the same topic the controller emits
# leg odom on (/go2/odom). path/speed/cmd_vel/operator_command already share names+topics
# with the controller blueprint, so they wire up by the controller's transports.
_BENCHMARK_TRANSPORTS: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
    ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
}


# NOTE: the public blueprint var ends in a chained ``.global_config(...)`` (a
# recognized blueprint method) so the AST-based all_blueprints generator
# discovers it. A bare ``X = autoconnect(...)`` is invisible to the generator.
unitree_go2_holonomic_benchmark = (
    autoconnect(
        unitree_go2_holonomic_controller,
        # "all" = the tangent-heading geometry (square, rounded_square, circle,
        # ...) PLUS the decoupled-yaw full-pose cases. Use K (skip) at the gate
        # to trim a session; battery="fullpose" runs just the decoupled cases.
        Benchmarker.blueprint(robot="go2", battery="all", gate_source="stream"),
    )
    # Record the command the robot actually receives (/go2/cmd_vel, written by
    # the coordinator's base adapter) — NOT /cmd_vel, which only carries the
    # operator's teleop nudges.
    .remappings([(Benchmarker, "cmd_vel", "go2_cmd_vel")])
    .transports(_BENCHMARK_TRANSPORTS)
    .global_config(obstacle_avoidance=False, n_workers=6)
)
