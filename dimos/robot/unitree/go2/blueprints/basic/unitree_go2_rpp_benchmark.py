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

"""Unitree Go2 RPP benchmark — controller + benchmark in ONE launchable.

A dimos blueprint is a composition of Modules that each run as their own worker
and communicate over the transport. So "two independently-launchable entities
that talk only over LCM" is realized as ONE ``dimos run`` that deploys both
module-sets — the controller (``unitree-go2-rpp-controller``: GO2Connection +
ControlCoordinator + KeyboardTeleop) and the ``Benchmarker``. They are still
fully decoupled: the Benchmarker imports nothing from the controller and only
talks to it over LCM topics —

    Benchmarker.path  -> /path           -> coordinator arms the follower
    Benchmarker.speed -> /speed          -> coordinator retunes the follower
    Benchmarker.odom  <- /go2/odom       -- the executed pose
    Benchmarker.cmd_vel <- /cmd_vel      -- the command sent to the robot
    Benchmarker.gate  <- /benchmark/gate -- operator advance/skip/quit (teleop)

(Two separate ``dimos run`` processes can't share an LCM bus — the runtime's
``Coordinator`` RPC service is a per-bus singleton — so one blueprint is the
correct dimos shape. The Benchmarker stays controller-agnostic: point it at a
different controller just by composing a different one here.)

Run (one terminal). Focus the pygame window: WASD to position the robot, then
ENTER to start each run (K=skip, Backspace=quit). Completion is detected from
odom automatically::

    dimos run unitree-go2-rpp-benchmark
    # afterwards, score offline:
    python -m dimos.control.benchmarking.score data/benchmark/go2

To drive paths from some OTHER source instead of the benchmark, launch the
controller alone: ``dimos run unitree-go2-rpp-controller``.
"""

from __future__ import annotations

from typing import Any

from dimos.control.benchmarking.benchmark import Benchmarker
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.stream import Transport
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_rpp_controller import (
    unitree_go2_rpp_controller,
)

# The Benchmarker's ``odom`` In must read the same topic the controller emits leg
# odom on (/go2/odom). path/speed/cmd_vel/gate already share names+topics with
# the controller blueprint, so they wire up by the controller's transports.
_BENCHMARK_TRANSPORTS: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
    ("odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
}


# NOTE: the public blueprint var ends in a chained ``.global_config(...)`` (a
# recognized blueprint method) so the AST-based all_blueprints generator
# discovers it. A bare ``X = autoconnect(...)`` is invisible to the generator.
unitree_go2_rpp_benchmark = (
    autoconnect(
        unitree_go2_rpp_controller,
        Benchmarker.blueprint(robot="go2", gate_source="stream"),
    )
    .transports(_BENCHMARK_TRANSPORTS)
    .global_config(obstacle_avoidance=False, n_workers=6)
)
