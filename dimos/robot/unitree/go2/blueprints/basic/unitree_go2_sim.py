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

"""Minimal cmd_vel-driven Go2 simulation backed by PimSim."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.sim_connect import Go2Sim
from dimos.visualization.vis_module import vis_module

unitree_go2_sim = autoconnect(
    Go2Sim.blueprint().remappings([(Go2Sim, "cmd_vel", "tele_cmd_vel")]),
    vis_module(viewer_backend="rerun"),
)
