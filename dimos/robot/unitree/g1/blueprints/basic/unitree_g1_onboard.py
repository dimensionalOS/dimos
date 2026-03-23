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

"""Minimal G1 stack without navigation: effectors + mapping + visualization."""

from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.primitive._mapper import _mapper
from dimos.robot.unitree.g1.blueprints.primitive._vis import _vis
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

unitree_g1_onboard = autoconnect(
    _vis,
    _mapper,
    WebsocketVisModule.blueprint(),
    G1HighLevelDdsSdk.blueprint(),
).global_config(n_workers=4, robot_model="unitree_g1")

__all__ = ["unitree_g1_onboard"]
