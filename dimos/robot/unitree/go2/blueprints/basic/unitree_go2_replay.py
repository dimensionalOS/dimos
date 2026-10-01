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

"""Replay a Go2 recording with the Go2 viewer layout, no robot needed.

``run replay`` picks the viewer layout from the recording's run folder name, so a dataset
under ``data/`` gets the generic layout. This blueprint always uses the Go2 one: camera,
odom, joystick, tele_cmd_vel and cmd_vel plots, robot body and camera frustum.

The dataset is resolved the way ``dimos.memory.blueprints`` does it: a bare name only in
the process running this blueprint, and exported as ``REPLAY_DB`` because workers import
this module with the default global config.

Usage:
    dimos --replay-db go2_teleop_sf_office_2026-09-18 run unitree-go2-replay
    dimos --replay-db recordings/<run-id>/memory.db run unitree-go2-replay
"""

import os
import sys

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.memory.replay_module import dataset_path, replay_module
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.visualization.vis_module import vis_module

_DATASET = dataset_path(global_config.replay_db, explicit="unitree-go2-replay" in sys.argv[1:])
if _DATASET:
    os.environ["REPLAY_DB"] = _DATASET

Go2Replay = replay_module(_DATASET, name="Go2Replay")

unitree_go2_replay = autoconnect(
    vis_module(global_config.viewer, rerun_config=rerun_config),
    Go2Replay.blueprint(dataset=_DATASET),
)
