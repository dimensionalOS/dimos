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

import pytest

from dimos.cli.bake.cli import emit_config
from dimos.cli.bake.discovery import discover_modules, select_modules
from dimos.cli.bake.graph import build_graph
from dimos.navigation.motion.adapter.follower_native import TrajectoryFollowerNativeConfig
from dimos.navigation.motion.adapter.planner_native import MotionPlannerNativeConfig
from dimos.navigation.movement_manager.cmd_vel_mux_native import CmdVelMuxNativeConfig
from dimos.robot.unitree.go2.tf.go2_tf import Go2TfConfig
from dimos.robot.unitree.go2.zenoh.blueprints import MOTION_BODY_DILATE_M
from dimos.robot.unitree.go2.zenoh.motion_host import GO2_MOTION_HOST, MAX_SPEED

CONFIGS = {
    "motion_planner": MotionPlannerNativeConfig,
    "trajectory_follower": TrajectoryFollowerNativeConfig,
    "cmd_vel_mux": CmdVelMuxNativeConfig,
    "go2_tf": Go2TfConfig,
}


@pytest.fixture(scope="module")
def blob():
    """What `dimos bake --deployment ...:GO2_MOTION_HOST -o motion-host` embeds."""
    selected = select_modules(discover_modules(), GO2_MOTION_HOST.modules)
    graph = build_graph("motion-host", selected)
    return emit_config(graph, selected, GO2_MOTION_HOST.configs, GO2_MOTION_HOST.session)


def test_every_module_block_is_its_native_config_again(blob):
    """The drift that shipped a string embodiment: every block must re-validate."""
    assert set(blob["modules"]) == set(GO2_MOTION_HOST.modules) == set(CONFIGS)
    for module, config_type in CONFIGS.items():
        block = blob["modules"][module]["config"]
        assert config_type(**block).to_config_dict() == block


def test_the_deployment_overrides_land_in_both_halves(blob):
    for module in ("motion_planner", "trajectory_follower"):
        cfg = blob["modules"][module]["config"]
        assert cfg["body_dilate_m"] == MOTION_BODY_DILATE_M
        assert cfg["embodiment"]["max_speed"] == MAX_SPEED


def test_the_session_is_a_loopback_client_of_the_router(blob):
    assert blob["session"] == {
        "mode": "client",
        "connect": ["tcp/127.0.0.1:7447"],
        "listen": [],
        "multicast": False,
        "gossip": True,
        "interface": "lo",
        "connect_timeout_ms": 3000,
    }


def test_the_blob_is_stamped(blob):
    assert blob["graph"]
