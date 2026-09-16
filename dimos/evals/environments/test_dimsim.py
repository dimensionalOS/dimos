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

from unittest.mock import Mock

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.dimsim import DimSimEnvironment
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def test_dimsim_launch_setup_and_pose(mocker):
    client = mocker.patch("dimos.evals.environments.dimsim.DimSimClient").return_value
    setup = Mock()
    env = DimSimEnvironment(blueprint=["unitree-go2"], scene="empty", setup=setup)
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator == "dimsim"
    assert proc.global_args == ["--dimsim-scene", "empty"]
    try:
        env.setup_scene()
        client.start.assert_called_once()
        setup.assert_called_once_with(client)
        with MemoryStore() as store:
            with pytest.raises(LookupError):
                env.latest_pose(store)
            pose = PoseStamped(ts=123, frame_id="world")
            store.stream("odom", PoseStamped).append(pose)
            assert env.latest_pose(store).ts == 123
    finally:
        env.stop()
    client.stop.assert_called_once()
