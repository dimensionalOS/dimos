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
from dimos.message_utils.time import time_from_seconds, to_seconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header


def test_dimsim_launch_setup_and_pose(mocker):
    client_cls = mocker.patch("dimos.evals.environments.dimsim.DimSimClient")
    client = client_cls.return_value
    setup = Mock()
    env = DimSimEnvironment(blueprint=["unitree-go2"], scene="empty", port=18090, setup=setup)
    proc = DimosCliCall()
    env.configure_launch(proc)
    assert proc.simulator == "dimsim"
    assert proc.extra_env["GO2CONNECTION__DIMSIM__SCENE"] == "empty"
    assert proc.dimsim_port == 18090
    try:
        env.setup_scene()
        client_cls.assert_called_once_with(18090)
        client.start.assert_called_once()
        setup.assert_called_once_with(client)
        with MemoryStore() as store:
            with pytest.raises(LookupError):
                env.latest_pose(store)
            pose = PoseStamped(
                header=Header(stamp=time_from_seconds(123), frame_id="world"),
                pose=Pose(
                    position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
                ),
            )
            store.stream("odom", PoseStamped).append(pose)
            assert to_seconds(env.latest_pose(store).header.stamp) == 123
    finally:
        env.stop()
    client.stop.assert_called_once()
