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

from contextlib import ExitStack

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig, global_config
from dimos.manipulation.control.coordinator_client import CoordinatorClient


@pytest.mark.parametrize("transport", ["zenoh", "lcm"])
@pytest.mark.parametrize("instance_name", [None, "arm/controller"])
def test_client_discovers_the_deployed_control_coordinator(transport, instance_name):
    global_config.update(transport=transport)
    config = GlobalConfig(n_workers=1, viewer="none", transport=transport)
    with ExitStack() as cleanup:
        coordinator = ModuleCoordinator(g=config)
        cleanup.callback(coordinator.stop)
        coordinator.start()
        module = coordinator.deploy(ControlCoordinator, instance_name=instance_name)
        coordinator.start_rpc_service()
        client = CoordinatorClient()
        cleanup.callback(client.stop)
        client._rpc.rpc.default_rpc_timeout = 5.0

        assert client.list_tasks() == []
        assert client.list_hardware() == []
        client.stop()
        assert module.list_tasks() == []


def test_missing_control_coordinator_closes_discovery_connection(mocker):
    source = mocker.patch(
        "dimos.manipulation.control.coordinator_client.RemoteModuleSource"
    ).return_value
    source.get_module.side_effect = KeyError("ControlCoordinator")

    with pytest.raises(KeyError, match="ControlCoordinator"):
        CoordinatorClient()

    source.close.assert_called_once_with()
