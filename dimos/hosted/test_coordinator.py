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

from dimos.core.coordination.module_coordinator import ModuleDescriptor
from dimos.hosted.coordinator import HostedCoordinator
from dimos.protocol.rpc.spec import RPCSpec


def test_introspection_combines_hosts_and_preserves_module_rpc_names(mocker):
    rpc = mocker.Mock(spec=RPCSpec)
    first = ModuleDescriptor("Camera", "camera.Camera", [], "runs/demo/hosts/a/modules/Camera")
    second = ModuleDescriptor("Leader", "leader.Leader", [], "runs/demo/hosts/b/modules/Leader")
    rpc.call_sync.side_effect = [([first], None), ([second], None)]
    stop = mocker.Mock()
    coordinator = HostedCoordinator(rpc, "demo", ["b", "a", "b"], stop)

    assert coordinator.list_modules() == [first, second]
    assert [call.args[0] for call in rpc.call_sync.call_args_list] == [
        "runs/demo/hosts/a/coordinator/list_modules",
        "runs/demo/hosts/b/coordinator/list_modules",
    ]
    coordinator.rpcs["shutdown"]()
    stop.assert_called_once_with()
