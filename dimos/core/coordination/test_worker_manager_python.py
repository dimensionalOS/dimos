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

from dimos.core.coordination.test_worker import SimpleModule
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig
from dimos.protocol.encode.cbor import decode, encode


@pytest.mark.parametrize("deployment", ["deploy", "deploy_parallel", "deploy_fresh"])
@pytest.mark.parametrize("instance_name", [None, "robot/counter"])
def test_runs_do_not_share_module_rpcs(each_transport, deployment, instance_name):
    config = GlobalConfig(n_workers=1, viewer="none", transport=each_transport)
    kwargs = {"instance_name": instance_name}
    with ExitStack() as cleanup:
        managers = [WorkerManagerPython(g=config), WorkerManagerPython(g=config)]
        proxies = []
        for manager in managers:
            cleanup.callback(manager.stop)
            if deployment == "deploy_parallel":
                proxy = manager.deploy_parallel([(SimpleModule, config, kwargs)])[0]
            else:
                proxy = getattr(manager, deployment)(SimpleModule, config, kwargs)
            cleanup.callback(proxy.stop_rpc_client)
            proxies.append(proxy)
        first, second = proxies

        assert first.rpc_address != second.rpc_address
        assert first.remote_name == second.remote_name == (instance_name or "SimpleModule")
        assert first.increment() == 1
        assert second.increment() == 1
        assert first.get_counter() == 1

        # Module references sent into other workers retain the private address.
        restored = decode(encode(second))
        cleanup.callback(restored.stop_rpc_client)
        assert restored.increment() == 2
        assert first.get_counter() == 1

        # A fire-and-forget stop from one run must leave the other reachable.
        second.stop()
        managers[1].stop()
        assert first.increment() == 2

    assert kwargs == {"instance_name": instance_name}
