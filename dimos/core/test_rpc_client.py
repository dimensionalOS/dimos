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

import atexit

from IPython.core.completer import provisionalcompleter
from IPython.core.interactiveshell import InteractiveShell
import pytest
from traitlets.config import Config

from dimos.core.demos.stress_test_module import StressTestModule
from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.spec import RPCSpec


@pytest.fixture
def proxy(mocker):
    transport = mocker.Mock(spec=RPCSpec)
    return RPCClient(None, StressTestModule, rpc=transport), transport


@pytest.fixture
def ipython(proxy, tmp_path):
    client, _ = proxy
    shell = InteractiveShell(
        user_ns={"motion": client},
        ipython_dir=str(tmp_path),
        config=Config({"HistoryManager": {"enabled": False}}),
    )
    try:
        yield shell
    finally:
        shell.cleanup()
        atexit.unregister(shell.atexit_operations)
        shell.atexit_operations()


def test_dir_exposes_rpcs_and_proxy_attributes_without_transport_calls(proxy):
    client, transport = proxy

    names = dir(client)

    assert set(StressTestModule.rpcs) <= set(names)
    assert {"remote_name", "stop_rpc_client"} <= set(names)
    assert transport.mock_calls == []


def test_ipython_completes_rpc_without_transport_calls(ipython, proxy):
    _, transport = proxy

    with provisionalcompleter():
        completions = list(ipython.Completer.completions("motion.ec", len("motion.ec")))

    assert "echo" in {completion.text for completion in completions}
    assert transport.mock_calls == []
