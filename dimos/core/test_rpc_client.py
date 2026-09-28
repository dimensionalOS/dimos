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
from pytest_mock import MockerFixture
from traitlets.config import Config

from dimos.core.core import rpc
from dimos.core.demos.stress_test_module import StressTestModule
from dimos.core.module import Module
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


class VariadicModule(Module):
    @rpc
    def named(self, a: int, b: int = 0) -> int:
        return a + b

    @rpc
    def star(self, *args: int) -> list[int]:
        return list(args)

    @rpc
    def keyed(self, value: int, **options: int) -> dict[str, int]:
        return {"value": value, **options}

    @rpc
    def only(self, a: int, /, b: int = 0, c: int = 0, **options: int) -> dict[str, int]:
        return {"a": a, "b": b, "c": c, **options}


def test_named_params_bind_by_name_and_keep_variadic_positionals(mocker: MockerFixture) -> None:
    transport = mocker.Mock(spec=RPCSpec, named_params=True)
    transport.call_sync.return_value = (None, lambda: None)
    client = RPCClient(None, VariadicModule, rpc=transport)

    client.named(1, b=2)
    client.star()
    client.star(1, 2)
    client.keyed(1, timeout=2)
    client.only(1, b=2)

    assert [call.args for call in transport.call_sync.call_args_list] == [
        ("VariadicModule/named", ([], {"a": 1, "b": 2})),
        ("VariadicModule/star", ([], {})),
        ("VariadicModule/star", ([1, 2], {})),
        ("VariadicModule/keyed", ([], {"value": 1, "timeout": 2})),
        ("VariadicModule/only", ([1, 2], {})),
    ]


@pytest.mark.parametrize("kwargs", [{"c": 2}, {"a": 2}])
def test_named_params_reject_mixed_positional_only_calls(
    mocker: MockerFixture, kwargs: dict[str, int]
) -> None:
    transport = mocker.Mock(spec=RPCSpec, named_params=True)
    client = RPCClient(None, VariadicModule, rpc=transport)

    with pytest.raises(TypeError, match="cannot mix positional and named arguments"):
        client.only(1, **kwargs)
