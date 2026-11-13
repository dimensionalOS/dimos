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

from types import SimpleNamespace

from pytest_mock import MockerFixture

from dimos.core.module import Module
from dimos.hosted.client import HostClient, describe_all, discover_host_ids
from dimos.hosted.fragment import HostFragment, PythonFragmentPayload
from dimos.hosted.protocol import DeploymentStatus, HostDescriptor


class ClientTestModule(Module):
    pass


def _descriptor() -> HostDescriptor:
    return HostDescriptor(
        host_id="host-1",
        name="compute",
        tags={"compute": ""},
        revision="revision-1",
        active_run_ids=(),
    )


def _fragment() -> HostFragment:
    blueprint = ClientTestModule.blueprint()
    return HostFragment.create(
        run_id="run-1",
        host_id="host-1",
        revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint),
    )


def test_discover_host_ids_uses_only_valid_live_keys(mocker: MockerFixture) -> None:
    rpc = mocker.MagicMock()
    rpc.session.liveliness.return_value.get.return_value = (
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-b/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-a/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-a/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-c/other")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/x/y/live")),
        SimpleNamespace(ok=None),
    )

    assert discover_host_ids(rpc, timeout=0.5) == ("host-a", "host-b")
    rpc.session.liveliness.return_value.get.assert_called_once_with(
        "dimos/hosts/*/live", timeout=0.5
    )


def test_describe_unsubscribes_after_validating_response(mocker: MockerFixture) -> None:
    rpc = mocker.MagicMock()
    unsubscribe = mocker.Mock()
    rpc.call_sync.return_value = (_descriptor(), unsubscribe)

    client = HostClient(rpc, "host-1", timeout=1.0)

    assert client.describe() == _descriptor()
    rpc.call_sync.assert_called_once_with("hosts/host-1/describe", ([], {}), rpc_timeout=1.0)
    unsubscribe.assert_called_once_with()


def test_host_client_sends_lifecycle_calls(mocker: MockerFixture) -> None:
    rpc = mocker.MagicMock()
    unsubscribe = mocker.Mock()
    running = DeploymentStatus("running", "run-1", 123, "/logs", None)
    stopped = DeploymentStatus("absent", None, None, None, None)
    rpc.call_sync.side_effect = (
        (running, unsubscribe),
        (running, unsubscribe),
        (stopped, unsubscribe),
    )
    fragment = _fragment()
    client = HostClient(rpc, "host-1", timeout=2.0)

    assert client.start(fragment) == running
    assert client.status("run-1") == running
    assert client.stop("run-1") == stopped
    assert rpc.call_sync.call_args_list == [
        mocker.call("hosts/host-1/start", ([fragment], {}), rpc_timeout=2.0),
        mocker.call("hosts/host-1/status", (["run-1"], {}), rpc_timeout=2.0),
        mocker.call("hosts/host-1/stop", (["run-1"], {}), rpc_timeout=2.0),
    ]
    assert unsubscribe.call_count == 3


def test_describe_all_keeps_the_round_when_one_host_does_not_answer(
    mocker: MockerFixture,
) -> None:
    rpc = mocker.MagicMock()
    mocker.patch("dimos.hosted.client.discover_host_ids", return_value=("host-1", "host-2"))

    def describe(name: str, *_: object, **__: object) -> tuple[object, object]:
        if name.startswith("hosts/host-2"):
            raise TimeoutError("busy")
        return _descriptor(), mocker.Mock()

    rpc.call_sync.side_effect = describe
    assert [h.host_id for h in describe_all(rpc, timeout=0.5)] == ["host-1"]
