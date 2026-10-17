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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import Module
from dimos.hosted.client import HostClient, discover_host_ids, get_host_descriptor
from dimos.hosted.daemon import DeploymentStatus, HostDescriptor
from dimos.hosted.fragment import HostFragment, PythonFragmentPayload


class ClientTestModule(Module):
    pass


def _descriptor() -> HostDescriptor:
    return HostDescriptor(
        host_id="host-1",
        epoch="epoch-1",
        name="compute",
        tags=frozenset({"compute"}),
        versions={"dimos": "revision-1"},
        state="available",
        active_run_ids=(),
    )


def _fragment() -> HostFragment:
    blueprint = ClientTestModule.blueprint()
    config = BlueprintConfigParser(blueprint).parse(environ={})
    return HostFragment.create(
        run_id="run-1",
        generation=1,
        host_id="host-1",
        application_name="client-test",
        application_revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint, config=config),
    )


def test_discover_host_ids_uses_only_valid_live_keys(mocker: MockerFixture) -> None:
    rpc = mocker.MagicMock()
    rpc.session.liveliness.return_value.get.return_value = (
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-b/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-a/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="dimos/hosts/host-a/live")),
        SimpleNamespace(ok=SimpleNamespace(key_expr="unrelated/key")),
        SimpleNamespace(ok=None),
    )

    host_ids = discover_host_ids(rpc, timeout=0.5)

    assert host_ids == ("host-a", "host-b")
    rpc.session.liveliness.return_value.get.assert_called_once_with(
        "dimos/hosts/*/live", timeout=0.5
    )


def test_get_host_descriptor_unsubscribes_after_validating_response(
    mocker: MockerFixture,
) -> None:
    rpc = mocker.MagicMock()
    unsubscribe = mocker.Mock()
    rpc.call_sync.return_value = (_descriptor(), unsubscribe)

    result = get_host_descriptor(rpc, "host-1", timeout=1.0)

    assert result == _descriptor()
    rpc.call_sync.assert_called_once_with("hosts/host-1/describe", ([], {}), rpc_timeout=1.0)
    unsubscribe.assert_called_once_with()


def test_host_client_sends_identity_bound_lifecycle_calls(mocker: MockerFixture) -> None:
    rpc = mocker.MagicMock()
    unsubscribe = mocker.Mock()
    running = DeploymentStatus("running", "run-1", 1, 123, "/logs", None)
    stopped = DeploymentStatus("available", None, None, None, None, None)
    rpc.call_sync.side_effect = (
        (running, unsubscribe),
        (running, unsubscribe),
        (stopped, unsubscribe),
    )
    fragment = _fragment()
    client = HostClient(rpc, _descriptor(), timeout=2.0)

    assert client.start(fragment) == running
    assert client.status("run-1") == running
    assert client.stop(fragment) == stopped
    assert rpc.call_sync.call_args_list == [
        mocker.call(
            "hosts/host-1/start",
            (["epoch-1", fragment], {}),
            rpc_timeout=2.0,
        ),
        mocker.call(
            "hosts/host-1/status",
            (["epoch-1", "run-1"], {}),
            rpc_timeout=2.0,
        ),
        mocker.call(
            "hosts/host-1/stop",
            (["epoch-1", "run-1", 1, fragment.payload_digest], {}),
            rpc_timeout=2.0,
        ),
    ]
    assert unsubscribe.call_count == 3
