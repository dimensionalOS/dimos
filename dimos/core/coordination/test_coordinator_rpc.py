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

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC


def test_connect_retries_when_coordinator_becomes_ready(mocker) -> None:
    rpc = mocker.Mock()
    rpc.call_sync.side_effect = [
        TimeoutError("coordinator not ready"),
        ("pong", mocker.Mock()),
    ]
    backend = mocker.Mock(return_value=rpc)
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.rpc_backend",
        return_value=backend,
    )
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.time.monotonic",
        side_effect=[10.0, 10.0, 10.25],
    )

    client = CoordinatorRPC.connect(timeout=1.0)

    assert client.rpc is rpc
    assert rpc.call_sync.call_count == 2
    assert [call.kwargs["rpc_timeout"] for call in rpc.call_sync.call_args_list] == [
        0.25,
        0.25,
    ]
    rpc.stop.assert_not_called()


def test_connect_stops_transport_after_retry_budget_expires(mocker) -> None:
    rpc = mocker.Mock()
    rpc.call_sync.side_effect = TimeoutError("coordinator not ready")
    backend = mocker.Mock(return_value=rpc)
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.rpc_backend",
        return_value=backend,
    )
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.time.monotonic",
        side_effect=[10.0, 10.0, 11.0],
    )

    with pytest.raises(TimeoutError, match="did not respond within 1.0 seconds"):
        CoordinatorRPC.connect(timeout=1.0)

    rpc.stop.assert_called_once_with()


def test_serve_and_connect_use_selected_rpc_name(mocker) -> None:
    server_rpc = mocker.Mock()
    client_rpc = mocker.Mock()
    client_rpc.call_sync.return_value = ("pong", mocker.Mock())
    backend = mocker.Mock(side_effect=[server_rpc, client_rpc])
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.rpc_backend",
        return_value=backend,
    )
    ensure = mocker.patch.object(CoordinatorRPC, "_ensure_no_existing_service")
    coordinator = mocker.Mock()
    name = "runs/run-1/hosts/host-a/coordinator"

    server = CoordinatorRPC.serve(coordinator, name=name)
    client = CoordinatorRPC.connect(timeout=1.0, name=name)
    result = client.call("ping")

    ensure.assert_called_once_with(name)
    server_rpc.serve_module_rpc.assert_called_once_with(coordinator, name=name)
    assert server.name == name
    assert client.name == name
    assert result == "pong"
    assert client_rpc.call_sync.call_args_list == [
        mocker.call(f"{name}/ping", ([], {}), rpc_timeout=0.25),
        mocker.call(f"{name}/ping", ([], {}), rpc_timeout=None),
    ]
