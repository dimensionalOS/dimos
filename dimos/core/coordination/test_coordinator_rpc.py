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


def test_zenoh_connect_dials_the_seed_advertised_by_the_latest_run(mocker) -> None:
    rpc = mocker.Mock()
    rpc.call_sync.return_value = ("pong", mocker.Mock())
    zenoh_rpc = mocker.Mock(return_value=rpc)
    mocker.patch("dimos.core.coordination.coordinator_rpc.ZenohRPC", zenoh_rpc)
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.rpc_backend",
        return_value=zenoh_rpc,
    )
    entry = mocker.Mock(zenoh_peer_endpoint="tcp/127.0.0.1:51234")
    mocker.patch(
        "dimos.core.coordination.coordinator_rpc.get_most_recent",
        return_value=entry,
    )

    client = CoordinatorRPC.connect(timeout=1.0)

    assert client.rpc is rpc
    zenoh_rpc.assert_called_once_with(
        mode="peer",
        connect=["tcp/127.0.0.1:51234"],
        listen=["tcp/127.0.0.1:0"],
        multicast=False,
        gossip=True,
        connect_timeout=1.0,
    )
