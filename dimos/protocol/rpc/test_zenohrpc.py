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

"""ZenohRPC refuses a query that zenoh delivered to the wrong queryable."""

from collections.abc import Callable, Iterator
import pickle
from typing import Any, ClassVar

import pytest
import zenoh

from dimos.protocol.encode.cbor import decode, encode
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.zenohservice import ZenohSessionPool


class _Coordinator:
    """The two ControlCoordinator RPCs seen crossed on Alfred: stop's handler ran on task_invoke's arguments."""

    def __init__(self) -> None:
        self.stops = 0
        self.invokes: list[tuple[str, str, dict[str, Any]]] = []

    @property
    def rpcs(self) -> dict[str, Callable[..., Any]]:
        return {"stop": self.stop, "task_invoke": self.task_invoke}

    def stop(self) -> None:
        self.stops += 1

    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any]) -> float:
        self.invokes.append((task_name, method, kwargs))
        return 0.5


@pytest.fixture
def rpc_pair() -> Iterator[tuple[ZenohRPC, ZenohRPC]]:
    pool = ZenohSessionPool()
    server = ZenohRPC(rpc_timeouts={}, default_rpc_timeout=DEFAULT_RPC_TIMEOUT, session_pool=pool)
    client = ZenohRPC(rpc_timeouts={}, default_rpc_timeout=DEFAULT_RPC_TIMEOUT, session_pool=pool)
    server.start()
    client.start()
    try:
        yield server, client
    finally:
        server.stop()
        client.stop()
        pool.close_all()


def _deliver_task_invoke_to_stop(client: ZenohRPC, monkeypatch: pytest.MonkeyPatch) -> None:
    """Make zenoh hand every task_invoke query to the stop queryable, as it did on Alfred."""
    issue_query = client._issue_query

    def misdeliver(call_id: int, key: str, payload: bytes, deadline: float) -> None:
        issue_query(call_id, key.replace("/task_invoke", "/stop"), payload, deadline)

    monkeypatch.setattr(client, "_issue_query", misdeliver)


def test_call_reaches_its_own_handler(rpc_pair: tuple[ZenohRPC, ZenohRPC]) -> None:
    server, client = rpc_pair
    coordinator = _Coordinator()
    server.serve_module_rpc(coordinator, name="Coord")

    result, _ = client.call_sync(
        "Coord/task_invoke", (["left_arm_gripper", "get_normalized", {}], {}), rpc_timeout=5.0
    )

    assert result == 0.5
    assert coordinator.invokes == [("left_arm_gripper", "get_normalized", {})]
    assert coordinator.stops == 0


def test_misdelivered_call_is_refused_without_running_the_wrong_handler(
    rpc_pair: tuple[ZenohRPC, ZenohRPC], monkeypatch: pytest.MonkeyPatch
) -> None:
    server, client = rpc_pair
    coordinator = _Coordinator()
    server.serve_module_rpc(coordinator, name="Coord")
    _deliver_task_invoke_to_stop(client, monkeypatch)

    with pytest.raises(RuntimeError, match="RPC for Coord/task_invoke was delivered to Coord/stop"):
        client.call_sync(
            "Coord/task_invoke", (["left_arm_gripper", "get_normalized", {}], {}), rpc_timeout=5.0
        )

    assert coordinator.stops == 0
    assert coordinator.invokes == []


def _ask(client: ZenohRPC, payload: bytes) -> list[Any]:
    return list(
        client.session.get(
            "dimos/rpc/Coord/task_invoke",
            payload=payload,
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            timeout=5.0,
        )
    )


def test_a_controller_request_gets_a_pickled_reply(rpc_pair: tuple[ZenohRPC, ZenohRPC]) -> None:
    """dim-controller's Rust server still speaks pickle."""
    server, client = rpc_pair
    server.serve_module_rpc(_Coordinator(), name="Coord")

    replies = _ask(
        client, pickle.dumps((["right_arm_gripper", "get_normalized", {}], {}), protocol=4)
    )

    assert [pickle.loads(reply.ok.payload.to_bytes()) for reply in replies if reply.ok] == [0.5]


class _RunsCodeWhenRead:
    ran: ClassVar[list[str]] = []

    def __reduce__(self) -> tuple[Any, tuple[str]]:
        return (_RunsCodeWhenRead.ran.append, ("ran",))


def test_a_pickled_request_cannot_run_code(rpc_pair: tuple[ZenohRPC, ZenohRPC]) -> None:
    server, client = rpc_pair
    coordinator = _Coordinator()
    server.serve_module_rpc(coordinator, name="Coord")

    replies = _ask(client, pickle.dumps(([_RunsCodeWhenRead()], {}), protocol=4))

    assert _RunsCodeWhenRead.ran == []
    assert coordinator.invokes == []
    assert [reply for reply in replies if reply.ok] == []


def test_caller_without_a_name_is_still_served(rpc_pair: tuple[ZenohRPC, ZenohRPC]) -> None:
    """dim-controller's Rust server and older dimos send a bare (args, kwargs)."""
    server, client = rpc_pair
    coordinator = _Coordinator()
    server.serve_module_rpc(coordinator, name="Coord")

    replies = list(
        client.session.get(
            "dimos/rpc/Coord/task_invoke",
            payload=encode((["right_arm_gripper", "get_normalized", {}], {})),
            target=zenoh.QueryTarget.ALL,
            consolidation=zenoh.ConsolidationMode.NONE,
            timeout=5.0,
        )
    )

    assert [decode(reply.ok.payload.to_bytes()) for reply in replies if reply.ok] == [0.5]
    assert coordinator.invokes == [("right_arm_gripper", "get_normalized", {})]
