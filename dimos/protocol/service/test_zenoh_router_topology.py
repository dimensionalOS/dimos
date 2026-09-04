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

"""Live sessions in all three modes, forwarding through a router."""

from __future__ import annotations

import socket
import time
from typing import Any

import pytest

from dimos.protocol.service.zenohservice import ZenohService, ZenohSessionPool


def _free_endpoint() -> str:
    """A loopback endpoint nothing holds. A fixed port collides between runs."""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{sock.getsockname()[1]}"


_KEY = "dimos_test/router_topology"


@pytest.fixture
def pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


@pytest.fixture
def router_endpoint() -> str:
    return _free_endpoint()


def _service(pool: ZenohSessionPool, **config: Any) -> ZenohService:
    """A started service with discovery off, so only the dialed links exist."""
    service = ZenohService(session_pool=pool, multicast=False, gossip=False, **config)
    service.start()
    return service


def test_a_client_and_a_peer_talk_through_a_router(zenoh_defaults, pool, router_endpoint) -> None:
    router = _service(pool, mode="router", listen=[router_endpoint], connect=[])
    publisher = _service(pool, mode="client", connect=[router_endpoint])
    subscriber = _service(pool, mode="peer", connect=[router_endpoint])
    assert len({id(router.session), id(publisher.session), id(subscriber.session)}) == 3

    # The client must see the hub as a router, not merely as a linked peer.
    router_zids = [str(zid) for zid in publisher.session.info.routers_zid()]
    assert router_zids == [str(router.session.info.zid())]

    received: list[bytes] = []
    subscriber.session.declare_subscriber(
        _KEY, lambda sample: received.append(sample.payload.to_bytes())
    )

    # Republish until the subscriber's declaration has reached the publisher.
    deadline = time.monotonic() + 10.0
    while not received and time.monotonic() < deadline:
        publisher.session.put(_KEY, b"through the router")
        time.sleep(0.05)

    assert set(received) == {b"through the router"}


def test_a_client_without_a_router_fails_to_start(zenoh_defaults, pool) -> None:
    """Nothing listens on this port, and a client has no other way in."""
    with pytest.raises(RuntimeError, match="needs a reachable router"):
        _service(pool, mode="client", connect=["tcp/127.0.0.1:17453"], connect_timeout=0.5)
