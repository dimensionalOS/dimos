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

import asyncio
from collections.abc import AsyncIterator
from typing import cast

import pytest
import pytest_asyncio
from microduck_world.udp_forwarder import UdpForwarder


class Echo(asyncio.DatagramProtocol):
    def __init__(self):
        self.incoming = asyncio.Queue()

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        self.incoming.put_nowait((data, addr))
        self.transport.sendto(data, addr)


class Inbox(asyncio.DatagramProtocol):
    def __init__(self):
        self.messages = asyncio.Queue()

    def datagram_received(self, data, addr):
        self.messages.put_nowait(data)


@pytest_asyncio.fixture
async def rig() -> AsyncIterator[tuple]:
    loop = asyncio.get_running_loop()
    echo = Echo()
    upstream, _ = await loop.create_datagram_endpoint(lambda: echo, local_addr=("127.0.0.1", 0))
    proxy = UdpForwarder(max_peers=2)
    clients = []
    try:
        await proxy.start("127.0.0.1", 0)
        proxy.set_target(upstream.get_extra_info("sockname"))
        target = proxy.transport.get_extra_info("sockname")
        for _ in range(2):
            inbox = Inbox()
            transport, _ = await loop.create_datagram_endpoint(lambda: inbox, remote_addr=target)
            clients.append((cast(asyncio.DatagramTransport, transport), inbox))
        yield proxy, echo, clients
    finally:
        for transport, _ in clients:
            transport.close()
        await proxy.stop()
        upstream.close()


@pytest.mark.asyncio
async def test_bidirectional_packets_keep_clients_separate(rig):
    _, echo, clients = rig
    clients[0][0].sendto(b"first browser")
    clients[1][0].sendto(b"second browser")
    assert await asyncio.wait_for(clients[0][1].messages.get(), 2) == b"first browser"
    assert await asyncio.wait_for(clients[1][1].messages.get(), 2) == b"second browser"
    _, first_addr = await asyncio.wait_for(echo.incoming.get(), 2)
    _, second_addr = await asyncio.wait_for(echo.incoming.get(), 2)
    assert first_addr != second_addr


@pytest.mark.asyncio
async def test_peer_limit_and_relay_restart_cleanup(rig):
    proxy, _, clients = rig
    for transport, inbox in clients:
        transport.sendto(b"connect")
        assert await asyncio.wait_for(inbox.messages.get(), 2) == b"connect"
    proxy.datagram_received(b"over capacity", ("127.0.0.1", 12345))
    assert ("127.0.0.1", 12345) not in proxy.peers
    upstream_sockets = [peer.socket for peer in proxy.peers.values()]
    new_port = 12346 if proxy.target[1] != 12346 else 12347
    proxy.set_target(("127.0.0.1", new_port))
    assert proxy.peers == {}
    assert [sock.fileno() for sock in upstream_sockets] == [-1, -1]


def test_non_loopback_upstream_is_rejected():
    proxy = UdpForwarder()
    with pytest.raises(ValueError, match="loopback"):
        proxy.set_target(("203.0.113.1", 443))
