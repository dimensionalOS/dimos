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

"""Bounded UDP forwarding for a private gateway to a loopback QUIC relay."""

import asyncio
import logging
import socket
from contextlib import suppress
from dataclasses import dataclass
from ipaddress import ip_address
from typing import cast

logger = logging.getLogger(__name__)
Address = tuple[str, int]


@dataclass
class Peer:
    socket: socket.socket
    last_seen: float


class UdpForwarder(asyncio.DatagramProtocol):
    """Preserve a distinct upstream UDP connection for each browser endpoint."""

    def __init__(self, max_peers: int = 64, idle_seconds: float = 60.0) -> None:
        self.max_peers = max_peers
        self.idle_seconds = idle_seconds
        self.peers: dict[Address, Peer] = {}
        self.target: Address | None = None
        self.transport: asyncio.DatagramTransport | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._reaper: asyncio.Task[None] | None = None

    async def start(self, host: str, port: int) -> None:
        self._loop = asyncio.get_running_loop()
        await self._loop.create_datagram_endpoint(lambda: self, local_addr=(host, port))
        self._reaper = asyncio.create_task(self._expire_peers())

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = cast(asyncio.DatagramTransport, transport)

    def set_target(self, target: Address) -> None:
        if not ip_address(target[0]).is_loopback or not 1 <= target[1] <= 65535:
            raise ValueError("UDP upstream must be a loopback address and valid port")
        if target != self.target:
            for addr in tuple(self.peers):
                self._remove(addr)
            self.target = target

    def datagram_received(self, data: bytes, addr: Address) -> None:
        if self.target is None or self._loop is None:
            return
        peer = self.peers.get(addr)
        if peer is None:
            if len(self.peers) >= self.max_peers:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setblocking(False)
                sock.connect(self.target)
                self._loop.add_reader(sock.fileno(), self._reply, addr)
            except OSError:
                sock.close()
                logger.exception("Could not open UDP upstream")
                return
            peer = Peer(sock, self._loop.time())
            self.peers[addr] = peer
        peer.last_seen = self._loop.time()
        try:
            peer.socket.send(data)
        except BlockingIOError:
            pass  # UDP is lossy; never queue stale control/video packets.
        except OSError:
            self._remove(addr)

    def _reply(self, addr: Address) -> None:
        peer = self.peers.get(addr)
        if peer is None or self.transport is None:
            return
        try:
            data = peer.socket.recv(65535)
        except BlockingIOError:
            return
        except OSError:
            self._remove(addr)
            return
        self.transport.sendto(data, addr)

    def _remove(self, addr: Address) -> None:
        peer = self.peers.pop(addr)
        assert self._loop is not None
        self._loop.remove_reader(peer.socket.fileno())
        peer.socket.close()

    async def _expire_peers(self) -> None:
        assert self._loop is not None
        while True:
            await asyncio.sleep(min(5.0, self.idle_seconds))
            cutoff = self._loop.time() - self.idle_seconds
            for addr, peer in tuple(self.peers.items()):
                if peer.last_seen < cutoff:
                    self._remove(addr)

    async def stop(self) -> None:
        if self._reaper is not None:
            self._reaper.cancel()
            with suppress(asyncio.CancelledError):
                await self._reaper
            self._reaper = None
        for addr in tuple(self.peers):
            self._remove(addr)
        if self.transport is not None:
            self.transport.close()
            self.transport = None
