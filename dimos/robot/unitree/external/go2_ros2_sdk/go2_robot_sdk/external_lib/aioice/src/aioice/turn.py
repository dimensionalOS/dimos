import asyncio
import hashlib
import logging
import socket
import struct
import time
from typing import (
    Any,
    Callable,
    Dict,
    List,
    Optional,
    Text,
    Tuple,
    TypeVar,
    Union,
    cast,
)

from . import stun
from .utils import random_transaction_id

logger = logging.getLogger(__name__)

DEFAULT_CHANNEL_REFRESH_TIME = 500
DEFAULT_ALLOCATION_LIFETIME = 600
TCP_TRANSPORT = 0x06000000
UDP_TRANSPORT = 0x11000000
UDP_SOCKET_BUFFER_SIZE = 262144

_ProtocolT = TypeVar("_ProtocolT", bound=asyncio.BaseProtocol)


def is_channel_data(data: bytes) -> bool:
    return (data[0] & 0xC0) == 0x40


def make_integrity_key(username: str, realm: str, password: str) -> bytes:
    return hashlib.md5(":".join([username, realm, password]).encode("utf8")).digest()


class TurnStreamMixin:
    datagram_received: Callable
    transport: asyncio.BaseTransport

    def data_received(self, data: bytes) -> None:
        if not hasattr(self, "buffer"):
            self.buffer = b""
        self.buffer += data

        while len(self.buffer) >= 4:
            _, length = struct.unpack("!HH", self.buffer[0:4])
            length += stun.padding_length(length)
            if is_channel_data(self.buffer):
                full_length = 4 + length
            else:
                full_length = 20 + length
            if len(self.buffer) < full_length:
                break

            addr = self.transport.get_extra_info("peername")
            self.datagram_received(self.buffer[0:full_length], addr)
            self.buffer = self.buffer[full_length:]

    def _padded(self, data: bytes) -> bytes:
        # TCP and TCP-over-TLS must pad messages to 4-byte boundaries.
        padding = stun.padding_length(len(data))
        if padding:
            data += bytes(padding)
        return data


class TurnClientMixin:
    _send: Callable

    def __init__(
        self,
        server: Tuple[str, int],
        username: Optional[str],
        password: Optional[str],
        lifetime: int,
        channel_refresh_time: int,
    ) -> None:
        self.channel_refresh_at: Dict[int, float] = {}
        self.channel_to_peer: Dict[int, Tuple[str, int]] = {}
        self.peer_connect_waiters: Dict[
            Tuple[str, int], List[asyncio.Future[None]]
        ] = {}
        self.peer_to_channel: Dict[Tuple[str, int], int] = {}

        self.channel_number = 0x4000
        self.channel_refresh_time = channel_refresh_time
        self.integrity_key: Optional[bytes] = None
        self.lifetime = lifetime
        self.nonce: Optional[bytes] = None
        self.password = password
        self.receiver = None
        self.realm: Optional[str] = None
        self.refresh_task: Optional[asyncio.Task] = None
        self.relayed_address: Optional[Tuple[str, int]] = None
        self.server = server
        self.transactions: Dict[bytes, stun.Transaction] = {}
        self.username = username

    async def channel_bind(self, channel_number: int, addr: Tuple[str, int]) -> None:
        request = stun.Message(
            message_method=stun.Method.CHANNEL_BIND, message_class=stun.Class.REQUEST
        )
        request.attributes["CHANNEL-NUMBER"] = channel_number
        request.attributes["XOR-PEER-ADDRESS"] = addr
        await self.request_with_retry(request)
        logger.info("TURN channel bound %d %s", channel_number, addr)

    async def connect(self) -> Tuple[str, int]:
        """
        Create a TURN allocation.
        """
        request = stun.Message(
            message_method=stun.Method.ALLOCATE, message_class=stun.Class.REQUEST
        )
        request.attributes["LIFETIME"] = self.lifetime
        request.attributes["REQUESTED-TRANSPORT"] = UDP_TRANSPORT
        response, _ = await self.request_with_retry(request)

        time_to_expiry = response.attributes["LIFETIME"]
        self.relayed_address = response.attributes["XOR-RELAYED-ADDRESS"]
        logger.info(
            "TURN allocation created %s (expires in %d seconds)",
            self.relayed_address,
            time_to_expiry,
        )

        # periodically refresh allocation
        self.refresh_task = asyncio.create_task(self.refresh(time_to_expiry))

        return self.relayed_address

    def connection_lost(self, exc: Exception) -> None:
        logger.debug("%s connection_lost(%s)", self, exc)
        if self.receiver:
            self.receiver.connection_lost(exc)

    def connection_made(self, transport) -> None:
        logger.debug("%s connection_made(%s)", self, transport)
        self.transport = transport

    def datagram_received(self, data: Union[bytes, Text], addr) -> None:
        data = cast(bytes, data)

        # demultiplex channel data
        if len(data) >= 4 and is_channel_data(data):
            channel, length = struct.unpack("!HH", data[0:4])

            if len(data) >= length + 4 and self.receiver:
                peer_address = self.channel_to_peer.get(channel)
                if peer_address:
                    payload = data[4 : 4 + length]
                    self.receiver.datagram_received(payload, peer_address)

            return

        try:
            message = stun.parse_message(data)
            logger.debug("%s < %s %s", self, addr, message)
        except ValueError:
            return

        if (
            message.message_class == stun.Class.RESPONSE
            or message.message_class == stun.Class.ERROR
        ) and message.transaction_id in self.transactions:
            transaction = self.transactions[message.transaction_id]
            transaction.response_received(message, addr)

    async def delete(self) -> None:
        """
        Delete the TURN allocation.
        """
        if self.refresh_task:
            self.refresh_task.cancel()
            self.refresh_task = None

        request = stun.Message(
            message_method=stun.Method.REFRESH, message_class=stun.Class.REQUEST
        )
        request.attributes["LIFETIME"] = 0
        try:
            await self.request_with_retry(request)
        except stun.TransactionError:
            # we do not care, we need to shutdown
            pass

        logger.info("TURN allocation deleted %s", self.relayed_address)
        self.transport.close()

    async def refresh(self, time_to_expiry) -> None:
        """
        Periodically refresh the TURN allocation.
        """
        while True:
            await asyncio.sleep(5 / 6 * time_to_expiry)

            request = stun.Message(
                message_method=stun.Method.REFRESH, message_class=stun.Class.REQUEST
            )
            request.attributes["LIFETIME"] = self.lifetime
            response, _ = await self.request_with_retry(request)

            time_to_expiry = response.attributes["LIFETIME"]
            logger.info(
                "TURN allocation refreshed %s (expires in %d seconds)",
                self.relayed_address,
                time_to_expiry,
            )

    async def request(
        self, request: stun.Message
    ) -> Tuple[stun.Message, Tuple[str, int]]:
        """
        Execute a STUN transaction and return the response.
        """
        assert request.transaction_id not in self.transactions

        if self.integrity_key:
            self.__add_authentication(request)

        transaction = stun.Transaction(request, self.server, self)
        self.transactions[request.transaction_id] = transaction
        try:
            return await transaction.run()
        finally:
            del self.transactions[request.transaction_id]

    async def request_with_retry(
        self, request: stun.Message
    ) -> Tuple[stun.Message, Tuple[str, int]]:
        """
        Execute a STUN transaction and return the response.

        On recoverable errors it will retry the request.
        """
        try:
            response, addr = await self.request(request)
        except stun.TransactionFailed as e:
            error_code = e.response.attributes["ERROR-CODE"][0]
            if (
                "NONCE" in e.response.attributes
                and self.username is not None
                and self.password is not None
                and (
                    (error_code == 401 and "REALM" in e.response.attributes)
                    or (error_code == 438 and self.realm is not None)
                )
            ):
                # update long-term credentials
                self.nonce = e.response.attributes["NONCE"]
                if error_code == 401:
                    self.realm = e.response.attributes["REALM"]
                self.integrity_key = make_integrity_key(
                    self.username, self.realm, self.password
                )

                # retry request with authentication
                request.transaction_id = random_transaction_id()
                response, addr = await self.request(request)
            else:
                raise

        return response, addr

    async def send_data(self, data: bytes, addr: Tuple[str, int]) -> None:
        """
        Send data to a remote host via the TURN server.
        """
        # if a channel is being bound for the peer, wait
        if addr in self.peer_connect_waiters:
            loop = asyncio.get_event_loop()
            waiter = loop.create_future()
            self.peer_connect_waiters[addr].append(waiter)
            await waiter

        channel = self.peer_to_channel.get(addr)
        now = time.time()
        if channel is None:
            self.peer_connect_waiters[addr] = []
            channel = self.channel_number
            self.channel_number += 1

            # bind channel
            await self.channel_bind(channel, addr)

            # update state
            self.channel_refresh_at[channel] = now + self.channel_refresh_time
            self.channel_to_peer[channel] = addr
            self.peer_to_channel[addr] = channel

            # notify waiters
            for waiter in self.peer_connect_waiters.pop(addr):
                waiter.set_result(None)
        elif now > self.channel_refresh_at[channel]:
            # refresh channel
            await self.channel_bind(channel, addr)

            # update state
            self.channel_refresh_at[channel] = now + self.channel_refresh_time

        header = struct.pack("!HH", channel, len(data))
        self._send(header + data)

    def send_stun(self, message: stun.Message, addr: Tuple[str, int]) -> None:
        """
        Send a STUN message to the TURN server.
        """
        logger.debug("%s > %s %s", self, addr, message)
        self._send(bytes(message))

    def __add_authentication(self, request: stun.Message) -> None:
        request.attributes["USERNAME"] = self.username
        request.attributes["NONCE"] = self.nonce
        request.attributes["REALM"] = self.realm
        request.add_message_integrity(self.integrity_key)


class TurnClientTcpProtocol(TurnClientMixin, TurnStreamMixin, asyncio.Protocol):
    """
    Protocol for handling TURN over TCP.
    """

    def _send(self, data: bytes) -> None:
        self.transport.write(self._padded(data))

    def __repr__(self) -> str:
        return "turn/tcp"


class TurnClientUdpProtocol(TurnClientMixin, asyncio.DatagramProtocol):
    """
    Protocol for handling TURN over UDP.
    """

    def _send(self, data: bytes) -> None:
        self.transport.sendto(data)

    def __repr__(self) -> str:
        return "turn/udp"


class TurnTransport:
    """
    Behaves like a Datagram transport, but uses a TURN allocation.
    """

    def __init__(self, protocol, inner_protocol) -> None:
        self.protocol = protocol
        self.__inner_protocol = inner_protocol
        self.__inner_protocol.receiver = protocol
        self.__relayed_address = None

    def close(self) -> None:
        """
        Close the transport.

        After the TURN allocation has been deleted, the protocol's
        `connection_lost()` method will be called with None as its argument.
        """
        asyncio.create_task(self.__inner_protocol.delete())

    def get_extra_info(self, name: str, default: Any = None) -> Any:
        """
        Return optional transport information.

        - `'related_address'`: the related address
        - `'sockname'`: the relayed address
        """
        if name == "related_address":
            return self.__inner_protocol.transport.get_extra_info("sockname")
        elif name == "sockname":
            return self.__relayed_address
        return default

    def sendto(self, data: bytes, addr: Tuple[str, int]) -> None:
        """
        Sends the `data` bytes to the remote peer given `addr`.

        This will bind a TURN channel as necessary.
        """
        asyncio.create_task(self.__inner_protocol.send_data(data, addr))

    async def _connect(self) -> None:
        self.__relayed_address = await self.__inner_protocol.connect()
        self.protocol.connection_made(self)


async def create_turn_endpoint(
    protocol_factory: Callable[[], _ProtocolT],
    server_addr: Tuple[str, int],
    username: Optional[str],
    password: Optional[str],
    lifetime: int = DEFAULT_ALLOCATION_LIFETIME,
    channel_refresh_time: int = DEFAULT_CHANNEL_REFRESH_TIME,
    ssl: bool = False,
    transport: str = "udp",
) -> Tuple[TurnTransport, _ProtocolT]:
    """
    Create datagram connection relayed over TURN.
    """
    loop = asyncio.get_event_loop()
    inner_protocol: asyncio.BaseProtocol
    inner_transport: asyncio.BaseTransport
    if transport == "tcp":
        inner_transport, inner_protocol = await loop.create_connection(
            lambda: TurnClientTcpProtocol(
                server_addr,
                username=username,
                password=password,
                lifetime=lifetime,
                channel_refresh_time=channel_refresh_time,
            ),
            host=server_addr[0],
            port=server_addr[1],
            ssl=ssl,
        )
    else:
        inner_transport, inner_protocol = await loop.create_datagram_endpoint(
            lambda: TurnClientUdpProtocol(
                server_addr,
                username=username,
                password=password,
                lifetime=lifetime,
                channel_refresh_time=channel_refresh_time,
            ),
            remote_addr=server_addr,
        )
        sock = inner_transport.get_extra_info("socket")
        if sock is not None:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_SOCKET_BUFFER_SIZE)

    try:
        protocol = protocol_factory()
        turn_transport = TurnTransport(protocol, inner_protocol)
        await turn_transport._connect()
    except Exception:
        inner_transport.close()
        raise

    return turn_transport, protocol
