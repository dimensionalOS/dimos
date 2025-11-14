import asyncio
import re
import socket
import sys
import uuid
from typing import Dict, List, Optional, Set, Text, Tuple, Union, cast

import dns.exception
import dns.flags
import dns.message
import dns.name
import dns.rdata
import dns.rdataclass
import dns.rdataset
import dns.rdatatype
import dns.zone

MDNS_ADDRESS = "224.0.0.251"
MDNS_PORT = 5353
MDNS_HOSTNAME_RE = re.compile(r"^[a-zA-Z0-9-]{1,63}\.local$")

MDNS_RDCLASS = dns.rdataclass.IN | 0x8000


def create_mdns_hostname():
    return str(uuid.uuid4()) + ".local"


def is_mdns_hostname(name: str) -> bool:
    return MDNS_HOSTNAME_RE.match(name) is not None


class MDnsProtocol(asyncio.DatagramProtocol):
    def __init__(self, tx_transport: asyncio.DatagramTransport) -> None:
        self.__closed: asyncio.Future[bool] = asyncio.Future()
        self.zone = dns.zone.Zone("", relativize=False, rdclass=MDNS_RDCLASS)
        self.queries: Dict[dns.name.Name, Set[asyncio.Future[str]]] = {}

        self.rx_transport: Optional[asyncio.DatagramTransport] = None
        self.tx_transport = tx_transport

    def connection_lost(self, exc: Exception) -> None:
        # abort any outstanding queries
        for name, futures in list(self.queries.items()):
            for future in futures:
                future.set_exception(asyncio.TimeoutError)
        self.__closed.set_result(True)

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.rx_transport = cast(asyncio.DatagramTransport, transport)

    def datagram_received(self, data: Union[bytes, Text], addr: Tuple) -> None:
        # parse message
        try:
            message = dns.message.from_wire(cast(bytes, data))
        except dns.exception.FormError:
            return

        if isinstance(message, dns.message.QueryMessage):
            # answer question
            for question in message.question:
                rdtypes: List[int] = []
                if question.rdtype in (
                    dns.rdatatype.ANY,
                    dns.rdatatype.A,
                    dns.rdatatype.AAAA,
                ):
                    rdtypes += [dns.rdatatype.A, dns.rdatatype.AAAA]

                response = dns.message.QueryMessage(id=0)
                response.flags |= dns.flags.QR
                response.flags |= dns.flags.AA

                for rdtype in rdtypes:
                    try:
                        response.answer.append(
                            self.zone.find_rrset(name=question.name, rdtype=rdtype)
                        )
                    except KeyError:
                        continue
                if response.answer:
                    self.tx_transport.sendto(
                        response.to_wire(), (MDNS_ADDRESS, MDNS_PORT)
                    )

            # handle answer
            for answer in message.answer:
                for item in answer:
                    item = item.to_generic()
                    if (
                        isinstance(item, dns.rdata.GenericRdata)
                        and item.rdclass == MDNS_RDCLASS
                        and item.rdtype in (dns.rdatatype.A, dns.rdatatype.AAAA)
                    ):
                        if item.rdtype == dns.rdatatype.A:
                            result = socket.inet_ntop(socket.AF_INET, item.data)
                        else:
                            result = socket.inet_ntop(socket.AF_INET6, item.data)
                        for future in self.queries.pop(answer.name, []):
                            future.set_result(result)

    # custom

    async def close(self) -> None:
        self.rx_transport.close()
        self.tx_transport.close()
        await self.__closed

    async def publish(self, hostname: str, addr: str) -> None:
        name = dns.name.from_text(hostname)
        try:
            data = socket.inet_pton(socket.AF_INET, addr)
            rdtype = dns.rdatatype.A
        except OSError:
            data = socket.inet_pton(socket.AF_INET6, addr)
            rdtype = dns.rdatatype.AAAA

        rdata = dns.rdata.GenericRdata(rdclass=MDNS_RDCLASS, rdtype=rdtype, data=data)
        self.zone.replace_rdataset(name, dns.rdataset.from_rdata(120, rdata))

    async def resolve(
        self, hostname: str, timeout: Optional[float] = 1.0
    ) -> Optional[str]:
        name = dns.name.from_text(hostname)
        future: asyncio.Future[str] = asyncio.Future()

        if name in self.queries:
            # a query for this name is already pending
            self.queries[name].add(future)
        else:
            # no query for this name is pending, send a request
            self.queries[name] = set([future])
            message = dns.message.make_query(name, rdtype=dns.rdatatype.A)
            message.id = 0
            message.flags = 0
            self.tx_transport.sendto(message.to_wire(), (MDNS_ADDRESS, MDNS_PORT))

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            return None
        finally:
            if name in self.queries:
                self.queries[name].discard(future)
                if not self.queries[name]:
                    del self.queries[name]


async def create_mdns_protocol() -> MDnsProtocol:
    """
    Using a single socket works fine on Linux, but on OS X we need to use
    separate sockets for sending and receiving.
    """
    loop = asyncio.get_event_loop()

    # sender

    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    tx_sock.bind(("", MDNS_PORT))

    tx_transport, _ = await loop.create_datagram_endpoint(
        lambda: asyncio.DatagramProtocol(),
        sock=tx_sock,
    )

    # receiver

    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    rx_sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        socket.inet_aton(MDNS_ADDRESS) + b"\x00\x00\x00\x00",
    )
    if sys.platform == "win32":
        rx_sock.bind(("", MDNS_PORT))
    else:
        rx_sock.bind((MDNS_ADDRESS, MDNS_PORT))

    _, protocol = await loop.create_datagram_endpoint(
        lambda: MDnsProtocol(tx_transport=tx_transport),
        sock=rx_sock,
    )

    return protocol
