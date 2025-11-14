import argparse
import asyncio
import contextlib
import logging
import os
import ssl
import struct
import time
from typing import Optional, Tuple

from aioice import stun
from aioice.ice import get_host_addresses
from aioice.turn import (
    DEFAULT_ALLOCATION_LIFETIME,
    UDP_TRANSPORT,
    TurnStreamMixin,
    is_channel_data,
    make_integrity_key,
)
from aioice.utils import random_string

logger = logging.getLogger("turn")

CHANNEL_RANGE = range(0x4000, 0x7FFF)

ROOT = os.path.dirname(__file__)
CERT_FILE = os.path.join(ROOT, "turnserver.crt")
KEY_FILE = os.path.join(ROOT, "turnserver.key")


def create_self_signed_cert(name="localhost"):
    from OpenSSL import crypto

    # create key pair
    key = crypto.PKey()
    key.generate_key(crypto.TYPE_RSA, 2048)

    # create self-signed certificate
    cert = crypto.X509()
    cert.get_subject().CN = name
    cert.set_serial_number(1000)
    cert.gmtime_adj_notBefore(0)
    cert.gmtime_adj_notAfter(10 * 365 * 86400)
    cert.set_issuer(cert.get_subject())
    cert.set_pubkey(key)
    cert.sign(key, "sha1")

    with open(CERT_FILE, "wb") as fp:
        fp.write(crypto.dump_certificate(crypto.FILETYPE_PEM, cert))
    with open(KEY_FILE, "wb") as fp:
        fp.write(crypto.dump_privatekey(crypto.FILETYPE_PEM, key))


class Allocation(asyncio.DatagramProtocol):
    def __init__(self, client_address, client_protocol, expiry, username):
        self.channel_to_peer = {}
        self.peer_to_channel = {}

        self.client_address = client_address
        self.client_protocol = client_protocol
        self.expiry = expiry
        self.username = username

    def connection_made(self, transport):
        self.relayed_address = transport.get_extra_info("sockname")
        self.transport = transport

    def datagram_received(self, data, addr):
        """
        Relay data from peer to client.
        """
        channel = self.peer_to_channel.get(addr)
        if channel:
            self.client_protocol._send(
                struct.pack("!HH", channel, len(data)) + data, self.client_address
            )


class TurnServerMixin:
    def __init__(self, server):
        self.server = server

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        # demultiplex channel data
        if len(data) >= 4 and is_channel_data(data):
            channel, length = struct.unpack("!HH", data[0:4])
            allocation = self.server.allocations.get((self, addr))

            if len(data) >= length + 4 and allocation:
                peer_address = allocation.channel_to_peer.get(channel)
                if peer_address:
                    payload = data[4 : 4 + length]
                    allocation.transport.sendto(payload, peer_address)

            return

        try:
            message = stun.parse_message(data)
        except ValueError:
            return
        logger.debug("< %s %s", addr, message)

        assert message.message_class == stun.Class.REQUEST

        if message.message_method == stun.Method.BINDING:
            response = self.handle_binding(message, addr)
            self.send_stun(response, addr)
            return

        # generate failure for test purposes
        if self.server.simulated_failure:
            response = self.error_response(message, *self.server.simulated_failure)
            self.server.simulated_failure = None
            self.send_stun(response, addr)
            return

        if "USERNAME" not in message.attributes:
            response = self.error_response(message, 401, "Unauthorized")
            response.attributes["NONCE"] = random_string(16).encode("ascii")
            response.attributes["REALM"] = self.server.realm
            self.send_stun(response, addr)
            return

        # check credentials
        username = message.attributes["USERNAME"]
        password = self.server.users[username]
        integrity_key = make_integrity_key(username, self.server.realm, password)
        try:
            stun.parse_message(data, integrity_key=integrity_key)
        except ValueError:
            return

        if message.message_method == stun.Method.ALLOCATE:
            asyncio.create_task(self.handle_allocate(message, addr, integrity_key))
            return
        elif message.message_method == stun.Method.REFRESH:
            response = self.handle_refresh(message, addr)
        elif message.message_method == stun.Method.CHANNEL_BIND:
            response = self.handle_channel_bind(message, addr)
        else:
            response = self.error_response(
                message, 400, "Unsupported STUN request method"
            )

        response.add_message_integrity(integrity_key)
        self.send_stun(response, addr)

    async def handle_allocate(self, message, addr, integrity_key):
        key = (self, addr)
        if key in self.server.allocations:
            response = self.error_response(message, 437, "Allocation already exists")
        elif "REQUESTED-TRANSPORT" not in message.attributes:
            response = self.error_response(
                message, 400, "Missing REQUESTED-TRANSPORT attribute"
            )
        elif message.attributes["REQUESTED-TRANSPORT"] != UDP_TRANSPORT:
            response = self.error_response(
                message, 442, "Unsupported transport protocol"
            )
        else:
            lifetime = message.attributes.get("LIFETIME", DEFAULT_ALLOCATION_LIFETIME)
            lifetime = min(lifetime, self.server.maximum_lifetime)

            # create allocation
            loop = asyncio.get_event_loop()
            _, allocation = await loop.create_datagram_endpoint(
                lambda: Allocation(
                    client_address=addr,
                    client_protocol=self,
                    expiry=time.time() + lifetime,
                    username=message.attributes["USERNAME"],
                ),
                local_addr=("127.0.0.1", 0),
            )
            self.server.allocations[key] = allocation

            logger.info("Allocation created %s", allocation.relayed_address)

            # build response
            response = stun.Message(
                message_method=message.message_method,
                message_class=stun.Class.RESPONSE,
                transaction_id=message.transaction_id,
            )
            response.attributes["LIFETIME"] = lifetime
            response.attributes["XOR-MAPPED-ADDRESS"] = addr
            response.attributes["XOR-RELAYED-ADDRESS"] = allocation.relayed_address

        # send response
        response.add_message_integrity(integrity_key)
        self.send_stun(response, addr)

    def handle_binding(self, message, addr):
        response = stun.Message(
            message_method=message.message_method,
            message_class=stun.Class.RESPONSE,
            transaction_id=message.transaction_id,
        )
        response.attributes["XOR-MAPPED-ADDRESS"] = addr
        return response

    def handle_channel_bind(self, message, addr):
        try:
            key = (self, addr)
            allocation = self.server.allocations[key]
        except KeyError:
            return self.error_response(message, 437, "Allocation does not exist")

        if message.attributes["USERNAME"] != allocation.username:
            return self.error_response(message, 441, "Wrong credentials")

        for attr in ["CHANNEL-NUMBER", "XOR-PEER-ADDRESS"]:
            if attr not in message.attributes:
                return self.error_response(message, 400, "Missing %s attribute" % attr)

        channel = message.attributes["CHANNEL-NUMBER"]
        peer_address = message.attributes["XOR-PEER-ADDRESS"]
        if channel not in CHANNEL_RANGE:
            return self.error_response(
                message, 400, "Channel number is outside valid range"
            )

        if allocation.channel_to_peer.get(channel) not in [None, peer_address]:
            return self.error_response(
                message, 400, "Channel is already bound to another peer"
            )
        if allocation.peer_to_channel.get(peer_address) not in [None, channel]:
            return self.error_response(
                message, 400, "Peer is already bound to another channel"
            )

        # register channel
        allocation.channel_to_peer[channel] = peer_address
        allocation.peer_to_channel[peer_address] = channel

        # build response
        response = stun.Message(
            message_method=message.message_method,
            message_class=stun.Class.RESPONSE,
            transaction_id=message.transaction_id,
        )
        return response

    def handle_refresh(self, message, addr):
        try:
            key = (self, addr)
            allocation = self.server.allocations[key]
        except KeyError:
            return self.error_response(message, 437, "Allocation does not exist")

        if message.attributes["USERNAME"] != allocation.username:
            return self.error_response(message, 441, "Wrong credentials")

        if "LIFETIME" not in message.attributes:
            return self.error_response(message, 400, "Missing LIFETIME attribute")

        # refresh allocation
        lifetime = min(message.attributes["LIFETIME"], self.server.maximum_lifetime)
        if lifetime:
            logger.info("Allocation refreshed %s", allocation.relayed_address)
            allocation.expiry = time.time() + lifetime
        else:
            logger.info("Allocation deleted %s", allocation.relayed_address)
            self.server._remove_allocation(key)

        # build response
        response = stun.Message(
            message_method=message.message_method,
            message_class=stun.Class.RESPONSE,
            transaction_id=message.transaction_id,
        )
        response.attributes["LIFETIME"] = lifetime
        return response

    def error_response(self, request, code, message):
        """
        Build an error response for the given request.
        """
        response = stun.Message(
            message_method=request.message_method,
            message_class=stun.Class.ERROR,
            transaction_id=request.transaction_id,
        )
        response.attributes["ERROR-CODE"] = (code, message)
        return response

    def send_stun(self, message, addr):
        logger.debug("> %s %s", addr, message)
        self._send(bytes(message), addr)


class TurnServerTcpProtocol(TurnServerMixin, TurnStreamMixin, asyncio.Protocol):
    def _send(self, data, addr):
        self.transport.write(self._padded(data))


class TurnServerUdpProtocol(TurnServerMixin, asyncio.DatagramProtocol):
    def _send(self, data, addr):
        self.transport.sendto(data, addr)


class TurnServer:
    """
    STUN / TURN server.
    """

    def __init__(self, realm="test", users={}):
        self.allocations = {}
        self.maximum_lifetime = 3600
        self.realm = realm
        self.simulated_failure: Optional[Tuple[int, str]] = None
        self.users = users

        self._expire_task = None

    async def close(self):
        # stop expiry loop
        if self._expire_task is not None:
            self._expire_task.cancel()

        # close allocations
        for key in list(self.allocations.keys()):
            self._remove_allocation(key)

        # shutdown servers
        self.tcp_server.close()
        self.tls_server.close()
        self.udp_server.transport.close()
        await asyncio.gather(
            self.tcp_server.wait_closed(), self.tls_server.wait_closed()
        )

    async def listen(self, port=0, tls_port=0):
        loop = asyncio.get_event_loop()
        hostaddr = get_host_addresses(use_ipv4=True, use_ipv6=False)[0]

        # listen for TCP
        self.tcp_server = await loop.create_server(
            lambda: TurnServerTcpProtocol(server=self), host=hostaddr, port=port
        )
        self.tcp_address = self.tcp_server.sockets[0].getsockname()
        logger.info("Listening for TCP on %s", self.tcp_address)

        # listen for UDP
        transport, self.udp_server = await loop.create_datagram_endpoint(
            lambda: TurnServerUdpProtocol(server=self), local_addr=(hostaddr, port)
        )
        self.udp_address = transport.get_extra_info("sockname")
        logger.info("Listening for UDP on %s", self.udp_address)

        # listen for TLS
        ssl_context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(CERT_FILE, KEY_FILE)
        self.tls_server = await loop.create_server(
            lambda: TurnServerTcpProtocol(server=self),
            host=hostaddr,
            port=tls_port,
            ssl=ssl_context,
        )
        self.tls_address = self.tls_server.sockets[0].getsockname()
        logger.info("Listening for TLS on %s", self.tls_address)

        # start expiry loop
        self._expire_task = asyncio.create_task(self._expire_allocations())

    async def _expire_allocations(self):
        while True:
            now = time.time()
            for key, allocation in list(self.allocations.items()):
                if allocation.expiry < now:
                    logger.info("Allocation expired %s", allocation.relayed_address)
                    self.server._remove_allocation(key)

            await asyncio.sleep(1)

    def _remove_allocation(self, key):
        allocation = self.allocations.pop(key)
        allocation.transport.close()


@contextlib.asynccontextmanager
async def run_turn_server(**kwargs):
    server = TurnServer(**kwargs)
    await server.listen()
    try:
        yield server
    finally:
        await server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="STUN / TURN server")
    parser.add_argument("--verbose", "-v", action="count")
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    srv = TurnServer(realm="test", users={"foo": "bar"})
    loop = asyncio.get_event_loop()
    loop.run_until_complete(srv.listen(port=3478, tls_port=5349))
    loop.run_forever()
