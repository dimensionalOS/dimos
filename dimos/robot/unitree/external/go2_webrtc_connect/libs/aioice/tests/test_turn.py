import asyncio
import ssl
import unittest

from aioice import stun, turn

from .echoserver import run_echo_server
from .turnserver import run_turn_server
from .utils import asynctest, read_message

PROTOCOL_KWARGS = {
    "username": "foo",
    "password": "bar",
    "lifetime": turn.DEFAULT_ALLOCATION_LIFETIME,
    "channel_refresh_time": turn.DEFAULT_CHANNEL_REFRESH_TIME,
}


class DummyClientProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.received = []

    def datagram_received(self, data, addr):
        self.received.append((data, addr))


class TurnClientTcpProtocolTest(unittest.TestCase):
    def setUp(self):
        class MockProtocol:
            def get_extra_info(self, name):
                return ("1.2.3.4", 1234)

        self.protocol = turn.TurnClientTcpProtocol(("1.2.3.4", 1234), **PROTOCOL_KWARGS)
        self.protocol.connection_made(MockProtocol())

    def test_receive_stun_fragmented(self):
        data = read_message("binding_request.bin")
        self.protocol.data_received(data[0:10])
        self.protocol.data_received(data[10:])

    def test_receive_junk(self):
        self.protocol.data_received(b"\x00" * 20)

    def test_repr(self):
        self.assertEqual(repr(self.protocol), "turn/tcp")


class TurnClientUdpProtocolTest(unittest.TestCase):
    def setUp(self):
        self.protocol = turn.TurnClientUdpProtocol(("1.2.3.4", 1234), **PROTOCOL_KWARGS)

    def test_receive_junk(self):
        self.protocol.datagram_received(b"\x00" * 20, ("1.2.3.4", 1234))

    def test_repr(self):
        self.assertEqual(repr(self.protocol), "turn/udp")


class TurnTest(unittest.TestCase):
    @asynctest
    async def test_tcp_transport(self):
        await self._test_transport("tcp", "tcp_address")

    @asynctest
    async def test_tls_transport(self):
        ssl_context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        await self._test_transport("tcp", "tls_address", ssl=ssl_context)

    @asynctest
    async def test_udp_transport(self):
        await self._test_transport("udp", "udp_address")

    async def _test_transport(self, transport, server_addr_attr, ssl=False):
        await self._test_transport_ok(
            transport=transport, server_addr_attr=server_addr_attr, ssl=ssl
        )
        await self._test_transport_ok_multi(
            transport=transport, server_addr_attr=server_addr_attr, ssl=ssl
        )
        await self._test_transport_allocate_failure(
            transport=transport, server_addr_attr=server_addr_attr, ssl=ssl
        )
        await self._test_transport_delete_failure(
            transport=transport, server_addr_attr=server_addr_attr, ssl=ssl
        )

    async def _test_transport_ok(self, *, transport, server_addr_attr, ssl):
        async with run_turn_server(realm="test", users={"foo": "bar"}) as turn_server:
            transport, protocol = await turn.create_turn_endpoint(
                DummyClientProtocol,
                server_addr=getattr(turn_server, server_addr_attr),
                username="foo",
                password="bar",
                channel_refresh_time=5,
                lifetime=6,
                ssl=ssl,
                transport=transport,
            )
            self.assertIsNone(transport.get_extra_info("peername"))
            self.assertIsNotNone(transport.get_extra_info("sockname"))

            async with run_echo_server() as echo_server:
                # bind channel, send ping, expect pong
                transport.sendto(b"ping", echo_server.udp_address)
                await asyncio.sleep(1)
                self.assertEqual(
                    protocol.received, [(b"ping", echo_server.udp_address)]
                )

                # wait some more to allow allocation refresh
                protocol.received.clear()
                await asyncio.sleep(5)

                # refresh channel, send ping, expect pong
                transport.sendto(b"ping", echo_server.udp_address)
                await asyncio.sleep(1)
                self.assertEqual(
                    protocol.received, [(b"ping", echo_server.udp_address)]
                )

            # close
            transport.close()
            await asyncio.sleep(0)

    async def _test_transport_ok_multi(self, *, transport, server_addr_attr, ssl):
        async with run_turn_server(realm="test", users={"foo": "bar"}) as turn_server:
            transport, protocol = await turn.create_turn_endpoint(
                DummyClientProtocol,
                server_addr=getattr(turn_server, server_addr_attr),
                username="foo",
                password="bar",
                channel_refresh_time=5,
                lifetime=6,
                ssl=ssl,
                transport=transport,
            )
            self.assertIsNone(transport.get_extra_info("peername"))
            self.assertIsNotNone(transport.get_extra_info("sockname"))

            # Bind channel, send ping, expect pong.
            #
            # We use different lengths to trigger both padded an unpadded
            # ChannelData messages over TCP.
            async with run_echo_server() as echo_server1:
                async with run_echo_server() as echo_server2:
                    transport.sendto(b"ping", echo_server1.udp_address)  # never padded
                    transport.sendto(b"ping11", echo_server1.udp_address)
                    transport.sendto(b"ping20", echo_server2.udp_address)
                    transport.sendto(b"ping21", echo_server2.udp_address)
                    await asyncio.sleep(1)
                    self.assertEqual(
                        sorted(protocol.received),
                        [
                            (b"ping", echo_server1.udp_address),
                            (b"ping11", echo_server1.udp_address),
                            (b"ping20", echo_server2.udp_address),
                            (b"ping21", echo_server2.udp_address),
                        ],
                    )

            # close
            transport.close()
            await asyncio.sleep(0)

    async def _test_transport_allocate_failure(
        self, *, transport, server_addr_attr, ssl
    ):
        async with run_turn_server(realm="test", users={"foo": "bar"}) as turn_server:
            # make the server reject the ALLOCATE request
            turn_server.simulated_failure = (403, "Forbidden")

            with self.assertRaises(stun.TransactionFailed) as cm:
                await turn.create_turn_endpoint(
                    DummyClientProtocol,
                    server_addr=getattr(turn_server, server_addr_attr),
                    username="foo",
                    password="bar",
                    ssl=ssl,
                    transport=transport,
                )
        self.assertEqual(str(cm.exception), "STUN transaction failed (403 - Forbidden)")

    async def _test_transport_delete_failure(self, *, transport, server_addr_attr, ssl):
        async with run_turn_server(realm="test", users={"foo": "bar"}) as turn_server:
            transport, protocol = await turn.create_turn_endpoint(
                DummyClientProtocol,
                server_addr=getattr(turn_server, server_addr_attr),
                username="foo",
                password="bar",
                ssl=ssl,
                transport=transport,
            )
            self.assertIsNone(transport.get_extra_info("peername"))
            self.assertIsNotNone(transport.get_extra_info("sockname"))

            # make the server reject the final REFRESH request
            turn_server.simulated_failure = (403, "Forbidden")

            # close client
            transport.close()
            await asyncio.sleep(0)
