import asyncio
import functools
import os
import unittest
from unittest import mock

import ifaddr
from aioice import Candidate, TransportPolicy, ice, mdns, stun

from .turnserver import run_turn_server
from .utils import asynctest, invite_accept

RUNNING_ON_CI = os.environ.get("GITHUB_ACTIONS") == "true"


async def delay(coro):
    await asyncio.sleep(1)
    await coro()


class ProtocolMock:
    local_candidate = Candidate(
        foundation="some-foundation",
        component=1,
        transport="udp",
        priority=1234,
        host="1.2.3.4",
        port=1234,
        type="host",
    )

    sent_message = None

    async def request(self, message, addr, integrity_key=None):
        return (self.response_message, self.response_addr)

    def send_stun(self, message, addr):
        self.sent_message = message


class IceComponentTest(unittest.TestCase):
    @asynctest
    async def test_peer_reflexive(self):
        connection = ice.Connection(ice_controlling=True)
        connection.remote_password = "remote-password"
        connection.remote_username = "remote-username"

        protocol = ProtocolMock()
        protocol.response_addr = ("2.3.4.5", 2345)
        protocol.response_message = "bad"

        request = stun.Message(
            message_method=stun.Method.BINDING, message_class=stun.Class.REQUEST
        )
        request.attributes["PRIORITY"] = 456789

        connection.check_incoming(request, ("2.3.4.5", 2345), protocol)
        self.assertIsNone(protocol.sent_message)

        # check we have discovered a peer-reflexive candidate
        self.assertEqual(len(connection.remote_candidates), 1)
        candidate = connection.remote_candidates[0]
        self.assertEqual(candidate.component, 1)
        self.assertEqual(candidate.transport, "udp")
        self.assertEqual(candidate.priority, 456789)
        self.assertEqual(candidate.host, "2.3.4.5")
        self.assertEqual(candidate.port, 2345)
        self.assertEqual(candidate.type, "prflx")
        self.assertEqual(candidate.generation, None)

        # check a new pair was formed
        self.assertEqual(len(connection._check_list), 1)
        pair = connection._check_list[0]
        self.assertEqual(pair.protocol, protocol)
        self.assertEqual(pair.remote_candidate, candidate)

        # check a triggered check was scheduled
        self.assertIsNotNone(pair.task)
        await pair.task

    @asynctest
    async def test_request_with_invalid_method(self):
        connection = ice.Connection(ice_controlling=True)

        protocol = ProtocolMock()

        request = stun.Message(
            message_method=stun.Method.ALLOCATE, message_class=stun.Class.REQUEST
        )

        connection.request_received(
            request, ("2.3.4.5", 2345), protocol, bytes(request)
        )
        self.assertIsNotNone(protocol.sent_message)
        self.assertEqual(protocol.sent_message.message_method, stun.Method.ALLOCATE)
        self.assertEqual(protocol.sent_message.message_class, stun.Class.ERROR)
        self.assertEqual(
            protocol.sent_message.attributes["ERROR-CODE"], (400, "Bad Request")
        )

    @asynctest
    async def test_response_with_invalid_address(self):
        connection = ice.Connection(ice_controlling=True)
        connection.remote_password = "remote-password"
        connection.remote_username = "remote-username"

        protocol = ProtocolMock()
        protocol.response_addr = ("3.4.5.6", 3456)
        protocol.response_message = "bad"

        pair = ice.CandidatePair(
            protocol,
            Candidate(
                foundation="some-foundation",
                component=1,
                transport="udp",
                priority=2345,
                host="2.3.4.5",
                port=2345,
                type="host",
            ),
        )
        self.assertEqual(
            repr(pair), "CandidatePair(('1.2.3.4', 1234) -> ('2.3.4.5', 2345))"
        )

        await connection.check_start(pair)
        self.assertEqual(pair.state, ice.CandidatePair.State.FAILED)


class IceConnectionTest(unittest.TestCase):
    def assertCandidateTypes(self, conn, expected):
        types = set([c.type for c in conn.local_candidates])
        self.assertEqual(types, expected)

    def tearDown(self):
        ice.CONSENT_FAILURES = 6
        ice.CONSENT_INTERVAL = 5
        stun.RETRY_MAX = 6

    @mock.patch("ifaddr.get_adapters")
    def test_get_host_addresses(self, mock_get_adapters):
        mock_get_adapters.return_value = [
            ifaddr.Adapter(
                ips=[
                    ifaddr.IP(ip="127.0.0.1", network_prefix=8, nice_name="lo"),
                    ifaddr.IP(ip=("::1", 0, 0), network_prefix=128, nice_name="lo"),
                ],
                name="lo",
                nice_name="lo",
            ),
            ifaddr.Adapter(
                ips=[
                    ifaddr.IP(
                        ip="1.2.3.4",
                        network_prefix=24,
                        nice_name="eth0",
                    ),
                    ifaddr.IP(
                        ip=("2a02:0db8:85a3:0000:0000:8a2e:0370:7334", 0, 0),
                        network_prefix=64,
                        nice_name="eth0",
                    ),
                    ifaddr.IP(
                        ip=("fe80::1234:5678:9abc:def0", 0, 2),
                        network_prefix=64,
                        nice_name="eth0",
                    ),
                ],
                name="eth0",
                nice_name="eth0",
            ),
        ]

        # IPv4 only
        addresses = ice.get_host_addresses(use_ipv4=True, use_ipv6=False)
        self.assertEqual(addresses, ["1.2.3.4"])

        # IPv6 only
        addresses = ice.get_host_addresses(use_ipv4=False, use_ipv6=True)
        self.assertEqual(addresses, ["2a02:0db8:85a3:0000:0000:8a2e:0370:7334"])

        # both
        addresses = ice.get_host_addresses(use_ipv4=True, use_ipv6=True)
        self.assertEqual(
            addresses, ["1.2.3.4", "2a02:0db8:85a3:0000:0000:8a2e:0370:7334"]
        )

    @asynctest
    async def test_close(self):
        conn_a = ice.Connection(ice_controlling=True)

        # close
        event, _ = await asyncio.gather(conn_a.get_event(), conn_a.close())
        self.assertTrue(isinstance(event, ice.ConnectionClosed))

        # no more events
        event = await conn_a.get_event()
        self.assertIsNone(event)

        # close again
        await conn_a.close()

    @asynctest
    async def test_connect(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # we should only have host candidates
        self.assertCandidateTypes(conn_a, set(["host"]))
        self.assertCandidateTypes(conn_b, set(["host"]))

        # there should be a default candidate for component 1
        candidate = conn_a.get_default_candidate(1)
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.type, "host")

        # there should not be a default candidate for component 2
        candidate = conn_a.get_default_candidate(2)
        self.assertIsNone(candidate)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_close(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # close while connecting
        await conn_b.close()
        done, pending = await asyncio.wait(
            [
                asyncio.create_task(conn_a.connect()),
                asyncio.create_task(delay(conn_a.close)),
            ]
        )
        for task in pending:
            task.cancel()
        self.assertEqual(len(done), 2)

        exceptions = [x.exception() for x in done if x.exception()]
        self.assertEqual(len(exceptions), 1)
        self.assertTrue(isinstance(exceptions[0], ConnectionError))

    @asynctest
    async def test_connect_early_checks(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await conn_a.connect()
        await asyncio.sleep(1)
        await conn_b.connect()

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_early_checks_2(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # both sides gather local candidates and exchange credentials
        await conn_a.gather_candidates()
        await conn_b.gather_candidates()
        conn_a.remote_username = conn_b.local_username
        conn_a.remote_password = conn_b.local_password
        conn_b.remote_username = conn_a.local_username
        conn_b.remote_password = conn_a.local_password

        async def connect_b():
            # side B receives offer and connects
            for candidate in conn_a.local_candidates:
                await conn_b.add_remote_candidate(candidate)
            await conn_b.add_remote_candidate(None)
            await conn_b.connect()

            # side A receives candidates
            for candidate in conn_b.local_candidates:
                await conn_a.add_remote_candidate(candidate)
            await conn_a.add_remote_candidate(None)

        # The sequence is:
        # - side A starts connecting immediately, but has no candidates
        # - side B receives candidates and connects
        # - side A receives candidates, and connection completes
        await asyncio.gather(conn_a.connect(), connect_b())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_two_components(self):
        conn_a = ice.Connection(ice_controlling=True, components=2)
        conn_b = ice.Connection(ice_controlling=False, components=2)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # we should only have host candidates
        self.assertCandidateTypes(conn_a, set(["host"]))
        self.assertCandidateTypes(conn_b, set(["host"]))

        # there should be a default candidate for component 1
        candidate = conn_a.get_default_candidate(1)
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.type, "host")

        # there should be a default candidate for component 2
        candidate = conn_a.get_default_candidate(2)
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.type, "host")

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertEqual(conn_a._components, set([1, 2]))
        self.assertEqual(conn_b._components, set([1, 2]))

        # send data a -> b (component 1)
        await conn_a.sendto(b"howdee", 1)
        data, component = await conn_b.recvfrom()
        self.assertEqual(data, b"howdee")
        self.assertEqual(component, 1)

        # send data b -> a (component 1)
        await conn_b.sendto(b"gotcha", 1)
        data, component = await conn_a.recvfrom()
        self.assertEqual(data, b"gotcha")
        self.assertEqual(component, 1)

        # send data a -> b (component 2)
        await conn_a.sendto(b"howdee 2", 2)
        data, component = await conn_b.recvfrom()
        self.assertEqual(data, b"howdee 2")
        self.assertEqual(component, 2)

        # send data b -> a (component 2)
        await conn_b.sendto(b"gotcha 2", 2)
        data, component = await conn_a.recvfrom()
        self.assertEqual(data, b"gotcha 2")
        self.assertEqual(component, 2)

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_two_components_vs_one_component(self):
        """
        It is possible that some of the local candidates won't get paired with
        remote candidates, and some of the remote candidates won't get paired
        with local candidates.  This can happen if one agent doesn't include
        candidates for the all of the components for a media stream.  If this
        happens, the number of components for that media stream is effectively
        reduced, and considered to be equal to the minimum across both agents
        of the maximum component ID provided by each agent across all
        components for the media stream.
        """
        conn_a = ice.Connection(ice_controlling=True, components=2)
        conn_b = ice.Connection(ice_controlling=False, components=1)

        # invite / accept
        await invite_accept(conn_a, conn_b)
        self.assertTrue(len(conn_a.local_candidates) > 0)
        for candidate in conn_a.local_candidates:
            self.assertEqual(candidate.type, "host")

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertEqual(conn_a._components, set([1]))
        self.assertEqual(conn_b._components, set([1]))

        # send data a -> b (component 1)
        await conn_a.sendto(b"howdee", 1)
        data, component = await conn_b.recvfrom()
        self.assertEqual(data, b"howdee")
        self.assertEqual(component, 1)

        # send data b -> a (component 1)
        await conn_b.sendto(b"gotcha", 1)
        data, component = await conn_a.recvfrom()
        self.assertEqual(data, b"gotcha")
        self.assertEqual(component, 1)

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_to_ice_lite(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_a.remote_is_lite = True
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # we should only have host candidates
        self.assertCandidateTypes(conn_a, set(["host"]))
        self.assertCandidateTypes(conn_b, set(["host"]))

        # there should be a default candidate for component 1
        candidate = conn_a.get_default_candidate(1)
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate.type, "host")

        # there should not be a default candidate for component 2
        candidate = conn_a.get_default_candidate(2)
        self.assertIsNone(candidate)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_to_ice_lite_nomination_fails(self):
        def mock_request_received(self, message, addr, protocol, raw_data):
            if "USE-CANDIDATE" in message.attributes:
                self.respond_error(message, addr, protocol, (500, "Internal Error"))
            else:
                self.real_request_received(message, addr, protocol, raw_data)

        conn_a = ice.Connection(ice_controlling=True)
        conn_a.remote_is_lite = True
        conn_b = ice.Connection(ice_controlling=False)
        conn_b.real_request_received = conn_b.request_received
        conn_b.request_received = functools.partial(mock_request_received, conn_b)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        with self.assertRaises(ConnectionError) as cm:
            await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertEqual(str(cm.exception), "ICE negotiation failed")

        # close
        await conn_a.close()
        await conn_b.close()

    @unittest.skipIf(RUNNING_ON_CI, "CI lacks ipv6")
    @asynctest
    async def test_connect_ipv6(self):
        conn_a = ice.Connection(ice_controlling=True, use_ipv4=False, use_ipv6=True)
        conn_b = ice.Connection(ice_controlling=False, use_ipv4=False, use_ipv6=True)

        # invite / accept
        await invite_accept(conn_a, conn_b)
        self.assertTrue(len(conn_a.local_candidates) > 0)
        for candidate in conn_a.local_candidates:
            self.assertEqual(candidate.type, "host")

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_reverse_order(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # introduce a delay so that B's checks complete before A's
        await asyncio.gather(delay(conn_a.connect), conn_b.connect())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_invalid_password(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite
        await conn_a.gather_candidates()
        for candidate in conn_a.local_candidates:
            await conn_b.add_remote_candidate(candidate)
        await conn_b.add_remote_candidate(None)
        conn_b.remote_username = conn_a.local_username
        conn_b.remote_password = conn_a.local_password

        # accept
        await conn_b.gather_candidates()
        for candidate in conn_b.local_candidates:
            await conn_a.add_remote_candidate(candidate)
        await conn_a.add_remote_candidate(None)
        conn_a.remote_username = conn_b.local_username
        conn_a.remote_password = "wrong-password"

        # connect
        done, pending = await asyncio.wait(
            [
                asyncio.create_task(conn_a.connect()),
                asyncio.create_task(conn_b.connect()),
            ],
            return_when=asyncio.FIRST_EXCEPTION,
        )
        for task in pending:
            task.cancel()
        self.assertEqual(len(done), 1)
        self.assertTrue(isinstance(done.pop().exception(), ConnectionError))

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_invalid_username(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite
        await conn_a.gather_candidates()
        for candidate in conn_a.local_candidates:
            await conn_b.add_remote_candidate(candidate)
        await conn_b.add_remote_candidate(None)
        conn_b.remote_username = conn_a.local_username
        conn_b.remote_password = conn_a.local_password

        # accept
        await conn_b.gather_candidates()
        for candidate in conn_b.local_candidates:
            await conn_a.add_remote_candidate(candidate)
        await conn_a.add_remote_candidate(None)
        conn_a.remote_username = "wrong-username"
        conn_a.remote_password = conn_b.local_password

        # connect
        done, pending = await asyncio.wait(
            [
                asyncio.create_task(conn_a.connect()),
                asyncio.create_task(conn_b.connect()),
            ]
        )
        for task in pending:
            task.cancel()
        self.assertEqual(len(done), 2)
        self.assertTrue(isinstance(done.pop().exception(), ConnectionError))
        self.assertTrue(isinstance(done.pop().exception(), ConnectionError))

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_no_gather(self):
        """
        If local candidates gathering was not performed, connect fails.
        """
        conn = ice.Connection(ice_controlling=True)
        await conn.add_remote_candidate(
            Candidate.from_sdp(
                "6815297761 1 udp 659136 1.2.3.4 31102 typ host generation 0"
            )
        )
        await conn.add_remote_candidate(None)
        conn.remote_username = "foo"
        conn.remote_password = "bar"
        with self.assertRaises(ConnectionError) as cm:
            await conn.connect()
        self.assertEqual(
            str(cm.exception), "Local candidates gathering was not performed"
        )
        await conn.close()

    @asynctest
    async def test_connect_no_local_candidates(self):
        """
        If local candidates gathering yielded no candidates, connect fails.
        """
        conn = ice.Connection(ice_controlling=True)
        conn._local_candidates_end = True
        await conn.add_remote_candidate(
            Candidate.from_sdp(
                "6815297761 1 udp 659136 1.2.3.4 31102 typ host generation 0"
            )
        )
        await conn.add_remote_candidate(None)
        conn.remote_username = "foo"
        conn.remote_password = "bar"
        with self.assertRaises(ConnectionError) as cm:
            await conn.connect()
        self.assertEqual(str(cm.exception), "ICE negotiation failed")
        await conn.close()

    @asynctest
    async def test_connect_no_remote_candidates(self):
        """
        If no remote candidates were provided, connect fails.
        """
        conn = ice.Connection(ice_controlling=True)
        await conn.gather_candidates()
        await conn.add_remote_candidate(None)
        conn.remote_username = "foo"
        conn.remote_password = "bar"
        with self.assertRaises(ConnectionError) as cm:
            await conn.connect()
        self.assertEqual(str(cm.exception), "ICE negotiation failed")
        await conn.close()

    @asynctest
    async def test_connect_no_remote_credentials(self):
        """
        If remote credentials have not been provided, connect fails.
        """
        conn = ice.Connection(ice_controlling=True)
        await conn.gather_candidates()
        await conn.add_remote_candidate(
            Candidate.from_sdp(
                "6815297761 1 udp 659136 1.2.3.4 31102 typ host generation 0"
            )
        )
        await conn.add_remote_candidate(None)
        with self.assertRaises(ConnectionError) as cm:
            await conn.connect()
        self.assertEqual(str(cm.exception), "Remote username or password is missing")
        await conn.close()

    @asynctest
    async def test_connect_role_conflict_both_controlling(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=True)

        # set tie breaker for a deterministic outcome
        conn_a._tie_breaker = 1
        conn_b._tie_breaker = 2

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertFalse(conn_a.ice_controlling)
        self.assertTrue(conn_b.ice_controlling)

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_role_conflict_both_controlled(self):
        conn_a = ice.Connection(ice_controlling=False)
        conn_b = ice.Connection(ice_controlling=False)

        # set tie breaker for a deterministic outcome
        conn_a._tie_breaker = 1
        conn_b._tie_breaker = 2

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertFalse(conn_a.ice_controlling)
        self.assertTrue(conn_b.ice_controlling)

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_timeout(self):
        # lower STUN retries
        stun.RETRY_MAX = 1

        conn = ice.Connection(ice_controlling=True)
        await conn.gather_candidates()
        await conn.add_remote_candidate(
            Candidate.from_sdp(
                "6815297761 1 udp 659136 1.2.3.4 31102 typ host generation 0"
            )
        )
        await conn.add_remote_candidate(None)
        conn.remote_username = "foo"
        conn.remote_password = "bar"
        with self.assertRaises(ConnectionError) as cm:
            await conn.connect()
        self.assertEqual(str(cm.exception), "ICE negotiation failed")
        await conn.close()

    @asynctest
    async def test_connect_with_stun_server(self):
        async with run_turn_server() as stun_server:
            conn_a = ice.Connection(
                ice_controlling=True, stun_server=stun_server.udp_address
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould have both host and server-reflexive candidates
            self.assertCandidateTypes(conn_a, set(["host", "srflx"]))
            self.assertCandidateTypes(conn_b, set(["host"]))

            # the default candidate should be server-reflexive
            candidate = conn_a.get_default_candidate(1)
            self.assertIsNotNone(candidate)
            self.assertEqual(candidate.type, "srflx")
            self.assertIsNotNone(candidate.related_address)
            self.assertIsNotNone(candidate.related_port)

            # connect
            await asyncio.gather(conn_a.connect(), conn_b.connect())

            # send data a -> b
            await conn_a.send(b"howdee")
            data = await conn_b.recv()
            self.assertEqual(data, b"howdee")

            # send data b -> a
            await conn_b.send(b"gotcha")
            data = await conn_a.recv()
            self.assertEqual(data, b"gotcha")

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_connect_with_stun_server_dns_lookup_error(self):
        conn_a = ice.Connection(ice_controlling=True, stun_server=("invalid.", 1234))
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # we whould have only host candidates
        self.assertCandidateTypes(conn_a, set(["host"]))
        self.assertCandidateTypes(conn_b, set(["host"]))

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_connect_with_stun_server_timeout(self):
        async with run_turn_server() as stun_server:
            # immediately stop turn server
            await stun_server.close()

            conn_a = ice.Connection(
                ice_controlling=True, stun_server=stun_server.udp_address
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould have only host candidates
            self.assertCandidateTypes(conn_a, set(["host"]))
            self.assertCandidateTypes(conn_b, set(["host"]))

            # connect
            await asyncio.gather(conn_a.connect(), conn_b.connect())

            # send data a -> b
            await conn_a.send(b"howdee")
            data = await conn_b.recv()
            self.assertEqual(data, b"howdee")

            # send data b -> a
            await conn_b.send(b"gotcha")
            data = await conn_a.recv()
            self.assertEqual(data, b"gotcha")

            # close
            await conn_a.close()
            await conn_b.close()

    @unittest.skipIf(RUNNING_ON_CI, "CI lacks ipv6")
    @asynctest
    async def test_connect_with_stun_server_ipv6(self):
        async with run_turn_server() as stun_server:
            conn_a = ice.Connection(
                ice_controlling=True,
                stun_server=stun_server.udp_address,
                use_ipv4=False,
                use_ipv6=True,
            )
            conn_b = ice.Connection(
                ice_controlling=False, use_ipv4=False, use_ipv6=True
            )

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we only want host candidates : no STUN for IPv6
            self.assertTrue(len(conn_a.local_candidates) > 0)
            for candidate in conn_a.local_candidates:
                self.assertEqual(candidate.type, "host")

            # connect
            await asyncio.gather(conn_a.connect(), conn_b.connect())

            # send data a -> b
            await conn_a.send(b"howdee")
            data = await conn_b.recv()
            self.assertEqual(data, b"howdee")

            # send data b -> a
            await conn_b.send(b"gotcha")
            data = await conn_a.recv()
            self.assertEqual(data, b"gotcha")

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_connect_with_turn_server_tcp(self):
        async with run_turn_server(users={"foo": "bar"}) as turn_server:
            # create connections
            conn_a = ice.Connection(
                ice_controlling=True,
                turn_server=turn_server.tcp_address,
                turn_username="foo",
                turn_password="bar",
                turn_transport="tcp",
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould have both host and relayed candidates
            self.assertCandidateTypes(conn_a, set(["host", "relay"]))
            self.assertCandidateTypes(conn_b, set(["host"]))

            # the default candidate should be relayed
            candidate = conn_a.get_default_candidate(1)
            self.assertIsNotNone(candidate)
            self.assertEqual(candidate.type, "relay")
            self.assertIsNotNone(candidate.related_address)
            self.assertIsNotNone(candidate.related_port)

            # connect
            await asyncio.gather(conn_a.connect(), conn_b.connect())

            # send data a -> b
            await conn_a.send(b"howdee")
            data = await conn_b.recv()
            self.assertEqual(data, b"howdee")

            # send data b -> a
            await conn_b.send(b"gotcha")
            data = await conn_a.recv()
            self.assertEqual(data, b"gotcha")

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_connect_with_turn_server_udp(self):
        async with run_turn_server(users={"foo": "bar"}) as turn_server:
            # create connections
            conn_a = ice.Connection(
                ice_controlling=True,
                turn_server=turn_server.udp_address,
                turn_username="foo",
                turn_password="bar",
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould have both host and relayed candidates
            self.assertCandidateTypes(conn_a, set(["host", "relay"]))
            self.assertCandidateTypes(conn_b, set(["host"]))

            # the default candidate should be relayed
            candidate = conn_a.get_default_candidate(1)
            self.assertIsNotNone(candidate)
            self.assertEqual(candidate.type, "relay")
            self.assertIsNotNone(candidate.related_address)
            self.assertIsNotNone(candidate.related_port)

            # connect
            await asyncio.gather(conn_a.connect(), conn_b.connect())

            # send data a -> b
            await conn_a.send(b"howdee")
            data = await conn_b.recv()
            self.assertEqual(data, b"howdee")

            # send data b -> a
            await conn_b.send(b"gotcha")
            data = await conn_a.recv()
            self.assertEqual(data, b"gotcha")

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_consent_expired(self):
        # lower consent timer
        ice.CONSENT_FAILURES = 1
        ice.CONSENT_INTERVAL = 1

        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertEqual(len(conn_a._nominated), 1)

        # let consent expire
        await conn_b.close()
        await asyncio.sleep(2)
        self.assertEqual(len(conn_a._nominated), 0)

        # close
        await conn_a.close()

    @asynctest
    async def test_consent_valid(self):
        # lower consent timer
        ice.CONSENT_FAILURES = 1
        ice.CONSENT_INTERVAL = 1

        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())
        self.assertEqual(len(conn_a._nominated), 1)

        # check consent
        await asyncio.sleep(2)
        self.assertEqual(len(conn_a._nominated), 1)

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_set_selected_pair(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # we should only have host candidates
        self.assertCandidateTypes(conn_a, set(["host"]))
        self.assertCandidateTypes(conn_b, set(["host"]))

        # force selected pair
        default_a = conn_a.get_default_candidate(1)
        default_b = conn_a.get_default_candidate(1)
        conn_a.set_selected_pair(1, default_a.foundation, default_b.foundation)
        conn_b.set_selected_pair(1, default_b.foundation, default_a.foundation)

        # send data a -> b
        await conn_a.send(b"howdee")
        data = await conn_b.recv()
        self.assertEqual(data, b"howdee")

        # send data b -> a
        await conn_b.send(b"gotcha")
        data = await conn_a.recv()
        self.assertEqual(data, b"gotcha")

        # close
        await conn_a.close()
        await conn_b.close()

    @asynctest
    async def test_recv_not_connected(self):
        conn_a = ice.Connection(ice_controlling=True)
        with self.assertRaises(ConnectionError) as cm:
            await conn_a.recv()
        self.assertEqual(str(cm.exception), "Cannot receive data, not connected")

    @asynctest
    async def test_recv_connection_lost(self):
        conn_a = ice.Connection(ice_controlling=True)
        conn_b = ice.Connection(ice_controlling=False)

        # invite / accept
        await invite_accept(conn_a, conn_b)

        # connect
        await asyncio.gather(conn_a.connect(), conn_b.connect())

        # disconnect while receiving
        with self.assertRaises(ConnectionError) as cm:
            await asyncio.gather(conn_a.recv(), delay(conn_a.close))
        self.assertEqual(str(cm.exception), "Connection lost while receiving data")

        # close
        await conn_b.close()

    @asynctest
    async def test_send_not_connected(self):
        conn_a = ice.Connection(ice_controlling=True)
        with self.assertRaises(ConnectionError) as cm:
            await conn_a.send(b"howdee")
        self.assertEqual(str(cm.exception), "Cannot send data, not connected")

    @asynctest
    async def test_add_remote_candidate(self):
        conn_a = ice.Connection(ice_controlling=True)

        remote_candidate = Candidate(
            foundation="some-foundation",
            component=1,
            transport="udp",
            priority=1234,
            host="1.2.3.4",
            port=1234,
            type="host",
        )

        # add candidate
        await conn_a.add_remote_candidate(remote_candidate)
        self.assertEqual(len(conn_a.remote_candidates), 1)
        self.assertEqual(conn_a.remote_candidates[0].host, "1.2.3.4")
        self.assertEqual(conn_a._remote_candidates_end, False)

        # end-of-candidates
        await conn_a.add_remote_candidate(None)
        self.assertEqual(len(conn_a.remote_candidates), 1)
        self.assertEqual(conn_a._remote_candidates_end, True)

        # try adding another candidate
        with self.assertRaises(ValueError) as cm:
            await conn_a.add_remote_candidate(remote_candidate)
        self.assertEqual(
            str(cm.exception), "Cannot add remote candidate after end-of-candidates."
        )
        self.assertEqual(len(conn_a.remote_candidates), 1)
        self.assertEqual(conn_a._remote_candidates_end, True)

    @asynctest
    async def test_add_remote_candidate_mdns_bad(self):
        """
        Add an mDNS candidate which cannot be resolved.
        """
        conn_a = ice.Connection(ice_controlling=True)

        await conn_a.add_remote_candidate(
            Candidate(
                foundation="some-foundation",
                component=1,
                transport="udp",
                priority=1234,
                host=mdns.create_mdns_hostname(),
                port=1234,
                type="host",
            )
        )
        self.assertEqual(len(conn_a.remote_candidates), 0)
        self.assertEqual(conn_a._remote_candidates_end, False)

        # close
        await conn_a.close()

    @asynctest
    async def test_add_remote_candidate_mdns_good(self):
        """
        Add an mDNS candidate which can be resolved.
        """
        hostname = mdns.create_mdns_hostname()
        publisher = await mdns.create_mdns_protocol()
        await publisher.publish(hostname, "1.2.3.4")

        conn_a = ice.Connection(ice_controlling=True)

        await conn_a.add_remote_candidate(
            Candidate(
                foundation="some-foundation",
                component=1,
                transport="udp",
                priority=1234,
                host=hostname,
                port=1234,
                type="host",
            )
        )
        self.assertEqual(len(conn_a.remote_candidates), 1)
        self.assertEqual(conn_a.remote_candidates[0].host, "1.2.3.4")
        self.assertEqual(conn_a._remote_candidates_end, False)

        # close
        await conn_a.close()
        await publisher.close()

    @asynctest
    async def test_add_remote_candidate_unknown_type(self):
        conn_a = ice.Connection(ice_controlling=True)

        await conn_a.add_remote_candidate(
            Candidate(
                foundation="some-foundation",
                component=1,
                transport="udp",
                priority=1234,
                host="1.2.3.4",
                port=1234,
                type="bogus",
            )
        )
        self.assertEqual(len(conn_a.remote_candidates), 0)
        self.assertEqual(conn_a._remote_candidates_end, False)

    @mock.patch("asyncio.base_events.BaseEventLoop.create_datagram_endpoint")
    @asynctest
    async def test_gather_candidates_oserror(self, mock_create):
        exc = OSError()
        exc.errno = 99
        exc.strerror = "Cannot assign requested address"
        mock_create.side_effect = exc

        conn = ice.Connection(ice_controlling=True)
        await conn.gather_candidates()
        self.assertEqual(conn.local_candidates, [])

    @asynctest
    async def test_gather_candidates_relay_only_no_servers(self):
        with self.assertRaises(ValueError) as cm:
            ice.Connection(ice_controlling=True, transport_policy=TransportPolicy.RELAY)
        self.assertEqual(
            str(cm.exception),
            "Relay transport policy requires a STUN and/or TURN server.",
        )

    @asynctest
    async def test_gather_candidates_relay_only_with_stun_server(self):
        async with run_turn_server() as stun_server:
            conn_a = ice.Connection(
                ice_controlling=True,
                stun_server=stun_server.udp_address,
                transport_policy=TransportPolicy.RELAY,
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould only have a server-reflexive candidate in connection a
            self.assertCandidateTypes(conn_a, set(["srflx"]))

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_gather_candidates_relay_only_with_turn_server(self):
        async with run_turn_server(users={"foo": "bar"}) as turn_server:
            conn_a = ice.Connection(
                ice_controlling=True,
                turn_server=turn_server.udp_address,
                turn_username="foo",
                turn_password="bar",
                transport_policy=TransportPolicy.RELAY,
            )
            conn_b = ice.Connection(ice_controlling=False)

            # invite / accept
            await invite_accept(conn_a, conn_b)

            # we whould only have a server-reflexive candidate in connection a
            self.assertCandidateTypes(conn_a, set(["relay"]))

            # close
            await conn_a.close()
            await conn_b.close()

    @asynctest
    async def test_repr(self):
        conn = ice.Connection(ice_controlling=True)
        conn._id = 1
        self.assertEqual(repr(conn), "Connection(1)")


class StunProtocolTest(unittest.TestCase):
    @asynctest
    async def test_error_received(self):
        protocol = ice.StunProtocol(None)
        protocol.error_received(OSError("foo"))

    @asynctest
    async def test_repr(self):
        protocol = ice.StunProtocol(None)
        protocol.id = 1
        self.assertEqual(repr(protocol), "protocol(1)")
