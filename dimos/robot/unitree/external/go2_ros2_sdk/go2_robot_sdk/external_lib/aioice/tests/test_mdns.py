import asyncio
import contextlib
import unittest

from aioice import mdns

from .utils import asynctest


@contextlib.asynccontextmanager
async def querier_and_responder():
    querier = await mdns.create_mdns_protocol()
    responder = await mdns.create_mdns_protocol()

    try:
        yield querier, responder
    finally:
        await querier.close()
        await responder.close()


class MdnsTest(unittest.TestCase):
    @asynctest
    async def test_receive_junk(self):
        async with querier_and_responder() as (querier, _):
            querier.datagram_received(b"junk", None)

    @asynctest
    async def test_resolve_bad(self):
        hostname = mdns.create_mdns_hostname()

        async with querier_and_responder() as (querier, _):
            result = await querier.resolve(hostname)
            self.assertEqual(result, None)

    @asynctest
    async def test_resolve_close(self):
        hostname = mdns.create_mdns_hostname()

        # close the querier while the query is ongoing
        async with querier_and_responder() as (querier, _):
            result = await asyncio.gather(
                querier.resolve(hostname, timeout=None), querier.close()
            )
            self.assertEqual(result, [None, None])

    @asynctest
    async def test_resolve_good_ipv4(self):
        hostaddr = "1.2.3.4"
        hostname = mdns.create_mdns_hostname()

        async with querier_and_responder() as (querier, responder):
            await responder.publish(hostname, hostaddr)

            result = await querier.resolve(hostname)
            self.assertEqual(result, hostaddr)

    @asynctest
    async def test_resolve_good_ipv6(self):
        hostaddr = "::ffff:1.2.3.4"
        hostname = mdns.create_mdns_hostname()

        async with querier_and_responder() as (querier, responder):
            await responder.publish(hostname, hostaddr)

            result = await querier.resolve(hostname)
            self.assertEqual(result, hostaddr)

    @asynctest
    async def test_resolve_simultaneous_bad(self):
        hostname = mdns.create_mdns_hostname()

        async with querier_and_responder() as (querier, _):
            results = await asyncio.gather(
                querier.resolve(hostname), querier.resolve(hostname)
            )
            self.assertEqual(results, [None, None])

    @asynctest
    async def test_resolve_simultaneous_good(self):
        hostaddr = "1.2.3.4"
        hostname = mdns.create_mdns_hostname()

        async with querier_and_responder() as (querier, responder):
            await responder.publish(hostname, hostaddr)

            results = await asyncio.gather(
                querier.resolve(hostname), querier.resolve(hostname)
            )
            self.assertEqual(results, [hostaddr, hostaddr])
