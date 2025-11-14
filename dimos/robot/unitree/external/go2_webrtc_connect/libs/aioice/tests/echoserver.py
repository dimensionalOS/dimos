import asyncio
import contextlib


class EchoServerProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        self.transport.sendto(data, addr)


class EchoServer:
    async def close(self):
        self.udp_server.transport.close()

    async def listen(self, host="127.0.0.1", port=0):
        loop = asyncio.get_event_loop()

        # listen for UDP
        transport, self.udp_server = await loop.create_datagram_endpoint(
            EchoServerProtocol, local_addr=(host, port)
        )
        self.udp_address = transport.get_extra_info("sockname")


@contextlib.asynccontextmanager
async def run_echo_server(**kwargs):
    server = EchoServer(**kwargs)
    await server.listen()
    try:
        yield server
    finally:
        await server.close()
