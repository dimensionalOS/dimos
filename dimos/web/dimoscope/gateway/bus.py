#!/usr/bin/env python3
# dimoscope bus: the single in-process tap every transport reads from. Ingests both backends at once
# (Zenoh declare_subscriber("**"); LCM UDP-multicast with LC03→LC02 reassembly) and normalises each sample
# to Sample(topic, type, lc02, payload). lc02 is the self-describing "LC02"<seq><channel>\0<payload> packet
# the browser decodes; payload is the bare lcm_encode bytes (fed to Image.lcm_decode by the media plane).
# Consumer callbacks run on the loop and must be cheap (enqueue, don't await); Zenoh delivers on its own
# thread → call_soon_threadsafe, the LCM tap already runs on the loop.
from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass
import socket
import struct

from dimos.utils.logging_config import setup_logger

LC02 = 0x4C433032  # "LC02" — a single, complete packet
LC03 = 0x4C433033  # "LC03" — one fragment of a larger message
FRAG_HDR = 20  # LC03 fragment header length

logger = setup_logger()


@dataclass(frozen=True)
class Sample:
    topic: str  # canonical topic, leading slash, no backend prefix (e.g. "/lidar")
    type: str  # message type (e.g. "sensor_msgs.PointCloud2"); "?" if unknown
    lc02: bytes  # "LC02"<seq><channel>\0<payload> — the frame the browser decodes
    payload: bytes  # bare message bytes (lcm_encode output)


Consumer = Callable[[Sample], None]


def _canonical(topic: str) -> str:
    """Drop the Zenoh `dimos` namespace so the browser sees the same names LCM uses ("/lidar")."""
    topic = topic if topic.startswith("/") else "/" + topic
    return topic[len("/dimos") :] if topic.startswith("/dimos/") else topic


class Bus:
    def __init__(self) -> None:
        self._loop: asyncio.AbstractEventLoop | None = None
        self._seq = 0
        self.topics: dict[str, str] = {}  # topic -> type (discovery registry)
        self._consumers: list[Consumer] = []
        self._on_topic: list[Callable[[str, str], None]] = []
        self._zenoh_session = None
        self._lcm_transport = None
        # LC03 reassembly, keyed by sequence number → partial message
        self._pending: dict[int, _Assembly] = {}

    def subscribe(self, cb: Consumer) -> Callable[[], None]:
        """Register a fan-out consumer. Returns an unsubscribe callable."""
        self._consumers.append(cb)
        return lambda: self._consumers.remove(cb) if cb in self._consumers else None

    def on_new_topic(self, cb: Callable[[str, str], None]) -> None:
        """Notified (topic, type) the first time each topic is seen — for live discovery."""
        self._on_topic.append(cb)

    def topic_list(self) -> list[dict]:
        return [{"topic": t, "type": ty} for t, ty in self.topics.items()]

    def _make_lc02(self, channel: str, payload: bytes) -> bytes:
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return struct.pack(">II", LC02, self._seq) + channel.encode() + b"\x00" + payload

    # publish (loop thread)
    def _publish(self, topic: str, typ: str, lc02: bytes, payload: bytes) -> None:
        if topic not in self.topics:
            self.topics[topic] = typ
            for cb in self._on_topic:
                cb(topic, typ)
        s = Sample(topic, typ, lc02, payload)
        for cb in self._consumers:
            cb(s)  # cheap + sync (enqueue); never await here

    def start_zenoh(self, key: str = "**") -> None:
        self._loop = asyncio.get_running_loop()
        import zenoh

        def on_sample(sample) -> None:  # runs on a Zenoh thread
            k = str(sample.key_expr)
            try:
                payload = sample.payload.to_bytes()
            except Exception:
                payload = bytes(sample.payload)
            base, _, typ = k.rpartition("/")  # dimos/lidar/sensor_msgs.PointCloud2
            topic = _canonical(base)
            lc02 = self._make_lc02(f"{topic}#{typ}", payload)
            loop = self._loop
            if loop is not None:
                loop.call_soon_threadsafe(self._publish, topic, typ, lc02, payload)

        self._zenoh_session = zenoh.open(zenoh.Config())
        self._zenoh_session.declare_subscriber(key, on_sample)
        logger.info("zenoh subscriber declared", key=key)

    # LCM ingest: UDP multicast + LC03→LC02 reassembly
    async def start_lcm(self, host: str = "239.255.76.67", port: int = 7667) -> None:
        self._loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        try:
            sock.bind(("", port))
            mreq = struct.pack("=4sl", socket.inet_aton(host), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        except OSError as e:
            logger.warning("lcm tap disabled", host=host, port=port, error=str(e))
            sock.close()
            return
        sock.setblocking(False)
        await self._loop.create_datagram_endpoint(lambda: _LcmProtocol(self), sock=sock)
        logger.info("lcm tap listening", host=host, port=port)

    def _on_lcm_datagram(self, data: bytes) -> None:  # runs on the loop
        pkt = self._reassemble(data)
        if pkt is None:
            return
        meta = _parse_channel(pkt)
        if meta is None:
            return
        topic, typ = meta
        # `pkt` is already an LC02 packet; recover the bare payload after "<channel>\0".
        nul = pkt.index(0, 8)
        payload = pkt[nul + 1 :]
        self._publish(topic, typ, pkt, payload)

    def _reassemble(self, b: bytes) -> bytes | None:
        """LC03 fragments → a synthetic LC02 packet; pass LC02 through."""
        if len(b) < 8:
            return None
        magic = struct.unpack(">I", b[0:4])[0]
        if magic == LC02:
            return b
        if magic != LC03:
            return None
        seqno, msg_size, frag_off = struct.unpack(">III", b[4:16])
        frag_no, frags = struct.unpack(">HH", b[16:20])
        a = self._pending.get(seqno)
        if a is None:
            a = _Assembly(bytearray(msg_size), "", set(), frags)
            self._pending[seqno] = a
        data_start = FRAG_HDR
        if frag_no == 0:
            end = FRAG_HDR
            while end < len(b) and b[end] != 0:
                end += 1
            a.channel = b[FRAG_HDR:end].decode("utf-8", "replace")
            data_start = end + 1
        a.buf[frag_off : frag_off + (len(b) - data_start)] = b[data_start:]
        a.got.add(frag_no)
        if len(a.got) == a.total and a.channel:
            self._pending.pop(seqno, None)
            return self._make_lc02(a.channel, bytes(a.buf))
        if len(self._pending) > 256:
            self._pending.clear()  # leak guard
        return None

    def close(self) -> None:
        if self._zenoh_session is not None:
            try:
                self._zenoh_session.close()
            except Exception:
                pass


@dataclass
class _Assembly:
    buf: bytearray
    channel: str
    got: set
    total: int


class _LcmProtocol(asyncio.DatagramProtocol):
    def __init__(self, bus: Bus) -> None:
        self._bus = bus

    def datagram_received(self, data: bytes, addr) -> None:
        self._bus._on_lcm_datagram(data)


def _parse_channel(pkt: bytes) -> tuple[str, str] | None:
    """Pull ("<topic>", "<type>") out of an LC02 packet's "<topic>#<type>\\0" channel."""
    if len(pkt) < 8:
        return None
    try:
        nul = pkt.index(0, 8)
    except ValueError:
        return None
    channel = pkt[8:nul].decode("utf-8", "replace")
    h = channel.find("#")
    if h >= 0:
        return channel[:h], channel[h + 1 :]
    return channel, "?"
