package com.dimensional.mini4pro.video

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

/**
 * The passthrough sink: frame in, RTP datagrams out, counters that tell the truth.
 *
 * The behaviour that matters here is not "it sends packets" — it is what happens
 * on the bad days: a receiver that joins after the first keyframe, a socket that
 * refuses, a producer faster than the link. All three are provable on the laptop,
 * and one of them (the late joiner) is the difference between "video works" and
 * "QGC shows a black rectangle forever".
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | parameter-set cache never populated | 2 |
 *  | cached SPS/PPS injected on every frame, not only bare IDRs | 3 |
 *  | injection skipped entirely (late joiner gets nothing) | 2 |
 *  | `onEncodedFrame` keeps MSDK's array instead of copying | 2 |
 *  | send exception propagated instead of counted | 2 |
 *  | `framesSent` incremented even when a send threw | 2 |
 *  | queue overflow blocks instead of dropping oldest | 1 |
 *  | drop not counted | 1 |
 *
 * Extended 2026-07-26 with the socket binder and the counter carry-over, both
 * needed to wire this into the app. Mutation-checked the same way:
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M20 | `udp` ignores the `bind` argument (socket follows the default route) | 2 |
 *  | M21 | a throwing binder leaks the socket instead of closing it | 1 |
 *  | M22 | `Counters.plus` returns the live half, discarding the carry | 3 |
 *  | M23 | `Counters.plus` falls back to the older `lastError` when the live half has none | 1 |
 *  | M24 | `Counters.plus` drops `parameterSetsInjected` from the sum | 1 |
 *
 * M23 is the mutation that was written *first*, as the implementation, and the
 * test disagreed with it. Settling that argument is what produced the asymmetry
 * documented on `Counters.plus`: counts are a record and add, `lastError`
 * describes the socket that exists now and does not.
 */
class RtpVideoSinkTest {

    private class Capture : RtpVideoSink.Transport {
        val packets = mutableListOf<ByteArray>()
        var failNext = false
        override fun send(packet: ByteArray, length: Int) {
            if (failNext) throw java.io.IOException("network unreachable")
            packets += packet.copyOf(length)
        }
    }

    private fun nal(type: Int, len: Int): ByteArray =
        ByteArray(len) { i -> if (i == 0) (0x60 or type).toByte() else (i and 0x7F).toByte() }

    private fun annexB(vararg nals: ByteArray): ByteArray {
        var out = ByteArray(0)
        for (n in nals) out += byteArrayOf(0, 0, 0, 1) + n
        return out
    }

    private fun info(keyFrame: Boolean = false, ptsMs: Long = 0) = RawFrameInfo(
        mime = "H264", width = 1920, height = 1080, frameRate = 30,
        keyFrame = keyFrame, presentationTimeMs = ptsMs,
    )

    private fun sink(t: RtpVideoSink.Transport, mtu: Int = RtpH264.DEFAULT_MTU) =
        RtpVideoSink(transport = t, mtu = mtu, inlineForTests = true)

    private fun payloadOf(p: ByteArray) = p.copyOfRange(RtpH264.RTP_HEADER_LEN, p.size)
    private fun nalTypeOfPayload(p: ByteArray) = payloadOf(p)[0].toInt() and 0x1F

    // ---- the happy path -------------------------------------------------------

    @Test
    fun `a frame becomes RTP packets carrying the original bytes`() {
        val cap = Capture()
        val s = sink(cap)
        val body = nal(1, 40)
        val frame = annexB(body)
        s.onEncodedFrame(frame, 0, frame.size, info())

        assertEquals(1, cap.packets.size)
        assertArrayEquals(body, payloadOf(cap.packets[0]))
        val c = s.counters()
        assertEquals(1, c.framesIn)
        assertEquals(1, c.framesSent)
        assertEquals(1, c.packetsSent)
        assertEquals(cap.packets[0].size.toLong(), c.bytesSent)
        assertEquals(0, c.sendErrors)
        assertNull(c.lastError)
    }

    @Test
    fun `the offset and length MSDK gives are respected`() {
        val cap = Capture()
        val s = sink(cap)
        val inner = annexB(nal(1, 30))
        val padded = ByteArray(7) { 0x55 } + inner + ByteArray(9) { 0x55 }
        s.onEncodedFrame(padded, 7, inner.size, info())
        assertEquals(1, cap.packets.size)
        assertEquals(30, payloadOf(cap.packets[0]).size)
    }

    @Test
    fun `MSDK's buffer is copied at hand-off, not read later by the sender thread`() {
        // The RawFrameSink contract says MSDK's array is valid only for the
        // duration of the call. The queued path is where that bites: if the Job
        // holds MSDK's array instead of a copy, the sender thread reads whatever
        // MSDK has since written into it. The latch makes that race deterministic
        // — the caller overwrites the buffer *before* the sender is allowed to run.
        val released = java.util.concurrent.CountDownLatch(1)
        val sent = java.util.concurrent.CountDownLatch(1)
        val seen = mutableListOf<ByteArray>()
        val gated = RtpVideoSink.Transport { packet, length ->
            released.await()
            synchronized(seen) { seen += packet.copyOf(length) }
            sent.countDown()
        }
        val s = RtpVideoSink(transport = gated)
        try {
            val body = nal(1, 40)
            val frame = annexB(body)
            s.onEncodedFrame(frame, 0, frame.size, info())

            frame.fill(0x7E, 4, frame.size) // MSDK reuses its buffer
            released.countDown()
            assertTrue("sender never ran", sent.await(3, java.util.concurrent.TimeUnit.SECONDS))

            assertArrayEquals(body, payloadOf(synchronized(seen) { seen.single() }))
        } finally {
            s.close()
        }
    }

    @Test
    fun `presentation time reaches the wire on the ninety kilohertz clock`() {
        val cap = Capture()
        val s = sink(cap)
        val frame = annexB(nal(1, 40))
        s.onEncodedFrame(frame, 0, frame.size, info(ptsMs = 2000))
        val p = cap.packets[0]
        val ts = ((p[4].toLong() and 0xFF) shl 24) or ((p[5].toLong() and 0xFF) shl 16) or
            ((p[6].toLong() and 0xFF) shl 8) or (p[7].toLong() and 0xFF)
        assertEquals(180_000L, ts)
    }

    @Test
    fun `an unparseable frame is counted in but sends nothing`() {
        val cap = Capture()
        val s = sink(cap)
        val junk = ByteArray(50) { 0x33 } // no start code anywhere
        s.onEncodedFrame(junk, 0, junk.size, info())
        assertEquals(0, cap.packets.size)
        assertEquals(1, s.counters().framesIn)
        assertEquals(0, s.counters().framesSent)
    }

    @Test
    fun `an empty frame is ignored entirely`() {
        val cap = Capture()
        val s = sink(cap)
        s.onEncodedFrame(ByteArray(0), 0, 0, info())
        assertEquals(0, s.counters().framesIn)
    }

    // ---- the late joiner ------------------------------------------------------

    @Test
    fun `a keyframe that carries its own parameter sets is passed through untouched`() {
        val cap = Capture()
        val s = sink(cap)
        val frame = annexB(nal(RtpH264.NAL_SPS, 12), nal(RtpH264.NAL_PPS, 8), nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(frame, 0, frame.size, info(keyFrame = true))
        assertEquals(3, cap.packets.size)
        assertEquals(0, s.counters().parameterSetsInjected)
    }

    @Test
    fun `a bare keyframe gets the cached parameter sets prepended`() {
        // This is the late-joiner case: QGC attaches after the stream started, the
        // aircraft sends an IDR without repeating SPS/PPS, and without this the
        // decoder never has enough to start. It looks exactly like "no video".
        val cap = Capture()
        val s = sink(cap)

        val withSets = annexB(nal(RtpH264.NAL_SPS, 12), nal(RtpH264.NAL_PPS, 8), nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(withSets, 0, withSets.size, info(keyFrame = true))
        cap.packets.clear()

        val bare = annexB(nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(bare, 0, bare.size, info(keyFrame = true, ptsMs = 100))

        assertEquals(3, cap.packets.size)
        assertEquals(RtpH264.NAL_SPS, nalTypeOfPayload(cap.packets[0]))
        assertEquals(RtpH264.NAL_PPS, nalTypeOfPayload(cap.packets[1]))
        assertEquals(RtpH264.NAL_IDR, nalTypeOfPayload(cap.packets[2]))
        assertEquals(1, s.counters().parameterSetsInjected)
    }

    @Test
    fun `injected parameter sets share the keyframe's timestamp and do not set the marker`() {
        val cap = Capture()
        val s = sink(cap)
        val withSets = annexB(nal(RtpH264.NAL_SPS, 12), nal(RtpH264.NAL_PPS, 8), nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(withSets, 0, withSets.size, info(keyFrame = true))
        cap.packets.clear()
        val bare = annexB(nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(bare, 0, bare.size, info(keyFrame = true, ptsMs = 100))

        fun ts(p: ByteArray) = ((p[4].toLong() and 0xFF) shl 24) or ((p[5].toLong() and 0xFF) shl 16) or
            ((p[6].toLong() and 0xFF) shl 8) or (p[7].toLong() and 0xFF)
        val expected = RtpH264.timestamp90k(100)
        cap.packets.forEach { assertEquals(expected, ts(it)) }
        assertTrue("only the last packet may carry the marker", (cap.packets[0][1].toInt() and 0x80) == 0)
        assertTrue((cap.packets[1][1].toInt() and 0x80) == 0)
        assertTrue((cap.packets[2][1].toInt() and 0x80) != 0)
    }

    @Test
    fun `a non-keyframe never triggers injection`() {
        val cap = Capture()
        val s = sink(cap)
        val withSets = annexB(nal(RtpH264.NAL_SPS, 12), nal(RtpH264.NAL_PPS, 8), nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(withSets, 0, withSets.size, info(keyFrame = true))
        cap.packets.clear()
        val inter = annexB(nal(1, 60))
        s.onEncodedFrame(inter, 0, inter.size, info(ptsMs = 33))
        assertEquals(1, cap.packets.size)
        assertEquals(0, s.counters().parameterSetsInjected)
    }

    @Test
    fun `a bare keyframe before any parameter set has been seen is still sent`() {
        // Nothing to inject yet: send what we have rather than dropping the frame.
        val cap = Capture()
        val s = sink(cap)
        val bare = annexB(nal(RtpH264.NAL_IDR, 60))
        s.onEncodedFrame(bare, 0, bare.size, info(keyFrame = true))
        assertEquals(1, cap.packets.size)
        assertEquals(0, s.counters().parameterSetsInjected)
    }

    // ---- failure surfaces -----------------------------------------------------

    @Test
    fun `a send failure is counted and described, not thrown`() {
        val cap = Capture()
        val s = sink(cap)
        cap.failNext = true
        val frame = annexB(nal(1, 40))
        s.onEncodedFrame(frame, 0, frame.size, info()) // must not throw

        val c = s.counters()
        assertEquals(1, c.framesIn)
        assertEquals(0, c.framesSent)
        assertEquals(1, c.sendErrors)
        assertNotNull(c.lastError)
        assertTrue(c.lastError!!, c.lastError!!.contains("network unreachable"))
    }

    @Test
    fun `the sink keeps working after a failed send`() {
        val cap = Capture()
        val s = sink(cap)
        val frame = annexB(nal(1, 40))
        cap.failNext = true
        s.onEncodedFrame(frame, 0, frame.size, info())
        cap.failNext = false
        s.onEncodedFrame(frame, 0, frame.size, info())
        assertEquals(1, cap.packets.size)
        assertEquals(1, s.counters().framesSent)
    }

    // ---- back pressure --------------------------------------------------------

    @Test
    fun `a full queue drops the oldest frame and counts it instead of blocking`() {
        // Blocking here would stall MSDK's decode path for every consumer.
        val blocked = RtpVideoSink.Transport { _, _ -> error("must not be reached") }
        val s = RtpVideoSink(transport = blocked, queueDepth = 2)
        try {
            val frame = annexB(nal(1, 40))
            repeat(50) { s.onEncodedFrame(frame, 0, frame.size, info(ptsMs = it.toLong())) }
            assertEquals(50, s.counters().framesIn)
            assertTrue("expected drops, got ${s.counters().framesDropped}", s.counters().framesDropped > 0)
        } finally {
            s.close()
        }
    }

    // ---- the real socket ------------------------------------------------------

    @Test
    fun `the UDP transport delivers a real datagram on loopback`() {
        // Proves the whole path end to end without an aircraft: Annex-B in one
        // side, an RTP datagram out of a real socket on the other.
        DatagramSocket(0, InetAddress.getByName("127.0.0.1")).use { receiver ->
            receiver.soTimeout = 3000
            val s = RtpVideoSink(
                transport = RtpVideoSink.udp("127.0.0.1", receiver.localPort),
                inlineForTests = true,
            )
            try {
                val body = nal(RtpH264.NAL_IDR, 100)
                val frame = annexB(body)
                s.onEncodedFrame(frame, 0, frame.size, info(keyFrame = true, ptsMs = 1000))

                val buf = ByteArray(2048)
                val dp = DatagramPacket(buf, buf.size)
                receiver.receive(dp)

                assertEquals(RtpH264.RTP_HEADER_LEN + body.size, dp.length)
                assertEquals(0x80.toByte(), buf[0])
                assertEquals(RtpH264.PAYLOAD_TYPE, buf[1].toInt() and 0x7F)
                assertArrayEquals(body, buf.copyOfRange(RtpH264.RTP_HEADER_LEN, dp.length))
            } finally {
                s.close()
            }
        }
    }

    @Test
    fun `the sender thread delivers frames handed over from another thread`() {
        DatagramSocket(0, InetAddress.getByName("127.0.0.1")).use { receiver ->
            receiver.soTimeout = 3000
            val s = RtpVideoSink(transport = RtpVideoSink.udp("127.0.0.1", receiver.localPort))
            try {
                val frame = annexB(nal(1, 60))
                s.onEncodedFrame(frame, 0, frame.size, info())
                val buf = ByteArray(2048)
                receiver.receive(DatagramPacket(buf, buf.size))
                assertEquals(RtpH264.PAYLOAD_TYPE, buf[1].toInt() and 0x7F)
            } finally {
                s.close()
            }
        }
    }

    // ---- binding the socket to a network ---------------------------------------

    /**
     * `wifi-fix.md` gotcha #2. An unbound socket follows Android's *default*
     * network, and Android 16 moves that to cellular whenever WiFi fails the
     * router's validation — at which point the aircraft's camera leaves over the
     * operator's mobile data and never arrives. The MAVLink socket has been bound
     * for exactly this reason since `WifiBind`; the video socket was written
     * without it and would have inherited the entire bug.
     */
    @Test
    fun `the binder is handed the socket before it is used`() {
        var bound: DatagramSocket? = null
        RtpVideoSink.udp("127.0.0.1", 5600) { bound = it }.use {
            assertNotNull("bind was never called", bound)
            assertTrue("socket was already closed when bound", !bound!!.isClosed)
        }
    }

    @Test
    fun `a binder that throws closes the socket rather than leaking it`() {
        // The caller only ever learns about a Transport this function returned, so
        // a socket abandoned on the failure path is a file descriptor nobody can
        // ever close — and video retries once a second.
        var leaked: DatagramSocket? = null
        try {
            RtpVideoSink.udp("127.0.0.1", 5600) { leaked = it; throw SecurityException("denied") }
            throw AssertionError("expected the bind failure to propagate")
        } catch (e: java.io.IOException) {
            assertTrue(e.message!!, e.message!!.contains("binding the RTP socket"))
        }
        assertNotNull(leaked)
        assertTrue("the socket was left open", leaked!!.isClosed)
    }

    // ---- counters across a socket rebuild --------------------------------------

    /**
     * `rtpPkts` is one of the two numbers docs/video.md's failure table is read
     * through, and every row that mentions it is really about whether it is
     * *rising*. A WiFi flap rebuilds the socket; if the counter went back to zero
     * with it, the screen would report `rtpPkts=0` for a path that had been
     * working perfectly a second earlier.
     */
    @Test
    fun `an earlier sockets totals are carried into the new one`() {
        val old = RtpVideoSink.Counters(
            framesIn = 100, framesSent = 98, framesDropped = 2, packetsSent = 400,
            bytesSent = 500_000, sendErrors = 1, parameterSetsInjected = 3,
        )
        val fresh = RtpVideoSink.Counters(
            framesIn = 10, framesSent = 10, packetsSent = 40, bytesSent = 50_000,
            parameterSetsInjected = 1,
        )
        val sum = old + fresh
        assertEquals(110, sum.framesIn)
        assertEquals(108, sum.framesSent)
        assertEquals(2, sum.framesDropped)
        assertEquals(440, sum.packetsSent)
        assertEquals(550_000, sum.bytesSent)
        assertEquals(1, sum.sendErrors)
        assertEquals(4, sum.parameterSetsInjected)
    }

    /**
     * The one field that does not add up, deliberately. A dead socket's error
     * would put "check the target is routable" on screen beside a target the live
     * socket is reaching perfectly. `sendErrors` is where that history belongs,
     * and it does add up.
     */
    @Test
    fun `the live half owns the last error, including when it has none`() {
        val old = RtpVideoSink.Counters(sendErrors = 5, lastError = "IOException: unreachable")
        val healed = old + RtpVideoSink.Counters(packetsSent = 9)
        assertNull("a closed socket's error outlived it", healed.lastError)
        // …but the fact that five sends failed is not erased.
        assertEquals(5, healed.sendErrors)
        assertEquals("boom", (old + RtpVideoSink.Counters(lastError = "boom")).lastError)
    }

    @Test
    fun `folding in an empty live half leaves every count alone`() {
        val c = RtpVideoSink.Counters(framesIn = 7, packetsSent = 21, sendErrors = 2)
        assertEquals(c, c + RtpVideoSink.Counters())
    }
}
