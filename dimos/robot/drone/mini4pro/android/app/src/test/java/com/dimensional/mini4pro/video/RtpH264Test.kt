package com.dimensional.mini4pro.video

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The passthrough path's arithmetic, pinned on the laptop.
 *
 * Every byte QGroundControl's GStreamer pipeline reads is produced here, so a
 * mistake in this file is indistinguishable from "the aircraft is not sending
 * video" — the most expensive kind of bug this milestone can have. None of it
 * needs an aircraft, so none of it should be discovered next to one.
 *
 * The wire format is RFC 6184. Where a test encodes a constant (packet type 28,
 * the 0x80/0x40 start/end bits, the 90 kHz clock) it is quoting the RFC, not the
 * implementation.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `startCodeLengthAt` accepts only 4-byte start codes | 2 |
 *  | `startCodeLengthAt` accepts only 3-byte start codes | **0 — equivalent** |
 *  | trailing-zero trim removed from `addNal` | 1 |
 *  | FU-A copies the original NAL header byte into the payload | 1 |
 *  | FU-A end bit never set (`0x40` dropped) | 1 |
 *  | FU-A start bit never set (`0x80` dropped) | 1 |
 *  | marker bit set on every packet, not just the last | 1 |
 *  | sequence number not incremented | 1 |
 *  | sequence number not masked to 16 bits | 0 → **1** after adding a test |
 *  | `timestamp90k` multiplies by 1000 instead of 90 | 1 |
 *  | single-NAL threshold uses `mtu` instead of `mtu - RTP_HEADER_LEN` | 1 |
 *  | SSRC written little-endian | 1 |
 *
 * Two of those deserve a note rather than a number.
 *
 * **Dropping the 4-byte start-code branch changes nothing, and that is correct.**
 * `00 00 00 01` contains `00 00 01` at its second byte, so a scanner that only
 * knows the 3-byte form still finds the same NAL start — it just walks one extra
 * byte to get there, and the trailing-zero trim keeps the previous NAL clean.
 * The mutant is genuinely equivalent; no test was added, because there is no
 * behaviour to pin.
 *
 * **An unmasked sequence counter is invisible on the wire**, because the header
 * bytes truncate to 16 bits either way — it only shows up when the counter
 * eventually overflows `Int`. That was a real gap, and
 * `the sequence counter itself stays inside sixteen bits` now closes it.
 */
class RtpH264Test {

    // ---- Annex-B scanning -----------------------------------------------------

    private fun annexB(vararg nals: ByteArray): ByteArray {
        val out = ArrayList<Byte>()
        for (n in nals) {
            out += listOf<Byte>(0, 0, 0, 1)
            n.forEach { out += it }
        }
        return out.toByteArray()
    }

    /** A NAL of [len] bytes whose header byte encodes [type] with NRI=3. */
    private fun nal(type: Int, len: Int): ByteArray =
        ByteArray(len) { i -> if (i == 0) (0x60 or type).toByte() else (i and 0x7F).toByte() }

    @Test
    fun `finds NAL units separated by four-byte start codes`() {
        val buf = annexB(nal(RtpH264.NAL_SPS, 10), nal(RtpH264.NAL_PPS, 6), nal(RtpH264.NAL_IDR, 20))
        val nals = RtpH264.nalUnits(buf)
        assertEquals(listOf(10, 6, 20), nals.map { it.length })
        assertEquals(
            listOf(RtpH264.NAL_SPS, RtpH264.NAL_PPS, RtpH264.NAL_IDR),
            nals.map { RtpH264.nalType(buf, it) },
        )
    }

    @Test
    fun `finds NAL units separated by three-byte start codes`() {
        // DJI's own parser accepts both lengths (NalUnitUtil.getStartCodeLength),
        // so assuming one is a way to see zero video on hardware and nowhere else.
        val buf = byteArrayOf(0, 0, 1) + nal(RtpH264.NAL_SPS, 5) +
            byteArrayOf(0, 0, 1) + nal(1, 7)
        val nals = RtpH264.nalUnits(buf)
        assertEquals(listOf(5, 7), nals.map { it.length })
    }

    @Test
    fun `handles a buffer mixing three and four byte start codes`() {
        val buf = byteArrayOf(0, 0, 0, 1) + nal(RtpH264.NAL_SPS, 4) +
            byteArrayOf(0, 0, 1) + nal(RtpH264.NAL_PPS, 4) +
            byteArrayOf(0, 0, 0, 1) + nal(RtpH264.NAL_IDR, 9)
        assertEquals(listOf(4, 4, 9), RtpH264.nalUnits(buf).map { it.length })
    }

    @Test
    fun `trailing zero padding is not part of the NAL`() {
        val buf = annexB(nal(1, 8)) + byteArrayOf(0, 0, 0)
        assertEquals(listOf(8), RtpH264.nalUnits(buf).map { it.length })
    }

    @Test
    fun `respects offset and length rather than scanning the whole buffer`() {
        // MSDK hands us (array, offset, length); reading outside it is reading
        // someone else's frame.
        val inner = annexB(nal(1, 6))
        val buf = ByteArray(5) { 0x7F } + inner + ByteArray(5) { 0x7F }
        val nals = RtpH264.nalUnits(buf, 5, inner.size)
        assertEquals(1, nals.size)
        assertEquals(6, nals[0].length)
    }

    @Test
    fun `a buffer with no start code yields nothing instead of throwing`() {
        assertEquals(emptyList<RtpH264.Nal>(), RtpH264.nalUnits(byteArrayOf(9, 9, 9, 9)))
        assertEquals(emptyList<RtpH264.Nal>(), RtpH264.nalUnits(ByteArray(0)))
    }

    @Test
    fun `out of range slices yield nothing rather than an exception`() {
        val buf = annexB(nal(1, 4))
        assertEquals(emptyList<RtpH264.Nal>(), RtpH264.nalUnits(buf, 0, buf.size + 10))
        assertEquals(emptyList<RtpH264.Nal>(), RtpH264.nalUnits(buf, -1, 4))
    }

    @Test
    fun `parameter sets are recognised and slices are not`() {
        assertTrue(RtpH264.isParameterSet(RtpH264.NAL_SPS))
        assertTrue(RtpH264.isParameterSet(RtpH264.NAL_PPS))
        listOf(1, RtpH264.NAL_IDR, RtpH264.NAL_SEI, RtpH264.NAL_AUD).forEach {
            assertTrue("type $it must not count as a parameter set", !RtpH264.isParameterSet(it))
        }
    }

    // ---- RTP header -----------------------------------------------------------

    private fun seqOf(p: ByteArray) = ((p[2].toInt() and 0xFF) shl 8) or (p[3].toInt() and 0xFF)
    private fun tsOf(p: ByteArray): Long =
        ((p[4].toLong() and 0xFF) shl 24) or ((p[5].toLong() and 0xFF) shl 16) or
            ((p[6].toLong() and 0xFF) shl 8) or (p[7].toLong() and 0xFF)

    private fun ssrcOf(p: ByteArray): Int =
        ((p[8].toInt() and 0xFF) shl 24) or ((p[9].toInt() and 0xFF) shl 16) or
            ((p[10].toInt() and 0xFF) shl 8) or (p[11].toInt() and 0xFF)

    private fun markerOf(p: ByteArray) = (p[1].toInt() and 0x80) != 0
    private fun ptOf(p: ByteArray) = p[1].toInt() and 0x7F

    @Test
    fun `header carries version two, the payload type and the ssrc`() {
        val p = RtpH264.Packetizer(ssrc = 0x11223344)
        val buf = annexB(nal(1, 20))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        assertEquals(1, out.size)
        assertEquals(0x80.toByte(), out[0][0]) // V=2, no padding, no extension, CC=0
        assertEquals(RtpH264.PAYLOAD_TYPE, ptOf(out[0]))
        assertEquals(0x11223344, ssrcOf(out[0]))
    }

    @Test
    fun `sequence numbers increase by one per packet and wrap at sixteen bits`() {
        val p = RtpH264.Packetizer(ssrc = 1)
        val buf = annexB(nal(1, 20))
        val first = p.packetizeAnnexB(buf, 0, buf.size, 0).single()
        val second = p.packetizeAnnexB(buf, 0, buf.size, 0).single()
        assertEquals(seqOf(first) + 1, seqOf(second))

        repeat(0x1_0000 - 2) { p.packetizeAnnexB(buf, 0, buf.size, 0) }
        val wrapped = p.packetizeAnnexB(buf, 0, buf.size, 0).single()
        assertEquals(seqOf(first), seqOf(wrapped))
    }

    @Test
    fun `the sequence counter itself stays inside sixteen bits`() {
        // The header bytes truncate anyway, so an unmasked counter looks correct
        // on the wire right up until it overflows Int. Pin the counter, not just
        // the bytes.
        val p = RtpH264.Packetizer(ssrc = 1)
        val buf = annexB(nal(1, 20))
        repeat(0x1_0000 + 5) { p.packetizeAnnexB(buf, 0, buf.size, 0) }
        assertTrue("sequence escaped 16 bits: ${p.sequence}", p.sequence in 0..0xFFFF)
        assertEquals(5, p.sequence)
    }

    @Test
    fun `presentation time is converted to the ninety kilohertz clock`() {
        // RFC 6184 s8.1 fixes H.264's RTP clock at 90 kHz: 1 ms = 90 ticks.
        assertEquals(0L, RtpH264.timestamp90k(0))
        assertEquals(90L, RtpH264.timestamp90k(1))
        assertEquals(90_000L, RtpH264.timestamp90k(1000))
        val p = RtpH264.Packetizer(ssrc = 1)
        val buf = annexB(nal(1, 20))
        assertEquals(
            90_000L,
            tsOf(p.packetizeAnnexB(buf, 0, buf.size, RtpH264.timestamp90k(1000)).single()),
        )
    }

    @Test
    fun `the timestamp stays inside thirty two bits`() {
        val huge = RtpH264.timestamp90k(Long.MAX_VALUE / 90)
        assertTrue("timestamp $huge does not fit 32 bits", huge <= 0xFFFF_FFFFL && huge >= 0)
    }

    // ---- single NAL unit packets ---------------------------------------------

    @Test
    fun `a NAL that fits the MTU is sent verbatim as the payload`() {
        val p = RtpH264.Packetizer(ssrc = 1, mtu = 100)
        val body = nal(RtpH264.NAL_SPS, 20)
        val buf = annexB(body)
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        assertEquals(1, out.size)
        assertEquals(RtpH264.RTP_HEADER_LEN + 20, out[0].size)
        // Payload is the NAL including its header byte, with no start code.
        assertArrayEquals(body, out[0].copyOfRange(RtpH264.RTP_HEADER_LEN, out[0].size))
    }

    @Test
    fun `a NAL exactly filling the MTU is not fragmented`() {
        val mtu = 100
        val p = RtpH264.Packetizer(ssrc = 1, mtu = mtu)
        val buf = annexB(nal(1, mtu - RtpH264.RTP_HEADER_LEN))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        assertEquals(1, out.size)
        assertEquals(mtu, out[0].size)
    }

    @Test
    fun `one byte over the MTU is fragmented`() {
        val mtu = 100
        val p = RtpH264.Packetizer(ssrc = 1, mtu = mtu)
        val buf = annexB(nal(1, mtu - RtpH264.RTP_HEADER_LEN + 1))
        assertTrue(p.packetizeAnnexB(buf, 0, buf.size, 0).size > 1)
    }

    @Test
    fun `no packet ever exceeds the MTU`() {
        val mtu = 300
        val p = RtpH264.Packetizer(ssrc = 1, mtu = mtu)
        val buf = annexB(nal(RtpH264.NAL_IDR, 5000), nal(1, 17), nal(1, 4000))
        p.packetizeAnnexB(buf, 0, buf.size, 0).forEach {
            assertTrue("packet of ${it.size} exceeds mtu $mtu", it.size <= mtu)
        }
    }

    // ---- FU-A -----------------------------------------------------------------

    @Test
    fun `fragments reassemble to the original NAL`() {
        val mtu = 64
        val p = RtpH264.Packetizer(ssrc = 1, mtu = mtu)
        val body = nal(RtpH264.NAL_IDR, 500)
        val buf = annexB(body)
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        assertTrue(out.size > 1)

        // Reassemble the way a receiver does: rebuild the NAL header from the FU
        // indicator's F/NRI plus the FU header's type, then concatenate bodies.
        val ind = out[0][RtpH264.RTP_HEADER_LEN].toInt()
        val fu = out[0][RtpH264.RTP_HEADER_LEN + 1].toInt()
        val rebuiltHeader = ((ind and 0xE0) or (fu and 0x1F)).toByte()
        val rebuilt = ArrayList<Byte>()
        rebuilt += rebuiltHeader
        out.forEach { pkt ->
            rebuilt += pkt.copyOfRange(RtpH264.RTP_HEADER_LEN + 2, pkt.size).toList()
        }
        assertArrayEquals(body, rebuilt.toByteArray())
    }

    @Test
    fun `fragment indicators carry FU-A and the original NRI`() {
        val p = RtpH264.Packetizer(ssrc = 1, mtu = 64)
        val buf = annexB(nal(RtpH264.NAL_IDR, 500))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        out.forEach {
            val ind = it[RtpH264.RTP_HEADER_LEN].toInt()
            assertEquals(RtpH264.FU_A, ind and 0x1F)
            assertEquals(0x60, ind and 0xE0) // NRI preserved from the source NAL
        }
    }

    @Test
    fun `only the first fragment sets start and only the last sets end`() {
        val p = RtpH264.Packetizer(ssrc = 1, mtu = 64)
        val buf = annexB(nal(RtpH264.NAL_IDR, 500))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        out.forEachIndexed { i, pkt ->
            val fu = pkt[RtpH264.RTP_HEADER_LEN + 1].toInt()
            assertEquals("start bit on packet $i", i == 0, (fu and 0x80) != 0)
            assertEquals("end bit on packet $i", i == out.lastIndex, (fu and 0x40) != 0)
            assertEquals("type on packet $i", RtpH264.NAL_IDR, fu and 0x1F)
        }
    }

    @Test
    fun `all fragments of one NAL share a timestamp`() {
        val p = RtpH264.Packetizer(ssrc = 1, mtu = 64)
        val buf = annexB(nal(RtpH264.NAL_IDR, 500))
        val ts = RtpH264.timestamp90k(1234)
        val out = p.packetizeAnnexB(buf, 0, buf.size, ts)
        out.forEach { assertEquals(ts, tsOf(it)) }
    }

    // ---- the marker bit -------------------------------------------------------

    @Test
    fun `the marker bit is set only on the last packet of an access unit`() {
        // RFC 6184 s5.1. A receiver uses it to release the frame immediately
        // instead of waiting for the next timestamp — one frame of latency, every
        // frame, if this is wrong.
        val p = RtpH264.Packetizer(ssrc = 1, mtu = 64)
        val buf = annexB(nal(RtpH264.NAL_SPS, 10), nal(RtpH264.NAL_PPS, 8), nal(RtpH264.NAL_IDR, 500))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0)
        out.forEachIndexed { i, pkt ->
            assertEquals("marker on packet $i of ${out.size}", i == out.lastIndex, markerOf(pkt))
        }
    }

    @Test
    fun `marker can be suppressed so parameter sets do not end a frame`() {
        val p = RtpH264.Packetizer(ssrc = 1)
        val buf = annexB(nal(RtpH264.NAL_SPS, 10))
        val out = p.packetizeAnnexB(buf, 0, buf.size, 0, marker = false)
        assertTrue(out.none { markerOf(it) })
    }

    @Test
    fun `an empty access unit produces no packets and burns no sequence numbers`() {
        val p = RtpH264.Packetizer(ssrc = 1)
        assertEquals(emptyList<ByteArray>(), p.packetize(ByteArray(0), emptyList(), 0))
        assertEquals(0, p.sequence)
    }

    // ---- construction guards --------------------------------------------------

    @Test(expected = IllegalArgumentException::class)
    fun `an MTU with no room for a payload is refused`() {
        RtpH264.Packetizer(ssrc = 1, mtu = RtpH264.RTP_HEADER_LEN)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a payload type that does not fit seven bits is refused`() {
        RtpH264.Packetizer(ssrc = 1, payloadType = 200)
    }

    // ---- SDP ------------------------------------------------------------------

    @Test
    fun `the SDP names H264 at ninety kilohertz on the right port`() {
        // This is what makes `ffplay stream.sdp` work on the laptop with no
        // aircraft; a wrong payload type here means "no video" for the wrong reason.
        val sdp = RtpH264.sdp("127.0.0.1", 5600)
        assertTrue(sdp, sdp.contains("m=video 5600 RTP/AVP 96"))
        assertTrue(sdp, sdp.contains("a=rtpmap:96 H264/90000"))
        assertTrue(sdp, sdp.contains("a=fmtp:96 packetization-mode=1"))
        assertTrue(sdp, sdp.contains("c=IN IP4 127.0.0.1"))
        assertTrue("SDP lines must be CRLF-terminated", sdp.contains("\r\n"))
    }

    @Test
    fun `the SDP payload type follows a non-default choice`() {
        assertTrue(RtpH264.sdp("127.0.0.1", 5600, payloadType = 97).contains("RTP/AVP 97"))
        assertNotEquals(RtpH264.sdp("127.0.0.1", 5600), RtpH264.sdp("127.0.0.1", 5601))
    }
}
