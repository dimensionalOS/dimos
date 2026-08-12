package com.dimensional.mini4pro.video

/**
 * H.264 Annex-B → RTP (RFC 6184), pure and DJI-free.
 *
 * ## Why this exists
 *
 * The aircraft already sends H.264. `ICameraStreamManager.addReceiveStreamListener`
 * hands us those bytes untouched — DJI's own doc calls it "video stream data …
 * for functions such as self-decoding display and third-party livestream".
 * Wrapping them in RTP is therefore a **passthrough**: no decode, no scale, no
 * re-encode, no quality choice, and no added latency beyond one UDP hop.
 *
 * MSDK's RTSP server is not a passthrough. `ILiveStreamManager` exposes
 * `setLiveStreamQuality(SD|HD|FULL_HD)` (fixed 960×540 / 1280×720 / 1920×1080),
 * `setLiveVideoBitrate(int)` and — conclusively — `setLiveStreamScaleType(ScaleType)`.
 * A scaler in the path means the picture is decoded, resized and encoded again on
 * the phone. That is a second encode of an already-encoded stream: worse
 * latency, worse picture, and CPU spent to get both.
 *
 * ## Format
 *
 * The tap delivers Annex-B: NAL units separated by `00 00 01` or `00 00 00 01`
 * start codes. DJI's own parser confirms both lengths are in play —
 * `javap dji.v5.lib.codec.util.NalUnitUtil` has `isNalHeader(int startCodeLen,
 * byte[], int)` accepting 3 and 4, and `getStartCodeLength(boolean)` returning 3
 * or 4. [nalUnits] therefore handles both rather than assuming one.
 *
 * ## Packetisation
 *
 * Per RFC 6184: a NAL that fits the MTU goes out as a *single NAL unit packet*
 * (the NAL bytes are the RTP payload verbatim); anything larger is split into
 * *FU-A* fragments. STAP-A aggregation is deliberately not implemented — it saves
 * a few packets on SPS/PPS and buys nothing on slices, which are what actually
 * fill the link.
 *
 * Everything here is arithmetic on byte arrays, so all of it is proven on the
 * laptop. That is the point: this is the part of the video path that does *not*
 * need an aircraft to verify.
 */
object RtpH264 {

    /** Dynamic payload type for H.264. 96 is the conventional first dynamic PT. */
    const val PAYLOAD_TYPE = 96

    /** RTP clock for H.264, fixed by RFC 6184 §8.1. */
    const val CLOCK_HZ = 90_000

    /**
     * Largest RTP payload (header included) we will emit.
     *
     * 1400 leaves room under a 1500-byte Ethernet MTU for the IPv4 (20) and UDP
     * (8) headers with slack for any tunnel in the path. Fragmenting at the IP
     * layer instead would mean one lost fragment destroys the whole packet.
     */
    const val DEFAULT_MTU = 1400

    /** Fixed RTP header length, no CSRCs and no extension. */
    const val RTP_HEADER_LEN = 12

    // NAL unit types we care about by name (RFC 6184 Table 1 / H.264 Table 7-1).
    const val NAL_IDR = 5
    const val NAL_SEI = 6
    const val NAL_SPS = 7
    const val NAL_PPS = 8
    const val NAL_AUD = 9

    /** FU-A packet type, RFC 6184 §5.8. */
    const val FU_A = 28

    /** A NAL unit inside a buffer, *excluding* its Annex-B start code. */
    data class Nal(val offset: Int, val length: Int)

    /**
     * Splits an Annex-B buffer into NAL units.
     *
     * Tolerates 3- and 4-byte start codes, leading garbage before the first start
     * code, and trailing zero padding. Returns an empty list rather than throwing
     * when there is no start code at all — a malformed frame must not take the
     * video path down, it must show up as a counter.
     */
    fun nalUnits(data: ByteArray, offset: Int = 0, length: Int = data.size - offset): List<Nal> {
        if (length <= 0 || offset < 0 || offset + length > data.size) return emptyList()
        val end = offset + length
        val out = ArrayList<Nal>(4)

        var scan = offset
        var nalStart = -1
        while (scan < end) {
            val sc = startCodeLengthAt(data, scan, end)
            if (sc == 0) {
                scan++
                continue
            }
            if (nalStart >= 0) addNal(out, data, nalStart, scan)
            nalStart = scan + sc
            scan = nalStart
        }
        if (nalStart in 0 until end) addNal(out, data, nalStart, end)
        return out
    }

    /**
     * Trailing zero bytes belong to the next start code, not to this NAL. Emitting
     * them is legal but they inflate every packet, so they are trimmed.
     */
    private fun addNal(out: MutableList<Nal>, data: ByteArray, start: Int, endExclusive: Int) {
        var e = endExclusive
        while (e > start && data[e - 1] == 0.toByte()) e--
        if (e > start) out += Nal(start, e - start)
    }

    /** 3 or 4 if a start code begins at [i], else 0. */
    private fun startCodeLengthAt(data: ByteArray, i: Int, end: Int): Int {
        if (i + 3 > end) return 0
        if (data[i] != 0.toByte() || data[i + 1] != 0.toByte()) return 0
        if (data[i + 2] == 1.toByte()) return 3
        if (i + 4 <= end && data[i + 2] == 0.toByte() && data[i + 3] == 1.toByte()) return 4
        return 0
    }

    /** The 5-bit `nal_unit_type` of [nal]. */
    fun nalType(data: ByteArray, nal: Nal): Int = data[nal.offset].toInt() and 0x1F

    /** True for the parameter sets a decoder cannot start without. */
    fun isParameterSet(type: Int): Boolean = type == NAL_SPS || type == NAL_PPS

    /** Milliseconds → the 90 kHz RTP clock, kept in 32 bits. */
    fun timestamp90k(presentationTimeMs: Long): Long =
        (presentationTimeMs * (CLOCK_HZ / 1000)) and 0xFFFF_FFFFL

    /**
     * Turns NAL units into RTP packets.
     *
     * Not thread-safe by design: the sequence number is per-stream state and one
     * stream has one sender. [RtpVideoSink] owns exactly one of these on exactly
     * one thread.
     *
     * @param ssrc synchronisation source. Any non-zero value; it identifies this
     *   stream to the receiver and must not change mid-stream.
     * @param mtu largest datagram to emit, RTP header included.
     */
    class Packetizer(
        val ssrc: Int,
        val payloadType: Int = PAYLOAD_TYPE,
        val mtu: Int = DEFAULT_MTU,
    ) {
        init {
            require(mtu > RTP_HEADER_LEN + 2) { "mtu $mtu leaves no room for a payload" }
            require(payloadType in 0..127) { "payload type $payloadType is not 7-bit" }
        }

        /** Next sequence number. Wraps at 16 bits, as RTP requires. */
        var sequence: Int = 0
            private set

        /**
         * Packetises one access unit.
         *
         * @param marker whether to set the RTP marker bit on the final packet.
         *   RFC 6184 §5.1: the marker marks the last packet of an access unit, and
         *   a receiver uses it to know the frame is complete without waiting for
         *   the next timestamp. Getting this wrong costs one frame of latency on
         *   every frame.
         * @return datagram payloads, in send order. Empty when [nals] is empty.
         */
        fun packetize(
            data: ByteArray,
            nals: List<Nal>,
            timestamp90k: Long,
            marker: Boolean = true,
        ): List<ByteArray> {
            if (nals.isEmpty()) return emptyList()
            val out = ArrayList<ByteArray>(nals.size)
            nals.forEachIndexed { i, nal ->
                val last = i == nals.lastIndex
                if (nal.length <= mtu - RTP_HEADER_LEN) {
                    out += single(data, nal, timestamp90k, marker && last)
                } else {
                    out += fragment(data, nal, timestamp90k, marker && last)
                }
            }
            return out
        }

        /** Convenience: scan [data] for NAL units and packetise the lot. */
        fun packetizeAnnexB(
            data: ByteArray,
            offset: Int,
            length: Int,
            timestamp90k: Long,
            marker: Boolean = true,
        ): List<ByteArray> = packetize(data, nalUnits(data, offset, length), timestamp90k, marker)

        private fun single(data: ByteArray, nal: Nal, ts: Long, marker: Boolean): ByteArray {
            val pkt = ByteArray(RTP_HEADER_LEN + nal.length)
            writeHeader(pkt, ts, marker)
            System.arraycopy(data, nal.offset, pkt, RTP_HEADER_LEN, nal.length)
            return pkt
        }

        /**
         * FU-A, RFC 6184 §5.8. The original NAL header byte is *not* copied into
         * the payload: its F/NRI bits go into the FU indicator and its type goes
         * into the FU header, so the byte is reconstructed by the receiver.
         */
        private fun fragment(data: ByteArray, nal: Nal, ts: Long, marker: Boolean): List<ByteArray> {
            val header = data[nal.offset].toInt()
            val fnri = header and 0xE0
            val type = header and 0x1F
            val bodyOffset = nal.offset + 1
            val bodyLength = nal.length - 1
            // Two bytes of every fragment are FU indicator + FU header.
            val perPacket = mtu - RTP_HEADER_LEN - 2

            val out = ArrayList<ByteArray>((bodyLength + perPacket - 1) / perPacket)
            var sent = 0
            while (sent < bodyLength) {
                val n = minOf(perPacket, bodyLength - sent)
                val start = sent == 0
                val end = sent + n >= bodyLength
                val pkt = ByteArray(RTP_HEADER_LEN + 2 + n)
                writeHeader(pkt, ts, marker && end)
                pkt[RTP_HEADER_LEN] = (fnri or FU_A).toByte()
                var fu = type
                if (start) fu = fu or 0x80
                if (end) fu = fu or 0x40
                pkt[RTP_HEADER_LEN + 1] = fu.toByte()
                System.arraycopy(data, bodyOffset + sent, pkt, RTP_HEADER_LEN + 2, n)
                out += pkt
                sent += n
            }
            return out
        }

        private fun writeHeader(pkt: ByteArray, ts: Long, marker: Boolean) {
            pkt[0] = 0x80.toByte() // V=2, P=0, X=0, CC=0
            pkt[1] = ((if (marker) 0x80 else 0) or payloadType).toByte()
            val seq = sequence
            pkt[2] = (seq ushr 8).toByte()
            pkt[3] = seq.toByte()
            sequence = (seq + 1) and 0xFFFF
            pkt[4] = (ts ushr 24).toByte()
            pkt[5] = (ts ushr 16).toByte()
            pkt[6] = (ts ushr 8).toByte()
            pkt[7] = ts.toByte()
            pkt[8] = (ssrc ushr 24).toByte()
            pkt[9] = (ssrc ushr 16).toByte()
            pkt[10] = (ssrc ushr 8).toByte()
            pkt[11] = ssrc.toByte()
        }
    }

    /**
     * A minimal SDP describing the stream, for players that need one.
     *
     * QGroundControl does **not** need this — its "UDP h.264 Video Stream" source
     * builds its own caps. `ffplay` does: pointed at a bare UDP port it cannot
     * know the payload type or codec, so it either guesses wrong or waits
     * forever. Writing this file and running `ffplay -protocol_whitelist
     * file,udp,rtp stream.sdp` is how the RTP path is proven on the laptop with no
     * aircraft in the room.
     */
    fun sdp(host: String, port: Int, payloadType: Int = PAYLOAD_TYPE): String = buildString {
        append("v=0\r\n")
        append("o=- 0 0 IN IP4 127.0.0.1\r\n")
        append("s=mini4pro\r\n")
        append("c=IN IP4 $host\r\n")
        append("t=0 0\r\n")
        append("m=video $port RTP/AVP $payloadType\r\n")
        append("a=rtpmap:$payloadType H264/$CLOCK_HZ\r\n")
        append("a=fmtp:$payloadType packetization-mode=1\r\n")
    }
}
