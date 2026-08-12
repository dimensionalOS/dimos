package com.dimensional.mini4pro.video

import java.io.Closeable
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.atomic.AtomicLong

/**
 * The passthrough video path: the aircraft's own H.264, wrapped in RTP, pushed
 * to the GCS over UDP. No re-encode anywhere.
 *
 * Hangs off [CameraStreamTap] through [RawFrameSink], so nothing here touches
 * DJI types and all of it runs — and is tested — on the laptop.
 *
 * ## Shape
 *
 * ```
 * MSDK callback thread ──copy──> bounded queue ──sender thread──> DatagramSocket
 * ```
 *
 * The copy is not optional: [RawFrameSink]'s contract is that MSDK's buffer is
 * valid only for the duration of the call. The queue is bounded and **drops the
 * oldest frame** when full rather than blocking, because blocking in the callback
 * stalls MSDK's decode path for every consumer, not just us. Drops are counted,
 * not swallowed — see [Counters.framesDropped].
 *
 * ## Parameter sets
 *
 * A receiver that joins mid-stream cannot decode until it has seen SPS and PPS.
 * The aircraft normally repeats them before each IDR, but "normally" is how this
 * milestone loses hardware windows, so the last SPS/PPS seen are cached and
 * re-sent ahead of any IDR access unit that arrives without them.
 *
 * ## Which direction the packets travel, and why that matters here
 *
 * This pushes *from* the phone *to* the GCS. On this network that is the
 * direction that works: `wifi-fix.md` records that laptop→phone unicast is
 * blackholed by the AP while phone→outbound is fine, which is exactly why MAVLink
 * runs through a relay. An RTSP server on the phone has to be dialled *into* and
 * is therefore on the wrong side of that bug; this is not.
 */
class RtpVideoSink(
    private val transport: Transport,
    ssrc: Int = DEFAULT_SSRC,
    mtu: Int = RtpH264.DEFAULT_MTU,
    payloadType: Int = RtpH264.PAYLOAD_TYPE,
    queueDepth: Int = DEFAULT_QUEUE_DEPTH,
    /** Run the sender inline instead of on a thread. Tests only. */
    private val inlineForTests: Boolean = false,
) : RawFrameSink, Closeable {

    /** Where packets go. Injectable so the whole path is testable off-device. */
    fun interface Transport : Closeable {
        fun send(packet: ByteArray, length: Int)
        override fun close() {}
    }

    data class Counters(
        val framesIn: Long = 0,
        val framesSent: Long = 0,
        val framesDropped: Long = 0,
        val packetsSent: Long = 0,
        val bytesSent: Long = 0,
        val sendErrors: Long = 0,
        val parameterSetsInjected: Long = 0,
        val lastError: String? = null,
    ) {
        /**
         * Fold an earlier sink's totals into a later one's, so a socket rebuilt
         * mid-session does not reset the numbers on the status screen.
         *
         * That reset would not be cosmetic. `rtpPkts` is one of the two counters
         * docs/video.md's failure table is read through, and every row that
         * mentions it is really about whether it is *rising*. A counter that
         * silently returns to zero when WiFi flaps would put "rtpPkts=0" on screen
         * for a path that had been working perfectly a second earlier — the exact
         * false diagnosis this class's counters exist to prevent.
         *
         * Not a general monoid, and the asymmetry is the point: the right-hand
         * side is the **live** sink and the left is a closed one. Counts add,
         * because they are a record of the session. [lastError] does **not** — it
         * is taken from the live half unconditionally, including when that is
         * null. An error belonging to a socket that no longer exists would put
         * "check the target is routable" on screen next to a target the current
         * socket is reaching perfectly well; the count in [sendErrors] is what
         * preserves the history, and it is the honest place for it.
         */
        operator fun plus(newer: Counters): Counters = Counters(
            framesIn = framesIn + newer.framesIn,
            framesSent = framesSent + newer.framesSent,
            framesDropped = framesDropped + newer.framesDropped,
            packetsSent = packetsSent + newer.packetsSent,
            bytesSent = bytesSent + newer.bytesSent,
            sendErrors = sendErrors + newer.sendErrors,
            parameterSetsInjected = parameterSetsInjected + newer.parameterSetsInjected,
            lastError = newer.lastError,
        )
    }

    private val packetizer = RtpH264.Packetizer(ssrc = ssrc, payloadType = payloadType, mtu = mtu)

    private val framesIn = AtomicLong()
    private val framesSent = AtomicLong()
    private val framesDropped = AtomicLong()
    private val packetsSent = AtomicLong()
    private val bytesSent = AtomicLong()
    private val sendErrors = AtomicLong()
    private val parameterSetsInjected = AtomicLong()

    @Volatile private var lastError: String? = null
    @Volatile private var running = true

    /** Owned by the sender thread only. */
    private var cachedSps: ByteArray? = null
    private var cachedPps: ByteArray? = null

    private class Job(val data: ByteArray, val timestamp90k: Long)

    private val queue = ArrayBlockingQueue<Job>(queueDepth)

    private val sender: Thread? =
        if (inlineForTests) {
            null
        } else {
            Thread({ drain() }, "rtp-video-sink").apply { isDaemon = true; start() }
        }

    override fun onEncodedFrame(data: ByteArray, offset: Int, length: Int, info: RawFrameInfo) {
        if (!running || length <= 0) return
        framesIn.incrementAndGet()

        // Copy: MSDK's buffer is only valid for this call.
        val copy = data.copyOfRange(offset, offset + length)
        val job = Job(copy, RtpH264.timestamp90k(info.presentationTimeMs))

        if (inlineForTests) {
            emit(job)
            return
        }
        // Drop the oldest rather than block the MSDK callback. Video is the one
        // stream where a stale frame is worth strictly less than a fresh one.
        while (!queue.offer(job)) {
            if (queue.poll() != null) framesDropped.incrementAndGet() else return
        }
    }

    private fun drain() {
        while (running) {
            val job = try {
                queue.take()
            } catch (_: InterruptedException) {
                return
            }
            emit(job)
        }
    }

    private fun emit(job: Job) {
        val data = job.data
        val nals = RtpH264.nalUnits(data, 0, data.size)
        if (nals.isEmpty()) return

        rememberParameterSets(data, nals)
        val packets = buildPackets(data, nals, job.timestamp90k)

        var ok = true
        for (p in packets) {
            try {
                transport.send(p, p.size)
                packetsSent.incrementAndGet()
                bytesSent.addAndGet(p.size.toLong())
            } catch (t: Throwable) {
                ok = false
                sendErrors.incrementAndGet()
                lastError = "${t.javaClass.simpleName}: ${t.message}"
            }
        }
        if (ok) framesSent.incrementAndGet()
    }

    private fun buildPackets(
        data: ByteArray,
        nals: List<RtpH264.Nal>,
        ts: Long,
    ): List<ByteArray> {
        val hasIdr = nals.any { RtpH264.nalType(data, it) == RtpH264.NAL_IDR }
        val hasParams = nals.any { RtpH264.isParameterSet(RtpH264.nalType(data, it)) }
        val sps = cachedSps
        val pps = cachedPps

        if (!hasIdr || hasParams || sps == null || pps == null) {
            return packetizer.packetize(data, nals, ts, marker = true)
        }

        // A keyframe with no parameter sets: a receiver joining now would never
        // decode it. Prepend the cached sets at the same timestamp.
        parameterSetsInjected.incrementAndGet()
        val out = ArrayList<ByteArray>(nals.size + 2)
        out += packetizer.packetize(sps, listOf(RtpH264.Nal(0, sps.size)), ts, marker = false)
        out += packetizer.packetize(pps, listOf(RtpH264.Nal(0, pps.size)), ts, marker = false)
        out += packetizer.packetize(data, nals, ts, marker = true)
        return out
    }

    private fun rememberParameterSets(data: ByteArray, nals: List<RtpH264.Nal>) {
        for (nal in nals) {
            when (RtpH264.nalType(data, nal)) {
                RtpH264.NAL_SPS ->
                    cachedSps = data.copyOfRange(nal.offset, nal.offset + nal.length)
                RtpH264.NAL_PPS ->
                    cachedPps = data.copyOfRange(nal.offset, nal.offset + nal.length)
            }
        }
    }

    fun counters(): Counters = Counters(
        framesIn = framesIn.get(),
        framesSent = framesSent.get(),
        framesDropped = framesDropped.get(),
        packetsSent = packetsSent.get(),
        bytesSent = bytesSent.get(),
        sendErrors = sendErrors.get(),
        parameterSetsInjected = parameterSetsInjected.get(),
        lastError = lastError,
    )

    override fun close() {
        running = false
        sender?.interrupt()
        runCatching { transport.close() }
    }

    companion object {
        /**
         * Depth of the hand-off queue. At 30 fps this is a third of a second of
         * video; deeper only adds latency to frames that are already late.
         */
        const val DEFAULT_QUEUE_DEPTH = 10

        /** Arbitrary but fixed: the SSRC only has to be stable within a stream. */
        const val DEFAULT_SSRC = 0x4D34_5052 // "M4PR"

        /** QGroundControl's default `udpUrl` is `0.0.0.0:5600`. Match it. */
        const val DEFAULT_PORT = 5600

        /**
         * A real UDP socket.
         *
         * @param bind called with the fresh socket before it is used, to pin it to
         *   a particular `android.net.Network`. **Not optional in the field**, and
         *   the reason is `wifi-fix.md` gotcha #2: an unbound socket follows
         *   Android's *default* network, and Android 16 moves that to cellular
         *   whenever WiFi fails the router's validation. RTP addressed to a LAN
         *   relay then leaves over LTE and dies — silently, because a
         *   `DatagramSocket` send to an unroutable LAN address over a carrier
         *   network does not always throw. This is the same trap `WifiBind`
         *   exists for on the MAVLink socket; the video path was written without
         *   it and would have inherited the whole bug.
         *
         *   Nullable rather than mandatory only so the JVM tests, which have no
         *   `ConnectivityManager`, can open a plain loopback socket.
         *
         * A bind that throws must not leak the socket: the caller only ever learns
         * about a `Transport` this function returned.
         */
        fun udp(host: String, port: Int, bind: ((DatagramSocket) -> Unit)? = null): Transport {
            val socket = DatagramSocket()
            try {
                bind?.invoke(socket)
            } catch (e: Exception) {
                socket.close()
                throw IOException("binding the RTP socket to WiFi failed: ${e.message}", e)
            }
            val address = InetAddress.getByName(host)
            return object : Transport {
                override fun send(packet: ByteArray, length: Int) =
                    socket.send(DatagramPacket(packet, length, address, port))

                override fun close() = socket.close()
            }
        }
    }
}
