package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.video.RawFrameInfo
import com.dimensional.mini4pro.video.RawFrameSink

/**
 * **Encoded video onto the bus: its own queue, its own thread, its own session, and a drop policy
 * that drops whole GOPs rather than frames.**
 *
 * `docs/mem2-converter.md` §4 is the contract and this is the live half of it. The access units
 * `video/CameraStreamTap` delivers already satisfy `foxglove_msgs.CompressedVideo` **byte for
 * byte** — Annex-B, one access unit per callback, SPS ahead of every IDR, and measured **zero
 * B-frames** over 6952 real frames — so publishing is a copy and a stamp. **Nothing here decodes,
 * re-encodes, repackages or transcodes anything, ever**, and the moment something does, this
 * class has become a different and much more dangerous thing.
 *
 * ## Why this is not an eighth channel on the telemetry publisher
 *
 * Three reasons, and each independently decides it.
 *
 * **1. The rate.** Video measured **5.85 Mbit/s at 43 fps** on
 * `datasets/2026-07-27-apriltag-nadir-25hz` — roughly fifty times the message rate of all
 * telemetry combined. [ZenohPublisher]'s queue is one `ArrayBlockingQueue` shared by every
 * channel and sized to absorb a stall rather than to buffer a flight, so video sharing it would
 * evict telemetry continuously. **Telemetry must be unaffected by video in every case**, and the
 * only way to make that structural rather than a matter of tuning is a second queue.
 *
 * **2. `status` blocks.** `ZenohChannel.STATUS` is `NEVER_DROP` — reliable + *block* — by
 * definition, so a subscriber that stops reading can wedge a `put` indefinitely. That block lands
 * on the publisher's thread. A video stream sharing that thread would be parked behind an operator
 * sentence nobody is reading; worse, a congested video stream would park the *sentence*. Two
 * threads is the fix, and `ZenohSink`'s contract — *"[declare] and [put] are called only from the
 * publisher's own thread, never from a caller's and never from two at once"* — means two threads
 * requires two sinks. Hence two sessions.
 *
 * **The cost of that, stated rather than hidden**: a second `Zenoh.open` against the same router,
 * so a second TCP connection and a second native runtime. It has **never been measured on the
 * phone** — nothing in `zenoh/` has — and it is listed in `docs/zenoh-android-transport.md` §7
 * beside everything else only the device can settle. The alternative was one session shared by two
 * threads, which the seam's contract forbids and which no JVM test could ever check.
 *
 * **3. The drop policy is different in kind.** See below. It is the part of this class that is
 * arithmetic rather than plumbing, and the part worth reading twice.
 *
 * ## Dropping a P frame does not lose a frame — it corrupts every frame until the next keyframe
 *
 * [ZenohPublisher]'s queue drops individual items, which is correct for telemetry: a lost `pose`
 * is followed by another 200 ms later and a consumer sees a gap. **An inter-frame codec has no
 * such property.** Drop one P frame and every P frame after it in the GOP decodes against
 * references that were never applied, so the subscriber does not see a gap — it sees garbage, for
 * up to a whole keyframe interval, with nothing anywhere reporting a fault.
 * `tools/memexport`'s `--keyframes-only` argues exactly this for the offline case:
 * *"decimating an inter-frame codec is not a filter but a re-encode"*.
 *
 * So the policy is **drop until the next keyframe**:
 *
 *  - When a frame does not fit in the queue, that frame is dropped **and so is every frame after
 *    it until the next keyframe**. The subscriber gets a clean cut and a clean restart instead of
 *    a corrupt tail.
 *  - The same rule covers every other way the stream can break: a session that is not open yet,
 *    a session that dropped, a stop and a restart, and the operator's own switch. Each sets
 *    [awaitingKey], and each therefore resumes at a decodable boundary. This is
 *    `record/VideoSidecar`'s rule — Ivan's, from the flight where 90 frames of a sidecar part were
 *    undecodable because recording started mid-GOP — and it is better than any repair because
 *    every byte a subscriber receives came from the aircraft.
 *  - **The drop count is in whole GOPs as well as frames**, because "1400 frames dropped" and
 *    "3 GOPs dropped" are the same event described at two usefully different scales, and only the
 *    second says how many times a viewer's picture broke.
 *
 * ## The mid-stream join, and what was chosen
 *
 * A subscriber that attaches mid-flight cannot decode until it has heard a keyframe — measured
 * **4.15 s apart on average and 8.01 s at worst** — 11 keyframes over 48.6 s on the reference
 * dataset, 13 over 42.4 s on the other. A store does not have this
 * problem because a file may only *start* at a keyframe; a bus does, because there is no start.
 * Three options were available:
 *
 *  1. **The subscriber waits** for the next keyframe — measured up to 8 s of nothing.
 *  2. **Republish SPS/PPS more often** — cheap, and it does not help: parameter sets let a decoder
 *     configure itself, not resynchronise. Without an IDR there is still no reference picture.
 *  3. **Keyframes on a separate key**, so a late joiner subscribes there first and switches.
 *
 * **Option 1 is what this does**, and the reason is that option 2 solves nothing and option 3
 * makes the *publisher* correct at the cost of making every subscriber more complicated and the
 * catalogue less honest — two keys carrying overlapping halves of one stream, which a consumer
 * has to join, in order, without duplicating the keyframe. The waiting is bounded, it is a
 * property of DJI's encoder rather than of anything we chose, and it is the same wait the operator
 * already accepts when the flight recorder's video switch is flipped.
 *
 * **What is done instead, and it costs nothing**: the GOP rule above means a subscriber's wait is
 * *at most* one keyframe interval and never longer, because the publisher itself never emits a
 * partial GOP. Without it, a late joiner could attach immediately after a mid-GOP resumption and
 * decode garbage rather than wait — which is worse than waiting, because garbage looks like data.
 * The interval is DJI's to choose and is not commandable through MSDK; if it ever needs to be
 * shorter, that is a request to DJI's encoder and not something this class can fake.
 *
 * ## What a caller pays
 *
 * [onEncodedFrame] runs on **MSDK's decode thread**, beside `record/VideoSidecar` and the RTP
 * packetiser, under `video/RawFrameSink`'s contract: *"do no work here."* What it does is one
 * volatile read, one boolean test, **one** copy of the access unit — into the LCM message itself,
 * via [CompressedVideoCodec.encodeSlice] — and one `ArrayBlockingQueue.offer`. It cannot block and
 * it cannot throw. Everything a caller could be hurt by — the session, the socket, the router — is
 * on the other side of the queue, which is [ZenohPublisher]'s entire reason for existing and is why
 * this class holds one rather than reimplementing it.
 *
 * **One copy and not two**, which is a measurement rather than a preference: the obvious shape is
 * `copyOfRange` into an `LcmCompressedVideo` and then encode that, and it moves ~17 kB twice at
 * 43 fps — 1.46 MB/s of memcpy and 43 large allocations a second on a thread that is decoding
 * video. `encodeSlice` writes the window straight into a message it owns, so `RawFrameSink`'s
 * *"the buffer is valid only for the duration of the call"* is satisfied by construction rather
 * than by a defensive copy nobody would notice was redundant.
 *
 * ## Testability
 *
 * No Android, no DJI, no zenoh. `video/RawFrameSink` and `video/RawFrameInfo` are plain Kotlin —
 * the seam that package built for exactly this — the transport is [ZenohSinkFactory], and
 * [pumpOnce] drives the whole thing synchronously with no thread at all.
 */
class ZenohVideoPublisher(
    config: ZenohConfig,
    factory: ZenohSinkFactory,
    settings: Settings = Settings(),
    /** Monotonic, for the session backoff. [ZenohPublisher]'s clock, and never a stamp. */
    nowMs: () -> Long,
    /**
     * Wall clock, and the **only** thing that stamps a frame (D-5: the reading's own time, in Unix
     * seconds, at millisecond resolution).
     *
     * **Never DJI's `presentationTimeMs`.** `LogEntry.Frame` records that verbatim precisely
     * because nobody knows what clock it is on, and a stamp from an unknown clock is worse than no
     * stamp: it looks joinable. The frame's arrival instant is on the same clock as every other
     * message on this bus, so a consumer joining a frame to a `pose` has no second clock to
     * reconcile — which is the property `record/VideoSidecar` was built around and this preserves.
     */
    private val nowUnixMs: () -> Long = { System.currentTimeMillis() },
    private val log: (String) -> Unit = {},
    onPhase: (ZenohPublisher.Phase, String) -> Unit = { _, _ -> },
) : RawFrameSink {

    /**
     * @param queueCapacity **frames, not bytes, and small on purpose.** 24 access units is about
     *   half a second at the measured 43 fps and roughly 420 kB at the measured 17.0 kB mean —
     *   held in memory, in a process that flies an aircraft. Smaller than
     *   [ZenohPublisher.Settings.queueCapacity]'s 512 by a factor of twenty, because those are
     *   ~100-byte telemetry messages and these are not. A queue exists to absorb a stall, not to
     *   buffer a flight; if half a second of video will not drain, no capacity fixes it and the
     *   GOP rule is what makes the outcome clean.
     * @param maxFrameBytes a refusal, not a limit. An access unit larger than this is not a frame
     *   this stream produces — the measured maximum is 96 kB — and building a 4 MB LCM message on
     *   the decode thread because a length field was wrong is the one allocation here that could
     *   hurt. Counted as a drop and named in the log.
     */
    data class Settings(
        val queueCapacity: Int = 24,
        val maxFrameBytes: Int = 1 shl 20,
        val publisher: ZenohPublisher.Settings = ZenohPublisher.Settings(),
    ) {
        /** The queue settings the inner publisher is built with. */
        fun publisherSettings(): ZenohPublisher.Settings =
            publisher.copy(queueCapacity = queueCapacity)
    }

    /**
     * What happened to the stream, in numbers an operator and a flight record can both read.
     *
     * [gopsDropped] is the one to look at. [framesDropped] counts every frame that did not go out
     * and is therefore dominated by the frames deliberately skipped while waiting for a keyframe;
     * [gopsDropped] counts how many times the stream was **cut**, which is how many times a
     * viewer's picture broke and had to wait for a keyframe to come back.
     */
    data class Counters(
        val enabled: Boolean,
        val phase: ZenohPublisher.Phase,
        /** Frames MSDK delivered, whether or not any of them were published. */
        val seen: Long,
        val published: Long,
        /** Bytes of H.264 payload published — the number the uplink actually carries. */
        val bytesPublished: Long,
        val framesDropped: Long,
        /** Times the stream was cut and had to resume at a keyframe. */
        val gopsDropped: Long,
        /** Frames thrown away while waiting for the keyframe that starts a GOP. */
        val framesAwaitingKey: Long,
        /** Frames refused outright — oversized, empty, or not H.264. */
        val framesRefused: Long,
        val queued: Int,
        val peakQueued: Int,
        val lastError: String?,
    )

    private val publisher = ZenohPublisher(
        config = config,
        factory = factory,
        settings = settings.publisherSettings(),
        nowMs = nowMs,
        log = log,
        onPhase = onPhase,
    )

    private val maxFrameBytes = settings.maxFrameBytes

    /**
     * **The operator's switch, and the only thing that decides whether a frame reaches the bus.**
     *
     * Deliberately *inside* this class rather than deciding whether the class is handed the stream
     * at all — `record/VideoSidecar` makes the same choice, and the second half of its reasoning is
     * the load-bearing one here: a publisher that saw nothing while off could not know what it had
     * missed, and it has to watch the stream continuously to know when the **next keyframe**
     * arrives, which is the only moment a GOP may start.
     *
     * Off by default. The bandwidth is real, it doubles an uplink this project has measured
     * blackholing traffic in one direction, and it is the operator's to spend.
     */
    @Volatile
    var enabled: Boolean = false
        set(value) {
            synchronized(gate) {
                if (value && !field) {
                    awaitingKey = true
                    droppingGop = false
                }
                field = value
            }
        }

    // ── frame-thread state, all under `gate` ──────────────────────────────────
    //
    // MSDK delivers on one callback thread, so this lock is uncontended in the normal case; it
    // exists because `enabled` and `stop` arrive from the UI thread while a frame is in flight.

    private val gate = Any()

    /** True until the next keyframe, during which frames are counted and thrown away. */
    private var awaitingKey = true

    /**
     * Whether [awaitingKey] is the tail of a **cut GOP** or merely a stream that has not started.
     *
     * The two look identical from inside the loop and are different events to an operator. Frames
     * skipped after a cut are frames a subscriber would have had and lost, so they count as
     * [framesDropped]; frames skipped before the stream has started — the switch just flipped, the
     * session is not up yet — are frames nobody could have received, and counting those as drops
     * would make a healthy start look like a fault. Only the first kind bumps [gopsDropped].
     */
    private var droppingGop = false

    private var seen = 0L
    private var framesAwaitingKey = 0L
    private var framesDropped = 0L
    private var gopsDropped = 0L
    private var framesRefused = 0L
    private var publishedFrames = 0L
    private var publishedBytes = 0L

    val isRunning: Boolean get() = publisher.isRunning

    fun counters(): Counters {
        val c = publisher.counters()
        return synchronized(gate) {
            Counters(
                enabled = enabled,
                phase = c.phase,
                seen = seen,
                published = publishedFrames,
                bytesPublished = publishedBytes,
                framesDropped = framesDropped,
                gopsDropped = gopsDropped,
                framesAwaitingKey = framesAwaitingKey,
                framesRefused = framesRefused,
                queued = c.queued,
                peakQueued = c.peakQueued,
                lastError = c.lastError,
            )
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // lifecycle — the inner publisher's, forwarded
    // ─────────────────────────────────────────────────────────────────────────

    /** Starts the worker. Returns immediately and opens nothing — [ZenohPublisher.start]. */
    fun start(startThread: Boolean = true) {
        synchronized(gate) { awaitingKey = true; droppingGop = false }
        publisher.start(startThread)
    }

    /**
     * Stops the worker and closes the session, abandoning whatever is queued.
     *
     * [awaitingKey] is set as well, so a publisher that is started again begins at a keyframe
     * rather than wherever the stream happens to be. That is not tidiness: the frames still in the
     * queue at [stop] are the tail of a GOP whose head has already gone out, and resuming into it
     * would put a subscriber back in the state this whole class exists to avoid.
     */
    fun stop() {
        synchronized(gate) { awaitingKey = true; droppingGop = false }
        publisher.stop()
    }

    /** Drives the inner worker synchronously. Tests only — [ZenohPublisher.pumpOnce]. */
    fun pumpOnce(waitMs: Long = 0) = publisher.pumpOnce(waitMs)

    // ─────────────────────────────────────────────────────────────────────────
    // the frame path — MSDK's decode thread, and it may never block or throw
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One access unit onto the bus, or not.
     *
     * The order of the tests is the order of their cost, cheapest first, because this runs at
     * 43 fps on a thread that is decoding video: the switch is a volatile read, the keyframe check
     * is a boolean DJI already computed, and only a frame that passes both is copied.
     *
     * @param info DJI's own `StreamInfo`, projected. [RawFrameInfo.keyFrame] is **DJI's verdict,
     *   not our parse** — the sidecar makes the same choice, and it is what keeps the rule that
     *   decides when a GOP may start from costing a walk over every NAL of every frame.
     */
    override fun onEncodedFrame(data: ByteArray, offset: Int, length: Int, info: RawFrameInfo) {
        synchronized(gate) {
            seen++
            if (!enabled) return
            if (length <= 0 || length > maxFrameBytes || info.mime?.let { !isH264(it) } == true) {
                framesRefused++
                if (framesRefused == 1L) {
                    log(
                        "zenoh video: refusing a $length-byte ${info.mime ?: "?"} access unit " +
                            "(first of possibly many)",
                    )
                }
                // A frame that never entered the stream is not a cut, but the *next* one still
                // needs a keyframe before anything downstream is decodable.
                cut(counted = !awaitingKey)
                return
            }
            // A session that is not up is not a place to send a GOP into. Nothing was lost that
            // could have arrived, so this is a stall rather than a drop — and the flag guarantees
            // that whatever the session comes back to is decodable from its first byte.
            if (publisher.counters().phase != ZenohPublisher.Phase.PUBLISHING) {
                awaitingKey = true
                droppingGop = false
                framesAwaitingKey++
                return
            }
            if (awaitingKey) {
                if (!info.keyFrame) {
                    if (droppingGop) framesDropped++ else framesAwaitingKey++
                    return
                }
                awaitingKey = false
                droppingGop = false
            }
            // **One copy, and the LCM message is it.** `encodeSlice` writes the access unit
            // straight out of MSDK's buffer into a message this call owns, so the caller's array
            // is free the moment `onEncodedFrame` returns — `RawFrameSink`'s contract satisfied by
            // construction. Building an `LcmCompressedVideo` first and encoding it afterwards
            // would copy 17 kB twice at 43 fps, which is 1.46 MB/s of memcpy and 43 large
            // allocations a second on the thread that is decoding video.
            val payload = CompressedVideoCodec.encodeSlice(
                timestamp = LcmTime.ofEpochSeconds(nowUnixMs() / 1000.0),
                frameId = ZenohTelemetryEncoder.FRAME_CAMERA_OPTICAL,
                data = data,
                offset = offset,
                length = length,
                format = LcmCompressedVideo.FORMAT_H264,
            )
            if (publisher.offer(ZenohChannel.VIDEO, payload)) {
                publishedFrames++
                publishedBytes += length
                return
            }
            // **The GOP cut.** The queue would not take it, so this frame is gone — and every
            // frame after it in this GOP would decode against a reference that never arrived, so
            // they go too. One clean break instead of several seconds of plausible garbage.
            framesDropped++
            cut(counted = true)
        }
    }

    /** Must hold [gate]. Enters the drop-to-next-keyframe state, counting the GOP when it is one. */
    private fun cut(counted: Boolean) {
        awaitingKey = true
        if (!counted) return
        droppingGop = true
        gopsDropped++
        if (gopsDropped == 1L) {
            log("zenoh video: dropping to the next keyframe (first of possibly many)")
        }
    }

    private fun isH264(mime: String): Boolean = mime.equals("H264", ignoreCase = true)
}
