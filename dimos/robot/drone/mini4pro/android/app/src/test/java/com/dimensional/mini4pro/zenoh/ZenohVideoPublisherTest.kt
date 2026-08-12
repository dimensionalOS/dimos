package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.video.RawFrameInfo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.Timeout
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

/**
 * [ZenohVideoPublisher] — 5.87 Mbit/s of H.264 onto a bus that also carries the telemetry an
 * operator is watching, and the rules that keep the first from ever costing the second.
 *
 * **Three properties, and they are the ones that can hurt somebody.** Each is broken deliberately
 * in the table below rather than argued for in prose:
 *
 *  - **video cannot starve or block telemetry** — separate queue, separate thread, separate
 *    session, and a wedged video transport that leaves the telemetry publisher publishing
 *  - **a full video queue drops to the next keyframe rather than mid-GOP** — because dropping one
 *    P frame does not lose a frame, it corrupts every frame until the next keyframe, and a
 *    subscriber sees garbage rather than a gap
 *  - **nothing in the flight-control path ever waits on Zenoh** — `onEncodedFrame` runs on MSDK's
 *    decode thread and completes in bounded time against a transport that never returns
 *
 * ## What is covered here, and what is only covered by the seam
 *
 * Covered: the GOP state machine in every direction, the counters, the switch, the refusals, the
 * bytes on the wire, and — the two that need real threads — that a wedged video `put` cannot block
 * a frame callback, and that a wedged video `put` cannot delay a telemetry publisher sharing the
 * same [ZenohSinkFactory].
 *
 * **Not covered, and it must be said plainly**: that two `libzenoh_jni` sessions coexist in one
 * process holding MSDK and OpenCV natives; that the router accepts two connections from one phone;
 * what a second native runtime costs in memory, CPU and heat on the device. Those are claims about
 * the library and the phone, and only a real session on the real phone settles them —
 * `docs/zenoh-android-transport.md` §7 is the list.
 *
 * ## Mutation-checked 2026-07-27
 *
 * One breakage at a time, whole suite run, reverted after each, by `tmp/mutate.py`, with
 * `app/build/test-results/testDebugUnitTest` deleted before every run — a mutation that fails to
 * compile otherwise leaves the previous run's XML on disk and the harness reports a confident zero.
 * **Counts are failing tests across the whole 2068-test suite, measured, not estimated.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | video shares the telemetry publisher's settings instead of its own | 4 |
 *  | the video queue capacity never reaches the queue (512 frames of H.264) | **5** |
 *  | **a full queue drops one frame and keeps sending the rest of the GOP** | **4** |
 *  | the GOP cut resumes on the next frame rather than the next keyframe | 3 |
 *  | the stream starts mid-GOP rather than at a keyframe | **0 — equivalent, see below** |
 *  | the switch turning on does not require a keyframe to resume | 2 |
 *  | a session that is not PUBLISHING is published into anyway | 1 |
 *  | **GOP drops counted as frames only — no whole-GOP count** | **5** |
 *  | frames awaiting the first keyframe counted as drops | 1 |
 *  | `enabled` defaults to true | 1 |
 *  | a frame is published while the switch is off | 1 |
 *  | the access unit is written from offset 0 regardless of the window | 1 |
 *  | the whole buffer is sent rather than the access unit's window | 1 |
 *  | the payload is stamped with DJI's `presentationTimeMs` | 1 |
 *  | the frame is stamped `drone/camera` rather than `drone/camera_optical` | 2 |
 *  | the format string is `H264` rather than `h264` | 1 |
 *  | an oversized access unit is encoded rather than refused | 1 |
 *  | video is offered on the `pose` key | 2 |
 *  | video declared `NEVER_DROP` | **5** |
 *
 * ### The three findings in that table
 *
 * **Dropping one frame and continuing kills 4, and every one of them had to be written on
 * purpose.** It is the mutation a byte-level test cannot see: the messages that *do* go out are
 * byte-perfect, more of them go out than the correct code sends, the counters look *healthier*, and
 * the only thing wrong is that a subscriber decodes several seconds of garbage. This is the shape of
 * failure this whole class exists to refuse.
 *
 * **The two isolation mutants kill 4 and 5, and the second is the more interesting.** Sharing the
 * telemetry publisher's *settings* leaves the queue at 512 items — which for video is about
 * 8.6 MB of H.264 held in a process that flies an aircraft, and which changes the drop behaviour
 * completely because the queue effectively stops filling. That the capacity reaching the queue is
 * worth five failing tests is the argument for `Settings.publisherSettings()` existing at all
 * rather than the two settings objects being assumed to line up.
 *
 * **`NEVER_DROP` on video kills 5, which is the QoS argument made concrete.** Reliable + *block* on
 * a 5.87 Mbit/s stream means a slow subscriber parks the publisher's thread indefinitely; five
 * separate assertions notice, including both wedged-transport tests. The catalogue's one blocking
 * channel is `status`, whose messages never repeat, and that is the whole rule.
 *
 * ### Two mutants are alive, and both are equivalent rather than uncaught
 *
 *  - ***The stream starts mid-GOP rather than at a keyframe.*** The mutation flips
 *    `awaitingKey`'s **field initialiser**, and no code path observes it: [start] sets it, and the
 *    [enabled] setter sets it, and nothing can publish without both having happened — `enabled`
 *    defaults false. The initialiser stays because the invariant it states ("this class begins
 *    waiting for a keyframe") is one a future edit could break by adding a fourth way in, and a
 *    test cannot guard a path that does not exist yet. Its two *reachable* siblings — the switch and
 *    the returning session — kill 2 and 1.
 *  - ***`CompressedVideo` nests `std_msgs.Time` rather than `builtin_interfaces.Time`.*** Both are
 *    `int32 sec, int32 nsec` and a nested LCM struct carries **no fingerprint of its own**, so the
 *    two produce byte-identical messages and nothing downstream can tell them apart. The
 *    distinction is real only in the *outer* type's recursive hash, which is a pinned constant —
 *    and `LcmFixtureTest` checks that constant against a fixture generated by DiMOS's own Python,
 *    which is the only place it could ever be caught.
 */
class ZenohVideoPublisherTest {

    /**
     * **Twenty seconds each, for the reason [ZenohPublisherTest] states and one more.**
     *
     * The property under test is *"this never blocks"*, so the natural failure of the code under
     * test is a thread that never returns — and a JUnit assertion cannot fire on a thread that
     * never reaches it. Here the caller is MSDK's decode thread, and a mutation that made
     * `onEncodedFrame` wait would hang the suite rather than fail it.
     */
    @get:Rule
    val timeout: Timeout = Timeout.seconds(20)

    private companion object {
        val CONFIG = ZenohConfig(endpoint = "tcp/10.55.1.50:7447")
        const val VIDEO_KEY = "dimos/drone/video/foxglove_msgs.CompressedVideo"

        /** The measured stream: 1920×1080, H.264, 43 fps. */
        fun info(keyFrame: Boolean, presentationMs: Long = 12_345L) = RawFrameInfo(
            mime = "H264",
            width = 1920,
            height = 1080,
            frameRate = 43,
            keyFrame = keyFrame,
            presentationTimeMs = presentationMs,
        )
    }

    /** A transport that records everything and can be told to misbehave on demand. */
    private class FakeSink : ZenohSink {
        val declared = ArrayList<Pair<String, ZenohQos>>()
        val put = ArrayList<Pair<String, ByteArray>>()
        var closed = 0
        var gate: CountDownLatch? = null

        override fun declare(key: String, qos: ZenohQos) {
            declared += key to qos
        }

        override fun put(key: String, payload: ByteArray) {
            gate?.await()
            synchronized(put) { put += key to payload }
        }

        override fun close() {
            closed++
        }
    }

    /**
     * One factory, **one sink per `open`** — so a test that builds a telemetry publisher and a
     * video publisher from the same factory gets two independent transports, exactly as two zenoh
     * sessions against one router would.
     */
    private class FakeFactory : ZenohSinkFactory {
        val sinks = ArrayList<FakeSink>()
        var gate: CountDownLatch? = null

        override fun open(config: ZenohConfig): ZenohSink =
            FakeSink().also { it.gate = gate; synchronized(sinks) { sinks += it } }
    }

    private var clock = 0L
    private var unix = 1_753_600_000_250L

    private fun publisher(
        factory: ZenohSinkFactory,
        settings: ZenohVideoPublisher.Settings = ZenohVideoPublisher.Settings(),
    ) = ZenohVideoPublisher(
        config = CONFIG,
        factory = factory,
        settings = settings,
        nowMs = { clock },
        nowUnixMs = { unix },
    )

    /** A publisher with a live session, ready to take a keyframe. */
    private fun started(
        factory: FakeFactory,
        settings: ZenohVideoPublisher.Settings = ZenohVideoPublisher.Settings(),
    ): ZenohVideoPublisher {
        val v = publisher(factory, settings)
        v.start(startThread = false)
        v.enabled = true
        // One pump with nothing queued opens the session, so the first frame meets a live one.
        v.pumpOnce()
        return v
    }

    /** An access unit of [size] bytes, distinguishable by [tag]. */
    private fun unit(size: Int, tag: Byte): ByteArray = ByteArray(size) { i ->
        if (i < 4) byteArrayOf(0, 0, 0, 1)[i] else tag
    }

    private fun feed(v: ZenohVideoPublisher, keyFrame: Boolean, tag: Byte, size: Int = 64) {
        val data = unit(size, tag)
        v.onEncodedFrame(data, 0, data.size, info(keyFrame))
    }

    private fun sentTags(sink: FakeSink): List<Byte> = synchronized(sink.put) {
        sink.put.map { CompressedVideoCodec.decode(it.second).data.last() }
    }

    /**
     * Waits for the worker thread to have a session.
     *
     * Load-bearing in the two threaded tests below rather than incidental: a frame offered while
     * the publisher is still CONNECTING is correctly *skipped* rather than queued — that is the
     * rule that guarantees a returning session resumes at a keyframe — so a test that fed a
     * keyframe before the session existed would find nothing wedged and nothing dropped, and would
     * pass for a reason that has nothing to do with what it claims to check.
     */
    private fun awaitPublishing(v: ZenohVideoPublisher) {
        val deadline = System.currentTimeMillis() + 5_000
        while (v.counters().phase != ZenohPublisher.Phase.PUBLISHING &&
            System.currentTimeMillis() < deadline
        ) {
            Thread.sleep(5)
        }
        assertEquals(ZenohPublisher.Phase.PUBLISHING, v.counters().phase)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 1. the happy path, so the failures below mean something
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun `a keyframe and its GOP reach the transport on the video key at the default QoS`() {
        val factory = FakeFactory()
        val v = started(factory)
        feed(v, keyFrame = true, tag = 1)
        feed(v, keyFrame = false, tag = 2)
        feed(v, keyFrame = false, tag = 3)
        v.pumpOnce()

        val sink = factory.sinks.single()
        // DEFAULT — reliable + drop. NEVER_DROP would let a slow subscriber wedge a 5.85 Mbit/s
        // stream behind a blocking put, on a thread that is also draining this queue.
        assertEquals(listOf(VIDEO_KEY to ZenohQos.DEFAULT), sink.declared)
        assertEquals(listOf<Byte>(1, 2, 3), sentTags(sink))
        assertEquals(3L, v.counters().published)
        assertEquals(0L, v.counters().gopsDropped)
    }

    /**
     * **The bytes are the aircraft's own, taken from MSDK's buffer at their own window and never
     * transformed.**
     *
     * Two halves, and the second exists because the first alone let a real mutation live.
     *
     * The **window** half: MSDK delivers an access unit as `[offset, offset+length)` into a buffer
     * that holds other things, so a publisher that ignored the offset would send the wrong bytes
     * and a publisher that ignored the length would send too many. The frame is also mutated after
     * the call returns, because `RawFrameSink`'s contract is that the buffer is valid only for the
     * duration of the call.
     *
     * The **whole-array** half: the common case is `offset == 0 && length == size`, and an
     * offset-aware implementation can be correct there while a length-aware one is not. Measured
     * 2026-07-27: without it, a mutation that returned MSDK's own array unchanged whenever the
     * window was the whole buffer survived the entire suite.
     */
    @Test
    fun `the access unit is taken from MSDK's buffer, verbatim, at its own offset`() {
        val factory = FakeFactory()
        val v = started(factory)
        // A buffer with the access unit in the middle of it, as MSDK's is.
        val buffer = ByteArray(128) { 0x7F }
        val payload = byteArrayOf(0, 0, 0, 1, 0x67, 0x42, 0, 0, 0, 1, 0x65, 0x11)
        payload.copyInto(buffer, 32)
        v.onEncodedFrame(buffer, 32, payload.size, info(keyFrame = true))
        // MSDK reuses the buffer the instant the call returns.
        buffer.fill(0x00)
        v.pumpOnce()

        val decoded = CompressedVideoCodec.decode(factory.sinks.single().put.single().second)
        assertEquals(
            payload.joinToString("") { "%02x".format(it) },
            decoded.data.joinToString("") { "%02x".format(it) },
        )
        assertEquals("h264", decoded.format)
        assertEquals("drone/camera_optical", decoded.frameId)

        // **And again with the whole array as the access unit**, which is the case a copy that
        // optimised itself away for `offset == 0 && length == size` would get wrong — and the
        // shape MSDK actually delivers most often. Measured: without this second half, that
        // mutation survives the entire suite.
        val whole = byteArrayOf(0, 0, 0, 1, 0x65, 0x22, 0x33, 0x44)
        val expected = whole.copyOf()
        v.onEncodedFrame(whole, 0, whole.size, info(keyFrame = true))
        whole.fill(0x00)
        v.pumpOnce()
        val second = CompressedVideoCodec.decode(factory.sinks.single().put.last().second)
        assertEquals(
            "a frame held by reference would be whatever the decoder wrote next",
            expected.joinToString("") { "%02x".format(it) },
            second.data.joinToString("") { "%02x".format(it) },
        )
    }

    /**
     * **The stamp is the frame's arrival instant, never DJI's `presentationTimeMs`.**
     *
     * `LogEntry.Frame` records DJI's number verbatim precisely because nobody knows what clock it
     * is on. A stamp from an unknown clock is worse than no stamp: it looks joinable, and the
     * whole reason the sidecar's design works is that a video frame and a `pose` share one clock.
     */
    @Test
    fun `the frame is stamped with its arrival instant on the same clock as every other message`() {
        val factory = FakeFactory()
        val v = started(factory)
        unix = 1_753_600_042_250L
        v.onEncodedFrame(unit(64, 1), 0, 64, info(keyFrame = true, presentationMs = 999_999L))
        v.pumpOnce()

        val decoded = CompressedVideoCodec.decode(factory.sinks.single().put.single().second)
        assertEquals(LcmTime.ofEpochSeconds(1_753_600_042_250L / 1000.0), decoded.timestamp)
        // And it is emphatically not DJI's presentation time, on any scale.
        assertNotEquals(999_999L, decoded.timestamp.sec.toLong())
        assertNotEquals(999L, decoded.timestamp.sec.toLong())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 2. the GOP rule — the arithmetic rather than the plumbing
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **Nothing goes out until a keyframe**, even with a live session and a switch that is on.
     *
     * `record/VideoSidecar`'s rule, and Ivan's: start at a keyframe and accept the delay. Every
     * byte a subscriber receives then came from the aircraft, and the alternative — 90 undecodable
     * frames at the head of a stream — is what that rule was written after.
     */
    @Test
    fun `the stream starts at a keyframe and the frames before it are skipped, not dropped`() {
        val factory = FakeFactory()
        val v = started(factory)
        feed(v, keyFrame = false, tag = 1)
        feed(v, keyFrame = false, tag = 2)
        feed(v, keyFrame = true, tag = 3)
        feed(v, keyFrame = false, tag = 4)
        v.pumpOnce()

        assertEquals(listOf<Byte>(3, 4), sentTags(factory.sinks.single()))
        val c = v.counters()
        assertEquals(4L, c.seen)
        assertEquals(2L, c.published)
        // Skipped waiting for the first keyframe, not dropped: nothing was lost that could have
        // arrived, and counting these as drops would make a healthy start look like a fault.
        assertEquals(2L, c.framesAwaitingKey)
        assertEquals(0L, c.framesDropped)
        assertEquals(0L, c.gopsDropped)
    }

    /**
     * **The property this class exists for.** A queue that will not take a frame cuts the whole
     * GOP: the frame is dropped and so is every frame after it until the next keyframe.
     *
     * The mutant that drops one frame and keeps sending sends *more* messages, each byte-perfect,
     * and produces a subscriber decoding garbage until the next keyframe with nothing anywhere
     * reporting a fault.
     */
    @Test
    fun `a full queue drops the rest of the GOP, not just the frame that did not fit`() {
        val factory = FakeFactory()
        // Two deep, so the third frame of a GOP is the one that will not fit.
        val v = started(factory, ZenohVideoPublisher.Settings(queueCapacity = 2))

        feed(v, keyFrame = true, tag = 1)
        feed(v, keyFrame = false, tag = 2)
        feed(v, keyFrame = false, tag = 3) // does not fit — the cut
        feed(v, keyFrame = false, tag = 4) // and these would corrupt the picture anyway
        feed(v, keyFrame = false, tag = 5)
        v.pumpOnce()

        // Exactly the frames that fitted, and nothing from after the cut.
        assertEquals(listOf<Byte>(1, 2), sentTags(factory.sinks.single()))
        val c = v.counters()
        assertEquals("one cut", 1L, c.gopsDropped)
        assertEquals("the frame that did not fit, plus the tail of its GOP", 3L, c.framesDropped)
        assertEquals(2L, c.published)

        // And the next keyframe restarts it cleanly.
        v.pumpOnce()
        feed(v, keyFrame = true, tag = 6)
        feed(v, keyFrame = false, tag = 7)
        v.pumpOnce()
        assertEquals(listOf<Byte>(1, 2, 6, 7), sentTags(factory.sinks.single()))
        assertEquals("still one cut", 1L, v.counters().gopsDropped)
    }

    /** A second cut is a second GOP, so the count says how many times the picture broke. */
    @Test
    fun `each cut is one GOP, however many frames it costs`() {
        val factory = FakeFactory()
        val v = started(factory, ZenohVideoPublisher.Settings(queueCapacity = 1))
        // Every GOP here is keyframe + two P frames, and the queue holds one.
        for (gop in 0 until 3) {
            feed(v, keyFrame = true, tag = (10 * gop).toByte())
            feed(v, keyFrame = false, tag = (10 * gop + 1).toByte())
            feed(v, keyFrame = false, tag = (10 * gop + 2).toByte())
            v.pumpOnce()
        }
        val c = v.counters()
        assertEquals("three keyframes went out", 3L, c.published)
        assertEquals("one cut per GOP", 3L, c.gopsDropped)
        assertEquals("two frames lost per cut", 6L, c.framesDropped)
    }

    /**
     * **A session that is not up is not a place to send a GOP into**, and what it comes back to is
     * decodable from its first byte.
     *
     * The same rule as the queue-full cut, arrived at from the other direction: with no session,
     * `ZenohPublisher` would discard the batch, so a stream resumed at whatever frame happened to
     * be next would put a subscriber back in exactly the state the cut exists to avoid.
     */
    @Test
    fun `a session that comes back resumes at a keyframe, not mid-GOP`() {
        val factory = FakeFactory()
        val v = publisher(factory)
        v.start(startThread = false)
        v.enabled = true

        // Before the first pump there is no session at all.
        feed(v, keyFrame = true, tag = 1)
        feed(v, keyFrame = false, tag = 2)
        assertEquals(0L, v.counters().published)
        assertEquals("nothing lost that could have arrived", 0L, v.counters().framesDropped)
        assertEquals(2L, v.counters().framesAwaitingKey)

        // The session opens. Mid-GOP frames are still refused.
        v.pumpOnce()
        feed(v, keyFrame = false, tag = 3)
        v.pumpOnce()
        assertEquals(0L, v.counters().published)

        // And the next keyframe starts it.
        feed(v, keyFrame = true, tag = 4)
        feed(v, keyFrame = false, tag = 5)
        v.pumpOnce()
        assertEquals(listOf<Byte>(4, 5), sentTags(factory.sinks.single()))
    }

    /**
     * The operator's switch, and it obeys the same rule: turning video on mid-flight starts at the
     * next keyframe rather than wherever the stream happens to be.
     *
     * Off is the default, which is the bandwidth argument — those bytes already leave the phone as
     * RTP, so publishing them here doubles the uplink.
     */
    @Test
    fun `video is off by default and resumes at a keyframe when switched on`() {
        val factory = FakeFactory()
        val v = publisher(factory)
        v.start(startThread = false)
        v.pumpOnce()
        assertFalse("the bandwidth is the operator's to spend", v.enabled)

        feed(v, keyFrame = true, tag = 1)
        feed(v, keyFrame = false, tag = 2)
        v.pumpOnce()
        assertEquals(0L, v.counters().published)
        // Seen even while off: the class has to watch the stream continuously to know when the
        // next keyframe arrives, which is the only moment a GOP may start.
        assertEquals(2L, v.counters().seen)

        v.enabled = true
        feed(v, keyFrame = false, tag = 3) // mid-GOP: refused
        feed(v, keyFrame = true, tag = 4)
        v.pumpOnce()
        assertEquals(listOf<Byte>(4), sentTags(factory.sinks.single()))
    }

    /** A stop abandons a queue that is the tail of a GOP whose head has already gone out. */
    @Test
    fun `a stopped and restarted publisher begins at a keyframe`() {
        val factory = FakeFactory()
        val v = started(factory)
        feed(v, keyFrame = true, tag = 1)
        v.pumpOnce()
        v.stop()

        v.start(startThread = false)
        v.enabled = true
        v.pumpOnce()
        feed(v, keyFrame = false, tag = 2)
        v.pumpOnce()
        val sink = factory.sinks.last()
        assertTrue("a restart must not resume into the old GOP", sentTags(sink).isEmpty())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 3. refusals — cheap, counted, and never an exception
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun `an empty, oversized or non-H264 access unit is refused rather than encoded`() {
        val factory = FakeFactory()
        val v = started(factory, ZenohVideoPublisher.Settings(maxFrameBytes = 128))
        feed(v, keyFrame = true, tag = 1)
        v.pumpOnce()

        v.onEncodedFrame(ByteArray(0), 0, 0, info(keyFrame = false))
        v.onEncodedFrame(ByteArray(512), 0, 512, info(keyFrame = false))
        v.onEncodedFrame(
            unit(64, 9), 0, 64,
            info(keyFrame = false).copy(mime = "H265"),
        )
        v.pumpOnce()

        assertEquals(3L, v.counters().framesRefused)
        assertEquals("only the keyframe", 1L, v.counters().published)
        // A refusal breaks the GOP exactly once, because everything after it decodes against a
        // frame the subscriber never received.
        assertEquals(1L, v.counters().gopsDropped)
    }

    /** A null mime is DJI not saying, which is not the same as saying the wrong thing. */
    @Test
    fun `a frame with no stated mime is published rather than refused`() {
        val factory = FakeFactory()
        val v = started(factory)
        v.onEncodedFrame(unit(64, 1), 0, 64, info(keyFrame = true).copy(mime = null))
        v.pumpOnce()
        assertEquals(1L, v.counters().published)
        assertEquals(0L, v.counters().framesRefused)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 4. the isolation — the property to check hardest
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **Telemetry is unaffected by video, measured against a wall clock from another thread.**
     *
     * The video transport is held inside `put` and never released. A telemetry publisher built
     * from the same factory — as `ZenohBus` builds them, two sessions against one router — must go
     * on publishing throughout, and its own `offer` must complete in bounded time.
     *
     * A single shared publisher fails this in both directions at once: the wedged `put` parks the
     * one worker thread, and the video backlog evicts telemetry from the one queue.
     */
    @Test
    fun `a wedged video transport leaves telemetry publishing`() {
        val videoFactory = FakeFactory()
        val gate = CountDownLatch(1)
        videoFactory.gate = gate
        val v = ZenohVideoPublisher(
            CONFIG, videoFactory, ZenohVideoPublisher.Settings(queueCapacity = 4),
            nowMs = { clock }, nowUnixMs = { unix },
        )
        val telemetryFactory = FakeFactory()
        val telemetry = ZenohPublisher(
            CONFIG, telemetryFactory, ZenohPublisher.Settings(), nowMs = { clock },
        )
        try {
            v.start()
            v.enabled = true
            telemetry.start()
            awaitPublishing(v)

            // Wedge the video worker inside `put`.
            feed(v, keyFrame = true, tag = 1)
            Thread.sleep(100)
            repeat(200) { feed(v, keyFrame = false, tag = 2) }

            // Telemetry, offered while the video transport is stuck.
            repeat(200) { telemetry.offer(ZenohChannel.POSE, byteArrayOf(1, 2, 3, 4)) }
            val deadline = System.currentTimeMillis() + 5_000
            while (telemetry.counters().published < 200 && System.currentTimeMillis() < deadline) {
                Thread.sleep(10)
            }
            assertEquals(
                "every telemetry message must go out while video is wedged",
                200L, telemetry.counters().published,
            )
            assertEquals("telemetry must lose nothing", 0L, telemetry.counters().dropped)
            // Video, meanwhile, dropped — which is the correct outcome and the whole point.
            assertTrue("video must have cut", v.counters().gopsDropped > 0)
        } finally {
            gate.countDown()
            v.stop()
            telemetry.stop()
        }
    }

    /**
     * **MSDK's decode thread completes a burst of frames against a transport that never returns.**
     *
     * The caller is the thing under test. `RawFrameSink`'s contract is *"do no work here"*, and a
     * frame callback that blocked would stall the decoder feeding the RTP stream QGroundControl is
     * drawing — so the failure would not be a quiet bus, it would be a black rectangle in front of
     * an operator.
     */
    @Test
    fun `a wedged transport cannot block MSDK's frame callback`() {
        val factory = FakeFactory()
        val gate = CountDownLatch(1)
        factory.gate = gate
        val v = ZenohVideoPublisher(
            CONFIG, factory, ZenohVideoPublisher.Settings(queueCapacity = 4),
            nowMs = { clock }, nowUnixMs = { unix },
        )
        try {
            v.start()
            v.enabled = true
            awaitPublishing(v)
            feed(v, keyFrame = true, tag = 1)
            Thread.sleep(100)

            val done = CountDownLatch(1)
            val delivered = AtomicInteger()
            // A daemon, so a mutation that makes the callback block cannot keep the test JVM alive
            // after the assertion below has already failed.
            Thread {
                repeat(500) { feed(v, keyFrame = false, tag = 2); delivered.incrementAndGet() }
                done.countDown()
            }.apply { isDaemon = true }.start()

            assertTrue(
                "500 frames against a wedged transport must complete in well under a second",
                done.await(3, TimeUnit.SECONDS),
            )
            assertEquals(500, delivered.get())
            assertTrue("and they must have been dropped, not buffered", v.counters().gopsDropped > 0)
        } finally {
            gate.countDown()
            v.stop()
        }
    }

    /**
     * The two publishers open **two sessions**, which is what makes the isolation structural
     * rather than a matter of tuning.
     *
     * `ZenohSink`'s contract is that `declare` and `put` are called only from the publisher's own
     * thread and never from two at once, so two threads requires two sinks. The cost is a second
     * TCP connection and a second native runtime, and it is unmeasured on the phone.
     */
    @Test
    fun `the video publisher opens its own session rather than sharing one`() {
        val factory = FakeFactory()
        val v = started(factory)
        feed(v, keyFrame = true, tag = 1)
        v.pumpOnce()
        val telemetry = ZenohPublisher(
            CONFIG, factory, ZenohPublisher.Settings(), nowMs = { clock },
        )
        telemetry.start(startThread = false)
        telemetry.offer(ZenohChannel.POSE, byteArrayOf(1))
        telemetry.pumpOnce()

        assertEquals("one session each", 2, factory.sinks.size)
        assertEquals(
            listOf(VIDEO_KEY to ZenohQos.DEFAULT),
            factory.sinks[0].declared,
        )
        assertEquals(
            listOf("dimos/drone/pose/geometry_msgs.PoseStamped" to ZenohQos.DEFAULT),
            factory.sinks[1].declared,
        )
        v.stop()
        telemetry.stop()
    }

    /**
     * Video's queue is **frames**, not the telemetry publisher's 512 messages, and it is small on
     * purpose: 24 access units is about half a second at 43 fps and roughly 420 kB held in a
     * process that flies an aircraft.
     */
    @Test
    fun `the video queue is sized in frames and defaults far below the telemetry queue's`() {
        assertEquals(24, ZenohVideoPublisher.Settings().queueCapacity)
        assertTrue(
            ZenohVideoPublisher.Settings().queueCapacity <
                ZenohPublisher.Settings().queueCapacity / 10,
        )
        // And it is the capacity the inner publisher is actually built with — a setting that did
        // not reach the queue would be 512 frames of H.264, about 9 MB, held in memory.
        assertEquals(
            7,
            ZenohVideoPublisher.Settings(queueCapacity = 7).publisherSettings().queueCapacity,
        )
    }

    /** Bytes on the uplink, counted, because that is the number the bandwidth argument is about. */
    @Test
    fun `the published byte count is the H264 payload, not the encoded message`() {
        val factory = FakeFactory()
        val v = started(factory)
        feed(v, keyFrame = true, tag = 1, size = 17_000)
        feed(v, keyFrame = false, tag = 2, size = 3_000)
        v.pumpOnce()
        assertEquals(20_000L, v.counters().bytesPublished)
        // The message is larger than the payload by a fixed envelope, which is what makes the two
        // worth reporting separately.
        val wire = factory.sinks.single().put.sumOf { it.second.size }
        assertTrue(wire > 20_000)
        assertEquals(
            wire,
            factory.sinks.single().put.sumOf {
                CompressedVideoCodec.encodedSize(
                    CompressedVideoCodec.decode(it.second).data.size,
                    "drone/camera_optical",
                    "h264",
                )
            },
        )
    }
}
