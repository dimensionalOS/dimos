package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.video.RawFrameInfo
import com.dimensional.mini4pro.video.RtpH264
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The video half of the flight record: an index line per frame, the bytes in a sidecar, and the
 * three properties that make the pair worth having.
 *
 * Driven entirely by an in-memory [VideoSinkFactory] and a hand-cranked clock — no phone, no MSDK,
 * no filesystem. That is the whole reason [VideoSink] is an interface: rotation, the budget and the
 * parameter-set re-injection are the parts most likely to be wrong and least likely to be exercised
 * before a flight, which is the argument [LogSink] already makes for the JSONL.
 *
 * Written to fail loudly for:
 *
 *  - **an index that does not point at the bytes.** Every `off`/`len` in the log must address
 *    exactly the frame it claims, or the whole record is a list of times with no pictures.
 *  - **a keyframe that is not a seek point** — parameter sets not re-injected ahead of a bare IDR,
 *    which leaves a record decodable only from its first frame and quietly useless for the one job
 *    it exists for.
 *  - **a budget spent in silence**, which looks exactly like an aircraft that stopped sending.
 *  - **a frame time taken anywhere but first**, which would put the video on a different clock from
 *    the telemetry and lose the property the whole design rests on.
 *  - **a throw reaching MSDK's callback thread**, turning an evidence problem into a video problem.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, whole suite run against each, confirmed red,
 * reverted. Counts are failing tests across the suite — measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | parameter sets never re-injected ahead of a bare IDR | 2 |
 *  | the injected parameter sets lose their Annex-B start codes | 2 |
 *  | rotation never happens (one part forever) | 2 |
 *  | the frame ordinal restarts with each part | 2 |
 *  | parameter sets injected ahead of *every* keyframe, including ones that carry them | 1 |
 *  | the budget never bites | 1 |
 *  | the budget bites silently (no event) | 1 |
 *  | geometry written on every frame instead of on change | 1 |
 *  | a sink that throws propagates to the caller | 1 |
 *  | the frame is stamped after its bytes are written rather than on arrival | 1 |
 *
 * ### The start codes were a real defect, found before the phone
 *
 * `RtpH264.nalUnits` reports NALs **without** their start codes, because RTP packetisation strips
 * them — right for the packetiser, exactly wrong for a sidecar, which is an Annex-B elementary
 * stream. The first version cached and re-injected them bare, which would have written 19 bytes no
 * decoder can find in front of every seek point: worse than not injecting at all, because it looks
 * like it worked. `THE INJECTED BYTES ARE COUNTED IN THE FRAME'S LENGTH` caught it on the first run
 * as an eight-byte arithmetic mismatch.
 *
 * ## The first flight, and what it changed — measured 2026-07-27
 *
 * The design's seek-point mechanism was **the wrong one**, and only real frames showed it. It
 * re-injected cached SPS/PPS ahead of a *bare* IDR; DJI never sends one — `psi injected 0 of 16` —
 * so that branch never fired, while the two cases that decide whether a file opens went uncovered.
 * Part 1 of `20260727-160758` began where the operator flipped the switch, mid-GOP: **90 frames
 * undecodable**. Part 2 began where rotation landed: 12. Both played fine after their first
 * keyframe, which is exactly why nothing looked wrong.
 *
 * Ivan's rule replaced it: **a file may only start at a keyframe**, accepting a start delay. Every
 * byte then comes from the aircraft and every part opens at byte 0. Rotation waits for a keyframe
 * rather than cutting at the byte count, so no frames are lost mid-flight — the part simply
 * overshoots its budget by up to a GOP.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | writing starts wherever it lands, not at a keyframe | 3 |
 *  | frames counted only while recording (the witness goes blind) | 2 |
 *  | a part is cut mid-GOP again | 1 |
 *  | a resume does not wait for a keyframe | 1 |
 *  | the first frame is not witnessed | 1 |
 *  | a stall is never reported | 1 |
 *  | a stream that never delivered is reported as stalled | 1 |
 *
 * **"A part is cut mid-GOP" survived the first version of its own test.** The fixture used a GOP of
 * 4 against a part that filled after 4 frames, so the mutated rotation landed on keyframes by
 * arithmetic coincidence and the test passed against the bug it was written for. The GOP is now 5
 * against the same part size, which is the whole difference between a test and a decoration.
 *
 * ### Two mutations needed a harness that models time passing
 *
 * *The frame is stamped after its bytes are written* scored 0 against a clock that only moves
 * between frames, because with an instantaneous write the two stamps are equal. At 625 kB/s a write
 * is not instantaneous, and folding the disk's latency into the video's would quietly break the one
 * property this design exists for — the frame's `t` being on the same clock as the gimbal's. The
 * harness now models a write that costs 20 ms and the mutation dies.
 */
class VideoSidecarTest {

    private class MemorySink(
        override val name: String,
        /** Called on every write, so a test can model a write that takes measurable time. */
        private val onWrite: () -> Unit = {},
    ) : VideoSink {
        val bytes = java.io.ByteArrayOutputStream()
        var closed = false
        var throwOnAppend: Throwable? = null

        override val bytesWritten: Long get() = bytes.size().toLong()

        override fun append(data: ByteArray, offset: Int, length: Int): Long {
            throwOnAppend?.let { throw it }
            onWrite()
            val at = bytes.size().toLong()
            bytes.write(data, offset, length)
            return at
        }

        override fun flush() = Unit
        override fun close() { closed = true }
    }

    private class MemoryFactory(private val onWrite: () -> Unit = {}) : VideoSinkFactory {
        val opened = LinkedHashMap<Int, MemorySink>()
        var pruned = 0
        var keepSeen: Int? = null
        var throwOnOpen: Throwable? = null

        override fun open(session: String, part: Int): VideoSink {
            throwOnOpen?.let { throw it }
            val sink = MemorySink(FileVideoSinkFactory.fileName(session, part), onWrite)
            opened[part] = sink
            return sink
        }

        override fun prune(session: String, keep: Int): Int {
            keepSeen = keep
            return pruned
        }
    }

    private class Harness(
        budgetBytes: Long = 1_000_000,
        partBytes: Long = 1_000_000,
        keepParts: Int = 3,
    ) {
        var now = 1_000_000_000L

        /** Milliseconds the modelled write takes; 0 for every test but the one about ordering. */
        var writeCostMs = 0L
        val sinks = MemoryFactory(onWrite = { now += writeCostMs * 1_000_000 })
        val entries = mutableListOf<LogEntry>()
        val logs = mutableListOf<String>()
        val sidecar = VideoSidecar(
            session = "20260727-150000",
            sinks = sinks,
            budgetBytes = budgetBytes,
            partBytes = partBytes,
            keepParts = keepParts,
            emit = { entries += it },
            nowNanos = { now },
            log = { logs += it },
        ).also { it.recording = true }

        fun frames(): List<LogEntry.Frame> = entries.filterIsInstance<LogEntry.Frame>()

        fun events(): List<LogEntry.Event> = entries.filterIsInstance<LogEntry.Event>()

        /**
         * Feeds one keyframe so that writing may begin, since nothing is written before the first
         * one. Called by the tests that are about something else.
         */
        fun open(): Harness {
            send(idr(8), keyFrame = true, advanceMs = 0)
            entries.clear()
            return this
        }

        /**
         * [count] frames in groups of [gop], each group opening with a keyframe — the shape a real
         * stream has, and the shape rotation now needs, since a part may only be cut on one.
         */
        fun gop(count: Int, gop: Int = 5, bytes: Int = 200) {
            for (i in 0 until count) {
                val key = i % gop == 0
                send(if (key) idr(bytes) else slice(bytes), keyFrame = key)
            }
        }

        /** One frame, advancing the clock by [advanceMs] first. */
        fun send(
            payload: ByteArray,
            keyFrame: Boolean = false,
            advanceMs: Long = 33,
            width: Int = 1920,
            height: Int = 1080,
            mime: String? = "H264",
            ptsMs: Long = 0,
        ) {
            now += advanceMs * 1_000_000
            sidecar.onEncodedFrame(
                payload, 0, payload.size,
                RawFrameInfo(mime, width, height, 30, keyFrame, ptsMs),
            )
        }
    }

    private companion object {
        /** An Annex-B NAL of [type] with [size] bytes of payload after the header. */
        fun nal(type: Int, size: Int, fill: Int = 0x11): ByteArray =
            byteArrayOf(0, 0, 0, 1, type.toByte()) + ByteArray(size) { fill.toByte() }

        fun sps(size: Int = 12) = nal(RtpH264.NAL_SPS, size, 0x71)
        fun pps(size: Int = 5) = nal(RtpH264.NAL_PPS, size, 0x81)
        fun idr(size: Int = 400) = nal(RtpH264.NAL_IDR, size, 0x55)
        fun slice(size: Int = 120) = nal(1, size, 0x33)
    }

    // ------------------------------------------------------------------ the index points at bytes

    @Test
    fun `THE INDEX ADDRESSES THE BYTES - every offset and length reads back the frame it names`() {
        // The property the whole design rests on. An index whose offsets are wrong is a list of
        // timestamps with no pictures behind them, and nothing else in this file matters if it
        // fails: a replay tool seeks by these two numbers and nothing else.
        val h = Harness()
        val payloads = listOf(sps() + pps() + idr(300), slice(90), slice(140), slice(70))
        h.send(payloads[0], keyFrame = true)
        payloads.drop(1).forEach { h.send(it) }

        val written = h.sinks.opened.getValue(1).bytes.toByteArray()
        val frames = h.frames()
        assertEquals(4, frames.size)
        for ((i, f) in frames.withIndex()) {
            val slice = written.copyOfRange(f.offset.toInt(), f.offset.toInt() + payloads[i].size)
            assertTrue("frame ${f.n} does not read back", slice.contentEquals(payloads[i]))
        }
        // Ordinals are 1-based and monotonic — a gap is a drop, and a drop is evidence.
        assertEquals(listOf(1L, 2L, 3L, 4L), frames.map { it.n })
    }

    @Test
    fun `the frame's time is the clock's, taken before anything can fail`() {
        // The one sentence that makes this an index rather than a container: the frame's `t` is on
        // the same monotonic clock as the gimbal's and the altitude's, so a replay joins them with
        // no second clock to reconcile. Asserted by moving the clock between frames and reading it
        // back off the entries.
        val h = Harness()
        h.send(idr(), keyFrame = true, advanceMs = 0)
        val first = h.now
        h.send(slice(), advanceMs = 100)
        val second = h.now
        assertEquals(listOf(first, second), h.frames().map { it.monoNanos })
        assertEquals(100_000_000L, second - first)
    }

    @Test
    fun `THE TIME IS THE FRAME'S ARRIVAL, NOT THE WRITE'S COMPLETION`() {
        // At 625 kB/s a write is not free, and a timestamp taken *after* it would fold the disk's
        // latency into the video's. That is the one number this whole design exists to keep clean:
        // the frame's `t` has to be when the bytes arrived, because that is what gets joined to a
        // gimbal angle and an altitude taken on the same clock. Modelled with a sink whose writes
        // cost 20 ms — an offset that would be invisible against a clock nobody advances.
        val h = Harness()
        h.writeCostMs = 20
        val before = h.now
        h.send(idr(), keyFrame = true, advanceMs = 0)
        val stamped = h.frames().single().monoNanos
        assertEquals("the frame was stamped after its write, not on arrival", before, stamped)
        assertTrue("the modelled write did not cost anything", h.now > stamped)
    }

    @Test
    fun `the frame ordinal is the session's, not the part's`() {
        // A reader joins an index line to bytes with the (part, offset) pair; the ordinal is what
        // tells it whether anything went missing. Restarting the count at each rotation would make
        // "frame 1" ambiguous and hide every drop that happened to straddle a part boundary.
        val h = Harness(partBytes = 300)
        h.gop(9, gop = 3)
        assertEquals((1L..9L).toList(), h.frames().map { it.n })
        assertTrue("never rotated, so the property was not exercised", h.sinks.opened.size > 1)
        assertEquals(9, h.frames().map { it.n }.distinct().size)
    }

    // ------------------------------------------------------------- keyframes are seek points

    @Test
    fun `A KEYFRAME IS A SEEK POINT - cached parameter sets are re-injected ahead of a bare IDR`() {
        // DJI sends SPS/PPS ahead of some IDRs and not others. An offline tool seeking to an
        // arbitrary keyframe is a late joiner in exactly the sense `RtpVideoSink` already handles
        // for a late-joining receiver — and without this a record is decodable only from its first
        // frame, which is the one thing an index is supposed to buy.
        val h = Harness()
        val s = sps()
        val p = pps()
        h.send(s + p + idr(200), keyFrame = true) // carries its own: nothing to inject
        h.send(slice())
        h.send(idr(200), keyFrame = true) // bare: the cached sets go in front

        val frames = h.frames()
        assertFalse("injected ahead of a frame that carried its own", frames[0].parameterSets)
        assertFalse(frames[1].parameterSets)
        assertTrue("a bare IDR was left undecodable", frames[2].parameterSets)

        // And they are really in the file, immediately before the frame, in SPS-then-PPS order.
        val written = h.sinks.opened.getValue(1).bytes.toByteArray()
        val at = frames[2].offset.toInt()
        val injected = written.copyOfRange(at - s.size - p.size, at)
        assertTrue("the injected bytes are not the cached parameter sets", injected.contentEquals(s + p))
    }

    @Test
    fun `the injected bytes are counted in the frame's length, so the index still addresses them`() {
        val h = Harness()
        h.send(sps() + pps() + idr(200), keyFrame = true)
        h.send(idr(200), keyFrame = true)
        val bare = h.frames()[1]
        assertEquals(idr(200).size + sps().size + pps().size, bare.length)
    }

    @Test
    fun `with no parameter sets ever seen, a bare IDR is written as it came`() {
        // Fail-open rather than fail-closed, and deliberately: a frame nobody can seek to is still
        // a frame, and refusing to write it would turn a decoding inconvenience into missing
        // evidence. The `psi` flag is what tells the reader which it got.
        val h = Harness()
        h.send(idr(200), keyFrame = true)
        assertFalse(h.frames().single().parameterSets)
        assertEquals(idr(200).size, h.frames().single().length)
    }

    // ------------------------------------------------------------------ geometry, on change

    @Test
    fun `geometry is written once and then only when it changes`() {
        // A resolution change invalidates the camera intrinsics every pose solution rests on, so it
        // has to be a discrete timestamped line — not something a reader infers by diffing two
        // frames it happened to look at. Repeating it on every frame would cost ~9 % of the index
        // for a fact that changes approximately never.
        val h = Harness()
        h.send(idr(), keyFrame = true)
        h.send(slice())
        h.send(slice(), width = 1280, height = 720)
        h.send(slice(), width = 1280, height = 720)

        val f = h.frames()
        assertEquals(1920, f[0].width)
        assertEquals("H264", f[0].mime)
        assertNull("geometry repeated on an unchanged frame", f[1].width)
        assertNull(f[1].mime)
        assertEquals(1280, f[2].width)
        assertEquals(720, f[2].height)
        assertNull(f[3].width)
    }

    @Test
    fun `a zero presentation time is recorded as absent, not as a time`() {
        // DJI supplies 0 when it has nothing to say. Writing that as a timestamp would put a
        // fictitious pts on every frame, and §3.2's whole hope is that this field turns out to
        // carry something real — which cannot be judged if it is faked when absent.
        val h = Harness()
        h.send(idr(), keyFrame = true, ptsMs = 0)
        h.send(slice(), ptsMs = 12_345)
        assertNull(h.frames()[0].presentationMs)
        assertEquals(12_345L, h.frames()[1].presentationMs)
    }

    // ------------------------------------------------- every part begins at a keyframe

    @Test
    fun `EVERY PART BEGINS AT A KEYFRAME, however the stream was joined`() {
        // **The property the first flight broke, in both ways it can be broken.** Part 1 of
        // `20260727-160758` began where the operator flipped the switch — mid-GOP — and 90 frames
        // were undecodable. Part 2 began where rotation happened to land, and 12 were. Both files
        // played fine after their first keyframe, which is what made it look like nothing was
        // wrong.
        //
        // The rule that fixes both is one sentence: **a file may only start at a keyframe.** Not
        // re-injected parameter sets, which is what was built first and never fires — DJI puts
        // SPS/PPS in front of every keyframe it sends (`psi injected 0 of 16`), so the branch
        // guarding against a bare IDR was guarding against something that does not happen while
        // the two cases that do went uncovered.
        val h = Harness(partBytes = 600)
        // Joined mid-GOP, as a stream always is.
        h.send(slice(150))
        h.send(slice(150))
        // **gop = 5 against a 600-byte part is deliberate.** At 155 bytes a frame the part fills
        // after four, which is *not* a multiple of five — so a rotation that cut where the budget
        // runs out would land mid-GOP and this test would see it. With gop = 4 the two coincide
        // and the mutation "cut mid-GOP again" survives, which is how the first version of this
        // test passed against the bug it was written for.
        h.gop(30, gop = 5, bytes = 150)

        val frames = h.frames()
        assertTrue("nothing was written at all", frames.isNotEmpty())
        assertTrue("never rotated, so the rotation half was not exercised", h.sinks.opened.size > 1)
        for (part in h.sinks.opened.keys) {
            val firstInPart = frames.first { it.part == part }
            assertTrue(
                "part $part starts at frame ${firstInPart.n}, which is not a keyframe",
                firstInPart.keyframe,
            )
            assertEquals("part $part does not start at byte 0", 0L, firstInPart.offset)
        }
    }

    @Test
    fun `nothing is written before the first keyframe, and the wait is counted`() {
        val h = Harness()
        repeat(7) { h.send(slice(150)) }
        assertEquals("frames were written before any keyframe", 0, h.frames().size)
        assertEquals("no part should have been opened yet", 0, h.sinks.opened.size)

        h.send(idr(300), keyFrame = true)
        assertEquals(1, h.frames().size)
        assertTrue(h.frames().single().keyframe)
        assertTrue("the skipped frames were not mentioned", h.logs.any { it.contains("skipping 7") })
    }

    @Test
    fun `a resume waits for a keyframe too, so the gap does not corrupt what follows`() {
        // Resuming mid-GOP writes frames whose reference frames were never written. They decode —
        // the parameter sets are earlier in the same file — but they decode into garbage until the
        // next keyframe, which is worse than a clean gap because it looks like data.
        val h = Harness().open()
        h.gop(8, gop = 4, bytes = 150)
        val before = h.frames().size
        h.sidecar.recording = false
        repeat(3) { h.send(slice(150)) }
        h.sidecar.recording = true
        h.send(slice(150))
        h.send(slice(150))
        assertEquals("wrote a mid-GOP frame on resume", before, h.frames().size)
        h.send(idr(300), keyFrame = true)
        assertEquals(before + 1, h.frames().size)
        assertTrue(h.frames().last().keyframe)
    }

    // ----------------------------------------------- the stream's liveness, recorded

    @Test
    fun `THE FIRST FRAME IS RECORDED EVEN WHEN NOTHING IS BEING WRITTEN`() {
        // **The gap this closes.** `video_phase` reaching SERVING says only that MSDK was asked to
        // start; it says nothing about whether the aircraft delivered. A record could not tell
        // "the camera never sent anything" from "it sent and we did not record it" — the two
        // explanations for an empty index, and they want opposite responses. Ivan's passthrough
        // that sometimes needs a bridge restart is exactly that question.
        val h = Harness()
        h.sidecar.recording = false
        h.send(slice(120))
        val first = h.events().filter { it.code == EventCode.VIDEO_FIRST_FRAME }
        assertEquals("the first frame was not witnessed while idle", 1, first.size)
        assertTrue(first.single().message!!.contains("1920x1080"))
        assertEquals(1L, h.sidecar.counters().seen)
        assertEquals("a frame was written while recording was off", 0, h.frames().size)

        // Said once, not once a frame.
        repeat(30) { h.send(slice(120)) }
        assertEquals(1, h.events().count { it.code == EventCode.VIDEO_FIRST_FRAME })
        assertEquals(31L, h.sidecar.counters().seen)
    }

    @Test
    fun `a stream that goes quiet is reported, and so is its return`() {
        val h = Harness()
        h.sidecar.recording = false
        h.send(slice(120))
        // Nothing yet: a stream is not stalled until it has been quiet a while.
        h.sidecar.checkStall(h.now)
        assertTrue(h.events().none { it.code == EventCode.VIDEO_STALLED })

        h.now += (VideoSidecar.STALL_MS + 500) * 1_000_000
        h.sidecar.checkStall(h.now)
        val stalled = h.events().filter { it.code == EventCode.VIDEO_STALLED }
        assertEquals(1, stalled.size)
        assertTrue(stalled.single().message!!.contains("3.5s"))
        // Said once, however often it is asked.
        repeat(5) { h.sidecar.checkStall(h.now) }
        assertEquals(1, h.events().count { it.code == EventCode.VIDEO_STALLED })

        h.send(slice(120))
        assertEquals(1, h.events().count { it.code == EventCode.VIDEO_RESUMED })
        assertFalse(h.sidecar.counters().stalled)
    }

    @Test
    fun `a stream that never delivered is not reported as stalled`() {
        // Nothing to say: a camera that has sent nothing has not *stopped* sending, and reporting
        // it would put a warning in every session that flies without video.
        val h = Harness()
        h.now += 60_000L * 1_000_000
        h.sidecar.checkStall(h.now)
        assertTrue(h.events().isEmpty())
        assertEquals(0L, h.sidecar.counters().seen)
    }

    // ------------------------------------------------------------------ the budget

    @Test
    fun `THE BUDGET IS SPENT LOUDLY - frames stop and an event says so`() {
        // Silent truncation of a video record looks exactly like an aircraft that stopped
        // delivering frames, and those two want opposite diagnoses. The event is the difference.
        val h = Harness(budgetBytes = 1_000)
        h.gop(20)

        val spent = h.events().filter { it.code == EventCode.VIDEO_BUDGET_SPENT }
        assertEquals("the budget did not bite, or bit more than once", 1, spent.size)
        assertEquals(LogEntry.SEV_WARN, spent.single().severity)
        assertTrue(h.sidecar.counters().budgetSpent)
        // It stops at the budget rather than somewhere near it, and stops for good.
        val frames = h.frames().size
        repeat(5) { h.send(slice(200)) }
        assertEquals("frames kept being written after the budget", frames, h.frames().size)
        assertTrue(h.sidecar.counters().bytes >= 1_000)
    }

    // ------------------------------------------------------------------ rotation

    @Test
    fun `parts rotate on their own budget and the oldest are pruned`() {
        val h = Harness(budgetBytes = 100_000, partBytes = 500, keepParts = 2)
        h.gop(12, gop = 3)

        assertTrue("never rotated", h.sinks.opened.size > 1)
        assertEquals(2, h.sinks.keepSeen)
        // The ordinal is the *session's*, not the part's: a reader joining index to bytes needs the
        // pair, and a per-part counter would make frame 1 ambiguous.
        assertEquals((1L..12L).toList(), h.frames().map { it.n })
        // Every frame names the part its bytes actually went to.
        for (f in h.frames()) {
            val sink = h.sinks.opened.getValue(f.part)
            assertTrue("frame ${f.n} points past the end of part ${f.part}", f.offset < sink.bytesWritten)
        }
    }

    @Test
    fun `an index line survives its part being pruned away`() {
        // Knowing a frame existed and when it arrived is evidence even when its bytes are gone.
        // That is the difference between an index and a container, and it is why a reader must
        // treat a missing part as a gap rather than as a corrupt record.
        val h = Harness(partBytes = 400, keepParts = 1)
        h.sinks.pruned = 1
        h.gop(9, gop = 3)
        assertEquals(9, h.frames().size)
        assertTrue(h.frames().map { it.part }.distinct().size > 1)
    }

    // ------------------------------------------------------------- nothing reaches the caller

    @Test
    fun `A THROWING SINK NEVER REACHES MSDK'S THREAD`() {
        // This runs on the decode thread. An evidence problem must not become a video problem —
        // the contract every implementation in this package inherits from `Tap`.
        val h = Harness()
        h.send(idr(), keyFrame = true)
        h.sinks.opened.getValue(1).throwOnAppend = IllegalStateException("disk full")
        h.send(slice()) // must not throw
        h.send(slice())
        assertEquals("a failed write still produced an index line", 1, h.frames().size)
        assertEquals(2L, h.sidecar.counters().failures)
        assertTrue(h.logs.any { it.contains("failed") })
    }

    @Test
    fun `a factory that cannot open a part is survived too`() {
        val h = Harness()
        h.sinks.throwOnOpen = IllegalStateException("no such directory")
        h.send(idr(), keyFrame = true)
        assertEquals(0, h.frames().size)
        assertTrue(h.sidecar.counters().failures > 0)
    }

    @Test
    fun `an empty frame is ignored rather than indexed`() {
        val h = Harness()
        h.send(ByteArray(0))
        assertEquals(0, h.frames().size)
    }

    @Test
    fun `close is safe twice and reports what was written`() {
        val h = Harness()
        h.send(idr(), keyFrame = true)
        h.sidecar.close()
        h.sidecar.close()
        assertTrue(h.sinks.opened.getValue(1).closed)
        assertNotNull(h.logs.lastOrNull { it.contains("closed") })
    }

    // ------------------------------------------------------------------ the file naming

    @Test
    fun `the sidecar sits beside the JSONL and cannot be confused with it`() {
        // `<session>.v001.h264` against `<session>.001.jsonl`. The `v` is what stops a glob, a sort
        // or a person mixing the two — they share a prefix and a session and differ only here.
        assertEquals("20260727-150000.v001.h264", FileVideoSinkFactory.fileName("20260727-150000", 1))
        assertEquals("20260727-150000.v012.h264", FileVideoSinkFactory.fileName("20260727-150000", 12))
        // The JSONL's own naming is `<session>.001.jsonl`, so the two differ in the one character
        // that a glob or a sort would otherwise run together.
        assertFalse(FileVideoSinkFactory.fileName("s", 1).contains(".001."))
        assertTrue(FileVideoSinkFactory.fileName("s", 1).contains(".v001."))
    }
}
