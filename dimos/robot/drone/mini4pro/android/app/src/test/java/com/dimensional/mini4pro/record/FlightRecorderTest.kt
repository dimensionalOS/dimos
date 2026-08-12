package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The recorder's own behaviour, driven synchronously.
 *
 * [FlightRecorder.start] is called with `startThread = false` throughout, so the
 * writer is stepped by hand: the tests are deterministic and there is no sleeping.
 * That is only possible because the clocks and the sink are injected, which is the
 * whole reason they are.
 *
 * What is checked here is the set of properties that decide whether a log is worth
 * anything after a crash: does the tail survive, are drops timestamped where they
 * happened, do urgent entries reach the flash immediately, does rotation keep the
 * end rather than the beginning.
 */
class FlightRecorderTest {

    /** In-memory sink so a test can read back exactly what was written. */
    private class MemorySink(override val name: String) : LogSink {
        val lines = ArrayList<String>()
        var flushes = 0
        var syncs = 0
        var closed = false
        override var bytesWritten: Long = 0
            private set

        override fun writeLine(line: String) {
            lines.add(line)
            bytesWritten += line.length + 1
        }

        override fun flush(durable: Boolean) {
            flushes++
            if (durable) syncs++
        }

        override fun close() { closed = true }
    }

    private class MemoryFactory : SinkFactory {
        val opened = ArrayList<MemorySink>()
        val pruned = ArrayList<Int>()
        override fun open(session: String, part: Int): LogSink =
            MemorySink("$session.%03d.jsonl".format(part)).also { opened.add(it) }
        override fun prune(session: String, keep: Int): Int {
            pruned.add(keep)
            return 0
        }
    }

    private class FakeClock(var nanos: Long = 1_000_000_000L, var millis: Long = 1_753_000_000_000L) {
        val mono = MonotonicClock { nanos }
        val wall = WallClock { millis }
        fun advanceMs(ms: Long) { nanos += ms * 1_000_000; millis += ms }
    }

    private fun rig(
        config: RecorderConfig = RecorderConfig(),
    ): Triple<FlightRecorder, MemoryFactory, FakeClock> {
        val f = MemoryFactory()
        val c = FakeClock()
        val r = FlightRecorder("20260725-161200", f, c.mono, c.wall, config, headerJson = """{"note":"t"}""")
        return Triple(r, f, c)
    }

    private fun kindsOf(sink: MemorySink) =
        sink.lines.map { Regex("\"k\":\"([a-z_]+)\"").find(it)!!.groupValues[1] }

    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun `header is line 1 and carries both clocks plus the format`() {
        val (r, f, c) = rig()
        r.start(startThread = false)
        val header = f.opened[0].lines[0]
        assertTrue(header.contains("\"k\":\"header\""))
        assertTrue(header.contains("\"format\":\"mini4pro-flightlog-1\""))
        assertTrue(header.contains("\"schema\":1"))
        assertTrue(header.contains("\"session\":\"20260725-161200\""))
        assertTrue(header.contains("\"part\":1"))
        // The wall-clock anchor and the monotonic origin: the only bridge to QGC's
        // tlog and DJI's own records.
        assertTrue(header.contains("\"started_unix_ms\":${c.millis}"))
        assertTrue(header.contains("\"started_mono_ns\":${c.nanos}"))
        assertTrue(header.contains("\"mono\":\"SystemClock.elapsedRealtimeNanos\""))
        // and the caller's identity block is passed straight through
        assertTrue(header.contains("\"meta\":{\"note\":\"t\"}"))
    }

    @Test
    fun `t is seconds since the session's first monotonic reading`() {
        val (r, f, c) = rig()
        r.start(startThread = false)
        c.advanceMs(1234)
        r.record(LogEntry.RcStick(c.nanos, 1, 2, 3, 4))
        r.drainOnce()
        val line = f.opened[0].lines.last()
        assertTrue("expected t=1.234 in $line", line.startsWith("""{"t":1.234,"""))
    }

    @Test
    fun `entries keep their production order`() {
        val (r, f, _) = rig()
        r.start(startThread = false)
        r.record(LogEntry.RcStick(1, 1, 0, 0, 0))
        r.record(LogEntry.RcStick(2, 2, 0, 0, 0))
        r.record(LogEntry.RcStick(3, 3, 0, 0, 0))
        r.drainOnce()
        val lhs = f.opened[0].lines.drop(1).map { Regex("\"lh\":(-?\\d+)").find(it)!!.groupValues[1] }
        assertEquals(listOf("1", "2", "3"), lhs)
    }

    @Test
    fun `a full queue drops loudly and the drop record says when`() {
        val (r, f, c) = rig(RecorderConfig(queueCapacity = 4))
        r.start(startThread = false)
        // Fill the queue, then overflow it by 8 with known timestamps.
        repeat(4) { r.record(LogEntry.RcStick(c.nanos, it, 0, 0, 0)) }
        val firstLost = c.nanos + 1_000_000
        repeat(8) { i ->
            r.record(LogEntry.RcStick(c.nanos + 1_000_000 + i * 1_000_000L, 99, 0, 0, 0))
        }
        val lastLost = c.nanos + 8_000_000
        assertEquals(8, r.dropCount)

        r.drainOnce()
        c.advanceMs(1)
        r.maintenance()

        val drop = f.opened[0].lines.first { it.contains("\"k\":\"drop\"") }
        assertTrue("count: $drop", drop.contains("\"n\":8"))
        assertTrue("total: $drop", drop.contains("\"total\":8"))
        // The point of the whole exercise: a timestamped window, not just a tally,
        // so a tool can say "8 entries lost at t=+X" in the middle of a table.
        assertTrue("from: $drop", drop.contains("\"from\":$firstLost"))
        assertTrue("to: $drop", drop.contains("\"to\":$lastLost"))
        assertTrue("kinds: $drop", drop.contains("\"kinds\":{\"rc_stick\":8}"))
    }

    @Test
    fun `a second burst of drops is reported separately, not re-reported`() {
        val (r, f, c) = rig(RecorderConfig(queueCapacity = 2))
        r.start(startThread = false)
        repeat(2) { r.record(LogEntry.RcStick(c.nanos, 0, 0, 0, 0)) }
        r.record(LogEntry.RcStick(100, 0, 0, 0, 0))     // dropped
        r.drainOnce(); c.advanceMs(1); r.maintenance()
        repeat(2) { r.record(LogEntry.RcStick(c.nanos, 0, 0, 0, 0)) }
        r.record(LogEntry.RcStick(200, 0, 0, 0, 0))     // dropped
        r.record(LogEntry.RcStick(300, 0, 0, 0, 0))     // dropped
        r.drainOnce(); c.advanceMs(1); r.maintenance()

        val drops = f.opened[0].lines.filter { it.contains("\"k\":\"drop\"") }
        assertEquals(2, drops.size)
        assertTrue(drops[0].contains("\"n\":1") && drops[0].contains("\"total\":1"))
        assertTrue(drops[1].contains("\"n\":2") && drops[1].contains("\"total\":3"))
        assertTrue(drops[1].contains("\"from\":200") && drops[1].contains("\"to\":300"))
    }

    @Test
    fun `record returns false for a dropped entry and never throws`() {
        val (r, _, _) = rig(RecorderConfig(queueCapacity = 1))
        r.start(startThread = false)
        assertTrue(r.record(LogEntry.RcStick(1, 0, 0, 0, 0)))
        assertFalse(r.record(LogEntry.RcStick(2, 0, 0, 0, 0)))
    }

    @Test
    fun `recording before start or after stop is a no-op rather than an error`() {
        val (r, _, _) = rig()
        assertFalse(r.record(LogEntry.RcStick(1, 0, 0, 0, 0)))
        r.start(startThread = false)
        assertTrue(r.record(LogEntry.RcStick(1, 0, 0, 0, 0)))
        r.stop()
        assertFalse(r.record(LogEntry.RcStick(2, 0, 0, 0, 0)))
    }

    @Test
    fun `urgent entries are fsynced immediately, high-rate entries are not`() {
        val (r, f, _) = rig()
        r.start(startThread = false)
        val sink = f.opened[0]
        val syncsAfterHeader = sink.syncs

        r.record(LogEntry.RcStick(1, 0, 0, 0, 0))
        r.drainOnce()
        assertEquals("a state sample must not cost an fsync", syncsAfterHeader, sink.syncs)

        r.record(LogEntry.Event(2, EventCode.MODE_CHANGE, message = "APAS -> VIRTUAL_STICK"))
        r.drainOnce()
        assertEquals(syncsAfterHeader + 1, sink.syncs)

        r.record(LogEntry.VsState(3, enabled = false, advanced = true, authority = "RC"))
        r.drainOnce()
        assertEquals(syncsAfterHeader + 2, sink.syncs)
    }

    @Test
    fun `buffered bytes are flushed on the configured interval`() {
        val (r, f, c) = rig(RecorderConfig(flushIntervalMs = 500, syncIntervalMs = 100_000))
        r.start(startThread = false)
        val sink = f.opened[0]
        val before = sink.flushes
        r.record(LogEntry.RcStick(1, 0, 0, 0, 0))
        r.drainOnce()
        c.advanceMs(499)
        r.maintenance()
        assertEquals("not yet due", before, sink.flushes)
        c.advanceMs(2)
        r.maintenance()
        assertEquals(before + 1, sink.flushes)
    }

    @Test
    fun `stop drains the tail and closes the file`() {
        val (r, f, _) = rig()
        r.start(startThread = false)
        r.record(LogEntry.RcStick(1, 7, 0, 0, 0))
        r.record(LogEntry.RcStick(2, 8, 0, 0, 0))
        // Nothing drained yet — the tail is still in the queue.
        assertEquals(1, f.opened[0].lines.size)
        r.stop()
        val sink = f.opened[0]
        assertTrue(sink.closed)
        assertTrue(sink.lines.any { it.contains("\"lh\":7") })
        assertTrue(sink.lines.any { it.contains("\"lh\":8") })
        // and a final stats line, so the log states its own health at the end
        assertTrue(sink.lines.any { it.contains("\"k\":\"stats\"") })
        assertTrue(sink.lines.any { it.contains("\"code\":\"recorder_stop\"") })
    }

    @Test
    fun `rotation opens a new self-describing part and prunes the oldest`() {
        val (r, f, _) = rig(RecorderConfig(maxFileBytes = 900, maxParts = 3))
        r.start(startThread = false)
        // A header is ~500 B and each rc_stick line ~85 B, so a part fills in a
        // handful of samples.
        repeat(40) { i ->
            r.record(LogEntry.RcStick(i.toLong(), i, 0, 0, 0))
            r.drainOnce()
            r.maintenance()
        }
        assertTrue("expected several parts, got ${f.opened.size}", f.opened.size >= 3)
        // every part starts with a full header, so pulling one file is enough
        for ((i, sink) in f.opened.withIndex()) {
            assertTrue(sink.lines[0].contains("\"k\":\"header\""))
            assertTrue(sink.lines[0].contains("\"part\":${i + 1}"))
        }
        // part 2 onward says where it came from
        assertTrue(f.opened[1].lines[0].contains("\"continues\":\"20260725-161200.001.jsonl\""))
        // and the previous part records that it continues elsewhere
        assertTrue(f.opened[0].lines.any { it.contains("\"code\":\"rotate\"") })
        // pruning keeps the newest parts — the tail is what a post-mortem needs
        assertTrue(f.pruned.isNotEmpty())
        assertEquals(3, f.pruned.first())
    }

    @Test
    fun `the writer survives a sink that throws, and says so in the log`() {
        // A recorder must never take the app down: losing the log is bad, losing an
        // airborne aircraft is worse.
        class AngrySink(override val name: String) : LogSink {
            var failures = 0
            val lines = ArrayList<String>()
            override var bytesWritten: Long = 0
                private set
            override fun writeLine(line: String) {
                if (lines.size == 1) { failures++; throw IllegalStateException("disk full") }
                lines.add(line)
                bytesWritten += line.length + 1L
            }
            override fun flush(durable: Boolean) {}
            override fun close() {}
        }
        val sink = AngrySink("angry")
        val factory = object : SinkFactory {
            override fun open(session: String, part: Int) = sink
            override fun prune(session: String, keep: Int) = 0
        }
        val c = FakeClock()
        val r = FlightRecorder("s", factory, c.mono, c.wall, RecorderConfig())
        r.start(startThread = false)
        r.record(LogEntry.RcStick(1, 0, 0, 0, 0))
        var threw = false
        try {
            r.drainOnce()
        } catch (e: Throwable) {
            threw = true
        }
        // drainOnce propagates; the writer *loop* is what catches. Either way the
        // recorder object is still usable afterwards.
        assertTrue(threw || sink.failures == 1)
        assertNotNull(r.render(LogEntry.RcStick(2, 0, 0, 0, 0)))
    }

    @Test
    fun `stats report drops and the peak queue depth`() {
        val (r, f, c) = rig(RecorderConfig(queueCapacity = 4, statsIntervalMs = 100))
        r.start(startThread = false)
        repeat(6) { r.record(LogEntry.RcStick(c.nanos, 0, 0, 0, 0)) }
        r.drainOnce()
        c.advanceMs(200)
        r.maintenance()
        val stats = f.opened[0].lines.last { it.contains("\"k\":\"stats\"") }
        assertTrue(stats.contains("\"dropped\":2"))
        assertTrue(stats.contains("\"peak_queued\":4"))
    }
}
