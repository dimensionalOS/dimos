package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/**
 * `dji_call` — **"we asked DJI to do X, and DJI answered Y"**, correlated.
 *
 * The three properties [DjiCalls] exists for, and one test each for the ways each of them is easy
 * to lose:
 *
 *  1. **The ask and its answer are one record.** Written to fail if `seq` stops joining them, if
 *     the ask stops being written before the SDK call, or if a second callback writes a second
 *     answer.
 *  2. **A refusal is at least as interesting as an acceptance.** Written to fail if a refusal
 *     stops being `urgent` — i.e. stops reaching the flash before the crash that follows it —
 *     when its ask was not.
 *  3. **A swallowed callback is a line, not an absence.** This is the measured DJI behaviour
 *     (`docs/measurements/2026-07-26-m2-first-command.md`: a `performAction` on a healthy
 *     connected aircraft invoked **neither** callback, four times) and it is the shape that makes
 *     the failure invisible — the same shape as the change-driven-key trap this project has hit
 *     three times, where silence means two entirely different things.
 *
 * Plus the non-negotiable: **recording failure never reaches the caller.** A throwing sink, a
 * throwing clock — nothing here may propagate into a DJI callback thread or a control loop.
 *
 * Mutation counts are in `RecordingSeamTest`'s KDoc table, where the whole seam's are collected.
 */
class DjiCallsTest {

    private val entries = ArrayList<LogEntry.DjiCall>()
    private var nanos = 0L

    private fun calls(config: DjiCalls.Config = DjiCalls.Config()) = DjiCalls(
        sink = { entries.add(it as LogEntry.DjiCall) },
        nowNanos = { nanos },
        config = config,
    )

    private fun advanceMs(ms: Long) { nanos += ms * 1_000_000L }

    // ─────────────────────────────────────────────────────────────────────────
    // 1. the ask and its answer are one record
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun theAskIsRecordedBeforeAnythingIsAskedOfDji() {
        calls().begin(DjiOp.TAKEOFF, null, urgent = true)
        assertEquals(1, entries.size)
        assertEquals(DjiPhase.ASK, entries[0].phase)
        assertEquals(DjiOp.TAKEOFF, entries[0].op)
        assertEquals(LogEntry.KIND_DJI_CALL, entries[0].kind)
        // The elapsed time belongs to an answer. An ask has not elapsed anything.
        assertNull(entries[0].elapsedMs)
    }

    @Test
    fun theAnswerCarriesTheAsksSequenceAndItsElapsedTime() {
        val call = calls().begin(DjiOp.LAND, null, urgent = true)
        advanceMs(37)
        call.accepted()

        assertEquals(2, entries.size)
        val (ask, answer) = entries[0] to entries[1]
        assertEquals(DjiPhase.ASK, ask.phase)
        assertEquals(DjiPhase.OK, answer.phase)
        assertEquals(
            "the answer must carry the ask's sequence — without it the pair cannot be joined " +
                "and the record is back to two unrelated lines",
            ask.sequence, answer.sequence,
        )
        assertEquals(37L, answer.elapsedMs)
    }

    @Test
    fun eachAskGetsItsOwnSequenceSoConcurrentActionsDoNotCross() {
        val c = calls()
        val land = c.begin(DjiOp.LAND, null, urgent = true)
        val gimbal = c.begin(DjiOp.GIMBAL_ROTATE, null, urgent = false)
        gimbal.refused("GIMBAL_BUSY")
        land.accepted()

        val bySeq = entries.groupBy { it.sequence }
        assertEquals(2, bySeq.size)
        for ((_, pair) in bySeq) {
            assertEquals(2, pair.size)
            assertEquals("both lines of a pair must name the same op", pair[0].op, pair[1].op)
        }
        // Interleaving is real: the second ask answered first, and the record says so.
        assertEquals(
            listOf(DjiOp.LAND, DjiOp.GIMBAL_ROTATE, DjiOp.GIMBAL_ROTATE, DjiOp.LAND),
            entries.map { it.op },
        )
    }

    @Test
    fun theRefusalCarriesDjisOwnWordVerbatim() {
        // The three that have each sent someone down a wrong path in a single week.
        for (error in listOf(
            "CONTROL_AUTH_HAS_NO_CONTROL_AUTH", "REQUEST_HANDLER_NOT_FOUND", "SYSTEM_ERROR",
        )) {
            entries.clear()
            calls().begin(DjiOp.GO_HOME, null, urgent = true).refused(error)
            assertEquals(DjiPhase.ERR, entries[1].phase)
            assertEquals(
                "the error name is what an operator searches DJI's forums for; it must not be " +
                    "reworded, clamped or mapped here",
                error, entries[1].error,
            )
        }
    }

    /**
     * DJI's callbacks are not ours to trust: nothing documents that `onSuccess` and `onFailure`
     * are mutually exclusive or that either fires once. A second report must not write a second
     * answer, because that is exactly the correlation this class exists to keep honest.
     */
    @Test
    fun asecondCallbackDoesNotWriteASecondAnswer() {
        val call = calls().begin(DjiOp.CONFIRM_LANDING, null, urgent = true)
        call.accepted()
        call.refused("SYSTEM_ERROR")
        call.accepted()
        assertEquals(2, entries.size)
        assertEquals(DjiPhase.OK, entries[1].phase)
    }

    /**
     * The arguments ride on the ask, and they ride as a **complete JSON object**.
     *
     * `JsonObject.putRaw` splices its string in verbatim, so a caller that passed bare members
     * would write an unparseable line — a log we cannot `jq` is a log we will not read (`Json`).
     * This test was written passing bare members, produced exactly that, and is the reason
     * `Tap.aircraftOut` now says "complete" in bold.
     */
    @Test
    fun argumentsRideOnTheAskAsACompleteJsonObject() {
        val args = JsonObject.render { o ->
            o.put("lat", 37.9, 7); o.put("lon", 23.7, 7); o.put("sats", 14)
        }
        calls().begin(DjiOp.SIM_START, args, urgent = true)
        assertEquals(args, entries[0].argsJson)
        val json = JsonObject.render { entries[0].writeBody(it) }
        assertEquals(
            """{"seq":1,"op":"sim_start","phase":"ask","args":{"lat":37.9,"lon":23.7,"sats":14}}""",
            json,
        )
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 2. a refusal is at least as interesting as an acceptance
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The asymmetry that is the whole of property 2. An accepted gimbal aim is one of thousands
     * and can wait for the next 500 ms flush; the one that came back
     * `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` is the line someone will be reading in an hour,
     * and it must survive the crash that follows it.
     */
    @Test
    fun arefusalIsUrgentEvenWhenItsAskWasNot() {
        val c = calls()
        c.begin(DjiOp.GIMBAL_ROTATE, null, urgent = false).accepted()
        c.begin(DjiOp.GIMBAL_ROTATE, null, urgent = false).refused("GIMBAL_BUSY")

        assertFalse("a non-urgent ask must not fsync", entries[0].urgent)
        assertFalse("nor its acceptance — this is a 5 Hz path", entries[1].urgent)
        assertFalse(entries[2].urgent)
        assertTrue(
            "a refusal must reach the flash immediately whatever its ask asked for",
            entries[3].urgent,
        )
    }

    @Test
    fun anUrgentAskAndItsAcceptanceBothReachTheFlash() {
        calls().begin(DjiOp.TAKEOFF, null, urgent = true).accepted()
        assertTrue(entries[0].urgent)
        assertTrue(entries[1].urgent)
    }

    @Test
    fun asynchronousCallThatThrewIsUrgentToo() {
        calls().begin(DjiOp.CANCEL_LISTENS, null, urgent = false)
            .settled("IllegalStateException: no such holder")
        assertEquals(DjiPhase.SYNC, entries[1].phase)
        assertTrue("a synchronous call that threw is a failure and is urgent", entries[1].urgent)

        entries.clear()
        calls().begin(DjiOp.CANCEL_LISTENS, null, urgent = false).settled(null)
        assertEquals(DjiPhase.SYNC, entries[1].phase)
        assertNull(entries[1].error)
        assertFalse("a clean synchronous return is not worth an fsync", entries[1].urgent)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 3. a swallowed callback is a line, not an absence
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun anAskThatIsNeverAnsweredBecomesALineAfterTheTimeout() {
        val c = calls(DjiCalls.Config(unansweredAfterMs = 5_000))
        c.begin(DjiOp.GO_HOME, null, urgent = true)

        advanceMs(4_999)
        c.sweep()
        assertEquals("must not give up early — a slow answer is not a swallowed one", 1, entries.size)

        advanceMs(1)
        c.sweep()
        assertEquals(2, entries.size)
        assertEquals(DjiPhase.NONE, entries[1].phase)
        assertEquals(DjiOp.GO_HOME, entries[1].op)
        assertEquals(entries[0].sequence, entries[1].sequence)
        assertEquals(5_000L, entries[1].elapsedMs)
        assertTrue("'DJI never answered' must survive the crash that follows it", entries[1].urgent)
        assertFalse("this one was the timeout, not the cap", entries[1].overflow)
    }

    @Test
    fun sweepingTwiceDoesNotReportTheSameSilenceTwice() {
        val c = calls()
        c.begin(DjiOp.TAKEOFF, null, urgent = true)
        advanceMs(6_000)
        c.sweep()
        c.sweep()
        c.sweep()
        assertEquals(2, entries.size)
        assertEquals(0, c.outstandingCount)
    }

    /**
     * A late answer after a `none` is still recorded, with its true elapsed time, so the pair
     * reads "we gave up at 5 s" / "DJI replied at 9 s". Dropping it as unexpected would erase the
     * one piece of evidence that DJI *does* eventually answer on this path.
     */
    @Test
    fun anAnswerArrivingAfterTheTimeoutIsStillRecorded() {
        val c = calls()
        val call = c.begin(DjiOp.LAND, null, urgent = true)
        advanceMs(6_000)
        c.sweep()
        advanceMs(3_000)
        call.accepted()

        assertEquals(3, entries.size)
        assertEquals(DjiPhase.NONE, entries[1].phase)
        assertEquals(6_000L, entries[1].elapsedMs)
        assertEquals(DjiPhase.OK, entries[2].phase)
        assertEquals(9_000L, entries[2].elapsedMs)
        assertEquals(entries[0].sequence, entries[2].sequence)
    }

    @Test
    fun anAnsweredAskIsNeverSweptAsSilent() {
        val c = calls()
        c.begin(DjiOp.LAND, null, urgent = true).accepted()
        advanceMs(60_000)
        c.sweep()
        assertEquals(2, entries.size)
        assertEquals(0, c.outstandingCount)
    }

    /**
     * The backstop for a caller that never sweeps. Bounded, and **visibly** bounded: the evicted
     * ask is recorded with `overflow`, so the cap cannot quietly eat evidence.
     */
    @Test
    fun theOutstandingMapIsBoundedAndSaysWhenItEvicts() {
        val c = calls(DjiCalls.Config(maxOutstanding = 4))
        repeat(10) { advanceMs(1); c.begin(DjiOp.GIMBAL_ROTATE, null, urgent = false) }

        assertTrue("the map must not grow without limit", c.outstandingCount <= 4)
        val evicted = entries.filter { it.phase == DjiPhase.NONE }
        assertEquals(6, evicted.size)
        assertTrue("an eviction must say it was the cap and not the timeout", evicted.all { it.overflow })
        // Oldest first, so what survives is the most recent — the ask most likely still in flight.
        assertEquals(listOf(1L, 2L, 3L, 4L, 5L, 6L), evicted.map { it.sequence })
    }

    // ─────────────────────────────────────────────────────────────────────────
    // recording failure never reaches the caller
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **The non-negotiable.** These methods run on DJI's callback thread and inside the send path.
     * An evidence problem must never become a flight problem, and containment lives here — once —
     * rather than at each port decorator, because a decorator obliged to wrap every tap call in a
     * `try` will eventually forget one, and the forgotten one will be on the path that turns a
     * recorder bug into a refused landing.
     */
    @Test
    fun athrowingSinkNeverReachesTheCaller() {
        val c = DjiCalls(
            sink = { throw IllegalStateException("disk is on fire") },
            nowNanos = { nanos },
        )
        // None of these may throw.
        val call = c.begin(DjiOp.TAKEOFF, null, urgent = true)
        call.accepted()
        call.refused("SYSTEM_ERROR")
        call.settled(null)
        advanceMs(60_000)
        c.sweep()
        assertEquals(0, entries.size)
    }

    @Test
    fun athrowingClockNeverReachesTheCaller() {
        val c = DjiCalls(
            sink = { entries.add(it as LogEntry.DjiCall) },
            nowNanos = { throw IllegalStateException("no clock") },
        )
        val call = c.begin(DjiOp.LAND, null, urgent = true)
        // A handle is always returned, so the caller has something to answer through.
        assertNotNull(call)
        call.accepted()
        c.sweep()
        assertEquals(0, entries.size)
    }

    /**
     * `begin` is called from the `mavlink-rx` thread and the UI thread while answers arrive on
     * DJI's callback thread and `sweep` runs on the recorder's sampler. No locks, so this asserts
     * the lock-free bookkeeping actually holds: every ask gets a distinct sequence, every ask is
     * answered exactly once, and nothing is lost.
     */
    @Test
    fun concurrentAsksAndAnswersLoseNothingAndCrossNothing() {
        val seen = java.util.Collections.synchronizedList(ArrayList<LogEntry.DjiCall>())
        val c = DjiCalls(
            sink = { seen.add(it as LogEntry.DjiCall) },
            nowNanos = { System.nanoTime() },
        )
        val threads = 8
        val each = 200
        val pool = Executors.newFixedThreadPool(threads)
        val done = CountDownLatch(threads)
        repeat(threads) { t ->
            pool.execute {
                repeat(each) { i ->
                    val call = c.begin(DjiOp.GIMBAL_ROTATE, null, urgent = false)
                    if ((t + i) % 2 == 0) call.accepted() else call.refused("GIMBAL_BUSY")
                }
                done.countDown()
            }
        }
        assertTrue(done.await(30, TimeUnit.SECONDS))
        pool.shutdown()

        val total = threads * each
        assertEquals("every ask and every answer must be recorded", 2 * total, seen.size)
        assertEquals(
            "sequences must be distinct across threads",
            total, seen.filter { it.phase == DjiPhase.ASK }.map { it.sequence }.toSet().size,
        )
        val answers = seen.filter { it.phase != DjiPhase.ASK }
        assertEquals(total, answers.size)
        assertEquals("each ask must be answered exactly once", total, answers.map { it.sequence }.toSet().size)
        assertEquals("nothing may be left outstanding", 0, c.outstandingCount)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the rendered line
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun therenderedLineIsTheDocumentedShape() {
        val call = calls().begin(DjiOp.TAKEOFF, null, urgent = true)
        advanceMs(12)
        call.refused("CONTROL_AUTH_HAS_NO_CONTROL_AUTH")

        assertEquals(
            """{"seq":1,"op":"takeoff","phase":"ask"}""",
            JsonObject.render { entries[0].writeBody(it) },
        )
        assertEquals(
            """{"seq":1,"op":"takeoff","phase":"err","err":"CONTROL_AUTH_HAS_NO_CONTROL_AUTH","ms":12}""",
            JsonObject.render { entries[1].writeBody(it) },
        )
    }

    /** `overflow` is omitted when false — the format's rule is that absent is the ordinary state. */
    @Test
    fun overflowIsOmittedWhenItIsNotTheReason() {
        calls().begin(DjiOp.LAND, null, urgent = true)
        assertFalse(JsonObject.render { entries[0].writeBody(it) }.contains("overflow"))
    }

    @Test
    fun elapsedMsIsNeverNegativeWhenAClockGoesBackwards() {
        assertEquals(0L, DjiCalls.elapsedMs(1_000_000_000L, 999_000_000L))
        assertEquals(1L, DjiCalls.elapsedMs(0L, 1_000_000L))
    }
}
