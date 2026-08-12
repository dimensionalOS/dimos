package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

/**
 * **The whole pipeline on the JVM**, with a fake frame source and a fake detector, which is the
 * point of putting both behind interfaces.
 *
 * ## What this covers and what it cannot
 *
 * It covers everything above `FrameSource` and `TagDetector`: arming, the cap, the queue, the
 * geometry, the latch, the tap, the staleness rule and the containment. That is all the policy in
 * the package.
 *
 * **It cannot cover the two things only an aircraft can settle**, and those are stated rather than
 * simulated:
 *
 *  - *whether occupying the frame callback stalls the video passthrough.* That is a property of
 *    MSDK's threading. Measured on the aircraft 2026-07-28 and pinned by
 *    `vision/FrameListenerProbeTest`: 100 ms held in the frame callback took decoded delivery from
 *    24.0 fps to 9.7 and left the encoded passthrough at 24.0, three runs in a row.
 *  - *what the detector costs.* 40.0 ms/frame at two threads, 1.11 cores capped against a 0.43
 *    floor, same session.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | the frame callback ignores the arming rule | 3 |
 * | the frame callback ignores the rate cap | 1 |
 * | a detector throw is not contained | 1 |
 * | a tap throw is not contained | 1 |
 * | a pose read that throws takes the sighting with it | 1 |
 * | an arming rule that throws leaves it armed | 1 |
 * | the sighting is stamped at publish time, not at frame arrival | 1 |
 * | `latest()` hands out a stale sighting | 1 |
 * | a new flight does not reset the latch | 1 |
 * | `stop` does not detach the frame source | 1 |
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 */
class TagRecogniserTest {

    /**
     * Same bound and the same reason as `LatestFrameTest`'s: this class drives a real worker thread
     * through a real mailbox, so a mutation that makes something wait does not fail an assertion —
     * it stops the suite. [eventually] bounds the polls; this bounds everything else.
     */
    @get:org.junit.Rule
    val timeout: org.junit.rules.Timeout = org.junit.rules.Timeout.seconds(60)

    // ───────────────────────────────────────────────────────────────── fakes

    /** A frame source that is driven by the test rather than by a camera. */
    private class FakeSource : FrameSource {
        var listener: FrameListener? = null
        var starts = 0
        var stops = 0
        var refuseWith: String? = null

        override fun start(listener: FrameListener): String? {
            refuseWith?.let { return it }
            this.listener = listener
            starts++
            return null
        }

        override fun stop() {
            listener = null
            stops++
        }

        /** One 4×4 frame with a marker byte, straight into the listener as MSDK would. */
        fun deliver(atNanos: Long, marker: Byte = 1) {
            listener?.onLuma(ByteArray(16) { marker }, 0, 4, 4, atNanos)
        }
    }

    /** A detector that reports whatever the test tells it to, and counts its calls. */
    private class FakeDetector : TagDetector {
        @Volatile var result: Found = Found.NOTHING
        @Volatile var throwOnDetect: Boolean = false
        val calls = AtomicLong()
        @Volatile var closed = false

        override fun detect(luma: ByteArray, width: Int, height: Int): Found {
            calls.incrementAndGet()
            if (throwOnDetect) throw IllegalStateException("native detector fell over")
            return result
        }

        override fun close() { closed = true }
    }

    private fun found(id: Int, cx: Double = 960.0, cy: Double = 540.0, px: Double = 100.0) =
        Found(listOf(TagDetection(id, 0, cx, cy, px, 42.0)))

    /**
     * Waits for [what] to become true, or fails. The worker is a real thread on a 250 ms idle wait,
     * so every assertion about it is necessarily a poll — written once here rather than as a sleep
     * in every test, because a sleep long enough to be reliable is a slow suite and a sleep short
     * enough to be fast is a flaky one.
     */
    private fun eventually(message: String, timeoutMs: Long = 5_000, what: () -> Boolean) {
        val until = System.nanoTime() + timeoutMs * 1_000_000
        while (System.nanoTime() < until) {
            if (what()) return
            Thread.sleep(5)
        }
        throw AssertionError("timed out waiting: $message")
    }

    /** Everything a recogniser needs, with knobs the test turns. */
    private class Rig(
        capHz: Double = 0.0,
        staleMillis: Long = 1_000,
        minSightings: Int = 3,
    ) {
        val source = FakeSource()
        val detector = FakeDetector()
        val clock = AtomicLong(0)
        // Nullable so a test can make the supplier fail, which is how "the arming rule threw" is
        // reached without giving the production class a knob that exists only for a test.
        val view = AtomicReference<FlightView?>(
            FlightView(
                flying = true, relativeAltitudeM = 2.0,
                returning = false, landing = false, landingOnTag = false,
            )
        )
        val pose = AtomicReference(CameraPose(0.0, 0.0, 10.0, 0.0, -90.0))
        val sightings = java.util.Collections.synchronizedList(
            ArrayList<Triple<TagSighting.Sighting, TagFix?, Boolean>>()
        )
        @Volatile var tapThrows = false
        @Volatile var poseThrows = false

        val recogniser = TagRecogniser(
            source = source,
            detectorFactory = { detector },
            flight = { view.get() ?: throw IllegalStateException("state cache is gone") },
            pose = { if (poseThrows) throw IllegalStateException("state cache is gone") else pose.get() },
            onSighting = { s, f, l ->
                sightings.add(Triple(s, f, l))
                if (tapThrows) throw IllegalStateException("the record is on fire")
            },
            nowNanos = { clock.get() },
            config = TagRecogniser.Config(
                capHz = capHz,
                staleMillis = staleMillis,
                minSightings = minSightings,
                idleWaitMillis = 20,
            ),
        )
    }

    // ────────────────────────────────────────────────────────────── lifecycle

    @Test
    fun startAttachesToTheSourceAndStopDetaches() {
        val rig = Rig()
        assertNull(rig.recogniser.start())
        assertEquals(1, rig.source.starts)
        assertNotNull("the source must have been handed the recogniser", rig.source.listener)
        rig.recogniser.stop()
        assertEquals(1, rig.source.stops)
        assertNull("a stopped recogniser must not still be on the stream", rig.source.listener)
        eventually("the detector is closed") { rig.detector.closed }
    }

    @Test
    fun aRefusedSourceIsReportedAsASentenceAndNothingStarts() {
        val rig = Rig()
        rig.source.refuseWith = "no aircraft connected"
        assertEquals("no aircraft connected", rig.recogniser.start())
        assertEquals("no aircraft connected", rig.recogniser.why)
        // Nothing to stop, and stopping is still safe.
        rig.recogniser.stop()
        assertEquals(0, rig.source.stops)
    }

    @Test
    fun startIsIdempotent() {
        val rig = Rig()
        rig.recogniser.start()
        rig.recogniser.start()
        assertEquals(1, rig.source.starts)
        rig.recogniser.stop()
    }

    // ──────────────────────────────────────────────────────────── the gating

    /**
     * **Disarmed means the frame callback does nothing at all** — not even the copy. The measured
     * difference is 0.06 cores against 0.68.
     */
    @Test
    fun whileDisarmedNoFrameIsEvenCopied() {
        val rig = Rig()
        rig.view.set(
            FlightView(false, 0.0, returning = false, landing = false, landingOnTag = false)
        )
        rig.recogniser.start()
        eventually("the rule has run and disarmed") { rig.recogniser.counters().why == "on the ground" }
        repeat(50) { rig.source.deliver(it * 1_000_000L) }
        Thread.sleep(100)
        val c = rig.recogniser.counters()
        assertEquals("nothing should have reached the mailbox", 0L, c.mailbox.offered)
        assertEquals(0L, rig.detector.calls.get())
        rig.recogniser.stop()
    }

    @Test
    fun armedFramesReachTheDetector() {
        val rig = Rig()
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("the detector saw a frame") { rig.detector.calls.get() >= 1 }
        rig.recogniser.stop()
    }

    /** The operator's OFF beats the rule, immediately, which is the point of a thermal escape. */
    @Test
    fun theOperatorsOffOverridesTheRule() {
        val rig = Rig()
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.recogniser.mode = TagArm.OFF
        eventually("disarmed by the operator") { rig.recogniser.counters().why == "off (operator)" }
        repeat(20) { rig.source.deliver(it * 1_000_000L) }
        Thread.sleep(50)
        assertEquals(0L, rig.recogniser.counters().mailbox.offered)
        rig.recogniser.stop()
    }

    /** And ON beats it the other way, on the ground, which is what a bench experiment needs. */
    @Test
    fun theOperatorsOnArmsOnTheGround() {
        val rig = Rig()
        rig.view.set(
            FlightView(false, 0.0, returning = false, landing = false, landingOnTag = false)
        )
        rig.recogniser.mode = TagArm.ON
        rig.recogniser.start()
        eventually("armed on the ground") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("the detector saw a frame") { rig.detector.calls.get() >= 1 }
        rig.recogniser.stop()
    }

    /** The cap runs on the producer's side, so a refused frame never costs the copy. */
    @Test
    fun theRateCapRefusesOnTheProducerSide() {
        val rig = Rig(capHz = 10.0)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        // Ten frames 10 ms apart: one 100 ms window, so two admitted (t=0 and t=100 ms).
        for (i in 0..10) rig.source.deliver(i * 10_000_000L)
        Thread.sleep(100)
        val c = rig.recogniser.counters()
        assertEquals(2L, c.capAdmitted)
        assertEquals(9L, c.capRefused)
        assertEquals("only admitted frames may be copied", 2L, c.mailbox.offered)
        rig.recogniser.stop()
    }

    // ───────────────────────────────────────────────────────── the sighting

    @Test
    fun aDetectionBecomesASightingAndReachesTheTap() {
        val rig = Rig()
        rig.detector.result = found(id = 4, cx = 1160.0, cy = 540.0, px = 88.0)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(500_000_000L)
        eventually("a sighting reached the tap") { rig.sightings.isNotEmpty() }

        val (s, fix, _) = rig.sightings[0]
        assertEquals(4, s.tagId)
        assertEquals(88.0, s.pixelSize, 1e-9)
        assertEquals(1160.0, s.centreX, 1e-9)
        assertEquals(4, s.imageWidth)
        assertEquals(4, s.imageHeight)
        assertFalse("metric must stay false", s.metric)
        assertNotNull("a complete pose should have produced a fix", fix)
        rig.recogniser.stop()
    }

    /**
     * **A trusted solve rides the whole pipeline into a metric fix** — the detector's solve,
     * through `TagWorld.fix`'s one switch, out the tap with `metric = true` and the solved
     * range, while the raw solve stays on the sighting ungated for the record. The wiring
     * half of what `TagWorldTest` pins arithmetically.
     */
    @Test
    fun aTrustedSolveReachesTheTapAsAMetricFix() {
        val rig = Rig()
        val solve = TagPoseSolve(
            qx = 0.005, qy = 0.017, qz = -0.9998, qw = 0.011,
            tx = 0.02, ty = -0.05, tz = 1.4,
            err1 = 5.0e-8, err2 = Double.POSITIVE_INFINITY, tagSizeM = 0.075,
        )
        rig.detector.result = Found(listOf(TagDetection(4, 0, 2.0, 2.0, 88.0, 42.0, solve = solve)))
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(500_000_000L)
        eventually("a sighting reached the tap") { rig.sightings.isNotEmpty() }
        val (s, fix, _) = rig.sightings[0]
        assertSame("the raw solve stays on the sighting, ungated", solve, s.solve)
        assertNotNull(fix)
        assertTrue("a trusted solve must surface as a metric fix", fix!!.metric)
        assertEquals(1.4, fix.rangeM!!, 1e-12)
        assertEquals("height stays the baro", 10.0, fix.fromHeightM, 1e-12)
        assertEquals(fix, rig.recogniser.latestFix())
        rig.recogniser.stop()
    }

    /**
     * **The sighting is stamped when the frame arrived, not when it was published.**
     *
     * The single most important field for honesty about age: a sighting handed to a 25 Hz loop is
     * 60–160 ms old, and a stamp taken at publish time would erase exactly that.
     */
    @Test
    fun theSightingCarriesTheFramesArrivalTimeNotThePublishTime() {
        val rig = Rig()
        rig.detector.result = found(0)
        rig.clock.set(9_000_000_000L)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_234_000_000L)
        eventually("published") { rig.sightings.isNotEmpty() }
        assertEquals(1_234_000_000L, rig.sightings[0].first.atNanos)
        // And the age is the difference, which is what a controller must compensate for.
        assertEquals(7_766L, rig.sightings[0].first.ageMillisAt(9_000_000_000L))
        rig.recogniser.stop()
    }

    /** A frame with nothing in it produces no sighting, and does not reset anything. */
    @Test
    fun aFrameWithNoTagProducesNoSighting() {
        val rig = Rig()
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("the detector ran") { rig.recogniser.counters().detected >= 1 }
        assertTrue(rig.sightings.isEmpty())
        assertEquals(0L, rig.recogniser.counters().hits)
        assertNull(rig.recogniser.latest())
        rig.recogniser.stop()
    }

    /** With two tags in one frame the largest is the one reported — the only ranking there is. */
    @Test
    fun theLargestTagInAFrameIsTheOneReported() {
        val rig = Rig()
        rig.detector.result = Found(
            listOf(
                TagDetection(1, 0, 100.0, 100.0, 20.0, 30.0),
                TagDetection(2, 0, 200.0, 200.0, 140.0, 60.0),
                TagDetection(3, 0, 300.0, 300.0, 90.0, 55.0),
            )
        )
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("published") { rig.sightings.isNotEmpty() }
        assertEquals(2, rig.sightings[0].first.tagId)
        rig.recogniser.stop()
    }

    /** No pose means a sighting with no fix — recorded as null rather than as a zero. */
    @Test
    fun anIncompletePoseStillProducesASightingButNoFix() {
        val rig = Rig()
        rig.pose.set(CameraPose(null, null, null, null, null))
        rig.detector.result = found(0)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("published") { rig.sightings.isNotEmpty() }
        assertNotNull(rig.sightings[0].first)
        assertNull("a fix that could not be made must be null", rig.sightings[0].second)
        rig.recogniser.stop()
    }

    // ───────────────────────────────────────────────────────────── the latch

    @Test
    fun threeSightingsLatchAndTheEdgeIsReportedOnce() {
        val rig = Rig()
        rig.detector.result = found(11)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        for (i in 1..5) {
            rig.source.deliver(i * 10_000_000L)
            eventually("frame $i handled") { rig.sightings.size >= i }
        }
        assertEquals(1, rig.sightings.count { it.third })
        assertTrue("the third frame is the latching one", rig.sightings[2].third)
        val held = rig.recogniser.latched()
        assertNotNull(held)
        assertEquals(11, held!!.tagId)
        rig.recogniser.stop()
    }

    /** A new flight clears the latch, so a fix from a previous takeoff point cannot be inherited. */
    @Test
    fun anewFlightClearsTheLatch() {
        val rig = Rig()
        rig.detector.result = found(11)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        for (i in 1..3) {
            rig.source.deliver(i * 10_000_000L)
            eventually("frame $i handled") { rig.sightings.size >= i }
        }
        assertNotNull(rig.recogniser.latched())

        // Land, then take off again.
        rig.view.set(
            FlightView(false, 0.0, returning = false, landing = false, landingOnTag = false)
        )
        eventually("on the ground") { rig.recogniser.counters().why == "on the ground" }
        rig.view.set(
            FlightView(true, 0.5, returning = false, landing = false, landingOnTag = false)
        )
        eventually("the latch cleared for the new flight") { rig.recogniser.latched() == null }
        rig.recogniser.stop()
    }

    // ─────────────────────────────────────────────────────────── staleness

    @Test
    fun latestGoesQuietOnceTheSightingIsStale() {
        val rig = Rig(staleMillis = 500)
        rig.detector.result = found(0)
        rig.clock.set(1_000_000_000L)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000_000L)
        eventually("published") { rig.sightings.isNotEmpty() }

        assertNotNull(rig.recogniser.latest())
        rig.clock.set(1_400_000_000L)
        assertNotNull("400 ms is inside the bound", rig.recogniser.latest())
        rig.clock.set(1_600_000_000L)
        assertNull("600 ms is past a 500 ms bound", rig.recogniser.latest())
        // The latch is not stale, though: it is a statement about the site, not about the frame.
        rig.recogniser.stop()
    }

    /** Stopping forgets the sighting, so a stopped session cannot go on reporting a tag. */
    @Test
    fun stoppingForgetsTheLatestSighting() {
        val rig = Rig()
        rig.detector.result = found(0)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("published") { rig.sightings.isNotEmpty() }
        assertNotNull(rig.recogniser.latest())
        rig.recogniser.stop()
        assertNull(rig.recogniser.latest())
    }

    // ────────────────────────────────────────────────────────── containment

    /**
     * **A detector fault is an evidence problem and must never become a flight problem.**
     * `record/Tap` sets that rule for the whole project; this thread sits closer to the flight
     * than most of the places it is applied.
     */
    @Test
    fun aDetectorThatThrowsDoesNotStopTheWorker() {
        val rig = Rig()
        rig.detector.throwOnDetect = true
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        // One at a time and waited for. The mailbox is depth one and drops the oldest, so a burst
        // of five offers is *supposed* to reach the detector as one or two — which is the design
        // working and would make a "five throws" assertion a test of the queue's timing.
        for (i in 1..3) {
            rig.source.deliver(i * 10_000_000L)
            eventually("throw $i survived") { rig.detector.calls.get() >= i }
        }

        // And it recovers: turn the fault off and detections resume.
        rig.detector.throwOnDetect = false
        rig.detector.result = found(3)
        rig.source.deliver(900_000_000L)
        eventually("a sighting after the fault") { rig.sightings.isNotEmpty() }
        rig.recogniser.stop()
    }

    /** Same rule for the tap. A recorder on fire must not take the detector down with it. */
    @Test
    fun aTapThatThrowsDoesNotStopTheWorker() {
        val rig = Rig()
        rig.tapThrows = true
        rig.detector.result = found(3)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        // One at a time, for the reason the detector-throw test above gives: a depth-one mailbox
        // dropping the oldest is the design, not a failure to test around.
        for (i in 1..4) {
            rig.source.deliver(i * 10_000_000L)
            eventually("throwing tap $i survived") { rig.sightings.size >= i }
        }
        // The latch still advanced, because the tap is the last thing that happens.
        assertNotNull(rig.recogniser.latched())
        rig.recogniser.stop()
    }

    /**
     * A pose supplier that throws costs the **fix**, not the sighting.
     *
     * The distinction is the point: "the camera can see tag 0" survives a broken state cache, and
     * "tag 0 is 1.4 m north" does not, because the second one rests on the thing that broke.
     */
    @Test
    fun aPoseSupplierThatThrowsStillProducesASighting() {
        val rig = Rig()
        rig.detector.result = found(0)
        rig.poseThrows = true
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.source.deliver(1_000_000)
        eventually("published despite a broken pose") { rig.sightings.isNotEmpty() }
        assertEquals(0, rig.sightings[0].first.tagId)
        assertNull("no fix may be invented from a pose that could not be read", rig.sightings[0].second)
        rig.recogniser.stop()
    }

    /** An arming rule that throws disarms rather than continuing on a rule nobody is evaluating. */
    @Test
    fun anArmingRuleThatThrowsDisarms() {
        val rig = Rig()
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        rig.view.set(null)
        eventually("disarmed after the rule threw") { rig.recogniser.counters().why == "arming rule failed" }
        assertFalse(rig.recogniser.counters().armed)
        repeat(10) { rig.source.deliver(it * 1_000_000L) }
        Thread.sleep(50)
        assertEquals(0L, rig.recogniser.counters().mailbox.offered)
        rig.recogniser.stop()
    }

    // ──────────────────────────────────────────────────────────── counters

    @Test
    fun theCountersReportWhatHappened() {
        val rig = Rig()
        rig.detector.result = found(0)
        rig.recogniser.start()
        eventually("armed") { rig.recogniser.counters().armed }
        for (i in 1..3) {
            rig.source.deliver(i * 10_000_000L)
            eventually("frame $i handled") { rig.sightings.size >= i }
        }
        val c = rig.recogniser.counters()
        assertTrue(c.armed)
        assertEquals(3L, c.detected)
        assertEquals(3L, c.hits)
        assertEquals(3L, c.mailbox.offered)
        rig.recogniser.stop()
    }
}
