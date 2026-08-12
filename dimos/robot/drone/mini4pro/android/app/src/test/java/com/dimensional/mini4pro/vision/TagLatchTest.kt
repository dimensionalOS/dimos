package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **"We have actually seen a tag at this site, this flight."**
 *
 * The one place in this package where the measured false-id rate has a consequence. The comparison
 * found **2 false ids in 1978 frames** for this detector against OpenCV's zero — both single frames,
 * both alongside the correct id, both where the real tag was small — and said in as many words that
 * a two-of-three temporal filter would erase them and that *this measurement did not test one*.
 * This is that filter, and these are its tests.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | latch on the first sighting — no filter at all | **12** |
 * | no window: evidence accumulates forever | 3 |
 * | the window restarts on every sighting | 3 |
 * | sightings after the latch are ignored | 3 |
 * | `observe` reports the latching edge on every sighting | 2 |
 * | keeps the newest fix rather than the largest | 1 |
 * | `reset` leaves unlatched evidence behind | 1 |
 *
 * The last row measured **zero** on the first pass, and that was a defect in the test rather than a
 * surviving property: it built its unlatched candidate *after* something had already latched, where
 * `observe` records none. Fixed in `resetClearsUnlatchedEvidenceUnderneathIt`.
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 */
class TagLatchTest {

    private val s = 1_000_000_000L

    private fun det(id: Int, px: Double = 100.0) =
        TagDetection(id = id, hamming = 0, centreX = 960.0, centreY = 540.0, longestEdgePixels = px, decisionMargin = 50.0)

    private fun fix(id: Int, n: Double, e: Double, px: Double = 100.0, at: Long = 0) =
        TagFix(tagId = id, northM = n, eastM = e, fromHeightM = 3.0, atNanos = at, pixelSize = px)

    // ──────────────────────────────────────────────── the filter

    @Test
    fun oneSightingIsNotEnough() {
        val latch = TagLatch()
        assertFalse(latch.observe(det(0), fix(0, 1.0, 2.0), 0))
        assertNull(latch.latched())
        assertFalse(latch.isLatched())
    }

    @Test
    fun twoSightingsAreNotEnoughEither() {
        val latch = TagLatch()
        assertFalse(latch.observe(det(0), null, 0))
        assertFalse(latch.observe(det(0), null, 100_000_000))
        assertNull(latch.latched())
    }

    @Test
    fun threeSightingsInsideTheWindowLatch() {
        val latch = TagLatch()
        assertFalse(latch.observe(det(0), fix(0, 1.0, 2.0), 0))
        assertFalse(latch.observe(det(0), fix(0, 1.1, 2.1), 100_000_000))
        assertTrue("the third should latch", latch.observe(det(0), fix(0, 1.2, 2.2), 200_000_000))
        val held = latch.latched()
        assertNotNull(held)
        assertEquals(0, held!!.tagId)
        assertEquals(3, held.sightings)
        assertEquals(0L, held.firstSeenNanos)
        assertEquals(200_000_000L, held.lastSeenNanos)
    }

    /**
     * **The measured failure, reproduced.** Two isolated false ids among many true ones, spread far
     * apart, exactly as they appeared in 1978 frames: neither latches, and the real tag does.
     */
    @Test
    fun isolatedFalseIdsNeverLatch() {
        val latch = TagLatch()
        // The true tag, and beside it in the same frame a one-frame false id — which is exactly how
        // both measured false positives appeared: `0 102` and `0 46`, alongside the real id, where
        // the real tag was small.
        latch.observe(det(102, px = 19.0), null, 0)
        latch.observe(det(0), fix(0, 0.0, 0.0), 0)
        latch.observe(det(0), fix(0, 0.0, 0.0), 100_000_000)
        // The other false id, five minutes later — far outside the window, so it cannot combine
        // with the first one however many minutes pass.
        latch.observe(det(46, px = 18.0), null, 300 * s)
        assertNull("two isolated false ids must never latch", latch.latched())

        // The real tag, three times inside a window, does.
        latch.observe(det(0), fix(0, 0.0, 0.0), 300 * s + 100_000_000)
        latch.observe(det(0), fix(0, 0.0, 0.0), 300 * s + 200_000_000)
        assertTrue(latch.observe(det(0), fix(0, 0.0, 0.0), 300 * s + 300_000_000))
        assertEquals("the real tag must be the one latched", 0, latch.latched()!!.tagId)
    }

    /**
     * **The window is what makes three sightings one observation.** Three of the same false id,
     * spread out, must not accumulate into a latch — which is exactly what a decay-free counter
     * would let them do.
     */
    @Test
    fun evidenceSpreadPastTheWindowDoesNotAccumulate() {
        val latch = TagLatch()
        latch.observe(det(102), null, 0)
        latch.observe(det(102), null, 3 * s)
        latch.observe(det(102), null, 6 * s)
        assertNull("three sightings three seconds apart are not one observation", latch.latched())
        // But three *inside* the window, right after, do latch — the counter restarts rather than
        // being poisoned forever.
        latch.observe(det(102), null, 6 * s + 100_000_000)
        assertTrue(latch.observe(det(102), null, 6 * s + 200_000_000))
    }

    /** Two ids racing: only the one that gets three inside a window wins, and the loser is dropped. */
    @Test
    fun twoCandidatesAreCountedSeparatelyAndOnlyOneWins() {
        val latch = TagLatch()
        latch.observe(det(7), null, 0)
        latch.observe(det(9), null, 10_000_000)
        latch.observe(det(7), null, 20_000_000)
        latch.observe(det(9), null, 30_000_000)
        assertTrue(latch.observe(det(7), null, 40_000_000))
        assertEquals(7, latch.latched()!!.tagId)
        // 9's evidence is gone: the board this is aimed at is one pad, and a second id is the same
        // site rather than a competing one.
        latch.observe(det(9), null, 50_000_000)
        assertEquals(7, latch.latched()!!.tagId)
    }

    // ─────────────────────────────────────────────────── which fix it keeps

    /**
     * **The largest tag's fix, not the newest.** Restricted to tags at least 40 px across the
     * focal-length fit's residual is 0.051 m; including everything down to 14 px triples it to
     * 0.130. Keeping the newest would let the many poor fixes outvote the few good ones.
     */
    @Test
    fun theLargestTagsFixIsTheOneKept() {
        val latch = TagLatch()
        latch.observe(det(0, px = 20.0), fix(0, 1.0, 1.0, px = 20.0), 0)
        latch.observe(det(0, px = 20.0), fix(0, 1.0, 1.0, px = 20.0), 10_000_000)
        latch.observe(det(0, px = 140.0), fix(0, 5.0, 6.0, px = 140.0), 20_000_000)
        assertEquals(5.0, latch.latched()!!.fix!!.northM, 1e-9)
        // A later, smaller detection must not displace it.
        latch.observe(det(0, px = 22.0), fix(0, 9.9, 9.9, px = 22.0), 30_000_000)
        assertEquals(5.0, latch.latched()!!.fix!!.northM, 1e-9)
        // A later, larger one must.
        latch.observe(det(0, px = 300.0), fix(0, 2.0, 3.0, px = 300.0), 40_000_000)
        assertEquals(2.0, latch.latched()!!.fix!!.northM, 1e-9)
    }

    /**
     * **Seen but not placed is a real state.** No position fix, no heading, or a camera not near
     * nadir: the latch still fires, because "there is a tag here" is by itself the reason to spend
     * CPU on the descent.
     */
    @Test
    fun aTagCanLatchWithNoPositionAtAll() {
        val latch = TagLatch()
        latch.observe(det(0), null, 0)
        latch.observe(det(0), null, 10_000_000)
        assertTrue(latch.observe(det(0), null, 20_000_000))
        val held = latch.latched()!!
        assertEquals(0, held.tagId)
        assertNull("a fix that could not be made must be null, not zero", held.fix)
    }

    /** And a later fix fills it in, rather than the latch staying blind for the rest of the flight. */
    @Test
    fun aLaterFixFillsInAPositionlessLatch() {
        val latch = TagLatch()
        repeat(3) { latch.observe(det(0), null, it * 10_000_000L) }
        assertNull(latch.latched()!!.fix)
        latch.observe(det(0, px = 50.0), fix(0, 3.0, 4.0, px = 50.0), 100_000_000)
        assertEquals(3.0, latch.latched()!!.fix!!.northM, 1e-9)
    }

    // ────────────────────────────────────────────────────── after the latch

    /** `observe` returns true exactly once, so a caller can announce the event without its own state. */
    @Test
    fun theLatchingEdgeIsReportedOnceOnly() {
        val latch = TagLatch()
        latch.observe(det(0), null, 0)
        latch.observe(det(0), null, 10_000_000)
        assertTrue(latch.observe(det(0), null, 20_000_000))
        repeat(5) { assertFalse(latch.observe(det(0), null, 30_000_000L + it)) }
    }

    /** Post-latch sightings refresh the timestamp and the count. A frozen latch is a stale one. */
    @Test
    fun sightingsAfterTheLatchStillCount() {
        val latch = TagLatch()
        repeat(3) { latch.observe(det(0), null, it * 10_000_000L) }
        assertEquals(3, latch.latched()!!.sightings)
        latch.observe(det(0), null, 500 * s)
        val held = latch.latched()!!
        assertEquals(4, held.sightings)
        assertEquals(500 * s, held.lastSeenNanos)
    }

    /** A different id after the latch changes nothing at all. */
    @Test
    fun aDifferentIdAfterTheLatchIsIgnored() {
        val latch = TagLatch()
        repeat(3) { latch.observe(det(0), fix(0, 1.0, 1.0), it * 10_000_000L) }
        latch.observe(det(99), fix(99, 50.0, 50.0), 100_000_000)
        val held = latch.latched()!!
        assertEquals(0, held.tagId)
        assertEquals(1.0, held.fix!!.northM, 1e-9)
        assertEquals(3, held.sightings)
    }

    /** A frame with nothing in it is not evidence against. Detection is 92 % at best above 7 m. */
    @Test
    fun aMissIsNotEvidenceAgainst() {
        val latch = TagLatch()
        latch.observe(det(0), null, 0)
        latch.observedNothing()
        latch.observe(det(0), null, 10_000_000)
        latch.observedNothing()
        assertTrue(latch.observe(det(0), null, 20_000_000))
    }

    // ──────────────────────────────────────────────────────────── lifetime

    @Test
    fun resetClearsTheLatch() {
        val latch = TagLatch()
        repeat(3) { latch.observe(det(0), fix(0, 1.0, 1.0), it * 10_000_000L) }
        latch.reset()
        assertNull(latch.latched())
        assertFalse(latch.isLatched())
    }

    /**
     * **Reset clears the evidence *under* the latch too**, and that is the half a first attempt at
     * this test missed: it built its unlatched candidate *after* something had already latched,
     * where `observe` never records one at all, so the mutation "reset leaves unlatched evidence
     * behind" survived with zero failures. The candidate has to exist while nothing is latched.
     *
     * What it protects: a new flight inheriting two sightings from the last one would latch on the
     * *first* frame of the new takeoff — the filter that exists to defeat a false id silently
     * reduced to one frame, on the flight where the takeoff point has changed.
     */
    @Test
    fun resetClearsUnlatchedEvidenceUnderneathIt() {
        val latch = TagLatch()
        // Two sightings of id 5 — one short of latching, and nothing latched.
        assertFalse(latch.observe(det(5), null, 0))
        assertFalse(latch.observe(det(5), null, 10_000_000))
        assertNull(latch.latched())

        latch.reset()

        // The full three are needed again from scratch. Without the clear, the first of these
        // would be the third sighting and would latch.
        assertFalse("reset left evidence behind", latch.observe(det(5), null, 20_000_000))
        assertFalse("reset left evidence behind", latch.observe(det(5), null, 30_000_000))
        assertTrue(latch.observe(det(5), null, 40_000_000))
    }

    @Test
    fun theDefaultsAreTheOnesTheMeasurementArguesFor() {
        // Three, because both measured false ids were single frames and this is one frame of margin.
        assertEquals(3, TagLatch.DEFAULT_MIN_SIGHTINGS)
        // Two seconds: up to twenty chances at the 10 Hz cap, so a 15 % detection rate still latches.
        assertEquals(2L, TagLatch.DEFAULT_WINDOW_NANOS / s)
    }

    /** The count is configurable, because `maxhamming` is untested and the right number may move. */
    @Test
    fun theFilterStrengthIsConfigurable() {
        val strict = TagLatch(minSightings = 5)
        repeat(4) { assertFalse(strict.observe(det(0), null, it * 10_000_000L)) }
        assertTrue(strict.observe(det(0), null, 40_000_000))
    }
}
