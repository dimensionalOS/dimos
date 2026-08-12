package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **The 10 Hz cap**, which is what makes the detector affordable at all.
 *
 * Measured on the aircraft 2026-07-28: uncapped at two threads the detector cost **2.00 cores**
 * against a 0.43-core floor; capped it cost **1.11**. The cap is not a nicety, it is two thirds of
 * the saving.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | no cap at all — `admit` always true | **6** |
 * | the first frame is refused rather than admitted | 3 |
 * | the interval catches up instead of being a floor | 2 |
 * | `reset` does not forget the last admission | 2 |
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 */
class RateCapTest {

    private val ms = 1_000_000L

    @Test
    fun theFirstFrameIsAlwaysAdmitted() {
        // A cap is a limit on how often, not a delay before starting. A landing that had to wait
        // 100 ms for its first look would be paying for nothing.
        assertTrue(RateCap(10.0).admit(0))
        assertTrue(RateCap(10.0).admit(Long.MIN_VALUE + 1))
        assertTrue(RateCap(1.0).admit(5_000_000_000L))
    }

    @Test
    fun framesInsideTheIntervalAreRefused() {
        val cap = RateCap(10.0)
        assertTrue(cap.admit(0))
        assertFalse(cap.admit(50 * ms))
        assertFalse(cap.admit(99 * ms))
        assertTrue(cap.admit(100 * ms))
        assertEquals(2L, cap.admitted)
        assertEquals(2L, cap.refused)
    }

    /**
     * **The interval is a floor, not a schedule.** After a gap, the cap does not fire twice to catch
     * up — a burst of detections is precisely what the CPU budget cannot afford.
     */
    @Test
    fun aGapDoesNotProduceABurst() {
        val cap = RateCap(10.0)
        assertTrue(cap.admit(0))
        // Nothing for a second, then frames every 10 ms.
        assertTrue(cap.admit(1_000 * ms))
        assertFalse(cap.admit(1_010 * ms))
        assertFalse(cap.admit(1_090 * ms))
        assertTrue(cap.admit(1_100 * ms))
        assertEquals(3L, cap.admitted)
    }

    /**
     * **The achieved rate is a multiple of the frame interval, and it is 8 Hz not 10.**
     *
     * This is the behaviour measured on the aircraft — `detector: 121 frames in 15s = 8.1 Hz` at a
     * 10 Hz cap — and it is arithmetic rather than a defect: with frames every 41.5 ms, the first
     * one at or past 100 ms is the third, so the period is 124.5 ms. Pinned here so that nobody
     * later "fixes" the cap to hit 10.0 exactly by catching up, which would reintroduce the burst
     * the test above forbids.
     */
    @Test
    fun againstA24FpsStreamA10HzCapAchievesAbout8Hz() {
        val cap = RateCap(10.0)
        val frameNanos = 41_500_000L
        var admitted = 0
        for (i in 0 until 240) {
            if (cap.admit(i * frameNanos)) admitted++
        }
        // 240 frames is 9.96 s of a 24 fps stream.
        val hz = admitted / (240 * frameNanos / 1e9)
        assertTrue("achieved $hz Hz, expected about 8", hz > 7.5 && hz < 8.5)
    }

    @Test
    fun aZeroOrNegativeRateMeansNoCapAtAll() {
        val cap = RateCap(0.0)
        assertEquals(0L, cap.minIntervalNanos)
        repeat(5) { assertTrue(cap.admit(it.toLong())) }
        assertEquals(5L, cap.admitted)
        assertEquals(0L, cap.refused)
    }

    @Test
    fun resetMakesTheNextFrameTheFirstAgain() {
        val cap = RateCap(10.0)
        assertTrue(cap.admit(1_000 * ms))
        assertFalse(cap.admit(1_010 * ms))
        cap.reset()
        assertTrue(cap.admit(1_020 * ms))
    }

    @Test
    fun theShippedDefaultIsTenHertz() {
        // Ivan's cap. Pinned so a change is a deliberate edit to a test rather than a silent one.
        assertEquals(10.0, RateCap.DEFAULT_HZ, 0.0)
        assertEquals(100_000_000L, RateCap(RateCap.DEFAULT_HZ).minIntervalNanos)
    }
}
