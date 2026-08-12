package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Heading-follows-course — the law only. What the *engine* does with it (the ROI outranking it, the
 * announcement, the flag, and yaw staying exactly zero everywhere the bridge does not generate it)
 * is next door in `GuidedMissionTest` and `GuidedRepositionTest`.
 *
 * Design authority: `docs/decisions/2026-07-27-heading-follows-course.md`, requested by Ivan after
 * watching a leg flown with the nose fixed.
 *
 * Written to fail loudly for:
 *
 *  - **a new envelope**. This is the orbit's law with the feed-forward dropped; if the gain or the
 *    clamp ever stop being [OrbitGuidance.K_YAW_PER_S] and [GuidedEnvelope.YAW_RATE_MAX_DEGS], the
 *    tests that name those constants go red.
 *  - **the nose hunting on top of the target** — the hold-heading regime inside the arrival radius.
 *  - **a stale heading guessed at** rather than commanding zero, which is the mutation the orbit's
 *    own suite already kills four ways and which this law must not reintroduce.
 *  - the sign flipped, which points the camera exactly backwards and is invisible in a symmetric
 *    test.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one at a time, applied to the shipped source, the **whole** suite
 * run, confirmed red, reverted. Counts are failing tests across all 1802.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the hold-inside-the-arrival-radius regime dropped (the nose hunts on the target) | 1 |
 *  | a stale heading guessed at zero instead of commanding no yaw | 1 |
 *  | the 30 °/s clamp removed | 5 |
 *  | the sign flipped (the nose turns away from the target) | 6 |
 *
 * The clamp and the sign are cheap to break and expensive to miss, which is why they score high: the
 * engine suites see them too, because a leg that yaws the wrong way is a leg every goto test notices.
 */
class HeadingGuidanceTest {

    @Test
    fun `flying due north with the nose east turns the aircraft left, at the cap`() {
        // Target 50 m north. Nose at 090; the error is −90°, so the aircraft must turn anticlockwise
        // — a negative rate — and the 90° error saturates the 30 °/s clamp.
        val rate = HeadingGuidance.yawRate(errorNorthM = 50.0, errorEastM = 0.0, headingDeg = 90.0)
        assertEquals(-GuidedEnvelope.YAW_RATE_MAX_DEGS, rate, 1e-9)
    }

    @Test
    fun `flying due east with the nose north turns the aircraft right, at the cap`() {
        val rate = HeadingGuidance.yawRate(errorNorthM = 0.0, errorEastM = 50.0, headingDeg = 0.0)
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, rate, 1e-9)
    }

    @Test
    fun `the nose already on the target commands no yaw at all`() {
        assertEquals(0.0, HeadingGuidance.yawRate(50.0, 0.0, 0.0), 1e-9)
        assertEquals(0.0, HeadingGuidance.yawRate(0.0, 50.0, 90.0), 1e-9)
        // South-west, and the bearing convention is atan2(east, north) in (−180, 180].
        assertEquals(0.0, HeadingGuidance.yawRate(-50.0, -50.0, -135.0), 1e-9)
    }

    @Test
    fun `THE LAW IS THE ORBIT'S - the gain is K_YAW_PER_S with no feed-forward`() {
        // A 10° error, well inside the clamp, is exactly k_yaw · error and nothing else. A leg has
        // no orbital rate, so any additional term here would be an invention.
        val heading = 10.0
        val rate = HeadingGuidance.yawRate(errorNorthM = 50.0, errorEastM = 0.0, headingDeg = heading)
        assertEquals(OrbitGuidance.K_YAW_PER_S * -10.0, rate, 1e-9)
    }

    @Test
    fun `the rate is clamped to the existing 30 degrees per second and no new constant`() {
        val rate = HeadingGuidance.yawRate(errorNorthM = -50.0, errorEastM = 1.0, headingDeg = 0.0)
        assertTrue(kotlin.math.abs(rate) <= GuidedEnvelope.YAW_RATE_MAX_DEGS + 1e-9)
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, kotlin.math.abs(rate), 1e-9)
    }

    @Test
    fun `HOLD INSIDE THE ARRIVAL RADIUS - a metre of GPS noise must not swing the nose`() {
        // Inside R_ACCEPT_M the bearing is ill-conditioned: the last couple of metres are flown with
        // the nose already where it ended up. No new constant — this reuses the arrival radius.
        assertEquals(0.0, HeadingGuidance.yawRate(1.5, 0.0, 90.0), 1e-12)
        assertEquals(0.0, HeadingGuidance.yawRate(RepositionGuidance.R_ACCEPT_M, 0.0, 90.0), 1e-12)
        assertTrue(HeadingGuidance.holdingHeading(1.5, 0.0))
        // Just outside, and the loop is live again.
        assertFalse(HeadingGuidance.holdingHeading(RepositionGuidance.R_ACCEPT_M + 0.01, 0.0))
        assertTrue(HeadingGuidance.yawRate(RepositionGuidance.R_ACCEPT_M + 0.01, 0.0, 90.0) != 0.0)
    }

    @Test
    fun `A VERTICAL-ONLY LEG HOLDS HEADING - a pure climb has no direction to face`() {
        // It falls out of the same rule rather than needing its own: no horizontal error means
        // inside the radius by construction.
        assertEquals(0.0, HeadingGuidance.yawRate(0.0, 0.0, 42.0), 1e-12)
        assertTrue(HeadingGuidance.holdingHeading(0.0, 0.0))
    }

    @Test
    fun `A STALE HEADING COMMANDS ZERO - never a guess`() {
        assertEquals(0.0, HeadingGuidance.yawRate(50.0, 0.0, null), 1e-12)
        assertEquals(0.0, HeadingGuidance.yawRate(50.0, 0.0, Double.NaN), 1e-12)
    }

    @Test
    fun `a non-finite error commands zero rather than a NaN yaw rate`() {
        assertEquals(0.0, HeadingGuidance.yawRate(Double.NaN, 0.0, 0.0), 1e-12)
        assertEquals(0.0, HeadingGuidance.yawRate(50.0, Double.POSITIVE_INFINITY, 0.0), 1e-12)
        assertTrue(HeadingGuidance.holdingHeading(Double.NaN, 0.0))
    }

    @Test
    fun `the error wraps the short way round the compass`() {
        // Target due north, nose at 350: the error is +10°, so the aircraft turns *right* by 10°/s
        // rather than left by 350.
        assertEquals(10.0 * OrbitGuidance.K_YAW_PER_S, HeadingGuidance.yawRate(50.0, 0.0, 350.0), 1e-9)
    }
}
