package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * The region-of-interest arithmetic, on its own: the pointing solution against hand-computed
 * angles, the bearing-rate feed-forward, the bounded yaw law, and the close-in regime where both
 * axes stop being worth chasing.
 *
 * What the *engine* does with all of it — the ack, the refusals and their sentences, the scope of
 * the yaw permission, the rate limiter, the abort — is pinned next door in `GuidedRoiTest`, the
 * same split `OrbitGuidanceTest`/`GuidedOrbitTest` uses.
 *
 * The one test in here worth reading rather than counting is
 * `the ROI feed-forward IS the orbit's, generalised`. `docs/decisions/2026-07-27-orbit-yaw.md`
 * argues that an orbit pointing at its centre and an ROI pointing at a click are the same problem;
 * the design brief for this feature turns that into an instruction — *generalise what is there, do
 * not write a second one*. That test is the instruction made checkable: at every bearing, in both
 * directions, the general expression and the circle-specific one agree to the last bit.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, run,
 * confirmed red and reverted. Counts are failing tests across the two ROI suites plus the three
 * guided suites they share code with (`RoiGuidanceTest`, `GuidedRoiTest`, `GuidedOrbitTest`,
 * `OrbitGuidanceTest`, `GuidedRepositionTest`) — **measured, not estimated**. See the table in
 * `GuidedRoiTest`'s KDoc, which carries both halves so the numbers are in one place.
 */
class RoiGuidanceTest {

    private companion object {
        /** `atan2(1, √3) = 30°` and `atan2(√3, 1) = 60°` — the two exact triangles, by hand. */
        val ROOT3 = sqrt(3.0)
    }

    // ------------------------------------------------------- the pointing solution

    @Test
    fun `the pitch solution at hand-computed angles, at several ranges and altitudes`() {
        // −atan2(height above our own datum, horizontal distance to the target), negative down.
        // The target is assumed to be at the datum's ground level, so both quantities are ours.
        val cases = listOf(
            // altitude, horizontal, expected pitch
            Triple(10.0, 10.0, -45.0),
            Triple(3.0, 3.0, -45.0),
            Triple(10.0, 10.0 * ROOT3, -30.0),
            Triple(30.0, 30.0 * ROOT3, -30.0),
            Triple(10.0 * ROOT3, 10.0, -60.0),
            Triple(20.0, 20.0 / ROOT3, -60.0),
            // Level: a target at our own height is straight ahead, and the camera does not tilt.
            Triple(0.0, 50.0, 0.0),
            // Long and low: 5 m up, 100 m out — the shallow end of the useful range.
            Triple(5.0, 100.0, -Math.toDegrees(Math.atan2(5.0, 100.0))),
        )
        for ((altitude, horizontal, expected) in cases) {
            assertEquals(
                "alt=$altitude d=$horizontal",
                expected,
                OrbitGuidance.gimbalPitchDeg(altitude, horizontal),
                1e-9,
            )
        }
    }

    @Test
    fun `the pitch is negative down - a camera aimed at the sky is the sign bug this catches`() {
        // The whole family, not one example: from any positive height the solution must point below
        // the horizon, and it must get steeper as the aircraft closes in.
        var previous = 0.0
        for (d in listOf(100.0, 50.0, 20.0, 10.0, 5.0, 3.0)) {
            val pitch = OrbitGuidance.gimbalPitchDeg(20.0, d)
            assertTrue("d=$d gave $pitch", pitch < 0.0)
            assertTrue("d=$d did not steepen: $pitch vs $previous", pitch < previous)
            previous = pitch
        }
    }

    // -------------------------------------------------------- the feed-forward

    @Test
    fun `the bearing rate of a fixed target, by hand`() {
        // 20 m due north of the target (so the line of sight points south), flying east at 2 m/s.
        // The target slides to the left, so the bearing to it increases — clockwise-positive.
        assertEquals(
            Math.toDegrees(0.1),
            RoiGuidance.bearingRateDegPerS(-20.0, 0.0, velocityNorthMs = 0.0, velocityEastMs = 2.0),
            1e-12,
        )
        // Flying the other way reverses it, and nothing else changes.
        assertEquals(
            -Math.toDegrees(0.1),
            RoiGuidance.bearingRateDegPerS(-20.0, 0.0, velocityNorthMs = 0.0, velocityEastMs = -2.0),
            1e-12,
        )
        // Flying straight *at* the target changes the bearing not at all, however fast.
        assertEquals(
            0.0,
            RoiGuidance.bearingRateDegPerS(-20.0, 0.0, velocityNorthMs = -3.0, velocityEastMs = 0.0),
            1e-12,
        )
    }

    @Test
    fun `the ROI feed-forward IS the orbit's, generalised - at every bearing, both directions`() {
        // The design's instruction was to generalise the orbit's pointing rather than write a second
        // one. This is that claim, checkable: put the aircraft on a circle, give it the tangential
        // velocity the orbit would command, and the general expression must reproduce the
        // circle-specific feed-forward `direction · v_t / R` exactly.
        val radius = 20.0
        val tangential = 2.2360679774997896 // sqrt(a_max · R) at a_max = 0.5, the curvature cap
        for (direction in listOf(1, -1)) {
            for (step in 0 until 36) {
                val beta = Math.toRadians(step * 10.0)
                // The aircraft sits at bearing β from the centre; the line of sight to the centre is
                // the negation of that offset.
                val losNorth = -radius * cos(beta)
                val losEast = -radius * sin(beta)
                // The orbit's own tangential unit vector: direction · rotCW(unit radial).
                val velocityNorth = tangential * direction * -sin(beta)
                val velocityEast = tangential * direction * cos(beta)
                assertEquals(
                    "direction=$direction bearing=${step * 10}",
                    direction * Math.toDegrees(tangential / radius),
                    RoiGuidance.bearingRateDegPerS(losNorth, losEast, velocityNorth, velocityEast),
                    1e-12,
                )
            }
        }
    }

    @Test
    fun `the yaw laws agree on a circle - the ROI at the centre is nose-to-centre`() {
        // The whole-law version of the test above: same target, same geometry, same answer. A
        // circling aircraft told to look at its own centre must be flown by the same numbers whether
        // the instruction arrived as `DO_ORBIT`'s implied ROI or as an explicit `DO_SET_ROI`.
        val radius = 20.0
        val tangential = 2.0
        for (direction in listOf(1, -1)) {
            for (step in 0 until 36) {
                val beta = step * 10.0
                val rad = Math.toRadians(beta)
                val fromCentreNorth = radius * cos(rad)
                val fromCentreEast = radius * sin(rad)
                // Nose 4° off the centre — small enough that neither law clamps.
                val heading = beta + 180.0 + 4.0
                val orbit = OrbitGuidance.yawRate(
                    northFromCentreM = fromCentreNorth,
                    eastFromCentreM = fromCentreEast,
                    radiusM = radius,
                    direction = direction,
                    tangentialMs = tangential,
                    headingDeg = heading,
                )
                val roi = RoiGuidance.yawRate(
                    losNorthM = -fromCentreNorth,
                    losEastM = -fromCentreEast,
                    headingDeg = heading,
                    commandedNorthMs = tangential * direction * -sin(rad),
                    commandedEastMs = tangential * direction * cos(rad),
                )
                assertEquals("direction=$direction bearing=$beta", orbit, roi, 1e-12)
            }
        }
    }

    // --------------------------------------------------------------- the yaw law

    @Test
    fun `the yaw closes the heading error, in the shorter direction, wrapped`() {
        // Target due north at 50 m; nothing is moving, so the feed-forward is zero and what is left
        // is the proportional term alone: k_yaw = 1.0 °/s per degree of error.
        fun rate(headingDeg: Double) = RoiGuidance.yawRate(50.0, 0.0, headingDeg, 0.0, 0.0)

        assertEquals(0.0, rate(0.0), 1e-12)
        assertEquals(-10.0, rate(10.0), 1e-12)
        assertEquals(10.0, rate(350.0), 1e-12)
        // The wrap: pointed just past due south, the shorter way round is *clockwise*, not a
        // 179° swing back the way it came.
        assertEquals(29.0 * -1.0, rate(29.0), 1e-12)
        assertTrue(rate(181.0) > 0.0)
        assertTrue(rate(179.0) < 0.0)
    }

    @Test
    fun `the yaw is clamped to the existing 30 deg per second envelope, both ways`() {
        // A 90° error would ask for 90 °/s under k_yaw = 1.0. No new envelope was invented for the
        // ROI: it is `GuidedEnvelope.YAW_RATE_MAX_DEGS`, the same one the orbit and the operator's
        // own stick are bounded by.
        assertEquals(
            GuidedEnvelope.YAW_RATE_MAX_DEGS,
            RoiGuidance.yawRate(50.0, 0.0, headingDeg = 270.0, 0.0, 0.0),
            1e-12,
        )
        assertEquals(
            -GuidedEnvelope.YAW_RATE_MAX_DEGS,
            RoiGuidance.yawRate(50.0, 0.0, headingDeg = 90.0, 0.0, 0.0),
            1e-12,
        )
        // And the feed-forward cannot escape it either: a fast pass close by asks for more than the
        // cap on its own.
        val fast = RoiGuidance.yawRate(0.0, -4.0, headingDeg = 180.0, commandedNorthMs = 3.0, commandedEastMs = 0.0)
        assertTrue("clamp escaped: $fast", abs(fast) <= GuidedEnvelope.YAW_RATE_MAX_DEGS + 1e-12)
    }

    @Test
    fun `a stale or absent heading commands zero yaw, never a guess`() {
        assertEquals(0.0, RoiGuidance.yawRate(50.0, 0.0, headingDeg = null, 0.0, 0.0), 1e-12)
        assertEquals(0.0, RoiGuidance.yawRate(50.0, 0.0, headingDeg = Double.NaN, 0.0, 0.0), 1e-12)
        assertNull(RoiGuidance.azimuthErrorDeg(50.0, 0.0, headingDeg = null))
    }

    // ------------------------------------------------------------- the close-in regime

    @Test
    fun `inside the minimum range nothing is commanded - neither yaw nor a feed-forward`() {
        // At 1 m of range a metre of GPS drift is a 45° step in bearing, and the aircraft would be
        // asked to pirouette under the operator's subject at the yaw cap.
        for (d in listOf(0.0, 0.5, 1.0, 2.9999)) {
            assertEquals("d=$d", 0.0, RoiGuidance.yawRate(-d, 0.0, headingDeg = 0.0, 0.0, 0.0), 1e-12)
            assertEquals("d=$d", 0.0, RoiGuidance.bearingRateDegPerS(-d, 0.0, 0.0, 3.0), 1e-12)
            assertNull("d=$d", RoiGuidance.azimuthErrorDeg(-d, 0.0, headingDeg = 0.0))
            assertTrue("d=$d", RoiGuidance.tooClose(d))
        }
        // …and at the threshold itself the loop is live again. The boundary is inclusive upward, so
        // a target exactly at the limit is tracked rather than falling into a gap.
        assertTrue(abs(RoiGuidance.yawRate(-RoiGuidance.MIN_RANGE_M, 0.0, 90.0, 0.0, 0.0)) > 0.0)
        assertTrue(!RoiGuidance.tooClose(RoiGuidance.MIN_RANGE_M))
    }

    @Test
    fun `a non-finite range is treated as too close, not as far away`() {
        assertTrue(RoiGuidance.tooClose(Double.NaN))
        assertEquals(0.0, RoiGuidance.yawRate(Double.NaN, 0.0, 0.0, 0.0, 0.0), 1e-12)
        assertEquals(0.0, RoiGuidance.bearingRateDegPerS(Double.NaN, 0.0, 0.0, 0.0), 1e-12)
    }

    // ------------------------------------------------------------- the azimuth error

    @Test
    fun `the azimuth error is what the pitch-only sentence is measured against`() {
        // Target due east at 50 m, nose due north: 90° off, which is emphatically outside the
        // deadband and is exactly the case where an operator needs telling that we are not turning.
        assertEquals(90.0, RoiGuidance.azimuthErrorDeg(0.0, 50.0, headingDeg = 0.0)!!, 1e-12)
        assertTrue(abs(RoiGuidance.azimuthErrorDeg(0.0, 50.0, 0.0)!!) > RoiGuidance.YAW_DEADBAND_DEG)
        // Nose 2° off: the subject is still in frame and there is nothing worth saying.
        assertTrue(abs(RoiGuidance.azimuthErrorDeg(50.0, 0.0, 2.0)!!) < RoiGuidance.YAW_DEADBAND_DEG)
        // The wrap, again, because a 358° error is a 2° error.
        assertEquals(2.0, RoiGuidance.azimuthErrorDeg(50.0, 0.0, headingDeg = -2.0)!!, 1e-12)
        // …and the cases the raw subtraction gets wrong by a whole turn. A target due south with
        // the nose 10° west of north is **10° away clockwise**, not 350° the other way: without the
        // fold, every one of these reads as an enormous error and the pitch-only sentence fires for
        // an aircraft that is very nearly pointed at the thing.
        assertEquals(-10.0, RoiGuidance.azimuthErrorDeg(-50.0, 0.0, headingDeg = -170.0)!!, 1e-12)
        assertEquals(100.0, RoiGuidance.azimuthErrorDeg(0.0, -50.0, headingDeg = 170.0)!!, 1e-12)
        for (heading in 0 until 360) {
            val error = RoiGuidance.azimuthErrorDeg(-50.0, 0.0, heading.toDouble())!!
            assertTrue("heading $heading gave $error", error > -180.0 && error <= 180.0)
        }
    }
}
