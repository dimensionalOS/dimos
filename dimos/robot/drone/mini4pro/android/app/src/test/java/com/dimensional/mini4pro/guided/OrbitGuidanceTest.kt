package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * M3 Stage C's arithmetic in isolation — [OrbitGuidance] and the two functions
 * [RepositionGuidance] grew for it. No engine, no clock, no aircraft: every number that decides
 * where a circling aircraft goes, checked against hand-computed values.
 *
 * Written to fail loudly for Stage C's landmines:
 *
 *  - **the curvature cap** relaxed, dropped, or turned into dead code — pinned as a *property*
 *    over generated radii (`v²/R ≤ a_max` everywhere in the band), not as three examples, and
 *    with a companion test that the `sqrt` term actually *binds* below R = 18 m so the property
 *    cannot be satisfied by the envelope cap alone
 *  - **a missing `cos(latitude)`** in the join point — invisible at the equator, 21% at 38°N,
 *    and the third place in this project it has to be pinned by name
 *  - **the ±180° wrap** dropped from the swept-angle accumulator, which charges ~360° to one
 *    tick and "completes" the orbit instantly
 *  - the tangential/radial **signs** — a clockwise circle flown anticlockwise, or a radial
 *    correction that pushes outward
 *  - the yaw feed-forward saturating the 30 °/s cap on its own, which is the stated reason
 *    `ORBIT_R_MIN_M` is 5 m and not 2 m
 *  - a gimbal solution with the sign flipped, which aims at the sky
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted across
 * `OrbitGuidanceTest` + `GuidedOrbitTest` + `RepositionGuidanceTest` + `GuidedRepositionTest`,
 * code reverted after each — the table is in `GuidedOrbitTest`'s header, where the engine
 * mutants live too.
 */
class OrbitGuidanceTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7
        const val EPS = 1e-9
    }

    private fun northOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

    private fun eastOf(metres: Double): Double =
        LON + metres / (RepositionGuidance.METRES_PER_DEG * cos(Math.toRadians(LAT)))

    // ------------------------------------------------------- the curvature cap

    @Test
    fun `the tangential cap is min of the envelope and sqrt of a_max times R`() {
        // Hand-computed from a_max = 0.5: sqrt(0.5 * 10) = 2.2360679…
        assertEquals(sqrt(0.5 * 10.0), OrbitGuidance.tangentialCap(10.0), EPS)
        // sqrt(0.5 * 18) = 3.0 exactly — the radius at which the envelope cap takes over.
        assertEquals(3.0, OrbitGuidance.tangentialCap(18.0), EPS)
        // Above it the Q1 envelope binds, not the curvature term.
        assertEquals(GuidedEnvelope.HORIZONTAL_MAX_MS, OrbitGuidance.tangentialCap(50.0), EPS)
        // The smallest circle we will fly: sqrt(0.5 * 5) = 1.5811…
        assertEquals(sqrt(0.5 * 5.0), OrbitGuidance.tangentialCap(OrbitGuidance.R_MIN_M), EPS)
    }

    @Test
    fun `PROPERTY - no orbit in the band is ever asked for more than a_max of centripetal acceleration`() {
        // The whole safety claim, over generated orbits rather than examples: circular motion at
        // v on radius R needs v²/R, and that must never exceed the same a_max the stopping
        // envelope promises. Deterministic seed so a failure is reproducible.
        val random = Random(20260727)
        repeat(2_000) {
            val radius = OrbitGuidance.R_MIN_M +
                random.nextDouble() * (OrbitGuidance.R_MAX_M - OrbitGuidance.R_MIN_M)
            val v = OrbitGuidance.tangentialCap(radius)
            assertTrue(
                "R=$radius asked for v=$v → ${v * v / radius} m/s² of centripetal acceleration",
                v * v / radius <= RepositionGuidance.A_MAX_MS2 + 1e-12,
            )
            assertTrue("R=$radius exceeded the Q1 envelope", v <= GuidedEnvelope.HORIZONTAL_MAX_MS + 1e-12)
            assertTrue("R=$radius produced no speed at all", v > 0.0)
        }
    }

    @Test
    fun `PROPERTY - the curvature term actually binds below 18 m, so it is not dead code`() {
        // A cap that never binds is a safety property with a test that cannot fail — the exact
        // trap `RepositionGuidance.A_MAX_MS2`'s KDoc records for the stopping envelope. Below
        // R = a_max_reciprocal * envelope² = 18 m the sqrt term must be the one that decides.
        val random = Random(1)
        repeat(500) {
            val radius = OrbitGuidance.R_MIN_M + random.nextDouble() * (17.9 - OrbitGuidance.R_MIN_M)
            assertTrue(
                "at R=$radius the envelope cap decided, not the curvature cap",
                OrbitGuidance.tangentialCap(radius) < GuidedEnvelope.HORIZONTAL_MAX_MS,
            )
        }
    }

    @Test
    fun `the tangential speed is ramped at a_max and never stepped`() {
        // One 100 ms tick of a_max = 0.5 m/s² is 0.05 m/s. From rest, that is all the first tick
        // of a circle may command — a stepped tangential speed asks for infinite acceleration at
        // exactly the moment the cap exists to bound.
        assertEquals(0.05, OrbitGuidance.rampedTangential(0.0, 10.0, 100L), EPS)
        assertEquals(0.10, OrbitGuidance.rampedTangential(0.05, 10.0, 100L), EPS)
        // …and it saturates at the cap rather than running past it.
        assertEquals(
            OrbitGuidance.tangentialCap(10.0),
            OrbitGuidance.rampedTangential(2.2, 10.0, 10_000L),
            EPS,
        )
        // A zero or backwards dt never *raises* the speed.
        assertEquals(1.0, OrbitGuidance.rampedTangential(1.0, 10.0, 0L), EPS)
    }

    @Test
    fun `reaching the cap from rest takes the time a_max implies`() {
        var v = 0.0
        var ticks = 0
        while (v < OrbitGuidance.tangentialCap(10.0) - EPS && ticks < 1_000) {
            v = OrbitGuidance.rampedTangential(v, 10.0, 100L)
            ticks++
        }
        // 2.236 m/s at 0.05 m/s per 100 ms tick = 45 ticks = 4.5 s. If this ever collapses to one
        // tick the ramp has been removed.
        assertEquals(45, ticks)
    }

    // ------------------------------------------------- the radial axis is the M3 law

    @Test
    fun `the radial axis is the M3 law under its own smaller cap`() {
        // Outside the circle by 10 m: min(k_p·e = 5, V_RADIAL_MAX = 1, sqrt(2·0.5·10) = 3.16) = 1,
        // pulling *inward*, so negative along the outward unit radial.
        assertEquals(-OrbitGuidance.V_RADIAL_MAX_MS, OrbitGuidance.radialSpeed(10.0), EPS)
        // Close in, the gain decides: k_p · 0.5 = 0.25.
        assertEquals(-0.25, OrbitGuidance.radialSpeed(0.5), EPS)
        // Inside the circle the correction pushes outward — the sign, which a mutation loves.
        assertEquals(0.25, OrbitGuidance.radialSpeed(-0.5), EPS)
        // Exactly on the circle: nothing.
        assertEquals(0.0, OrbitGuidance.radialSpeed(0.0), EPS)
    }

    @Test
    fun `the radial axis calls the same clamp the reposition law does`() {
        // Not a paraphrase of it: the same function, so the braking curve cannot drift apart. At
        // e = 1.5 the stopping envelope sqrt(2·0.5·1.5) = 1.22 is above both k_p·e = 0.75 and the
        // radial cap, so the gain decides and both call sites must agree on that.
        assertEquals(
            RepositionGuidance.clampedSpeed(1.5, OrbitGuidance.V_RADIAL_MAX_MS),
            abs(OrbitGuidance.radialSpeed(1.5)),
            EPS,
        )
    }

    // -------------------------------------------------------------- the signs

    @Test
    fun `clockwise north of the centre flies east, and anticlockwise flies west`() {
        // The one sign check worth doing by hand: an aircraft due north of the centre, going
        // clockwise seen from above, must be moving east.
        val cw = OrbitGuidance.circleVelocity(
            northFromCentreM = 10.0, eastFromCentreM = 0.0,
            radiusM = 10.0, direction = 1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(0.0, cw.north, EPS)
        assertEquals(2.0, cw.east, EPS)

        val ccw = OrbitGuidance.circleVelocity(
            northFromCentreM = 10.0, eastFromCentreM = 0.0,
            radiusM = 10.0, direction = -1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(0.0, ccw.north, EPS)
        assertEquals(-2.0, ccw.east, EPS)
    }

    @Test
    fun `clockwise east of the centre flies south`() {
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 0.0, eastFromCentreM = 10.0,
            radiusM = 10.0, direction = 1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(-2.0, v.north, EPS)
        assertEquals(0.0, v.east, EPS)
    }

    @Test
    fun `outside the circle the radial component pulls inward while the tangent keeps going`() {
        // 14 m north of a 10 m circle: 4 m of radial error, capped at the 1 m/s radial limit and
        // pointing back at the centre (south), with the tangential 2 m/s still going east.
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 14.0, eastFromCentreM = 0.0,
            radiusM = 10.0, direction = 1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(-OrbitGuidance.V_RADIAL_MAX_MS, v.north, EPS)
        assertEquals(2.0, v.east, EPS)
    }

    // ------------------------------------------------- S-3: the envelope bounds ground speed
    //
    // Mutation-checked 2026-07-27, each breakage applied to the shipped source, run, confirmed red
    // and reverted:
    //
    //  | # | mutation | tests that failed |
    //  |---|---|---|
    //  | A | the clamp removed entirely — the defect the flight found | 2 |
    //  | B | per-axis clipping (`coerceIn`) instead of uniform scaling | 2 |
    //  | C | the clamp measured on the tangential term alone, before composition | 2 |
    //  | D | the scale applied to one axis only, so the direction rotates | 4 |
    //  | E | scaled unconditionally, so a slow orbit is *sped up* to the envelope | 9 |
    //
    // `the envelope clamp preserves the commanded direction` scores 0 against A, B and C and only
    // earns its place against D — and that is worth stating rather than leaving to be inferred.
    // Under per-axis clipping (B) this particular geometry is untouched, because neither axis
    // exceeds 3.0 on its own (−1.0 and +3.0); what B breaks is the *magnitude*, which the test
    // above catches. D is the mutation that rotates the vector, and it is the only one that can
    // distinguish "scaled" from "scaled uniformly".

    /**
     * **The case a real flight found**, 2026-07-27
     * (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.1).
     *
     * The radial and tangential components are orthogonal, so bounding each separately leaves the
     * *magnitude* unbounded at `sqrt(v_r² + v_t²)`. With both axes at their caps that is
     * `sqrt(3.0² + 1.0²) = 3.162 m/s`, 5.4 % over the envelope, and nothing downstream clamps it —
     * `StickMapping.toDji` is a pure relabelling. In flight only 3.0021 m/s of it was ever spent,
     * because a settled orbit has almost no radial error; this test is the gust that spends the
     * rest, which is why it has to be written rather than waited for.
     *
     * Ivan settled S-3 as "ground speed", the reading `RepositionGuidance.velocity` has always
     * used. The assertion is therefore on `hypot`, not on either axis.
     */
    @Test
    fun `a full radial correction at full tangential speed still respects the ground-speed envelope`() {
        // Far outside a 30 m circle → radial pinned at V_RADIAL_MAX_MS, tangential at the cap.
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 40.0, eastFromCentreM = 0.0,
            radiusM = 30.0, direction = 1,
            tangentialMs = GuidedEnvelope.HORIZONTAL_MAX_MS, downMs = 0.0,
        )
        val speed = hypot(v.north, v.east)
        // Unclamped this is sqrt(3² + 1²) = 3.162…, which is the mutation to try.
        assertTrue(
            "ground speed $speed exceeds the envelope",
            speed <= GuidedEnvelope.HORIZONTAL_MAX_MS + EPS,
        )
        assertEquals(GuidedEnvelope.HORIZONTAL_MAX_MS, speed, EPS)
    }

    /**
     * And the scaling is **uniform**, not per-axis clipping. Clipping each axis to the envelope
     * would leave this case untouched (neither axis exceeds 3.0 alone) and, where it did bite,
     * would rotate the commanded direction — on a circle, cutting the corner. The direction of the
     * composed vector must survive the clamp exactly.
     */
    @Test
    fun `the envelope clamp preserves the commanded direction`() {
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 40.0, eastFromCentreM = 0.0,
            radiusM = 30.0, direction = 1,
            tangentialMs = GuidedEnvelope.HORIZONTAL_MAX_MS, downMs = 0.0,
        )
        // Unscaled: north = −1.0 (inward), east = +3.0 (tangent). The ratio is what must hold.
        assertEquals(-1.0 / 3.0, v.north / v.east, EPS)
        assertTrue("inward", v.north < 0.0)
        assertTrue("eastward", v.east > 0.0)
    }

    /** A setpoint already inside the envelope is returned untouched — no silent rescaling. */
    @Test
    fun `an orbit within the envelope is not scaled at all`() {
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 30.0, eastFromCentreM = 0.0,
            radiusM = 30.0, direction = 1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(0.0, v.north, EPS)
        assertEquals(2.0, v.east, EPS)
    }

    @Test
    fun `inside the circle the radial component pushes outward`() {
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 6.0, eastFromCentreM = 0.0,
            radiusM = 10.0, direction = 1, tangentialMs = 2.0, downMs = 0.0,
        )
        assertEquals(OrbitGuidance.V_RADIAL_MAX_MS, v.north, EPS)
        assertEquals(2.0, v.east, EPS)
    }

    @Test
    fun `the circling law never commands yaw itself - the engine adds it, and only while circling`() {
        val v = OrbitGuidance.circleVelocity(
            northFromCentreM = 10.0, eastFromCentreM = 3.0,
            radiusM = 10.0, direction = 1, tangentialMs = 2.0, downMs = -0.4,
        )
        assertEquals(0.0, v.yawRateDegPerS, EPS)
        // The vertical axis is passed through untouched: it is the M3 law's, computed elsewhere.
        assertEquals(-0.4, v.down, EPS)
    }

    @Test
    fun `the commanded horizontal speed on a settled circle is exactly the tangential speed`() {
        // On the circle the radial term is zero, so the magnitude is the tangential speed and the
        // curvature cap therefore bounds the *whole* horizontal command, not just one component.
        for (bearing in 0 until 360 step 17) {
            val rad = Math.toRadians(bearing.toDouble())
            val v = OrbitGuidance.circleVelocity(
                northFromCentreM = 12.0 * kotlin.math.cos(rad),
                eastFromCentreM = 12.0 * kotlin.math.sin(rad),
                radiusM = 12.0, direction = 1,
                tangentialMs = OrbitGuidance.tangentialCap(12.0), downMs = 0.0,
            )
            assertEquals(OrbitGuidance.tangentialCap(12.0), hypot(v.north, v.east), 1e-9)
        }
    }

    // --------------------------------------------------- the swept angle and its wrap

    @Test
    fun `the swept angle wraps at the atan2 discontinuity instead of charging a whole turn`() {
        // Due south, where atan2(east, north) jumps from +180 to −180. Without wrap180 this one
        // tick charges −358° and the orbit "completes" (or un-completes) instantly.
        assertEquals(2.0, OrbitGuidance.sweptDeltaDeg(179.0, -179.0, 1), 1e-12)
        // And the other way, for an anticlockwise orbit.
        assertEquals(2.0, OrbitGuidance.sweptDeltaDeg(-179.0, 179.0, -1), 1e-12)
    }

    @Test
    fun `the swept angle crosses due north without a discontinuity`() {
        // `docs/m4-mission-execution.md` §8.4 names due north as the crossing that needs its own
        // test. In *this* convention (atan2, ±180) north is the continuous point and south is the
        // discontinuity — so both are pinned, because the doc's sentence is about the class of bug.
        assertEquals(2.0, OrbitGuidance.sweptDeltaDeg(-1.0, 1.0, 1), 1e-12)
        assertEquals(2.0, OrbitGuidance.sweptDeltaDeg(1.0, -1.0, -1), 1e-12)
    }

    @Test
    fun `a full clockwise turn integrates to exactly 360 degrees, crossing both north and south`() {
        var swept = 0.0
        var previous = OrbitGuidance.bearingDeg(10.0, 0.0) // due north of the centre
        for (step in 1..360) {
            val rad = Math.toRadians(step.toDouble())
            val bearing = OrbitGuidance.bearingDeg(10.0 * kotlin.math.cos(rad), 10.0 * kotlin.math.sin(rad))
            swept += OrbitGuidance.sweptDeltaDeg(previous, bearing, 1)
            previous = bearing
        }
        assertEquals(360.0, swept, 1e-9)
    }

    @Test
    fun `a full anticlockwise turn also integrates upward to 360`() {
        var swept = 0.0
        var previous = OrbitGuidance.bearingDeg(10.0, 0.0)
        for (step in 1..360) {
            val rad = Math.toRadians(-step.toDouble())
            val bearing = OrbitGuidance.bearingDeg(10.0 * kotlin.math.cos(rad), 10.0 * kotlin.math.sin(rad))
            swept += OrbitGuidance.sweptDeltaDeg(previous, bearing, -1)
            previous = bearing
        }
        assertEquals(360.0, swept, 1e-9)
    }

    @Test
    fun `drifting backwards takes the swept angle down, because it is an integral not a counter`() {
        assertEquals(-3.0, OrbitGuidance.sweptDeltaDeg(10.0, 7.0, 1), 1e-12)
    }

    @Test
    fun `bearing is degrees clockwise from north`() {
        assertEquals(0.0, OrbitGuidance.bearingDeg(1.0, 0.0), EPS)
        assertEquals(90.0, OrbitGuidance.bearingDeg(0.0, 1.0), EPS)
        assertEquals(180.0, OrbitGuidance.bearingDeg(-1.0, 0.0), EPS)
        assertEquals(-90.0, OrbitGuidance.bearingDeg(0.0, -1.0), EPS)
    }

    @Test
    fun `the time cap expressed as an angle bounds a request for more turns than there is time for`() {
        // R = 50 m at the 3 m/s envelope cap: ω = 0.06 rad/s, × 180 s = 10.8 rad = 618.7°.
        assertEquals(Math.toDegrees(180.0 * 3.0 / 50.0), OrbitGuidance.maxSweepDeg(50.0), 1e-9)
        // Less than two turns fit on the biggest circle, which is worth knowing: a 2-turn request
        // at R = 50 is truncated and announced rather than silently cut off by the clock.
        assertTrue(OrbitGuidance.maxSweepDeg(50.0) < 2.0 * OrbitGuidance.DEGREES_PER_TURN)
        assertTrue(OrbitGuidance.maxSweepDeg(50.0) > OrbitGuidance.DEGREES_PER_TURN)
    }

    // ------------------------------------------------------------- the join point

    @Test
    fun `the join point is on the circle, nearest the aircraft, WITH the cos(latitude) factor`() {
        // Aircraft 100 m due east of the centre at 38°N; a 30 m circle. The nearest point is 30 m
        // due east of the centre — and the longitude that is 30 m east at this latitude is
        // 30 / (111194.93 · cos 38°), which is **21% further** than the equator arithmetic gives.
        val (joinLat, joinLon) = OrbitGuidance.joinPoint(LAT, LON, 30.0, LAT, eastOf(100.0))
        assertEquals(LAT, joinLat, 1e-12)
        assertEquals(eastOf(30.0), joinLon, 1e-12)

        // Said again as the thing the mutation would produce, so the failure names itself: without
        // cos(latitude) the join point lands 23.6 m east of the centre, not 30.
        val equatorial = LON + 30.0 / RepositionGuidance.METRES_PER_DEG
        assertNotEquals(equatorial, joinLon, 1e-9)
        val wouldBe = RepositionGuidance.horizontalMetres(LAT, LON, LAT, equatorial)
        assertEquals(30.0 * cos(Math.toRadians(LAT)), wouldBe, 0.05)
    }

    @Test
    fun `the join point is the near side of the circle, never the far side`() {
        // 100 m north of the centre, 30 m circle → 30 m north of the centre, i.e. 70 m of leg.
        val (lat, lon) = OrbitGuidance.joinPoint(LAT, LON, 30.0, northOf(100.0), LON)
        assertEquals(30.0, RepositionGuidance.horizontalMetres(LAT, LON, lat, lon), 1e-6)
        assertEquals(70.0, RepositionGuidance.horizontalMetres(northOf(100.0), LON, lat, lon), 1e-3)
    }

    @Test
    fun `the join point is always exactly one radius from the centre, from any bearing`() {
        for (bearing in 0 until 360 step 13) {
            val rad = Math.toRadians(bearing.toDouble())
            val (lat, lon) = RepositionGuidance.offsetCoordinate(
                LAT, LON, 60.0 * kotlin.math.cos(rad), 60.0 * kotlin.math.sin(rad),
            )
            val (joinLat, joinLon) = OrbitGuidance.joinPoint(LAT, LON, 25.0, lat, lon)
            assertEquals(25.0, RepositionGuidance.horizontalMetres(LAT, LON, joinLat, joinLon), 1e-3)
        }
    }

    @Test
    fun `from the exact centre the join point is due north, not a NaN`() {
        val (lat, lon) = OrbitGuidance.joinPoint(LAT, LON, 20.0, LAT, LON)
        assertEquals(northOf(20.0), lat, 1e-12)
        assertEquals(LON, lon, 1e-12)
    }

    @Test
    fun `offsetCoordinate inverts nedMetres, cos(latitude) and all`() {
        val (lat, lon) = RepositionGuidance.offsetCoordinate(LAT, LON, 40.0, -25.0)
        val (north, east) = RepositionGuidance.nedMetres(LAT, LON, lat, lon)
        assertEquals(40.0, north, 1e-9)
        assertEquals(-25.0, east, 1e-9)
    }

    // ------------------------------------------------------------------- the yaw law

    @Test
    fun `a stale heading commands exactly zero yaw, never a guess`() {
        assertEquals(
            0.0,
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = null),
            EPS,
        )
        assertEquals(
            0.0,
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = Double.NaN),
            EPS,
        )
    }

    @Test
    fun `already nose-on, the yaw rate is the orbital rate alone`() {
        // 10 m due north of the centre means the centre is due *south*, bearing 180. A nose on
        // 180 has no error, so all that is left is the feed-forward v/R = 2/10 rad/s = 11.459 °/s.
        assertEquals(
            Math.toDegrees(2.0 / 10.0),
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = 180.0),
            1e-9,
        )
        // Anticlockwise, the aircraft turns the other way.
        assertEquals(
            -Math.toDegrees(2.0 / 10.0),
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, -1, 2.0, headingDeg = 180.0),
            1e-9,
        )
    }

    @Test
    fun `a heading error adds a proportional correction on top of the feed-forward`() {
        // Nose 10° short of the centre: k_yaw·10 = 10 °/s of correction plus 11.459 of orbit.
        assertEquals(
            OrbitGuidance.K_YAW_PER_S * 10.0 + Math.toDegrees(2.0 / 10.0),
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = 170.0),
            1e-9,
        )
    }

    @Test
    fun `the heading error wraps, so a nose just past north corrects the short way`() {
        // Aircraft due *south* of the centre: the centre bears 0°. A nose at 359° is 1° short, not
        // 359° long, and the correction must be +1 °/s rather than a full-cap swing the wrong way.
        val rate = OrbitGuidance.yawRate(-10.0, 0.0, 10.0, 1, 0.0, headingDeg = 359.0)
        assertEquals(1.0 * OrbitGuidance.K_YAW_PER_S, rate, 1e-9)
    }

    @Test
    fun `the yaw rate is clamped to the envelope, no new constant`() {
        // The centre bears 180 from a nose at 0, so the correction is the largest a bearing can
        // ask for and the clamp is the only thing that can be answering — whatever the clamp is.
        val far = OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = 0.0)
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, far, EPS)
        val other = OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = 359.0)
        assertEquals(-GuidedEnvelope.YAW_RATE_MAX_DEGS, other, EPS)
    }

    @Test
    fun `the feed-forward at the 5 m radius floor, and the argument that used to bind`() {
        // **This test recorded a constraint that no longer binds, and says so rather than being
        // quietly renumbered.** At the old 30 °/s clamp the feed-forward was the reason `R_MIN_M`
        // is 5 m and not 2 m: sqrt(a_max/R)·57.3 is 18.1 °/s at 5 m, inside 30 with 11.9 left for
        // the correction, while at 2 m it is 28.6 °/s with almost nothing left. Raising the clamp
        // to 90 on 2026-07-27 dissolved that argument — 28.6 is now a third of the envelope.
        //
        // The arithmetic is still pinned, because it is the arithmetic the floor was chosen from
        // and a change to `A_MAX_MS2` still has to be seen. What is no longer asserted is that it
        // *binds*, because it does not. `R_MIN_M` now rests on geometry and fix noise — a 5 m
        // circle flown on a fix with metre-scale noise is already a marginal circle — which is a
        // weaker argument than the one it had, and is recorded in `docs/open-questions.md`.
        val atFloor = Math.toDegrees(OrbitGuidance.tangentialCap(OrbitGuidance.R_MIN_M) / OrbitGuidance.R_MIN_M)
        assertEquals(18.1, atFloor, 0.1)
        val atTwoMetres = Math.toDegrees(OrbitGuidance.tangentialCap(2.0) / 2.0)
        assertEquals(28.6, atTwoMetres, 0.1)
        // What still has to hold: the feed-forward alone must never eat the whole envelope, or the
        // correction term would have nothing left to steer with at the tightest legal radius.
        assertTrue(atFloor < GuidedEnvelope.YAW_RATE_MAX_DEGS * 0.7)
    }

    @Test
    fun `THE ROI LAW IS THE ORBIT'S OWN LAW, on the ring`() {
        // **The substitution this file has to justify.** On 2026-07-27 the engine stopped calling
        // `OrbitGuidance.yawRate` and started pointing a circle through the ROI system, so that a
        // join leg turns towards the centre instead of flying to it sideways and so that one
        // pointing system exists instead of two. Replacing a flight-verified law is only honest if
        // the replacement is the same arithmetic, and this asserts it rather than asserting it in
        // prose — across radii, both directions, and every position on the circle.
        //
        // Why they agree: `RoiGuidance.bearingRateDegPerS` computes how fast the bearing to a
        // target moves given the velocity about to be commanded. On the ring that velocity is
        // tangential and the range is R, so it evaluates to v/R — which is exactly the orbital
        // feed-forward `OrbitGuidance.yawRate` computes analytically. The general form and the
        // special case meet, and off the ring the general form is the more correct of the two
        // because it uses the geometry the aircraft is in rather than the one it should be in.
        for (radius in listOf(5.0, 12.0, 20.0, 50.0)) {
            for (direction in listOf(1, -1)) {
                val v = OrbitGuidance.tangentialCap(radius)
                for (bearing in 0 until 360 step 15) {
                    val rad = Math.toRadians(bearing.toDouble())
                    val fromCentreNorth = radius * kotlin.math.cos(rad)
                    val fromCentreEast = radius * kotlin.math.sin(rad)
                    // The orbit's own commanded velocity at this point, radial error zero.
                    val setpoint = OrbitGuidance.circleVelocity(
                        northFromCentreM = fromCentreNorth,
                        eastFromCentreM = fromCentreEast,
                        radiusM = radius,
                        direction = direction,
                        tangentialMs = v,
                        downMs = 0.0,
                    )
                    for (heading in listOf(0.0, 47.0, 180.0, 275.0)) {
                        val orbitLaw = OrbitGuidance.yawRate(
                            northFromCentreM = fromCentreNorth,
                            eastFromCentreM = fromCentreEast,
                            radiusM = radius,
                            direction = direction,
                            tangentialMs = v,
                            headingDeg = heading,
                        )
                        // The ROI sees the line of sight *to* the centre, which is the negation.
                        val roiLaw = RoiGuidance.yawRate(
                            losNorthM = -fromCentreNorth,
                            losEastM = -fromCentreEast,
                            headingDeg = heading,
                            commandedNorthMs = setpoint.north,
                            commandedEastMs = setpoint.east,
                        )
                        assertEquals(
                            "R=$radius dir=$direction bearing=$bearing heading=$heading",
                            orbitLaw, roiLaw, 1e-9,
                        )
                    }
                }
            }
        }
    }

    @Test
    fun `OrbitGuidance yawRate is kept as the reference the substitution is measured against`() {
        // It has no caller since 2026-07-27 and that is deliberate rather than an oversight: it is
        // the flight-verified law the test above compares the shipped one to, and deleting it would
        // leave that comparison with nothing on the other side. If it is ever removed, the
        // equivalence test goes with it and the substitution stops being measured.
        assertEquals(
            GuidedEnvelope.YAW_RATE_MAX_DEGS,
            OrbitGuidance.yawRate(10.0, 0.0, 10.0, 1, 2.0, headingDeg = 0.0),
            EPS,
        )
    }

    // ---------------------------------------------------------- the gimbal solution

    @Test
    fun `the gimbal solution is minus atan2 of altitude over horizontal distance, negative down`() {
        // Hand-computed: equal height and distance is 45° down, and down is negative (DJI's
        // convention and QGC's, with no sign flip anywhere between here and the camera).
        assertEquals(-45.0, OrbitGuidance.gimbalPitchDeg(10.0, 10.0), 1e-9)
        // Directly overhead: straight down.
        assertEquals(-90.0, OrbitGuidance.gimbalPitchDeg(10.0, 0.0), 1e-9)
        // At the target's own height: level, looking at the horizon, which is the honest answer.
        assertEquals(0.0, OrbitGuidance.gimbalPitchDeg(0.0, 20.0), 1e-9)
        // Twice as high as far: −63.435°.
        assertEquals(-63.43494882292201, OrbitGuidance.gimbalPitchDeg(20.0, 10.0), 1e-9)
    }

    @Test
    fun `on a steady circle the gimbal solution does not move, which is why 500 ms is enough`() {
        // Radius and altitude both held → atan2 has two fixed arguments → the camera sits still.
        val first = OrbitGuidance.gimbalPitchDeg(20.0, 30.0)
        val later = OrbitGuidance.gimbalPitchDeg(20.0, 30.0)
        assertEquals(first, later, 0.0)
        // And a 1 m radial wobble on a 30 m circle at 20 m moves it by well under the deadband.
        assertTrue(abs(OrbitGuidance.gimbalPitchDeg(20.0, 31.0) - first) < OrbitGuidance.GIMBAL_DEADBAND_DEG)
    }

    // ------------------------------------------------- the arrival test, now shared

    @Test
    fun `the shared arrival test withholds arrival when the velocity feed cannot vouch for a speed`() {
        // A dead velocity feed and a hover are the same bytes on this airframe, so a null speed
        // must never be read as a settled zero.
        assertNull(null as Double?)
        assertTrue(RepositionGuidance.settled(1.0, 0.2, speedMs = 0.1))
        assertTrue(!RepositionGuidance.settled(1.0, 0.2, speedMs = null))
        assertTrue(!RepositionGuidance.settled(1.0, 0.2, speedMs = 0.9))
        assertTrue(!RepositionGuidance.settled(9.0, 0.2, speedMs = 0.1))
        assertTrue(!RepositionGuidance.settled(1.0, 5.0, speedMs = 0.1))
        // An unknowable vertical error does not block arrival — the graduated treatment.
        assertTrue(RepositionGuidance.settled(1.0, null, speedMs = 0.1))
    }
}
