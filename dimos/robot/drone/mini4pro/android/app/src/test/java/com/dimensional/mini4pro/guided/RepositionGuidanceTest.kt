package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt

/**
 * Stage B's arithmetic — the lat/lon → NED conversion and the guidance law — every number
 * that decides where a commanded reposition flies, without an aircraft.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests counted across
 * the three guided suites (this one, `GuidedRepositionTest`, `GuidedStickEngineTest`), code
 * reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | cos(latitude) factor dropped from the east conversion | 2 |
 *  | cos taken of the *longitude* instead of the latitude | 3 |
 *  | stopping envelope removed (`v_cmd = min(k_p·e, v_max)` only) | 3 |
 *  | gain applied unclamped (`v_cmd = k_p·e`, no envelope at all) | 16 |
 *  | horizontal clamp applied per-axis instead of to the vector (path curves) | 3 |
 *  | vertical envelope borrowed from horizontal (±3 instead of ±1.5) | 3 |
 *  | vertical error sign flipped in the law | 5 |
 *  | non-finite error guard removed | 1 |
 *  | longitude delta not normalised across the antimeridian | 1 |
 *
 * The stopping-envelope row only kills because [RepositionGuidance.A_MAX_MS2] was chosen so
 * the braking curve actually binds between ~4 m and ~9 m of error — see its KDoc; at
 * a_max = 1.0 the "safety property" would have been unfalsifiable dead code.
 */
class RepositionGuidanceTest {

    // ------------------------------------------------------------- nedMetres()

    @Test
    fun `a pure latitude offset converts at the metres-per-degree constant`() {
        val (north, east) = RepositionGuidance.nedMetres(38.0, 23.7, 38.001, 23.7)
        assertEquals(111.19, north, 0.01)
        assertEquals(0.0, east, 1e-9)
    }

    @Test
    fun `the east conversion carries the cos-latitude factor - pinned at 38 north, not the equator`() {
        // The landmine: a missing cos factor is invisible at the equator and a 21% error at
        // this project's home latitude. 0.001 deg of longitude at 38 N is 87.6 m, NOT 111.2 m.
        val (north, east) = RepositionGuidance.nedMetres(38.0, 23.7, 38.0, 23.701)
        assertEquals(0.0, north, 1e-9)
        assertEquals(0.001 * RepositionGuidance.METRES_PER_DEG * cos(Math.toRadians(38.0)), east, 1e-6)
        assertEquals(87.62, east, 0.05)
        assertTrue("a missing cos factor would read ~111.2 m here", abs(east - 111.19) > 20.0)
    }

    @Test
    fun `south and west offsets are negative north and east`() {
        val (north, east) = RepositionGuidance.nedMetres(38.001, 23.701, 38.0, 23.7)
        assertTrue(north < 0)
        assertTrue(east < 0)
    }

    @Test
    fun `the longitude delta is normalised across the antimeridian`() {
        // 100 m east of 179.9995° is at -179.999...; the raw delta is ~-360°, which read
        // literally would be a 30,000 km westward reposition.
        val (_, east) = RepositionGuidance.nedMetres(0.0, 179.9995, 0.0, -179.9995)
        assertEquals(0.001 * RepositionGuidance.METRES_PER_DEG, east, 0.01)
    }

    @Test
    fun `horizontalMetres is the plain hypotenuse of the offset`() {
        val (n, e) = RepositionGuidance.nedMetres(38.0, 23.7, 38.0003, 23.7004)
        assertEquals(hypot(n, e), RepositionGuidance.horizontalMetres(38.0, 23.7, 38.0003, 23.7004), 1e-9)
    }

    // -------------------------------------------------------------- velocity()

    @Test
    fun `zero error commands zero on every axis including yaw`() {
        val v = RepositionGuidance.velocity(0.0, 0.0, 0.0)
        assertEquals(StickVelocities.ZERO, v)
    }

    @Test
    fun `yaw rate is always zero - the measured Go-to keeps heading`() {
        assertEquals(0.0, RepositionGuidance.velocity(50.0, -20.0, 5.0).yawRateDegPerS, 0.0)
    }

    @Test
    fun `close in, the gain rules - v equals kp times error`() {
        // At 1.5 m: k_p·e = 0.75, braking sqrt(2·0.5·1.5) = 1.22, envelope 3 — gain wins.
        val v = RepositionGuidance.velocity(1.5, 0.0, null)
        assertEquals(RepositionGuidance.KP_PER_S * 1.5, v.north, 1e-9)
        assertEquals(0.75, v.north, 1e-9)
    }

    @Test
    fun `mid-range, the stopping envelope rules - the approach rides the braking curve`() {
        // At 6 m: k_p·e = 3.0, envelope 3.0, braking sqrt(2·0.5·6) = 2.449 — braking wins.
        // This is the safety property being a real constraint, not decoration.
        val v = RepositionGuidance.velocity(6.0, 0.0, null)
        assertEquals(sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * 6.0), v.north, 1e-9)
        assertTrue(v.north < GuidedEnvelope.HORIZONTAL_MAX_MS)
        assertTrue(v.north < RepositionGuidance.KP_PER_S * 6.0)
    }

    @Test
    fun `far out, the Q1 envelope rules - never faster than 3 m per s however far the target`() {
        assertEquals(3.0, RepositionGuidance.velocity(100.0, 0.0, null).north, 1e-9)
        assertEquals(-3.0, RepositionGuidance.velocity(-1e6, 0.0, null).north, 1e-9)
    }

    @Test
    fun `the horizontal clamp preserves direction - a diagonal target is approached in a straight line`() {
        // Clamping per-axis instead of by vector magnitude would fly 3.0/3.0 = 45° regardless
        // of the true bearing; the vector clamp keeps the 2:1 ratio.
        val v = RepositionGuidance.velocity(80.0, 40.0, null)
        assertEquals(2.0, v.north / v.east, 1e-9)
        assertEquals(3.0, hypot(v.north, v.east), 1e-9)
    }

    @Test
    fun `vertical runs its own Q1 limit - a deep target climbs at exactly 1 point 5`() {
        // error down −10 (aircraft far below the target): full climb, NED down negative.
        val climb = RepositionGuidance.velocity(0.0, 0.0, -10.0)
        assertEquals(-GuidedEnvelope.VERTICAL_MAX_MS, climb.down, 1e-9)
        val descend = RepositionGuidance.velocity(0.0, 0.0, 10.0)
        assertEquals(+1.5, descend.down, 1e-9)
    }

    @Test
    fun `a small vertical error follows the gain with the right sign`() {
        assertEquals(-0.25, RepositionGuidance.velocity(0.0, 0.0, -0.5).down, 1e-9)
        assertEquals(+0.25, RepositionGuidance.velocity(0.0, 0.0, +0.5).down, 1e-9)
    }

    @Test
    fun `a null vertical error commands zero vertical - unknown is not zero error`() {
        val v = RepositionGuidance.velocity(20.0, 0.0, null)
        assertEquals(0.0, v.down, 0.0)
        assertTrue(v.north > 0)
    }

    @Test
    fun `non-finite errors command zero everywhere - garbage in, hover out`() {
        assertEquals(StickVelocities.ZERO, RepositionGuidance.velocity(Double.NaN, 5.0, 0.0))
        assertEquals(StickVelocities.ZERO, RepositionGuidance.velocity(5.0, Double.POSITIVE_INFINITY, 0.0))
        assertEquals(StickVelocities.ZERO, RepositionGuidance.velocity(5.0, 5.0, Double.NaN))
    }

    @Test
    fun `no error can command beyond either envelope or the braking curve`() {
        for (e in doubleArrayOf(0.1, 0.5, 1.0, 2.0, 4.0, 6.0, 9.0, 25.0, 100.0, 1e7)) {
            for (sign in doubleArrayOf(-1.0, 1.0)) {
                val v = RepositionGuidance.velocity(e * sign, e * sign * 0.5, e * sign)
                val horizontal = hypot(v.north, v.east)
                val horizontalError = hypot(e, e * 0.5)
                assertTrue(horizontal <= GuidedEnvelope.HORIZONTAL_MAX_MS + 1e-9)
                assertTrue(horizontal <= sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * horizontalError) + 1e-9)
                assertTrue(abs(v.down) <= GuidedEnvelope.VERTICAL_MAX_MS + 1e-9)
            }
        }
    }

    // ---------------------------------------------------------------- constants

    @Test
    fun `the Stage B constants are the decided numbers - changing the safety argument must be loud`() {
        // Q1's 100 m until 2026-07-30, when Ivan replaced it: "2 km is our new limit"
        // (`docs/decisions/2026-07-30-two-kilometre-envelope.md`).
        assertEquals(2000.0, GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, 0.0)
        assertEquals(0.5, RepositionGuidance.KP_PER_S, 0.0) // design doc §1.4
        assertEquals(0.5, RepositionGuidance.A_MAX_MS2, 0.0)
        assertEquals(2.0, RepositionGuidance.R_ACCEPT_M, 0.0)
        assertEquals(0.5, RepositionGuidance.V_SETTLE_MS, 0.0)
        assertEquals(5, RepositionGuidance.ARRIVE_TICKS)
        assertEquals(10_000L, RepositionGuidance.POSITION_LOST_MS)
    }
}
