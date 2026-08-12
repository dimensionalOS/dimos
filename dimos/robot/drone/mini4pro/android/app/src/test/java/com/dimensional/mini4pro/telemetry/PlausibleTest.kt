package com.dimensional.mini4pro.telemetry

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * The bounds that decide whether a non-coordinate DJI reading is a measurement.
 *
 * The values exercised here are real ones. The accepted cases are taken from the
 * 17 session logs audited in
 * `docs/measurements/2026-07-26-filler-value-audit.md` — including the two flights
 * of 2026-07-26 — so a bound that would have rejected a good flight fails a test
 * rather than an operator. The rejected cases are the observed DJI filler
 * (`4.583662361046586E7`), the exact edges, and the non-finite values.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately in `Plausible.kt`,
 * failing tests counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | attitude range check dropped (`angleOrNull` returns `d`) | 3 |
 *  | velocity range check dropped | 3 |
 *  | altitude range check dropped | 2 |
 *  | `batteryPercentOrNull` upper bound dropped | 1 |
 *  | `batteryPercentOrNull` lower bound dropped | 1 |
 *  | `satelliteCountOrNull` upper bound dropped | 2 |
 *  | non-finite check dropped in `angleOrNull` | 1 |
 *  | non-finite check dropped in `altitudeMOrNull` | 1 |
 *  | attitude made per-member, survivors kept and rejects zeroed | 4 |
 *  | velocity made per-member, survivors kept and rejects zeroed | 3 |
 *  | all five bounds made exclusive (`<=`/`>=` on the edges) | 5 |
 *  | `AttitudeDeg` built as (pitch, roll, yaw) | 1 |
 *  | `VelocityNed` built as (east, north, down) | 1 |
 *  | `MAX_ATTITUDE_DEG` widened to 1e9 | 3 |
 *  | `MAX_SPEED_MS` widened to 1e9 | 3 |
 *
 * Fifteen mutations, fifteen killed. The per-member mutations are the ones worth
 * reading: they model the *plausible* mistake — keeping the members that passed —
 * and the suite rejects it, which is the whole argument of `Plausible`'s
 * all-or-nothing rule.
 */
class PlausibleTest {

    /** What `KeyHomeLocation` returned with no home point, in both fields, at 1 Hz. */
    private val djiFiller = 4.583662361046586E7

    // ── attitude ──────────────────────────────────────────────────────────────

    /** The widest attitude in the two real flights: roll 65.4°, pitch −63.6°. */
    @Test
    fun acceptsTheWidestAttitudeInTheRealFlights() {
        val a = Plausible.attitudeOrNull(65.4, -63.6, -172.9)!!
        assertEquals(65.4, a.roll, 0.0)
        assertEquals(-63.6, a.pitch, 0.0)
        assertEquals(-172.9, a.yaw, 0.0)
    }

    @Test
    fun rejectsTheMeasuredFillerInAnyAttitudeMember() {
        assertNull(Plausible.attitudeOrNull(djiFiller, 0.0, 90.0))
        assertNull(Plausible.attitudeOrNull(0.0, djiFiller, 90.0))
        assertNull(Plausible.attitudeOrNull(0.0, 1.0, djiFiller))
    }

    /**
     * All-or-nothing: two thirds of a reading we have decided not to believe is
     * still a horizon QGC will draw.
     */
    @Test
    fun oneBadAngleDiscardsTheWholeAttitude() {
        assertNull(Plausible.attitudeOrNull(200.0, 4.5, 121.4))
        assertNull(Plausible.attitudeOrNull(-0.2, 4.5, null))
    }

    /** ±180° is the bound of the representation, and both ends are orientations. */
    @Test
    fun attitudeBoundsAreInclusive() {
        assertNotNull(Plausible.attitudeOrNull(180.0, -180.0, 180.0))
        assertNull(Plausible.attitudeOrNull(Math.nextUp(180.0), 0.0, 0.0))
        assertNull(Plausible.attitudeOrNull(0.0, Math.nextDown(-180.0), 0.0))
        assertNull(Plausible.attitudeOrNull(0.0, 0.0, Math.nextUp(180.0)))
    }

    @Test
    fun rejectsNonFiniteAngles() {
        assertNull(Plausible.attitudeOrNull(Double.NaN, 0.0, 0.0))
        assertNull(Plausible.attitudeOrNull(0.0, Double.POSITIVE_INFINITY, 0.0))
        assertNull(Plausible.attitudeOrNull(0.0, 0.0, Double.NEGATIVE_INFINITY))
    }

    // ── velocity ──────────────────────────────────────────────────────────────

    /** The fastest tick in the two real flights: vn 8.1, ve −7.3, vd −2.2 m/s. */
    @Test
    fun acceptsTheFastestVelocityInTheRealFlights() {
        val v = Plausible.velocityOrNull(8.1, -7.3, -2.2)!!
        assertEquals(8.1, v.north, 0.0)
        assertEquals(-7.3, v.east, 0.0)
        assertEquals(-2.2, v.down, 0.0)
    }

    @Test
    fun rejectsTheMeasuredFillerInAnyVelocityComponent() {
        assertNull(Plausible.velocityOrNull(djiFiller, 0.0, 0.0))
        assertNull(Plausible.velocityOrNull(0.0, djiFiller, 0.0))
        assertNull(Plausible.velocityOrNull(0.0, 0.0, djiFiller))
    }

    /** Ground speed and course over ground need north and east together. */
    @Test
    fun oneBadComponentDiscardsTheWholeVelocity() {
        assertNull(Plausible.velocityOrNull(101.0, 0.1, 0.0))
        assertNull(Plausible.velocityOrNull(0.4, null, 0.0))
        assertNull(Plausible.velocityOrNull(0.4, 0.1, Double.NaN))
    }

    /** A stationary aircraft reports exact zeros on all three axes, for hours. */
    @Test
    fun acceptsAllZeroVelocity() {
        val v = Plausible.velocityOrNull(0.0, 0.0, 0.0)!!
        assertEquals(0.0, v.north, 0.0)
        assertEquals(0.0, v.east, 0.0)
        assertEquals(0.0, v.down, 0.0)
    }

    @Test
    fun velocityBoundsAreInclusive() {
        assertNotNull(Plausible.velocityOrNull(100.0, -100.0, 100.0))
        assertNull(Plausible.velocityOrNull(Math.nextUp(100.0), 0.0, 0.0))
        assertNull(Plausible.velocityOrNull(0.0, Math.nextDown(-100.0), 0.0))
    }

    // ── altitude ──────────────────────────────────────────────────────────────

    /**
     * Both real halves of the AMSL sum: the takeoff-relative altitude at the top
     * of flight1 (9.9 m) and the widest AMSL datum measured across the two days
     * (103.17 m on 2026-07-25, 59.18 m on 2026-07-26).
     */
    @Test
    fun acceptsTheRealAltitudes() {
        assertEquals(9.9, Plausible.altitudeMOrNull(9.9)!!, 0.0)
        assertEquals(-0.3, Plausible.altitudeMOrNull(-0.3)!!, 0.0)
        assertEquals(103.17, Plausible.altitudeMOrNull(103.17)!!, 0.0)
        assertEquals(59.181, Plausible.altitudeMOrNull(59.181)!!, 0.0)
        assertEquals(0.0, Plausible.altitudeMOrNull(0.0)!!, 0.0)
    }

    @Test
    fun rejectsTheMeasuredFillerAsAnAltitude() {
        assertNull(Plausible.altitudeMOrNull(djiFiller))
        assertNull(Plausible.altitudeMOrNull(-djiFiller))
    }

    @Test
    fun altitudeBoundsAreInclusiveAndNonFiniteIsRejected() {
        assertNotNull(Plausible.altitudeMOrNull(10_000.0))
        assertNotNull(Plausible.altitudeMOrNull(-10_000.0))
        assertNull(Plausible.altitudeMOrNull(Math.nextUp(10_000.0)))
        assertNull(Plausible.altitudeMOrNull(Math.nextDown(-10_000.0)))
        assertNull(Plausible.altitudeMOrNull(Double.NaN))
        assertNull(Plausible.altitudeMOrNull(Double.POSITIVE_INFINITY))
        assertNull(Plausible.altitudeMOrNull(null))
    }

    // ── battery percentage ────────────────────────────────────────────────────

    /** Every percentage seen in the logs, and both ends of the scale. */
    @Test
    fun acceptsEveryRealBatteryPercentage() {
        for (p in listOf(0, 1, 80, 86, 93, 98, 99, 100)) {
            assertEquals(p, Plausible.batteryPercentOrNull(p))
        }
    }

    /**
     * Out of range it is not a percentage, and `-1` — MAVLink's "not estimated" —
     * is what a null becomes downstream.
     */
    @Test
    fun rejectsPercentagesOutsideTheScale() {
        assertNull(Plausible.batteryPercentOrNull(101))
        assertNull(Plausible.batteryPercentOrNull(-1))
        assertNull(Plausible.batteryPercentOrNull(255))
        assertNull(Plausible.batteryPercentOrNull(Int.MAX_VALUE))
        assertNull(Plausible.batteryPercentOrNull(Int.MIN_VALUE))
        assertNull(Plausible.batteryPercentOrNull(null))
    }

    // ── satellite count ───────────────────────────────────────────────────────

    /** 0 and 31 are the extremes actually logged; 31 was in flight, level-5 fix. */
    @Test
    fun acceptsEveryRealSatelliteCount() {
        for (n in listOf(0, 4, 15, 22, 28, 31, 64)) {
            assertEquals(n, Plausible.satelliteCountOrNull(n))
        }
    }

    /**
     * The one value that could talk `fixType` into a 3D fix it has no evidence
     * for: a count high enough to pass the `MIN_SATS_FOR_3D` floor without being
     * a count at all.
     */
    @Test
    fun rejectsImpossibleSatelliteCounts() {
        assertNull(Plausible.satelliteCountOrNull(65))
        assertNull(Plausible.satelliteCountOrNull(-1))
        assertNull(Plausible.satelliteCountOrNull(Int.MAX_VALUE))
        assertNull(Plausible.satelliteCountOrNull(null))
    }

    /**
     * Rejecting the count must not silently *improve* the fix: `fixType` treats an
     * unknown count as "no cross-check available" and still reports 3D. That is
     * the documented trade — a missing count must not blank the aircraft off the
     * map — so it is asserted here rather than left as a surprise.
     */
    @Test
    fun rejectedSatelliteCountLeavesFixTypeUngated() {
        val withFiller = AircraftState(
            gpsSignalLevel = 5,
            satelliteCount = Plausible.satelliteCountOrNull(Int.MAX_VALUE),
        )
        assertNull(withFiller.satelliteCount)
        assertEquals(
            io.dronefleet.mavlink.common.GpsFixType.GPS_FIX_TYPE_3D_FIX,
            TelemetryEncoder.fixType(withFiller),
        )
    }
}
