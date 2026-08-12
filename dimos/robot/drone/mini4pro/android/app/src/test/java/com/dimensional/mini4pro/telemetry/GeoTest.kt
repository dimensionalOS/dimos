package com.dimensional.mini4pro.telemetry

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The rule that decides whether a pair of doubles is a place.
 *
 * This exists because of a measured failure, not a hypothetical one: on
 * 2026-07-26 11:17 the aircraft was powered, linked and had never flown, DJI's
 * `KeyHomeLocation` returned a **non-null** coordinate whose latitude and
 * longitude were both `4.583662361046586E7`, and the bridge published 220
 * `HOME_POSITION` messages off the back of it
 * (docs/measurements/2026-07-26-home-position-sentinel.md).
 *
 * So the boundaries are asserted by hand rather than derived: the exact observed
 * filler, the exact ±90/±180 edges on both sides, and the one-number-in-both-fields
 * shape that a range check alone would let through.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | latitude range check dropped | 6 |
 *  | longitude range check dropped | 5 |
 *  | `lat == lon` check dropped | 2 |
 *  | bounds made exclusive (`<=` / `>=` on the edges) | 2 |
 *  | latitude bound widened to 180 | 4 |
 *  | non-finite check dropped | 2 |
 *  | pair returned as (lon, lat) | 7 |
 *  | longitude falls back to latitude when null | **0 — see below** |
 *
 * The last one **survived, and correctly**: `longitude ?: lat` makes the two
 * coordinates equal, which rule 3 then rejects, so the mutant returns null exactly
 * where the original does. It is recorded rather than deleted because a surviving
 * mutation that turns out to be an equivalent program is a finding about the code,
 * not a gap in the suite — the `lat == lon` rule is load-bearing enough to absorb
 * a null-handling bug.
 */
class GeoTest {

    /**
     * What `KeyHomeLocation` actually returned with no home point recorded, in
     * both fields, at 1 Hz, for the whole session.
     */
    private val djiNoHomeFiller = 4.583662361046586E7

    /** The real site, from the same aircraft once it had a fix. */
    private val athensLat = 37.9938232
    private val athensLon = 23.7253477

    @Test
    fun rejectsTheMeasuredDjiNoHomeFiller() {
        assertNull(Geo.coordinateOrNull(djiNoHomeFiller, djiNoHomeFiller))
        assertFalse(Geo.isValid(djiNoHomeFiller, djiNoHomeFiller))
    }

    @Test
    fun rejectsTheFillerEvenBesideARealCoordinate() {
        // All-or-nothing: half a position is still something QGC would plot.
        assertNull(Geo.coordinateOrNull(djiNoHomeFiller, athensLon))
        assertNull(Geo.coordinateOrNull(athensLat, djiNoHomeFiller))
    }

    @Test
    fun acceptsARealFixAndKeepsTheOrder() {
        val c = Geo.coordinateOrNull(athensLat, athensLon)!!
        assertEquals(athensLat, c.first, 0.0)
        assertEquals(athensLon, c.second, 0.0)
    }

    @Test
    fun rejectsEitherHalfMissing() {
        assertNull(Geo.coordinateOrNull(null, athensLon))
        assertNull(Geo.coordinateOrNull(athensLat, null))
        assertNull(Geo.coordinateOrNull(null, null))
    }

    @Test
    fun rejectsNonFinite() {
        assertNull(Geo.coordinateOrNull(Double.NaN, athensLon))
        assertNull(Geo.coordinateOrNull(athensLat, Double.NaN))
        assertNull(Geo.coordinateOrNull(Double.POSITIVE_INFINITY, athensLon))
        assertNull(Geo.coordinateOrNull(athensLat, Double.NEGATIVE_INFINITY))
    }

    /** The poles and the antimeridian are real places, so the bounds are inclusive. */
    @Test
    fun acceptsTheExactBounds() {
        assertTrue(Geo.isValid(90.0, 0.5))
        assertTrue(Geo.isValid(-90.0, 0.5))
        assertTrue(Geo.isValid(0.5, 180.0))
        assertTrue(Geo.isValid(0.5, -180.0))
        // Both extremes at once, still a place.
        assertTrue(Geo.isValid(90.0, 180.0))
        assertTrue(Geo.isValid(-90.0, -180.0))
    }

    @Test
    fun rejectsOneUlpOutsideTheBounds() {
        assertNull(Geo.coordinateOrNull(Math.nextUp(90.0), 0.5))
        assertNull(Geo.coordinateOrNull(Math.nextDown(-90.0), 0.5))
        assertNull(Geo.coordinateOrNull(0.5, Math.nextUp(180.0)))
        assertNull(Geo.coordinateOrNull(0.5, Math.nextDown(-180.0)))
    }

    /**
     * A latitude of 100 is inside the longitude range and outside its own. The
     * two bounds are separate on purpose; conflating them lets half the filler
     * values through.
     */
    @Test
    fun latitudeIsBoundedTighterThanLongitude() {
        assertTrue(Geo.isValid(0.5, 100.0))
        assertNull(Geo.coordinateOrNull(100.0, 0.5))
    }

    /**
     * The second gate: an in-range filler still has the shape of one number
     * written into both fields, and no real fix is bit-identical across axes.
     */
    @Test
    fun rejectsAnInRangeValueRepeatedInBothFields() {
        assertNull(Geo.coordinateOrNull(45.0, 45.0))
        assertNull(Geo.coordinateOrNull(-12.3456789, -12.3456789))
        // Including the classic: 0/0 is the Gulf of Guinea and also every
        // uninitialised struct ever written.
        assertNull(Geo.coordinateOrNull(0.0, 0.0))
    }

    /** ...but only bit-identical. One ulp apart is two independent numbers. */
    @Test
    fun acceptsNearlyEqualCoordinates() {
        assertTrue(Geo.isValid(45.0, Math.nextUp(45.0)))
        assertTrue(Geo.isValid(0.0, Math.nextUp(0.0)))
        assertTrue(Geo.isValid(0.0, 0.0000001))
    }
}
