package com.dimensional.mini4pro.telemetry

import com.dimensional.mini4pro.guided.RepositionGuidance
import com.dimensional.mini4pro.mission.MissionGeo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos

/**
 * The flat-earth geodesy in isolation: `cos(latitude)`, the two axis conventions, and the
 * round trip between them.
 *
 * ## Why this file exists
 *
 * Until 2026-07-27 this arithmetic existed in **three independent copies** plus an inverse —
 * `RepositionGuidance.nedMetres` (the flight-verified original), `MissionGeo.distanceM`,
 * `Geo.enuMetres`, and `RepositionGuidance.offsetCoordinate` — two of them written by agents
 * that could not see the third. [Geo] now owns one implementation and this suite is its guard.
 *
 * The landmine is the **`cos(latitude)` factor on the east component**. Dropping or misplacing
 * it is:
 *
 *  - **exactly zero error at the equator**, so any test written at 0°N passes either way;
 *  - **a 21 % east error at this project's home latitude** (38°N — cos 38° = 0.7880, so
 *    1/0.7880 = 1.269, i.e. an east offset comes out 27 % too long or a computed point lands
 *    21 % short), which at the Q1 100 m cap is 21 m of flight path in the wrong place;
 *  - invisible in any test using a degenerate coordinate (a pure-north offset, or from == to).
 *
 * This project has been bitten by exactly that class of mistake three times, so the term is
 * pinned here by name, at a non-equatorial latitude, in both directions, and the *shape* of the
 * error is asserted rather than just a number — `east < METRES_PER_DEG` would pass for a factor
 * of 0.5 too, so the exact cos is asserted *and* the equatorial value is asserted to be wrong.
 *
 * ## The round trip
 *
 * [Geo.nedMetres] and [Geo.offsetCoordinate] are inverses, and they are inverses **only because
 * they share [Geo.METRES_PER_DEG] and [Geo.longitudeScale]**. That is the entire reason the
 * inverse lives beside the forward conversion instead of in `guided/`. A round-trip test at
 * several latitudes is the cheapest possible guard against the two halves drifting apart, so
 * there is one, closing to well inside a **micrometre** — four orders of magnitude tighter than
 * the centimetre that would already be meaningless to an aircraft.
 *
 * Mutation-checked 2026-07-27 — each breakage made deliberately in `telemetry/Geo.kt`, failing
 * tests counted **across the whole suite**, code reverted after each. Measured, not estimated:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | **`cos(latitude)` dropped from the east component** (`longitudeScale` returns 1.0) | **16** |
 *  | `cos(latitude)` applied to *north* as well as east | 27 |
 *  | `longitudeScale` takes degrees as radians (`Math.toRadians` dropped) | 16 |
 *  | `cos(latitude)` dropped from `offsetCoordinate` only | 11 |
 *  | `nedMetres` returns `(east, north)` | 52 |
 *  | `enuMetres` returns `(north, east)` | 9 |
 *  | ±180° longitude wrap dropped | 3 |
 *  | `METRES_PER_DEG` set to `MissionGeo`'s 111_320.0 | 4 |
 *  | `enuMetres` re-derived with its own `cos` instead of delegating | **0 — see below** |
 *
 * The 16 on the first row spans five suites — `GeoMetresTest`, `RepositionGuidanceTest`,
 * `OrbitGuidanceTest`, `MissionStoreTest`, `ZenohTelemetryEncoderTest` — plus the two end-to-end
 * guided suites, which is what one shared implementation buys: a single dropped term now fails
 * everywhere it would have flown, instead of in whichever copy the author remembered.
 *
 * Two things that mutation **does not** break, and should not: the round-trip tests. Drop the
 * factor from `longitudeScale` and both halves drop it together, so `offsetCoordinate` then
 * `nedMetres` still closes perfectly. That is exactly why the round trip cannot be the only
 * guard and why `oneDegreeOfLongitudeAt38North…` asserts the absolute value — a self-consistent
 * pair of wrong functions is the failure mode a round trip is blind to. Row 4, which breaks only
 * the inverse, is the one the round trip does catch (11 tests, five of them round trips).
 *
 * The final zero is honest and is a statement about the *design*, not a gap in the suite: an
 * `enuMetres` that recomputes the same formula correctly is observationally identical to one
 * that delegates. No test can tell them apart — what makes the delegation load-bearing is that
 * the second copy is where the *next* dropped `cos` would live, and there is no second copy to
 * put it in. [enuMetresIsExactlyNedMetresWithTheAxesSwapped] asserts bit-for-bit equality across
 * five cases including the antimeridian, which is the strongest statement available from outside.
 */
class GeoMetresTest {

    /** The site: Athens, 38°N. cos 38° = 0.7880, which is what makes the landmine visible. */
    private val siteLat = 37.9938232
    private val siteLon = 23.7253477

    // ── the landmine: cos(latitude), by name ─────────────────────────────────────

    /**
     * **The test that must fail if the `cos(latitude)` term is dropped.**
     *
     * A degree of longitude at 38°N is `METRES_PER_DEG * cos(38°)` = 87 631 m, not the
     * 111 195 m a degree of latitude is worth. Dropping the factor is **a 21 % error at this
     * latitude** and *exactly zero* at the equator — which is why this assertion is made here
     * and not at 0°N, and why the equatorial value is asserted to be wrong rather than merely
     * different.
     */
    @Test
    fun `one degree of longitude at 38 north is shortened by cos(latitude)`() {
        val lat = 38.0
        val (north, east) = Geo.nedMetres(lat, 0.0, lat, 1.0)

        // The factor itself, exactly.
        assertEquals(Geo.METRES_PER_DEG * cos(Math.toRadians(lat)), east, 1e-9)

        // And its shape, so a factor of, say, 0.5 could not pass the line above by accident:
        // cos 38° = 0.7880, so the east metre is between 78 % and 79 % of the north one.
        assertTrue("cos(38) must shorten the east degree", east < Geo.METRES_PER_DEG * 0.79)
        assertTrue("...but only by cos(38), not more", east > Geo.METRES_PER_DEG * 0.78)

        // The 21 % headline, stated as an assertion rather than a comment: a conversion that
        // dropped the term would report 111 195 m here, 26.9 % too long, and a point computed
        // from it would land 21.2 % short of where it was asked for.
        val withoutTheFactor = Geo.METRES_PER_DEG
        assertEquals(0.269, withoutTheFactor / east - 1.0, 0.001)
        assertEquals(0.212, 1.0 - east / withoutTheFactor, 0.001)

        // Latitude degrees do NOT shorten — the factor belongs on east and nowhere else.
        assertEquals(0.0, north, 0.0)
    }

    /** The complement: a pure-north offset is untouched by latitude, at every latitude. */
    @Test
    fun `the north component is never scaled by cos(latitude)`() {
        for (lat in listOf(0.0, 38.0, 60.0, 71.0, -33.9)) {
            val (north, east) = Geo.nedMetres(lat, 12.0, lat + 0.001, 12.0)
            assertEquals("north at $lat", 0.001 * Geo.METRES_PER_DEG, north, 1e-9)
            assertEquals("east at $lat", 0.0, east, 0.0)
        }
    }

    /** And the whole reason a test at 0°N proves nothing: there, the factor is exactly 1. */
    @Test
    fun `at the equator the factor is exactly one - which is why nothing is pinned there`() {
        assertEquals(1.0, Geo.longitudeScale(0.0), 0.0)
        val (_, east) = Geo.nedMetres(0.0, 0.0, 0.0, 1.0)
        assertEquals(Geo.METRES_PER_DEG, east, 1e-9)
    }

    /** `longitudeScale` takes **degrees**. Feeding it radians is a 0.03 % error at 38°N... */
    @Test
    fun `longitudeScale takes degrees, not radians`() {
        assertEquals(cos(Math.toRadians(38.0)), Geo.longitudeScale(38.0), 0.0)
        assertEquals(cos(Math.toRadians(60.0)), Geo.longitudeScale(60.0), 0.0)
        // ...and it is even-symmetric, so the southern hemisphere shortens identically.
        assertEquals(Geo.longitudeScale(38.0), Geo.longitudeScale(-38.0), 0.0)
    }

    /** High latitude, where the factor stops being a rounding correction: cos 71° = 0.326. */
    @Test
    fun `the factor bites hardest at high latitude`() {
        val (_, east) = Geo.nedMetres(71.0, 0.0, 71.0, 1.0)
        assertEquals(Geo.METRES_PER_DEG * cos(Math.toRadians(71.0)), east, 1e-9)
        assertTrue("a degree of longitude at 71N is a third of one at the equator", east < 0.33 * Geo.METRES_PER_DEG)
    }

    // ── the two conventions, each by its axis order ──────────────────────────────

    /** NED means `(north, east)` — the order `guided/` and `mission/` read it in. */
    @Test
    fun `nedMetres returns north first, east second`() {
        // 100 m north and 50 m east of the site, built from the constants rather than the code.
        val to = Geo.offsetCoordinate(siteLat, siteLon, 100.0, 50.0)
        val (north, east) = Geo.nedMetres(siteLat, siteLon, to.first, to.second)
        assertEquals(100.0, north, 1e-6)
        assertEquals(50.0, east, 1e-6)
    }

    /** ENU means `(east, north)` — the order the Zenoh bus reads it in. */
    @Test
    fun `enuMetres returns east first, north second`() {
        val to = Geo.offsetCoordinate(siteLat, siteLon, 100.0, 50.0)
        val (east, north) = Geo.enuMetres(siteLat, siteLon, to.first, to.second)
        assertEquals(50.0, east, 1e-6)
        assertEquals(100.0, north, 1e-6)
    }

    /**
     * The strongest available statement that ENU is not a second copy of the formula: for the
     * same input the two functions produce **bit-identical** doubles with the pair swapped.
     * A re-derived `enuMetres` could still pass this; it could not pass it while carrying a
     * different `cos`, a different constant, or a missing antimeridian wrap.
     */
    @Test
    fun enuMetresIsExactlyNedMetresWithTheAxesSwapped() {
        val cases = listOf(
            listOf(siteLat, siteLon, siteLat + 0.01, siteLon + 0.02),
            listOf(0.0, 0.0, 0.001, -0.003),
            listOf(60.5, -122.4, 60.4, -122.6),
            listOf(-33.9, 151.2, -33.7, 151.5),
            listOf(38.0, 179.999, 38.0, -179.999),
        )
        for (c in cases) {
            val (north, east) = Geo.nedMetres(c[0], c[1], c[2], c[3])
            val (e, n) = Geo.enuMetres(c[0], c[1], c[2], c[3])
            assertEquals("east, exactly, for $c", east, e, 0.0)
            assertEquals("north, exactly, for $c", north, n, 0.0)
        }
    }

    /** A target across the antimeridian is a short hop, not a lap of the planet. */
    @Test
    fun `the longitude delta is normalised into plus or minus 180`() {
        val (_, east) = Geo.nedMetres(38.0, 179.9995, 38.0, -179.9995)
        assertEquals(0.001 * Geo.METRES_PER_DEG * cos(Math.toRadians(38.0)), east, 1e-6)
        // ...and the other way round.
        val (_, west) = Geo.nedMetres(38.0, -179.9995, 38.0, 179.9995)
        assertEquals(-0.001 * Geo.METRES_PER_DEG * cos(Math.toRadians(38.0)), west, 1e-6)
    }

    // ── the round trip: forward, then inverse ───────────────────────────────────

    /**
     * **The guard against the two halves drifting apart.**
     *
     * `offsetCoordinate` then `nedMetres` must return the offset it was given, at every
     * latitude and both signs of longitude. It closes to a micrometre — not because that
     * precision means anything to an aircraft, but because anything *looser* would still pass
     * with a subtly different constant in one half, and the point is to catch that.
     *
     * 0°N is in the list deliberately, as the case that proves nothing on its own; 38°N is the
     * site; 60°N and 71°N are where the factor stops being a correction and starts being a
     * multiplier.
     */
    @Test
    fun `forward then inverse returns the original offset at every latitude`() {
        val latitudes = listOf(0.0, 38.0, 37.9938232, 60.0, 71.0, -33.9, -60.0)
        val longitudes = listOf(23.7253477, -122.4194, 0.0, -0.1276, 151.2093)
        val offsets = listOf(
            0.0 to 0.0,
            100.0 to 0.0,
            0.0 to 100.0,
            -100.0 to -100.0,
            2500.0 to -1750.0,
            0.01 to -0.01,
        )
        for (lat in latitudes) {
            for (lon in longitudes) {
                for ((n, e) in offsets) {
                    val (toLat, toLon) = Geo.offsetCoordinate(lat, lon, n, e)
                    val (backNorth, backEast) = Geo.nedMetres(lat, lon, toLat, toLon)
                    val where = "at ($lat, $lon) offset ($n, $e)"
                    assertTrue("north round trip $where", abs(backNorth - n) < 1e-6)
                    assertTrue("east round trip $where", abs(backEast - e) < 1e-6)
                }
            }
        }
    }

    /**
     * And the same loop the other way: coordinate → metres → coordinate must land back on the
     * same place, measured in metres rather than degrees so "within a centimetre" means what it
     * says at every latitude (a degree of longitude at 71°N is a third of one at the equator,
     * so a degrees-based tolerance would be three times looser there).
     */
    @Test
    fun `inverse then forward returns the original coordinate, well inside a centimetre`() {
        val cases = listOf(
            listOf(38.0, 23.7, 38.0009, 23.7011),
            listOf(0.0, -0.0005, 0.0009, 0.0011),
            listOf(60.0, -122.4194, 60.0009, -122.4183),
            listOf(71.0, 25.78, 70.9991, 25.7823),
            listOf(-33.8688, 151.2093, -33.8679, 151.2104),
        )
        for (c in cases) {
            val (north, east) = Geo.nedMetres(c[0], c[1], c[2], c[3])
            val (backLat, backLon) = Geo.offsetCoordinate(c[0], c[1], north, east)
            val (dNorth, dEast) = Geo.nedMetres(c[2], c[3], backLat, backLon)
            assertTrue("north residual for $c: $dNorth m", abs(dNorth) < 1e-6)
            assertTrue("east residual for $c: $dEast m", abs(dEast) < 1e-6)
        }
    }

    /**
     * The inverse pinned on its own, so a dropped factor *there* cannot hide behind a matching
     * drop in the forward direction. 1000 m east of 38°N is 0.01141°, not the 0.00899° an
     * unscaled conversion would give — the same 21 % error, seen from the other side.
     */
    @Test
    fun `offsetCoordinate divides the east metres by cos(latitude)`() {
        val (lat, lon) = Geo.offsetCoordinate(38.0, 10.0, 0.0, 1000.0)
        assertEquals(38.0, lat, 0.0)
        assertEquals(10.0 + 1000.0 / (Geo.METRES_PER_DEG * cos(Math.toRadians(38.0))), lon, 1e-12)
        // Without the factor it would be this, and it is 21 % short of the truth.
        val unscaled = 10.0 + 1000.0 / Geo.METRES_PER_DEG
        assertTrue("the scaled longitude must reach further east", lon > unscaled)
        assertEquals(0.212, 1.0 - (unscaled - 10.0) / (lon - 10.0), 0.001)
    }

    // ── the callers: one implementation, no re-copies ────────────────────────────

    /**
     * `RepositionGuidance` keeps the names (they are cited by `docs/m3-stage-c.md`,
     * `docs/m4-mission-execution.md` and four suites) but must not keep a body. If someone
     * re-inlines the arithmetic there, the copy is back and this fails.
     */
    @Test
    fun `RepositionGuidance delegates rather than restating the geodesy`() {
        assertEquals(Geo.METRES_PER_DEG, RepositionGuidance.METRES_PER_DEG, 0.0)
        val a = Geo.nedMetres(siteLat, siteLon, siteLat + 0.003, siteLon - 0.004)
        val b = RepositionGuidance.nedMetres(siteLat, siteLon, siteLat + 0.003, siteLon - 0.004)
        assertEquals(a.first, b.first, 0.0)
        assertEquals(a.second, b.second, 0.0)

        val c = Geo.offsetCoordinate(siteLat, siteLon, 40.0, -25.0)
        val d = RepositionGuidance.offsetCoordinate(siteLat, siteLon, 40.0, -25.0)
        assertEquals(c.first, d.first, 0.0)
        assertEquals(c.second, d.second, 0.0)
    }

    /**
     * **The two layers now measure in the same units — asserted so they cannot drift apart again.**
     *
     * They did drift, and this test is the scar. The mission layer used 111 320.0 while the flight
     * law used 111 194.93, and `MissionGeo.METRES_PER_DEG`'s KDoc claimed they were "the same
     * spherical constant". A divergence describing itself as a duplicate is how a 0.11 %
     * difference in an admission gate survives review.
     *
     * Unified 2026-07-27 on Ivan's ruling, onto **111 194.93** — the mean-radius value, which sits
     * between the two quantities one constant has to serve (a latitude degree at 38°N is ≈ 110 996
     * m, the longitude coefficient ≈ 111 320 m), and which is the number every goto and every
     * orbit has actually flown with. The gate became 0.11 % more permissive as a result: eleven
     * centimetres on a hundred metres, accepted deliberately, because a gate that measures in
     * different units from the flight it is gating is the worse property.
     *
     * | mutation | tests that fail | measured |
     * |---|---|---|
     * | `MissionGeo.METRES_PER_DEG` set back to 111 320.0 | this test | 1 |
     */
    @Test
    fun `the mission layer and the flight law measure in the same units`() {
        assertEquals(
            "MissionGeo.METRES_PER_DEG must remain an alias for Geo.METRES_PER_DEG — they were " +
                "different once, by 0.11 %, and the KDoc claimed they were not",
            Geo.METRES_PER_DEG,
            MissionGeo.METRES_PER_DEG,
            0.0,
        )
        assertEquals(111_194.93, Geo.METRES_PER_DEG, 0.0)

        // The east ruler is that constant times the one shared cos factor, evaluated at the mean
        // latitude of the two points — a pure east-west leg makes the mean equal to both.
        val from = com.dimensional.mini4pro.mission.GeoPoint(38.0, 10.0)
        val to = com.dimensional.mini4pro.mission.GeoPoint(38.0, 11.0)
        assertEquals(
            MissionGeo.METRES_PER_DEG * Geo.longitudeScale(38.0),
            MissionGeo.distanceM(from, to),
            1e-6,
        )
    }
}
