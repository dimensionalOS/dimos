package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.telemetry.Geo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.hypot

/**
 * The world→screen half of the situation view: the projection, and the auto-fit that chooses
 * its scale.
 *
 * Everything here is at **38 °N**, this project's home latitude, and never at the equator. That
 * is not decoration: `telemetry/Geo.longitudeScale` documents the one recurring mistake in this
 * codebase — dropping or misplacing the `cos(latitude)` term — as *exactly zero error at the
 * equator and 21 % east error at 38 °N*. A projection suite written at 0° would pass with the
 * term deleted, and the picture would put the aircraft a fifth of the way off in east for every
 * flight this bridge will ever make.
 *
 * Written to fail loudly for:
 *
 *  - **north drawn downward** — the sign on `y`, which is the difference between a north-up map
 *    and a mirror of one, and which looks entirely plausible either way
 *  - **east and north swapped**, which is a 90° rotation that also looks plausible
 *  - **a non-uniform scale**, which turns an orbit into an ellipse and reads as a manoeuvre
 *    nobody commanded
 *  - **an orbit fitted by its centre**, putting the flown circle outside the frame
 *  - **a division by zero** on a single known point, or the absurd zoom that a nearly-zero span
 *    produces without a floor
 *  - `cos(latitude)` dropped from the fit's bounding box, or evaluated per point rather than at
 *    the projection centre (which shears the frame)
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time against the shipped source, run,
 * confirmed red, reverted. Counts are failing tests across `ProjectionTest` and
 * `SituationSceneTest` — measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `y` sign flipped (north drawn downward) | 2 |
 *  | east and north swapped in `toScreen` | 5 |
 *  | `cos(latitude)` dropped from `Geo.nedMetres`' east axis | 6 |
 *  | scale taken from the north axis alone instead of the worse of the two | 1 |
 *  | orbit radius dropped from `Extent` in the fit | 3 |
 *  | `MIN_SPAN_M` floor removed | 8 |
 *  | fit centred on the first extent instead of the bounding-box centre | 2 |
 *  | scale bar rounded to nearest instead of down | 1 |
 *
 * Two of these earned their numbers rather than starting with them. The `y` flip and the scale
 * bar's rounding both **survived the first pass** — the flip because the only assertions on it
 * were in `Projection`'s own coordinate space, and the rounding because the end-to-end budget
 * check happened to land in the lower half of a 1-2-5 step, where round-to-nearest and
 * round-down agree. `north on the earth is up in the scene`, `east on the earth is right in the
 * scene` and `the bar is rounded down, never up past its budget` were added for exactly those
 * two survivors, and are the reason the counts above are 2 and 1 rather than 0 and 0.
 */
class ProjectionTest {

    private val siteLat = 37.9938232
    private val siteLon = 23.7253477
    private val viewport = Viewport(widthPx = 800.0, heightPx = 400.0, padPx = 20.0)

    private fun at(northM: Double, eastM: Double): Pair<Double, Double> =
        Geo.offsetCoordinate(siteLat, siteLon, northM, eastM)

    @Test
    fun `nothing known projects to nothing at all`() {
        assertNull(Projection.fit(emptyList(), viewport))
    }

    @Test
    fun `a viewport with no room in it projects to nothing`() {
        val squeezed = Viewport(widthPx = 30.0, heightPx = 30.0, padPx = 20.0)
        assertNull(Projection.fit(listOf(Extent(siteLat, siteLon)), squeezed))
    }

    @Test
    fun `one known point sits at the centre of the canvas`() {
        val p = Projection.fit(listOf(Extent(siteLat, siteLon)), viewport)!!
        val screen = p.toScreen(siteLat, siteLon)
        assertEquals(400.0, screen.x, 1e-6)
        assertEquals(200.0, screen.y, 1e-6)
    }

    @Test
    fun `one known point is floored at the minimum span rather than dividing by zero`() {
        val p = Projection.fit(listOf(Extent(siteLat, siteLon)), viewport)!!
        assertTrue("scale must be finite and positive", p.metresPerPx.isFinite() && p.metresPerPx > 0.0)
        // The shorter axis (400 px here, 360 usable) shows exactly MIN_SPAN_M.
        val shortAxisM = p.metres(viewport.usableHeightPx)
        assertEquals(Projection.MIN_SPAN_M, shortAxisM, 1e-9)
    }

    @Test
    fun `north is up and east is right`() {
        val p = Projection.fit(
            listOf(Extent(siteLat, siteLon), at(100.0, 100.0).let { Extent(it.first, it.second) }),
            viewport,
        )!!
        val centre = p.toScreen(siteLat, siteLon)
        val (northLat, northLon) = at(50.0, 0.0)
        val (eastLat, eastLon) = at(0.0, 50.0)
        val north = p.toScreen(northLat, northLon)
        val east = p.toScreen(eastLat, eastLon)

        assertTrue("50 m north must be ABOVE the origin on the canvas", north.y < centre.y)
        assertEquals("50 m north must not move x", centre.x, north.x, 1e-6)
        assertTrue("50 m east must be RIGHT of the origin", east.x > centre.x)
        assertEquals("50 m east must not move y", centre.y, east.y, 1e-6)
    }

    @Test
    fun `the scale is the same on both axes, so a circle stays a circle`() {
        // A deliberately lopsided world: 300 m east by 20 m north, in a 2:1 viewport.
        val p = Projection.fit(
            listOf(
                Extent(siteLat, siteLon),
                at(20.0, 300.0).let { Extent(it.first, it.second) },
            ),
            viewport,
        )!!
        val centre = p.toScreen(siteLat, siteLon)
        val (nLat, nLon) = at(60.0, 0.0)
        val (eLat, eLon) = at(0.0, 60.0)
        val northPx = abs(p.toScreen(nLat, nLon).y - centre.y)
        val eastPx = abs(p.toScreen(eLat, eLon).x - centre.x)
        // 1e-3 px, not 0: the *test's* own `offsetCoordinate` evaluates cos(latitude) at the
        // site while the projection evaluates it at the frame centre 10 m away, which is a
        // fifth of a thousandth of a pixel here. A per-axis scale would miss by ~50 px.
        assertEquals("60 m must be the same number of pixels in both directions", northPx, eastPx, 1e-3)
    }

    @Test
    fun `a degree of longitude is shorter than a degree of latitude at 38 degrees north`() {
        // The cos(latitude) tripwire, stated as a picture property rather than as arithmetic:
        // one degree east must project to ~79 % of the pixels one degree north does.
        val p = Projection.fit(
            listOf(
                Extent(siteLat, siteLon),
                Extent(siteLat + 1.0, siteLon + 1.0),
            ),
            Viewport(4000.0, 4000.0, 0.0),
        )!!
        val origin = p.toScreen(siteLat, siteLon)
        val northPx = abs(p.toScreen(siteLat + 1.0, siteLon).y - origin.y)
        val eastPx = abs(p.toScreen(siteLat, siteLon + 1.0).x - origin.x)
        val ratio = eastPx / northPx
        // Evaluated at the projection *centre*, once for the whole frame — that is what keeps
        // the picture from shearing, and it is why this is not `longitudeScale(siteLat)`.
        assertEquals(Geo.longitudeScale(p.centreLatDeg), ratio, 1e-9)
        assertTrue("at 38N the ratio must be near 0.79, not 1.0 — got $ratio", abs(ratio - 1.0) > 0.15)
    }

    @Test
    fun `two points are framed about their midpoint, not about the first of them`() {
        val far = at(0.0, 200.0)
        val p = Projection.fit(
            listOf(Extent(siteLat, siteLon), Extent(far.first, far.second)),
            viewport,
        )!!
        val a = p.toScreen(siteLat, siteLon)
        val b = p.toScreen(far.first, far.second)
        assertEquals("the pair must straddle the canvas centre", 400.0, (a.x + b.x) / 2.0, 1e-6)
        assertEquals(200.0, (a.y + b.y) / 2.0, 1e-6)
    }

    @Test
    fun `every fitted point lands inside the padded canvas`() {
        val corners = listOf(
            Extent(siteLat, siteLon),
            at(150.0, -400.0).let { Extent(it.first, it.second) },
            at(-90.0, 260.0).let { Extent(it.first, it.second) },
        )
        val p = Projection.fit(corners, viewport)!!
        for (e in corners) {
            val s = p.toScreen(e.latDeg, e.lonDeg)
            assertTrue("x=${s.x} outside the pad", s.x >= viewport.padPx - 1e-6 && s.x <= viewport.widthPx - viewport.padPx + 1e-6)
            assertTrue("y=${s.y} outside the pad", s.y >= viewport.padPx - 1e-6 && s.y <= viewport.heightPx - viewport.padPx + 1e-6)
        }
    }

    @Test
    fun `an orbit is framed by its circle, not by its centre`() {
        val orbit = Extent(siteLat, siteLon, radiusM = 120.0)
        val p = Projection.fit(listOf(orbit), viewport)!!
        val centre = p.toScreen(siteLat, siteLon)
        val radiusPx = p.lengthPx(120.0)
        assertTrue(
            "the whole circle must fit the padded height — r=$radiusPx px",
            radiusPx <= viewport.usableHeightPx / 2.0 + 1e-6,
        )
        // And it must genuinely use the room: a centre-only fit would floor at MIN_SPAN_M and
        // leave the circle enormously off-scale.
        assertTrue("the circle must fill most of the frame", radiusPx > viewport.usableHeightPx / 2.0 - 1.0)
        assertEquals(400.0, centre.x, 1e-6)
    }

    @Test
    fun `a point on an orbit circle is exactly a radius from its centre on screen`() {
        val p = Projection.fit(listOf(Extent(siteLat, siteLon, 80.0)), viewport)!!
        val centre = p.toScreen(siteLat, siteLon)
        val radiusPx = p.lengthPx(80.0)
        for (bearing in listOf(0.0, 37.0, 90.0, 180.0, 271.0)) {
            val rad = Math.toRadians(bearing)
            val (lat, lon) = Geo.offsetCoordinate(
                siteLat, siteLon,
                80.0 * kotlin.math.cos(rad), 80.0 * kotlin.math.sin(rad),
            )
            val on = p.toScreen(lat, lon)
            val d = hypot(on.x - centre.x, on.y - centre.y)
            assertEquals("bearing $bearing must land on the drawn circle", radiusPx, d, 0.05)
        }
    }

    @Test
    fun `metres and pixels round-trip`() {
        val p = Projection.fit(listOf(Extent(siteLat, siteLon, 60.0)), viewport)!!
        assertEquals(137.0, p.metres(p.lengthPx(137.0)), 1e-9)
    }

    @Test
    fun `a non-finite extent is ignored rather than poisoning the whole frame`() {
        val p = Projection.fit(
            listOf(
                Extent(siteLat, siteLon),
                Extent(Double.NaN, Double.NaN),
                at(0.0, 100.0).let { Extent(it.first, it.second) },
            ),
            viewport,
        )
        assertNotNull("one bad extent must not lose the picture", p)
        assertTrue(p!!.metresPerPx.isFinite())
    }

    @Test
    fun `every extent being non-finite is nothing to draw`() {
        assertNull(
            Projection.fit(
                listOf(Extent(Double.NaN, 1.0), Extent(2.0, Double.POSITIVE_INFINITY)),
                viewport,
            ),
        )
    }
}
