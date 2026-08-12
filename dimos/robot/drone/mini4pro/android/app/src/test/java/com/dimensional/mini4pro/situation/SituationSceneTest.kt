package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.telemetry.Geo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.hypot

/**
 * [Situation] → [Scene]: what is actually drawn, in pixels, before any `Canvas` exists.
 *
 * This suite is the reason the `View` can be dumb. Every decision with an opinion in it — the
 * aircraft symbol's shape and orientation, which way the orbit chevron points, which lines are
 * dashed, what the scale bar says, what is drawn *over* what — is settled here on the JVM, so
 * the Android class that paints it has no branch left to get wrong and no test that would need
 * a phone.
 *
 * Written to fail loudly for:
 *
 *  - **an aircraft dart pointing the wrong way**, which is wrong by a reflection or a rotation
 *    and looks entirely plausible in a screenshot
 *  - **an orbit chevron on the wrong side**, showing a clockwise circle flown anticlockwise
 *  - **a goto line drawn from nowhere** when the aircraft symbol was withheld — a line whose
 *    start is a position we just refused to draw
 *  - **an ROI line drawn for a suspended ROI**, claiming a camera that is not being driven
 *  - **the aircraft painted under something else**, so the thing being looked for is hidden
 *  - **a scale bar longer than its budget**, overhanging the picture it is measuring
 *
 * Mutation counts are recorded in `SituationReadingTest` and `ProjectionTest`, which is where
 * the mutations were applied; several of them are killed here.
 */
class SituationSceneTest {

    private val lat = 37.9938232
    private val lon = 23.7253477
    private val viewport = Viewport(900.0, 500.0, 24.0)

    private fun at(northM: Double, eastM: Double): Fix =
        Geo.offsetCoordinate(lat, lon, northM, eastM).let { Fix(it.first, it.second) }

    private fun situation(
        aircraft: AircraftMark? = AircraftMark(Fix(lat, lon), 0.0),
        orbit: OrbitMark? = null,
        goto: GotoMark? = null,
        roi: RoiMark? = null,
        plan: PlanMark? = null,
        home: Fix? = null,
        source: SituationSource = SituationSource.LIVE,
    ) = Situation(source, aircraft, home, orbit, goto, roi, plan)

    private inline fun <reified T : Shape> Scene.of(ink: Ink? = null): List<T> =
        shapes.filterIsInstance<T>().filter { ink == null || it.ink == ink }

    // ── nothing known ────────────────────────────────────────────────────────

    @Test
    fun `an empty situation paints nothing at all`() {
        val scene = SituationScene.build(Situation.UNKNOWN, viewport)
        assertTrue("an honest blank, not a map of somewhere", scene.isEmpty)
        assertNull(scene.scaleBarMetres)
    }

    @Test
    fun `the notes survive an empty picture, so a blank frame is explained`() {
        val s = Situation(SituationSource.LIVE, notes = listOf(SituationReading.NOTE_NO_FIX))
        val scene = SituationScene.build(s, viewport)
        assertTrue(scene.isEmpty)
        assertEquals(listOf(SituationReading.NOTE_NO_FIX), scene.notes)
    }

    @Test
    fun `the source reaches the scene, because the View must not be told to guess`() {
        val scene = SituationScene.build(situation(source = SituationSource.REPLAY), viewport)
        assertEquals(SituationSource.REPLAY, scene.source)
    }

    /**
     * **So does the publishing flag** — on both exits, including the empty one.
     *
     * `SituationView` picks the whole-picture watermark from it: a recording that has left the
     * phone for a ground station or a bus says so across the canvas, and a recording that is only
     * being looked at does not. Dropping it here would be silent — the picture is identical, the
     * source is still `REPLAY`, and the only thing lost is the one indicator an operator sees
     * without looking for it.
     *
     * **The empty branch is not decoration.** A recording being published *starts* with no fix at
     * all — the cursor is at a sample before takeoff — so the first seconds of every published
     * replay take the early return, and that is exactly when an operator is deciding whether they
     * meant to do this.
     */
    @Test
    fun `the publishing flag reaches the scene on both exits`() {
        val published = situation(source = SituationSource.REPLAY).copy(publishing = true)
        assertTrue(SituationScene.build(published, viewport).publishing)
        assertFalse(SituationScene.build(situation(source = SituationSource.REPLAY), viewport).publishing)

        // …and on the early return, where there is nothing on the earth to draw.
        val nothingKnown = Situation(SituationSource.REPLAY, publishing = true)
        assertTrue(SituationScene.build(nothingKnown, viewport).isEmpty)
        assertTrue(SituationScene.build(nothingKnown, viewport).publishing)
    }

    // ── the aircraft symbol ──────────────────────────────────────────────────

    @Test
    fun `a known heading draws a dart, and its nose points that way`() {
        // All four cardinals: a swapped sin/cos or a flipped y survives any single one of them.
        val cases = mapOf(
            0.0 to { nose: ScreenPoint, c: ScreenPoint -> nose.y < c.y && abs(nose.x - c.x) < 1e-6 },
            90.0 to { nose: ScreenPoint, c: ScreenPoint -> nose.x > c.x && abs(nose.y - c.y) < 1e-6 },
            180.0 to { nose: ScreenPoint, c: ScreenPoint -> nose.y > c.y && abs(nose.x - c.x) < 1e-6 },
            270.0 to { nose: ScreenPoint, c: ScreenPoint -> nose.x < c.x && abs(nose.y - c.y) < 1e-6 },
        )
        for ((heading, holds) in cases) {
            val centre = ScreenPoint(100.0, 100.0)
            val nose = SituationScene.triangle(centre, heading, SituationScene.AIRCRAFT_PX).first()
            assertTrue("heading $heading points the wrong way: $nose", holds(nose, centre))
        }
    }

    @Test
    fun `the dart is a triangle whose rear corners straddle the nose`() {
        val centre = ScreenPoint(0.0, 0.0)
        val t = SituationScene.triangle(centre, 0.0, 20.0)
        assertEquals(3, t.size)
        // Nose ahead, both rear corners behind, one either side.
        assertTrue(t[0].y < 0.0)
        assertTrue(t[1].y > 0.0 && t[2].y > 0.0)
        assertTrue("the corners must be on opposite sides", t[1].x * t[2].x < 0.0)
    }

    @Test
    fun `an unknown heading draws a dot with no direction to read off it`() {
        val scene = SituationScene.build(
            situation(aircraft = AircraftMark(Fix(lat, lon), null)),
            viewport,
        )
        assertTrue("no dart may be drawn", scene.of<Shape.Polygon>(Ink.AIRCRAFT).isEmpty())
        assertEquals(1, scene.of<Shape.Dot>(Ink.AIRCRAFT).size)
    }

    @Test
    fun `the aircraft is painted last, so nothing is ever drawn over it`() {
        val scene = SituationScene.build(
            situation(
                orbit = OrbitMark(at(0.0, 0.0), 50.0, 1),
                goto = GotoMark(at(30.0, 30.0), arrived = false),
                roi = RoiMark(at(-20.0, 40.0), tracking = true),
                plan = PlanMark(listOf(PlanPoint(1, at(10.0, -30.0), "WAYPOINT"))),
                home = Fix(lat, lon),
            ),
            viewport,
        )
        val last = scene.shapes.last()
        assertEquals(Ink.AIRCRAFT, last.ink)
    }

    // ── the orbit ────────────────────────────────────────────────────────────

    @Test
    fun `an orbit draws its circle, its centre and a direction chevron`() {
        val scene = SituationScene.build(
            situation(orbit = OrbitMark(Fix(lat, lon), 60.0, 1)),
            viewport,
        )
        val circle = scene.of<Shape.Circle>(Ink.ORBIT).single()
        assertTrue(circle.radiusPx > 0.0)
        assertEquals(1, scene.of<Shape.Dot>(Ink.ORBIT).size)
        assertEquals(1, scene.of<Shape.Polygon>(Ink.ORBIT).size)
    }

    @Test
    fun `the chevron points east at the north of a clockwise circle and west anticlockwise`() {
        val centre = ScreenPoint(200.0, 200.0)
        val clockwise = SituationScene.chevron(centre, 80.0, 1).points
        val anti = SituationScene.chevron(centre, 80.0, -1).points
        // Both sit at the circle's north point.
        assertTrue("chevron belongs at the top of the circle", clockwise.all { it.y < centre.y })
        // The tip leads the arms: clockwise goes east (+x), anticlockwise west (−x).
        assertTrue("clockwise must lead east — got ${clockwise[0]}", clockwise[0].x > centre.x)
        assertTrue("anticlockwise must lead west — got ${anti[0]}", anti[0].x < centre.x)
    }

    @Test
    fun `a zero direction is read as clockwise rather than as no direction`() {
        // `OrbitCommand` normalises the sign at acceptance and only ±1 reaches here; a 0 must
        // still produce a drawn direction rather than a degenerate shape.
        val c = SituationScene.chevron(ScreenPoint(0.0, 0.0), 50.0, 0)
        assertEquals(3, c.points.size)
        assertTrue(c.points[0].x > 0.0)
    }

    @Test
    fun `the drawn circle passes through the points the aircraft would fly`() {
        val orbit = OrbitMark(Fix(lat, lon), 70.0, 1)
        val scene = SituationScene.build(situation(aircraft = null, orbit = orbit), viewport)
        val circle = scene.of<Shape.Circle>(Ink.ORBIT).single()
        val projection = Projection.fit(SituationScene.extents(situation(null, orbit)), viewport)!!
        val onCircle = Geo.offsetCoordinate(lat, lon, 0.0, 70.0)
        val p = projection.toScreen(onCircle.first, onCircle.second)
        assertEquals(
            circle.radiusPx,
            hypot(p.x - circle.centre.x, p.y - circle.centre.y),
            0.05,
        )
    }

    // ── the goto ─────────────────────────────────────────────────────────────

    @Test
    fun `a goto draws its target and the leg to it`() {
        val scene = SituationScene.build(
            situation(goto = GotoMark(at(80.0, 0.0), arrived = false)),
            viewport,
        )
        val leg = scene.of<Shape.Path>(Ink.GOTO).single()
        assertEquals(2, leg.points.size)
        assertFalse("still going somewhere — a solid leg", leg.dashed)
        assertEquals(1, scene.of<Shape.Dot>(Ink.GOTO).size)
    }

    @Test
    fun `an arrived goto is a place we are at, not a place we are heading for`() {
        val scene = SituationScene.build(
            situation(goto = GotoMark(at(80.0, 0.0), arrived = true)),
            viewport,
        )
        assertTrue(scene.of<Shape.Path>(Ink.GOTO).single().dashed)
    }

    @Test
    fun `with no aircraft symbol the goto keeps its target and loses its leg`() {
        val scene = SituationScene.build(
            situation(aircraft = null, goto = GotoMark(at(80.0, 0.0), arrived = false)),
            viewport,
        )
        assertTrue(
            "a line from a position we refused to draw is a claim about that position",
            scene.of<Shape.Path>(Ink.GOTO).isEmpty(),
        )
        assertEquals(1, scene.of<Shape.Dot>(Ink.GOTO).size)
    }

    // ── the region of interest ───────────────────────────────────────────────

    @Test
    fun `a tracked ROI draws the camera's line of sight`() {
        val scene = SituationScene.build(
            situation(roi = RoiMark(at(0.0, 90.0), tracking = true)),
            viewport,
        )
        assertEquals(1, scene.of<Shape.Path>(Ink.ROI).size)
        assertEquals(1, scene.of<Shape.Dot>(Ink.ROI).size)
    }

    @Test
    fun `a suspended ROI keeps its point and loses its line`() {
        val scene = SituationScene.build(
            situation(roi = RoiMark(at(0.0, 90.0), tracking = false)),
            viewport,
        )
        assertTrue("the camera is not on it", scene.of<Shape.Path>(Ink.ROI).isEmpty())
        assertTrue("no ghost line either", scene.of<Shape.Path>(Ink.GHOST).isEmpty())
        assertEquals("but the place asked about is still shown", 1, scene.of<Shape.Dot>(Ink.GHOST).size)
    }

    // ── the plan ─────────────────────────────────────────────────────────────

    @Test
    fun `a loaded plan is drawn broken, because nothing is flying it`() {
        val plan = PlanMark(
            listOf(
                PlanPoint(1, at(0.0, 0.0), "WAYPOINT"),
                PlanPoint(2, at(60.0, 0.0), "WAYPOINT"),
                PlanPoint(3, at(60.0, 60.0), "WAYPOINT"),
            ),
        )
        val scene = SituationScene.build(situation(plan = plan), viewport)
        val legs = scene.of<Shape.Path>(Ink.PLAN).single()
        assertEquals(3, legs.points.size)
        assertTrue("a solid line reads as a path being flown", legs.dashed)
        assertEquals(3, scene.of<Shape.Dot>(Ink.PLAN).size)
        assertEquals(
            listOf("1", "2", "3"),
            scene.of<Shape.Label>(Ink.PLAN).map { it.text },
        )
    }

    @Test
    fun `a single-waypoint plan is a point, not a line of length zero`() {
        val scene = SituationScene.build(
            situation(plan = PlanMark(listOf(PlanPoint(4, at(0.0, 0.0), "WAYPOINT")))),
            viewport,
        )
        assertTrue(scene.of<Shape.Path>(Ink.PLAN).isEmpty())
        assertEquals(1, scene.of<Shape.Dot>(Ink.PLAN).size)
    }

    @Test
    fun `a current leg would be distinguishable if anything ever knew of one`() {
        // Unreachable from `SituationReading.planMarkOf` today by construction — the capability
        // is pinned so the day an executor exists the picture is already able to say so.
        val plan = PlanMark(
            points = listOf(PlanPoint(1, at(0.0, 0.0), "WAYPOINT"), PlanPoint(2, at(50.0, 0.0), "WAYPOINT")),
            currentSeq = 2,
            flying = true,
        )
        val scene = SituationScene.build(situation(plan = plan), viewport)
        assertEquals(1, scene.of<Shape.Dot>(Ink.PLAN_CURRENT).size)
        assertFalse("a flown plan is drawn solid", scene.of<Shape.Path>(Ink.PLAN).single().dashed)
    }

    // ── home ─────────────────────────────────────────────────────────────────

    @Test
    fun `home is drawn when it is in frame`() {
        val scene = SituationScene.build(situation(home = Fix(lat, lon)), viewport)
        assertEquals(1, scene.of<Shape.Dot>(Ink.HOME).size)
        assertEquals("H", scene.of<Shape.Label>(Ink.HOME).single().text)
    }

    @Test
    fun `a distant home is left off rather than shrinking everything else to a smudge`() {
        // Home 3 km away while an orbit is being flown: fitting it would compress the circle to
        // a few pixels, which loses the picture to gain a marker.
        val far = Geo.offsetCoordinate(lat, lon, 3_000.0, 0.0)
        val scene = SituationScene.build(
            situation(orbit = OrbitMark(Fix(lat, lon), 40.0, 1), home = Fix(far.first, far.second)),
            viewport,
        )
        assertTrue(scene.of<Shape.Dot>(Ink.HOME).isEmpty())
        // The orbit kept its scale.
        assertTrue(scene.of<Shape.Circle>(Ink.ORBIT).single().radiusPx > 50.0)
    }

    @Test
    fun `home alone is enough to draw a picture`() {
        val scene = SituationScene.build(
            situation(aircraft = null, home = Fix(lat, lon)),
            viewport,
        )
        // Nothing else is known, so home *is* the frame and lands in the middle of it.
        assertEquals(1, scene.of<Shape.Dot>(Ink.HOME).size)
    }

    // ── the scale bar ────────────────────────────────────────────────────────

    @Test
    fun `the scale bar is a round number that fits its budget`() {
        val scene = SituationScene.build(
            situation(orbit = OrbitMark(Fix(lat, lon), 60.0, 1)),
            viewport,
        )
        val metres = scene.scaleBarMetres!!
        val px = scene.scaleBarPx!!
        assertTrue("$metres is not on the 1-2-5 ladder", metres in SituationScene.SCALE_STEPS_M)
        assertTrue(
            "the bar must not overhang its budget: ${px}px of ${viewport.widthPx * SituationScene.SCALE_BAR_FRACTION}",
            px <= viewport.widthPx * SituationScene.SCALE_BAR_FRACTION + 1e-9,
        )
    }

    @Test
    fun `the bar is rounded down, never up past its budget`() {
        // Pinned directly rather than only through `build`, because "round to nearest" survives
        // an end-to-end assertion whenever the budget happens to land in the lower half of a
        // step. Here it does not: 90 m of budget must produce the 50 m bar, not the 100 m one.
        val projection = Projection.fit(
            listOf(Extent(lat, lon, radiusM = 200.0)),
            Viewport(1000.0, 1000.0, 0.0),
        )!!
        val budgetM = projection.metres(1000.0 * SituationScene.SCALE_BAR_FRACTION)
        val (metres, px) = SituationScene.scaleBar(projection, Viewport(1000.0, 1000.0, 0.0))
        assertTrue("test setup: budget $budgetM should straddle a 1-2-5 step", budgetM > 50.0)
        assertTrue(
            "a bar of ${metres}m for a budget of ${budgetM}m has been rounded up",
            metres!! <= budgetM,
        )
        assertTrue(px!! <= 1000.0 * SituationScene.SCALE_BAR_FRACTION)
    }

    @Test
    fun `north on the earth is up in the scene, not merely up in the projection`() {
        // The y-sign tripwire asserted through `build`, where an operator would see it: a goto
        // 100 m north of the aircraft must be drawn above the aircraft symbol.
        val scene = SituationScene.build(
            situation(
                aircraft = AircraftMark(Fix(lat, lon), 0.0),
                goto = GotoMark(at(100.0, 0.0), arrived = false),
            ),
            viewport,
        )
        val leg = scene.of<Shape.Path>(Ink.GOTO).single()
        val from = leg.points.first()
        val to = leg.points.last()
        assertTrue("the target is north, so it must be higher on screen", to.y < from.y)
        assertEquals("and directly above, not off to one side", from.x, to.x, 1e-6)
    }

    @Test
    fun `east on the earth is right in the scene`() {
        val scene = SituationScene.build(
            situation(
                aircraft = AircraftMark(Fix(lat, lon), 0.0),
                goto = GotoMark(at(0.0, 100.0), arrived = false),
            ),
            viewport,
        )
        val leg = scene.of<Shape.Path>(Ink.GOTO).single()
        assertTrue(leg.points.last().x > leg.points.first().x)
        assertEquals(leg.points.first().y, leg.points.last().y, 1e-6)
    }

    @Test
    fun `the scale bar tracks the scale it is measuring`() {
        val small = SituationScene.build(situation(orbit = OrbitMark(Fix(lat, lon), 25.0, 1)), viewport)
        val big = SituationScene.build(situation(orbit = OrbitMark(Fix(lat, lon), 400.0, 1)), viewport)
        assertNotNull(small.scaleBarMetres)
        assertTrue(
            "a 400 m circle must be labelled larger than a 25 m one",
            big.scaleBarMetres!! > small.scaleBarMetres!!,
        )
    }
}
