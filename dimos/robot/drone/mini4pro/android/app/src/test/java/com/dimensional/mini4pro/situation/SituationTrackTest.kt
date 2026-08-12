package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.telemetry.Geo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.hypot

/**
 * **The flown track as a picture**: what it is drawn behind, how a gap looks, how the old part
 * recedes, and — the decision worth arguing about — what the frame is fitted to.
 *
 * The accumulation rules are next door in `FlownTrackTest`; this file only cares what
 * [SituationScene] does with a finished [TrackMark]. Everything here is pixels on a `Viewport`
 * with no `Canvas` anywhere, which is the whole reason the framing decision can be *measured*
 * rather than looked at on a phone.
 *
 * ## The framing decision, measured
 *
 * A track is the first thing in this picture that grows without bound, and fitting it naively
 * fails quietly: nothing disappears, everything simply becomes too small to read. With a 30 m
 * orbit under the aircraft and a 500 m approach behind it, on a 1000×600 viewport:
 *
 *  | fitted to | orbit across the short axis |
 *  |---|---|
 *  | the whole track | **11.3 %** |
 *  | the recent 120 m ([SituationScene.TRACK_RECENT_M]) | **50.0 %** |
 *
 * The picture exists to show what the aircraft is doing now. Both framings are *honest* — this
 * is not a truth question — but one of them has answered "where has it been" by making "what is
 * it doing" unreadable, and the operator can only ask the first question about a flight they can
 * still see. The older track is drawn either way; it just gets no vote on the scale.
 */
class SituationTrackTest {

    private val siteLat = 37.9938232
    private val siteLon = 23.7253477

    private fun at(northM: Double, eastM: Double): Fix {
        val (lat, lon) = Geo.offsetCoordinate(siteLat, siteLon, northM, eastM)
        return Fix(lat, lon)
    }

    private val viewport = Viewport(1000.0, 600.0, 20.0)

    /** A straight track of [metres] running north, one point every [FlownTrack.STEP_M]. */
    private fun straightTrack(metres: Double, fromNorth: Double = 0.0): TrackMark {
        val points = ArrayList<Fix>()
        var north = fromNorth
        while (north <= fromNorth + metres) {
            points.add(at(north, 0.0))
            north += FlownTrack.STEP_M
        }
        return mark(listOf(TrackSegment(points)))
    }

    /**
     * A [TrackMark] with its recent window computed the way [FlownTrack] would for a track flown
     * fast enough that the time window is not the binding one — i.e. by walking
     * [FlownTrack.RECENT_M] back along the newest segment.
     *
     * These tests construct tracks by hand, with no clock, because what they are about is paint
     * order and framing rather than the window rule (which `FlownTrackTest` owns). Computing the
     * count here rather than passing a literal keeps them honest: a change to the floor moves
     * these fixtures with it instead of silently leaving them asserting an old shape.
     */
    private fun mark(segments: List<TrackSegment>): TrackMark {
        val points = segments.lastOrNull()?.points.orEmpty()
        var walked = 0.0
        var count = if (points.isEmpty()) 0 else 1
        for (i in points.size - 1 downTo 1) {
            walked += FlownTrack.metresBetween(points[i - 1], points[i])
            if (walked > FlownTrack.RECENT_M) break
            count++
        }
        return TrackMark(segments, recentCount = count)
    }

    private fun trackShapes(scene: Scene) =
        scene.shapes.filter { it.ink == Ink.TRACK || it.ink == Ink.TRACK_PAST }

    private fun legs(path: Shape.Path): List<Double> =
        (1 until path.points.size).map {
            hypot(
                path.points[it].x - path.points[it - 1].x,
                path.points[it].y - path.points[it - 1].y,
            )
        }

    // ── it is context, so it is behind everything ────────────────────────────

    @Test
    fun `the track is drawn first and the aircraft last`() {
        val situation = Situation(
            SituationSource.LIVE,
            aircraft = AircraftMark(at(100.0, 0.0), 0.0),
            orbit = OrbitMark(at(70.0, 0.0), 30.0, 1),
            track = straightTrack(100.0),
        )
        val scene = SituationScene.build(situation, viewport)
        assertTrue("the track must be the first thing painted", scene.shapes.first().ink == Ink.TRACK ||
            scene.shapes.first().ink == Ink.TRACK_PAST)
        assertEquals(
            "the aircraft is the thing being looked for and nothing may be drawn over it",
            Ink.AIRCRAFT,
            scene.shapes.last().ink,
        )
        // And it is under the orbit, not merely under the aircraft.
        val lastTrack = scene.shapes.indexOfLast { it.ink == Ink.TRACK || it.ink == Ink.TRACK_PAST }
        val firstOrbit = scene.shapes.indexOfFirst { it.ink == Ink.ORBIT }
        assertTrue(lastTrack < firstOrbit)
    }

    @Test
    fun `no track, no shapes — the picture is exactly what it was`() {
        val situation = Situation(SituationSource.LIVE, aircraft = AircraftMark(at(0.0, 0.0), 0.0))
        val without = SituationScene.build(situation, viewport)
        val withEmpty = SituationScene.build(situation.copy(track = TrackMark.EMPTY), viewport)
        assertEquals(without.shapes, withEmpty.shapes)
    }

    // ── a gap looks like a gap ───────────────────────────────────────────────

    @Test
    fun `a gap is never drawn as a straight line`() {
        // Two runs of known positions 300 m apart, with nothing known in between.
        val track = mark(
            listOf(
                straightTrack(60.0).segments[0],
                straightTrack(60.0, fromNorth = 360.0).segments[0],
            ),
        )
        val situation = Situation(
            SituationSource.LIVE,
            aircraft = AircraftMark(at(420.0, 0.0), 0.0),
            track = track,
        )
        val scene = SituationScene.build(situation, viewport)
        val paths = trackShapes(scene).filterIsInstance<Shape.Path>()
        assertTrue("each segment is its own path", paths.size >= 2)

        // The load-bearing assertion: the 300 m hole must appear in no path. Every drawn leg is
        // one step of the track, so joining the segments would leave a single leg sixty times
        // longer than any other — which is exactly what a fabricated transit looks like.
        val projection = Projection.fit(SituationScene.extents(situation), viewport)!!
        val stepPx = projection.lengthPx(FlownTrack.STEP_M)
        for (path in paths) {
            for (leg in legs(path)) {
                assertTrue(
                    "a leg of $leg px against a ${stepPx} px step is the gap, drawn as flight",
                    leg <= stepPx * 3.0,
                )
            }
        }
        // The points either side of the hole are all still drawn; they are simply not connected.
        // (A fade boundary inside the newest segment repeats one point, hence the +1.)
        assertTrue(paths.sumOf { it.points.size } in track.pointCount..track.pointCount + 1)
    }

    @Test
    fun `a lone fix between two gaps is a mark rather than nothing`() {
        val track = mark(
            listOf(
                TrackSegment(listOf(at(0.0, 0.0))),
                straightTrack(40.0, fromNorth = 200.0).segments[0],
            ),
        )
        val situation = Situation(SituationSource.LIVE, track = track)
        val scene = SituationScene.build(situation, viewport)
        val dots = scene.shapes.filterIsInstance<Shape.Dot>().filter {
            it.ink == Ink.TRACK || it.ink == Ink.TRACK_PAST
        }
        assertEquals(
            "a one-point path draws nothing at all, and that position was genuinely known",
            1,
            dots.size,
        )
        assertEquals(Ink.TRACK_PAST, dots[0].ink)
    }

    // ── the old part recedes ─────────────────────────────────────────────────

    @Test
    fun `the recent window is drawn apart from the history behind it`() {
        val situation = Situation(
            SituationSource.LIVE,
            aircraft = AircraftMark(at(400.0, 0.0), 0.0),
            track = straightTrack(400.0),
        )
        val scene = SituationScene.build(situation, viewport)
        val paths = trackShapes(scene).filterIsInstance<Shape.Path>()
        assertEquals(2, paths.size)
        assertEquals("history first, so the recent part is painted over it", Ink.TRACK_PAST, paths[0].ink)
        assertEquals(Ink.TRACK, paths[1].ink)
        assertEquals(
            "the two halves share their boundary point, so the fade has no seam in it",
            paths[0].points.last(),
            paths[1].points.first(),
        )
        // The bright half is the head of the track, at the aircraft.
        val projection = Projection.fit(SituationScene.extents(situation), viewport)!!
        val nose = projection.toScreen(at(400.0, 0.0).latDeg, at(400.0, 0.0).lonDeg)
        val head = paths[1].points.last()
        assertTrue(hypot(head.x - nose.x, head.y - nose.y) < 1e-6)
    }

    @Test
    fun `a track shorter than the window is all recent`() {
        val situation = Situation(
            SituationSource.LIVE,
            aircraft = AircraftMark(at(40.0, 0.0), 0.0),
            track = straightTrack(40.0),
        )
        val paths = trackShapes(SituationScene.build(situation, viewport)).filterIsInstance<Shape.Path>()
        assertEquals(1, paths.size)
        assertEquals(Ink.TRACK, paths[0].ink)
    }

    // ── the framing decision ─────────────────────────────────────────────────

    @Test
    fun `the frame follows the recent window and not the whole flight`() {
        val orbitRadiusM = 30.0
        val situation = Situation(
            SituationSource.LIVE,
            aircraft = AircraftMark(at(30.0, 0.0), 0.0),
            orbit = OrbitMark(at(0.0, 0.0), orbitRadiusM, 1),
            // Half a kilometre of approach from the south, ending at the aircraft.
            track = straightTrack(530.0, fromNorth = -500.0),
        )

        val windowed = Projection.fit(SituationScene.extents(situation), viewport)!!
        val whole = Projection.fit(
            SituationScene.extents(situation.copy(track = null)) +
                situation.track!!.segments.flatMap { s -> s.points.map { Extent(it.latDeg, it.lonDeg) } },
            viewport,
        )!!

        val shortAxisPx = minOf(viewport.usableWidthPx, viewport.usableHeightPx)
        val windowedFraction = windowed.lengthPx(2 * orbitRadiusM) / shortAxisPx
        val wholeFraction = whole.lengthPx(2 * orbitRadiusM) / shortAxisPx

        // The measured numbers in this file's header. They are the argument, not decoration:
        // fitting the whole track is not *wrong*, it has simply made the manoeuvre unreadable.
        assertEquals(0.1132, wholeFraction, 0.005)
        assertEquals(0.5000, windowedFraction, 0.005)
        assertTrue(
            "the recent window must frame the manoeuvre, not the flight so far",
            windowedFraction > 4 * wholeFraction,
        )

        // The old track is still drawn — it just runs off the edge, as a moving map's does.
        val scene = SituationScene.build(situation, viewport)
        val drawn = trackShapes(scene).filterIsInstance<Shape.Path>().sumOf { it.points.size }
        assertTrue("all of it is painted, $drawn points", drawn >= situation.track!!.pointCount)
        assertTrue(
            "and some of it is off screen, which is the price of the decision",
            trackShapes(scene).filterIsInstance<Shape.Path>()
                .any { p -> p.points.any { !SituationScene.inside(it, viewport) } },
        )
    }

    @Test
    fun `a track alone still frames a picture`() {
        // No aircraft — the fix went stale — but the history is still worth looking at.
        val situation = Situation(SituationSource.LIVE, track = straightTrack(60.0))
        val scene = SituationScene.build(situation, viewport)
        assertTrue(situation.hasAnything)
        assertTrue(trackShapes(scene).isNotEmpty())
        assertEquals(
            "with a track in the picture the scale bar has something to measure",
            true,
            scene.scaleBarMetres != null,
        )
    }
}
