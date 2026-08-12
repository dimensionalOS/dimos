package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.replay.FlightRecord
import com.dimensional.mini4pro.replay.FlightRecordReader
import com.dimensional.mini4pro.replay.FlightReplay
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

/**
 * **[FlownTrack]: what may be remembered, how much of it, and where the line must break.**
 *
 * Pure JVM, hand-cranked clock, no aircraft and no phone — the same protocol the rest of
 * `situation/` is tested under. The scene's half of the feature (paint order, the fade, and the
 * framing decision) is next door in `SituationTrackTest`.
 *
 * A track is the only thing in this package that *remembers*, which makes it the only thing here
 * that can lie about more than one instant at a time. A polyline is a claim about every point
 * between its vertices as well as about the vertices themselves, so the two failures worth
 * writing tests against are:
 *
 *  - **a gap drawn as a straight line** — the aircraft was out of contact for fifteen seconds and
 *    the picture draws the chord, which is a transit across ground it may never have crossed;
 *  - **a stale position becoming a track point** — the live symbol correctly disappears while the
 *    history quietly keeps plotting the last fix, so the aircraft is absent from the picture and
 *    present in the picture at the same time.
 *
 * Both are mutation-checked below. The rest of the suite pins the engineering: thinning by
 * distance so a hover costs nothing and a fast leg is not sparse, a fixed bound that holds under
 * a synthetic twenty-five-minute flight, and the clearing rules.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, run,
 * confirmed red, reverted. Counts are failing tests across `FlownTrackTest` and
 * `SituationTrackTest` — **measured, not estimated**.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a gap joined up (the break rule dropped: every fix continues the segment) | 4 |
 *  | the gap measured between *kept* points instead of known samples (a hover breaks the line) | 3 |
 *  | a no-fix sample plotted at the previous position (a stale fix becomes a point) | 4 |
 *  | a no-fix sample counted as a known *time* (a dropout hidden by the freshness of nothing) | 2 |
 *  | the implausible-jump break removed (a datum change drawn as a transit) | 1 |
 *  | thinning switched from distance to every-Nth-sample | 5 |
 *  | thinning dropped entirely (every fix kept) | 7 |
 *  | the ring bound removed (points appended without limit) | 1 |
 *  | the ring dropping the *newest* point instead of the oldest | 1 |
 *  | the long-silence clear downgraded to a mere break | 2 |
 *  | a change of source left the old flight's points in place | 1 |
 *  | the auto-fit taking the whole track instead of the recent window | 1 |
 *  | the recent window walking back across a break | 1 |
 *  | the track drawn over the aircraft instead of behind it | 1 |
 *  | `cos(latitude)` dropped from the thinning distance | 1 |
 *
 * Two of these rows started at **zero** and are the reason the suite has the fixture it has.
 * Both hid behind a *different* rule catching the same fixture by accident: a dropout written
 * with the aircraft resuming 300 m away is broken by the implausible-jump rule whatever the gap
 * rule does, and a window walked across a 300 m hole runs out of budget on the first step. Only
 * a dropout that resumes **20 m** on — the shape a real dropout has — tests the rule that is
 * supposed to be doing the work. A mutation table is worth writing precisely because it finds
 * tests that pass for the wrong reason.
 *
 * The two rows the file exists for are the first and the third. Both are silent in every other
 * test in the project: the picture still draws, still frames, still scales, and is wrong about
 * where an aircraft has been.
 */
class FlownTrackTest {

    // The project's home site, 38°N — where a dropped `cos(latitude)` is a 21 % east error and
    // an equator-based fixture would notice nothing.
    private val siteLat = 37.9938232
    private val siteLon = 23.7253477

    /** A fix [northM] / [eastM] metres from the site. */
    private fun at(northM: Double, eastM: Double): Fix {
        val (lat, lon) = Geo.offsetCoordinate(siteLat, siteLon, northM, eastM)
        return Fix(lat, lon)
    }

    /** A live situation carrying that fix, or carrying no aircraft at all when null. */
    private fun live(fix: Fix?): Situation =
        Situation(SituationSource.LIVE, aircraft = fix?.let { AircraftMark(it, null) })

    private fun points(track: FlownTrack): List<Fix> =
        track.accept(live(null), Long.MIN_VALUE / 4)?.segments?.flatMap { it.points } ?: emptyList()

    /** Straight-line distance between two fixes, through the same `Geo` the source uses. */
    private fun metres(a: Fix, b: Fix) = FlownTrack.metresBetween(a, b)

    // ── thinning ─────────────────────────────────────────────────────────────

    @Test
    fun `a hover adds nothing, however long it hovers`() {
        val track = FlownTrack()
        var now = 0L
        // Five minutes of a stationary aircraft with 1.5 m of GPS noise on it, at 5 Hz.
        repeat(1_500) { i ->
            val a = i * 0.7
            track.accept(live(at(1.5 * sin(a), 1.5 * cos(a))), now)
            now += 200L
        }
        assertEquals(
            "a stationary aircraft costs one point — thinning is by distance, and it did not move",
            1,
            track.pointCount,
        )
    }

    @Test
    fun `a straight leg is kept in proportion to its length`() {
        val short = flyStraight(metres = 100.0, speedMs = 10.0)
        val long = flyStraight(metres = 400.0, speedMs = 10.0)
        // 5 m thinning against 2 m sample steps keeps a point every 6 m.
        assertTrue("100 m gave ${short.pointCount}", short.pointCount in 15..21)
        assertTrue("400 m gave ${long.pointCount}", long.pointCount in 60..81)
        val ratio = long.pointCount.toDouble() / short.pointCount
        assertTrue("four times the distance, four times the points, got $ratio", ratio in 3.4..4.6)
    }

    @Test
    fun `thinning is by distance and not by time`() {
        // The same 90 m of ground, flown at 15 m/s and at 1,5 m/s: ten times the samples.
        val fast = flyStraight(metres = 90.0, speedMs = 15.0)
        val slow = flyStraight(metres = 90.0, speedMs = 1.5)
        assertTrue(
            "fast ${fast.pointCount} vs slow ${slow.pointCount} — a time-based thinner would " +
                "make the slow leg ten times denser and the fast one ten times sparser",
            kotlin.math.abs(fast.pointCount - slow.pointCount) <= 3,
        )
        assertTrue(fast.pointCount >= 10)
    }

    @Test
    fun `no two kept points are closer together than the step`() {
        val kept = points(flyStraight(metres = 300.0, speedMs = 8.0))
        for (i in 1 until kept.size) {
            val d = metres(kept[i - 1], kept[i])
            assertTrue("leg $i is ${d} m, under the ${FlownTrack.STEP_M} m step", d >= FlownTrack.STEP_M - 1e-6)
        }
        assertTrue(kept.size > 20)
    }

    @Test
    fun `a hundred metres east costs the same as a hundred metres north`() {
        val north = flyStraight(metres = 100.0, speedMs = 8.0)
        val east = flyStraight(metres = 100.0, speedMs = 8.0, eastward = true)
        // At 38°N a degree of longitude is 0,788 of a degree of latitude. Thinning that skipped
        // `Geo.longitudeScale` would measure this east leg as 127 m and keep a fifth fewer
        // points — a difference nobody would ever see on screen, and the exact mistake
        // `Geo.longitudeScale` exists to make impossible.
        assertTrue(
            "north ${north.pointCount} vs east ${east.pointCount}",
            kotlin.math.abs(north.pointCount - east.pointCount) <= 1,
        )
        assertTrue(north.pointCount >= 12)
    }

    // ── the gap rule ─────────────────────────────────────────────────────────

    @Test
    fun `a silence too long to explain is a gap, not a leg`() {
        val track = FlownTrack()
        var now = 0L
        var north = 0.0
        repeat(40) {
            track.accept(live(at(north, 0.0)), now)
            north += 2.0
            now += 200L
        }
        // Four seconds of no fresh fix. The aircraft could have gone anywhere and come back.
        repeat(20) {
            track.accept(live(null), now)
            now += 200L
        }
        // …and it resumes 300 m away, which is where it happened to be.
        var resumed = 300.0
        val mark = run {
            var m: TrackMark? = null
            repeat(40) {
                m = track.accept(live(at(resumed, 0.0)), now)
                resumed += 2.0
                now += 200L
            }
            m!!
        }
        assertEquals("the silence must cut the track in two", 2, mark.segments.size)
        val before = mark.segments[0].points.last()
        val after = mark.segments[1].points.first()
        assertTrue("the two sides are 220 m apart and must not be joined", metres(before, after) > 200.0)
        for (segment in mark.segments) {
            for (i in 1 until segment.points.size) {
                assertTrue(
                    "no drawn leg may cross the gap",
                    metres(segment.points[i - 1], segment.points[i]) < FlownTrack.JUMP_M,
                )
            }
        }
    }

    @Test
    fun `one dropped sample is not a gap`() {
        val track = FlownTrack()
        var now = 0L
        var north = 0.0
        repeat(20) {
            track.accept(live(at(north, 0.0)), now)
            north += 2.0
            now += 200L
        }
        // A single missing sample: 400 ms of not knowing, at most 6 m of assumption.
        track.accept(live(null), now)
        now += 200L
        var mark: TrackMark? = null
        repeat(20) {
            mark = track.accept(live(at(north, 0.0)), now)
            north += 2.0
            now += 200L
        }
        assertEquals(
            "breaking on every dropped sample would shred an ordinary flight into confetti",
            1,
            mark!!.segments.size,
        )
    }

    @Test
    fun `a stale position never becomes a track point`() {
        // The state a phone actually holds when the feed dies: a perfectly plausible coordinate
        // whose age is past `Signal.POSITION`'s limit. `SituationReading` removes the symbol…
        val stale = AircraftState(
            fcConnected = true,
            latitude = siteLat,
            longitude = siteLon,
            ages = SampleAges.of(mapOf(Signal.POSITION to 9_000L)),
        )
        val situation = SituationReading.read(SituationSource.LIVE, stale)
        assertNull("precondition: the live symbol is already gone", situation.aircraft)

        // …and the track must not quietly keep plotting it.
        val track = FlownTrack()
        var now = 0L
        repeat(50) {
            track.accept(situation, now)
            now += 200L
        }
        assertEquals(0, track.pointCount)
        assertNull("nothing known, nothing drawn", track.accept(situation, now))
    }

    @Test
    fun `a fresh fix either side of a stale run is still two segments`() {
        val fresh = { northM: Double ->
            SituationReading.read(
                SituationSource.LIVE,
                AircraftState(
                    fcConnected = true,
                    latitude = Geo.offsetCoordinate(siteLat, siteLon, northM, 0.0).first,
                    longitude = Geo.offsetCoordinate(siteLat, siteLon, northM, 0.0).second,
                    ages = SampleAges.of(mapOf(Signal.POSITION to 100L)),
                ),
            )
        }
        val stale = SituationReading.read(
            SituationSource.LIVE,
            AircraftState(
                fcConnected = true,
                latitude = siteLat,
                longitude = siteLon,
                ages = SampleAges.of(mapOf(Signal.POSITION to 9_000L)),
            ),
        )
        val track = FlownTrack()
        var now = 0L
        repeat(10) { track.accept(fresh(it * 6.0), now); now += 200L }
        repeat(30) { track.accept(stale, now); now += 200L }
        var mark: TrackMark? = null
        repeat(10) { mark = track.accept(fresh(200.0 + it * 6.0), now); now += 200L }
        assertEquals(2, mark!!.segments.size)
    }

    @Test
    fun `a step no aircraft could have flown is a break`() {
        val track = FlownTrack()
        track.accept(live(at(0.0, 0.0)), 0L)
        track.accept(live(at(10.0, 0.0)), 200L)
        // 400 m in 200 ms. A datum change, a new flight, or a fix nobody should have believed.
        val mark = track.accept(live(at(410.0, 0.0)), 400L)!!
        assertEquals(2, mark.segments.size)
        assertEquals(1, mark.segments[1].points.size)
    }

    // ── clearing ─────────────────────────────────────────────────────────────

    @Test
    fun `a minute of not knowing forgets the flight rather than breaking it`() {
        val track = FlownTrack()
        var now = 0L
        repeat(60) {
            track.accept(live(at(it * 6.0, 0.0)), now)
            now += 200L
        }
        assertTrue(track.pointCount > 20)
        // The aircraft was off, out of range, or the app was behind DJI Fly for two minutes.
        val mark = track.accept(live(at(0.0, 0.0)), now + 120_000L)!!
        assertEquals("what comes back is very likely a different flight", 1, mark.segments.size)
        assertEquals(1, track.pointCount)
    }

    @Test
    fun `a change of source clears everything`() {
        val track = FlownTrack()
        var now = 0L
        repeat(40) {
            track.accept(live(at(it * 6.0, 0.0)), now)
            now += 200L
        }
        val replayed = Situation(
            SituationSource.REPLAY,
            aircraft = AircraftMark(at(500.0, 500.0), null),
        )
        val mark = track.accept(replayed, now)!!
        assertEquals("a live track under a replayed one is two afternoons in one picture", 1, mark.pointCount)
        // And back again.
        assertEquals(1, track.accept(live(at(0.0, 0.0)), now + 200L)!!.pointCount)
    }

    @Test
    fun `a clock that runs backwards starts the track over`() {
        val track = FlownTrack()
        var now = 100_000L
        repeat(40) {
            track.accept(live(at(it * 6.0, 0.0)), now)
            now += 200L
        }
        // A replay looping back to the start of the recording.
        val mark = track.accept(live(at(0.0, 0.0)), 0L)!!
        assertEquals(1, mark.pointCount)
    }

    @Test
    fun `clear empties the track outright`() {
        val track = FlownTrack()
        repeat(40) { track.accept(live(at(it * 6.0, 0.0)), it * 200L) }
        assertTrue(track.pointCount > 1)
        track.clear()
        assertEquals(0, track.pointCount)
        assertNull(track.accept(live(null), 10_000L))
    }

    // ── the bound ────────────────────────────────────────────────────────────

    @Test
    fun `a long flight cannot grow the buffer past its bound`() {
        val track = FlownTrack()
        var now = 0L
        var north = 0.0
        // Twenty-five minutes at 15 m/s, sampled at 5 Hz: 7500 samples, 22.5 km of ground —
        // more than a battery, flown flat out, which is the worst case that exists.
        repeat(7_500) {
            track.accept(live(at(north, 0.0)), now)
            north += 3.0
            now += 200L
        }
        assertEquals(FlownTrack.MAX_POINTS, track.pointCount)
        val mark = track.accept(live(at(north, 0.0)), now)!!
        assertEquals(FlownTrack.MAX_POINTS, mark.pointCount)
        // The *recent* end is what survives: the newest point is where the aircraft just was.
        val newest = mark.newest!!
        assertTrue("the newest point must be the latest fix", metres(newest, at(north - 6.0, 0.0)) < 12.0)
        // …and the oldest held point is the bound's worth of path behind it, not the take-off.
        val oldest = mark.segments.first().points.first()
        val held = metres(oldest, newest)
        assertTrue("held $held m of path", held in 8_000.0..14_000.0)
    }

    @Test
    fun `an unchanged track allocates no new mark`() {
        val track = FlownTrack()
        repeat(10) { track.accept(live(at(it * 6.0, 0.0)), it * 200L) }
        val a = track.accept(live(at(54.5, 0.0)), 2_100L)
        val b = track.accept(live(at(54.6, 0.0)), 2_300L)
        assertNotNull(a)
        assertSame(
            "a frame that adds no point must reuse the mark — at 5 Hz over 2048 points, " +
                "rebuilding it every frame is 200 kB/s of garbage on a phone that is flying",
            a,
            b,
        )
    }

    // ── the recent window ────────────────────────────────────────────────────

    /**
     * **The window is a span of time**, [FlownTrack.RECENT_MS] — Ivan's call on 2026-07-27 after
     * looking at the track on the phone. 400 m at 2 m/s is 200 s of flying, so the last 120 s is
     * the last 240 m and the first 160 m has aged out.
     *
     * This test used to be called *"the recent window is measured along the path"* and asserted
     * 110–126 m. That was the old rule, and the reason it changed is worth keeping: a distance
     * window shows less of a fast flight than of a slow one, so "the recent part of the track"
     * silently meant forty seconds on a transit and two minutes on an orbit. Seconds are what an
     * operator actually counts in.
     */
    @Test
    fun `the recent window is a span of time, not of path`() {
        val speed = 2.0
        val mark = flyStraight(metres = 400.0, speedMs = speed)
            .accept(live(null), 1_000_000L)!!
        val tail = mark.recentTail()
        val walked = (1 until tail.size).sumOf { metres(tail[it - 1], tail[it]) }
        val expected = speed * FlownTrack.RECENT_MS / 1000.0
        assertTrue(
            "the window walked $walked m; ${FlownTrack.RECENT_MS} ms at $speed m/s is $expected m",
            walked in (expected - FlownTrack.STEP_M * 2)..(expected + FlownTrack.STEP_M * 2),
        )
        assertTrue("it is a window, not the whole track", walked < 400.0 - FlownTrack.STEP_M)
        assertTrue("and it is the *recent* end", tail.last() == mark.newest)
    }

    /**
     * The same track flown fast enough to fit inside the window is **all** recent — the property
     * the time window exists for, and the one a distance window cannot have.
     */
    @Test
    fun `a whole fast flight inside the window is all recent`() {
        // 400 m at 8 m/s is 50 s, comfortably inside 120 s.
        val mark = flyStraight(metres = 400.0, speedMs = 8.0)
            .accept(live(null), 1_000_000L)!!
        assertEquals(mark.segments.last().points.size, mark.recentCount)
    }

    /**
     * **The distance floor, and why it has to be there.**
     *
     * Thinning is by distance, so a hover keeps exactly one point however long it lasts. Under a
     * pure time window, holding station for longer than [FlownTrack.RECENT_MS] would leave the
     * window containing that single point — and the auto-fit, which frames the window, would
     * collapse onto a dot the moment the aircraft stopped, taking the approach that led there out
     * of the picture. [FlownTrack.RECENT_M] of path is shown regardless.
     */
    @Test
    fun `flying on after a long hover still shows the approach, not just the last few metres`() {
        // Built inline with one clock rather than from `flyStraight`, because handing that track a
        // much later timestamp trips rule 4 (a silence past FORGET_MS is a different flight) and
        // clears it — correct behaviour, and it would make this test pass for the wrong reason.
        val track = FlownTrack()
        var now = 0L
        val step = 8.0 / 5.0
        var run = 0.0
        while (run <= 400.0) {
            track.accept(live(at(run, 0.0)), now)
            run += step
            now += 200L
        }
        // Hold station for well past the time window, still sampling at the drawing rate.
        val parked = at(400.0, 0.0)
        repeat(1500) { // 300 s at 5 Hz
            track.accept(live(parked), now)
            now += 200L
        }
        // **Then move off again.** This, not the hover itself, is the case the floor is for.
        // While the aircraft sits still the newest *point* stops advancing, so the window — which
        // is anchored on the newest point's own time, not on the wall clock — keeps showing the
        // 120 s of flying that led up to the hover, and needs no help. It is the first step
        // afterwards that re-anchors the window on a fresh timestamp and ages the entire approach
        // out at once, leaving a frame fitted to the last few metres.
        var mark = track.accept(live(parked), now)!!
        var on = 400.0
        while (on <= 420.0) {
            mark = track.accept(live(at(on, 0.0)), now)!!
            on += step
            now += 200L
        }

        val tail = mark.recentTail()
        val walked = (1 until tail.size).sumOf { metres(tail[it - 1], tail[it]) }
        assertTrue(
            "after moving off from a 300 s hover the window walked only $walked m — the approach " +
                "aged out in one step and the frame would snap to the last few metres",
            walked >= FlownTrack.RECENT_M - FlownTrack.STEP_M * 2,
        )
        assertTrue("and it is still the *recent* end", tail.last() == mark.newest)
    }

    /**
     * The companion to the test above: **during** the hover nothing is needed, because the window
     * is anchored on the newest point rather than on now. Worth pinning, because it is the reason
     * the floor's justification is "resuming from a hover" and not "hovering".
     */
    @Test
    fun `during a long hover the window still shows the approach on the clock alone`() {
        val track = FlownTrack()
        var now = 0L
        val step = 8.0 / 5.0
        var run = 0.0
        while (run <= 400.0) {
            track.accept(live(at(run, 0.0)), now)
            run += step
            now += 200L
        }
        val parked = at(400.0, 0.0)
        var mark = track.accept(live(parked), now)!!
        repeat(1500) {
            now += 200L
            mark = track.accept(live(parked), now)!!
        }
        val tail = mark.recentTail()
        val walked = (1 until tail.size).sumOf { metres(tail[it - 1], tail[it]) }
        assertTrue("the window walked $walked m during the hover", walked > FlownTrack.RECENT_M)
    }

    @Test
    fun `the recent window stops at a break the aircraft flew straight through`() {
        // The hard case, and the one a naive window gets wrong: the aircraft was lost for ten
        // seconds and came back **20 m away** — near enough that a walk measured only in metres
        // would step across the hole and keep counting into ground we did not watch it cross.
        val mark = dropoutNearby()
        val resumed = mark.segments.last().points.size
        assertEquals(2, mark.segments.size)
        assertTrue(
            "the window counted ${mark.recentCount} points against a " +
                "resumed segment of $resumed — it walked back over the break",
            mark.recentCount <= resumed,
        )
        for (p in mark.recentTail()) {
            assertTrue("every tail point belongs to the resumed segment", p in mark.segments.last().points)
        }
    }

    @Test
    fun `a dropout is a gap even when the aircraft comes back where it left`() {
        // Ten seconds of nothing, resuming 20 m on: too close for the implausible-jump rule to
        // catch, so this is the gap rule alone, and it is the case a real dropout looks like.
        // Counting a no-fix sample as a *known* time — a freshness taken from the arrival of
        // nothing — would join these two runs into one line across ten unwatched seconds.
        val mark = dropoutNearby()
        assertEquals(2, mark.segments.size)
        val across = metres(mark.segments[0].points.last(), mark.segments[1].points.first())
        assertTrue("the hole is $across m wide and must stay a hole", across in 10.0..FlownTrack.JUMP_M)
    }

    /** 200 m north, ten seconds of no fix, then 30 m more from 20 m further on. */
    private fun dropoutNearby(): TrackMark {
        val track = FlownTrack()
        var now = 0L
        var north = 0.0
        while (north <= 200.0) {
            track.accept(live(at(north, 0.0)), now)
            north += 3.0
            now += 200L
        }
        repeat(50) {
            track.accept(live(null), now)
            now += 200L
        }
        north = 220.0
        var mark: TrackMark? = null
        while (north <= 250.0) {
            mark = track.accept(live(at(north, 0.0)), now)
            north += 3.0
            now += 200L
        }
        return mark!!
    }

    // ── the same rules, applied to a real flight ─────────────────────────────

    /**
     * `replay/orbit-real-air.jsonl` — session `20260727-102510`, 371 s over Athens with a
     * commanded 24.7 m orbit in the middle of it, replayed through exactly the path the phone
     * uses: [FlightReplay.samples] → [SituationReading.read] → [FlownTrack.accept].
     *
     * This is the cheapest possible check that the thing looks right, and the measured numbers
     * are the point of it: **1855 samples become 36 points in 1 unbroken segment** — 187.1 m of
     * flown path, 1.9 % of the raw feed — and the widest span across those 36 points is
     * **49.9 m**, against a commanded orbit of 24.79 m radius, i.e. 49.58 m across. The shape
     * survived the thinning to within 30 cm. Thinning that flattened the circle, or a
     * `cos(latitude)` lost on the way in, shows up here as a span that no longer matches the
     * orbit the aircraft actually flew.
     */
    @Test
    fun `a recorded flight builds a track through the real pipeline`() {
        val samples = FlightReplay.samples(record)
        assertTrue("fixture parsed ${samples.size} samples", samples.size > 1_000)
        val track = FlownTrack()
        var mark: TrackMark? = null
        for (sample in samples) {
            val situation = SituationReading.read(SituationSource.REPLAY, sample.state)
            mark = track.accept(situation, (sample.tSeconds * 1000.0).toLong())
        }
        assertNotNull("the flight left a track", mark)
        val m = mark!!
        assertTrue("kept ${m.pointCount} of ${samples.size} samples", m.pointCount in 25..600)
        assertEquals("the recorded flight never lost position long enough to break", 1, m.segments.size)
        assertTrue("bounded", m.pointCount <= FlownTrack.MAX_POINTS)

        // No leg in any segment may be one the aircraft could not have flown.
        for (segment in m.segments) {
            for (i in 1 until segment.points.size) {
                val d = metres(segment.points[i - 1], segment.points[i])
                assertTrue("leg of $d m", d in FlownTrack.STEP_M - 1e-6..FlownTrack.JUMP_M)
            }
        }
        // The orbit is in there: the track spans more than the commanded 24.79 m circle.
        val all = m.segments.flatMap { it.points }
        var widest = 0.0
        for (i in all.indices) {
            for (j in i + 1 until all.size) widest = maxOf(widest, metres(all[i], all[j]))
        }
        assertTrue("the track spans $widest m — the flight's own orbit is 49.6 m across", widest > 45.0)
    }

    private val record: FlightRecord by lazy {
        javaClass.classLoader!!.getResourceAsStream("replay/orbit-real-air.jsonl")!!
            .bufferedReader().useLines { FlightRecordReader.read(it) }
    }

    /** A straight leg at [speedMs], sampled at 5 Hz, fed into a fresh track. */
    private fun flyStraight(metres: Double, speedMs: Double, eastward: Boolean = false): FlownTrack {
        val track = FlownTrack()
        val step = speedMs / 5.0
        var run = 0.0
        var now = 0L
        while (run <= metres) {
            track.accept(live(if (eastward) at(0.0, run) else at(run, 0.0)), now)
            run += step
            now += 200L
        }
        return track
    }
}
