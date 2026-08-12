package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.situation.SituationReading
import com.dimensional.mini4pro.situation.SituationSource
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The replay cursor: where in a recording we are, and what state that means.
 *
 * Hand-cranked clock throughout — [ReplayPlayer] reads no clock of its own, so every timing
 * property here is exact rather than flaky.
 *
 * Written to fail loudly for:
 *
 *  - **interpolation between samples**, which would invent an aircraft state that was never
 *    reported, at exactly the instants the recorded staleness says nothing was reported
 *  - **the cursor jumping by however long the phone was asleep** when the activity comes back,
 *    which looks precisely like a real aircraft teleporting
 *  - a replayed sample that has **lost its recorded staleness**, so the picture would stop
 *    degrading where the real one did
 *  - running off the end of the recording, or dividing by zero on an empty one
 *
 * Mutation counts for the replay path are recorded in `SituationHonestyTest`, which is where the
 * replay/command separation is argued. The three cursor mutations — interpolating between
 * samples (3), counting the background gap as playback time (2), and running past the end of the
 * recording (1) — are measured against this suite and listed in that table.
 */
class ReplayPlayerTest {

    private val lat = 37.9938232
    private val lon = 23.7253477

    /** A sample at `t`, [northM] north of the site, with the recorded ages a caller passes. */
    private fun sample(t: Double, northM: Double, positionAgeMs: Long? = 100L) = ReplaySample(
        tSeconds = t,
        monoNanos = null,
        unixMillis = null,
        state = AircraftState(
            fcConnected = true,
            latitude = lat + northM / 111_194.93,
            longitude = lon,
            yawDeg = 45.0,
            ages = SampleAges.of(
                buildMap {
                    positionAgeMs?.let { put(Signal.POSITION, it) }
                    put(Signal.ATTITUDE, 200L)
                },
            ),
        ),
    )

    private fun player(vararg samples: ReplaySample) = ReplayPlayer(samples.toList(), "20260727-1.jsonl")

    private val threeSeconds = listOf(sample(10.0, 0.0), sample(11.0, 10.0), sample(12.0, 20.0), sample(13.0, 30.0))

    @Test
    fun `an empty recording has no duration, no sample and no divide by zero`() {
        val p = ReplayPlayer(emptyList())
        assertTrue(p.isEmpty)
        assertEquals(0.0, p.durationSeconds, 0.0)
        assertNull(p.current())
        assertEquals(0.0, p.fraction(), 0.0)
        p.play(1_000L)
        p.onClock(9_000L)
        assertEquals(0.0, p.positionSeconds, 0.0)
    }

    @Test
    fun `duration is measured from the record's own first sample, not from zero`() {
        // A session file split at t=10 still replays as three seconds, not thirteen.
        assertEquals(3.0, ReplayPlayer(threeSeconds).durationSeconds, 1e-9)
    }

    @Test
    fun `a fresh player is paused at the beginning`() {
        val p = ReplayPlayer(threeSeconds)
        assertFalse("a replay is opened, then run", p.playing)
        assertEquals(0.0, p.positionSeconds, 0.0)
        assertEquals(10.0, p.current()!!.tSeconds, 1e-9)
    }

    /** Runs the player forward at the UI's own 200 ms cadence. */
    private fun run(p: ReplayPlayer, fromMs: Long, forMs: Long): Long {
        var t = fromMs
        while (t < fromMs + forMs) {
            t += 200L
            p.onClock(t)
        }
        return t
    }

    @Test
    fun `the cursor advances in real time`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(1_000L)
        run(p, 1_000L, 1_500L)
        assertEquals(1.6, p.positionSeconds, 1e-9)
        assertEquals(1.6 / 3.0, p.fraction(), 1e-9)
    }

    @Test
    fun `a gap the Activity spent in the background is not playback time`() {
        // Routine, not exotic: this app spends whole sessions behind DJI Fly. Play at t=1000,
        // the phone sleeps, the first frame back arrives at t=400000. Counting that gap would
        // leap 399 s in one frame — an aircraft teleporting across the picture.
        val p = ReplayPlayer(threeSeconds)
        p.play(1_000L)
        p.onClock(400_000L)
        assertEquals(
            "at most one clamped step, not the six and a half minutes it was asleep",
            ReplayPlayer.MAX_STEP_MS / 1000.0,
            p.positionSeconds,
            1e-9,
        )
    }

    @Test
    fun `a clock that runs backwards does not run the replay backwards`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(1_000L)
        run(p, 1_000L, 1_000L)
        p.onClock(1_500L)
        assertEquals(1.0, p.positionSeconds, 1e-9)
    }

    @Test
    fun `the sample in force is the last one at or before the cursor, never a blend`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(0L)
        run(p, 0L, 1_400L) // 1.4 s in: between the t=11 and t=12 samples
        val s = p.current()!!
        assertEquals("held, not interpolated", 11.0, s.tSeconds, 1e-9)
        // And the position is the recorded one, not something between two of them.
        assertEquals(threeSeconds[1].state.latitude!!, s.state.latitude!!, 1e-12)
    }

    @Test
    fun `the recorded staleness survives into the replayed picture`() {
        // The whole reason FlightReplay replays ages verbatim: a sample whose position feed had
        // died must produce a picture with no aircraft in it, exactly as it did on the day.
        val stale = ReplayPlayer(listOf(sample(0.0, 0.0, positionAgeMs = 6_000L)))
        val situation = SituationReading.read(SituationSource.REPLAY, stale.current()!!.state)
        assertNull("a replayed dead feed must degrade the picture too", situation.aircraft)
        assertTrue(SituationReading.NOTE_NO_FIX in situation.notes)

        val fresh = ReplayPlayer(listOf(sample(0.0, 0.0, positionAgeMs = 80L)))
        assertNotNull(SituationReading.read(SituationSource.REPLAY, fresh.current()!!.state).aircraft)
    }

    @Test
    fun `the cursor stops at the end and stops playing`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(0L)
        run(p, 0L, 5_000L)
        assertEquals(3.0, p.positionSeconds, 1e-9)
        assertTrue(p.atEnd)
        assertFalse("nothing keeps running past the end of a recording", p.playing)
        assertEquals(13.0, p.current()!!.tSeconds, 1e-9)
    }

    @Test
    fun `playing from the end starts over rather than sitting still`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(0L)
        val t = run(p, 0L, 5_000L)
        p.play(t)
        assertEquals(0.0, p.positionSeconds, 1e-9)
        assertTrue(p.playing)
    }

    @Test
    fun `pause stops time and keeps the picture`() {
        val p = ReplayPlayer(threeSeconds)
        p.play(0L)
        run(p, 0L, 1_000L)
        p.pause()
        run(p, 1_000L, 50_000L)
        assertEquals(1.0, p.positionSeconds, 1e-9)
        assertEquals(11.0, p.current()!!.tSeconds, 1e-9)
    }

    @Test
    fun `toggle alternates, and resuming does not swallow the gap it was paused for`() {
        val p = ReplayPlayer(threeSeconds)
        p.toggle(0L)
        assertTrue(p.playing)
        run(p, 0L, 400L)
        p.toggle(400L)
        assertFalse(p.playing)
        p.toggle(90_000L) // resumed after a minute and a half of standing still
        run(p, 90_000L, 600L)
        assertEquals("only the time spent playing counts", 1.0, p.positionSeconds, 1e-9)
    }

    @Test
    fun `seeking clamps to the recording and leaves play state alone`() {
        val p = ReplayPlayer(threeSeconds)
        p.seekFraction(0.5, 0L)
        assertEquals(1.5, p.positionSeconds, 1e-9)
        assertFalse(p.playing)
        p.seekFraction(9.0, 0L)
        assertEquals(3.0, p.positionSeconds, 1e-9)
        p.seekFraction(-4.0, 0L)
        assertEquals(0.0, p.positionSeconds, 1e-9)
        p.seekFraction(Double.NaN, 0L)
        assertEquals("a NaN seek is not a seek to zero, it is not a seek", 0.0, p.positionSeconds, 1e-9)
    }

    @Test
    fun `the rate multiplies elapsed time and refuses nonsense`() {
        val p = ReplayPlayer(threeSeconds)
        p.setRate(2.0, 0L)
        p.play(0L)
        run(p, 0L, 1_000L)
        assertEquals(2.0, p.positionSeconds, 1e-9)
        p.setRate(0.0, 1_000L)
        p.setRate(-1.0, 1_000L)
        p.setRate(Double.NaN, 1_000L)
        assertEquals("a bad rate leaves the good one standing", 2.0, p.rate, 1e-9)
    }

    @Test
    fun `a single-sample recording has no duration but still has a picture`() {
        val p = ReplayPlayer(listOf(sample(4.0, 0.0)))
        assertEquals(0.0, p.durationSeconds, 0.0)
        assertEquals(0.0, p.fraction(), 0.0)
        assertNotNull(p.current())
        p.play(0L)
        p.onClock(5_000L)
        assertEquals(0.0, p.positionSeconds, 1e-9)
    }
}
