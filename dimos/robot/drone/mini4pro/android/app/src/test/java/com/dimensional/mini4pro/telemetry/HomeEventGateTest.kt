package com.dimensional.mini4pro.telemetry

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The on-change rule for `HOME_POSITION` / `GPS_GLOBAL_ORIGIN`.
 *
 * Written against a measured failure: in `tmp/session-logs/retest.jsonl`
 * (2026-07-26 11:17–11:20, aircraft on the ground, never flown) the bridge sent
 * **220 of each** in 220 seconds, because the change key was a string containing a
 * barometric altitude that moved every tick — 62.104833984375006, 62.115…,
 * 62.123… The jitter figures below are the real ones from that session and from
 * `…-102245` / `…-094037`.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | altitude compared exactly (`last.altAmslM != alt`) | 5 |
 *  | altitude dropped from the comparison entirely | 1 |
 *  | latitude dropped from the comparison | 1 |
 *  | longitude dropped from the comparison | 1 |
 *  | threshold widened to 100 m | 1 |
 *  | threshold narrowed to 0.001 m | 4 |
 *  | `published` recorded before the change test (never publishes) | 1 |
 *  | `published` never recorded (publishes every call) | 10 |
 *  | quantised into fixed 1 m buckets instead of hysteresis | 2 |
 *  | `reset()` made a no-op | 1 |
 *  | unknown home clears `published` | 1 |
 */
class HomeEventGateTest {

    /** The real site, and a plausible takeoff datum for it. */
    private val homed = AircraftState(
        fcConnected = true,
        homeLatitude = 37.9938232,
        homeLongitude = 23.7253477,
        takeoffAltitudeAmsl = 62.104833984375006,
    )

    /**
     * What DJI actually reports before a home point exists: `KeyIsHomeLocationSet`
     * false, and `KeyHomeLocation` full of filler rather than null.
     */
    private val djiNoHome = homed.copy(
        homeLocationSet = false,
        homeLatitude = 4.583662361046586E7,
        homeLongitude = 4.583662361046586E7,
    )

    private fun names(messages: List<Any>) = messages.map { it.javaClass.simpleName }.toSet()

    @Test
    fun firstKnownHomePublishesBothMessages() {
        val gate = HomeEventGate()
        assertEquals(setOf("HomePosition", "GpsGlobalOrigin"), names(gate.messagesIfChanged(homed)))
    }

    @Test
    fun theSameHomeIsNotRepublished() {
        val gate = HomeEventGate()
        assertEquals(2, gate.messagesIfChanged(homed).size)
        assertTrue(gate.messagesIfChanged(homed).isEmpty())
        assertTrue(gate.messagesIfChanged(homed).isEmpty())
    }

    /**
     * The bug itself. These are consecutive `takeoffAltitudeAmsl` readings from
     * the retest log with the aircraft parked; every one of them re-sent the pair.
     */
    @Test
    fun barometricJitterDoesNotRepublish() {
        val gate = HomeEventGate()
        assertEquals(2, gate.messagesIfChanged(homed).size)
        val jitter = listOf(
            62.104833984375006, 62.115, 62.123, 62.11, 62.09,
            // Wider excursions measured over ~80 s in …-102245 and ~40 s in …-094037.
            62.712, 62.976, 62.552, 62.876, 62.621,
        )
        for (alt in jitter) {
            assertTrue(
                "alt $alt re-published a home that did not move",
                gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = alt)).isEmpty(),
            )
        }
    }

    /**
     * 220 ticks of jitter is exactly the session that produced 220 messages. One
     * publish, and one only.
     */
    @Test
    fun aWholeSessionOfJitterPublishesOnce() {
        val gate = HomeEventGate()
        var published = 0
        for (tick in 0 until 220) {
            // ±0.35 m of wander around the datum, which is wider than measured.
            val alt = 62.1 + 0.35 * Math.sin(tick / 7.0)
            if (gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = alt)).isNotEmpty()) {
                published++
            }
        }
        assertEquals(1, published)
    }

    @Test
    fun aRealHomeMovePublishesAgain() {
        val gate = HomeEventGate()
        assertEquals(2, gate.messagesIfChanged(homed).size)
        // ~4 m north — the aircraft took off, landed elsewhere, home re-recorded.
        val moved = homed.copy(homeLatitude = 37.9938624)
        assertEquals(setOf("HomePosition", "GpsGlobalOrigin"), names(gate.messagesIfChanged(moved)))
        assertTrue(gate.messagesIfChanged(moved).isEmpty())
    }

    @Test
    fun aLongitudeMoveAlonePublishesAgain() {
        val gate = HomeEventGate()
        gate.messagesIfChanged(homed)
        assertEquals(2, gate.messagesIfChanged(homed.copy(homeLongitude = 23.7254109)).size)
    }

    /**
     * The threshold is a real boundary, so it is asserted from both sides rather
     * than trusted. It is hysteresis against the last *published* datum, not a
     * fixed bucket — see the next test for why that distinction is the point.
     */
    @Test
    fun altitudeThresholdBoundary() {
        val base = 62.0
        val gate = HomeEventGate()
        gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = base))
        // Just under 1.0 m: still the same home datum.
        assertTrue(
            gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = base + 0.999)).isEmpty(),
        )
        // Exactly 1.0 m: worth saying.
        assertEquals(
            2,
            gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = base + 1.0)).size,
        )
        // And it rebases, so the next 1.0 m is measured from 63.0, not 62.0.
        assertTrue(
            gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = base + 1.5)).isEmpty(),
        )
        assertEquals(
            2,
            gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = base + 2.0)).size,
        )
    }

    /**
     * Why hysteresis and not quantisation: a value sitting on a bucket edge and
     * jittering by millimetres would flip buckets on every tick, which is the 1 Hz
     * bug with extra arithmetic. Here it publishes once.
     */
    @Test
    fun aValueSittingOnTheThresholdEdgeDoesNotOscillate() {
        val gate = HomeEventGate()
        var published = 0
        for (tick in 0 until 200) {
            val alt = if (tick % 2 == 0) 62.9999 else 63.0001
            if (gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = alt)).isNotEmpty()) {
                published++
            }
        }
        assertEquals(1, published)
    }

    // ── silence while home is unknown ─────────────────────────────────────────

    @Test
    fun djiNoHomeFillerPublishesNothingEver() {
        val gate = HomeEventGate()
        for (tick in 0 until 220) {
            val alt = 62.1 + 0.02 * tick % 0.3
            assertTrue(gate.messagesIfChanged(djiNoHome.copy(takeoffAltitudeAmsl = alt)).isEmpty())
        }
        assertNull(gate.lastPublished())
    }

    @Test
    fun anUnknownHomePublishesNothing() {
        val gate = HomeEventGate()
        assertTrue(gate.messagesIfChanged(AircraftState()).isEmpty())
        assertTrue(gate.messagesIfChanged(homed.copy(homeLatitude = null)).isEmpty())
        assertTrue(gate.messagesIfChanged(homed.copy(homeLongitude = null)).isEmpty())
        assertTrue(gate.messagesIfChanged(homed.copy(takeoffAltitudeAmsl = null)).isEmpty())
    }

    /** Both routes to "no home", and the one route to a home. */
    @Test
    fun bothSuppressionPathsAndTheValidOne() {
        // DJI says there is no home point, whatever the coordinates look like.
        assertTrue(
            HomeEventGate().messagesIfChanged(homed.copy(homeLocationSet = false)).isEmpty(),
        )
        // The key never arrived, and the coordinates are DJI's filler.
        assertTrue(HomeEventGate().messagesIfChanged(djiNoHome.copy(homeLocationSet = null)).isEmpty())
        // Genuinely set, genuine coordinates.
        assertEquals(2, HomeEventGate().messagesIfChanged(homed.copy(homeLocationSet = true)).size)
        // Never delivered, genuine coordinates — silence on the key is not a veto.
        assertEquals(2, HomeEventGate().messagesIfChanged(homed).size)
    }

    /**
     * A home that gets recorded mid-session — the 2026-07-25 19:16 sequence, where
     * `isHomeLocationSet` went false→true and the filler became the real site 55 s
     * later. Nothing before, exactly one publish after.
     */
    @Test
    fun aHomeRecordedMidSessionPublishesExactlyOnce() {
        val gate = HomeEventGate()
        var published = 0
        for (tick in 0 until 100) {
            val before = djiNoHome.copy(takeoffAltitudeAmsl = 97.512 + 0.001 * tick)
            if (gate.messagesIfChanged(before).isNotEmpty()) published++
        }
        assertEquals(0, published)
        for (tick in 0 until 100) {
            val after = homed.copy(
                homeLocationSet = true,
                takeoffAltitudeAmsl = 97.512 + 0.001 * tick,
            )
            if (gate.messagesIfChanged(after).isNotEmpty()) published++
        }
        assertEquals(1, published)
    }

    /**
     * Home going away and coming back unchanged tells the GCS nothing it does not
     * already have — and a home that flapped would otherwise reproduce the 1 Hz
     * failure at the flap rate.
     */
    @Test
    fun homeLostAndRegainedUnchangedDoesNotRepublish() {
        val gate = HomeEventGate()
        assertEquals(2, gate.messagesIfChanged(homed).size)
        assertTrue(gate.messagesIfChanged(djiNoHome).isEmpty())
        assertTrue(gate.messagesIfChanged(homed).isEmpty())
    }

    @Test
    fun resetPublishesAgainForANewLink() {
        val gate = HomeEventGate()
        assertEquals(2, gate.messagesIfChanged(homed).size)
        assertTrue(gate.messagesIfChanged(homed).isEmpty())
        gate.reset()
        assertNull(gate.lastPublished())
        assertEquals(2, gate.messagesIfChanged(homed).size)
    }

    @Test
    fun lastPublishedDescribesWhatWentOut() {
        val gate = HomeEventGate()
        assertNull(gate.lastPublished())
        gate.messagesIfChanged(homed)
        val line = gate.lastPublished()
        assertNotNull(line)
        assertTrue(line!!, line.startsWith("37.9938232,23.7253477,62.10"))
    }
}
