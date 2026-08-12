package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The gimbal deadband — **the decision that closes the one Zenoh channel a flight record could not
 * reproduce** (`replay/ReplayCoverage.GIMBAL`).
 *
 * `Recorder.sampleGimbal` needs Android and cannot be reached from here; what it *decides* is
 * [GimbalSample.movedFrom], which is pure, and this file is that decision under test.
 *
 * Written to fail loudly for the three ways a deadband on a change-driven key goes wrong:
 *
 *  - **applied to pitch alone**, on the reasonable-sounding grounds that pitch is the only axis
 *    this airframe means anything by. It is the only axis that *moves*; recording it as though it
 *    were the only axis that *exists* is the assumption `LogEntry.Gimbal` deliberately refuses,
 *    and it would silently flatten a future airframe with a real yaw axis.
 *  - **an axis appearing or disappearing treated as no move.** A gimbal that stops reporting yaw
 *    has told us something, and a deadband comparing only the axes both readings happen to carry
 *    would record the last known yaw forever. This is the absence-is-not-zero rule at the one
 *    place it is easiest to drop.
 *  - **too coarse to replay the camera** or so fine it becomes a fourth firehose. 0.5° is ~1 % of
 *    the Mini 4 Pro's ~44° vertical field of view — a framing error invisible in the picture, and
 *    an order finer than the whole-degree steps `guided/OrbitGimbal` commands at ~2 Hz.
 */
class GimbalSampleTest {

    private val band = Recorder.GIMBAL_DEADBAND_DEG

    private fun at(p: Double?, r: Double? = 0.0, y: Double? = 0.0) =
        GimbalSample(pitchDeg = p, rollDeg = r, yawDeg = y, ageMs = 0L)

    // ── the band itself ──────────────────────────────────────────────────────

    @Test
    fun theBandIsHalfADegree() {
        assertEquals(
            "0.5° is ~1 % of this airframe's vertical field of view. Changing it changes how " +
                "faithfully a record can replay where the camera was pointing — say why.",
            0.5, band, 0.0,
        )
    }

    @Test
    fun amoveSmallerThanTheBandIsNotAMove() {
        assertFalse(at(-30.0).movedFrom(at(-30.49), band))
        assertFalse(at(-30.0).movedFrom(at(-29.51), band))
        assertFalse("an identical reading is never a move", at(-30.0).movedFrom(at(-30.0), band))
    }

    @Test
    fun amoveOfTheBandOrMoreIsAMove() {
        assertTrue(at(-30.0).movedFrom(at(-30.5), band))
        assertTrue(at(-30.0).movedFrom(at(-29.5), band))
        assertTrue(at(-30.0).movedFrom(at(-60.0), band))
    }

    // ── every axis, not only pitch ───────────────────────────────────────────

    /**
     * The mutation this kills: `movedFrom` reduced to its pitch conjunct. Pitch is the only axis
     * that moves on this airframe; it is not the only axis recorded, and a roll excursion — which
     * on a Mini 4 Pro would mean the gimbal is in trouble — must not be invisible.
     */
    @Test
    fun everyAxisIsWatched() {
        assertTrue("roll", at(-30.0, r = 0.0).movedFrom(at(-30.0, r = 2.0), band))
        assertTrue("yaw", at(-30.0, y = 0.0).movedFrom(at(-30.0, y = 2.0), band))
        assertTrue("pitch", at(-30.0).movedFrom(at(-20.0), band))
        assertFalse("all three inside the band", at(-30.0, 0.1, 0.1).movedFrom(at(-30.2, 0.2, 0.3), band))
    }

    // ── absence is not zero ──────────────────────────────────────────────────

    @Test
    fun anAxisAppearingIsAlwaysAMove() {
        assertTrue(
            "yaw arriving is a fact about the gimbal, however small the number beside it",
            at(-30.0, y = 0.0).movedFrom(at(-30.0, y = null), band),
        )
        assertTrue(at(-30.0, r = 0.0).movedFrom(at(-30.0, r = null), band))
        assertTrue(at(-30.0).movedFrom(at(null), band))
    }

    @Test
    fun anAxisDisappearingIsAlwaysAMove() {
        assertTrue(
            "a gimbal that stops reporting yaw has told us something; without this the record " +
                "would repeat the last known yaw forever",
            at(-30.0, y = null).movedFrom(at(-30.0, y = 0.0), band),
        )
        assertTrue(at(null).movedFrom(at(-30.0), band))
    }

    @Test
    fun bothAxesAbsentIsNotAMove() {
        assertFalse(
            GimbalSample(null, null, null).movedFrom(GimbalSample(null, null, null), band),
        )
    }

    // ── non-finite readings ──────────────────────────────────────────────────

    @Test
    fun anaNArrivingIsAMoveAndAPersistingNaNIsNot() {
        assertTrue(at(Double.NaN).movedFrom(at(-30.0), band))
        assertTrue(at(-30.0).movedFrom(at(Double.NaN), band))
        assertFalse(
            "a gimbal stuck at NaN must not emit a line per sample",
            at(Double.NaN).movedFrom(at(Double.NaN), band),
        )
    }

    // ── the age is not part of the deadband ──────────────────────────────────

    /**
     * The age moves on every sample by construction. If it counted as a move, the deadband would
     * never suppress anything and the entry would become the fourth firehose it was designed not
     * to be. The age's job is the opposite one — it rides on whatever line *does* get written, so
     * "the camera held steady" and "the feed died" are separable.
     */
    @Test
    fun theAgeIsNotPartOfTheDeadband() {
        val still = GimbalSample(-30.0, 0.0, 0.0, ageMs = 20L)
        val stiller = GimbalSample(-30.0, 0.0, 0.0, ageMs = 9_000L)
        assertFalse(still.movedFrom(stiller, band))
    }

    // ── the line the deadband decides to write ───────────────────────────────

    @Test
    fun theRecordLineCarriesAllThreeAxesAndTheAge() {
        val json = JsonObject.render {
            LogEntry.Gimbal(0L, pitchDeg = -30.25, rollDeg = null, yawDeg = 118.0, ageMs = 9_000L)
                .writeBody(it)
        }
        assertEquals(
            "an absent axis is omitted, never written as 0 — 0° is a real angle and on this " +
                "airframe it is the resting one",
            """{"p":-30.25,"y":118,"age":9000}""", json,
        )
    }

    @Test
    fun anAgeThatHasNeverArrivedIsOmittedRatherThanZeroed() {
        val json = JsonObject.render {
            LogEntry.Gimbal(0L, -30.0, 0.0, 0.0, ageMs = null).writeBody(it)
        }
        assertFalse(
            "absent means never delivered; an age of 0 is the opposite claim",
            json.contains("age"),
        )
    }
}
