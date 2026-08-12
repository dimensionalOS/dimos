package com.dimensional.mini4pro.gimbal

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * The one implementation of the commanded/reported camera-pitch precedence — see [PitchBelief]'s
 * KDoc for why it exists (the 2026-07-28 RC-wheel sessions, where a camera genuinely at −90°
 * armed nothing because only the command was consulted).
 *
 * Four properties, each of which is a way a camera-pointing consumer has been or could be lied
 * to:
 *
 *  - **commanded outranks reported** — the reported angle lags a moving gimbal, and the command
 *    is this bridge's own exact act;
 *  - **reported is believed when nothing was commanded, and says so** — the provenance flag is
 *    what lets a flight record tell the two beliefs apart;
 *  - **null + null is null** — never zero: a level camera is a claim, and nobody made it;
 *  - **a reported 0.0 is a real value, not an absence** — the zero-substitution failure in the
 *    other direction.
 *
 * Kill counts for this file's mutants are in `GuidedTagDescentTest`'s addendum table, measured
 * across the whole suite: this resolution feeds the arm gate, the fix pipeline and the `tf`
 * edge, so its mutants are killed from several directions and the counts belong where the
 * campaign is.
 */
class PitchBeliefTest {

    @Test
    fun `commanded outranks reported, and the belief says it was not reported`() {
        assertEquals(
            PitchBelief(-90.0, reported = false),
            PitchBelief.of(commandedDeg = -90.0, reportedDeg = -33.0),
        )
    }

    @Test
    fun `with nothing commanded the reported angle is believed, and it says so`() {
        assertEquals(
            PitchBelief(-90.0, reported = true),
            PitchBelief.of(commandedDeg = null, reportedDeg = -90.0),
        )
    }

    @Test
    fun `with neither source there is no belief - null, never a zero`() {
        assertNull(PitchBelief.of(commandedDeg = null, reportedDeg = null))
    }

    @Test
    fun `a reported zero is a real level camera, not an absence`() {
        assertEquals(
            PitchBelief(0.0, reported = true),
            PitchBelief.of(commandedDeg = null, reportedDeg = 0.0),
        )
    }

    @Test
    fun `a commanded zero outranks a reported nadir - the wheel does not erase the ask`() {
        // The honesty gap carried knowingly: the belief is about what this bridge can claim, and
        // a commanded 0.0 with the wheel at -90 is the commanded angle being wrong — which the
        // nadir gates then refuse, rather than this type guessing which source to prefer.
        assertEquals(
            PitchBelief(0.0, reported = false),
            PitchBelief.of(commandedDeg = 0.0, reportedDeg = -90.0),
        )
    }
}
