package com.dimensional.mini4pro.simulator

import com.dimensional.mini4pro.command.CommandInterlock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The simulator's grip on the command interlock, as pure logic — no `Activity`, no aircraft.
 *
 * The change this pins removes two steps of bookkeeping and closes one hazard. The bookkeeping
 * was Ivan's complaint: *"I need to enable this interlock thing, and then QGC has control, and
 * then I need to turn on the SIM"* — three acts where one would do. The hazard is the one that
 * makes the change worth testing at all: **until this existed the interlock survived the
 * simulator**, so a simulation that quietly went away left commands live against a real
 * aircraft, with the arming switch still green and the operator with no reason to look at it.
 *
 * That is not hypothetical. DJI delivers `KeyIsSimulatorStarted` as **null** when the component
 * goes — observed in this project's own logs — and `SimulatorControl` correctly refuses to read
 * null as "stopped" while still leaving [SimulatorPhase.ACTIVE]. Under the old arrangement that
 * delivery changed nothing at all.
 *
 * Written to fail loudly for:
 *
 *  - **the interlock surviving the simulator**, in any of the ways a simulator can end —
 *    stopped, foreign, unknown, or DJI's null delivery. This is the mutation that matters and
 *    it is asserted from every non-ACTIVE phase rather than from one.
 *  - **arming on our own request rather than DJI's confirmation** — `STARTING` is us asking and
 *    claims nothing
 *  - **the confirmation dialog surviving into a confirmed simulator** (the friction Ivan asked
 *    to remove), or **disappearing outside one** (the safety question, which must not)
 *  - **the dialog being decided from a remembered flag** rather than from the live phase, which
 *    is what breaks the two awkward orderings
 *  - a `FOREIGN` simulator — one we cannot explain — being treated as reassurance
 *  - the drop happening silently
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time against the shipped source, run,
 * confirmed red, reverted. Counts are failing tests in this suite plus `CommandInterlockTest` —
 * measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | **leaving ACTIVE yields `NONE` (the interlock survives the simulator)** | 4 |
 *  | `DISABLE` returned only for `OFF`, not for every non-ACTIVE phase | 3 |
 *  | `ENABLE` on entering `STARTING` as well as `ACTIVE` | 4 |
 *  | `ENABLE` re-fired every tick while already ACTIVE | 1 |
 *  | first-ever ACTIVE treated as `NONE` because `previous` was null | 2 |
 *  | `interlockLine` silent while the simulator holds the interlock | 1 |
 *
 * Three rows were removed on 2026-07-30, all of them `confirmationRequired` mutants, when that
 * function and its four tests were deleted with the arming dialog. Their counts are not restated
 * here because a kill count for code that no longer exists is not a measurement of anything — the
 * remaining rows are unchanged and were not re-measured, since nothing they mutate was touched.
 *
 * The first row is the one the whole change is traded against — it restores the exact behaviour
 * that shipped before this, so a 4 there is the measurement that says the hazard cannot come
 * back quietly. The second row is its subtler cousin: a `DISABLE` that only fires on a *clean*
 * stop misses DJI's null delivery, which is the way this actually happens.
 */
class SimulatorInterlockTest {

    private val notActive = SimulatorPhase.entries.filter { it != SimulatorPhase.ACTIVE }

    // ── the edge detector ────────────────────────────────────────────────────

    @Test
    fun `DJI confirming our simulator arms the command path`() {
        assertEquals(
            SimulatorInterlock.Effect.ENABLE,
            SimulatorInterlock.effectOf(SimulatorPhase.STARTING, SimulatorPhase.ACTIVE),
        )
    }

    @Test
    fun `a first-ever ACTIVE arms, even with no previous phase observed`() {
        assertEquals(
            SimulatorInterlock.Effect.ENABLE,
            SimulatorInterlock.effectOf(null, SimulatorPhase.ACTIVE),
        )
    }

    @Test
    fun `our own request arms nothing — only DJI's confirmation does`() {
        // STARTING is "we asked". The whole safety story rests on not believing our own request.
        for (from in notActive) {
            assertEquals(
                "$from → STARTING must not arm",
                SimulatorInterlock.Effect.NONE,
                SimulatorInterlock.effectOf(from, SimulatorPhase.STARTING),
            )
        }
    }

    @Test
    fun `losing the confirmed simulator disarms, whichever way it is lost`() {
        // The hazard, from every direction it can arrive: a clean stop, a foreign takeover, and
        // DJI's null delivery — which lands in STARTING or UNKNOWN depending on what was
        // outstanding, and must disarm from either.
        for (to in notActive) {
            assertEquals(
                "ACTIVE → $to must switch commands off",
                SimulatorInterlock.Effect.DISABLE,
                SimulatorInterlock.effectOf(SimulatorPhase.ACTIVE, to),
            )
        }
    }

    @Test
    fun `staying ACTIVE does not re-arm on every tick`() {
        assertEquals(
            SimulatorInterlock.Effect.NONE,
            SimulatorInterlock.effectOf(SimulatorPhase.ACTIVE, SimulatorPhase.ACTIVE),
        )
    }

    @Test
    fun `movement between two non-simulator phases means nothing to the interlock`() {
        for (from in notActive) {
            for (to in notActive) {
                assertEquals(
                    "$from → $to must not touch the interlock",
                    SimulatorInterlock.Effect.NONE,
                    SimulatorInterlock.effectOf(from, to),
                )
            }
        }
    }

    // ── the whole sequence, against the real CommandInterlock ────────────────

    /** Drives the rule exactly as the Activity does, so the sequence is the tested unit. */
    private class Screen {
        val interlock = CommandInterlock()
        private var last: SimulatorPhase? = null
        var announcements = 0
            private set

        fun observe(phase: SimulatorPhase) {
            val effect = SimulatorInterlock.effectOf(last, phase)
            last = phase
            when (effect) {
                SimulatorInterlock.Effect.NONE -> Unit
                SimulatorInterlock.Effect.ENABLE -> interlock.enable()
                SimulatorInterlock.Effect.DISABLE -> {
                    interlock.disable()
                    announcements++
                }
            }
        }
    }

    @Test
    fun `the ordinary bench sequence needs one act from the operator, not three`() {
        val s = Screen()
        s.observe(SimulatorPhase.OFF)
        assertFalse("nothing armed by merely looking", s.interlock.enabled)
        s.observe(SimulatorPhase.STARTING)
        assertFalse("our request is not DJI's confirmation", s.interlock.enabled)
        s.observe(SimulatorPhase.ACTIVE)
        assertTrue("a confirmed simulator arms the command path", s.interlock.enabled)
        s.observe(SimulatorPhase.STOPPING)
        assertFalse("and loses it the moment the confirmation does", s.interlock.enabled)
        assertEquals("said out loud, once", 1, s.announcements)
    }

    @Test
    fun `DJI's null delivery takes the interlock with it`() {
        // The measured case: `KeyIsSimulatorStarted` arrives null, `SimulatorControl` keeps our
        // outstanding request and the phase leaves ACTIVE. Under the old arrangement this
        // changed nothing and left commands live against a real aircraft.
        val s = Screen()
        s.observe(SimulatorPhase.ACTIVE)
        assertTrue(s.interlock.enabled)
        s.observe(SimulatorPhase.UNKNOWN)
        assertFalse("a simulator we can no longer see is not a simulator", s.interlock.enabled)
    }

    @Test
    fun `an interlock armed by hand before the simulator still drops with it`() {
        // The awkward ordering. Disarming can only make things safer, so the drop does not ask
        // who armed it — and the operator who wants it back has one switch to press.
        val s = Screen()
        s.observe(SimulatorPhase.OFF)
        s.interlock.enable()
        assertTrue(s.interlock.enabled)
        s.observe(SimulatorPhase.ACTIVE)
        assertTrue(s.interlock.enabled)
        s.observe(SimulatorPhase.OFF)
        assertFalse(s.interlock.enabled)
    }

    @Test
    fun `an interlock armed by hand with no simulator anywhere is left alone`() {
        val s = Screen()
        s.observe(SimulatorPhase.UNKNOWN)
        s.interlock.enable()
        s.observe(SimulatorPhase.OFF)
        s.observe(SimulatorPhase.UNKNOWN)
        s.observe(SimulatorPhase.OFF)
        assertTrue("only a *confirmed* simulator going away disarms", s.interlock.enabled)
        assertEquals(0, s.announcements)
    }

    @Test
    fun `a foreign simulator neither arms nor is treated as ours`() {
        val s = Screen()
        s.observe(SimulatorPhase.FOREIGN)
        assertFalse("a simulator nothing here started arms nothing", s.interlock.enabled)
    }

    // ── the arming dialog: deleted 2026-07-30, along with the rule that skipped it ───────────
    //
    // Four tests stood here, pinning `confirmationRequired`. The dialog they were ultimately about
    // is gone (see `CockpitDefaults.COMMAND_INTERLOCK`), which left that function reachable only
    // from its own tests — dead code held up by the tests that tested it, which is the arrangement
    // that makes a suite's green mean less than it looks. Deleting the function without deleting
    // these would have been the same mistake in the other direction.
    //
    // Nothing they guarded is unguarded now: what they were protecting was a *shortcut* past a
    // question, and there is no question left to shortcut. The simulator's actual safety property —
    // leaving ACTIVE disarms the interlock — is the edge-detector section above, untouched, and
    // still the row with the highest kill count in the table.

    // ── what the operator is told ────────────────────────────────────────────

    @Test
    fun `a command path held by the simulator says so`() {
        assertEquals(
            "held by the simulator",
            SimulatorInterlock.interlockLine(SimulatorPhase.ACTIVE, interlockEnabled = true),
        )
    }

    @Test
    fun `an active simulator with commands off says that too`() {
        assertNotNull(SimulatorInterlock.interlockLine(SimulatorPhase.ACTIVE, interlockEnabled = false))
    }

    @Test
    fun `with no simulator the interlock line is none of the simulator's business`() {
        for (phase in notActive) {
            assertNull(SimulatorInterlock.interlockLine(phase, true))
            assertNull(SimulatorInterlock.interlockLine(phase, false))
        }
    }

    @Test
    fun `the drop has a sentence`() {
        assertTrue(SimulatorInterlock.DROPPED.isNotBlank())
        assertTrue(SimulatorInterlock.DROPPED.contains("commands", ignoreCase = true))
    }
}
