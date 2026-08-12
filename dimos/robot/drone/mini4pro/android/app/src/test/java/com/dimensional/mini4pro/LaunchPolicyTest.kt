package com.dimensional.mini4pro

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [LaunchPolicy] — whether launching the app also opens a telemetry link to an aircraft.
 *
 * This exists because on 2026-07-27 a bare `am start` on `MainActivity` opened a real flight
 * session and wrote a record. Nothing was flying, so nothing came of it; the same command during
 * a flight attaches a second link to a live aircraft. The decision was buried in `onCreate` where
 * no test could reach it, which is most of why it went unnoticed.
 *
 * The two properties worth breaking, in opposite directions:
 *
 *  - **A tool must be able to launch the Activity without starting anything.** That is the fix.
 *  - **Plugging in the RC must still come up talking.** That is what must not regress, and it is
 *    the more dangerous of the two to lose: the phone's only USB port belongs to the RC, so an
 *    app that needs adb to begin telemetry is useless in the field.
 *
 * Mutation-checked 2026-07-27, one breakage at a time, whole suite each time, reverted after
 * each — measured, not estimated:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the explicit `autostart=false` refusal ignored | 2 |
 *  | a bare launch with a saved host no longer starts (the field path) | 2 |
 *  | absent `autostart` treated as `false` | 2 |
 *  | absent `autostart` treated as `true` alongside a named host | 1 |
 *  | an empty saved host still starts | 1 |
 *
 * Every mutant dies, and the two collapses of the three-valued flag die in *opposite*
 * directions — which is the property that matters, because either one alone looks like a
 * simplification.
 */
class LaunchPolicyTest {

    @Test
    fun `the RC plugged in comes up talking, with no extras at all`() {
        // UsbAttachActivity passes nothing. This is the field path and the reason the
        // start-on-launch branch exists; losing it strands the operator with no telemetry and
        // no way to ask for any, because the USB port is taken by the RC.
        assertTrue(LaunchPolicy.shouldStartBridge(null, "10.55.1.50", null))
    }

    @Test
    fun `a tool can ask for the Activity and nothing else`() {
        // The 19:37 incident: `am start` with no extras, a host in preferences, and a real
        // session opened. A caller now has one flag that means "just the window".
        assertFalse(LaunchPolicy.shouldStartBridge(null, "10.55.1.50", false))
    }

    @Test
    fun `an explicit refusal outranks a named host asking to start`() {
        // Contradictory input, and the safe reading wins: nothing starts. A caller that passes
        // both is confused, and the failure that costs least is the one that does not transmit.
        assertFalse(LaunchPolicy.shouldStartBridge("10.55.1.12", "10.55.1.50", false))
    }

    @Test
    fun `a named host alone is not a request to start`() {
        // `tools/session` sets a host before it is ready to receive; a validator may want its
        // own port bound first. Silence here means "remember this", not "connect now".
        assertFalse(LaunchPolicy.shouldStartBridge("10.55.1.12", "", null))
        assertTrue(LaunchPolicy.shouldStartBridge("10.55.1.12", "", true))
    }

    @Test
    fun `no host anywhere starts nothing`() {
        assertFalse(LaunchPolicy.shouldStartBridge(null, "", null))
        assertFalse(LaunchPolicy.shouldStartBridge(null, "", true))
    }

    @Test
    fun `absent and false are different answers`() {
        // The whole design rests on this distinction. A two-valued flag defaulting to false
        // breaks the field path; defaulting to true gives a tool no way out. Both directions
        // are asserted here so neither collapse compiles quietly.
        assertTrue("absent must not read as false", LaunchPolicy.shouldStartBridge(null, "h", null))
        assertFalse("false must not read as absent", LaunchPolicy.shouldStartBridge(null, "h", false))
    }
}
