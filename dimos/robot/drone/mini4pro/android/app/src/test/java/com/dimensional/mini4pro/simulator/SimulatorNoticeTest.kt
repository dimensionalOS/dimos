package com.dimensional.mini4pro.simulator

import io.dronefleet.mavlink.common.MavSeverity
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The exact words an operator and a ground station are given.
 *
 * These are pinned rather than reviewed because both channels silently mangle over-long or
 * missing text: `STATUSTEXT.text` is a fixed 50-byte `char[50]` cut on the wire with no error
 * (`command/StatusTexts.kt`), and a banner that returns null shows the *previous* state's line.
 *
 * Mutation-checked 2026-07-26:
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M19 | `WIRE_ACTIVE` lengthened past the 50-byte field | 2 |
 *  | M20 | `WIRE_ACTIVE` and `WIRE_FOREIGN` made the same sentence | 3 |
 *  | M21 | severity downgraded from CRITICAL to INFO | 1 |
 *  | M22 | `banner` returns null for ACTIVE | 2 |
 *  | M23 | `banner` returns non-null for OFF (cries wolf every session) | 1 |
 *  | M24 | ACTIVE and FOREIGN given the same banner colour | 1 |
 *  | M25 | `statusLine` reports an unreadable flag as "off" | 1 |
 *  | M26 | running button labels show the verb instead of the condition | 2 |
 *  | M27 | button stays pressable while a request is in flight | 1 |
 */
class SimulatorNoticeTest {

    private fun utf8(s: String) = s.toByteArray(Charsets.UTF_8).size

    // ------------------------------------------------------------------ the wire

    @Test
    fun `both wire notices fit the STATUSTEXT field`() {
        assertTrue(
            "WIRE_ACTIVE is ${utf8(SimulatorNotice.WIRE_ACTIVE)} bytes",
            utf8(SimulatorNotice.WIRE_ACTIVE) <= SimulatorNotice.MAX_WIRE_BYTES,
        )
        assertTrue(
            "WIRE_FOREIGN is ${utf8(SimulatorNotice.WIRE_FOREIGN)} bytes",
            utf8(SimulatorNotice.WIRE_FOREIGN) <= SimulatorNotice.MAX_WIRE_BYTES,
        )
    }

    /**
     * Counted, so a future edit that lengthens the sentence trips here rather than being cut on
     * the wire into something the operator reads as a different claim.
     */
    @Test
    fun `the wire notices are the exact sentences that were counted`() {
        assertEquals("SIMULATOR ACTIVE - telemetry is not real flight", SimulatorNotice.WIRE_ACTIVE)
        assertEquals("SIMULATOR ON - not started by this bridge", SimulatorNotice.WIRE_FOREIGN)
        assertEquals(47, utf8(SimulatorNotice.WIRE_ACTIVE))
        assertEquals(41, utf8(SimulatorNotice.WIRE_FOREIGN))
    }

    /** Two different situations for the operator, so two different sentences. */
    @Test
    fun `the two wire notices are distinguishable`() {
        assertTrue(SimulatorNotice.WIRE_ACTIVE != SimulatorNotice.WIRE_FOREIGN)
    }

    /**
     * `MAV_SEVERITY_CRITICAL`, matching the emergency-stop notice rather than the routine ones:
     * every number on the operator's screen being fabricated is the most misleading condition
     * this bridge can be in.
     */
    @Test
    fun `the notice goes out as CRITICAL`() {
        val text = SimulatorNotice.statusText(SimulatorNotice.WIRE_ACTIVE)
        assertEquals(MavSeverity.MAV_SEVERITY_CRITICAL, text.severity().entry())
        assertEquals(SimulatorNotice.WIRE_ACTIVE, text.text())
    }

    // ---------------------------------------------------------------- the banner

    @Test
    fun `every running phase gets a banner`() {
        for (phase in listOf(
            SimulatorPhase.ACTIVE,
            SimulatorPhase.FOREIGN,
            SimulatorPhase.STOPPING,
            SimulatorPhase.STARTING,
        )) {
            assertNotNull("$phase must warn", SimulatorNotice.banner(phase))
        }
    }

    /**
     * OFF because a banner that is always there is a banner nobody reads. UNKNOWN because it is
     * the normal state of the first second of every session, and a warning that cries wolf every
     * launch trains the operator to ignore the one that matters — it is shown in the text block
     * instead.
     */
    @Test
    fun `off and unknown do not take over the banner`() {
        assertNull(SimulatorNotice.banner(SimulatorPhase.OFF))
        assertNull(SimulatorNotice.banner(SimulatorPhase.UNKNOWN))
    }

    @Test
    fun `every banner says the word SIMULATOR`() {
        for (phase in SimulatorPhase.entries) {
            val banner = SimulatorNotice.banner(phase) ?: continue
            assertTrue("$phase: ${banner.first}", banner.first.contains("SIMULATOR"))
        }
    }

    /** Colour is the second channel, never the only one — and the two conditions differ. */
    @Test
    fun `a foreign simulator is coloured differently from one we started`() {
        val ours = SimulatorNotice.banner(SimulatorPhase.ACTIVE)!!
        val theirs = SimulatorNotice.banner(SimulatorPhase.FOREIGN)!!
        assertTrue(ours.second != theirs.second)
        assertTrue(ours.first != theirs.first)
    }

    // ------------------------------------------------------------ the status line

    private fun line(phase: SimulatorPhase, activeForMs: Long? = null) =
        SimulatorNotice.statusLine(SimulatorControl.Snapshot(phase, activeForMs, null))

    @Test
    fun `every phase has a status line`() {
        for (phase in SimulatorPhase.entries) {
            assertTrue("$phase", line(phase).isNotBlank())
        }
    }

    /** The distinction the whole design turns on must survive into the words. */
    @Test
    fun `unknown is never rendered as off`() {
        val unknown = line(SimulatorPhase.UNKNOWN)
        assertTrue(unknown, unknown.contains("UNKNOWN"))
        assertTrue(unknown, !unknown.startsWith("off"))
    }

    @Test
    fun `off says DJI reported it`() {
        assertTrue(line(SimulatorPhase.OFF).contains("DJI reports not started"))
    }

    @Test
    fun `foreign names the thing that is wrong`() {
        val foreign = line(SimulatorPhase.FOREIGN)
        assertTrue(foreign, foreign.contains("NOT STARTED BY THIS BRIDGE"))
    }

    @Test
    fun `an active simulator shows how long it has been running`() {
        assertTrue(line(SimulatorPhase.ACTIVE, 92_000).contains("92s"))
    }

    // ------------------------------------------------------------- button label

    /**
     * The button is currently the loudest simulator warning on the phone — `banner()` is written
     * and tested but not yet wired into `MainActivity.renderBanner`, which another workstream
     * owns. So every phase in which a simulator is actually running must *say so* on the control
     * itself, rather than only offering the verb.
     */
    @Test
    fun `every running phase's button states the condition, not just the action`() {
        for (phase in listOf(SimulatorPhase.ACTIVE, SimulatorPhase.FOREIGN, SimulatorPhase.STOPPING)) {
            val (label, _) = SimulatorNotice.buttonLabel(phase)
            assertTrue("$phase: $label", label.contains("SIMULATOR ON"))
        }
    }

    /** A foreign simulator must not be offered as if it were ours. */
    @Test
    fun `the foreign button says it was not started here`() {
        assertTrue(SimulatorNotice.buttonLabel(SimulatorPhase.FOREIGN).first.contains("not started here"))
    }

    /** One press is one action: nothing is pressable while a request is unanswered. */
    @Test
    fun `the button is disabled while a request is in flight`() {
        assertEquals(false, SimulatorNotice.buttonLabel(SimulatorPhase.STARTING).second)
        assertEquals(false, SimulatorNotice.buttonLabel(SimulatorPhase.STOPPING).second)
    }

    @Test
    fun `off and unknown offer a start`() {
        for (phase in listOf(SimulatorPhase.OFF, SimulatorPhase.UNKNOWN)) {
            val (label, enabled) = SimulatorNotice.buttonLabel(phase)
            assertTrue("$phase: $label", label.startsWith("Start simulator"))
            assertTrue("$phase must be pressable", enabled)
        }
    }

    /** An unreadable flag must not be presented as a known-off one. */
    @Test
    fun `the unknown button admits it does not know`() {
        assertTrue(SimulatorNotice.buttonLabel(SimulatorPhase.UNKNOWN).first.contains("unknown"))
    }

    // ------------------------------------------------------------- aircraft line

    @Test
    fun `no simulated aircraft means no aircraft line`() {
        assertNull(SimulatorNotice.aircraftLine(null))
    }

    @Test
    fun `unknown fields read as question marks, never as zero or false`() {
        val text = SimulatorNotice.aircraftLine(SimulatedAircraft())!!
        assertTrue(text, text.contains("motors ?"))
        assertTrue(text, text.contains("xyz=?/?/?"))
    }

    /**
     * DJI documents neither the origin nor the unit of `getPositionX/Y/Z`, so the line must not
     * imply a frame it does not have.
     */
    @Test
    fun `the position triple is labelled undocumented`() {
        val text = SimulatorNotice.aircraftLine(
            SimulatedAircraft(motorsOn = true, flying = true, positionX = 1.0, positionY = 2.0, positionZ = 3.0),
        )!!
        assertTrue(text, text.contains("frame undocumented"))
        assertTrue(text, text.contains("motors spinning"))
        assertTrue(text, text.contains("flying"))
    }
}
