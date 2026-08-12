package com.dimensional.mini4pro.record

import io.dronefleet.mavlink.common.DebugVect
import io.dronefleet.mavlink.common.NamedValueFloat
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The GCS mirror.
 *
 * Two things are easy to get wrong and expensive to discover in flight: a name
 * longer than the 10-character MAVLink field (silently truncated, so two values
 * collide under one name in the tlog), and a missing value sent as 0 (a fabricated
 * "hold still" that is indistinguishable from a real one). Both are tested.
 */
class GcsMirrorTest {

    private val engaged = GcsMirror.Sample(
        timeBootMs = 12_345,
        commandedNorth = 0.0, commandedEast = 2.0, commandedDown = 0.0,
        commandedYawRate = 0.0,
        achievedNorth = 0.05, achievedEast = 1.9, achievedDown = -0.1,
        axes = StickAxes(pitch = 0.0, roll = 2.0, yaw = 0.0, verticalThrottle = 0.0),
        modes = StickModes("VELOCITY", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", advanced = true),
        vsEnabled = true, vsAdvanced = true, authority = "MSDK",
        rcLeftHorizontal = 0, rcLeftVertical = 0, rcRightHorizontal = 0, rcRightVertical = 0,
    )

    private fun named(msgs: List<Any>) =
        msgs.filterIsInstance<NamedValueFloat>().associate { it.name() to it.value() }

    private fun vects(msgs: List<Any>) =
        msgs.filterIsInstance<DebugVect>().associateBy { it.name() }

    @Test
    fun `every mirrored name fits the 10-character MAVLink field`() {
        // A longer name is silently truncated on the wire, so two values would land
        // in the tlog under one name and be indistinguishable.
        val names = listOf(
            GcsMirror.NAME_CMD_VEL, GcsMirror.NAME_ACH_VEL, GcsMirror.NAME_V_STICK,
            GcsMirror.NAME_RC_STICK, GcsMirror.NAME_VS_YAW, GcsMirror.NAME_CMD_YAW_RATE,
            GcsMirror.NAME_RC_LEFT_H, GcsMirror.NAME_VS_MODE, GcsMirror.NAME_VS_AUTH,
        )
        for (n in names) assertTrue("$n is ${n.length} chars", n.length <= 10)
        assertEquals("names must be unique", names.size, names.toSet().size)
    }

    @Test
    fun `a cycle carries commanded and achieved velocity as the same-frame pair`() {
        val msgs = GcsMirror.cycle(engaged)
        val v = vects(msgs)
        val cmd = v[GcsMirror.NAME_CMD_VEL]!!
        val ach = v[GcsMirror.NAME_ACH_VEL]!!
        // x=north y=east z=down in both, so the two plot directly against each other.
        assertEquals(0.0f, cmd.x(), 1e-6f)
        assertEquals(2.0f, cmd.y(), 1e-6f)
        assertEquals(0.0f, cmd.z(), 1e-6f)
        assertEquals(0.05f, ach.x(), 1e-6f)
        assertEquals(1.9f, ach.y(), 1e-6f)
        assertEquals(-0.1f, ach.z(), 1e-6f)
    }

    @Test
    fun `the DJI axes we actually sent are mirrored, not just our setpoint`() {
        val stick = vects(GcsMirror.cycle(engaged))[GcsMirror.NAME_V_STICK]!!
        assertEquals(0.0f, stick.x(), 1e-6f)   // pitch
        assertEquals(2.0f, stick.y(), 1e-6f)   // roll
        assertEquals(0.0f, stick.z(), 1e-6f)   // verticalThrottle
        assertEquals(0.0f, named(GcsMirror.cycle(engaged))[GcsMirror.NAME_VS_YAW]!!, 1e-6f)
    }

    @Test
    fun `control modes reach the tlog as a plottable integer`() {
        val code = named(GcsMirror.cycle(engaged))[GcsMirror.NAME_VS_MODE]!!
        assertEquals(1100.0f, code, 1e-6f)
        // The same numbers in ANGLE mode must land on a different code, or the tlog
        // cannot tell a control-mode fault from an axis fault.
        val angle = engaged.copy(modes = engaged.modes!!.copy(rollPitch = "ANGLE"))
        assertEquals(100.0f, named(GcsMirror.cycle(angle))[GcsMirror.NAME_VS_MODE]!!, 1e-6f)
    }

    @Test
    fun `authority code shows engagement and who holds the aircraft`() {
        // 111 = enabled, advanced, MSDK — the only state in which a stick command means anything.
        assertEquals(111, GcsMirror.authorityCode(engaged))
        // The shape of a takeback: still enabled, but the RC has it.
        assertEquals(110, GcsMirror.authorityCode(engaged.copy(authority = "RC")))
        assertEquals(11, GcsMirror.authorityCode(engaged.copy(vsEnabled = false)))
        // Unknown is 9, so it can never be read as a real state.
        // 1 (enabled) * 100 + 9 (advanced unknown) * 10 + 9 (authority unknown) = 199
        assertEquals(199, GcsMirror.authorityCode(engaged.copy(vsAdvanced = null, authority = null)))
    }

    @Test
    fun `a missing value is NaN, never a fabricated zero`() {
        val bare = GcsMirror.Sample(timeBootMs = 1, vsEnabled = true)
        val msgs = GcsMirror.cycle(bare)
        val cmd = vects(msgs)[GcsMirror.NAME_CMD_VEL]!!
        assertTrue("a commanded velocity we do not have must not read as 0", cmd.x().isNaN())
        assertTrue(cmd.y().isNaN())
        assertTrue(cmd.z().isNaN())
        assertTrue(named(msgs)[GcsMirror.NAME_VS_YAW]!!.isNaN())
    }

    @Test
    fun `the mirror is silent unless virtual stick is engaged, and switchable outright`() {
        assertTrue(GcsMirror.cycle(engaged.copy(vsEnabled = false)).isEmpty())
        assertTrue(GcsMirror.cycle(engaged.copy(vsEnabled = null)).isEmpty())
        assertTrue(GcsMirror.cycle(engaged, GcsMirror.MirrorConfig(enabled = false)).isEmpty())
        // ...and can be told to run regardless, for a bench session
        assertTrue(
            GcsMirror.cycle(
                engaged.copy(vsEnabled = false),
                GcsMirror.MirrorConfig(onlyWhenEngaged = false),
            ).isNotEmpty()
        )
    }

    @Test
    fun `a cycle is nine messages and the documented byte cost`() {
        // The bandwidth claim in the docs has to be checkable, not asserted.
        val msgs = GcsMirror.cycle(engaged)
        assertEquals(9, msgs.size)
        assertEquals(4, msgs.count { it is DebugVect })
        assertEquals(5, msgs.count { it is NamedValueFloat })
        // 4 × (42 frame + 28 UDP/IP) + 5 × (30 + 28) = 570 B per cycle on the wire.
        assertEquals(570, GcsMirror.CYCLE_WIRE_BYTES)
    }

    @Test
    fun `an event becomes a STATUSTEXT truncated to the field width`() {
        val short = GcsMirror.statusText(
            LogEntry.Event(0, EventCode.MODE_CHANGE, message = "APAS -> VIRTUAL_STICK")
        )
        assertEquals("mode_change: APAS -> VIRTUAL_STICK", short.text())

        val long = GcsMirror.statusText(
            LogEntry.Event(0, EventCode.VS_AUTHORITY_CHANGE, message = "x".repeat(200))
        )
        assertTrue(long.text().length <= GcsMirror.STATUSTEXT_CHARS)
    }

    @Test
    fun `severity is carried through so QGC colours the message`() {
        assertEquals(
            "MAV_SEVERITY_ERROR",
            GcsMirror.statusText(LogEntry.Event(0, "x", LogEntry.SEV_ERROR)).severity().entry().toString(),
        )
        assertEquals(
            "MAV_SEVERITY_WARNING",
            GcsMirror.statusText(LogEntry.Event(0, "x", LogEntry.SEV_WARN)).severity().entry().toString(),
        )
        assertEquals(
            "MAV_SEVERITY_INFO",
            GcsMirror.statusText(LogEntry.Event(0, "x")).severity().entry().toString(),
        )
    }
}
