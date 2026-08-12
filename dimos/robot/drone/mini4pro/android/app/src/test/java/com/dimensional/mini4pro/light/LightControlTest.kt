package com.dimensional.mini4pro.light

import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.record.DjiCalls
import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.DjiPhase
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Tap
import com.dimensional.mini4pro.vision.TagFix
import com.dimensional.mini4pro.vision.TagSighting.Sighting
import io.dronefleet.mavlink.MavlinkMessage
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [LightControl] and [RecordedLightPort] — the bottom auxiliary lamp.
 *
 * A lamp cannot move an aircraft, so this is the least dangerous actuator in the project and the
 * tests are correspondingly narrow. What they do pin is the part that *is* easy to get wrong: a
 * number off the wire becoming a mode nobody chose.
 *
 * DJI's `AuxiliaryLightMode` has an `UNKNOWN` member. That makes the lenient reading — map an
 * unrecognised `param1` to something the SDK will accept — both possible and silently wrong, and
 * it is the shape of the mistake this project keeps meeting: a value that produces a plausible
 * artifact rather than an error.
 *
 * Mutation-checked 2026-07-27, one at a time, whole suite each run, reverted after each. The
 * harness deletes the results directory before each run, because a mutation that fails to compile
 * otherwise reads the previous run's XML and reports a confident zero — measured, not estimated:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | an unnamed `param1` mapped to AUTO instead of refused | 3 |
 *  | `UNKNOWN` accepted as a commandable mode | 1 |
 *  | an unavailable port commanded anyway | 1 |
 *  | the mode written but not recorded under its own op | 2 |
 *  | DJI's refusal not recorded | 1 |
 *  | the observed mode never updated from the aircraft | 1 |
 */
class LightControlTest {

    private val recorded = ArrayList<LogEntry.DjiCall>()

    /** The real [DjiCalls], as `RecordedPortsTest` does it: assert the lines that reach a file. */
    private val tap = object : Tap {
        private val calls = DjiCalls(sink = { recorded.add(it as LogEntry.DjiCall) }, nowNanos = { 0L })
        override fun gcsOut(datagram: ByteArray) = throw AssertionError("not a GCS path")
        override fun gcsIn(message: MavlinkMessage<*>) = throw AssertionError("not a GCS path")
        override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean): Tap.Call =
            calls.begin(op, argsJson, urgent)

        // Not a vision path. Throwing rather than ignoring, exactly as the two GCS verbs above do:
        // a fake that quietly accepted traffic it was never meant to see would let a wiring mistake
        // pass a test that exists to pin the wiring.
        override fun tagSeen(sighting: Sighting, fix: TagFix?, latched: Boolean) =
            throw AssertionError("not a vision path")
    }

    private class FakePort : LightPort {
        var unavailable: String? = null
        var written: AuxiliaryLight? = null
        var ok: (() -> Unit)? = null
        var fail: ((String) -> Unit)? = null
        var listener: ((AuxiliaryLight?) -> Unit)? = null
        var cancelled = 0

        override fun unavailableReason(): String? = unavailable
        override fun setMode(mode: AuxiliaryLight, onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            written = mode; ok = onSuccess; fail = onFailure
        }
        override fun listenMode(onDelivery: (AuxiliaryLight?) -> Unit) { listener = onDelivery }
        override fun cancelListens() { cancelled++ }
    }

    // ── the wire mapping, which is the only place a number becomes a mode ─────

    @Test
    fun `the four modes are the four numbers, and nothing else is a mode`() {
        assertEquals(AuxiliaryLight.AUTO, AuxiliaryLight.fromParam(0f))
        assertEquals(AuxiliaryLight.ON, AuxiliaryLight.fromParam(1f))
        assertEquals(AuxiliaryLight.OFF, AuxiliaryLight.fromParam(2f))
        assertEquals(AuxiliaryLight.BEACON, AuxiliaryLight.fromParam(3f))
    }

    @Test
    fun `a number that names no mode is refused, never rounded to one`() {
        // DJI's own enum has an UNKNOWN member, so a lenient mapping is available and wrong: it
        // would ask the aircraft for a mode nobody chose. 4 is the near miss that matters.
        for (p in listOf(-1f, 0.5f, 4f, 31010f, Float.NaN, Float.POSITIVE_INFINITY)) {
            assertNull("param1=$p must name no mode", AuxiliaryLight.fromParam(p))
        }
    }

    @Test
    fun `UNKNOWN is something the aircraft says, never something we ask for`() {
        // Unreachable from the wire by construction, and refused again at the control, because it
        // is a member of the enum a Kotlin caller could hand us.
        assertTrue(AuxiliaryLight.entries.contains(AuxiliaryLight.UNKNOWN))
        assertNull(AuxiliaryLight.fromParam(4f))
        val port = FakePort()
        assertEquals(Verdict.DENIED, LightControl(port).setMode(AuxiliaryLight.UNKNOWN))
        assertNull("nothing may reach the aircraft", port.written)
    }

    // ── the control ──────────────────────────────────────────────────────────

    @Test
    fun `a mode is written and the write is what ACCEPTED means`() {
        val port = FakePort()
        val control = LightControl(port)
        assertEquals(Verdict.ACCEPTED, control.setMode(AuxiliaryLight.ON))
        assertEquals(AuxiliaryLight.ON, port.written)
        // ACCEPTED is DJI taking the write, never the lamp being lit. Nothing has reported yet.
        assertNull(control.observed())
    }

    @Test
    fun `an unavailable aircraft is refused without a write`() {
        val port = FakePort().apply { unavailable = "NO_PRODUCT" }
        assertEquals(Verdict.DENIED, LightControl(port).setMode(AuxiliaryLight.ON))
        assertNull(port.written)
    }

    @Test
    fun `the observed mode is the aircraft's, not the one we asked for`() {
        val port = FakePort()
        val control = LightControl(port)
        control.setMode(AuxiliaryLight.ON)
        assertNull("still nothing reported", control.observed())
        // The aircraft disagrees with the command, which is exactly the case worth distinguishing.
        port.listener!!(AuxiliaryLight.OFF)
        assertEquals(AuxiliaryLight.OFF, control.observed())
        port.listener!!(null)
        assertNull("a component-gone null is not a mode", control.observed())
    }

    @Test
    fun `the subscription is planted once, and torn down with the link`() {
        val port = FakePort()
        val control = LightControl(port)
        control.setMode(AuxiliaryLight.ON)
        val first = port.listener
        control.setMode(AuxiliaryLight.OFF)
        assertTrue("re-planting a listener leaks it", first === port.listener)
        control.stop()
        assertEquals(1, port.cancelled)
        assertNull("a stopped control knows nothing", control.observed())
    }

    // ── the recording, which is what makes the night experiment possible ─────

    @Test
    fun `every mode written to the lamp is on the record, with its mode`() {
        val port = FakePort()
        val control = LightControl(RecordedLightPort(port, tap))
        control.setMode(AuxiliaryLight.ON)
        val call = recorded.single { it.op == DjiOp.LIGHT_MODE }
        assertEquals(DjiPhase.ASK, call.phase)
        assertTrue(call.argsJson!!, call.argsJson!!.contains("\"mode\":\"ON\""))
        port.ok!!()
        assertEquals(
            listOf(DjiPhase.ASK, DjiPhase.OK),
            recorded.filter { it.op == DjiOp.LIGHT_MODE }.map { it.phase },
        )
    }

    @Test
    fun `a refusal carries DJI's own error name`() {
        val port = FakePort()
        LightControl(RecordedLightPort(port, tap)).setMode(AuxiliaryLight.ON)
        port.fail!!("SDK_SERVICE_NOT_SUPPORT")
        assertEquals(
            listOf(DjiPhase.ASK, DjiPhase.ERR),
            recorded.filter { it.op == DjiOp.LIGHT_MODE }.map { it.phase },
        )
    }

    @Test
    fun `reading the lamp back is not aircraft-outbound traffic`() {
        // The observed mode belongs in telemetry, not in the call log — the same distinction
        // RecordedGimbalPort draws between a commanded and a reported angle.
        val port = FakePort()
        val recording = RecordedLightPort(port, tap)
        recording.unavailableReason()
        recording.listenMode {}
        recording.cancelListens()
        assertTrue(recorded.isEmpty())
    }
}
