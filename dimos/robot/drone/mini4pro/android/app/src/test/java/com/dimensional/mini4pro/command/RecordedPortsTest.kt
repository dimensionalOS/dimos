package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.gimbal.GimbalAngles
import com.dimensional.mini4pro.gimbal.GimbalLimits
import com.dimensional.mini4pro.gimbal.GimbalPort
import com.dimensional.mini4pro.gimbal.RecordedGimbalPort
import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.DjiPhase
import com.dimensional.mini4pro.record.DjiCalls
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Tap
import com.dimensional.mini4pro.vision.TagFix
import com.dimensional.mini4pro.vision.TagSighting.Sighting
import com.dimensional.mini4pro.simulator.RecordedSimulatorPort
import com.dimensional.mini4pro.simulator.SimulatedAircraft
import com.dimensional.mini4pro.simulator.SimulatorPort
import io.dronefleet.mavlink.MavlinkMessage
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The two recording port decorators: **every ask and every answer on the way past, and nothing
 * else changed.**
 *
 * These are the classes that make aircraft-outbound recording structural rather than remembered.
 * They are also the classes that sit directly in the path of a takeoff, a landing and a
 * confirm-landing, so the property that matters most here is the *negative* one: a decorator must
 * not swallow a callback, must not reorder one, must not delay one, and must not let a recorder
 * fault reach the caller.
 *
 * `MsdkFlightActions` and `SimulatorControl` are untouched by this change and their own suites
 * still pass fakes directly. What is new is that a fake can now be *wrapped*, which is how the
 * assertions below get at what was recorded without an aircraft.
 *
 * Mutation counts for the two action/simulator decorators are in `record/RecordingSeamTest`'s KDoc
 * table with the rest of the seam's.
 *
 * ## `RecordedGimbalPort`, mutation-checked 2026-07-27
 *
 * One breakage at a time, whole suite each time, reverted after each — measured, not estimated:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the rotation recorded under another op — `GIMBAL_ROTATE` never written | 4 |
 *  | DJI's acceptance is not recorded | 1 |
 *  | DJI's refusal is not recorded | 1 |
 *  | the caller's success callback is swallowed | 1 |
 *  | the caller's failure callback is swallowed | 1 |
 *  | a null axis recorded as `0.0` (the #527 distinction lost) | 1 |
 *  | a non-finite angle written as bare `NaN` | 1 |
 *
 * **A note on how the first row was measured, because it was measured wrong first.** The initial
 * attempt introduced an unresolved symbol; the build failed, no tests ran, and the harness read
 * the *previous* run's XML still sitting on disk and reported a confident **0**. The same shape as
 * the truncated-log pull earlier the same day: a failure that parses as a clean result. The
 * harness now deletes the results directory before each run and reports `NO-RUN` when nothing is
 * written, and the mutation was rewritten as one that compiles.
 */
class RecordedPortsTest {

    // ─────────────────────────────────────────────────────────────────────────
    // a tap backed by the real correlation engine, so what is asserted is what ships
    // ─────────────────────────────────────────────────────────────────────────

    private val recorded = ArrayList<LogEntry.DjiCall>()
    private var nanos = 0L

    /**
     * The real [DjiCalls], not a stub. A test double for the correlation engine would assert that
     * the decorator called *something*; this asserts the lines that actually reach a file.
     */
    private val tap = object : Tap {
        private val calls = DjiCalls(
            sink = { recorded.add(it as LogEntry.DjiCall) },
            nowNanos = { nanos },
        )

        override fun gcsOut(datagram: ByteArray) = throw AssertionError("not a GCS path")
        override fun gcsIn(message: MavlinkMessage<*>) = throw AssertionError("not a GCS path")
        override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean): Tap.Call =
            calls.begin(op, argsJson, urgent)

        // Not a vision path, and throwing rather than ignoring for the same reason the two GCS
        // verbs above do: a fake that silently accepted traffic it was never meant to see would let
        // a wiring mistake pass the test that exists to pin the wiring.
        override fun tagSeen(sighting: Sighting, fix: TagFix?, latched: Boolean) =
            throw AssertionError("not a vision path")
    }

    private fun phasesOf(op: String) = recorded.filter { it.op == op }.map { it.phase }

    // ─────────────────────────────────────────────────────────────────────────
    // fakes
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * An `ActionPort` that hands its callbacks back so a test can fire them when it likes.
     *
     * `open` so one test can override `startTakeoff` to assert what is already on the record at
     * the instant the SDK is touched — the ordering property is not observable any other way.
     */
    private open class FakeActionPort : ActionPort {
        var goHomeFail: ((String) -> Unit)? = null
        var takeoffFail: ((String) -> Unit)? = null
        var landOk: (() -> Unit)? = null
        var landFail: ((String) -> Unit)? = null
        var stopLandFail: ((String) -> Unit)? = null
        var confirmOk: (() -> Unit)? = null
        var confirmFail: ((String) -> Unit)? = null
        var cancelListensThrows: Throwable? = null
        var cancelled = 0
        var reasonReads = 0

        override fun unavailableReason(): String? { reasonReads++; return null }
        override fun canStartGoHome() = true
        override fun canStartAutoLanding() = true
        override fun canStopAutoLanding() = true
        override fun canStartTakeoff() = true
        override fun startGoHome(onFailure: (String) -> Unit) { goHomeFail = onFailure }
        override fun startTakeoff(onFailure: (String) -> Unit) { takeoffFail = onFailure }
        override fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            landOk = onSuccess; landFail = onFailure
        }
        override fun stopAutoLanding(onFailure: (String) -> Unit) { stopLandFail = onFailure }
        override fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            confirmOk = onSuccess; confirmFail = onFailure
        }
        override fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit) = Unit
        override fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit) = Unit
        override fun cancelListens() {
            cancelled++
            cancelListensThrows?.let { throw it }
        }
    }

    private class FakeSimulatorPort : SimulatorPort {
        var startArgs: Triple<Double, Double, Int>? = null
        var startOk: (() -> Unit)? = null
        var startFail: ((String) -> Unit)? = null
        var stopOk: (() -> Unit)? = null
        var stopFail: ((String) -> Unit)? = null

        override fun unavailableReason(): String? = null
        override fun start(
            latitude: Double, longitude: Double, satelliteCount: Int,
            onSuccess: () -> Unit, onFailure: (String) -> Unit,
        ) {
            startArgs = Triple(latitude, longitude, satelliteCount)
            startOk = onSuccess; startFail = onFailure
        }
        override fun stop(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            stopOk = onSuccess; stopFail = onFailure
        }
        override fun listenIsSimulatorStarted(onDelivery: (Boolean?) -> Unit) = Unit
        override fun listenSimulatorState(onDelivery: (SimulatedAircraft?) -> Unit) = Unit
        override fun cancelListens() = Unit
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the ask is on the record before the SDK is touched
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The ordering that makes a `performAction` which *throws* still leave evidence — and, with
     * `DjiCalls.sweep`, makes a `performAction` which is silently swallowed leave evidence too.
     */
    @Test
    fun theAskIsRecordedBeforeTheSdkCallIsMade() {
        val inner = object : FakeActionPort() {
            override fun startTakeoff(onFailure: (String) -> Unit) {
                assertEquals(
                    "the ask must already be on the record when the SDK is touched",
                    listOf(DjiPhase.ASK), phasesOf(DjiOp.TAKEOFF),
                )
                throw IllegalStateException("MSDK exploded")
            }
        }
        val port = RecordedActionPort(inner, tap)
        try {
            port.startTakeoff {}
        } catch (expected: IllegalStateException) {
            // The SDK's throw is the SDK's; the decorator does not catch it, because swallowing a
            // failure to command an aircraft is not the recorder's decision to make.
        }
        assertEquals(listOf(DjiPhase.ASK), phasesOf(DjiOp.TAKEOFF))
    }

    @Test
    fun everyCommandingActionIsRecordedUnderItsOwnOp() {
        val inner = FakeActionPort()
        val port = RecordedActionPort(inner, tap)
        port.startTakeoff {}
        port.startGoHome {}
        port.startAutoLanding({}, {})
        port.stopAutoLanding {}
        port.confirmLanding({}, {})

        assertEquals(
            listOf(DjiOp.TAKEOFF, DjiOp.GO_HOME, DjiOp.LAND, DjiOp.STOP_LANDING, DjiOp.CONFIRM_LANDING),
            recorded.map { it.op },
        )
        assertTrue(
            "everything that can move an aircraft reaches the flash immediately",
            recorded.all { it.urgent },
        )
    }

    /**
     * Reading a precondition is not traffic. `unavailableReason` is called before every action and
     * `canStart*` on most of them; recording those would triple the line count of the one channel
     * whose whole virtue is that it is cheap and discrete.
     */
    @Test
    fun preconditionReadsAreNotRecorded() {
        val inner = FakeActionPort()
        val port = RecordedActionPort(inner, tap)
        repeat(5) {
            port.unavailableReason()
            port.canStartTakeoff()
            port.canStartGoHome()
            port.canStartAutoLanding()
        }
        assertTrue(recorded.isEmpty())
        assertEquals("and they still reach the real port", 5, inner.reasonReads)
    }

    @Test
    fun theListensAreNotRecordedBecauseTheyAreInboundState() {
        val port = RecordedActionPort(FakeActionPort(), tap)
        port.listenIsLandingConfirmationNeeded {}
        port.listenIsInLandingMode {}
        assertTrue(recorded.isEmpty())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the answer, both kinds
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun anAcceptedLandingIsRecordedAndStillReachesTheCaller() {
        val inner = FakeActionPort()
        val port = RecordedActionPort(inner, tap)
        var told = 0
        port.startAutoLanding(onSuccess = { told++ }, onFailure = { fail() })
        nanos += 25_000_000L
        inner.landOk!!()

        assertEquals(listOf(DjiPhase.ASK, DjiPhase.OK), phasesOf(DjiOp.LAND))
        assertEquals(25L, recorded[1].elapsedMs)
        assertEquals("the caller's callback must fire exactly once", 1, told)
    }

    /**
     * **The three refusal names that have each sent someone down a wrong path in a single week.**
     * The decorator must carry DJI's word through untouched to both the record and the caller —
     * the caller turns it into the operator's `STATUSTEXT`, and the two must agree.
     */
    @Test
    fun arefusalIsRecordedVerbatimAndStillReachesTheCaller() {
        val cases = listOf(
            DjiOp.TAKEOFF to "CONTROL_AUTH_HAS_NO_CONTROL_AUTH",
            DjiOp.GO_HOME to "REQUEST_HANDLER_NOT_FOUND",
            DjiOp.LAND to "SYSTEM_ERROR",
            DjiOp.CONFIRM_LANDING to "SYSTEM_ERROR",
        )
        for ((op, error) in cases) {
            recorded.clear()
            val inner = FakeActionPort()
            val port = RecordedActionPort(inner, tap)
            val heard = ArrayList<String>()
            when (op) {
                DjiOp.TAKEOFF -> { port.startTakeoff { heard.add(it) }; inner.takeoffFail!!(error) }
                DjiOp.GO_HOME -> { port.startGoHome { heard.add(it) }; inner.goHomeFail!!(error) }
                DjiOp.LAND -> { port.startAutoLanding({}, { heard.add(it) }); inner.landFail!!(error) }
                else -> { port.confirmLanding({}, { heard.add(it) }); inner.confirmFail!!(error) }
            }
            assertEquals(listOf(DjiPhase.ASK, DjiPhase.ERR), phasesOf(op))
            assertEquals(error, recorded[1].error)
            assertEquals(
                "the operator's sentence and the record must be built from the same string",
                listOf(error), heard,
            )
        }
    }

    /**
     * `startTakeoff` deliberately has no success callback (`ActionPort.startTakeoff`), so an
     * accepted takeoff and a swallowed one are indistinguishable *at this seam*. The record can
     * still tell them apart, because an unanswered ask is swept into a `none` — which is the first
     * time this path has been able to state the difference at all.
     */
    @Test
    fun anUnansweredTakeoffBecomesALineRatherThanAnAbsence() {
        val calls = DjiCalls(
            sink = { recorded.add(it as LogEntry.DjiCall) },
            nowNanos = { nanos },
        )
        val sweepingTap = object : Tap {
            override fun gcsOut(datagram: ByteArray) = Unit
            override fun gcsIn(message: MavlinkMessage<*>) = Unit
            override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean) =
                calls.begin(op, argsJson, urgent)
            override fun tagSeen(sighting: Sighting, fix: TagFix?, latched: Boolean) = Unit
        }
        RecordedActionPort(FakeActionPort(), sweepingTap).startTakeoff {}
        nanos += 6_000_000_000L
        calls.sweep()

        assertEquals(listOf(DjiPhase.ASK, DjiPhase.NONE), phasesOf(DjiOp.TAKEOFF))
        assertTrue(recorded[1].urgent)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // recorder faults never reach the caller
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **The non-negotiable.** A recorder that throws must not refuse a landing.
     *
     * Containment lives in `DjiCalls` — once — rather than in each decorator, so this asserts the
     * whole chain under a sink that fails on every write: the command still reaches the port, the
     * caller's callbacks still fire, and nothing propagates.
     */
    @Test
    fun arecorderThatThrowsOnEveryWriteNeverBreaksACommand() {
        val burning = object : Tap {
            private val calls = DjiCalls(
                sink = { throw IllegalStateException("disk is on fire") },
                nowNanos = { throw IllegalStateException("no clock either") },
            )
            override fun gcsOut(datagram: ByteArray) = Unit
            override fun gcsIn(message: MavlinkMessage<*>) = Unit
            override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean) =
                calls.begin(op, argsJson, urgent)
            override fun tagSeen(sighting: Sighting, fix: TagFix?, latched: Boolean) = Unit
        }
        val inner = FakeActionPort()
        val port = RecordedActionPort(inner, burning)

        var successes = 0
        var failures = 0
        port.startAutoLanding({ successes++ }, { failures++ })
        inner.landOk!!()
        port.confirmLanding({ successes++ }, { failures++ })
        inner.confirmFail!!("SYSTEM_ERROR")
        port.startTakeoff { failures++ }
        inner.takeoffFail!!("SYSTEM_ERROR")

        assertEquals("the accepted landing still reached the caller", 1, successes)
        assertEquals("both refusals still reached the caller", 2, failures)
        assertTrue("and not one line was recorded, which is the point", recorded.isEmpty())
    }

    /**
     * `cancelListens` is synchronous and void, so a clean return is the only acceptance signal
     * there is. A throw from it is re-raised — teardown failing is the caller's business — but it
     * is recorded first, which is the only order in which the evidence survives.
     */
    @Test
    fun asynchronousTeardownIsRecordedInBothDirections() {
        val inner = FakeActionPort()
        RecordedActionPort(inner, tap).cancelListens()
        assertEquals(listOf(DjiPhase.ASK, DjiPhase.SYNC), phasesOf(DjiOp.CANCEL_LISTENS))
        assertNull(recorded[1].error)
        assertFalse("teardown is not worth an fsync when it works", recorded[1].urgent)
        assertEquals(1, inner.cancelled)

        recorded.clear()
        val angry = FakeActionPort().apply { cancelListensThrows = IllegalStateException("no holder") }
        try {
            RecordedActionPort(angry, tap).cancelListens()
            fail()
        } catch (expected: IllegalStateException) {
            // re-raised on purpose
        }
        assertEquals(listOf(DjiPhase.ASK, DjiPhase.SYNC), phasesOf(DjiOp.CANCEL_LISTENS))
        assertEquals("IllegalStateException: no holder", recorded[1].error)
        assertTrue("a teardown that threw is a failure and is urgent", recorded[1].urgent)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the simulator port
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The origin a simulated flight is anchored to is on the record.
     *
     * A simulator started at the wrong coordinates produces a plausible, entirely wrong flight,
     * and until now the number that would prove it existed only in a logcat line that is gone by
     * the time anyone asks.
     */
    @Test
    fun theSimulatorsOriginAndSatelliteCountAreOnTheRecord() {
        val inner = FakeSimulatorPort()
        val port = RecordedSimulatorPort(inner, tap)
        port.start(37.9838, 23.7275, 14, {}, {})

        assertEquals(listOf(DjiPhase.ASK), phasesOf(DjiOp.SIM_START))
        assertEquals("""{"lat":37.9838,"lon":23.7275,"sats":14}""", recorded[0].argsJson)
        assertEquals(
            "the arguments must reach DJI unaltered — clamping belongs above the seam",
            Triple(37.9838, 23.7275, 14), inner.startArgs,
        )
    }

    @Test
    fun asimulatorRefusalIsRecordedAndStillReachesTheCaller() {
        val inner = FakeSimulatorPort()
        val port = RecordedSimulatorPort(inner, tap)
        var heard: String? = null
        port.start(0.0, 0.0, 10, {}, { heard = it })
        inner.startFail!!("SYSTEM_ERROR")

        assertEquals(listOf(DjiPhase.ASK, DjiPhase.ERR), phasesOf(DjiOp.SIM_START))
        assertEquals("SYSTEM_ERROR", recorded[1].error)
        assertEquals("SYSTEM_ERROR", heard)
    }

    @Test
    fun asimulatorStopIsRecordedInBothDirections() {
        val inner = FakeSimulatorPort()
        val port = RecordedSimulatorPort(inner, tap)
        var told = 0
        port.stop({ told++ }, {})
        inner.stopOk!!()
        assertEquals(listOf(DjiPhase.ASK, DjiPhase.OK), phasesOf(DjiOp.SIM_STOP))
        assertEquals(1, told)
    }

    @Test
    fun thesimulatorsListensAndPreconditionAreNotRecorded() {
        val port = RecordedSimulatorPort(FakeSimulatorPort(), tap)
        port.unavailableReason()
        port.listenIsSimulatorStarted {}
        port.listenSimulatorState {}
        port.cancelListens()
        assertTrue(recorded.isEmpty())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the gimbal, whose commanded angle no record carried at all until now
    // ─────────────────────────────────────────────────────────────────────────

    private class FakeGimbalPort : GimbalPort {
        var lastArgs: Array<Any?>? = null
        var ok: (() -> Unit)? = null
        var fail: ((String) -> Unit)? = null

        override fun unavailableReason(): String? = null
        override fun canRotateByAngle() = true
        override fun rotateByAngle(
            absolute: Boolean,
            pitchDeg: Double?,
            rollDeg: Double?,
            yawDeg: Double?,
            durationS: Double,
            onSuccess: () -> Unit,
            onFailure: (String) -> Unit,
        ) {
            lastArgs = arrayOf(absolute, pitchDeg, rollDeg, yawDeg, durationS)
            ok = onSuccess
            fail = onFailure
        }

        override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) = Unit
        override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) = Unit
        override fun listenWorkMode(onDelivery: (String?) -> Unit) = Unit
        override fun listenConnection(onDelivery: (Boolean?) -> Unit) = Unit
        override fun cancelListens() = Unit
    }

    @Test
    fun agimbalRotationIsRecordedInBothDirections() {
        val inner = FakeGimbalPort()
        val port = RecordedGimbalPort(inner, tap)
        var told = 0
        port.rotateByAngle(true, -90.0, null, null, 0.5, { told++ }, { fail() })
        assertEquals(listOf(DjiPhase.ASK), phasesOf(DjiOp.GIMBAL_ROTATE))
        inner.ok!!()
        assertEquals(listOf(DjiPhase.ASK, DjiPhase.OK), phasesOf(DjiOp.GIMBAL_ROTATE))
        assertEquals(1, told)
    }

    @Test
    fun agimbalRefusalCarriesDjisOwnErrorName() {
        val inner = FakeGimbalPort()
        val port = RecordedGimbalPort(inner, tap)
        var got: String? = null
        port.rotateByAngle(true, -90.0, null, null, 0.5, { fail() }, { got = it })
        inner.fail!!("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW")
        assertEquals(listOf(DjiPhase.ASK, DjiPhase.ERR), phasesOf(DjiOp.GIMBAL_ROTATE))
        assertEquals("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW", got)
    }

    @Test
    fun thecommandedAngleIsOnTheRecordWithEveryAxis() {
        // The recorded arguments are the whole point of this decorator: `tools/memexport` builds
        // the drone/base_link -> drone/camera edge from them, and before this existed it had
        // nothing to build from but the reported angle, which goes silent when the camera holds
        // still. A null axis is meaningful and must survive as `null` — "we asked for no yaw" and
        // "we asked for yaw 0" are the two sides of DJI issue #527.
        val port = RecordedGimbalPort(FakeGimbalPort(), tap)
        port.rotateByAngle(true, -90.0, null, null, 0.5, {}, {})
        val args = recorded.single { it.op == DjiOp.GIMBAL_ROTATE }.argsJson!!
        assertTrue(args, args.contains("\"absolute\":true"))
        assertTrue(args, args.contains("\"pitchDeg\":-90.0"))
        assertTrue(args, args.contains("\"rollDeg\":null"))
        assertTrue(args, args.contains("\"yawDeg\":null"))
        assertTrue(args, args.contains("\"durationS\":0.5"))
    }

    @Test
    fun anonFiniteAngleIsRecordedAsNullRatherThanBareNaN() {
        // Unreachable through MsdkGimbalAim, which refuses a non-finite pitch before the port.
        // Kept because this is the layer that would serialise one, and a bare NaN is not JSON:
        // it would make the whole record unparseable rather than one line wrong.
        val port = RecordedGimbalPort(FakeGimbalPort(), tap)
        port.rotateByAngle(true, Double.NaN, null, null, 0.5, {}, {})
        val args = recorded.single { it.op == DjiOp.GIMBAL_ROTATE }.argsJson!!
        assertFalse(args, args.contains("NaN"))
        assertTrue(args, args.contains("\"pitchDeg\":null"))
    }

    @Test
    fun thegimbalsListensAndPreconditionsAreNotRecorded() {
        val port = RecordedGimbalPort(FakeGimbalPort(), tap)
        port.unavailableReason()
        port.canRotateByAngle()
        port.listenAttitude {}
        port.listenAttitudeRange {}
        port.listenWorkMode {}
        port.listenConnection {}
        port.cancelListens()
        assertTrue(recorded.isEmpty())
    }

    private fun fail(): Nothing = throw AssertionError("must not be reached")
}
