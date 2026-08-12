package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.guided.ControlOrigin
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.handshake.ParamCodec
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Px4Mode
import com.dimensional.mini4pro.telemetry.TelemetryEncoder
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.CommandInt
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavMode
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.ParamRequestList
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.SetMode
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The MAVLink half of M2, driven with the frames QGroundControl 5.0.8 actually sends.
 *
 * Every wire fact asserted here is either measured (the `SET_MODE` custom modes, the 21196
 * force-disarm magic, the 3× ~1.34 s retry burst — all captured on 2026-07-25 and tabulated at
 * `HandshakeResponder.kt:129-182`) or read from QGC's source with the file and line named at the
 * assertion. Nothing is guessed.
 *
 * The suite is written to fail loudly for five specific mistakes, because they are the ones that
 * could hurt someone:
 *
 *  - the interlock defaulting to on, or being reachable from the link
 *  - the retry burst becoming three DJI actions
 *  - a refusal reported to QGC as a success
 *  - a requested mode echoed back into the heartbeat QGC polls
 *  - a command reaching the operator as **nothing at all** — added 2026-07-26 by measurement
 *    (`docs/measurements/2026-07-26-m2-first-command.md`), where a real Return press produced a
 *    `performAction` call, no DJI callback of either kind, and zero `STATUSTEXT` all session
 *
 * Mutation-checked 2026-07-26 for the dispatch announcement. Each breakage was made deliberately,
 * the failing tests counted, and the code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | dispatch announcement removed (`Requested` silent again) | 8 |
 *  | dispatch wording claims the aircraft is complying | 10 |
 *  | dispatch names the wrong action (Return announced as Land) | 7 |
 *  | dispatch announced at `INFO`, below QGC's display threshold | 1 |
 *  | dispatch bypasses `announce`'s window (raw `send` per `SET_MODE`) | **0 — see below** |
 *
 * The last one survived, and it is an **equivalent mutant rather than a hole in the suite**. A
 * duplicate dispatch announcement cannot occur: the action de-duplication drops the second and
 * third `SET_MODE` of a burst before the `Requested` branch is reached, and both windows are the
 * same `ACTION_REPEAT_MS`, so they expire together. A raw `send` emits exactly the same single
 * line. The call stays routed through `announce` because the two windows being one number is a
 * fact about today rather than a guarantee — and the redundancy is only for this text, since the
 * `Refused` and `Unavailable` branches are never stamped and genuinely rely on the window.
 *
 * ## Takeoff, added 2026-07-26 — `docs/decisions/2026-07-26-takeoff.md`
 *
 * Takeoff brings a hazard none of the other buttons have: **its `COMMAND_ACK` is an instruction.**
 * `MAV_RESULT_ACCEPTED` on command 22 makes QGC send `COMPONENT_ARM_DISARM param1=1` on its own
 * initiative (`PX4FirmwarePlugin.cc:307-315`, measured). So that half of the suite is organised
 * around three claims rather than around the code:
 *
 *  1. `ACCEPTED` leaves this class if and only if a takeoff reached `FlightActions`;
 *  2. the operator is told DJI's real 1.2 m beside the height they asked for, *before* the
 *     aircraft moves;
 *  3. `param7` is inverted against **our own published AMSL** and nothing else, so DJI's
 *     pressure-altitude datum cancels instead of leaking tens of metres into a commanded height.
 *
 * Mutation-checked 2026-07-26, same method — each breakage made deliberately, the failing tests
 * counted, the code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | interlock check removed from `onTakeoff` | 3 |
 *  | a DJI refusal answers `ACCEPTED` (QGC arms on a refusal) | 4 |
 *  | an `Unavailable` answers `ACCEPTED` (the simulator gate stops arming QGC) | 2 |
 *  | datum ignored: `param7` read as a height above ground | 6 |
 *  | datum added instead of subtracted | 16 |
 *  | a missing datum treated as zero | 3 |
 *  | non-finite altitude check removed | 1 |
 *  | altitude range check removed | 3 |
 *  | a downward takeoff allowed | 1 |
 *  | height notice removed | 4 |
 *  | height notice swaps the two numbers | 5 |
 *  | height notice sent at `INFO`, below QGC's display threshold | 1 |
 *  | takeoff de-duplication removed (one press, two DJI calls) | 3 |
 *  | takeoff routed to `land()` in the dispatcher's action table | 5 |
 *  | the mode dropdown's `AUTO_TAKEOFF` now takes off | 1 |
 *  | arm explanation removed | 3 |
 *  | arm predicate widened to catch disarm | 4 |
 *  | arm answered `ACCEPTED` | 7 |
 *
 * Two of those first read **0**, and both were holes rather than equivalent mutants, so tests
 * were added rather than the result recorded:
 *
 *  - **`!requested.isFinite()` removed.** Unreachable through the one production caller, because
 *    `TelemetryEncoder.amslMetres` never yields a non-finite datum — but the datum arrives here
 *    through an injected lambda, and this class's guarantee has to hold for what it is *given*.
 *    Closed by a test that supplies `Double.NaN`.
 *  - **DJI's error name prettified with `lowercase()`** (in `MsdkFlightActionsTest`). The only
 *    error that test fired was `-7`, whose lowercase is `-7`. Choosing the measured code had
 *    quietly disabled the assertion. Closed by firing a lettered name as well. Worth remembering
 *    as a *class* of mistake: a fixture that is invariant under the transformation the test
 *    exists to detect passes for the wrong reason.
 *
 * ## The phone's Take off button, added 2026-07-28
 *
 * A second door onto the same corridor (`takeoffFromPhone` → `dispatchTakeoff`), tested in its
 * own section below. Its mutation table — interlock gate dropped, the 10 m constant broken, the
 * corridor bypassed, the camera flag dropped, and the day's believed-pitch work — is measured
 * and recorded in `GuidedTakeoffClimbTest`'s 2026-07-28 addendum, beside the engine half of the
 * same feature.
 */
class CommandDispatcherTest {

    companion object {
        /**
         * `takeoffAltitudeAmsl` as the ground probe read it on 2026-07-25, at the site, on the
         * ground — and the datum QGC had when it composed the measured `param7 = 106.2` for a 3 m
         * takeoff (`HandshakeResponder.kt:131`).
         */
        const val AMSL_GROUND_PROBE = 103.2

        /**
         * The **same site, the same aircraft, the next morning**: 61.7 m.
         *
         * Not a different place and not a bug. `takeoffAltitudeAmsl` is pressure altitude on the
         * 1013.25 hPa reference, so a 5 hPa rise in surface pressure moved it 44 m
         * (`docs/measurements/2026-07-26-amsl-datum.md`, 98 % of the variance explained by
         * independent reanalysis pressure). It is here so the round-trip test can prove the datum
         * cancels rather than merely asserting that it does on one convenient day.
         */
        const val AMSL_DAY_TWO = 61.7
    }

    // ------------------------------------------------------------------- rig

    private val sent = mutableListOf<Any>()
    private var clock = 100_000L

    private val interlock = CommandInterlock(nowMs = { clock })

    /**
     * The AMSL this bridge publishes, as `TelemetryEncoder.amslMetres` would compute it —
     * `takeoffAltitudeAmsl + relativeAltitude`.
     *
     * **103.2 is the real 2026-07-25 ground-probe datum**, and the whole altitude suite is built
     * on the fact that this same number is what QGC added its requested height to before sending
     * `param7`. `AMSL_DAY_TWO` is the same site 44 m lower the next morning, because the value is
     * pressure altitude and the weather moved (`docs/measurements/2026-07-26-amsl-datum.md`).
     */
    private var publishedAmsl: Double? = AMSL_GROUND_PROBE

    /**
     * The takeoff's second phase, or **null — which is the state most of this suite runs in**.
     *
     * Null is not a placeholder: it is the real state of a bridge whose guided engine is not up
     * (before `Bridge.start`, after `Bridge.stop`), and in that state the takeoff is dispatched
     * and announced exactly as it was before 2026-07-27, because with nothing to fly a second
     * phase "DJI goes to 1.2m, not 3.0m" is true again. The whole altitude round-trip section
     * below therefore keeps asserting the old sentence, deliberately, and the two-phase section
     * sets this and asserts the new one.
     */
    private var climbPort: FakeClimb? = null

    private val dispatcher = CommandDispatcher(
        interlock = interlock,
        announcer = Announcer(StatusTextSink { sent.add(it) }),
        publishedAmslM = { publishedAmsl },
        climb = { climbPort },
        light = { lightControl },
        nowMs = { clock },
    )
    /** The lamp, behind a fake port so a test can see what reached the aircraft. */
    private val lightPort = object : com.dimensional.mini4pro.light.LightPort {
        var written: com.dimensional.mini4pro.light.AuxiliaryLight? = null
        override fun unavailableReason(): String? = null
        override fun setMode(
            mode: com.dimensional.mini4pro.light.AuxiliaryLight,
            onSuccess: () -> Unit,
            onFailure: (String) -> Unit,
        ) { written = mode }
        override fun listenMode(onDelivery: (com.dimensional.mini4pro.light.AuxiliaryLight?) -> Unit) = Unit
        override fun cancelListens() = Unit
    }
    private var lightControl: com.dimensional.mini4pro.light.LightControl? =
        com.dimensional.mini4pro.light.LightControl(lightPort)

    private val responder = HandshakeResponder(send = { sent.add(it) }, nowMs = { clock })
        .also { dispatcher.attachTo(it) }

    private val actions = RecordingActions().also { dispatcher.actions = it }

    private val gcsSysId = 255
    private val gcsCompId = 190

    /**
     * A stand-in for the DJI layer. Records what it was asked and answers with whatever the test
     * set — including by throwing, which is what an unwritten implementation does.
     */
    private class RecordingActions : FlightActions {
        val calls = mutableListOf<FlightAction>()
        var outcome: (FlightAction) -> ActionOutcome = { ActionOutcome.Requested }

        override fun takeoff() = record(FlightAction.TAKEOFF)
        override fun returnToHome() = record(FlightAction.RETURN_TO_HOME)
        override fun land() = record(FlightAction.LAND)

        private fun record(action: FlightAction): ActionOutcome {
            calls.add(action)
            return outcome(action)
        }
    }

    /**
     * A stand-in for `GuidedStickEngine`'s [PendingClimb] half. Records what it was asked to arm
     * and to cancel, and answers with whatever the test set — the three [ClimbArm] outcomes are
     * what the operator's sentence is chosen from, so each has a test.
     */
    private class FakeClimb : PendingClimb {
        val armed = mutableListOf<Double>()

        /** The nadir flag per arm, index-matched with [armed] — the takeoff-sequence property. */
        val armedNadir = mutableListOf<Boolean>()

        /**
         * The origin per arm, index-matched with [armed] — the landing08 property: the door must
         * name itself, because the climb's liveness watchdog is evaluated within this label.
         */
        val armedOrigin = mutableListOf<ControlOrigin>()
        val cancelled = mutableListOf<String>()
        var outcome: (Double) -> ClimbArm = { ClimbArm.Armed(it, capped = false) }

        override fun armTakeoffClimb(
            requestedRelAltM: Double,
            aimCameraNadir: Boolean,
            origin: ControlOrigin,
        ): ClimbArm {
            armed += requestedRelAltM
            armedNadir += aimCameraNadir
            armedOrigin += origin
            return outcome(requestedRelAltM)
        }

        override fun cancelTakeoffClimb(reason: String) {
            cancelled += reason
        }
    }

    private val statusTexts: List<Statustext> get() = sent.filterIsInstance<Statustext>()
    private val acks: List<CommandAck> get() = sent.filterIsInstance<CommandAck>()

    // ------------------------------------------------------------- interlock off

    @Test
    fun `with commands off a Return is refused exactly as it was before M2 existed`() {
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals("DJI must not be asked", emptyList<FlightAction>(), actions.calls)
        // The pre-M2 path, byte for byte: HandshakeResponder's own warning, not a second
        // refusal branch owned by the dispatcher.
        assertEquals(HandshakeResponder.MODE_REFUSAL_TEXT, statusTexts.single().text())
        // ERROR, matching the M2 path deliberately: QGroundControl surfaces only
        // EMERGENCY/ALERT/CRITICAL/ERROR (`StatusTextHandler.cc:18-24`), so at WARNING this
        // refusal reached a real operator as nothing at all. Raised in HandshakeResponder, which
        // is why it shows up here — this test asserts the off path *is* that path, so it is
        // supposed to move when that one does.
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.single().severity().entry())
        assertTrue("SET_MODE is never acked", acks.isEmpty())
        // Still recorded. requestedModes is a record of what was asked, which is true whatever
        // anyone does about it.
        assertEquals(Px4Mode.AUTO_RTL, responder.requestedModes.single().customMode)
    }

    @Test
    fun `with commands off a Land is refused the same way`() {
        responder.onMessage(setMode(Px4Mode.AUTO_LAND), gcsSysId, gcsCompId)

        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(HandshakeResponder.MODE_REFUSAL_TEXT, statusTexts.single().text())
        assertTrue(acks.isEmpty())
    }

    @Test
    fun `no inbound MAVLink message can switch commands on`() {
        // We do not authenticate the link at all, so a ground station on a hostile or noisy
        // network must not be able to arm the command path
        // (docs/decisions/2026-07-25-m2-command-safety.md, Q2). This replays the whole inbound
        // surface QGC can produce, plus the parameter-write channel, and asserts the interlock
        // never moves and DJI is never called.
        val everyPx4Button = listOf(
            MavCmd.MAV_CMD_NAV_TAKEOFF,
            MavCmd.MAV_CMD_COMPONENT_ARM_DISARM,
            MavCmd.MAV_CMD_DO_REPOSITION,
            MavCmd.MAV_CMD_DO_CHANGE_SPEED,
            MavCmd.MAV_CMD_DO_ORBIT,
            MavCmd.MAV_CMD_DO_SET_ROI_LOCATION,
            MavCmd.MAV_CMD_DO_SET_ROI_NONE,
            MavCmd.MAV_CMD_DO_SET_HOME,
            MavCmd.MAV_CMD_DO_GO_AROUND,
            MavCmd.MAV_CMD_LOGGING_START,
            MavCmd.MAV_CMD_LOGGING_STOP,
            MavCmd.MAV_CMD_DO_SET_MODE,
            MavCmd.MAV_CMD_NAV_LAND,
            MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            MavCmd.MAV_CMD_MISSION_START,
        )
        everyPx4Button.forEach { command ->
            // Both param polarities, and the force-disarm magic, so no combination is missed.
            listOf(0f to 0f, 1f to 0f, 0f to CommandDispatcher.FORCE_DISARM_MAGIC)
                .forEach { (p1, p2) ->
                    responder.onMessage(
                        CommandLong.builder()
                            .targetSystem(1).targetComponent(1)
                            .command(command).param1(p1).param2(p2).build(),
                        gcsSysId, gcsCompId,
                    )
                }
        }
        everyQgcMode.forEach { (_, customMode) ->
            responder.onMessage(setMode(customMode), gcsSysId, gcsCompId)
        }
        // The parameter channel: a writable parameter would be a MAVLink message that changes
        // our behaviour, which is the thing the interlock exists to forbid. Written with three
        // plausible spellings of "on".
        responder.onMessage(ParamRequestList.builder().targetSystem(1).targetComponent(0).build())
        responder.parameters.snapshot().forEach { parameter ->
            listOf(1f, 0f, 21196f).forEach { value ->
                responder.onMessage(
                    ParamSet.builder()
                        .targetSystem(1).targetComponent(1)
                        .paramId(parameter.name)
                        .paramValue(ParamCodec.encode(parameter.type, value))
                        .paramType(parameter.type)
                        .build(),
                    gcsSysId, gcsCompId,
                )
            }
        }

        assertFalse("the link must not be able to arm the command path", interlock.enabled)
        assertEquals(
            "and DJI must never have been called",
            emptyList<FlightAction>(),
            actions.calls,
        )
    }

    // -------------------------------------------------------------- interlock on

    @Test
    fun `enabling commands lets Return reach DJI, and says so`() {
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.RETURN_TO_HOME), actions.calls)
        // The dispatch is announced. This used to assert silence, on the reasoning that the only
        // honest confirmation is the flight mode in a later heartbeat and a message here would be
        // our own uncorroborated claim. Measurement overrode it
        // (docs/measurements/2026-07-26-m2-first-command.md): DJI accepted this call and then
        // never invoked either callback, the mode never moved, and the operator saw nothing at
        // all for the whole session. The sentence still claims nothing about the aircraft.
        assertEquals("Return sent to DJI", statusTexts.single().text())
        assertTrue("still never an ack for a SET_MODE", acks.isEmpty())
    }

    @Test
    fun `enabling commands lets Land reach DJI, and says so`() {
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_LAND), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.LAND), actions.calls)
        assertEquals("Land sent to DJI", statusTexts.single().text())
        assertTrue(acks.isEmpty())
    }

    @Test
    fun `the dispatch announcement reaches QGC at a severity it will actually show`() {
        // The measured reason this is ERROR and not INFO/NOTICE: QGC 5.0.8's StatusTextHandler
        // surfaces only EMERGENCY/ALERT/CRITICAL/ERROR to the operator
        // (`StatusTextHandler.cc:18-24`) — the same finding that raised MODE_REFUSAL_TEXT and the
        // landing announcement. An announcement below that threshold is one that never happened,
        // which is precisely the bug being fixed. It does abuse the taxonomy; that is a knowing
        // trade, recorded in CommandDispatcher.perform.
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.single().severity().entry())
    }

    @Test
    fun `the dispatch announcement never tells QGC the aircraft is complying`() {
        // The sentence exists only to report what the bridge did. If it ever grew into a claim
        // about the aircraft it would be worse than the silence it replaced, because the operator
        // would then have a positive reason to believe a return is under way.
        interlock.enable()
        listOf(Px4Mode.AUTO_RTL, Px4Mode.AUTO_LAND).forEach { mode ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            responder.onMessage(setMode(mode), gcsSysId, gcsCompId)

            val text = statusTexts.single().text().lowercase()
            listOf("returning", "engaged", "started", "underway", "success", "accepted")
                .forEach { assertFalse("'$text' claims '$it'", text.contains(it)) }
            assertTrue(text, text.contains("sent"))
        }
        // And the heartbeat still comes from the aircraft alone, which is the structural half of
        // the same guarantee.
        val parked = AircraftState(flightMode = "APAS")
        assertEquals(Px4Mode.POSCTL, TelemetryEncoder.heartbeat(parked).customMode())
    }

    @Test
    fun `QGC's three-shot retry burst produces exactly one announcement`() {
        // One press must read as one line. The burst is collapsed twice over — the action de-dup
        // stops the second and third from reaching DJI at all, and `announce`'s identical-text
        // window is the net under that. Three red lines for one click trains an operator to stop
        // reading them, which is what the whole de-duplication rule is for.
        interlock.enable()

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 1_340
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 1_340
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.RETURN_TO_HOME), actions.calls)
        assertEquals("one press, one announcement", 1, statusTexts.size)
        assertEquals("Return sent to DJI", statusTexts.single().text())
    }

    @Test
    fun `a genuine second press past the window is announced again`() {
        // The mirror of the rule above, and the case actually measured on 2026-07-26: the three
        // Returns that session arrived 10.5 s and 15.9 s apart — separate presses, not a burst.
        // Each is a fresh intent and each is owed its own answer.
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 10_500
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 15_900
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(3, actions.calls.size)
        assertEquals("three presses, three answers", 3, statusTexts.size)
        statusTexts.forEach { assertEquals("Return sent to DJI", it.text()) }
    }

    @Test
    fun `Return and Land back to back are announced separately`() {
        // Different text is always a different thing to say, so the announcement window never
        // swallows the second of two different intents.
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 200
        responder.onMessage(setMode(Px4Mode.AUTO_LAND), gcsSysId, gcsCompId)

        assertEquals(
            listOf("Return sent to DJI", "Land sent to DJI"),
            statusTexts.map { it.text() },
        )
    }

    @Test
    fun `a capability refusal from the DJI layer reaches the operator instead of a dispatch`() {
        // MsdkFlightActions answers Unavailable(CANNOT_PERFORM_ACTION) when DJI's own key
        // declaration says the action is not performable. The operator must read that, and must
        // not also read "Return sent to DJI" — nothing was sent.
        interlock.enable()
        actions.outcome = { ActionOutcome.Unavailable("CANNOT_PERFORM_ACTION") }

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        val text = statusTexts.single().text()
        assertEquals("Return failed: CANNOT_PERFORM_ACTION", text)
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.single().severity().entry())
        assertTrue(text, text.toByteArray().size <= 50)
        // Not stamped as a successful request, so pressing again really tries again.
        clock += 100
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(2, actions.calls.size)
    }

    // ----------------------------------------------------- what this does NOT do

    @Test
    fun `QGC's auto-arm chain opens only for a takeoff that actually reached DJI`() {
        // **This test used to assert the opposite**, and the change is the point.
        //
        // Until 2026-07-26 there was no `FlightActions.takeoff()` at all, and the resulting
        // MAV_RESULT_UNSUPPORTED was the only thing stopping PX4FirmwarePlugin::_mavCommandResult
        // from sending COMPONENT_ARM_DISARM param1=1 on QGC's own initiative
        // (`PX4FirmwarePlugin.cc:307-315`, measured). Takeoff now exists
        // (`docs/decisions/2026-07-26-takeoff.md`), so that guarantee could not survive
        // unchanged — the honest replacement is to pin the narrowest form of it that is still
        // true, rather than to delete the test and lose the property.
        //
        // The property: an ACCEPTED on command 22 leaves this class if and only if a takeoff
        // reached FlightActions. Everything else — commands off, no datum, a bad altitude, a DJI
        // refusal, no aircraft, a throw — leaves the chain shut, exactly as before.
        interlock.enable()
        val everyRefusal = listOf<Pair<String, () -> Unit>>(
            "commands off" to { interlock.disable() },
            "no datum" to { interlock.enable(); publishedAmsl = null },
            "bad altitude" to { publishedAmsl = AMSL_GROUND_PROBE; },
            "DJI refused" to { actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") } },
            "no DJI layer" to { dispatcher.actions = null },
        )
        everyRefusal.forEachIndexed { index, (name, arrange) ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            arrange()
            // The "bad altitude" case needs a bad altitude, not just a good datum.
            val param7 = if (name == "bad altitude") 9999f else 106.2f
            responder.onMessage(takeoffCommand(param7 = param7), gcsSysId, gcsCompId)

            assertNotEquals(
                "$index $name must not arm QGC",
                MavResult.MAV_RESULT_ACCEPTED,
                acks.single().result().entry(),
            )
        }
    }

    @Test
    fun `the aircraft cannot be armed or disarmed through this bridge`() {
        // Arm is the second half of QGC's takeoff sequence and stays refused — MSDK 5.18.0 has
        // no arm vocabulary at all (docs/msdk/actions-rth-and-arming.md §6). Plain disarm has no
        // MSDK equivalent either (KeyLockMotors is set-only and pre-flight).
        //
        // What changed on 2026-07-26 is only that the *arm* now explains itself: an operator
        // whose takeoff was just accepted would otherwise read "arm not supported" as "the
        // takeoff failed". The result code is untouched, in both forms.
        interlock.enable()
        listOf(1f to 0f, 0f to 0f).forEach { (p1, p2) ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            responder.onMessage(armDisarm(param1 = p1, param2 = p2), gcsSysId, gcsCompId)

            assertEquals(
                "p1=$p1 p2=$p2",
                MavResult.MAV_RESULT_UNSUPPORTED,
                acks.single().result().entry(),
            )
            assertEquals(emptyList<FlightAction>(), actions.calls)
            if (p1 == 1f) {
                assertEquals("one ack and one explanation", 2, sent.size)
                assertEquals(StatusTexts.NO_SEPARATE_ARM, statusTexts.single().text())
            } else {
                assertEquals("disarm: one ack and nothing else", 1, sent.size)
            }
        }
    }

    @Test
    fun `emergency stop is refused, and the operator is pointed at the control that works`() {
        // MSDK 5.18.0 has no in-flight motor cut at all — no callable action, and the two
        // emergency-stop keys only configure whether the RC's stick gesture is armed
        // (docs/msdk/actions-rth-and-arming.md, canPerformAction false on both). The nearest
        // thing the SDK offers is a several-second landing, and mapping "stop now" onto "start
        // a slow descent" is the most dangerous substitution available in this project.
        interlock.enable()
        responder.onMessage(emergencyStop(), gcsSysId, gcsCompId)

        // Refused, exactly as before M2. No ack anywhere claims the motors were cut.
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertEquals(
            CommandDispatcher.MAV_CMD_COMPONENT_ARM_DISARM,
            acks.single().command().value(),
        )
        assertEquals(emptyList<FlightAction>(), actions.calls)
        // But not silence: seconds matter here, and the RC in the operator's hands can do it.
        val text = statusTexts.single().text()
        assertEquals(CommandDispatcher.EMERGENCY_STOP_TEXT, text)
        assertEquals(MavSeverity.MAV_SEVERITY_CRITICAL, statusTexts.single().severity().entry())
        assertTrue("$text is ${text.toByteArray().size} bytes", text.toByteArray().size <= 50)
    }

    @Test
    fun `the emergency stop answer tells the operator what to do with their hands`() {
        // Measured, not stylistic (docs/measurements/2026-07-26-emergency-stop.md). The text is
        // read to an operator whose aircraft is airborne and who has already decided it must
        // stop — and QGC reads it *aloud*, and our own UNSUPPORTED ack raises a second dialog on
        // top of it ~4 ms later. An earlier version said "use RC stick gesture", which names a
        // category: the act is both sticks held inner-down, and nobody has ever done it in
        // flight. So this pins that the sentence contains an instruction, the same way the
        // dispatch texts are pinned against ever claiming compliance.
        val text = CommandDispatcher.EMERGENCY_STOP_TEXT
        assertTrue("$text must not promise a motor cut", text.startsWith("No motor cut"))
        assertTrue("$text must name the control", text.contains("stick", ignoreCase = true))
        assertTrue(
            "$text must say what to do with them, not just that a gesture exists",
            text.contains("hold", ignoreCase = true) && text.contains("down", ignoreCase = true),
        )
        assertTrue(
            "$text says 'gesture', which is a category and not an act",
            !text.contains("gesture", ignoreCase = true),
        )
        assertTrue("$text is ${text.toByteArray().size} bytes", text.toByteArray().size <= 50)
    }

    @Test
    fun `the emergency stop answer does not depend on the interlock`() {
        // It actuates nothing in either state, so gating it would only withhold the sentence
        // from the operator who most needs it.
        responder.onMessage(emergencyStop(), gcsSysId, gcsCompId)
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertEquals(CommandDispatcher.EMERGENCY_STOP_TEXT, statusTexts.single().text())

        sent.clear()
        clock += CommandDispatcher.ACTION_REPEAT_MS
        interlock.enable()
        responder.onMessage(emergencyStop(), gcsSysId, gcsCompId)
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertEquals(CommandDispatcher.EMERGENCY_STOP_TEXT, statusTexts.single().text())
    }

    @Test
    fun `an emergency stop in COMMAND_INT form is recognised as the same button`() {
        // QGC's PX4 plugin sends this one as COMMAND_LONG, but
        // MAV_PROTOCOL_CAPABILITY_COMMAND_INT is claimed and other clients use the INT form for
        // everything. Recognising it is what gets the operator the RC hint rather than a bare
        // "not supported".
        responder.onMessage(
            CommandInt.builder()
                .targetSystem(1).targetComponent(1)
                .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
                .frame(MavFrame.MAV_FRAME_GLOBAL)
                .param1(0f).param2(CommandDispatcher.FORCE_DISARM_MAGIC)
                .build(),
            gcsSysId, gcsCompId,
        )

        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertEquals(CommandDispatcher.EMERGENCY_STOP_TEXT, statusTexts.single().text())
    }

    @Test
    fun `every mode QGC can ask for that M2 does not implement keeps the pre-M2 refusal`() {
        interlock.enable()
        val notOurs = everyQgcMode.filter {
            it.second != Px4Mode.AUTO_RTL && it.second != Px4Mode.AUTO_LAND
        }
        assertEquals("the sweep must cover more than the two we implement", 6, notOurs.size)

        notOurs.forEach { (name, customMode) ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS // clear the announcement window
            responder.onMessage(setMode(customMode), gcsSysId, gcsCompId)

            assertEquals("$name must not reach DJI", emptyList<FlightAction>(), actions.calls)
            assertEquals(
                "$name",
                HandshakeResponder.MODE_REFUSAL_TEXT,
                statusTexts.single().text(),
            )
            assertTrue("$name", acks.isEmpty())
        }
    }

    // ---------------------------------------------------- one press, one action

    @Test
    fun `QGC's three-shot retry burst is one DJI action`() {
        // Measured: the guided buttons resend the same SET_MODE 3× about 1.34 s apart
        // (HandshakeResponder.kt:175-176). Three DJI calls for one click is a bug with an
        // aircraft attached to it.
        interlock.enable()

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 1_340
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 1_340
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.RETURN_TO_HOME), actions.calls)
        // Every attempt is still recorded — the record is of what QGC sent, not of what we did.
        assertEquals(3, responder.requestedModes.size)
    }

    @Test
    fun `a genuine second press after the window is a second action`() {
        // The window collapses a retry burst; it must not swallow an operator who presses Return
        // again because nothing happened.
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += CommandDispatcher.ACTION_REPEAT_MS
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(
            listOf(FlightAction.RETURN_TO_HOME, FlightAction.RETURN_TO_HOME),
            actions.calls,
        )
    }

    @Test
    fun `de-duplication is per action, not a global gate`() {
        // Land immediately after Return is a different intent, and the second one must go
        // through. A single "last request" timestamp with no action key would swallow it.
        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 200
        responder.onMessage(setMode(Px4Mode.AUTO_LAND), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.RETURN_TO_HOME, FlightAction.LAND), actions.calls)
    }

    @Test
    fun `a repeat after a refusal is tried again, not suppressed`() {
        // The one rule that keeps de-duplication from becoming its own hazard: only a request
        // that actually reached DJI is a duplicate. An operator pressing Return again because
        // the first said FC_AUTH_STATE must not be answered with silence.
        interlock.enable()
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += 500
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertEquals(
            listOf(FlightAction.RETURN_TO_HOME, FlightAction.RETURN_TO_HOME),
            actions.calls,
        )
    }

    // ---------------------------------------------- DJI's own word for a refusal

    @Test
    fun `a DJI refusal reaches the operator naming DJI's error verbatim`() {
        interlock.enable()
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        val text = statusTexts.single().text()
        assertTrue(text, text.contains("FC_AUTH_STATE"))
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.single().severity().entry())
        // Still no ack: SET_MODE has no acknowledgement, and a COMMAND_ACK QGC never queued is
        // discarded ("Ack not in list", MavCommandQueue.cc:489).
        assertTrue(acks.isEmpty())
        assertTrue("50 bytes is the STATUSTEXT field width", text.toByteArray().size <= 50)
    }

    @Test
    fun `each DJI refusal the recorder actually captured survives to the wire intact`() {
        // The 19:16 session on 2026-07-25 saw motorStartFailureError move through these three in
        // order. They are the reason Q3 asks for DJI's word rather than a paraphrase of ours.
        interlock.enable()
        listOf("FC_AUTH_STATE", "GPS_DISCONNECT", "NAV_SYS_EXCEPTION").forEach { error ->
            sent.clear()
            actions.outcome = { ActionOutcome.Refused(error) }
            clock += CommandDispatcher.ACTION_REPEAT_MS
            responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

            val text = statusTexts.single().text()
            assertTrue("$error → $text", text.contains(error))
            assertTrue("$error → $text", text.toByteArray().size <= 50)
        }
    }

    @Test
    fun `the reason usually arrives after the call, and still reaches the operator`() {
        // The realistic MSDK sequence, and the reason reportAsyncDjiError exists: the action
        // returns Requested because CompletionCallback.onFailure has not fired yet, and the
        // *useful* name turns up moments later on a telemetry key we already subscribe to. If
        // the operator only ever heard about synchronous refusals they would hear almost
        // nothing.
        interlock.enable()
        actions.outcome = { ActionOutcome.Requested }
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        // At call time the operator learns only that the request left the bridge — no claim
        // about the aircraft, and no claim that it will work.
        assertEquals("Return sent to DJI", statusTexts.single().text())

        clock += 300
        dispatcher.reportAsyncDjiError("FC_AUTH_STATE")

        val text = statusTexts.last().text()
        assertTrue(text, text.contains("FC_AUTH_STATE"))
        assertEquals(2, statusTexts.size)
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, statusTexts.last().severity().entry())
        assertTrue("and still never an ack for a SET_MODE", acks.isEmpty())
    }

    @Test
    fun `a stuck DJI error does not become a wall of identical warnings`() {
        // A DJI key parked in an error state repeats forever. Three identical red lines for one
        // condition trains an operator to stop reading them, which costs more than the message
        // is worth — the same reasoning as HandshakeResponder.announceModeRefusal.
        dispatcher.reportAsyncDjiError("GPS_DISCONNECT")
        clock += 1_000
        dispatcher.reportAsyncDjiError("GPS_DISCONNECT")
        assertEquals(1, statusTexts.size)

        // A different error is a different thing to say.
        dispatcher.reportAsyncDjiError("FC_AUTH_STATE")
        assertEquals(2, statusTexts.size)

        // And the same one again, later, is news again.
        clock += CommandDispatcher.ACTION_REPEAT_MS
        dispatcher.reportAsyncDjiError("FC_AUTH_STATE")
        assertEquals(3, statusTexts.size)
    }

    @Test
    fun `a nameless DJI error is dropped rather than announced as nothing`() {
        dispatcher.reportAsyncDjiError("")
        dispatcher.reportAsyncDjiError("   ")
        assertTrue(sent.isEmpty())
    }

    // ------------------------------------------ a failure is never a success

    @Test
    fun `no failure is ever reported to QGC as anything but a failure`() {
        // The assertion this whole file exists for. Telling QGC a command succeeded when nothing
        // happened is the one failure mode that can hurt someone. On the SET_MODE path there is
        // no ack to lie in, so the equivalent lie is *silence* — which reads as success, since
        // silence is exactly what a successful request produces. Every failure must therefore
        // produce a STATUSTEXT.
        val failures: List<Pair<String, (FlightAction) -> ActionOutcome>> = listOf(
            "DJI refused" to { _ -> ActionOutcome.Refused("FC_AUTH_STATE") },
            "no aircraft" to { _ -> ActionOutcome.Unavailable("NO_AIRCRAFT_LINK") },
            "threw" to { _ -> throw IllegalStateException("MSDK not registered") },
        )

        interlock.enable()
        failures.forEach { (what, outcome) ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            actions.outcome = outcome
            responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

            assertEquals("$what must tell the operator something", 1, statusTexts.size)
            assertNotEquals(
                "$what must not read as the generic interlock refusal",
                HandshakeResponder.MODE_REFUSAL_TEXT,
                statusTexts.single().text(),
            )
            assertTrue("$what must never be acked", acks.isEmpty())
        }
    }

    @Test
    fun `a DJI layer that throws fails closed and does not escape into the receive thread`() {
        // TODO() and a bare throw are what an unwritten or half-written DJI layer looks like.
        interlock.enable()
        actions.outcome = { throw UnsupportedOperationException("not implemented yet") }

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        assertTrue(acks.isEmpty())
        val text = statusTexts.single().text()
        assertTrue(text, text.contains("Return"))
        assertTrue(text, text.contains("not implemented"))
        // Not stamped as a successful request, so the next press is tried again rather than
        // suppressed as a duplicate.
        clock += 100
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(2, actions.calls.size)
    }

    @Test
    fun `with the interlock on but no DJI layer attached the operator is told, not ignored`() {
        // The window between switching commands on and the MSDK being wired up. Silence here
        // would be indistinguishable from success.
        dispatcher.actions = null
        interlock.enable()

        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        assertEquals(1, statusTexts.size)
        assertTrue(statusTexts.single().text().contains("NO_AIRCRAFT_LINK"))
        assertTrue(acks.isEmpty())
    }

    @Test
    fun `an outcome cannot be a refusal without a reason`() {
        // A refusal the operator cannot act on is barely better than silence, so the type
        // refuses to be constructed that way rather than trusting the implementer.
        listOf("", " ", "\n").forEach { blank ->
            try {
                ActionOutcome.Refused(blank)
                throw AssertionError("a blank DJI error must not be constructible")
            } catch (expected: IllegalArgumentException) {
                assertTrue(expected.message!!.contains("DJI"))
            }
            try {
                ActionOutcome.Unavailable(blank)
                throw AssertionError("a blank reason must not be constructible")
            } catch (expected: IllegalArgumentException) {
                assertTrue(expected.message!!.contains("why"))
            }
        }
    }

    @Test
    fun `every STATUSTEXT this layer can emit fits the 50 byte field`() {
        // Composed at runtime from a DJI string of unknown length, so the limit needs enforcing
        // rather than counting by hand as MODE_REFUSAL_TEXT does.
        interlock.enable()
        val longError = "SOME_ABSURDLY_LONG_DJI_ERROR_CONSTANT_NAME_NOBODY_EXPECTED"
        listOf<(FlightAction) -> ActionOutcome>(
            { ActionOutcome.Refused(longError) },
            { ActionOutcome.Unavailable(longError) },
            { throw IllegalStateException(longError) },
        ).forEach { outcome ->
            actions.outcome = outcome
            listOf(Px4Mode.AUTO_RTL, Px4Mode.AUTO_LAND).forEach { mode ->
                clock += CommandDispatcher.ACTION_REPEAT_MS
                responder.onMessage(setMode(mode), gcsSysId, gcsCompId)
            }
        }
        dispatcher.reportAsyncDjiError(longError)
        clock += CommandDispatcher.ACTION_REPEAT_MS
        responder.onMessage(emergencyStop(), gcsSysId, gcsCompId)

        assertTrue(statusTexts.size >= 7)
        statusTexts.forEach {
            assertTrue(
                "'${it.text()}' is ${it.text().toByteArray().size} bytes",
                it.text().toByteArray().size <= 50,
            )
        }
    }

    // --------------------------------------- the mode must come from the aircraft

    @Test
    fun `a requested mode is never echoed into the heartbeat`() {
        // The field QGC polls to decide whether the change worked is the flight mode in our own
        // heartbeat (_setFlightModeAndValidate, FirmwarePlugin.cc:246-274). Echoing a request
        // into it would report a return-to-home that is not happening — this bridge telling an
        // operator the aircraft is coming home while it hovers.
        //
        // The guarantee is structural: the heartbeat's mode comes from Px4Mode.customMode of
        // what the aircraft said, and the dispatcher has no route to it at all. This test holds
        // that shut from the outside.
        val parked = AircraftState(flightMode = "APAS") // the one measured DJI mode → POSCTL
        val before = TelemetryEncoder.heartbeat(parked).customMode()

        interlock.enable()
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)
        clock += CommandDispatcher.ACTION_REPEAT_MS
        responder.onMessage(setMode(Px4Mode.AUTO_LAND), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.RETURN_TO_HOME, FlightAction.LAND), actions.calls)
        val after = TelemetryEncoder.heartbeat(parked).customMode()
        assertEquals("the aircraft is still parked; so is the heartbeat", before, after)
        assertEquals(Px4Mode.POSCTL, after)
        assertNotEquals("QGC must not be told the return is happening", Px4Mode.AUTO_RTL, after)
        assertNotEquals(Px4Mode.AUTO_LAND, after)
    }

    @Test
    fun `a lying DJI layer still cannot make the heartbeat claim a return`() {
        // ActionOutcome.Requested is the strongest thing an implementation can return, and it is
        // deliberately not enough to move the mode. This is the containment the seam buys: for
        // Return and Land, truthful feedback is structural rather than something anyone has to
        // remember to implement.
        interlock.enable()
        actions.outcome = { ActionOutcome.Requested }
        responder.onMessage(setMode(Px4Mode.AUTO_RTL), gcsSysId, gcsCompId)

        // The aircraft never reported a mode at all.
        val silent = AircraftState(flightMode = null)
        assertEquals(Px4Mode.UNMAPPED, TelemetryEncoder.heartbeat(silent).customMode())
        // And once it really does return, the heartbeat says so — from the aircraft's word.
        val returning = AircraftState(flightMode = "GO_HOME")
        assertEquals(Px4Mode.AUTO_RTL, TelemetryEncoder.heartbeat(returning).customMode())
    }

    @Test
    fun `SET_MODE is never acknowledged, whatever the outcome`() {
        // QGC discards an ack it never queued ("Ack not in list", MavCommandQueue.cc:489), so
        // one here is noise at best and a fabricated reply to an unasked question at worst.
        interlock.enable()
        listOf<(FlightAction) -> ActionOutcome>(
            { ActionOutcome.Requested },
            { ActionOutcome.Refused("FC_AUTH_STATE") },
            { ActionOutcome.Unavailable("NO_AIRCRAFT_LINK") },
            { throw IllegalStateException("boom") },
        ).forEach { outcome ->
            actions.outcome = outcome
            listOf(Px4Mode.AUTO_RTL, Px4Mode.AUTO_LAND, Px4Mode.AUTO_MISSION).forEach { mode ->
                clock += CommandDispatcher.ACTION_REPEAT_MS
                responder.onMessage(setMode(mode), gcsSysId, gcsCompId)
            }
        }
        assertTrue(sent.filterIsInstance<CommandAck>().isEmpty())
    }

    @Test
    fun `a mode change addressed to another vehicle reaches nothing`() {
        interlock.enable()
        responder.onMessage(
            SetMode.builder()
                .targetSystem(2)
                .baseMode(EnumValue.create<MavMode>(1))
                .customMode(Px4Mode.AUTO_RTL)
                .build(),
            gcsSysId, gcsCompId,
        )
        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `the landing-confirmation announcement reaches QGC at a severity it will show`() {
        dispatcher.reportLandingConfirmed()

        val text = statusTexts.single()
        assertEquals(StatusTexts.LANDING_CONFIRMED, text.text())
        // ERROR because QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR
        // (`StatusTextHandler.cc:18-24`): this is the one automatic command the project sends,
        // and an announcement the operator never sees is an announcement that never happened.
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, text.severity().entry())
        assertTrue("nothing is acked for it", acks.isEmpty())
    }

    @Test
    fun `a re-announced landing confirmation inside the window is one line, not a wall`() {
        // The same de-duplication every other announcement gets: if the DJI layer ever
        // announced twice in a burst, the operator reads it once.
        dispatcher.reportLandingConfirmed()
        dispatcher.reportLandingConfirmed()
        assertEquals(1, statusTexts.size)

        clock += CommandDispatcher.ACTION_REPEAT_MS
        dispatcher.reportLandingConfirmed()
        assertEquals("a later landing is fresh news", 2, statusTexts.size)
    }

    // ------------------------------------------------------------------ helpers

    /**
     * Every `SET_MODE` QGroundControl 5.0.8 can send a PX4 vehicle, with the `custom_mode`
     * measured off the wire on 2026-07-25 (`HandshakeResponder.kt:173-182`). Written as literals
     * rather than through `Px4Mode.packed` so a mistake in the packing cannot hide here.
     */
    private val everyQgcMode = listOf(
        "Return" to 0x05_04_0000L,
        "Land" to 0x06_04_0000L,
        "Hold" to 0x03_04_0000L,
        "Mission" to 0x04_04_0000L,
        "Position" to 0x00_03_0000L,
        "Altitude" to 0x00_02_0000L,
        "Acro" to 0x00_05_0000L,
        "Stabilized" to 0x00_07_0000L,
    )

    @Test
    fun `the measured mode numbers are the ones the encoder's table produces`() {
        // Ties the wire capture to Px4Mode, so a change on either side fails here rather than
        // silently routing the Return button to nothing.
        assertEquals(0x05_04_0000L, Px4Mode.AUTO_RTL)
        assertEquals(0x06_04_0000L, Px4Mode.AUTO_LAND)
        assertEquals(400, CommandDispatcher.MAV_CMD_COMPONENT_ARM_DISARM)
        assertEquals(400, EnumValue.of(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM).value())
        assertEquals(21196f, CommandDispatcher.FORCE_DISARM_MAGIC, 0f)
    }

    // ═══════════════════════════════════════════════════════════════════ takeoff
    //
    // The most dangerous inbound message in the project. Three properties are asserted over and
    // over below, and each of them is a way someone gets hurt if it stops holding:
    //
    //   1. MAV_RESULT_ACCEPTED leaves this class if and only if a takeoff reached FlightActions,
    //      because that ack is what makes QGC arm (`PX4FirmwarePlugin.cc:307-315`).
    //   2. The operator is told DJI's real 1.2 m before the aircraft moves, never after.
    //   3. param7 is inverted against *our own published AMSL* and nothing else.

    @Test
    fun `with commands off a Takeoff is refused exactly as it was before takeoff existed`() {
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals("DJI must not be asked", emptyList<FlightAction>(), actions.calls)
        // UNSUPPORTED and not DENIED: docs/decisions/2026-07-25-m2-command-safety.md Q2 says the
        // interlock-off reply is *the reply that existed before the feature*, and this is
        // byte-for-byte what HandshakeResponder's `else` branch produced with nothing registered.
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        // No STATUSTEXT either. QGC raises its own modal for an unsupported command, so unlike
        // the SET_MODE path there is no silence to fill.
        assertTrue(statusTexts.isEmpty())
    }

    @Test
    fun `an accepted takeoff is the only thing that answers ACCEPTED`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(listOf(FlightAction.TAKEOFF), actions.calls)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
    }

    @Test
    fun `exactly one COMMAND_ACK goes out per takeoff`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // HandshakeResponder sends the ack; this class must not send a second one. Two acks for
        // one command id is how a GCS ends up arming twice.
        assertEquals(1, acks.size)
        assertEquals(
            MavCmd.MAV_CMD_NAV_TAKEOFF,
            acks.single().command().entry(),
        )
    }

    @Test
    fun `the operator is told DJI's real height beside the one they asked for`() {
        interlock.enable()

        // The measured frame: QGC offered param7 = 106.2 for a 3 m takeoff from a vehicle
        // reporting 103.2 m AMSL (HandshakeResponder.kt:131).
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        val texts = statusTexts.map { it.text() }
        assertTrue("dispatch must be announced: $texts", texts.contains("Takeoff sent to DJI"))
        // 3.0, recovered from an AMSL that is 103.2 — this is the round trip, end to end.
        assertTrue(
            "the two heights must both be named: $texts",
            texts.contains("Takeoff: DJI goes to 1.2m, not 3.0m"),
        )
    }

    @Test
    fun `the height notice is visible to a QGC operator`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR (`StatusTextHandler.cc:18-24`). A
        // WARNING here is a sentence that is filed and never read, which for this particular
        // sentence means an operator who believes a 3 m takeoff was agreed to.
        val notice = statusTexts.single { it.text().startsWith("Takeoff: DJI") }
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, notice.severity().entry())
    }

    @Test
    fun `nothing is said about height for a takeoff that was refused`() {
        interlock.enable()
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertTrue(
            "a height caveat for a non-event is noise",
            statusTexts.none { it.text().startsWith("Takeoff: DJI") },
        )
    }

    // ─────────────────────────────────────────── the takeoff's second phase (M2.5 → M3)
    //
    // DJI's takeoff carries no altitude and stops at ~1.2 m; QGC cannot ask for less than
    // 3.048 m. Since 2026-07-27 the height the operator asked for is flown *afterwards*, as an
    // ordinary Change Altitude, by whatever is behind `PendingClimb`. What this section pins is
    // the half that lives here: **that arming happens only on the accepted path, and that the
    // sentence the operator reads matches the plan.** Where the climb lives, when it starts and
    // every way it is cancelled are `guided/GuidedTakeoffClimbTest`.

    @Test
    fun `an accepted takeoff arms the climb with the height the operator asked for`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }

        // The measured frame again: param7 = 106.2 against our published 103.2 is a 3 m request.
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(1, climb.armed.size)
        // 1e-4 rather than 1e-9: `param7` is a float on the wire, so the round trip through
        // 106.2f lands 3.05e-6 off 3.0. That is the wire's own resolution and not a bug — it is
        // 3 µm of altitude, against a 1.0 m vertical acceptance.
        assertEquals(3.0, climb.armed.single(), 1e-4)
    }

    @Test
    fun `with a climb armed the operator is told both phases, not DJI's height alone`() {
        interlock.enable()
        climbPort = FakeClimb()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        val texts = statusTexts.map { it.text() }
        assertTrue("both phases must be named: $texts", texts.contains("Takeoff: DJI 1.2m, then we climb to 3.0m"))
        // And the old sentence must be gone: "DJI goes to 1.2m, not 3.0m" says the aircraft will
        // end at 1.2 m, which with a climb armed is false in the other direction. Both going out
        // would be the bridge contradicting itself two lines apart.
        assertFalse("the one-phase sentence is now a lie: $texts", texts.contains("Takeoff: DJI goes to 1.2m, not 3.0m"))
    }

    @Test
    fun `the two-phase notice is visible to a QGC operator`() {
        interlock.enable()
        climbPort = FakeClimb()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR (`StatusTextHandler.cc:18-24`).
        val notice = statusTexts.single { it.text().startsWith("Takeoff: DJI 1.2m") }
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, notice.severity().entry())
    }

    @Test
    fun `a capped climb names the height it will fly and says it was capped`() {
        interlock.enable()
        climbPort = FakeClimb().also { it.outcome = { ClimbArm.Armed(100.0, capped = true) } }

        // 121.92 m — the top of QGC's own takeoff slider, above the 100 m M3 ceiling.
        responder.onMessage(takeoffCommand(param7 = (AMSL_GROUND_PROBE + 121.92).toFloat()), gcsSysId, gcsCompId)

        val texts = statusTexts.map { it.text() }
        assertTrue("the height flown: $texts", texts.contains("Takeoff: DJI 1.2m, then we climb to 100.0m"))
        assertTrue("capped, announced, never silent: $texts", texts.contains("Takeoff climb capped at 100m"))
    }

    @Test
    fun `an uncapped climb says nothing about capping`() {
        interlock.enable()
        climbPort = FakeClimb()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertTrue(statusTexts.none { it.text().startsWith("Takeoff climb capped") })
    }

    @Test
    fun `nothing worth flying falls back to DJI's own height sentence`() {
        interlock.enable()
        climbPort = FakeClimb().also { it.outcome = { ClimbArm.NothingToDo } }

        responder.onMessage(takeoffCommand(param7 = (AMSL_GROUND_PROBE + 1.5).toFloat()), gcsSysId, gcsCompId)

        val texts = statusTexts.map { it.text() }
        assertTrue("the old sentence is true again here: $texts", texts.contains("Takeoff: DJI goes to 1.2m, not 1.5m"))
        assertTrue(texts.none { it.startsWith("Takeoff: DJI 1.2m,") })
    }

    @Test
    fun `a takeoff DJI refused arms nothing`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // The property, and the reason arming sits inside the ACCEPTED branch: a takeoff that did
        // not happen must not leave a climb waiting for an aircraft that never leaves the ground.
        assertEquals(emptyList<Double>(), climb.armed)
    }

    @Test
    fun `a takeoff our own gates refused arms nothing`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }
        publishedAmsl = null

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(emptyList<Double>(), climb.armed)
    }

    @Test
    fun `a takeoff with commands off arms nothing`() {
        val climb = FakeClimb().also { climbPort = it }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(emptyList<Double>(), climb.armed)
    }

    @Test
    fun `an async DJI error cancels a pending climb`() {
        val climb = FakeClimb().also { climbPort = it }

        // `SYSTEM_ERROR` shortly after a landing is a *measured* refusal on this airframe, and it
        // arrives on a channel that is not the takeoff's return path at all.
        dispatcher.reportAsyncDjiError("SYSTEM_ERROR")

        assertEquals(listOf("SYSTEM_ERROR"), climb.cancelled)
    }

    @Test
    fun `an async DJI error with no climb path attached is harmless`() {
        climbPort = null
        dispatcher.reportAsyncDjiError("SYSTEM_ERROR")
        assertTrue(statusTexts.any { it.text().contains("SYSTEM_ERROR") })
    }

    // ─────────────────────────────────────────────── the phone's Take off button
    //
    // The second door onto the takeoff corridor (Ivan, 2026-07-28: take off to 10 m, camera to
    // nadir). The property this section exists for is the shared path: everything past the door
    // — the range gate, the single `perform`, the de-dup window, the climb arming — must be the
    // same code QGC's `MAV_CMD_NAV_TAKEOFF` runs, which is asserted here by watching the two
    // doors interact (one press through each is ONE takeoff) and by the phone press hitting the
    // identical gates.

    @Test
    fun `the phone button with the interlock off is refused and DJI is never asked`() {
        val climb = FakeClimb().also { climbPort = it }

        assertEquals(Verdict.UNSUPPORTED, dispatcher.takeoffFromPhone())

        assertEquals("DJI must not be asked", emptyList<FlightAction>(), actions.calls)
        assertEquals("no climb may be left armed", emptyList<Double>(), climb.armed)
        // No sentence: the screen names the interlock on the refusal, and it is the only reader
        // of this verdict — the same silent-UNSUPPORTED contract as armTagDescent's.
        assertTrue(statusTexts.isEmpty())
    }

    @Test
    fun `the phone button dispatches one takeoff and arms the climb at 10m with the camera flag`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }

        assertEquals(Verdict.ACCEPTED, dispatcher.takeoffFromPhone())

        assertEquals(listOf(FlightAction.TAKEOFF), actions.calls)
        // 10 m exactly — PHONE_TAKEOFF_HEIGHT_M broken is a different flight, and this is the
        // test that goes red for it.
        assertEquals(10.0, climb.armed.single(), 1e-9)
        // The sequence's camera half rides the armed climb (both doors pass true since
        // 2026-07-29 — the QGC door's own test below pins its half).
        assertEquals(listOf(true), climb.armedNadir)
        // The door names itself: a phone climb judged as MAVLINK is landing08's dead climb.
        assertEquals(listOf(ControlOrigin.PHONE), climb.armedOrigin)
        // The operator reads both phases, exactly as a QGC takeoff's operator does.
        assertTrue(statusTexts.map { it.text() }.contains("Takeoff: DJI 1.2m, then we climb to 10.0m"))
    }

    /**
     * The deliberate successor of `a QGC takeoff never asks for the camera - its operator owns
     * it`. That test pinned the generic-GCS reading (false), which stood until 2026-07-29, when
     * the sole operator declared the camera-down sequence wanted on every takeoff regardless of
     * door (Ivan: *"I'm the only operator and prefer camera down"*). Both doors now pass true;
     * the plumbing stays per-door so a future generic-GCS deployment can split them again, and
     * this test is the one that goes red when someone does.
     */
    @Test
    fun `a QGC takeoff also asks for the camera at nadir - the sole-operator decision`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(listOf(true), climb.armedNadir)
    }

    /**
     * The origin, unlike the camera flag, **is** still a door difference and must stay one: the
     * QGC door's climb is that ground station's manoeuvre and keeps the MAVLink heartbeat
     * watchdog byte-for-byte (landing08's fix must not leak the phone's alive-by-identity
     * reading onto the transport door). Origin and camera flag ride the seam together but are
     * independent facts — this pair of asserts is where the independence is pinned: same door,
     * flag true, origin MAVLINK.
     */
    @Test
    fun `a QGC takeoff labels its climb MAVLINK - the heartbeat watchdog stays on the transport door`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(listOf(ControlOrigin.MAVLINK), climb.armedOrigin)
        assertEquals(listOf(true), climb.armedNadir)
    }

    @Test
    fun `the phone door needs no AMSL datum - it asks in our own relative metres`() {
        // The datum exists to invert QGC's AMSL param7; the phone speaks relative height, so a
        // fresh session whose KeyTakeoffLocationAltitude has not arrived yet can still take off
        // from the phone while a QGC takeoff is (correctly) still refused NO_ALT_DATUM.
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }
        publishedAmsl = null

        assertEquals(Verdict.ACCEPTED, dispatcher.takeoffFromPhone())
        assertEquals(listOf(10.0), climb.armed)
    }

    @Test
    fun `the phone door runs the same range gate as QGC's - the corridor does not trust its doors`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }

        assertEquals(Verdict.DENIED, dispatcher.takeoffFromPhone(relAltM = 500.0))
        assertEquals(Verdict.DENIED, dispatcher.takeoffFromPhone(relAltM = 0.0))
        assertEquals(Verdict.DENIED, dispatcher.takeoffFromPhone(relAltM = Double.NaN))

        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(emptyList<Double>(), climb.armed)
        assertTrue(statusTexts.any { it.text().contains("not in 0-122m") || it.text().contains("outside") || it.text().startsWith("Takeoff failed") })
    }

    @Test
    fun `one press through each door is one takeoff - the de-dup window is shared`() {
        // The property that makes "the same implementation" a behaviour rather than a slogan: a
        // phone press milliseconds after a QGC press (or the reverse) is QGC's lost-ack retry
        // case wearing two doors, and DJI must be asked once.
        interlock.enable()
        climbPort = FakeClimb()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        clock += 100
        assertEquals(Verdict.ACCEPTED, dispatcher.takeoffFromPhone())

        assertEquals("one takeoff, not two", listOf(FlightAction.TAKEOFF), actions.calls)
    }

    @Test
    fun `a phone takeoff DJI refused arms no climb and aims no camera`() {
        interlock.enable()
        val climb = FakeClimb().also { climbPort = it }
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }

        assertEquals(Verdict.DENIED, dispatcher.takeoffFromPhone())

        assertEquals(emptyList<Double>(), climb.armed)
        assertEquals(emptyList<Boolean>(), climb.armedNadir)
        assertTrue(statusTexts.any { it.text().contains("FC_AUTH_STATE") })
    }

    @Test
    fun `the phone default is ten metres, inside every bound it meets`() {
        // Ivan's number, and the reason a broken constant cannot hide: above the no-climb floor
        // (the climb always arms, so the camera move always has a handback to ride), under the
        // MAVLink range gate and the M3 ceiling (never refused, never capped).
        assertEquals(10.0, CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, 0.0)
        assertTrue(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M > com.dimensional.mini4pro.guided.TakeoffClimb.NO_CLIMB_BELOW_M)
        assertTrue(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M < CommandDispatcher.MAX_TAKEOFF_HEIGHT_M)
        assertTrue(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M <= com.dimensional.mini4pro.guided.GuidedEnvelope.CEILING_M)
    }

    // ───────────────────────────────────────────────────── the AMSL round trip

    @Test
    fun `the datum cancels, so the same 3m takeoff survives a 44m weather shift`() {
        interlock.enable()

        // Day one: site datum 103.2, QGC composes 103.2 + 3 = 106.2.
        publishedAmsl = AMSL_GROUND_PROBE
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        val dayOne = statusTexts.single { it.text().startsWith("Takeoff: DJI") }.text()

        // Day two, same spot, same aircraft, 5 hPa of extra pressure: DJI's "AMSL" now reads
        // 61.7, so QGC composes 61.7 + 3 = 64.7 for the identical operator request. If we
        // subtracted anything other than our own published datum — a survey, a terrain lookup,
        // the site's real ~89.5 m — these two would disagree by tens of metres.
        sent.clear()
        clock += CommandDispatcher.ACTION_REPEAT_MS
        publishedAmsl = AMSL_DAY_TWO
        responder.onMessage(takeoffCommand(param7 = 64.7f), gcsSysId, gcsCompId)
        val dayTwo = statusTexts.single { it.text().startsWith("Takeoff: DJI") }.text()

        assertEquals("Takeoff: DJI goes to 1.2m, not 3.0m", dayOne)
        assertEquals(dayOne, dayTwo)
    }

    @Test
    fun `the datum is subtracted, not added or ignored`() {
        interlock.enable()

        // 30 m above a 103.2 m datum. The three mutations this kills all produce a different
        // number here: ignoring the datum reads 133.2 m, adding it reads 236.4 m (and is refused
        // as out of range), and inverting the sign reads -73.2 m.
        responder.onMessage(takeoffCommand(param7 = 133.2f), gcsSysId, gcsCompId)

        assertEquals(
            "Takeoff: DJI goes to 1.2m, not 30.0m",
            statusTexts.single { it.text().startsWith("Takeoff: DJI") }.text(),
        )
    }

    @Test
    fun `a takeoff with no published AMSL is refused rather than guessed at`() {
        interlock.enable()
        publishedAmsl = null

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // Reading param7 as a height above ground would make this a 106 m takeoff, and assuming
        // a datum of zero is the same mistake spelled differently.
        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals(
            "Takeoff failed: NO_ALT_DATUM",
            statusTexts.single().text(),
        )
    }

    @Test
    fun `a non-finite commanded altitude is refused instead of propagating as NaN`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = Float.NaN), gcsSysId, gcsCompId)

        // Every comparison against NaN is false, so without an explicit test this would sail
        // through the range guard and be announced to the operator as "not NaNm".
        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals("Takeoff failed: ALT_NOT_A_NUMBER", statusTexts.single().text())
    }

    @Test
    fun `a non-finite datum is refused, even though the encoder cannot produce one`() {
        interlock.enable()
        publishedAmsl = Double.NaN

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // `TelemetryEncoder.amslMetres` filters non-finite values, so in the running bridge this
        // cannot happen — and that is exactly why it is tested. The datum arrives here through an
        // injected lambda, so this class's guarantee has to hold for whatever it is given rather
        // than for what today's single caller happens to pass. A `NaN` datum makes a finite
        // `param7` produce a `NaN` height, which every subsequent comparison waves through.
        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals("Takeoff failed: ALT_NOT_A_NUMBER", statusTexts.single().text())
    }

    @Test
    fun `a takeoff that does not go up is refused`() {
        interlock.enable()

        // param7 below our own reported AMSL. No correct caller sends this, and resolving it into
        // DJI's 1.2 m climb would turn "descend" into "ascend".
        responder.onMessage(takeoffCommand(param7 = 100.0f), gcsSysId, gcsCompId)

        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        // The sentence names the *cause* rather than the arithmetic. A negative result can only
        // mean the GCS composed param7 against a datum below ours, and on 2026-07-28 in real air
        // the old wording — "Takeoff failed: -49.5m not in 0-121m" — was true and unusable.
        assertEquals("Takeoff failed: GCS datum 3.2m below ours", statusTexts.single().text())
    }

    @Test
    fun `an altitude from a foreign datum is refused, not converted`() {
        interlock.enable()

        // The failure `docs/measurements/2026-07-26-amsl-datum.md` demands be refused: a param7
        // built on some other AMSL. Here it is a plan saved on a day when the pressure datum was
        // 44 m higher, replayed against today's — the arithmetic "works" and yields a 300 m
        // takeoff, which is exactly the shape of the mistake we cannot otherwise detect.
        responder.onMessage(takeoffCommand(param7 = 403.2f), gcsSysId, gcsCompId)

        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals("Takeoff failed: 300.0m not in 0-121m", statusTexts.single().text())
    }

    @Test
    fun `the whole of QGC's takeoff slider range is accepted`() {
        interlock.enable()

        // PX4 does not override minimumTakeoffAltitudeMeters, so QGC's slider runs from
        // FirmwarePlugin.h:204's 3.048 m to FlyView.SettingsGroup.json's 121.92 m default
        // maximum. A band that refused part of it would refuse takeoffs a stock QGC can send.
        listOf(3.048, 121.92).forEach { requested ->
            sent.clear()
            actions.calls.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            responder.onMessage(
                takeoffCommand(param7 = (AMSL_GROUND_PROBE + requested).toFloat()),
                gcsSysId, gcsCompId,
            )
            assertEquals(
                "$requested m must be accepted",
                MavResult.MAV_RESULT_ACCEPTED,
                acks.single().result().entry(),
            )
            assertEquals(listOf(FlightAction.TAKEOFF), actions.calls)
        }
    }

    // ─────────────────────────────────────────────── refusals from the DJI half

    @Test
    fun `a takeoff DJI refuses is DENIED with DJI's word verbatim`() {
        interlock.enable()
        // -7 is the real code this airframe produces (DJI #783).
        actions.outcome = { ActionOutcome.Refused("-7") }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals("Takeoff refused by DJI: -7", statusTexts.single().text())
    }

    @Test
    fun `the simulator refusal reaches the operator as its own word`() {
        interlock.enable()
        actions.outcome = { ActionOutcome.Unavailable(MsdkFlightActions.SIMULATOR_REQUIRED) }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        // "failed", not "refused by DJI": no flight controller was consulted, and the word tells
        // the operator exactly what would make the command work.
        assertEquals("Takeoff failed: SIMULATOR_REQUIRED", statusTexts.single().text())
    }

    @Test
    fun `a takeoff with no DJI layer attached is DENIED, never accepted`() {
        interlock.enable()
        dispatcher.actions = null

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
        assertEquals("Takeoff failed: NO_AIRCRAFT_LINK", statusTexts.single().text())
    }

    @Test
    fun `a throwing DJI layer is FAILED, never accepted`() {
        interlock.enable()
        actions.outcome = { throw IllegalStateException("takeoff not written yet") }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // An unwritten or half-written implementation must not become an arm command.
        assertEquals(MavResult.MAV_RESULT_FAILED, acks.single().result().entry())
        assertEquals("Takeoff failed: takeoff not written yet", statusTexts.single().text())
    }

    @Test
    fun `no refusal path ever answers ACCEPTED`() {
        interlock.enable()
        val refusals = listOf<() -> Unit>(
            { publishedAmsl = null },
            { publishedAmsl = AMSL_GROUND_PROBE; actions.outcome = { ActionOutcome.Refused("X") } },
            {
                actions.outcome = { ActionOutcome.Unavailable("NO_PRODUCT") }
            },
            { actions.outcome = { throw RuntimeException("boom") } },
        )
        refusals.forEachIndexed { index, arrange ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            arrange()
            responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
            assertNotEquals(
                "refusal $index must not arm QGC",
                MavResult.MAV_RESULT_ACCEPTED,
                acks.single().result().entry(),
            )
        }
    }

    // ───────────────────────────────────────────────────────── one press, one action

    @Test
    fun `a duplicated takeoff is answered but DJI is asked only once`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        clock += 200 // a retry after our ack was lost on UDP
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        assertEquals("one press, one takeoff", listOf(FlightAction.TAKEOFF), actions.calls)
        // Still answered, and answered the same way: the retry exists because QGC did not hear
        // the first ack, and leaving it unanswered would strand the command in QGC's queue.
        assertEquals(2, acks.size)
        acks.forEach { assertEquals(MavResult.MAV_RESULT_ACCEPTED, it.result().entry()) }
    }

    @Test
    fun `a takeoff after the repeat window reaches DJI again`() {
        interlock.enable()

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        clock += CommandDispatcher.ACTION_REPEAT_MS
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // Measured on 2026-07-26: DJI accepts commands it never enacts. A deliberate second press
        // after nothing happened must reach the aircraft.
        assertEquals(
            listOf(FlightAction.TAKEOFF, FlightAction.TAKEOFF),
            actions.calls,
        )
    }

    @Test
    fun `a refused takeoff does not suppress the next attempt`() {
        interlock.enable()
        actions.outcome = { ActionOutcome.Refused("FC_AUTH_STATE") }

        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        clock += 200
        actions.outcome = { ActionOutcome.Requested }
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)

        // Only Requested stamps the window, so an operator retrying after "FC_AUTH_STATE" is not
        // answered with silence.
        assertEquals(
            listOf(FlightAction.TAKEOFF, FlightAction.TAKEOFF),
            actions.calls,
        )
    }

    // ═════════════════════════════════════════════════ takeoff's other half: the arm

    @Test
    fun `the arm QGC sends after an accepted takeoff is refused and explained`() {
        interlock.enable()

        // The measured sequence: our ACCEPTED, then QGC's own COMPONENT_ARM_DISARM param1=1
        // milliseconds later (`PX4FirmwarePlugin.cc:307-315`).
        responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
        responder.onMessage(armDisarm(param1 = 1f, param2 = 0f), gcsSysId, gcsCompId)

        // Refused, because MSDK 5.18.0 has no arm at all. Accepting would be a claim that motors
        // were commanded by something that commanded nothing.
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.last().result().entry())
        // And explained, because "arm not supported" one second after a successful takeoff reads
        // as "the takeoff failed" and would make an operator act on that.
        assertEquals(
            StatusTexts.NO_SEPARATE_ARM,
            statusTexts.last().text(),
        )
    }

    @Test
    fun `a bare arm press is answered the same way, in every interlock state`() {
        // Unconditional, exactly like the emergency-stop notice: it is information rather than
        // actuation and it is equally true whether commands are switched on or off. A sentence
        // that is true always but said only sometimes is a sentence an operator cannot rely on.
        listOf(false, true).forEach { commandsOn ->
            sent.clear()
            clock += CommandDispatcher.ACTION_REPEAT_MS
            interlock.set(commandsOn)

            responder.onMessage(armDisarm(param1 = 1f, param2 = 0f), gcsSysId, gcsCompId)

            assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
            assertEquals(StatusTexts.NO_SEPARATE_ARM, statusTexts.single().text())
            assertEquals("nothing may reach DJI", emptyList<FlightAction>(), actions.calls)
        }
    }

    @Test
    fun `emergency stop keeps its own sentence and does not get the arm one`() {
        // param1 = 0 with the force magic. If the arm branch ever widened to catch it, an
        // operator pressing Emergency Stop would be told about takeoff instead of being told
        // where the only real motor cut is.
        responder.onMessage(
            armDisarm(param1 = 0f, param2 = CommandDispatcher.FORCE_DISARM_MAGIC),
            gcsSysId, gcsCompId,
        )

        assertEquals(CommandDispatcher.EMERGENCY_STOP_TEXT, statusTexts.single().text())
        assertEquals(MavSeverity.MAV_SEVERITY_CRITICAL, statusTexts.single().severity().entry())
    }

    @Test
    fun `a plain disarm stays silent`() {
        responder.onMessage(armDisarm(param1 = 0f, param2 = 0f), gcsSysId, gcsCompId)

        // Out of scope and refused, and silence about it is acceptable where silence about an
        // emergency stop is not.
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertTrue(statusTexts.isEmpty())
    }

    // ═════════════════════════════════════════ the routes that are still refused

    @Test
    fun `the mode dropdown's Takeoff is still refused, with no DJI call`() {
        interlock.enable()

        responder.onMessage(setMode(Px4Mode.AUTO_TAKEOFF), gcsSysId, gcsCompId)

        // Only QGC's Takeoff *button* is implemented. A SET_MODE carries no altitude to invert
        // against our datum and no acknowledgement to refuse through, so a second inbound route
        // to the most dangerous action here would have none of the first one's safeguards.
        assertEquals(emptyList<FlightAction>(), actions.calls)
        assertEquals(HandshakeResponder.MODE_REFUSAL_TEXT, statusTexts.single().text())
        assertTrue("SET_MODE is never acked", acks.isEmpty())
    }

    @Test
    fun `a takeoff command cannot switch the interlock on`() {
        // The interlock is unreachable from the link by construction, and a command that arms
        // the aircraft's command path from a network we do not authenticate is the specific
        // thing docs/decisions/2026-07-25-m2-command-safety.md Q2 forbids.
        repeat(3) {
            clock += CommandDispatcher.ACTION_REPEAT_MS
            responder.onMessage(takeoffCommand(param7 = 106.2f), gcsSysId, gcsCompId)
        }

        assertFalse(interlock.enabled)
        assertEquals(emptyList<FlightAction>(), actions.calls)
    }

    /**
     * `MAV_CMD_NAV_TAKEOFF` as QGC 5.0.8 sends it — `param1 = -1` (no pitch), `param2/3 = 0`,
     * `param4/5/6` NaN, `param7` the AMSL altitude. Measured, `HandshakeResponder.kt:131`.
     *
     * The NaNs are carried faithfully rather than zeroed: they are the frame that reached us on
     * the wire, including the `p1=NaN` an operator saw logged when takeoff was still unsupported.
     */
    private fun takeoffCommand(param7: Float) =
        CommandLong.builder()
            .targetSystem(1).targetComponent(1)
            .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
            .confirmation(0)
            .param1(Float.NaN)
            .param2(Float.NaN)
            .param3(0f)
            .param4(Float.NaN)
            .param5(Float.NaN)
            .param6(Float.NaN)
            .param7(param7)
            .build()

    private fun lightCommand(param1: Float) =
        CommandLong.builder()
            .targetSystem(1).targetComponent(1)
            .command(EnumValue.create<MavCmd>(31010)) // MAV_CMD_USER_1
            .confirmation(0)
            .param1(param1)
            .param2(0f).param3(0f).param4(0f).param5(0f).param6(0f).param7(0f)
            .build()

    private fun setMode(customMode: Long) =
        SetMode.builder()
            .targetSystem(1) // SET_MODE has no target_component field
            .baseMode(EnumValue.create<MavMode>(1)) // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            .customMode(customMode)
            .build()

    private fun armDisarm(param1: Float, param2: Float) =
        CommandLong.builder()
            .targetSystem(1).targetComponent(1)
            .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
            .confirmation(0)
            .param1(param1).param2(param2)
            .build()

    /** Measured: QGC's Emergency Stop is ARM_DISARM with `param1 = 0, param2 = 21196`. */
    private fun emergencyStop() =
        armDisarm(param1 = 0f, param2 = CommandDispatcher.FORCE_DISARM_MAGIC)

    // ---------------------------------------------------------------- the lamp (MAV_CMD_USER_1)

    /**
     * **The lamp is not behind the interlock**, and that is the whole point of these two tests.
     *
     * The interlock gates the commands that move an aircraft. A lamp cannot move one, and the
     * interlock starts off every session — which is exactly when an operator setting up after dark
     * wants the light. Gating it would make the feature unavailable at the only moment it is for.
     *
     * The argument is `gimbal/GimbalManager`'s, already settled for the camera, applied to a lamp.
     */
    @Test
    fun `the lamp answers with the interlock off, because a lamp cannot move an aircraft`() {
        assertFalse(interlock.enabled)

        responder.onMessage(lightCommand(param1 = 1f), gcsSysId, gcsCompId)

        assertEquals(com.dimensional.mini4pro.light.AuxiliaryLight.ON, lightPort.written)
        assertEquals(MavResult.MAV_RESULT_ACCEPTED, acks.single().result().entry())
        assertTrue("a lamp is not a flight action", actions.calls.isEmpty())
    }

    @Test
    fun `a param1 that names no mode is refused, and nothing reaches the aircraft`() {
        // 4 is the near miss that matters: DJI's own enum has an UNKNOWN member at that end, so a
        // lenient mapping would ask the aircraft for a mode nobody chose.
        responder.onMessage(lightCommand(param1 = 4f), gcsSysId, gcsCompId)

        assertNull(lightPort.written)
        assertEquals(MavResult.MAV_RESULT_DENIED, acks.single().result().entry())
    }

    @Test
    fun `the lamp is unsupported when no link is up, rather than silently accepted`() {
        lightControl = null

        responder.onMessage(lightCommand(param1 = 1f), gcsSysId, gcsCompId)

        assertNull(lightPort.written)
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
    }
}
