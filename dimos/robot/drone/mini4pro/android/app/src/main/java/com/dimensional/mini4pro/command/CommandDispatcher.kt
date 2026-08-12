package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.guided.ControlOrigin
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.light.AuxiliaryLight
import com.dimensional.mini4pro.light.LightControl
import com.dimensional.mini4pro.handshake.toMavResult
import com.dimensional.mini4pro.telemetry.Px4Mode

/**
 * Turns the three QGroundControl buttons this bridge implements — Takeoff, Return and Land — into
 * calls on [FlightActions], and turns whatever comes back into the one thing the operator can
 * read.
 *
 * This is the MAVLink half of M2 and M2.5. It contains no DJI code, no Android, and no clock it
 * did not receive — so the whole of it, including every refusal path, is exercised by
 * `CommandDispatcherTest` without an aircraft.
 *
 * ## Two inbound paths, and they are not alike
 *
 * The single most important fact about this class is that the buttons it answers do not arrive
 * the same way, and cannot be given the same feedback.
 *
 * | button | arrives as | acknowledged? | what we do |
 * |---|---|---|---|
 * | Takeoff | `COMMAND_LONG` 22, `param7` = **AMSL** | yes, and the ack **arms QGC** | [FlightActions.takeoff] behind [onTakeoff]'s gates |
 * | Return | `SET_MODE` (#11), `custom_mode` `AUTO.RTL` | **no** | [FlightActions.returnToHome]; QGC watches the flight mode in *our heartbeat* |
 * | Land | `SET_MODE` (#11), `custom_mode` `AUTO.LAND` | **no** | [FlightActions.land]; same |
 * | Arm | `COMMAND_LONG` 400, `param1 = 1` | yes | **refused**, always, plus [StatusTexts.NO_SEPARATE_ARM] |
 * | Emergency Stop | `COMMAND_LONG` 400, `param1 = 0, param2 = 21196` | yes, one `COMMAND_ACK` | **refused**, plus a `STATUSTEXT` pointing at the RC — see [EMERGENCY_STOP_TEXT] |
 *
 * The takeoff row is the odd one and drags a rule behind it: **its `COMMAND_ACK` is not only a
 * report, it is an instruction.** `MAV_RESULT_ACCEPTED` on command 22 makes QGC send an arm
 * command on its own initiative (`PX4FirmwarePlugin.cc:307-315`). So this class has exactly one
 * place that may return `ACCEPTED` for it ([perform]'s `Requested` branch) and one place that
 * answers the consequence ([onArmDisarm]).
 *
 * **The mode dropdown's `AUTO.TAKEOFF` is deliberately still refused.** Only QGC's Takeoff
 * *button* is implemented, because only the button carries an altitude, is acknowledged, and has
 * been measured off the wire. A `SET_MODE` to takeoff would be an unmeasured second route to the
 * most dangerous action here, with no ack to refuse through and no altitude to report on — see
 * [actionFor].
 *
 * All the measured rows come off the wire from QGC 5.0.8 on 2026-07-25 and are tabulated with
 * their QGC source references in `HandshakeResponder.kt:129-182`. The `SET_MODE` rows are the
 * consequence of PX4 leaving `MAV_CMD_DO_SET_MODE_is_supported()` false, so `Vehicle::
 * setFlightMode` takes the message branch instead of the command branch.
 *
 * Three rules fall out of that table, and each is a test:
 *
 *  1. **Never send a `COMMAND_ACK` for a `SET_MODE`.** QGC discards an ack it never queued
 *     ("Ack not in list", `MavCommandQueue.cc:489`), so it is noise at best; and the two
 *     `SET_MODE` actions have no ack semantics to borrow.
 *  2. **Never echo a requested mode into the heartbeat.** That field is exactly what
 *     `_setFlightModeAndValidate` polls (`FirmwarePlugin.cc:246-274`). Echoing it would report a
 *     return-to-home that is not happening. This class has no route to the heartbeat at all —
 *     the mode comes from `Px4Mode.customMode(state.flightMode)`, i.e. from what the aircraft
 *     said — and that absence is the guarantee, not a convention. It also means a
 *     [FlightActions] implementation that lies about return or land still cannot make QGC
 *     believe the aircraft complied.
 *  3. **One press must be one action.** QGC's guided buttons send the same `SET_MODE` three
 *     times about 1.34 s apart (measured; `HandshakeResponder.kt:175-176`). Three DJI calls for
 *     one click is a bug with an aircraft attached to it. See [ACTION_REPEAT_MS].
 *
 * ## Everything is gated
 *
 * [CommandInterlock] is consulted before anything reaches [FlightActions], and while it is off
 * this class **declines** rather than refusing in its own words: the `SET_MODE` handler returns
 * false so `HandshakeResponder`'s pre-M2 refusal runs unchanged. With commands off, the
 * actuating path is not merely quiet — it is byte-for-byte the bridge that existed before it
 * did, which is why `docs/decisions/2026-07-25-m2-command-safety.md` §Q2 calls the safe state
 * "the current honest behaviour" rather than a second code path.
 *
 * The `ARM_DISARM` handler is outside all of that, because it moves nothing in any state: it
 * answers `MAV_RESULT_UNSUPPORTED` unconditionally and only adds a sentence.
 */
class CommandDispatcher(
    private val interlock: CommandInterlock,
    /**
     * Where an operator-facing sentence goes — every attached interface, not one link.
     *
     * Replaced the `send: (Any) -> Unit` that built a `STATUSTEXT` here: composing that message
     * is a MAVLink fact and now lives in `mavlink/StatusTextSink`, while the decision to say
     * something lives here and reaches whatever is listening. Nothing else about [announce]
     * changed — the [ACTION_REPEAT_MS] window and the text it is keyed on are untouched.
     */
    private val announcer: Announcer,
    /**
     * **The AMSL this bridge publishes**, in metres, or null when it is unknown — the single
     * number every commanded altitude is interpreted against, and the only datum that may be
     * used for it.
     *
     * `Bridge` supplies `TelemetryEncoder.amslMetres(aircraftState())`, which is *literally the
     * function that fills `GLOBAL_POSITION_INT.alt`* rather than a re-derivation of it. That
     * identity is the whole safety argument for [relativeTakeoffAltitude]; see its KDoc for why
     * a second, equally-correct-looking source would be wrong.
     *
     * Defaults to null, so a dispatcher constructed without it refuses every commanded altitude
     * rather than inventing a datum of zero — which would turn QGC's `param7 = 106.2` into a
     * 106 m takeoff.
     */
    private val publishedAmslM: () -> Double? = { null },
    /**
     * The thing that flies the **second phase of a takeoff**, or null when there is none —
     * `Bridge` supplies `{ guidedStick }`, which is null before `start()` and after `stop()`.
     *
     * Read fresh on every use rather than held, for the reason [actions] is: the guided engine is
     * created and destroyed with the link, and a dispatcher holding a stale one would arm a climb
     * on an engine nobody is ticking.
     *
     * A null changes only what the operator is told: the takeoff is still dispatched, still
     * acknowledged, and the pre-existing [StatusTexts.takeoffHeight] sentence is used because with
     * nothing to fly the second phase it is true again. See [PendingClimb] for the whole shape.
     */
    private val climb: () -> PendingClimb? = { null },
    /**
     * **QGC's Start Mission and Continue Mission**, both of which arrive here as a `SET_MODE` to
     * PX4's `AUTO.MISSION` (M4-2: the button lives in QGC).
     *
     * Returns true when the executor took responsibility — which includes having *told the operator*
     * about a refusal, exactly as [onModeRequest]'s own contract does, because a refusal that has
     * been announced must not also raise `HandshakeResponder`'s generic "bridge is telemetry-only"
     * warning on top of a sentence that is more specific and more true.
     *
     * Null when there is no executor, and then `AUTO.MISSION` falls through to that generic refusal
     * — which is the pre-M4 behaviour, byte for byte.
     *
     * **Deliberately not routed through [perform] and its [ACTION_REPEAT_MS] window.** QGC sends the
     * same `SET_MODE` three times about 1.34 s apart, and the collapse is done inside the executor
     * instead: a Start while already `RUNNING` is a no-op there, which is both the honest answer and
     * the one that cannot re-run a launch check against a moving aircraft. Putting the burst filter
     * here would additionally swallow a *genuine* second press after a refusal, and the whole point
     * of a refusal that names its reason is that fixing the reason and pressing again works.
     */
    private val onMissionStart: (() -> Boolean)? = null,
    /**
     * **The bottom auxiliary lamp**, or null when there is none — `Bridge` supplies `{ light }`,
     * which is null before `start()` and after `stop()`.
     *
     * Read fresh on every use, for the reason [actions] and [climb] are: the control is created
     * and destroyed with the link.
     *
     * A null makes the command `UNSUPPORTED`, which is the pre-existing behaviour for a command
     * with no handler and is what QGC will render as a plain "unsupported" rather than as a
     * failure the operator should worry about.
     */
    private val light: (() -> LightControl?)? = null,
    /** Optional trace hook; keeps `android.util.Log` out of a unit-testable layer. */
    private val log: (String) -> Unit = {},
    /** Wall clock, injected so both repeat windows are testable without sleeping. */
    private val nowMs: () -> Long = { System.currentTimeMillis() },
) {

    companion object {
        /**
         * `MAV_CMD_NAV_TAKEOFF`. A number for the same reason [MAV_CMD_COMPONENT_ARM_DISARM] is.
         *
         * **The only acknowledged command in this class that can actuate anything**, and the ack
         * is itself a control input to QGroundControl: `MAV_RESULT_ACCEPTED` on this id makes
         * `PX4FirmwarePlugin::_mavCommandResult` send `MAV_CMD_COMPONENT_ARM_DISARM param1=1`
         * unprompted (`PX4FirmwarePlugin.cc:307-315`). Every non-accepted result stops that
         * chain, which is what the whole of [onTakeoff] is arranged around.
         */
        const val MAV_CMD_NAV_TAKEOFF = 22

        /**
         * How high DJI's automatic takeoff goes, in metres. **Not a setting and not a request —
         * the only height `FC.KeyStartTakeoff` can produce.**
         *
         * djidoc, `FlightControllerKey.KeyStartTakeoff`: *"To start the automatic taking off of
         * the aircraft. When the aircraft hovers at an altitude of **1.2 meters (4 feet)** above
         * the ground, taking off is completed."* Corroborated by DJI's own reference widget,
         * whose `TAKEOFF_HEIGHT = 1.2f` is the number it displays for the same key
         * (`TakeOffWidgetModel.kt:143-154`).
         *
         * Used for exactly one thing: telling the operator what will actually happen
         * ([StatusTexts.takeoffHeight]). Nothing branches on it, because there is nothing to
         * branch on — see [onTakeoff] for why a mismatch is announced rather than refused.
         */
        const val DJI_TAKEOFF_HEIGHT_M = 1.2

        /**
         * The largest commanded height, in metres above the aircraft, that this bridge will treat
         * as a takeoff. **A datum-provenance check wearing a range check's clothes.**
         *
         * `docs/measurements/2026-07-26-amsl-datum.md` ends in a requirement: *"A commanded AMSL
         * may only be converted back using a `takeoffAltitudeAmsl` sample the bridge itself
         * published. An AMSL from any other source — a saved plan, a terrain database, QGC's
         * elevation service, a survey, a number typed by a human who looked at a map — is in a
         * different datum and must be refused, not converted."* That requirement has no direct
         * enforcement available: `param7` is a bare float and carries no provenance.
         *
         * What it does leave is a **signature**. DJI's "AMSL" is pressure altitude on the
         * 1013.25 hPa reference, so it disagrees with any real-world AMSL by
         * `(1013.25 − QNH) × 8.3 m` — measured at +14 m on 2026-07-25 and −28 m on 2026-07-26,
         * and unbounded in principle (a deep low against a strong high is 30 hPa, i.e. 250 m).
         * A `param7` from a foreign datum therefore lands *tens to hundreds of metres* away from
         * where our own arithmetic would put it, which is far outside any height a human asks a
         * Mini 4 Pro to climb to.
         *
         * The bound is QGC's own: `guidedMaximumAltitude`, default **121.92 m** (400 ft) in
         * `FlyView.SettingsGroup.json`, is the top of the takeoff slider. Taking QGC's number
         * rather than inventing one means the band is exactly "what this ground station's takeoff
         * control can produce". An operator who raises that setting gets a refusal naming the
         * number ([StatusTexts.takeoffAltOutOfRange]) rather than a silent clamp — the M3
         * envelope's rule, for the same reason: a clamped command is one the operator believes
         * was obeyed.
         *
         * **Be clear about what this cannot do.** A foreign datum that happens to differ by a few
         * metres passes it, and so does one that lands inside the band by luck. It catches the
         * gross case, which is the likely one, and it fails closed on the absurd. It is not a
         * proof of provenance and nothing here should be written as though it were.
         */
        const val MAX_TAKEOFF_HEIGHT_M = 121.92

        /**
         * Told to the operator when a takeoff arrives and we have no AMSL of our own to invert
         * `param7` against — `Takeoff failed: NO_ALT_DATUM`, 30 bytes.
         *
         * This is `TelemetryEncoder.amslMetres` returning null, which it does whenever either
         * half of `takeoffAltitudeAmsl + relativeAltitude` is missing. **Refusing is the only
         * option**: with no datum the commanded altitude cannot be interpreted at all, and the
         * available fallbacks are worse than useless — treating `param7` as a height above ground
         * would read QGC's 106.2 as a 106 m takeoff, and assuming a datum of zero is the same
         * mistake spelled differently.
         *
         * Note it is also the state a fresh bridge is in before DJI's first
         * `KeyTakeoffLocationAltitude` delivery, so the first seconds of a session refuse takeoff
         * and then stop doing so. That is correct and is not worth smoothing over.
         */
        const val NO_ALT_DATUM = "NO_ALT_DATUM"

        /**
         * Told to the operator when `param7` is not a finite number —
         * `Takeoff failed: ALT_NOT_A_NUMBER`, 35 bytes.
         *
         * QGC guards its own side (`guidedModeTakeoff` refuses with *"Unable to takeoff, vehicle
         * position not known"* when `altitudeAMSL` is NaN, `PX4FirmwarePlugin.cc:319-322`), so
         * this should be unreachable from QGC. It is checked because *this* class must not be the
         * thing that trusts it: `NaN` propagates silently through every comparison below —
         * `NaN > MAX` and `NaN <= 0` are both false — so without an explicit test a non-finite
         * altitude would pass every guard and be announced as `NaNm`.
         */
        const val ALT_NOT_A_NUMBER = "ALT_NOT_A_NUMBER"

        /**
         * The height the phone's own **Take off** button climbs to, metres above the takeoff
         * datum — Ivan's requested default (2026-07-28: *"takes off to 10 meter height by
         * default"*), his test altitude choice and not a number derived from anything.
         *
         * Be honest about what it is: **above the tag band, and armable anyway.** The
         * reliable-detection band tops out at 7 m (`TagDescentGuidance.ARM_CEILING_M`), and
         * until 2026-07-29 every flight from this hover ate a `tag_descent_denied above 7m tag
         * band` (landing13 t=41.8) followed by a blind manual descent. A descent armed from
         * this height now enters the APPROACH segment (`TagDescentPhase.APPROACH`, under
         * `TagDescentGuidance.APPROACH_CEILING_M` = 12 m) and flies itself down into the band —
         * this constant is the approach's design use case, cited in its KDocs.
         *
         * Well inside every bound it meets: above `TakeoffClimb.NO_CLIMB_BELOW_M` (~2.7 m), so
         * the climb always arms and the nadir camera move always has a handback to ride; under
         * [MAX_TAKEOFF_HEIGHT_M] (121.92 m) and the M3 ceiling, so it is never refused or capped
         * on range. `CommandDispatcherTest` pins the armed height so this constant breaking is a
         * red suite, not a different flight.
         */
        const val PHONE_TAKEOFF_HEIGHT_M = 10.0

        /**
         * `MAV_CMD_COMPONENT_ARM_DISARM`. Spelled as a number for the same reason
         * `HandshakeResponder` spells its ids as numbers: a dialect that renames the enum entry
         * still compiles, and still routes the same button.
         */
        const val MAV_CMD_COMPONENT_ARM_DISARM = 400

        /**
         * `MAV_CMD_USER_1` — the bottom auxiliary lamp, with the mode in `param1`.
         *
         * MAVLink reserves 31010-31014 for user-defined commands, which is precisely what this is:
         * there is no standard command for a lamp on an airframe QGC believes is a PX4 quadcopter,
         * and inventing a meaning for a standard id would collide with whatever QGC or PX4 decides
         * that id means later. `param1` is 0 AUTO, 1 ON, 2 OFF, 3 BEACON — the mapping lives in
         * `light/AuxiliaryLight.fromParam` and is refused there rather than clamped.
         *
         * `qgc/MavlinkActions.json` is the file that turns this into three buttons in QGC's Fly
         * View; it needs no QGC changes, only a path in Fly View Settings.
         */
        const val MAV_CMD_USER_1 = 31010

        /**
         * `param2` of `MAV_CMD_COMPONENT_ARM_DISARM` for a **force** disarm — QGC's Emergency
         * Stop, measured (`HandshakeResponder.kt:134`). Without this magic the same command id
         * is the ordinary Arm/Disarm button. Neither form actuates anything here; the magic is
         * used only to tell the two apart so the operator gets the right sentence back.
         */
        const val FORCE_DISARM_MAGIC = 21196f

        /**
         * What an operator is told when they press **Emergency Stop**. 44 bytes, counted,
         * against the 50-byte `STATUSTEXT` field.
         *
         * `MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=21196` is MAVLink's force-disarm-in-
         * flight: cut the motors now. **MSDK 5.18.0 has no equivalent**, verified against the
         * class surface including the bytecode `canPerformAction` flags
         * (`docs/msdk/actions-rth-and-arming.md`): there is no callable in-flight motor stop at
         * all, `KeyAreMotorsOn` is telemetry-only, `KeyLockMotors` is set-only and pre-flight,
         * and `KeyEmergencyStopMotorEnable` / `KeyFCUrgentStopMotorMode` only configure whether
         * the RC's physical stick gesture is armed. The only thing on this airframe that takes
         * motors to zero is a several-second controlled landing.
         *
         * So the command stays refused — mapping "stop now" onto "start a slow descent" would be
         * the most dangerous substitution in this project, and an operator pressing Emergency
         * Stop has already decided the aircraft must stop being a flying object. But a bare
         * `MAV_RESULT_UNSUPPORTED` leaves them pressing the button again while seconds pass,
         * which is why this sentence exists: it names the one thing that *does* work, on the
         * controller in their hands.
         *
         * Sent whether or not [CommandInterlock] is enabled, because it is information rather
         * than actuation and it is equally true either way.
         *
         * **Reworded 2026-07-26 after measuring what the operator actually receives**
         * (`docs/measurements/2026-07-26-emergency-stop.md`). The old text said "use RC stick
         * gesture", which names a *category* rather than an act: the gesture is both sticks held
         * inner-down for about two seconds, and nobody has ever performed it in flight. Two things
         * the measurement turned up made that worse than it looked — QGC reads this **aloud**
         * (`readAloud` fires at severity ≤ NOTICE), where "gesture" is useless, and our own
         * `UNSUPPORTED` ack raises a second dialog ~4 ms later that talks over this one, so the
         * sentence has to survive being the lower of two. It now says the thing to do with the
         * hands already holding the controller.
         *
         * The remaining imprecision is deliberate: "inner-down" describes both Mode 1 and Mode 2
         * without naming a stick, and the 50-byte field cannot hold the qualification. Being
         * fully precise here would cost the instruction.
         */
        const val EMERGENCY_STOP_TEXT = "No motor cut. Hold both RC sticks inner-down"

        /**
         * How long the same action is considered one press.
         *
         * QGC's guided buttons send three identical `SET_MODE`s about 1.34 s apart, so the burst
         * spans ~2.7 s; 5 s covers it with margin and matches
         * `HandshakeResponder.MODE_REFUSAL_REPEAT_MS`, which collapses the same burst on the
         * refusal side. Deliberately the same number as that one: two windows over the same
         * measured burst that could drift apart would be a bug waiting for a QGC release.
         *
         * **Only a request that reached DJI is suppressed.** A repeat after a refusal or a
         * failure is passed through and tried again — an operator pressing Emergency Stop a
         * second time because the first said `FC_AUTH_STATE` must not be answered with silence.
         */
        const val ACTION_REPEAT_MS = 5_000L
    }

    /**
     * The DJI layer — `MsdkFlightActions` in the app, set by `Bridge.start` and cleared by
     * `Bridge.stop`, so it is non-null exactly while a link exists. A null is reported to the
     * operator as a failure rather than absorbed — an aircraft that cannot be asked has not
     * agreed. Read on the `mavlink-rx` thread.
     */
    @Volatile
    var actions: FlightActions? = null

    /** Last action that actually reached [FlightActions] and was not refused. */
    @Volatile
    private var lastRequestedAction: FlightAction? = null

    @Volatile
    private var lastRequestedAtMs: Long = Long.MIN_VALUE

    /** Last `STATUSTEXT` we produced, so a retry burst is not three identical red lines. */
    @Volatile
    private var lastAnnouncement: String? = null

    @Volatile
    private var lastAnnouncedAtMs: Long = Long.MIN_VALUE

    /**
     * Registers this dispatcher with the responder. Idempotent; all registrations overwrite.
     *
     * Only **two** of the three registrations can move an aircraft, and they are the mode handler
     * (Return and Land) and [onTakeoff]. The `ARM_DISARM` command handler is registered to
     * *speak*, not to act — it always answers `MAV_RESULT_UNSUPPORTED`, exactly as the
     * unregistered command did, and only adds a sentence ([EMERGENCY_STOP_TEXT] or
     * [StatusTexts.NO_SEPARATE_ARM]). It is therefore deliberately not gated on
     * [CommandInterlock]: it actuates nothing in any state.
     *
     * **Takeoff's registration re-opens QGC's auto-arm chain, deliberately and with the arm
     * handled.** Until 2026-07-26 the absence of a `MAV_CMD_NAV_TAKEOFF` handler was
     * load-bearing rather than a to-do: the resulting `MAV_RESULT_UNSUPPORTED` was precisely what
     * stopped QGC sending `MAV_CMD_COMPONENT_ARM_DISARM param1=1` on its own initiative
     * (`PX4FirmwarePlugin::_mavCommandResult`, cited at `HandshakeResponder.kt:132`). Registering
     * a handler does not by itself change that — the handler answers `UNSUPPORTED` in every state
     * except an actually-dispatched takeoff, so the chain stays shut in every one of them. Where
     * it does open, [onArmDisarm] answers the arm truthfully rather than being surprised by it,
     * and the pair is argued there. `docs/decisions/2026-07-26-takeoff.md`.
     *
     * Note what is *still* not registered: no parameter writer, because a writable parameter
     * would be a MAVLink message that changes our behaviour, which is the thing
     * [CommandInterlock] exists to forbid.
     */
    fun attachTo(responder: HandshakeResponder) {
        responder.registerModeHandler(::onModeRequest)
        // `.toMavResult()` is the MAVLink edge and the only place it appears in this class:
        // the handlers below decide in [Verdict], and the wire type is put on here, once, where
        // the wire is (`handshake/Verdicts.kt`).
        responder.registerCommandHandler(MAV_CMD_COMPONENT_ARM_DISARM) { onArmDisarm(it).toMavResult() }
        responder.registerCommandHandler(MAV_CMD_NAV_TAKEOFF) { onTakeoff(it).toMavResult() }
        responder.registerCommandHandler(MAV_CMD_USER_1) { onLight(it).toMavResult() }
    }

    // ------------------------------------------------------------ MAV_CMD_USER_1 (#31010)

    /**
     * The bottom auxiliary lamp, from QGC's Fly View actions — `param1` names the mode.
     *
     * **Not behind the interlock**, and that is argued in [LightControl]'s KDoc: the interlock
     * gates the commands that move the aircraft, the interlock starts off every session, and an
     * operator setting up after dark wants the lamp before they arm anything.
     *
     * **Not routed through [perform] and its [ACTION_REPEAT_MS] window.** That window exists to
     * stop QGC's repeated presses becoming repeated *flight actions*; writing the same lamp mode
     * twice is idempotent and harmless, and swallowing the second press would break the case an
     * operator will actually hit — pressing On, seeing nothing, and pressing it again.
     *
     * `MAV_CMD_USER_1` rather than a standard command id because there is no standard command for
     * a lamp on an airframe QGC thinks is a PX4 quadcopter. The user range is reserved for exactly
     * this, so nothing we do with it can collide with a meaning QGC or PX4 already has.
     */
    private fun onLight(request: HandshakeResponder.CommandRequest): Verdict {
        val control = light?.invoke()
        if (control == null) {
            log("light asked for with no link up — unsupported")
            return Verdict.UNSUPPORTED
        }
        val mode = AuxiliaryLight.fromParam(request.param1)
        if (mode == null) {
            // A number that names no mode. Refused rather than mapped to something the SDK would
            // accept — DJI's own enum has an UNKNOWN member, so a lenient mapping here would ask
            // the aircraft for a mode nobody chose.
            log("light refused: param1=${request.param1} names no mode")
            return Verdict.DENIED
        }
        return control.setMode(mode)
    }

    // ------------------------------------------------------------ SET_MODE (#11)

    /**
     * Return and Land. Returns true only when this class has taken responsibility for the
     * request — which includes having told the operator about a refusal, since in that case
     * `HandshakeResponder`'s generic "bridge is telemetry-only" warning would be both redundant
     * and less true than `FC_AUTH_STATE`.
     *
     * Returns false for every mode we do not implement (Hold, Mission, Position, Altitude, Acro,
     * Stabilized, Takeoff) and for every mode at all while the interlock is off, so those keep
     * the pre-M2 behaviour exactly.
     */
    private fun onModeRequest(request: HandshakeResponder.ModeRequest): Boolean {
        // Start Mission / Continue Mission. Ahead of [actionFor] because it is not a
        // [FlightAction] at all — nothing is *sent to DJI* here; the executor runs its own launch
        // check and engages the guided engine, which is a different door with different gates.
        //
        // The interlock is checked by the executor rather than here, and that is the one asymmetry
        // worth naming: with the interlock off the executor refuses with `interlock off`, which is
        // more useful than the generic pre-M2 refusal, and it must be the executor's answer because
        // it is also the answer a *resume* gets and the answer that spends a resume block.
        if (request.customMode == Px4Mode.AUTO_MISSION) {
            val start = onMissionStart ?: return false
            log("Start Mission asked for")
            return start()
        }
        val action = actionFor(request.customMode) ?: return false
        if (!interlock.enabled) {
            // Not our sentence to write. Declining hands the operator back to the existing
            // MODE_REFUSAL_TEXT path, which is what they saw before M2 and is still accurate.
            log("${action.label} asked for, interlock off — declined to HandshakeResponder")
            return false
        }
        perform(action)
        // Taken either way: the request was ours to answer, and whatever needed saying is said.
        // Note what the announcement on the success path is and is not: "Return sent to DJI"
        // reports that the request left this bridge, never that the aircraft is complying. The
        // only honest confirmation of *that* is still the flight mode changing in a later
        // heartbeat, and if it never changes QGC says so itself after ~3.9 s.
        return true
    }

    /**
     * The action QGC's mode dropdown or guided buttons asked for, or null if not one of ours.
     *
     * **`Px4Mode.AUTO_TAKEOFF` is not in this table, and its absence is a decision.** Adding it
     * would give the most dangerous action in the project a second inbound route that has none of
     * the first one's properties: a `SET_MODE` carries no altitude, so there would be nothing to
     * invert against our datum and nothing to tell the operator about DJI's 1.2 m; it carries no
     * acknowledgement, so every refusal in [onTakeoff] would become invisible; and unlike command
     * 22 it has never been captured off the wire from a real QGC. The dropdown therefore keeps
     * falling through to `HandshakeResponder`'s existing refusal, which is honest and already
     * tested.
     */
    private fun actionFor(customMode: Long): FlightAction? = when (customMode) {
        Px4Mode.AUTO_RTL -> FlightAction.RETURN_TO_HOME
        Px4Mode.AUTO_LAND -> FlightAction.LAND
        else -> null
    }

    // ---------------------------------------------------- COMMAND_LONG/INT (22)

    /**
     * `MAV_CMD_NAV_TAKEOFF` — QGC's **Takeoff** button, and the only inbound message in this
     * project that can put an aircraft into the air.
     *
     * ## What the result code does, before what it says
     *
     * This is the one ack in the bridge that is also an *instruction*. `MAV_RESULT_ACCEPTED` here
     * causes QGC to arm on its own initiative within milliseconds; anything else does not. So the
     * rule this whole method is built to keep is simply:
     *
     * > **ACCEPTED if and only if a takeoff actually reached [FlightActions].**
     *
     * Every refusal below therefore answers `MAV_RESULT_DENIED` — one code for all of them, and
     * not a taxonomy. The taxonomy would be decoration: QGC turns each `MAV_RESULT` into one
     * canned line ("denied", "failed", "temporarily rejected"), all of which say less than the
     * `STATUSTEXT` that accompanies them, and none of which QGC acts on differently — it does not
     * auto-retry any of them (measured, `docs/measurements/2026-07-26-qgc-goto-validation.md`).
     * Multiplying codes would multiply the ways this table can be got wrong while changing
     * nothing an operator sees.
     *
     * The **interlock-off case is the exception, and answers `UNSUPPORTED`** rather than `DENIED`.
     * That is `docs/decisions/2026-07-25-m2-command-safety.md` §Q2 taken literally: with commands
     * off, the reply must be *the reply that existed before this feature*, so the safe state is
     * the already-tested path rather than a second branch that could drift. It has a useful side
     * effect — an operator can tell "commands are switched off" (`not supported`) from "commands
     * are on and this takeoff was refused" (`denied` plus a sentence naming the reason) without
     * looking at the app.
     *
     * ## The altitude, and why a mismatch is announced rather than refused
     *
     * [relativeTakeoffAltitude] recovers the height the operator asked for. DJI cannot honour it:
     * `FC.KeyStartTakeoff` takes no parameter and climbs to [DJI_TAKEOFF_HEIGHT_M]
     * ([FlightActions.takeoff] has the bytecode). Three responses were possible.
     *
     *  - **Refuse unless the request matches 1.2 m.** Rejected, and not on grounds of convenience:
     *    it would refuse *every* takeoff QGC can send. QGC's slider is floored at
     *    `Vehicle::minimumTakeoffAltitudeMeters()`, which PX4 does not override, so its minimum is
     *    `FirmwarePlugin.h:204`'s 3.048 m and 1.2 m is not typeable. A feature that can never
     *    succeed is not a safe feature; it is an absent one that people work around.
     *  - **Accept silently.** Rejected outright — that is the bridge telling a ground station a
     *    30 m takeoff was agreed to and delivering 1.2 m, which is the class of lie this whole
     *    layer exists to prevent.
     *  - **Accept and say both numbers.** Taken. The discrepancy is in the safe direction (lower
     *    than asked, RC authority untouched) and the operator is told before the aircraft moves,
     *    in the message QGC reads aloud. `docs/decisions/2026-07-26-takeoff.md` Q2.
     *
     * ## No de-duplication, and that is not an oversight
     *
     * [perform]'s [ACTION_REPEAT_MS] window exists for QGC's 3× `SET_MODE` burst. **Commands do
     * not burst**: they are acknowledged, and QGC's queue re-sends only when an ack fails to
     * arrive. Ours goes out in the same call. So a second `MAV_CMD_NAV_TAKEOFF` is either a
     * genuine second press — which, after the measured case where DJI accepts a command and does
     * nothing (`docs/measurements/2026-07-26-m2-first-command.md`), is exactly the press that
     * must reach DJI — or a retry after a lost ack, where suppressing the *DJI call* while still
     * answering is precisely right. [perform] does both: it drops the duplicate action and still
     * returns the accepted result. Neither case can produce two takeoffs.
     */
    private fun onTakeoff(request: HandshakeResponder.CommandRequest): Verdict {
        if (!interlock.enabled) {
            // Byte-for-byte the pre-takeoff bridge: the same UNSUPPORTED the `else` branch of
            // HandshakeResponder.handleCommand produced when nothing was registered, down to the
            // "unsupported command 22" log line it emits on the result. No STATUSTEXT either —
            // QGC raises its own modal for an unsupported command, so unlike the SET_MODE path
            // there is no silence to fill, and §Q2's "the safe state is the old code" is kept
            // exactly rather than approximately.
            log("Takeoff asked for, interlock off — refused as before")
            return Verdict.UNSUPPORTED
        }

        val datum = publishedAmslM()
        if (datum == null) {
            announce(StatusTexts.unavailable(FlightAction.TAKEOFF, NO_ALT_DATUM))
            log("Takeoff refused: no published AMSL to interpret param7=${request.param7}")
            return Verdict.DENIED
        }

        val requested = relativeTakeoffAltitude(request.param7, datum)
        if (requested == null || !requested.isFinite()) {
            announce(StatusTexts.unavailable(FlightAction.TAKEOFF, ALT_NOT_A_NUMBER))
            log("Takeoff refused: param7=${request.param7} against datum $datum is not finite")
            return Verdict.DENIED
        }
        // aimCameraNadir true since 2026-07-29 — see the door-differences KDoc on
        // [takeoffFromPhone]: the sole operator declared the camera-down sequence wanted on
        // every takeoff, whichever door admitted it. The origin still differs: this door is
        // QGC's, and the climb keeps the MAVLink heartbeat watchdog.
        return dispatchTakeoff(requested, aimCameraNadir = true, origin = ControlOrigin.MAVLINK)
    }

    /**
     * The phone's own **Take off** button — the second and last inbound door to a takeoff, and
     * deliberately a door onto **the same corridor**: everything past the two entry differences
     * below is [dispatchTakeoff], the identical range gate, the identical [perform] (one DJI
     * call, one de-dup window shared with QGC's button — a phone press milliseconds after a QGC
     * press is one takeoff, not two), the identical [armClimb] and its sentences. The refactor
     * exists so this method *cannot* drift from `MAV_CMD_NAV_TAKEOFF`; `docs/decisions/
     * 2026-07-26-takeoff.md`'s whole argument against a second unmeasured route stands, and this
     * is not one — it is the measured route with a different door.
     *
     * The honest per-door facts, all properties of the door rather than of the takeoff (the
     * first is no longer a difference in value, and says so):
     *
     *  - **No AMSL inversion, so no [NO_ALT_DATUM] refusal.** QGC speaks AMSL and `param7` must
     *    be inverted against our own published datum; the phone asks in this bridge's own
     *    relative metres, so there is nothing to invert and a missing datum refuses nothing
     *    here. Every altitude question that actually needs the datum (the climb's target, the
     *    ceiling) is re-asked by the engine at fly time, where it always was.
     *  - **[aimCameraNadir]** — Ivan's sequence ("point the gimbal down 90 degrees") rides the
     *    armed climb and fires at DJI's handback, through the bridge's own gimbal path so
     *    `CommandedGimbalPort.pitchDeg` records it. *No longer a door difference in value*:
     *    "QGC's door passes false — its operator owns their camera" was the generic-GCS
     *    argument, and it stood until 2026-07-29, when the sole operator declared the
     *    camera-down sequence wanted on every takeoff regardless of door (Ivan: *"I'm the only
     *    operator and prefer camera down"*). Both doors now pass true; the flag stays plumbed
     *    per-door so a future generic-GCS deployment splits them again with one argument.
     *  - **[ControlOrigin.PHONE]** — this door names itself, riding the armed climb exactly as
     *    the camera flag does, so the climb's engagement watches the liveness of the controller
     *    that actually commanded it: this process, alive by identity. Labelled MAVLINK — the
     *    only label there was before 2026-07-29 — the climb demanded a QGC heartbeat a
     *    phone-only flight never has, and landing08 (`datasets/landing08/
     *    20260729-112216.001.jsonl`) measured the result: engaged t=32.33, released
     *    `link-lost` t=33.93, the aircraft stranded at DJI's 1.2 m hop. QGC's door passes
     *    MAVLINK and keeps the heartbeat watchdog byte-for-byte. Origin and camera flag are
     *    **independent facts on the climb** — one decides whose liveness ends it, the other
     *    decides where the camera goes — and since 2026-07-29 a MAVLINK-origin climb carries
     *    the camera flag too, which is why neither is derived from the other anywhere.
     *
     * Interlock off answers `UNSUPPORTED` with no sentence, exactly as [onTakeoff] and
     * `armTagDescent` do — the caller is a screen with the operator's eyes on it, and
     * `MainActivity` names the interlock on it before and after this call; a `STATUSTEXT` from
     * here would say the same thing to the same person twice.
     *
     * Runs on the main thread (a tap listener), which [perform] and [armTakeoffClimb] both
     * tolerate: the dispatcher's fields are volatile, and the engine takes its own lock.
     */
    fun takeoffFromPhone(relAltM: Double = PHONE_TAKEOFF_HEIGHT_M): Verdict {
        if (!interlock.enabled) {
            log("phone takeoff asked for, interlock off — refused")
            return Verdict.UNSUPPORTED
        }
        return dispatchTakeoff(relAltM, aimCameraNadir = true, origin = ControlOrigin.PHONE)
    }

    /**
     * **The one corridor a takeoff travels once its door has admitted it** — the range gate, the
     * single [perform] that may answer `ACCEPTED`, and the climb arming. Both doors ([onTakeoff]
     * after its AMSL inversion, [takeoffFromPhone] after its interlock check) end here, which is
     * the shared-path property the button was specified with: one implementation, one set of
     * refusals, one de-dup window, one place QGC's auto-arm chain can open.
     *
     * The caller has already checked the interlock — kept at the doors because the two answer it
     * differently in *reply shape* (QGC's byte-for-byte pre-feature `UNSUPPORTED` must precede
     * even the datum refusal; the phone's is silent for the screen to name) while being the same
     * refusal in substance. The doors also name themselves ([origin]): the corridor is shared,
     * the identity of the commander is not, and the climb's liveness watchdog is evaluated
     * within it (landing08 — see [takeoffFromPhone]'s third door difference).
     */
    private fun dispatchTakeoff(requested: Double, aimCameraNadir: Boolean, origin: ControlOrigin): Verdict {
        if (!requested.isFinite() || requested <= 0.0 || requested > MAX_TAKEOFF_HEIGHT_M) {
            // Two failures share this branch and both are worth refusing. A non-positive height
            // is a "takeoff" that does not go up, which no correct caller sends and which we must
            // not resolve into DJI's 1.2 m climb. An implausibly large one is the visible symptom
            // of a `param7` composed against a datum that is not ours — see MAX_TAKEOFF_HEIGHT_M.
            // The finite check is [onTakeoff]'s belt re-buckled for the second door: unreachable
            // from the phone's constant, and this corridor must not trust its doors.
            announce(StatusTexts.takeoffAltOutOfRange(requested, MAX_TAKEOFF_HEIGHT_M))
            log("Takeoff refused: ${requested}m outside 0..${MAX_TAKEOFF_HEIGHT_M}m")
            return Verdict.DENIED
        }

        val result = perform(FlightAction.TAKEOFF)
        if (result == Verdict.ACCEPTED) {
            // After the dispatch announcement and only on the accepted path: an altitude caveat
            // for a takeoff that was refused would be noise attached to a non-event, and — far
            // more important — a takeoff that never reached DJI must never leave a climb armed,
            // nor a camera move waiting behind one. Arming lives inside this `if` for that
            // reason and for no other.
            armClimb(requested, aimCameraNadir, origin)
        }
        return result
    }

    /**
     * Phase 2 of a takeoff: hand the requested height to whatever will fly it once DJI's own hop
     * is finished, and tell the operator which of the two possible futures they are in.
     *
     * **This function's whole job is that the sentence matches the plan.** Before 2026-07-27 there
     * was one sentence — [StatusTexts.takeoffHeight], *"DJI goes to 1.2m, not 3.0m"* — and it was
     * the honest one because 1.2 m was where the aircraft stopped. With a climb armed it becomes
     * false in the opposite direction, so the two are exclusive and the choice is made on
     * [ClimbArm], the only thing that knows whether anything was armed:
     *
     * | outcome | what the operator reads |
     * |---|---|
     * | no [climb] attached (no link, no engine) | [StatusTexts.takeoffHeight] — nothing will follow the hop |
     * | [ClimbArm.NothingToDo] | [StatusTexts.takeoffHeight] — the request is inside DJI's own hop |
     * | [ClimbArm.Armed] | [StatusTexts.takeoffThenClimb], naming the height that will be flown |
     * | [ClimbArm.Armed] with `capped` | …plus [StatusTexts.takeoffClimbCapped] |
     *
     * Severity is `ERROR` throughout for the measured reason every announcement here is: QGC 5.0.8
     * shows an operator nothing below it (`StatusTextHandler.cc:18-24`).
     *
     * Note what is *not* here: no result-code change, no second ack, no gate of its own. Arming is
     * not actuation — the aircraft is on the ground and DJI is about to fly it — and every gate
     * that matters is re-read by the engine on the tick that would actually start the climb.
     */
    private fun armClimb(requested: Double, aimCameraNadir: Boolean = false, origin: ControlOrigin = ControlOrigin.MAVLINK) {
        val port = climb()
        val armed = port?.armTakeoffClimb(requested, aimCameraNadir, origin)
        when (armed) {
            null, is ClimbArm.NothingToDo -> {
                // The pre-2026-07-27 sentence, unchanged, in exactly the two states where it is
                // still true. Sent every time rather than only when the numbers differ, because
                // they always differ (QGC cannot ask for 1.2 m) and a rule with no exceptions
                // cannot acquire one.
                if (port == null) log("takeoff accepted with no climb path attached — DJI's height only")
                announce(StatusTexts.takeoffHeight(requested, DJI_TAKEOFF_HEIGHT_M), Severity.ERROR)
            }

            is ClimbArm.Armed -> {
                log("takeoff climb armed: ${armed.relAltM}m${if (armed.capped) " (capped)" else ""}")
                announce(StatusTexts.takeoffThenClimb(armed.relAltM, DJI_TAKEOFF_HEIGHT_M), Severity.ERROR)
                if (armed.capped) {
                    // The height that will be flown is already in the sentence above; this one
                    // adds that it is not the height that was asked for. Capped and announced,
                    // never silently clamped — the goto's and the orbit's rule for their own
                    // altitudes, applied to this one.
                    announce(StatusTexts.takeoffClimbCapped(armed.relAltM), Severity.ERROR)
                }
            }
        }
    }

    /**
     * The commanded height above the aircraft, in metres, recovered from QGC's AMSL `param7` —
     * **the AMSL round trip, and the one place its rule is enforced.**
     *
     * QGC composes the number as `takeoffAltRel + vehicle->altitudeAMSL()`
     * (`PX4FirmwarePlugin.cc:317-338`), where `altitudeAMSL` is whatever arrived last in
     * `GLOBAL_POSITION_INT.alt` (`Vehicle.cc:870`; `GPS_RAW_INT.alt` is the fallback when no
     * global position has been seen, `Vehicle.cc:852`, and this bridge fills both from the same
     * `TelemetryEncoder.amslMetres`). So the inverse is one subtraction, and it is exact.
     *
     * **Why it is safe to subtract a number that is measurably wrong.** Our published AMSL is
     * `takeoffAltitudeAmsl + relativeAltitude`, and `takeoffAltitudeAmsl` is not AMSL at all: it
     * is pressure altitude on the 1013.25 hPa reference, off by `(1013.25 − QNH) × 8.3 m` — +14 m
     * on 2026-07-25 and −28 m on 2026-07-26, at one stationary site
     * (`docs/measurements/2026-07-26-amsl-datum.md`). None of that matters here, because the
     * datum enters QGC's sum with one sign and leaves ours with the other and **cancels exactly,
     * whatever the weather**. The measured proof is in that document: QGC offered `param7 = 106.2`
     * for a 3 m takeoff from a datum of 103.2, because it had taken our own reported AMSL and
     * added 3.
     *
     * **Why the datum must come from [publishedAmslM] and nowhere else.** The cancellation is the
     * *only* reason the arithmetic works, and it requires both appearances to be the same
     * quantity. Subtracting a terrain-database elevation, a GNSS altitude, a survey, or the site's
     * real ~89.5 m would leave the pressure offset uncancelled and turn a 3 m takeoff into a
     * request tens of metres out. `Bridge` therefore passes the very function that fills the
     * telemetry field, not a re-derivation, and [MAX_TAKEOFF_HEIGHT_M] catches the gross case
     * where `param7` itself came from a foreign datum.
     *
     * **Staleness deliberately does not gate this, and the reason is not laziness.**
     * `Signal.TAKEOFF_ALTITUDE` is a change-driven key whose age is measured at a 25.2 s median
     * and a 50.3 s maximum on a real aircraft (`SampleAges`), so a freshness limit would refuse
     * most takeoffs. It would also be answering the wrong question: QGC's `altitudeAMSL` was
     * computed from *the same sample* we are about to subtract, at most one 200 ms telemetry tick
     * earlier, so an old datum makes both sides equally old and cancels just as exactly. Drift
     * over that tick is ~0.6 mm at the measured 2.3 m per twelve minutes. What would break the
     * round trip is a datum from a *different session* — and that cannot happen, because QGC
     * rebuilds `altitudeAMSL` from our wire at 5 Hz and `StateCache` holds one session's samples.
     *
     * Returns null when [param7] is not finite; the caller also re-checks the result, since
     * `datum` being non-finite would produce a `NaN` from a finite input.
     */
    private fun relativeTakeoffAltitude(param7: Float, datum: Double): Double? {
        if (!param7.isFinite()) return null
        return param7.toDouble() - datum
    }

    // --------------------------------------------------- COMMAND_LONG/INT (400)

    /**
     * `MAV_CMD_COMPONENT_ARM_DISARM`. **Nothing on this command id actuates anything**, and this
     * handler exists only to say something truthful to an operator who pressed Emergency Stop.
     *
     * Every form answers `MAV_RESULT_UNSUPPORTED`, which is exactly what the `else` branch of
     * `HandshakeResponder.handleCommand` would have answered had nothing been registered — so
     * QGC's dialog is unchanged and no ack anywhere claims a motor cut happened.
     *
     *  - **Emergency Stop** (`param1 = 0, param2 = 21196`): refused, because MSDK 5.18.0 has no
     *    in-flight motor cut and the nearest thing is a several-second landing. Accompanied by
     *    [EMERGENCY_STOP_TEXT], which points at the RC stick gesture — the operator's only real
     *    option, and one where seconds matter.
     *  - **Arm** (`param1 = 1`): refused, and it now says why —
     *    [StatusTexts.NO_SEPARATE_ARM]. See below; this is the half of the takeoff pair that
     *    lives here.
     *  - **Plain disarm** (`param1 = 0`, no magic): refused. Out of scope, and the same MSDK gap
     *    applies — the on-ground motor stop the button implies is `KeyLockMotors`, which is
     *    set-only and pre-flight (`docs/msdk/actions-rth-and-arming.md`). Silence about it is
     *    acceptable where silence about an emergency stop is not.
     *
     * ## The other half of takeoff
     *
     * `MAV_CMD_NAV_TAKEOFF` answering `MAV_RESULT_ACCEPTED` makes
     * `PX4FirmwarePlugin::_mavCommandResult` send `MAV_CMD_COMPONENT_ARM_DISARM param1=1` on
     * QGC's own initiative, milliseconds later (`PX4FirmwarePlugin.cc:307-315`). Before
     * 2026-07-26 that never happened, because takeoff was never accepted. Now it does, so this
     * handler had to stop being merely correct and start being *legible*.
     *
     * **The result code does not change: `MAV_RESULT_UNSUPPORTED`, as it always was.** That is
     * not a gap being tolerated — MSDK 5.18.0 has no arm vocabulary whatsoever
     * (`docs/msdk/actions-rth-and-arming.md` §6), and the nearest callable thing,
     * `KeyTurnOnTheMotor`, is undocumented, has no symmetric stop, and would be this bridge
     * spinning propellers on its own authority. Accepting the command could only be a lie or a
     * hazard, so it is refused, and it is refused **whether or not a takeoff preceded it and
     * whether or not the interlock is on**. Nothing about the arm path is conditional; there is
     * no window, no ordering, and no state, so there is no state to get wrong.
     *
     * **What changes is that the refusal now explains itself**, because the pair would otherwise
     * mislead. An operator whose takeoff was just accepted gets a modal error about arming a
     * second later, and the natural reading — "the takeoff failed" — is wrong and is the sort of
     * wrong that makes someone press things while an aircraft is leaving the ground. The sentence
     * says the true thing: DJI arms as part of its own takeoff, so there is nothing for a separate
     * arm to do. Nothing false is left standing either way — QGC's armed indicator comes from
     * `MAV_MODE_FLAG_SAFETY_ARMED` in *our heartbeat*, which is derived from what the aircraft
     * reports about its motors, so it goes true when and only when the motors really spin, and
     * this refusal cannot make it say otherwise.
     *
     * Sent for a bare Arm press too, not only after a takeoff, and unconditional on the interlock
     * — exactly like [EMERGENCY_STOP_TEXT], and for the same stated reason: it is information
     * rather than actuation, and it is equally true in every state. Gating it on a timing window
     * would buy nothing and would create states in which a true, useful sentence is withheld.
     */
    private fun onArmDisarm(request: HandshakeResponder.CommandRequest): Verdict {
        if (isEmergencyStop(request)) {
            log("emergency stop pressed — no MSDK motor cut exists; pointing at the RC")
            announce(EMERGENCY_STOP_TEXT, Severity.CRITICAL)
        } else if (isArm(request)) {
            log("arm requested (p1=1) — MSDK has no arm; DJI arms as part of takeoff")
            announce(StatusTexts.NO_SEPARATE_ARM, Severity.ERROR)
        } else {
            log("ARM_DISARM p1=${request.param1} p2=${request.param2} — not implemented")
        }
        return Verdict.UNSUPPORTED
    }

    private fun isEmergencyStop(request: HandshakeResponder.CommandRequest): Boolean =
        request.param1 == 0f && request.param2 == FORCE_DISARM_MAGIC

    /**
     * `param1 = 1` is arm, in both QGC's manual form and the one
     * `PX4FirmwarePlugin::_mavCommandResult` sends after an accepted takeoff. The two are
     * byte-identical and deliberately not distinguished — see this handler's KDoc.
     */
    private fun isArm(request: HandshakeResponder.CommandRequest): Boolean =
        request.param1 == 1f

    // ---------------------------------------------------------------- the middle

    /**
     * The single place an action is attempted. The caller has already checked the interlock.
     *
     * **The returned [Verdict] is meaningful only to [onTakeoff]** — Return and Land arrive as
     * unacknowledged `SET_MODE` messages, so [onModeRequest] discards it and the only outward
     * channel for those two is the `STATUSTEXT` this function sends. That asymmetry is what
     * "unacknowledged" means, and it is still the reason a lie in [FlightActions] cannot become a
     * `MAV_RESULT_ACCEPTED` on the Return or Land path: there is no ack on that path at all.
     *
     * For takeoff the mapping is the safety property, and it is one line:
     *
     * | outcome | result | why |
     * |---|---|---|
     * | duplicate inside [ACTION_REPEAT_MS] | `ACCEPTED` | the first attempt was accepted and *is* the answer; DJI is not asked twice |
     * | no [actions] attached | `DENIED` | nothing was asked of anything |
     * | [FlightActions] threw | `FAILED` | matches `HandshakeResponder.guarded`'s own verdict for a throwing handler |
     * | [ActionOutcome.Requested] | `ACCEPTED` | and only here — this is the sole path on which QGC may arm |
     * | [ActionOutcome.Refused] / [ActionOutcome.Unavailable] | `DENIED` | a refusal is not an acceptance, whoever made it |
     *
     * The duplicate row is the only one that needs an argument. It returns `ACCEPTED` **without
     * calling DJI**, which is exactly right for the case it exists to serve — a retry after our
     * ack was lost on UDP — and is not a claim beyond what the first attempt already made. It
     * cannot be reached unless a previous attempt returned [ActionOutcome.Requested], because
     * that is the only branch that stamps [lastRequestedAtMs].
     */
    private fun perform(action: FlightAction): Verdict {
        val now = nowMs()
        if (action == lastRequestedAction && now - lastRequestedAtMs < ACTION_REPEAT_MS) {
            // QGC's retry burst, or a double click. The action is already under way; asking DJI
            // again would be a second command for one intent.
            log("${action.label} repeated within ${ACTION_REPEAT_MS}ms — already requested")
            return Verdict.ACCEPTED
        }

        val sink = actions
        if (sink == null) {
            // Interlock on but nothing behind it. Says so rather than pretending: an operator
            // whose Return does nothing is owed the reason, and "no aircraft" is the reason.
            announce(StatusTexts.unavailable(action, "NO_AIRCRAFT_LINK"))
            log("${action.label} requested with no FlightActions attached")
            return Verdict.DENIED
        }

        val outcome = try {
            invoke(sink, action)
        } catch (e: Throwable) {
            // An unwritten or half-written DJI layer throws. That is a failure, never a success,
            // and it must not kill the receive thread.
            announce(StatusTexts.threw(action, e), Severity.ERROR)
            log("${action.label} threw: $e")
            return Verdict.FAILED
        }

        return when (outcome) {
            is ActionOutcome.Requested -> {
                // Stamped only here. A refusal or a failure leaves the stamp alone, so the next
                // press is a fresh attempt rather than a suppressed duplicate.
                lastRequestedAction = action
                lastRequestedAtMs = now
                // The dispatch is announced — "Return sent to DJI" — and this reverses the
                // original design, by measurement
                // (`docs/measurements/2026-07-26-m2-first-command.md`). Silence here was
                // deliberate: the flight mode changing in a later heartbeat is the only honest
                // confirmation, and a cheerful message would be our own uncorroborated claim.
                // What that reasoning assumed is that DJI eventually answers. On 2026-07-26 it
                // did not — `performAction(KeyStartGoHome)` was called for a real operator press
                // and DJI invoked neither `onSuccess` nor `onFailure`, ever. The operator saw
                // absolutely nothing and could not tell a sent command from a broken bridge.
                //
                // The sentence still claims nothing about the aircraft (see StatusTexts
                // .dispatched): it reports only that the request left here. Severity is ERROR
                // for the measured reason the mode refusal and the landing announcement are —
                // QGC 5.0.8 surfaces only EMERGENCY/ALERT/CRITICAL/ERROR to an operator
                // (`StatusTextHandler.cc:18-24`), so anything below it is an announcement that
                // never happened. This does abuse the severity taxonomy: a successful dispatch
                // is not an error, and it will render red beside genuine failures. The trade is
                // taken knowingly and for the same reason as the other two — a momentarily
                // puzzling colour beats an operator who never learns what the bridge did.
                //
                // Routed through `announce`, so it inherits the 5 s identical-text window like
                // every other sentence here and QGC's 3× retry burst reads as one line.
                //
                // Be honest about what that window is doing on this path: **nothing, today.** The
                // action de-dup above has already dropped the second and third of a burst before
                // this branch is reached, and both windows are the same [ACTION_REPEAT_MS]
                // constant, so they expire together and a duplicate dispatch announcement is
                // unreachable by construction. Replacing this call with a raw `send` is therefore
                // an *equivalent mutant* — it was tried, and no test failed, because no behaviour
                // changes (recorded in `CommandDispatcherTest`'s header). It stays `announce`
                // anyway: the two windows are one number today and the guarantee should not
                // depend on nobody ever splitting them.
                //
                // Note the redundancy is only for *this* text. The Unavailable and Refused
                // branches are not stamped, so identical failures do collapse on the window and
                // it is load-bearing there.
                announce(StatusTexts.dispatched(action), Severity.ERROR)
                log("${action.label} handed to DJI")
                // The one branch that returns ACCEPTED, and therefore the one branch on which
                // QGC may arm. Keeping that fact to a single `return` in a single `when` is the
                // point of routing takeoff through here at all.
                Verdict.ACCEPTED
            }

            is ActionOutcome.Refused -> {
                // DJI's own word, verbatim. Not paraphrased, not prettified: the operator can
                // search for FC_AUTH_STATE and cannot search for our opinion of it.
                announce(
                    StatusTexts.refusal(action, outcome.djiError),
                    Severity.ERROR,
                )
                log("${action.label} refused by DJI: ${outcome.djiError}")
                Verdict.DENIED
            }

            is ActionOutcome.Unavailable -> {
                announce(
                    StatusTexts.unavailable(action, outcome.reason),
                    Severity.ERROR,
                )
                log("${action.label} unavailable: ${outcome.reason}")
                Verdict.DENIED
            }
        }
    }

    /**
     * The routing table from [FlightAction] to [FlightActions]. Named methods rather than one
     * `perform(action)` so an implementer reads a specification per action, and so adding a
     * fourth action later breaks every implementation loudly instead of falling into an `else`.
     */
    private fun invoke(sink: FlightActions, action: FlightAction): ActionOutcome = when (action) {
        FlightAction.TAKEOFF -> sink.takeoff()
        FlightAction.RETURN_TO_HOME -> sink.returnToHome()
        FlightAction.LAND -> sink.land()
    }

    // ------------------------------------------------------------------- output

    /**
     * An asynchronous DJI error, arriving on DJI's own schedule rather than in reply to
     * anything — `motorStartFailureError` going from `FC_AUTH_STATE` to `GPS_DISCONNECT` to
     * `NAV_SYS_EXCEPTION` during the 19:16 session on 2026-07-25, which the flight recorder
     * caught and QGC never heard about.
     *
     * Public because the DJI layer subscribes to those keys and this is the only channel that
     * reaches the operator. Blank strings are dropped: an error with no name is not information,
     * and putting "DJI: " on the wire would only teach the operator to ignore the prefix.
     */
    fun reportAsyncDjiError(djiError: String) {
        if (djiError.isBlank()) return
        // A DJI error arriving after a takeoff we started is evidence that the takeoff is in
        // doubt, and an armed climb must not be waiting behind a takeoff that is in doubt —
        // `SYSTEM_ERROR` shortly after a landing is a *measured* refusal on this airframe. This
        // does not attempt to work out whether the error is about the takeoff: it cannot, and
        // erring toward cancelling costs an operator a second press while erring the other way
        // leaves an intention armed behind a failure. The cancel is a no-op whenever nothing is
        // armed, which is almost always.
        climb()?.cancelTakeoffClimb(djiError)
        announce(StatusTexts.djiError(djiError), Severity.ERROR)
    }

    /**
     * The DJI layer auto-confirmed a landing this bridge started, past DJI's ~0.5 m stall —
     * `docs/decisions/2026-07-26-landing-confirmation.md`. The one automatic command this
     * project sends, so it is never sent silently: the operator watching QGC must be able to
     * read that the last half metre was released by the bridge, not by a hand on the RC.
     *
     * Severity is ERROR for the same measured reason the pre-M2 mode refusal was raised to it:
     * QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR to an operator
     * (`StatusTextHandler.cc:18-24`), and an announcement below that threshold is an
     * announcement that never happened. The taxonomy loses to the visibility — an operator
     * momentarily puzzled by the colour is better off than one who never learns an automatic
     * command was sent.
     */
    fun reportLandingConfirmed() {
        log("landing confirmed automatically at DJI's stall height")
        announce(StatusTexts.LANDING_CONFIRMED, Severity.ERROR)
    }

    /**
     * Says one sentence on every attached interface, at most once per [ACTION_REPEAT_MS] for
     * identical text.
     *
     * Same reasoning as `HandshakeResponder.announceModeRefusal`: QGC repeats a guided request
     * three times for one press, and a DJI key that is stuck in an error state repeats forever.
     * Three identical red lines for one click trains an operator to stop reading them, which
     * costs more than the message is worth. Different text is always a different thing to say
     * and always goes out.
     *
     * The 50-byte clamp is applied by the MAVLink sink, which is where the 50 bytes are; this
     * class composes sentences and nothing else.
     */
    private fun announce(text: String, severity: Severity = Severity.ERROR) {
        val now = nowMs()
        if (text == lastAnnouncement && now - lastAnnouncedAtMs < ACTION_REPEAT_MS) return
        lastAnnouncement = text
        lastAnnouncedAtMs = now
        announcer.say(severity, text)
    }
}
